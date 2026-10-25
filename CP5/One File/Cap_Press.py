"""
Capacitive press detector (CP4).
  GP20 -> R2 (1.421 MOhm) -> VC -> sensor C (~112 pF) to GND
                                           (Pico SOURCES the square wave)
  VC -> LM339 (+) pin 5, Vref 1.65 V -> LM339 (-) pin 4
  LM339 out pin 2 (10k pull-up to 3.3 V) -> GP21   (Pico READS the output)
  Expected td = 0.693*R*C: ~110 us untouched, ~133 us light corner,
  ~220 us hard center.

A PIO state machine makes each rising edge and counts, in hardware, how long
the comparator takes to go high (td). With the grounded plate on top, a press
raises C, so td goes UP (untouched ~14.3 us, pressed ~40-67 us). td above
TOUCH_ABOVE_US for a few cycles in a row = touched.
"""

import rp2
from machine import Pin
import time

DRIVE_PIN = 20          # square wave out to R
SENSE_PIN = 21          # comparator output in
F_SM = 50_000_000       # PIO clock: one count loop = 2 cycles = 0.04 us
HALF_US = 1000          # half-period -> ~500 Hz (a hard press needs 3RC ~ 950 us)
TIMEOUT_US = 800        # give up if the comparator never switches
TOUCH_ABOVE_US = 30.0   # td above this = touched (midway: ~14 us untouched, ~45 us pressed)
DEBOUNCE = 5            # cycles in a row needed to change state
REPORT_MS = 500         # print average td this often

US_PER_COUNT = 2 / F_SM * 1e6
TIMEOUT = int(TIMEOUT_US / US_PER_COUNT)


@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW)
def cap_delay():
    pull(block)               # CPU sends timeout count = "start a cycle"
    mov(x, osr)
    set(pins, 1)              # rising edge into the RC
    label("wait")
    jmp(pin, "done")          # comparator output high -> stop counting
    jmp(x_dec, "wait")        # otherwise count down and check again
    label("done")
    mov(isr, x)
    push(block)               # send remaining count back to the CPU
    pull(block)               # CPU says "go low"
    set(pins, 0)              # falling edge: capacitor discharges


sense = Pin(SENSE_PIN, Pin.IN)
sm = rp2.StateMachine(0, cap_delay, freq=F_SM,
                      set_base=Pin(DRIVE_PIN), jmp_pin=sense)


def measure_td_us():
    """One full square-wave cycle. Returns td in us, or None on timeout."""
    sm.put(TIMEOUT)
    x = sm.get()
    time.sleep_us(HALF_US)    # rest of the high half
    sm.put(0)                 # drive low
    time.sleep_us(HALF_US)    # low half: capacitor fully discharges
    if x == 0xFFFFFFFF:       # counter ran out: comparator never switched
        return None
    return (TIMEOUT - x) * US_PER_COUNT


def main():
    sm.active(1)
    for _ in range(20):       # a few warm-up cycles
        measure_td_us()

    print("Touched = td above {:.1f} us".format(TOUCH_ABOVE_US))

    touched = False          # debounced state
    touched_in_window = False
    streak = 0
    total = n = 0
    last = time.ticks_ms()

    while True:
        td = measure_td_us()
        if td is None:
            continue
        total += td
        n += 1

        is_touch = td > TOUCH_ABOVE_US
        if is_touch != touched:
            streak += 1
            if streak >= DEBOUNCE:
                touched = is_touch
                streak = 0
                print(">>> TOUCHED" if touched else ">>> RELEASED")
        else:
            streak = 0
        if touched:
            touched_in_window = True

        if time.ticks_diff(time.ticks_ms(), last) >= REPORT_MS:
            print("td = {:6.2f} us  ({})".format(
                total / n, "TOUCHED" if touched_in_window else "untouched"))
            total = n = 0
            touched_in_window = touched
            last = time.ticks_ms()


try:
    main()
finally:
    sm.active(0)
    Pin(DRIVE_PIN, Pin.OUT, value=0)   # stop the square wave, leave pin low
    print("Stopped. GP{} = 0 V".format(DRIVE_PIN))