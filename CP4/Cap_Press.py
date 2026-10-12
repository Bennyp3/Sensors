"""
Capacitive press detector (CP4).
  GP20 -> R2 (1.421 MOhm) -> VC -> sensor C (~112 pF) to GND
                                           (Pico SOURCES the square wave)
  VC -> LM339 (+) pin 5, Vref 1.65 V -> LM339 (-) pin 4
  LM339 out pin 2 (10k pull-up to 3.3 V) -> GP21   (Pico READS the output)
  Expected td = 0.693*R*C: ~110 us untouched, ~133 us light corner,
  ~220 us hard center.

A PIO state machine makes each rising edge and counts, in hardware, how long
the comparator takes to go high (td). Pressing raises C, so td gets longer.
At startup the untouched td is measured (don't touch the pad), and a press is
any td longer than baseline + THRESH_OFFSET_US for a few cycles in a row.
"""

import rp2
from machine import Pin
import time

DRIVE_PIN = 20          # square wave out to R
SENSE_PIN = 21          # comparator output in
F_SM = 50_000_000       # PIO clock: one count loop = 2 cycles = 0.04 us
HALF_US = 1000          # half-period -> ~500 Hz (a hard press needs 3RC ~ 950 us)
TIMEOUT_US = 800        # give up if the comparator never switches
CAL_CYCLES = 200        # untouched cycles averaged for the baseline
THRESH_OFFSET_US = 11.5 # threshold = baseline + half the smallest-press shift (~23 us)
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

    print("Calibrating -- don't touch the pad...")
    vals = [t for t in (measure_td_us() for _ in range(CAL_CYCLES)) if t is not None]
    if not vals:
        print("No comparator edges seen. Check wiring, pull-up and power.")
        return
    baseline = sum(vals) / len(vals)
    thresh = baseline + THRESH_OFFSET_US
    print("Baseline td = {:.2f} us, threshold = {:.2f} us".format(baseline, thresh))

    pressed = False
    streak = 0
    total = n = 0
    last = time.ticks_ms()

    while True:
        td = measure_td_us()
        if td is None:
            continue
        total += td
        n += 1

        is_long = td > thresh
        if is_long != pressed:
            streak += 1
            if streak >= DEBOUNCE:
                pressed = is_long
                streak = 0
                print(">>> PRESS" if pressed else ">>> RELEASE")
        else:
            streak = 0

        if time.ticks_diff(time.ticks_ms(), last) >= REPORT_MS:
            print("td = {:6.2f} us  ({})".format(total / n, "PRESSED" if pressed else "untouched"))
            total = n = 0
            last = time.ticks_ms()


try:
    main()
finally:
    sm.active(0)
    Pin(DRIVE_PIN, Pin.OUT, value=0)   # stop the square wave, leave pin low
    print("Stopped. GP{} = 0 V".format(DRIVE_PIN))