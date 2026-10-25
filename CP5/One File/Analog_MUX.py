"""
Analog mux: PPG + thermistor on ONE ADC (GP26), selected by GP17.

    GP17 = 1 (HIGH)  -> mux passes the PPG        (Q3 path)
    GP17 = 0 (LOW)   -> mux passes the thermistor (Q4 path)

This replaces PulseWave.py's free-running 1 kHz PWM with software-timed
switching, because the ADC has to be read DURING the correct half of the
wave and MicroPython cannot sync to PWM edges (see notes at the bottom).

    - PPG is sampled continuously at 250 Hz while GP17 is HIGH.
    - Once per second GP17 drops LOW for ~2 ms to read the thermistor,
      then goes back HIGH.
    - BMP280 (I2C) is the independent temperature reference.
    - One combined line prints every 2 seconds.

Wiring (unchanged from your three scripts):
    GP26 (ADC0, pin 31) <- mux output V_N (through a buffer)
    GP17 (pin 22)       -> R1 -> Q1 base (mux select / CON chain)
    GP0  = SDA, GP1 = SCL -> BMP280
"""

from machine import ADC, Pin, I2C
import bme280
import time

# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------
adc = ADC(Pin(26))                 # GP26 = ADC0 = mux output V_N
sel = Pin(17, Pin.OUT)             # GP17 -> Q1 base resistor -> CON

PPG, THERM = 1, 0                  # HIGH = PPG, LOW = thermistor
sel.value(PPG)

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
bmp = bme280.BME280(i2c=i2c)

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
SAMPLE_MS = 4          # PPG sample period -> 250 Hz (from PPG.py)
TEMP_MS   = 1000       # read thermistor + BMP280 once per second
REPORT_MS = 2000       # print one combined line every 2 s (from PPG.py)
WINDOW_MS = 6000       # BPM uses the last 6 s of beats (from PPG.py)

SETTLE_MS = 1          # settle time after flipping the mux
N_AVG     = 8          # thermistor readings averaged per update
BLANK_MS  = 200        # ignore PPG for beat detection after a mux switch
SMOOTH_N  = 4          # PPG moving average (4 x 4 ms = 16 ms, ~60 Hz null)

# ---- debug switches ----
TEMP_ENABLED = True    # False = never switch the mux (pure PPG test)
SHOW_WAVE    = False   # True = print PPG samples for the Plotter, no text
PLOT_EVERY   = 10      # plot 1 of every 10 samples (25 points/s)

# ---------------------------------------------------------------------------
# Thermistor calibration (from KalmanFilter_Thermistor_BMP.py)
# ---------------------------------------------------------------------------
ADC_VREF = 3.3
V_LOW,  T_AT_V_LOW  = 0.00, 30.0   # volts, deg C
V_HIGH, T_AT_V_HIGH = 3.193, 10.0   # volts, deg C
CAL_OFFSET_C = 1.8                 # offset vs BMP280 -- RE-CHECK through the mux
ADC_OFFSET_V = 0.000

SLOPE = (T_AT_V_HIGH - T_AT_V_LOW) / (V_HIGH - V_LOW)
INTERCEPT = T_AT_V_LOW + CAL_OFFSET_C - SLOPE * V_LOW


def c_to_f(c):
    return c * 9 / 5 + 32


def counts_to_temp(raw_12bit):
    voltage = raw_12bit * ADC_VREF / 4095 - ADC_OFFSET_V
    v = max(0.0, min(ADC_VREF, voltage))
    return SLOPE * v + INTERCEPT, voltage


# ---------------------------------------------------------------------------
# Kalman filter -- same three formulas, plus process noise Q
#   (0) E_EST += Q                        predict
#   (1) KG     = E_EST / (E_EST + E_MEA)  gain
#   (2) EST    = EST + KG * (MEA - EST)   estimate
#   (3) E_EST  = (1 - KG) * E_EST         error
# ---------------------------------------------------------------------------
class KalmanFilter:
    def __init__(self, initial_estimate, initial_error_est, error_mea, process_noise):
        self.EST = initial_estimate       # None = start on the first reading
        self.E_EST = initial_error_est
        self.E_MEA = error_mea
        self.Q = process_noise

    def update(self, MEA):
        if self.EST is None:              # first reading starts the filter
            self.EST = MEA
            return self.EST
        self.E_EST = self.E_EST + self.Q                    # (0)
        KG = self.E_EST / (self.E_EST + self.E_MEA)         # (1)
        self.EST = self.EST + KG * (MEA - self.EST)         # (2)
        self.E_EST = (1 - KG) * self.E_EST                  # (3)
        return self.EST


# temperature filter tuning from KalmanFilter_Thermistor_BMP.py
kf_temp = KalmanFilter(initial_estimate=25.0, initial_error_est=1.0,
                       error_mea=0.25, process_noise=0.01)
# BPM filter tuning from PPG.py (E_EST=10, E_MEA=5, Q=1)
kf_bpm = KalmanFilter(initial_estimate=None, initial_error_est=10.0,
                      error_mea=5.0, process_noise=1.0)


# ---------------------------------------------------------------------------
# Mux reads
# ---------------------------------------------------------------------------
def read_ppg():
    return adc.read_u16() >> 4             # mux is already on the PPG channel


def read_thermistor_via_mux():
    """Flip the mux LOW, let it settle, average N readings, flip back HIGH."""
    sel.value(THERM)
    time.sleep_ms(SETTLE_MS)
    adc.read_u16()                         # throw away the first reading
    total = 0
    for _ in range(N_AVG):
        total += adc.read_u16() >> 4
    sel.value(PPG)
    time.sleep_ms(SETTLE_MS)                # let the PPG path settle again
    adc.read_u16()
    return total // N_AVG


def read_bmp280_c():
    t, p, h = bmp.read_compensated_data()
    return t / 100


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def fmt(c):
    return "{:6.2f}C/{:6.2f}F".format(c, c_to_f(c))


def main():
    print("T(C) = {:.4f} * V + {:.4f}".format(SLOPE, INTERCEPT))
    print("GP17 HIGH = PPG, LOW = thermistor ({} ms low once per {} ms)".format(
        2 * SETTLE_MS, TEMP_MS))
    print("HR raw/KF | Therm raw | Therm KF | BMP280 | Diff (KF - BMP)")

    high = low = read_ppg()
    above = False
    beats = []

    therm_raw = therm_kf = bmp_c = None

    now = time.ticks_ms()
    next_sample = now
    last_temp = time.ticks_add(now, -TEMP_MS)   # read temps right away
    last_report = now
    blank_until = now
    total = count = 0
    hist = []

    while True:
        # ---- PPG sample every 4 ms, smoothed over 4 samples ----
        hist.append(read_ppg())
        if len(hist) > SMOOTH_N:
            hist.pop(0)
        sample = sum(hist) // len(hist)
        now = time.ticks_ms()

        if SHOW_WAVE:
            total += sample
            count += 1
            if count == PLOT_EVERY:
                print(total // count)
                total = count = 0

        # ---- beat detection (skipped right after a mux switch) ----
        if time.ticks_diff(now, blank_until) >= 0:
            high = max(sample, high - 1)
            low = min(sample, low + 1)
            threshold = low + 0.6 * (high - low)
            if sample > threshold and not above and (high - low) > 100:
                above = True
                if not beats or time.ticks_diff(now, beats[-1]) > 300:
                    beats.append(now)
            elif sample < threshold:
                above = False

        # ---- thermistor + BMP280 once per second ----
        if TEMP_ENABLED and time.ticks_diff(now, last_temp) >= TEMP_MS:
            counts = read_thermistor_via_mux()
            therm_raw, _ = counts_to_temp(counts)
            therm_kf = kf_temp.update(therm_raw)
            bmp_c = read_bmp280_c()
            last_temp = now
            blank_until = time.ticks_add(time.ticks_ms(), BLANK_MS)

        # ---- BPM + combined print every 2 s ----
        if not SHOW_WAVE and time.ticks_diff(now, last_report) >= REPORT_MS:
            beats = [t for t in beats if time.ticks_diff(now, t) <= WINDOW_MS]
            intervals = [time.ticks_diff(b, a) for a, b in zip(beats, beats[1:])]
            intervals = [i for i in intervals if 300 < i < 2000]
            if len(intervals) >= 2:
                intervals.sort()
                bpm_raw = 60000 / intervals[len(intervals) // 2]
                bpm_kf = kf_bpm.update(bpm_raw)
                hr = "HR {:3d}/{:3d} BPM".format(round(bpm_raw), round(bpm_kf))
            else:
                hr = "HR  --/ -- BPM"

            if therm_raw is not None:
                diff_c = therm_kf - bmp_c
                print("{} | Therm {} | KF {} | BMP {} | Diff {:+5.2f}C/{:+5.2f}F".format(
                    hr, fmt(therm_raw), fmt(therm_kf), fmt(bmp_c),
                    diff_c, diff_c * 9 / 5))
            else:
                print(hr)
            last_report = now

        # ---- hold the 250 Hz schedule ----
        next_sample = time.ticks_add(next_sample, SAMPLE_MS)
        wait = time.ticks_diff(next_sample, time.ticks_ms())
        if wait > 0:
            time.sleep_ms(wait)
        else:
            next_sample = time.ticks_ms()   # fell behind (temp read); resync


try:
    main()
finally:
    sel.value(PPG)     # leave GP17 HIGH, same as PulseWave.py's IDLE_LEVEL = 1
    print("Stopped. GP17 = 1 (PPG channel)")


# ---------------------------------------------------------------------------
# WHY THE 1 kHz PWM WAS REPLACED
#
# PulseWave.py ran GP17 at 1 kHz, 90% duty. That gives a 100 us LOW window
# for the thermistor. The problem is not the mux -- your BJTs switch in
# well under a microsecond -- it is the Pico side:
#
#   * One adc.read_u16() in MicroPython takes roughly 20-40 us including
#     interpreter overhead, so only 2-4 readings fit in the 100 us window
#     even if the timing were perfect.
#   * There is no way in MicroPython to know WHERE in the PWM period you
#     are. A read can land anywhere -- including on an edge -- so some
#     samples would be PPG, some thermistor, and some mid-transition.
#
# Software switching fixes both: the ADC is read only when the code itself
# knows which channel is selected. The effective duty cycle is still about
# the same as before (~1000 ms high, ~2 ms low = 99.8% high), it is just
# a much slower switching rate (1 Hz instead of 1 kHz). That is fine --
# skin temperature changes far slower than once per second, and the PPG
# loses only 2 ms out of every 1000 ms.
# ---------------------------------------------------------------------------