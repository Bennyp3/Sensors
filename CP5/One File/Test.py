"""
Wearable_Main.py -- merged wearable tracker, built up one sensor at a time.

Every sensor is its own subfunction that returns that sensor's value:

    read_ppg()         -> one PPG sample, 12-bit counts (0-4095)
    read_thermistor()  -> thermistor temperature, deg C (raw, before Kalman)
    read_bmp280()      -> BMP280 temperature, deg C
    read_ecg()         -> one ECG sample, 12-bit counts, or None if a lead is off
    ecg_leads_off()    -> which ECG electrode is off, or None if both are on
    read_accel()       -> LSM6DSO acceleration (x, y, z) in g
    read_press_td()    -> press sensor charge time td in us, or None on timeout

Step 1: Analog_MUX.py (PPG + thermistor + BMP280) split into subfunctions.
Step 2: AD8232 ECG added (from AD8232.py). ECG BPM is raw for now -- no
        Kalman filter yet. R peaks are found by slope.
Step 3: LSM6DSO accelerometer added (from Accel_BMP280.py), read at
        25 Hz on the same I2C bus as the BMP280.
Step 4: Capacitive press sensor added (from Cap_Press.py), read 10 times a
        second so its ~1 ms measurement does not stall the 4 ms loop.
Step 5: Bluetooth (BLE) streaming to the computer (needs ble_uart.py on the
        Pico, and Wearable_Receiver.py on the computer). USB printing still
        works as before, so Thonny can be used for debugging.

Bluetooth messages (one line each, comma separated):
    W,<ppg>,<ecg>       wave point, 25 per second (12-bit counts)
    S,<15 fields>       status every 2 s, fields in STATUS_FIELDS order
    E,TOUCHED / E,RELEASED   press sensor event, sent the moment it happens
An empty field means "no reading yet".

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
    GP0  = SDA, GP1 = SCL -> BMP280 (0x76) + LSM6DSO (0x6A), one shared bus
    GP27 (ADC1, pin 32) <- AD8232 OUTPUT (ECG)
    GP14 (pin 19)       <- AD8232 LO+  (high = electrode off)
    GP15 (pin 20)       <- AD8232 LO-  (high = electrode off)
    GP20 (pin 26)       -> R2 -> press sensor C (square wave out, PIO)
    GP21 (pin 27)       <- LM339 comparator output (press sensor)
"""

from machine import ADC, Pin, I2C
import ble_uart
import bme280
import lsm6dsox
import math
import rp2
import time

# ---------------------------------------------------------------------------
# Hardware
# ---------------------------------------------------------------------------
adc = ADC(Pin(26))                 # GP26 = ADC0 = mux output V_N
sel = Pin(17, Pin.OUT)             # GP17 -> Q1 base resistor -> CON

PPG, THERM = 1, 0                  # HIGH = PPG, LOW = thermistor
sel.value(PPG)

# ONE I2C object shared by both I2C sensors (different addresses).
# 100 kHz is the speed Accel_BMP280.py ran both sensors at together.
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
bmp = bme280.BME280(i2c=i2c)
imu = lsm6dsox.LSM6DSOX(i2c, gyro_odr=0)   # accel 104 Hz, +/-4 g; gyro off

ecg_adc  = ADC(Pin(27))            # GP27 = ADC1 = AD8232 OUTPUT
lo_plus  = Pin(14, Pin.IN)         # GP14 = AD8232 LO+
lo_minus = Pin(15, Pin.IN)         # GP15 = AD8232 LO-

PRESS_DRIVE_PIN = 20               # GP20 = square wave out to R2
PRESS_SENSE_PIN = 21               # GP21 = comparator output in

# ---------------------------------------------------------------------------
# Timing
# ---------------------------------------------------------------------------
SAMPLE_MS = 4          # PPG sample period -> 250 Hz (from PPG.py)
TEMP_MS   = 1000       # read thermistor + BMP280 once per second
IMU_MS    = 40         # read LSM6DSO accel every 40 ms (25 Hz)
REPORT_MS = 2000       # print one combined line every 2 s (from PPG.py)
WINDOW_MS = 6000       # BPM uses the last 6 s of beats (from PPG.py)

SETTLE_MS = 1          # settle time after flipping the mux
N_AVG     = 8          # thermistor readings averaged per update
BLANK_MS  = 200        # ignore PPG for beat detection after a mux switch
SMOOTH_N  = 4          # PPG moving average (4 x 4 ms = 16 ms, ~60 Hz null)

# ---- ECG R-peak detection (slope based, simplified Pan-Tompkins idea) ----
# The R peak rises much faster than the P wave, T wave or baseline drift, so
# beats are found on the signal's rising SLOPE, not its height.
ECG_SLOPE_LAG     = 4     # slope = sample now - sample 4 ago (16 ms)
ECG_MIN_SLOPE     = 40    # slope swing needed to count as a beat (counts)
ECG_DECAY         = 0.25  # envelope relaxes 0.25 count/sample (~60 counts/s)
ECG_REFRACTORY_MS = 350   # ignore beats closer than 350 ms (>171 BPM)

# ---- capacitive press sensor (from Cap_Press.py) ----
PRESS_MS         = 100        # measure td every 100 ms (10 Hz)
F_SM             = 50_000_000 # PIO clock: one count loop = 2 cycles = 0.04 us
HALF_US          = 1000       # high half of the square wave
TIMEOUT_US       = 800        # give up if the comparator never switches
TOUCH_ABOVE_US   = 300.0      # td above this = touched (measured: ~195 us
                              # untouched, ~245-290 us touched)
RELEASE_BELOW_US = 300.0      # td below this = released. The gap between the
                              # two thresholds (hysteresis) stops flicker while
                              # a finger lifts off. Set to 220.0 = old behaviour
DEBOUNCE         = 5          # readings in a row to change state (5 x 100 ms)

US_PER_COUNT = 2 / F_SM * 1e6
TIMEOUT = int(TIMEOUT_US / US_PER_COUNT)

# ---- debug switches ----
TEMP_ENABLED = True    # False = never switch the mux (pure PPG test)
SHOW_WAVE    = False   # True = print wave samples for the Plotter, no text
WAVE_SOURCE  = "ppg"   # which wave SHOW_WAVE plots: "ppg", "ecg" or
                       # "ecg_slope" (what the ECG beat detector sees)
PLOT_EVERY   = 10      # plot 1 of every 10 samples (25 points/s)
BLE_ENABLED  = True    # False = USB only, no Bluetooth
BLE_NAME     = "Wearable"   # name the computer looks for

# order of the fields in each Bluetooth status ("S") line
STATUS_FIELDS = ("ecg_bpm", "ecg_off", "ppg_bpm_raw", "ppg_bpm_kf",
                 "therm_raw_c", "therm_kf_c", "bmp_c", "diff_c",
                 "touched", "td_avg_us", "accel_x_g", "accel_y_g",
                 "accel_z_g", "loop_max_ms", "enmo_mg")

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
# Sensor subfunctions -- call one and it returns that sensor's value
# ---------------------------------------------------------------------------
def read_ppg():
    """PPG: return one 12-bit sample (0-4095) from the mux PPG channel."""
    # GP17 is normally already HIGH (read_thermistor always puts it back),
    # so this only re-asserts it; no settle delay is needed here
    sel.value(PPG)
    return adc.read_u16() >> 4


def read_thermistor():
    """Thermistor: return the raw (unfiltered) temperature in deg C.

    Flips the mux LOW, lets it settle, averages N readings, then flips back
    HIGH so the PPG channel is selected again when this returns."""
    sel.value(THERM)
    time.sleep_ms(SETTLE_MS)
    adc.read_u16()                         # throw away the first reading
    total = 0
    for _ in range(N_AVG):
        total += adc.read_u16() >> 4
    sel.value(PPG)
    time.sleep_ms(SETTLE_MS)                # let the PPG path settle again
    adc.read_u16()
    temp_c, _ = counts_to_temp(total // N_AVG)
    return temp_c


def read_bmp280():
    """BMP280: return the temperature in deg C (reference for the thermistor)."""
    t, p, h = bmp.read_compensated_data()
    return t / 100


def ecg_leads_off():
    """AD8232: return which electrode is off, or None if both are connected."""
    if lo_plus.value() and lo_minus.value():
        return "both"
    if lo_plus.value():
        return "LO+ electrode"
    if lo_minus.value():
        return "LO- electrode"
    return None


def read_accel():
    """LSM6DSO: return acceleration (x, y, z) in g (1 g = 9.81 m/s^2)."""
    return imu.accel()


# ---------------------------------------------------------------------------
# Capacitive press sensor -- PIO program unchanged from Cap_Press.py.
# The PIO makes the rising edge and counts, in hardware, how long the
# comparator takes to go high (td). A press raises C, so td goes UP.
# ---------------------------------------------------------------------------
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


press_sense = Pin(PRESS_SENSE_PIN, Pin.IN)
sm = rp2.StateMachine(0, cap_delay, freq=F_SM,
                      set_base=Pin(PRESS_DRIVE_PIN), jmp_pin=press_sense)


def read_press_td():
    """Press sensor: return td in us, or None if the comparator timed out.

    Same as measure_td_us() in Cap_Press.py, except it does not wait out the
    1 ms low half: it is only called every PRESS_MS (100 ms), so the
    capacitor has plenty of time to discharge before the next call. That
    keeps this call to about 1 ms inside the 4 ms loop."""
    sm.put(TIMEOUT)
    x = sm.get()
    time.sleep_us(HALF_US)    # rest of the high half
    sm.put(0)                 # drive low: capacitor discharges until next call
    if x == 0xFFFFFFFF:       # counter ran out: comparator never switched
        return None
    return (TIMEOUT - x) * US_PER_COUNT


class TouchDetector:
    """Debounced touched/untouched state from td readings, with hysteresis."""
    def __init__(self):
        self.touched = False
        self.streak = 0

    def update(self, td):
        """Feed one td reading. Returns True when the state just changed."""
        if self.touched:
            wants_change = td < RELEASE_BELOW_US
        else:
            wants_change = td > TOUCH_ABOVE_US
        if not wants_change:
            self.streak = 0
            return False
        self.streak += 1
        if self.streak < DEBOUNCE:
            return False
        self.touched = not self.touched
        self.streak = 0
        return True



def read_ecg():
    """AD8232: return one 12-bit ECG sample (0-4095), or None if a lead is off."""
    if ecg_leads_off():
        return None
    return ecg_adc.read_u16() >> 4


# ---------------------------------------------------------------------------
# Beat detection -- one copy of the algorithm, used by both PPG and ECG
#   track the signal's top (high) and bottom (low); both relax by `decay`
#   counts per sample
#   threshold = 60% of the way from low to high
#   a rising crossing of the threshold (with enough swing) = one beat
# PPG feeds in its smoothed samples with the original settings.
# ECG feeds in its slope (ecg_slope in main) with slower decay and a 350 ms
# limit.
# ---------------------------------------------------------------------------
class BeatDetector:
    def __init__(self, start, decay=1, min_swing=100, refractory_ms=300):
        self.high = self.low = start      # running envelope of the signal
        self.above = False
        self.beats = []                   # tick (ms) of each detected beat
        self.decay = decay
        self.min_swing = min_swing
        self.refractory_ms = refractory_ms

    def update(self, sample, now):
        self.high = max(sample, self.high - self.decay)
        self.low = min(sample, self.low + self.decay)
        threshold = self.low + 0.6 * (self.high - self.low)
        if sample > threshold and not self.above and (self.high - self.low) > self.min_swing:
            self.above = True
            # ignore double counts closer than the refractory time
            if not self.beats or time.ticks_diff(now, self.beats[-1]) > self.refractory_ms:
                self.beats.append(now)
        elif sample < threshold:
            self.above = False

    def clear(self):
        """Forget past beats (e.g. after an ECG electrode comes off)."""
        self.beats = []
        self.above = False

    def bpm(self, now):
        """Median BPM over the last WINDOW_MS, or None if too few beats."""
        self.beats = [t for t in self.beats if time.ticks_diff(now, t) <= WINDOW_MS]
        intervals = [time.ticks_diff(b, a) for a, b in zip(self.beats, self.beats[1:])]
        intervals = [i for i in intervals if 300 < i < 2000]
        if len(intervals) < 2:
            return None
        intervals.sort()
        return 60000 / intervals[len(intervals) // 2]


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------
def fmt(c):
    return "{:6.2f}C/{:6.2f}F".format(c, c_to_f(c))


def field(value, spec="{:.2f}"):
    """One status field for Bluetooth: formatted number, or '' if None."""
    return "" if value is None else spec.format(value)


def main():
    print("T(C) = {:.4f} * V + {:.4f}".format(SLOPE, INTERCEPT))
    print("GP17 HIGH = PPG, LOW = thermistor ({} ms low once per {} ms)".format(
        2 * SETTLE_MS, TEMP_MS))
    print("ECG raw | PPG raw/KF | Therm raw | Therm KF | BMP280 | Diff (KF - BMP)")

    ble = None
    if BLE_ENABLED:
        ble = ble_uart.BLEUART(BLE_NAME)
        print("Bluetooth: advertising as '{}'".format(BLE_NAME))

    ppg_beats = BeatDetector(read_ppg())
    ecg_beats = BeatDetector(0, decay=ECG_DECAY, min_swing=ECG_MIN_SLOPE,
                             refractory_ms=ECG_REFRACTORY_MS)
    ecg_hist = []                       # last few ECG samples, for the slope
    ecg_off = None                      # which electrode is off, if any

    therm_raw = therm_kf = bmp_c = None
    accel = None                        # latest LSM6DSO reading
    enmo_total = enmo_n = 0             # for the average movement in each report

    sm.active(1)                        # start the press-sensor PIO
    for _ in range(20):                 # a few warm-up cycles (Cap_Press.py)
        read_press_td()
        time.sleep_us(HALF_US)          # full low half during warm-up
    touch = TouchDetector()
    td_total = td_n = 0                 # for the average td in each report

    now = time.ticks_ms()
    next_sample = now
    last_temp = time.ticks_add(now, -TEMP_MS)   # read temps right away
    last_imu = now
    last_press = now
    last_report = now
    blank_until = now
    ppg_total = ecg_total = slope_total = count = 0   # wave point averages
    hist = []
    worst_us = 0                        # longest loop pass since last report

    while True:
        loop_start = time.ticks_us()

        # ---- PPG sample every 4 ms, smoothed over 4 samples ----
        hist.append(read_ppg())
        if len(hist) > SMOOTH_N:
            hist.pop(0)
        sample = sum(hist) // len(hist)

        # ---- ECG sample every 4 ms (no smoothing, as in AD8232.py) ----
        ecg_sample = read_ecg()
        now = time.ticks_ms()

        # ECG slope: how much the signal rose over the last ECG_SLOPE_LAG
        # samples. Falling parts are set to 0 -- only the R upstroke matters.
        ecg_slope = 0
        if ecg_sample is None:
            ecg_hist = []               # electrode off: start the slope fresh
        else:
            ecg_hist.append(ecg_sample)
            if len(ecg_hist) > ECG_SLOPE_LAG + 1:
                ecg_hist.pop(0)
            ecg_slope = max(0, ecg_hist[-1] - ecg_hist[0])

        # ---- wave points: average PLOT_EVERY samples -> 25 points/s ----
        ppg_total += sample
        ecg_total += 0 if ecg_sample is None else ecg_sample
        slope_total += ecg_slope
        count += 1
        if count == PLOT_EVERY:
            ppg_pt = ppg_total // count
            ecg_pt = ecg_total // count
            if SHOW_WAVE:                   # USB, for Thonny's Plotter
                if WAVE_SOURCE == "ecg":
                    print(ecg_pt)
                elif WAVE_SOURCE == "ecg_slope":
                    print(slope_total // count)
                else:
                    print(ppg_pt)
            if ble:                         # Bluetooth: both waves
                ble.send("W,{},{}".format(ppg_pt, ecg_pt))
            ppg_total = ecg_total = slope_total = count = 0

        # ---- PPG beat detection (skipped right after a mux switch) ----
        if time.ticks_diff(now, blank_until) >= 0:
            ppg_beats.update(sample, now)

        # ---- ECG beat detection (skipped while an electrode is off) ----
        if ecg_sample is None:
            ecg_off = ecg_leads_off()
            ecg_beats.clear()           # no BPM from intervals across the gap
        else:
            ecg_off = None
            ecg_beats.update(ecg_slope, now)

        # ---- press sensor at 10 Hz (skipped on the slow temperature pass) ----
        if (time.ticks_diff(now, last_press) >= PRESS_MS
                and time.ticks_diff(now, last_temp) < TEMP_MS):
            td = read_press_td()
            if td is not None:
                td_total += td
                td_n += 1
                if touch.update(td):
                    event = "TOUCHED" if touch.touched else "RELEASED"
                    if not SHOW_WAVE:
                        print(">>> " + event)
                    if ble:
                        ble.send("E," + event)
            last_press = now

        # ---- LSM6DSO accel at 25 Hz ----
        if time.ticks_diff(now, last_imu) >= IMU_MS:
            accel = read_accel()
            # ENMO (Euclidean Norm Minus One): total acceleration minus the
            # 1 g of gravity, in mg. Averaged over each report for the
            # activity level (Hildebrand et al. 2014).
            mag = math.sqrt(accel[0] ** 2 + accel[1] ** 2 + accel[2] ** 2)
            enmo_total += max(0.0, mag - 1.0) * 1000
            enmo_n += 1
            last_imu = now

        # ---- thermistor + BMP280 once per second ----
        if TEMP_ENABLED and time.ticks_diff(now, last_temp) >= TEMP_MS:
            therm_raw = read_thermistor()
            therm_kf = kf_temp.update(therm_raw)
            bmp_c = read_bmp280()
            last_temp = now
            blank_until = time.ticks_add(time.ticks_ms(), BLANK_MS)

        # ---- BPM + combined report every 2 s (USB text + Bluetooth) ----
        if time.ticks_diff(now, last_report) >= REPORT_MS:
            # ECG heart rate (raw -- no Kalman filter yet)
            ecg_bpm = ecg_beats.bpm(now)
            if ecg_off:
                ecg = "ECG off: {}".format(ecg_off)
            elif ecg_bpm is not None:
                ecg = "ECG {:3d} BPM".format(round(ecg_bpm))
            else:
                ecg = "ECG  -- BPM"

            # PPG heart rate (raw / Kalman)
            bpm_raw = ppg_beats.bpm(now)
            bpm_kf = None
            if bpm_raw is not None:
                bpm_kf = kf_bpm.update(bpm_raw)
                ppg = "PPG {:3d}/{:3d} BPM".format(round(bpm_raw), round(bpm_kf))
            else:
                ppg = "PPG  --/ -- BPM"

            hr = ecg + " | " + ppg
            diff_c = None if therm_raw is None else therm_kf - bmp_c
            td_avg = td_total / td_n if td_n else None
            ax, ay, az = accel if accel is not None else (None, None, None)

            if not SHOW_WAVE:                # USB text (Thonny Shell)
                if therm_raw is not None:
                    print("{} | Therm {} | KF {} | BMP {} | Diff {:+5.2f}C/{:+5.2f}F".format(
                        hr, fmt(therm_raw), fmt(therm_kf), fmt(bmp_c),
                        diff_c, diff_c * 9 / 5))
                else:
                    print(hr)

                # press sensor: current state + average td over the last 2 s
                if td_avg is not None:
                    print("    Press {} (td avg {:6.1f} us)".format(
                        "TOUCHED  " if touch.touched else "untouched", td_avg))
                else:
                    print("    Press -- (no reading: comparator timed out)")

                # LSM6DSO: latest reading, on its own line
                if accel is not None:
                    print("    Accel X {:+6.2f} Y {:+6.2f} Z {:+6.2f} g | ENMO {:5.1f} mg".format(
                        ax, ay, az, enmo_total / enmo_n if enmo_n else 0.0))
                # longest pass in the last 2 s (budget is SAMPLE_MS = 4 ms; the
                # once-per-second thermistor + BMP280 pass is expected to be longer)
                print("    Loop max {:5.1f} ms".format(worst_us / 1000))
                if ble:
                    print("    Bluetooth {}".format(
                        "connected" if ble.is_connected() else "waiting for computer"))

            if ble:                          # Bluetooth status line
                ble.send(",".join((
                    "S",
                    field(ecg_bpm, "{:.0f}"), ecg_off or "",
                    field(bpm_raw, "{:.0f}"), field(bpm_kf, "{:.0f}"),
                    field(therm_raw), field(therm_kf), field(bmp_c), field(diff_c),
                    "1" if touch.touched else "0", field(td_avg, "{:.1f}"),
                    field(ax), field(ay), field(az),
                    field(worst_us / 1000, "{:.1f}"),
                    field(enmo_total / enmo_n if enmo_n else None, "{:.1f}"))))

            td_total = td_n = 0
            enmo_total = enmo_n = 0
            worst_us = 0
            last_report = now

        # ---- Bluetooth: send one queued packet per pass ----
        if ble:
            ble.flush()

        # ---- loop timing check ----
        worst_us = max(worst_us, time.ticks_diff(time.ticks_us(), loop_start))

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
    sm.active(0)
    Pin(PRESS_DRIVE_PIN, Pin.OUT, value=0)   # stop the square wave, pin low
    print("GP{} = 0 V (press sensor off)".format(PRESS_DRIVE_PIN))


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