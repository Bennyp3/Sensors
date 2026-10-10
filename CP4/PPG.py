from machine import ADC, Pin
import time

ppg = ADC(Pin(26))       # GP26 (Pin 31)

SAMPLE_MS = 4            # 250 Hz sampling
PLOT_EVERY = 15          # ~17 points/s on the plotter
SHOW_WAVE = False        # False = BPM only (easy to read), True = waveform too
REPORT_MS = 2000         # print BPM every 2 seconds
WINDOW_MS = 6000         # use the last 6 s of beats (~7 beats) for each BPM

# Kalman filter on BPM
EST = None               # current estimate (set from first reading)
E_EST = 10.0             # starting error in the estimate (BPM)
E_MEA = 5.0              # measurement error (BPM): how noisy raw readings are
Q = 1.0                  # process noise: how much true HR can drift per update

def read_ppg():
    return ppg.read_u16() >> 4   # 12-bit (0-4095)

high = low = read_ppg()
above = False
beats = []
last_report = time.ticks_ms()
total = count = 0

while True:
    sample = read_ppg()
    now = time.ticks_ms()

    # adaptive threshold beat detection
    high = max(sample, high - 1)
    low = min(sample, low + 1)
    threshold = low + 0.6 * (high - low)
    if sample > threshold and not above and (high - low) > 100:
        above = True
        if not beats or time.ticks_diff(now, beats[-1]) > 300:
            beats.append(now)
    elif sample < threshold:
        above = False

    # waveform output (decimated)
    total += sample
    count += 1
    if count == PLOT_EVERY:
        if SHOW_WAVE:
            print(total // count)
        total = count = 0

    # BPM + Kalman filter every 2 s
    if time.ticks_diff(now, last_report) >= REPORT_MS:
        beats = [t for t in beats if time.ticks_diff(now, t) <= WINDOW_MS]
        intervals = [time.ticks_diff(b, a) for a, b in zip(beats, beats[1:])]
        intervals = [i for i in intervals if 300 < i < 2000]
        if len(intervals) >= 2:
            intervals.sort()
            raw = 60000 / intervals[len(intervals) // 2]
            if EST is None:
                EST = raw                          # first reading starts the filter
            else:
                E_EST += Q                         # predict: allow HR to drift
                KG = E_EST / (E_EST + E_MEA)       # Kalman gain
                EST = EST + KG * (raw - EST)       # update estimate
                E_EST = (1 - KG) * E_EST           # update estimate error
            print("BPM raw:", round(raw), "  KF:", round(EST))
        else:
            print("BPM: --")
        last_report = now

    time.sleep_ms(SAMPLE_MS)