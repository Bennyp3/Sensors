from machine import ADC, Pin
import time

ppg = ADC(Pin(26))       # GP26 (Pin 31)

SAMPLE_MS = 4            # sample at 250 Hz for accurate beat timing
PLOT_EVERY = 5           # plot 1 point per 5 samples -> 5x more time on screen
SHOW_WAVE = True         # True = waveform in Plotter, False = BPM text every 5 s
WINDOW_MS = 5000         # report BPM every 5 seconds


def read_ppg():
    """Return a 12-bit PPG sample (0-4095)."""
    return ppg.read_u16() >> 4


high = low = read_ppg()
above = False
beats = []               # times of detected beats (ms)
last_report = time.ticks_ms()
total = count = 0

while True:
    sample = read_ppg()
    now = time.ticks_ms()

    # --- Beat detection (adaptive threshold, same as before) ---
    high = max(sample, high - 1)
    low = min(sample, low + 1)
    threshold = low + 0.6 * (high - low)
    if sample > threshold and not above and (high - low) > 100:
        above = True
        if not beats or time.ticks_diff(now, beats[-1]) > 300:
            beats.append(now)
    elif sample < threshold:
        above = False

    # --- Plot: average groups of samples ---
    total += sample
    count += 1
    if count == PLOT_EVERY:
        if SHOW_WAVE:
            print(total // count)
        total = count = 0

    # --- BPM every 5 seconds, from the median beat-to-beat interval ---
    if time.ticks_diff(now, last_report) >= WINDOW_MS:
        beats = [t for t in beats if time.ticks_diff(now, t) <= WINDOW_MS]
        intervals = [time.ticks_diff(b, a) for a, b in zip(beats, beats[1:])]
        intervals = [i for i in intervals if 300 < i < 2000]
        if not SHOW_WAVE:
            if len(intervals) >= 2:
                intervals.sort()
                median = intervals[len(intervals) // 2]
                print("BPM:", round(60000 / median), "(", len(intervals) + 1, "beats )")
            else:
                print("BPM: -- (not enough clean beats yet)")
        last_report = now

    time.sleep_ms(SAMPLE_MS)