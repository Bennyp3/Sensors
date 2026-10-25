from machine import ADC, Pin
import time

# ---- Wiring (Pico 2 W) ----
# AD8232 OUTPUT -> GP27 (Pin 32, ADC1)
# AD8232 LO+    -> GP14 (Pin 19)
# AD8232 LO-    -> GP15 (Pin 20)
# AD8232 3.3v   -> 3V3 OUT (Pin 36)
# AD8232 SDN    -> 3V3 OUT (keeps the chip on)
# AD8232 GND    -> GND (Pin 38)

ecg = ADC(Pin(27))
lo_plus = Pin(14, Pin.IN)
lo_minus = Pin(15, Pin.IN)

MODE = "bpm"     # "plot" = waveform for Thonny's plotter, "bpm" = heart rate
SAMPLE_MS = 4     # 250 samples per second


def leads_off():
    """Return which electrode is off, or None if both are connected."""
    if lo_plus.value() and lo_minus.value():
        return "both"
    if lo_plus.value():
        return "LO+ electrode"
    if lo_minus.value():
        return "LO- electrode"
    return None


def read_ecg():
    """Return a 12-bit ECG sample (0-4095), or None if an electrode is off."""
    if leads_off():
        return None
    return ecg.read_u16() >> 4


def run_plot():
    # Prints numbers only, so Thonny's plotter can graph them
    while True:
        sample = read_ecg()
        print(0 if sample is None else sample)
        time.sleep_ms(SAMPLE_MS)


def run_bpm():
    high = low = 2048      # running envelope of the signal
    above = False
    last_peak = None
    last_warning = time.ticks_ms()

    while True:
        sample = read_ecg()
        now = time.ticks_ms()

        if sample is None:
            if time.ticks_diff(now, last_warning) > 1000:
                print("Electrode off:", leads_off())
                last_warning = now
            last_peak = None
            time.sleep_ms(SAMPLE_MS)
            continue

        # Track the signal's top and bottom; both slowly relax toward the signal
        high = max(sample, high - 1)
        low = min(sample, low + 1)
        threshold = low + 0.6 * (high - low)

        if sample > threshold and not above and (high - low) > 100:
            above = True
            if last_peak is None:
                last_peak = now
            else:
                rr = time.ticks_diff(now, last_peak)   # ms between beats
                if rr > 300:                           # ignore double counts (>200 BPM)
                    if rr < 2000:                      # ignore gaps (<30 BPM)
                        print("BPM:", round(60000 / rr))
                    last_peak = now
        elif sample < threshold:
            above = False

        time.sleep_ms(SAMPLE_MS)


if MODE == "plot":
    run_plot()
else:
    run_bpm()