from machine import ADC, Pin
import time

ppg = ADC(Pin(26))      # GP26 (Pin 31)

def read_ppg():
    """Return a 12-bit PPG sample (0-4095)."""
    return ppg.read_u16() >> 4

while True:
    print(read_ppg())
    time.sleep_ms(4)    # 250 samples per second