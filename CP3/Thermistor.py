"""
Reads BOTH the thermistor bridge (via ADC0/GP26) and the BMP280 (I2C),
printing them side by side. Useful for calibrating the thermistor's
linear fit against the BMP280 as a reference.
"""

from machine import ADC, Pin, I2C
import bme280
import time

# ---------------------------------------------------------------------------
# Thermistor bridge -> in-amp -> op-amp -> ADC0
# ---------------------------------------------------------------------------
adc = ADC(Pin(26))            # GP26 = ADC0 = physical pin 31
ADC_VREF = 3.3

V_LOW, T_AT_V_LOW   = 0.00, 30.0   # volts, deg C
V_HIGH, T_AT_V_HIGH = 3.27, 10.0   # volts, deg C

SLOPE = (T_AT_V_HIGH - T_AT_V_LOW) / (V_HIGH - V_LOW)
INTERCEPT = T_AT_V_LOW - SLOPE * V_LOW

ADC_OFFSET_V = 0.000


def read_thermistor_temp():
    raw_16bit = adc.read_u16()
    raw_12bit = raw_16bit >> 4
    voltage = raw_12bit * ADC_VREF / 4095
    voltage -= ADC_OFFSET_V
    v_clamped = max(0.0, min(ADC_VREF, voltage))
    temp_c = SLOPE * v_clamped + INTERCEPT
    return temp_c, voltage, raw_12bit


# ---------------------------------------------------------------------------
# BMP280 (I2C0: GP0=SDA, GP1=SCL) -- reference temperature
# ---------------------------------------------------------------------------
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
sensor = bme280.BME280(i2c=i2c)


def read_bmp280():
    temp, pressure, humidity = sensor.read_compensated_data()
    return temp / 100, pressure / 25600


# ---------------------------------------------------------------------------
# Main loop -- print both side by side
# ---------------------------------------------------------------------------
def main():
    print("Thermistor: T(C) = {:.4f} * V + {:.4f}".format(SLOPE, INTERCEPT))
    print("{:>10} | {:>10} | {:>10} | {:>10}".format(
        "Therm(C)", "Therm(V)", "BMP280(C)", "Diff(C)"))

    while True:
        therm_temp, therm_v, raw = read_thermistor_temp()
        bmp_temp, pressure = read_bmp280()
        diff = therm_temp - bmp_temp

        print("{:10.2f} | {:10.4f} | {:10.2f} | {:10.2f}".format(
            therm_temp, therm_v, bmp_temp, diff))

        time.sleep(1)


if __name__ == "__main__":
    main()