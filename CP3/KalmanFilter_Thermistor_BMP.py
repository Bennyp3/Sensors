"""
Thermistor bridge (ADC0/GP26) + Kalman filter + BMP280 reference, printed
side by side. The Kalman filter smooths the THERMISTOR's own noisy readings
over time -- it does NOT use the BMP280 as an input. The BMP280 is only
there as an independent reference so you can see how well the filtered
thermistor tracks reality (this is your "before vs after" comparison).
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
INTERCEPT = T_AT_V_LOW + 1.2 - SLOPE * V_LOW

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
# BMP280 (I2C0: GP0=SDA, GP1=SCL) -- reference temperature only
# ---------------------------------------------------------------------------
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
sensor = bme280.BME280(i2c=i2c)


def read_bmp280():
    temp, pressure, humidity = sensor.read_compensated_data()
    return temp / 100, pressure / 25600


# ---------------------------------------------------------------------------
# Scalar Kalman filter -- same three formulas from lecture, with a process-
# noise term (Q) added back in during the prediction step so E_EST doesn't
# shrink to zero and freeze the filter over a long run:
#   (0) E_EST_{t-1} <- E_EST_{t-1} + Q      [prediction: add process noise]
#   (1) KG      = E_EST / (E_EST + E_MEA)
#   (2) EST_t   = EST_{t-1} + KG * (MEA - EST_{t-1})
#   (3) E_EST_t = (1 - KG) * E_EST_{t-1}
# ---------------------------------------------------------------------------
class KalmanFilter:
    def __init__(self, initial_estimate, initial_error_est, error_mea, process_noise):
        self.EST = initial_estimate     # EST_{t-1}, starts as your first guess
        self.E_EST = initial_error_est  # E_EST_{t-1}, your initial confidence
        self.E_MEA = error_mea          # E_MEA, fixed -- your sensor's noise
        self.Q = process_noise          # process noise -- keeps filter "alive"

    def update(self, MEA):
        # (0) Predict: inflate the error estimate by Q before using it.
        #     This is the only change from the pure 3-formula version --
        #     it stops E_EST from decaying to 0 and freezing the filter.
        self.E_EST = self.E_EST + self.Q

        # (1) Calculating the Kalman gain
        KG = self.E_EST / (self.E_EST + self.E_MEA)

        # (2) Calculating the current estimate
        EST_t = self.EST + KG * (MEA - self.EST)

        # (3) Calculating the new estimate error
        E_EST_t = (1 - KG) * self.E_EST

        # store for next iteration
        self.EST = EST_t
        self.E_EST = E_EST_t

        return EST_t


# TUNE THESE:
#   error_mea (E_MEA)  = variance of thermistor temp readings at STEADY temp
#     -> collect ~30 readings sitting still, compute statistics.variance(readings)
#   initial_error_est  = how much you trust your very first guess
#   process_noise (Q)  = how much E_EST is allowed to "regrow" each step
#     -> small (0.001-0.01): keeps the filter smooth but still responsive
#     -> larger Q = filter trusts new measurements more, tracks faster, less smoothing
#     -> Q = 0 reduces this back to the exact 3-formula version (will freeze over time)
kf = KalmanFilter(initial_estimate=25.0, initial_error_est=1.0,
                   error_mea=0.25, process_noise=0.01)


# ---------------------------------------------------------------------------
# Main loop -- raw thermistor, filtered thermistor, and BMP280 side by side
# ---------------------------------------------------------------------------
def main():
    print("T(C) = {:.4f} * V + {:.4f}".format(SLOPE, INTERCEPT))
    print("{:>10} | {:>10} | {:>10} | {:>10}".format(
        "Raw(C)", "Filt(C)", "BMP280(C)", "Diff(C)"))

    while True:
        raw_temp, therm_v, raw = read_thermistor_temp()
        filtered_temp = kf.update(raw_temp)
        bmp_temp, pressure = read_bmp280()
        diff = filtered_temp - bmp_temp

        print("{:10.2f} | {:10.2f} | {:10.2f} | {:10.2f}".format(
            raw_temp, filtered_temp, bmp_temp, diff))

        time.sleep(1)


if __name__ == "__main__":
    main()