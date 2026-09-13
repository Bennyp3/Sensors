from machine import I2C, Pin
import bme280
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)

sensor = bme280.BME280(i2c=i2c)

while True:
    temp, pressure, humidity = sensor.read_compensated_data()
    print(f"Temp: {temp/100:.2f} C, Pressure: {pressure/25600:.2f} hPa")
    time.sleep(1)