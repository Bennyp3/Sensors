from machine import I2C, Pin
import bme280
import lsm6dsox
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)

# Initialize both sensors on the same bus
bmp = bme280.BME280(i2c=i2c)
imu = lsm6dsox.LSM6DSOX(i2c)

while True:
    # BMP280 readings
    temp, pressure, humidity = bmp.read_compensated_data()
    temp_c = temp / 100
    pressure_hpa = pressure / 25600

    # LSM6DSO readings
    ax, ay, az = imu.accel()
    gx, gy, gz = imu.gyro()

    print(f"Temp: {temp_c:.2f} C, Pressure: {pressure_hpa:.2f} hPa")
    print(f"Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f}")
    print(f"Gyro:  X={gx:.2f} Y={gy:.2f} Z={gz:.2f}")
    print()

    time.sleep(1)