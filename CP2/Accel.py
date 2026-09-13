from machine import I2C, Pin
import lsm6dsox
import time

i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)
imu = lsm6dsox.LSM6DSOX(i2c)

while True:
    ax, ay, az = imu.accel()
    gx, gy, gz = imu.gyro()
    print(f"Accel: X={ax:.2f} Y={ay:.2f} Z={az:.2f}")
    print(f"Gyro:  X={gx:.2f} Y={gy:.2f} Z={gz:.2f}")
    time.sleep(0.5)