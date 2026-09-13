from machine import I2C, Pin
import network
import time
import urequests
import bme280
import lsm6dsox
from secrets import WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_KEY

# ============ WiFi Setup ============
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

print("Connecting to WiFi...")
while wlan.status() != 3:
    time.sleep(1)

print("Connected! IP:", wlan.ifconfig()[0])

# ============ Sensor Setup ============
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=100000)

bmp = bme280.BME280(i2c=i2c)
imu = lsm6dsox.LSM6DSOX(i2c)

# ============ ThingSpeak Setup ============
def send_to_thingspeak(temp, pressure, ax, ay, az, gx, gy, gz):
    url = (
        f"https://api.thingspeak.com/update?api_key={THINGSPEAK_KEY}"
        f"&field1={temp}&field2={pressure}"
        f"&field3={ax}&field4={ay}&field5={az}"
        f"&field6={gx}&field7={gy}&field8={gz}"
    )
    response = urequests.get(url)
    print("ThingSpeak response:", response.text)
    response.close()

# ============ Main Loop ============
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

    send_to_thingspeak(temp_c, pressure_hpa, ax, ay, az, gx, gy, gz)

    print()
    time.sleep(20)  # respects ThingSpeak's free-tier 15-second minimum