import network
import time
import urequests
from machine import I2C, Pin
import bme280  # your corrected library
from secrets import WIFI_SSID, WIFI_PASSWORD, THINGSPEAK_KEY
# --- WiFi setup ---


wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

while wlan.status() != 3:
    time.sleep(1)

print("Connected! IP:", wlan.ifconfig()[0])

# --- BMP280 setup ---
i2c = I2C(0, sda=Pin(0), scl=Pin(1), freq=400000)
sensor = bme280.BME280(i2c=i2c)

# --- ThingSpeak setup ---
WRITE_API_KEY = THINGSPEAK_KEY

def send_to_thingspeak(temp, pressure):
    url = f"https://api.thingspeak.com/update?api_key={WRITE_API_KEY}&field1={temp}&field2={pressure}"
    response = urequests.get(url)
    print("ThingSpeak response:", response.text)
    response.close()

# --- Main loop ---
while True:
    temp, pressure, humidity = sensor.read_compensated_data()
    temp_c = temp / 100
    pressure_hpa = pressure / 25600

    print(f"Temp: {temp_c:.2f} C, Pressure: {pressure_hpa:.2f} hPa")
    send_to_thingspeak(temp_c, pressure_hpa)

    time.sleep(20)  # free ThingSpeak accounts need at least 15 sec between updatesS