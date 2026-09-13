import network
import time
from secrets import WIFI_SSID, WIFI_PASSWORD

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

for i in range(10):
    status = wlan.status()
    print(f"Status: {status}")
    if status == 3:
        print("Connected!")
        break
    time.sleep(1)

print("Final status:", wlan.status())
print("ifconfig:", wlan.ifconfig())