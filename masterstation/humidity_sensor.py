from time import sleep
from os import stat, path, remove
import re

import adafruit_dht
from board import D18

THERM_FILE = "/sys/bus/w1/devices/28-051673fdeeff/w1_slave"
# THERM_FILE = "w1_slave"


def readDHTTemp():
    dht_device = adafruit_dht.DHT22(pin=D18, use_pulseio=False)
    retryCount = 0
    gotValue = False
    # Try reading the device three times
    while not gotValue and retryCount < 5:
        retryCount += 1
        try:
            # dht_device.measure()
            # sleep(2)
            humidity = dht_device.humidity
            # print(f"Humidity: {humidity}")
            temperature = dht_device.temperature
            # print(f"Temp: {temperature}")
            if humidity is not None and temperature is not None:
                gotValue = True
            else:
                sleep(1)
        except RuntimeError as re:
            # print(re)
            sleep(2)
    dht_device.exit()

    if gotValue:
        return (temperature, humidity)
    else:
        return (-100, -100)


def readTemp():
    (temp, humidity) = readDHTTemp()
    if temp == -100:
        print("Failed to read temp from humidity sensor, trying thermometer\n")
        if path.exists(THERM_FILE):
            with open(THERM_FILE, "r", encoding="utf-8") as f:
                str = f.readline()
                # print(f"Temp 1st str: {str}")
                strLen = len(str)
                if str and strLen > 4 and str[strLen - 4] == "Y":
                    # Temp reading is good
                    try:
                        str = f.readline()
                        # print(f"Temp 2nd str: {str}")
                        temp = round(int(re.split("=", str)[1]) / 1000)
                    except:
                        print(f"Temp: Failed")
                        pass
    return (temp, humidity)


# (temperature, humidity) = readTemp()
# if temperature != -1:
#     print(f"{temperature:0.1f} {humidity:0.1f}")
# else:
#     print("FAIL")
