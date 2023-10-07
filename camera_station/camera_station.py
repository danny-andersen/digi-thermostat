# adding masterstation to the system path
import sys


import configparser
import subprocess
from datetime import datetime
from os import listdir
import fnmatch
from time import sleep
import requests

sys.path.insert(0, "../masterstation")
from humidity_sensor import readTemp

CHECK_WIFI_SCRIPT = "./check_status.sh"

# TODO: Convert to using pyton dropbox library rather than using status script to upload

# TODO: Move these to .ini file
TEMPERATURE_FILE_NEW = "temperature.txt"
HUMIDITY_FILE_NEW = "humidity.txt"
TEMPERATURE_FILE = "temp_avg.txt"
HUMIDITY_FILE = "humidity_avg.txt"
TEMP_AVERAGE_MIN = (
    5  # Number of mins at which to average temp over for historic change file
)
TEMP_AVG_FILE = "temp_avg.txt"
HUMID_AVERAGE_MIN = (
    15  # Number of mins at which to average humidity over for historic change file
)
HUMID_AVG_FILE = "humidity_avg.txt"

MONITOR_PERIOD = 30  # number of seconds between running monitor script
TEMP_PERIOD = 30  # number of seconds between reading temp + humidty


def getTemp(conf, hist: tuple[dict[int, float], dict[int, float]]):
    # Read temp and humidity from sensor and then send them to the masterstation
    (temp, humid) = readTemp()
    # Send to Masterstation
    if temp != -100 and humid != -100:
        sendMessage(conf, {"temp": int(temp * 10), "humidity": int(humid * 10)})
        # Write out temps to file to be used by check status (to be uploaded to dropbox)
        with open(TEMPERATURE_FILE_NEW, mode="w", encoding="utf-8") as f:
            f.write(f"{temp:.1f}\n")
        with open(HUMIDITY_FILE_NEW, mode="w", encoding="utf-8") as f:
            f.write(f"{humid:.1f}\n")
        # Create a rolling 5 min average for temp and humidity to be used by history
        now = datetime.now()
        (histTempD, histHumidD) = hist
        mins = now.minute % TEMP_AVERAGE_MIN
        histTemp = histTempD.get(mins, -100)
        if histTemp == -100:
            histTempD[mins] = temp
        else:
            histTempD[mins] = (temp + histTemp) / 2
        mins = now.minute % TEMP_AVERAGE_MIN
        histHumid = histHumidD.get(mins, -100)
        if histHumid == -100:
            histHumidD[mins] = humid
        else:
            histHumidD[mins] = (humid + histHumid) / 2
        if mins == 0:
            # Calculate temp average
            totalTemp = 0.0
            vals = 0
            for i in range(0, TEMP_AVERAGE_MIN):
                t = histTempD.get(i, -100)
                if t != -100:
                    totalTemp += t
                    vals += 1
                if i != 0:
                    histTempD[i] = -100
            if vals > 1:
                # Only process if it is first time processing this minute
                avgTemp = totalTemp / vals
                with open(TEMP_AVG_FILE, mode="w", encoding="utf-8") as f:
                    f.write(f"{avgTemp:.1f}\n")
            elif vals == 0:
                avgTemp = -100  # No measurement made in period
                with open(TEMP_AVG_FILE, mode="w", encoding="utf-8") as f:
                    f.write(f"{avgTemp:.1f}\n")
        mins = now.minute % HUMID_AVERAGE_MIN
        histHumid = histHumidD.get(mins, -100)
        if histHumid == -100:
            histHumidD[mins] = humid
        else:
            histHumidD[mins] = (humid + histHumid) / 2
        if mins == 0:
            # Calculate humid average
            totalHumid = 0.0
            vals = 0
            for i in range(0, HUMID_AVERAGE_MIN):
                h = histHumidD.get(i, -100)
                if h != -100:
                    totalHumid += h
                    vals += 1
                histHumidD[i] = -100
            if vals > 1:
                # Only process if it is first time processing this minute
                avgHumid = totalHumid / vals
                with open(HUMID_AVG_FILE, mode="w", encoding="utf-8") as f:
                    f.write(f"{avgHumid:.0f}\n")
            elif vals == 0:
                avgHumid = -100  # No measurement made in period
                with open(HUMID_AVG_FILE, mode="w", encoding="utf-8") as f:
                    f.write(f"{avgHumid:.10f}\n")


# def uploadToDropBox(config, remotePath, localFile):
#     dropboxAccessToken = config["dropboxAccessToken"]
#     dbx = dropbox.Dropbox(dropboxAccessToken)
#     dbx.files_upload(
#         dataStr.encode("utf-8"),
#         path,
#         mode=dropbox.files.WriteMode.overwrite,
#         mute=True,
#     )


def checkForMotionEvents(conf):
    # If there is a file in the motion directory, it signifies that a motion event is underway
    video_dir = conf["video_dir"]
    # Check for any mpg files
    mpgs = fnmatch.filter(listdir(video_dir), "*.mp4")
    if len(mpgs):
        # Tell masterstation an event is happening
        sendMessage(conf, {"pir": "1"})


def runScript(conf):
    camera_num = conf["camera_num"]
    video_dir = conf["video_dir"]
    cmd = f"{CHECK_WIFI_SCRIPT} {camera_num} {video_dir}"
    # print(f"Calling check script {cmd}")
    subprocess.run(args=cmd, shell=True, check=False)


def sendMessage(conf, args: dict[str, str]):
    # Format = /message?s=<station number>&rs=<1=rebooted>,&u=<1=update only, no resp msg needed>&t=<thermostat temp>&h=<humidity>&st=<set temp>&r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
    camera_num = conf["camera_num"]
    masterstation_url = conf["masterstation_url"]
    url_parts = [f"{masterstation_url}/message?s={camera_num}&u=1"]
    if conf["reset"] == "True":
        url_parts.append("&rs=1")
        conf["reset"] = "False"
    for arg, value in args.items():
        if "pir" in arg:
            url_parts.append(f"&p={value}")
        elif "temp" in arg:
            url_parts.append(f"&t={value}")
        elif "humid" in arg:
            url_parts.append(f"&h={value}")
        # When raspbian catches up and supports python v3.10+ use the following:
        # match arg:
        #     case "pir":
        #         params.append(f"&p={value}")
        #     case "temp":
        #         params.append(f"&t={value}")
        #     case "humid":
        #         params.append(f"&h={value}")
    url = "".join(url_parts)
    # Send HTTP request with a 5 sec timeout
    # print("Sending status update")
    try:
        resp = requests.get(url, timeout=5)
        # print(f"Received response code {resp.status_code}")
    except requests.exceptions.RequestException as re:
        print(f"Failed to send message to masterstation {re}")


if __name__ == "__main__":
    print("Starting camera monitoring service")
    config = configparser.ConfigParser()
    config.read("./camera_station.ini")
    cfg = config["setup"]
    lastTempTime = 0
    lastMonitorTime = 0

    sleep(15)
    history: tuple[dict[int, float], dict[int, float]] = (dict(), dict())
    cfg["reset"] = "True"
    while True:
        nowTime = datetime.now().timestamp()
        if (nowTime - lastTempTime) > TEMP_PERIOD:
            # Read temp and humidity and update latest files
            lastTempTime = nowTime
            getTemp(cfg, history)
        # Check if any motion file is present - if so flag to masterstation that a motion event is occurring
        checkForMotionEvents(cfg)
        if (nowTime - lastMonitorTime) > MONITOR_PERIOD:
            # Run the monitor script to upload new files, historic temp changes and check wifi still up
            lastMonitorTime = nowTime
            runScript(cfg)
        sleep(1)
