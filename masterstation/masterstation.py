from flask import *
from uwsgidecorators import *
from ctypes import *
from datetime import datetime
from os import stat, path, remove
import json
import copy
from multiprocessing import Process
from time import sleep
import subprocess

import crcmod
import crcmod.predefined
from filelock import FileLock, Timeout

from capture_camera import monitorAndRecord
from humidity_sensor import readTemp

MAX_MESSAGE_SIZE = 128
MAX_WIND_SIZE = 12
MAX_MOTD_SIZE = 90

NO_MESSAGE = 0
REQ_STATUS_MSG = 1
STATUS_MSG = 2
SET_TEMP_MSG = 3
SET_EXT_MSG = 4
ADJ_SETTIME_CONST_MSG = 5
MOTD_MSG = 6
GET_SCHEDULES_MSG = 7
SCHEDULE_MSG = 8
DELETE_ALL_SCHEDULES_MSG = 9
DELETE_SCHEDULE_MSG = 10
SET_DATE_TIME_MSG = 11
SET_HOLIDAY_MSG = 12
SET_THERM_TEMP_MSG = 13
RESET_MSG = 14

MOTD_FILE = "motd.txt"
MOTD_EXPIRY_SECS = 3600  # an hour
SCHEDULE_FILE = "schedule.txt"
EXTTEMP_FILE = "setExtTemp.txt"
HOLIDAY_FILE = "holiday.txt"
SET_TEMP_FILE = "setTemp.txt"
STATUS_FILE = "status.txt"
STATION_FILE = "-context.json"
RESET_FILE = "resetReq.txt"
TEMPERATURE_FILE_NEW = "../monitor_home/temperature.txt"
HUMIDITY_FILE_NEW = "../monitor_home/humidity.txt"
TEMPERATURE_FILE = "../monitor_home/temp_avg.txt"
HUMIDITY_FILE = "../monitor_home/humidity_avg.txt"
TEMP_AVERAGE_MIN = (
    5  # Number of mins at which to average temp over for historic change file
)
TEMP_AVG_FILE = "../monitor_home/temp_avg.txt"
HUMID_AVERAGE_MIN = (
    15  # Number of mins at which to average humidity over for historic change file
)
HUMID_AVG_FILE = "../monitor_home/humidity_avg.txt"

MONITOR_SCRIPT = "/home/danny/digi-thermostat/monitor_home/get_lan_devices.sh"
MONITOR_PERIOD = 30  # number of seconds between running monitor script
TEMP_PERIOD = 30  # number of seconds between reading temp + humidty

EXT_TEMP_EXPIRY_SECS = 3600  # an hour
SET_TEMP_EXPIRY_SECS = 30 * 60  # 30 mins
SET_SCHED_EXPIRY_SECS = 30 * 60  # 30 mins
TEMP_MOTD_EXPIRY_SECS = 60  # 1 min


class Status(Structure):
    _fields_ = [
        ("currentTemp", c_short),
        ("setTemp", c_short),
        ("heatOn", c_ubyte),
        ("minsToSet", c_ubyte),
        ("extTemp", c_short),
        ("noOfSchedules", c_ubyte),
    ]


class SetExt(Structure):
    _fields_ = [("setExt", c_short), ("windStr", c_char * MAX_WIND_SIZE)]


class Temp(Structure):
    _fields_ = [("temp", c_short)]


class AdjSetTimeConstants(Structure):
    _fields_ = [
        ("degPerHour", c_short),  # in tenths of a degree - 50 = 5.0
        ("extAdjustment", c_short),
    ]  # in tenths


# class Motd(Structure):
#     _fields_ = [
#         ("motdStr", c_char * MAX_MOTD_SIZE), #Message of the day, with a minimum of one character
#         ("expiry", c_ulong)] #Number of millis after which message expires


class DateTimeStruct(LittleEndianStructure):
    _fields_ = [
        ("sec", c_ubyte),
        ("min", c_ubyte),
        ("hour", c_ubyte),
        ("dayOfWeek", c_ubyte),
        ("dayOfMonth", c_ubyte),
        ("month", c_ubyte),
        ("year", c_ubyte),
    ]


class HolidayDateStr(Structure):
    _fields_ = [
        ("hour", c_ubyte),
        ("dayOfMonth", c_ubyte),
        ("month", c_ubyte),
        ("year", c_ubyte),
    ]


class HolidayStr(Structure):
    _fields_ = [
        ("startDate", HolidayDateStr),
        ("endDate", HolidayDateStr),
        ("temp", c_short),
        ("valid", c_ubyte),
    ]


# Struct is long word padded...
class SchedByElem(LittleEndianStructure):
    _fields_ = [
        ("day", c_ushort),  # = "0" for every day, "0x0100" for Weekday (Mon - Fri),
        # "0x0200" for Weekend (Sat, Sun), 1 - Sunday, 2 - Monday, 3 - Tuesday,....7 - Saturday
        ("start", c_ushort),  # Start minute (0 - 1440)
        ("end", c_ushort),  # End time minute (0 - 1440)
        ("temp", c_short),
    ]  # Set temperature tenths of C, 180 = 18.0C


class ScheduleElement:
    day: int
    start: int
    end: int
    temp: int

    def __init__(self, jsonStr=None):
        if jsonStr:
            load = json.loads(jsonStr)
            self.__dict__.update(**load)

    def getJson(self):
        return json.dumps(self.__dict__)


# class Content(Union):
#     _fields_ = [
#          ("status", Status),
#          ("setTemp", c_short),
#          ("setTherm", c_short),
#          ("setExt", SetExt),
#          ("adjSetTimeConstants", AdjSetTimeConstants),
#          ("motd", Motd),
#         ("dateTime", DateTimeStruct),
#         ("holiday", HolidayUnion),
#         #Note no update schedule message as is an exact match and must be deleted and inserted to update
#         ("schedule", SchedByElem)]


class Message(LittleEndianStructure):
    _fields_ = [("id", c_ubyte), ("len", c_ubyte), ("crc", c_ushort)]


class StationContext:
    stationNo: int = -1
    motdExpiry = MOTD_EXPIRY_SECS
    motdTime: int = 0
    schedSentTime = 0
    extTempTime = 0
    setTempTime = 0
    setHolidayTime = 0
    setSchedTime = 0
    scheduleMsgs: list = []
    tempMotd: str = None
    tempMotdTime = 0
    currentSetTemp = 0.0  # current set temp
    currentBoilerStatus = 0.0  # off
    currentPirStatus = 0  # off
    currentTemp: float = 0.0
    currentHumidity: float = 0.0
    currentExtTemp = 1000.0
    noOfSchedules = 0

    def __init__(self, stn=-1) -> None:
        self.stationNo = stn
        self.getSavedContext()

    # def __init__(self, stn) -> None:
    #     self.stationNo = stn
    #     self.__init__()

    # Gets the current global variables from a file
    # This allows multiple app threads to use the same values
    # Each station is single threaded and so there is no need for locks - the station number is sufficient
    def getSavedContext(self) -> dict:
        stationFile = f"{self.stationNo}{STATION_FILE}"
        loadStatus = {}
        if path.exists(stationFile):
            try:
                with open(stationFile, "r", encoding="utf-8") as f:
                    jsonStr = f.readline()
                    loadStatus = json.loads(jsonStr)
                    self.__dict__.update(**loadStatus)
            except:
                print(
                    f"Failed to load json data from station context file {stationFile}"
                )
        return loadStatus

    def saveStationContext(self, oldContext: object = None):
        # First determine if anything has changed - only update context file if it has
        changed = False
        if self.stationNo != -1 and self != oldContext:
            changed = True
            # save context
            jsonStr = json.dumps(self.__dict__)
            # Write json to station file
            with open(f"{self.stationNo}{STATION_FILE}", "w", encoding="utf-8") as f:
                try:
                    f.write(jsonStr)
                except:
                    print("Failed to write context file")
        return changed

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, StationContext):
            # don't attempt to compare against unrelated types
            return NotImplemented
        return (
            self.currentBoilerStatus == other.currentBoilerStatus
            and self.currentExtTemp == other.currentExtTemp
            and self.currentPirStatus == other.currentPirStatus
            and self.currentSetTemp == other.currentSetTemp
            and self.currentTemp == other.currentTemp
            and self.currentHumidity == other.currentHumidity
            and self.extTempTime == other.extTempTime
            and self.motdExpiry == other.motdExpiry
            and self.motdTime == other.motdTime
            and self.noOfSchedules == other.noOfSchedules
            and self.schedSentTime == other.schedSentTime
            and self.scheduleMsgs == other.scheduleMsgs
            and self.setHolidayTime == other.setHolidayTime
            and self.setTempTime == other.setTempTime
            and self.tempMotd == other.tempMotd
            and self.tempMotdTime == other.tempMotdTime
            and self.stationNo == other.stationNo
        )


def getTemp(history: tuple[dict[int, float], dict[int, float]]):
    # Read temp and humidity from sensor and write them to a file
    # This caters for the fact that reading from these can be slow (seconds)
    # and so allows the web server to read the file quickly and respond quickly
    (temp, humid) = readTemp()
    # Write out temps to be used by masterstation
    with (open(TEMPERATURE_FILE_NEW, mode="w", encoding="utf-8") as f):
        f.write(f"{temp:.1f}\n")
    with (open(HUMIDITY_FILE_NEW, mode="w", encoding="utf-8") as f):
        f.write(f"{humid:.1f}\n")
    # Create a rolling 5 min average for temp and humidity to be used by history
    now = datetime.now()
    (histTempD, histHumidD) = history
    min = now.minute % TEMP_AVERAGE_MIN
    histTemp = histTempD.get(min, -100)
    if histTemp == -100:
        histTempD[min] = temp
    else:
        histTempD[min] = (temp + histTemp) / 2
    min = now.minute % TEMP_AVERAGE_MIN
    histHumid = histHumidD.get(min, -100)
    if histHumid == -100:
        histHumidD[min] = humid
    else:
        histHumidD[min] = (humid + histHumid) / 2
    if min == 0:
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
            with (open(TEMP_AVG_FILE, mode="w", encoding="utf-8") as f):
                f.write(f"{avgTemp:.1f}\n")
        elif vals == 0:
            avgTemp = -100  # No measurement made in period
            with (open(TEMP_AVG_FILE, mode="w", encoding="utf-8") as f):
                f.write(f"{avgTemp:.1f}\n")
    min = now.minute % HUMID_AVERAGE_MIN
    histHumid = histHumidD.get(min, -100)
    if histHumid == -100:
        histHumidD[min] = humid
    else:
        histHumidD[min] = (humid + histHumid) / 2
    if min == 0:
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
            with (open(HUMID_AVG_FILE, mode="w", encoding="utf-8") as f):
                f.write(f"{avgHumid:.0f}\n")
        elif vals == 0:
            avgHumid = -100  # No measurement made in period
            with (open(HUMID_AVG_FILE, mode="w", encoding="utf-8") as f):
                f.write(f"{avgHumid:.10f}\n")


def runScript():
    print("Running monitoring script")
    subprocess.run(args=MONITOR_SCRIPT, shell=True)


def runMonitorScript():
    print(f"******Starting monitor script thread loop {MONITOR_SCRIPT}")
    lastTempTime = 0
    lastMonitorTime = 0
    sleep(15)
    history: tuple[dict[int, float], dict[int, float]] = (dict(), dict())
    # Wait until server up and running
    while True:
        nowTime = datetime.now().timestamp()
        if (nowTime - lastTempTime) > TEMP_PERIOD:
            # Read temp and humidity and update latest files
            lastTempTime = nowTime
            getTemp(history)
        nowTime = datetime.now().timestamp()
        if (nowTime - lastMonitorTime) > MONITOR_PERIOD:
            # Run the monitor script
            lastMonitorTime = nowTime
            runScript()
        sleep(1)


lock = FileLock("monitor_thread.lock")
app = Flask(__name__)


@postfork
def setup():
    global lock
    try:
        if lock.acquire(1):
            print("Starting monitoring threads")
            # Only one thread gets the lock
            cameraProcess = Process(target=monitorAndRecord, daemon=False)
            cameraProcess.start()
            monitorScriptProcess = Process(target=runMonitorScript, daemon=True)
            monitorScriptProcess.start()
            sleep(10)
            lock.release()
    except Timeout:
        print("This thread skipping creating monitoring threads")
        pass


def getNoMessage():
    msgBytes = getMessageEnvelope(NO_MESSAGE, bytearray(0), 0)
    return Response(response=msgBytes, mimetype="application/octet-stream")


def getMessageEnvelope(id, content: bytearray, len):
    msg = Message()
    msg.id = id
    msg.len = len + 4
    msg.crc = 0
    msgBytes = bytearray(msg) + content
    # print(f"len: {msg.len}, msg: {msgBytes}")
    crc_func = crcmod.predefined.mkCrcFun("crc-aug-ccitt")
    crc = crc_func(msgBytes) & 0xFFFF
    # print(f"CRC crc-aug-ccitt fun: {crc:02x}")
    # msg.crc = short(crc + 2**16).to_bytes(2, 'little')
    # msg.crc = crc.to_bytes(2, 'little')
    msg.crc = crc
    return bytearray(msg) + content


def retrieveTempValue(primary, secondary):
    retValue: float = -1000.0
    try:
        with open(primary, "r", encoding="utf-8") as f:
            str = f.readline()
            # Mulitply by 10 as temp in .1 degrees
            retValue = float(str) * 10
    except:
        try:
            with open(secondary, "r", encoding="utf-8") as f:
                str = f.readline()
                # print(f"Set temp str {str[:strLen-1]}")
                # Mulitply by 10 as humidity in .1 degrees
                retValue = float(str) * 10
        except:
            retValue = -1000.0
    return retValue


def readThermStr():
    # Read temp from temp file - use new file if it exist, otherwise use old file
    temp = retrieveTempValue(TEMPERATURE_FILE_NEW, TEMPERATURE_FILE)
    humidity = retrieveTempValue(HUMIDITY_FILE_NEW, HUMIDITY_FILE)
    return (temp, humidity)


def createThermMsg(temp: float):
    tempMsg = Temp()
    tempMsg.temp = c_int16(int(temp))
    msgBytes = getMessageEnvelope(SET_THERM_TEMP_MSG, bytearray(tempMsg), sizeof(Temp))
    response = Response(response=msgBytes, mimetype="application/octet-stream")
    # print(f"Temp: {tempMsg.temp}")
    return response


def generateStatusFile(sc: StationContext):
    # print("Generate status file")
    now: datetime = datetime.now()
    try:
        with open(STATUS_FILE, "w", encoding="utf-8") as statusf:
            statusf.write(f"Current temp: {sc.currentTemp/10:0.1f}\n")
            statusf.write(f"Current humidity: {sc.currentHumidity/10:0.1f}\n")
            statusf.write(f"Current set temp: {sc.currentSetTemp/10:0.1f}\n")
            heatOn = "No" if sc.currentBoilerStatus == 0 else "Yes"
            statusf.write(f"Heat on? {heatOn}\n")
            statusf.write(f"Mins to set temp: {sc.currentBoilerStatus}\n")
            if sc.currentExtTemp < 1000:
                statusf.write(f"External temp: {sc.currentExtTemp/10:0.2f}\n")
            else:
                statusf.write("External temp: Not Set\n")
            statusf.write(f"No of Schedules: {sc.noOfSchedules}\n")
            statusf.write(f"Last heard time: {now.strftime('%Y%m%d %H:%M:%S')}\n")
            statusf.write(f"PIR:{sc.currentPirStatus}\n")

    except:
        print("Failed to write status file")


# Format = /message?t=<thermostat temp>&st=<set temp>&r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
@app.route("/message", methods=["GET"])
def getMessage():
    args = request.args
    stn = args.get("s", type=int)
    if stn:
        sc: StationContext = StationContext(stn)
    else:
        # No station number given - use default context
        sc: StationContext = StationContext()
    startContext = copy.deepcopy(sc)
    resend = args.get("rs", type=int)
    if resend > 0:
        # Thermostat has rebooted or reconnected - resend any messages
        if resend == 1:
            print(f"RESET OF STATION {sc.stationNo} DETECTED")
        elif resend == 2:
            print(f"WIFI RESET OF STATION {sc.stationNo} DETECTED")
        sc.motdTime = 0
        sc.tempMotdTime = 0
        sc.setTempTime = 0
        sc.setHolidayTime = 0
        sc.extTempTime = 0
        sc.setSchedTime = 0
        sc.currentHumidity = 0
        sc.motdExpiry = MOTD_EXPIRY_SECS
        sc.scheduleMsgs = []
    temp = args.get("t", type=float)
    if temp:
        sc.currentTemp = temp
    therm = args.get("st", type=float)
    if therm:
        sc.currentSetTemp = therm
    heat = args.get("r", type=int)
    if heat or (heat == 0 and sc.currentBoilerStatus > 0):
        sc.currentBoilerStatus = heat
    pir = args.get("p", type=int)
    if (pir and not sc.currentPirStatus) or (not pir and sc.currentPirStatus):
        sc.currentPirStatus = pir
        generateStatusFile(sc)
    if sc.motdExpiry < TEMP_MOTD_EXPIRY_SECS:
        sc.motdExpiry = TEMP_MOTD_EXPIRY_SECS  # Have a minimum expiry time

    checkAndDeleteFile(MOTD_FILE, sc.motdExpiry)
    checkAndDeleteFile(SET_TEMP_FILE, SET_TEMP_EXPIRY_SECS)
    checkAndDeleteFile(EXTTEMP_FILE, EXT_TEMP_EXPIRY_SECS)
    checkAndDeleteFile(HOLIDAY_FILE, SET_TEMP_EXPIRY_SECS)
    checkAndDeleteFile(SCHEDULE_FILE, SET_SCHED_EXPIRY_SECS)
    if not path.exists(SCHEDULE_FILE):
        sc.setSchedTime = 0
    if sc.tempMotdTime and (
        datetime.now().timestamp() - sc.tempMotdTime > TEMP_MOTD_EXPIRY_SECS
    ):
        # Check if temp motd has expired
        sc.tempMotdTime = 0
        sc.motdTime = 0  # Resend any motd
    response: Response = getNoMessage()
    if (
        len(sc.scheduleMsgs) == 0
        and path.exists(SCHEDULE_FILE)
        and stat(SCHEDULE_FILE).st_mtime > sc.setSchedTime
    ):
        createScheduleMsgs(sc)
    # TODO Send info motd after sending schedule, holiday, temp etc.
    # Revert to motd after 5 mins
    if len(sc.scheduleMsgs) > 0:
        response = getNextScheduleMsg(sc)
        print(f"Number of schedules remaining to send: {len(sc.scheduleMsgs)}")
    elif path.exists(RESET_FILE):
        response = createResetMsg()
    elif path.exists(SET_TEMP_FILE) and stat(SET_TEMP_FILE).st_mtime > sc.setTempTime:
        response = getSetTemp(sc)
    elif path.exists(HOLIDAY_FILE) and stat(HOLIDAY_FILE).st_mtime > sc.setHolidayTime:
        response = getHoliday(sc)
    elif path.exists(EXTTEMP_FILE) and stat(EXTTEMP_FILE).st_mtime > sc.extTempTime:
        response = getExtTemp(sc)
    elif sc.tempMotd:
        response = createMotd(sc.tempMotd)
        sc.tempMotd = None
        sc.tempMotdTime = datetime.now().timestamp()
        # motdTime = 0 #Resend motd after temp motd has expired
    elif (
        not sc.tempMotdTime
        and path.exists(MOTD_FILE)
        and stat(MOTD_FILE).st_mtime > sc.motdTime
    ):
        # Only send new motd if temp Motd has timed out
        response = getMotd(sc)
        sc.motdTime = stat(MOTD_FILE).st_mtime
    else:
        (temp, humidity) = readThermStr()
        # TODO: Change thermostat and masterstation to return and display humidity (on top line?)
        if humidity != -1000:
            sc.currentHumidity = humidity
        if temp != -1000:
            sc.currentTemp = temp
            response = createThermMsg(temp)
        else:
            pass  # Do something else

    sc.saveStationContext(startContext)
    # Always generate status file to update last heard time
    generateStatusFile(sc)

    return response


@app.route("/datetime", methods=["POST", "GET"])
def getDateTime():
    now = datetime.now()
    dt = DateTimeStruct()
    dt.sec = now.second
    dt.min = now.minute
    dt.hour = now.hour
    dt.dayOfWeek = now.isoweekday()  # 1 = Monday, 7 = Sunday, to match RTC
    dt.dayOfMonth = now.day
    dt.month = now.month
    dt.year = now.year - 2000
    with open("dateMsg", "w", encoding="utf-8") as f:
        f.write(f"Time: {dt.hour}:{dt.min}:{dt.sec}")
    msgBytes = getMessageEnvelope(
        SET_DATE_TIME_MSG, bytearray(dt), sizeof(DateTimeStruct)
    )
    print(f"DateTime msg")

    return Response(response=msgBytes, mimetype="application/octet-stream")


@app.route("/motd", methods=["GET"])
def getMotd(sc: StationContext = None):
    readSC = False
    if not sc:
        args = request.args
        stn = args.get("s", type=int)
        if stn:
            sc: StationContext = StationContext(stn)
            readSC = True
        else:
            # No station number given - use default context
            sc: StationContext = StationContext()
    str = ""
    checkAndDeleteFile(MOTD_FILE, sc.motdExpiry)
    if path.exists(MOTD_FILE):
        with open(MOTD_FILE, "r", encoding="utf-8") as f:
            str = f.readline()
            exp = int(f.readline())
            sc.motdExpiry = (
                MOTD_EXPIRY_SECS if exp == "" else exp - 10000
            )  # Expire 10s before the thermostat does
            if sc.motdExpiry < TEMP_MOTD_EXPIRY_SECS:
                sc.motdExpiry = (
                    TEMP_MOTD_EXPIRY_SECS  # Have a minimum expiry time of 60 seconds
                )
            sc.motdTime = stat(MOTD_FILE).st_mtime
            sc.tempMotdTime = 0
            str = str[: len(str) - 1]
            resp = createMotd(str, exp)
    else:
        resp = getDefaultMotd()
        sc.motdTime = 0
        sc.tempMotdTime = 0
    if readSC:
        sc.saveStationContext()
    return resp


def getDefaultMotd():
    str = "No weather forecast, please wait....."
    return createMotd(str, TEMP_MOTD_EXPIRY_SECS * 1000)


def createResetMsg():
    remove(RESET_FILE)
    msgBytes = getMessageEnvelope(RESET_MSG, bytearray(0), 0)
    return Response(response=msgBytes, mimetype="application/octet-stream")


def createMotd(str, motdExpiry=TEMP_MOTD_EXPIRY_SECS * 1000):
    strLen = len(str) if len(str) < MAX_MOTD_SIZE - 1 else MAX_MOTD_SIZE - 1
    motdStr = bytes(str[:strLen], encoding="utf-8")
    motdLen = len(motdStr) + 1
    # print(f"Str len; {strLen}")
    class Motd(Structure):
        _fields_ = [
            ("expiry", c_uint32),  # Number of millis after which message expires
            ("motdStr", c_char * motdLen),
        ]  # Message of the day, with a minimum of one character

    motd = Motd()
    motd.motdStr = motdStr
    motd.expiry = motdExpiry
    print(f"Motd: {motd.motdStr} Expiry: {motd.expiry}")
    print(f"String Len: {motdLen} Len of content: {sizeof(Motd)}")
    msgBytes = getMessageEnvelope(MOTD_MSG, bytearray(motd), sizeof(Motd))
    return Response(response=msgBytes, mimetype="application/octet-stream")


@app.route("/settemp", methods=["GET"])
def getSetTemp(sc: StationContext = None):
    readSC = False
    changed = False
    if not sc:
        args = request.args
        stn = args.get("s", type=int)
        if stn:
            sc: StationContext = StationContext(stn)
            readSC = True
        else:
            # No station number given - use default context
            sc: StationContext = StationContext()
    tempMsg = Temp()
    gotTemp = False
    response: Response = None
    checkAndDeleteFile(SET_TEMP_FILE, SET_TEMP_EXPIRY_SECS)
    if path.exists(SET_TEMP_FILE):
        with open(SET_TEMP_FILE, "r", encoding="utf-8") as f:
            try:
                str = f.readline()
                strLen = len(str)
                # print(f"Set temp str {str[:strLen-1]}")
                temp = c_int16(int(float(str[: strLen - 1]) * 10))
                tempMsg.temp = temp
                print(f"Set Temp {temp}")
                msgBytes = getMessageEnvelope(
                    SET_TEMP_MSG, bytearray(tempMsg), sizeof(Temp)
                )
                response = Response(
                    response=msgBytes, mimetype="application/octet-stream"
                )
                gotTemp = True
                sc.setTempTime = stat(SET_TEMP_FILE).st_mtime + 1
                if readSC:
                    changed = True

            except:
                print(f"Set Temp: Failed")
                pass
    if not gotTemp:
        # print(f"No set temp file")
        response = getNoMessage()
    if changed:
        sc.saveStationContext()
    generateStatusFile(sc)

    return response


@app.route("/exttemp", methods=["GET"])
def getExtTemp(sc: StationContext = None):
    extMsg = SetExt()
    gotExt = False
    response: Response = None
    readSC = False
    changed = False
    if not sc:
        args = request.args
        stn = args.get("s", type=int)
        if stn:
            sc: StationContext = StationContext(stn)
            readSC = True
        else:
            # No station number given - use default context
            sc: StationContext = StationContext()
    checkAndDeleteFile(EXTTEMP_FILE, EXT_TEMP_EXPIRY_SECS)
    if path.exists(EXTTEMP_FILE):
        with open(EXTTEMP_FILE, "r", encoding="utf-8") as f:
            try:
                str = f.readline()
                strLen = len(str)
                extMsg.setExt = c_int16(int(float(str[: strLen - 1]) * 10))
                sc.currentExtTemp = extMsg.setExt
                print(f"Ext Temp {extMsg.setExt}")
                str = f.readline()
                strLen = len(str)
                if strLen > MAX_WIND_SIZE:
                    strLen = MAX_WIND_SIZE
                extMsg.windStr = bytes(str[: strLen - 1], encoding="utf-8")
                print(f"Wind str {extMsg.windStr}")
                msgBytes = getMessageEnvelope(
                    SET_EXT_MSG, bytearray(extMsg), sizeof(SetExt)
                )
                response = Response(
                    response=msgBytes, mimetype="application/octet-stream"
                )
                gotExt = True
                sc.extTempTime = stat(EXTTEMP_FILE).st_mtime + 1
                if readSC:
                    changed = True
            except:
                print(f"Ext Temp: Failed")
                pass
    if not gotExt:
        print(f"No ext temp file")
        currentExtTemp = 1000.0
        response = getNoMessage()
    if changed:
        sc.saveStationContext()
    generateStatusFile(sc)

    return response


@app.route("/temp", methods=["GET"])
def getThermTemp(sc: StationContext = StationContext()):
    # TODO: Change thermostat and masterstation to return and display humidity (on top line?)
    response: Response = None
    (temp, humidity) = readThermStr()
    if temp != -1000:
        sc.currentTemp = temp
        response = createThermMsg(temp)
    else:
        print(f"No temp file")
        response = getNoMessage()
    if humidity != -1000:
        sc.currentHumidity = float(humidity)
    return response


@app.route("/holiday", methods=["POST", "GET"])
def getHoliday(sc: StationContext = None):
    gotHoliday = False
    response: Response = None
    readSC = False
    changed = False
    if not sc:
        args = request.args
        stn = args.get("s", type=int)
        if stn:
            sc: StationContext = StationContext(stn)
            readSC = True
        else:
            # No station number given - use default context
            sc: StationContext = StationContext()
    checkAndDeleteFile(HOLIDAY_FILE, SET_TEMP_EXPIRY_SECS)
    if path.exists(HOLIDAY_FILE):
        with open(HOLIDAY_FILE, "r", encoding="utf-8") as f:
            try:
                str = f.readline()
                start = str.split(",")
                if readSC:
                    changed = True
                if "Start" in start[0]:
                    startDate = HolidayDateStr()
                    startDate.year = int(start[1])
                    startDate.month = int(start[2])
                    startDate.dayOfMonth = int(start[3])
                    startDate.hour = int(start[4])
                    str = f.readline()
                    end = str.split(",")
                    if "End" in end[0]:
                        endDate = HolidayDateStr()
                        endDate.year = int(end[1])
                        endDate.month = int(end[2])
                        endDate.dayOfMonth = int(end[3])
                        endDate.hour = int(end[4])
                        str = f.readline()
                        temp = str.split(",")
                        if "Temp" in temp[0]:
                            holiday = HolidayStr()
                            holiday.startDate = startDate
                            holiday.endDate = endDate
                            holiday.temp = c_int16(int(float(temp[1]) * 10))
                            holiday.valid = 1
                            msgBytes = getMessageEnvelope(
                                SET_HOLIDAY_MSG, bytearray(holiday), sizeof(HolidayStr)
                            )
                            response = Response(
                                response=msgBytes, mimetype="application/octet-stream"
                            )
                            gotHoliday = True
                            sc.setHolidayTime = stat(HOLIDAY_FILE).st_mtime + 1
                            print(f"Holiday msg")
                            sc.tempMotd = f"New Hols: St:{startDate.dayOfMonth}/{startDate.month} {startDate.hour} To:{endDate.dayOfMonth}/{endDate.month} {endDate.hour}"
            except TypeError as error:
                print(f"Set Holiday: Failed: {error}")
                pass
    if not gotHoliday:
        print(f"No holiday file")
        response = getNoMessage()
    if changed:
        sc.saveStationContext()
    generateStatusFile(sc)

    return response


def createScheduleMsgs(sc: StationContext):
    # Create a list of schedule messages to replace the existing schedules
    messages: list[str] = []

    checkAndDeleteFile(SCHEDULE_FILE, SET_SCHED_EXPIRY_SECS)
    if path.exists(SCHEDULE_FILE):
        sc.setSchedTime = stat(SCHEDULE_FILE).st_mtime + 1
        with open(SCHEDULE_FILE, "r", encoding="utf-8") as f:
            messages.append(None)
            try:
                for str in f:
                    # Read line, split by "," then process + create one message per line (schedule)
                    schedMsg: ScheduleElement = ScheduleElement()
                    sched = str.split(",")
                    schedMsg.day = 0xFF
                    if "Mon-Sun" in sched[0]:
                        schedMsg.day = 0x0000
                    elif "Mon-Fri" in sched[0]:
                        schedMsg.day = 0x0100
                    elif "Sat-Sun" in sched[0]:
                        schedMsg.day = 0x0200
                    elif "Sun" in sched[0]:
                        schedMsg.day = 0x0007
                    elif "Mon" in sched[0]:
                        schedMsg.day = 0x0001
                    elif "Tue" in sched[0]:
                        schedMsg.day = 0x0002
                    elif "Wed" in sched[0]:
                        schedMsg.day = 0x0003
                    elif "Thu" in sched[0]:
                        schedMsg.day = 0x0004
                    elif "Fri" in sched[0]:
                        schedMsg.day = 0x0005
                    elif "Sat" in sched[0]:
                        schedMsg.day = 0x0006
                    else:
                        print(f"Unidentified Day specified in schedule: {str}")
                    if schedMsg.day != 0xFF:
                        shours = int(sched[1][0:2])
                        smins = int(sched[1][2:4])
                        schedMsg.start = shours * 60 + smins
                        ehours = int(sched[2][0:2])
                        emins = int(sched[2][2:4])
                        schedMsg.end = ehours * 60 + emins
                        temp = float(sched[3]) * 10
                        schedMsg.temp = int(temp)
                        messages.append(schedMsg.getJson())
                        # print(f"Schedule: Day: {schedMsg.day} Start: {schedMsg.start}, End: {schedMsg.end}, Temp: {schedMsg.temp}\n")
            except:
                print(f"Processing schedule file failed")
                messages = []
                pass

    noMsgs = len(messages)
    if noMsgs <= 1:
        # File was empty - dont send the delete message
        sc.scheduleMsgs = []
        sc.noOfSchedules = 0
    else:
        sc.tempMotd = f"Rx {noMsgs-1} Schedules"
        sc.scheduleMsgs = messages
        sc.noOfSchedules = noMsgs - 1
    return


def getNextScheduleMsg(sc: StationContext):
    schedStr = sc.scheduleMsgs[0]
    if schedStr == None:
        # First delete all schedules
        msgBytes = getMessageEnvelope(DELETE_ALL_SCHEDULES_MSG, bytearray(0), 0)
    else:
        sched: ScheduleElement = ScheduleElement(schedStr)
        schedMsg: SchedByElem = SchedByElem()
        schedMsg.day = sched.day
        schedMsg.start = sched.start
        schedMsg.end = sched.end
        schedMsg.temp = sched.temp
        msgBytes = getMessageEnvelope(
            SCHEDULE_MSG, bytearray(schedMsg), sizeof(SchedByElem)
        )
    sc.scheduleMsgs = sc.scheduleMsgs[1:]
    return Response(response=msgBytes, mimetype="application/octet-stream")


def checkAndDeleteFile(filename, expiry: int):
    if path.exists(filename):
        nowtime = datetime.now().timestamp()
        if nowtime > (stat(filename).st_mtime + expiry):
            # File has expired
            remove(filename)


if __name__ == "__main__":
    setup()
    app.run(debug=True)
