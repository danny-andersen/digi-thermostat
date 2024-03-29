from ctypes import *
from os import path
import json

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
    _fields_ = [
        ("temp", c_short),
        ("humidity", c_short),
    ]


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
    lastPirTime = 0
    scheduleMsgs: list = []
    tempMotd: str = None
    tempMotdTime = 0
    currentSetTemp: float = -1000.0  # current set temp
    currentBoilerStatus = 0.0  # off
    currentPirStatus = 0  # off
    currentTemp: float = -1000.0
    currentHumidity: float = -1000.0
    currentExtTemp: float = -1000.0
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
