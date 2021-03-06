//Digital I/O Pins in use
//#define UP_BUTTON 2
//#define DOWN_BUTTON 3
#define ROTARY_A 2
#define ROTARY_B 3
#define BOOST_BUTTON 4
#define THERM_ONE_WIRE_BUS 5
#define RELAY 6
#define GREEN_LED 7
#define RED_LED 8
#define RADIO_CE 9
#define RADIO_CS 10

#define PIR_PIN A0 //Analogue 0

#define ON  1
#define OFF  0
#define LOOP_DELAY 50
#define RTC_READ_INTERVAL 500UL
#define TEMPERATURE_READ_INTERVAL 15000UL
#define SCHED_CHECK_INTERVAL 1000UL
//#define SCROLL_INTERVAL 150UL //Speed at which to scroll message (word shift speed)
#define SCROLL_PAUSE 1200UL //Pause at screen roll
#define RX_TEMP_INTERVAL SEND_TEMP_INTERVAL * 4 * 1000 //msecs, Timeout of temp from masterstation 
#define MAX_NO_MSG_RX_INTERVAL RX_TEMP_INTERVAL * 2  //msecs. If not rx msg in this time, reset radio

//Masterstation defines
#define SEND_TIME_INTERVAL 300UL //secs, Send time from masterstation every 300 secs
#define STATUS_INTERVAL 15UL //secs, how often masterstation polls for status
#define SEND_TEMP_INTERVAL 15UL //secs, Send temperature from masterstation every 15 secs

#define ANALOGUE_HIGH 600
//Schedule and Temp settings
#define MAX_SCHEDULES 18
#define HYSTERSIS 2  //10th Degrees over the set temp to drive the current temp to stop output on/off hysteris
#define DEBOUNCE_TIME 200 //switch must be down for 1000us 
#define BUTTON_HOLD_TIME 300UL //Time button is held down to increment or decrement
#define SET_INTERVAL 1 //Amount to increase the temp by, per button press
#define BOOST_TIME 30*60000 //Length of time to turn on heat regardless of set temperature
#define BACKLIGHT_TIME 30*1000 //30s time to leave on once PIR triggered

#define LCD_ROWS 4
#define LCD_COLS 20

#define EEPROM_I2C_ADDR 0x50

//Comms defines

#define RADIO_CHANNEL 90
#define MASTER_NODE_ID 00 
#define DIGI_THERM_NODE_ID 01

#define REQ_STATUS_MSG  1
#define STATUS_MSG  2
#define SET_TEMP_MSG  3
#define SET_EXT_MSG  4
#define ADJ_SETTIME_CONST_MSG  5
#define MOTD_MSG  6
#define GET_SCHEDULES_MSG  7
#define SCHEDULE_MSG  8
#define DELETE_ALL_SCHEDULES_MSG  9
#define DELETE_SCHEDULE_MSG  10
#define SET_DATE_TIME_MSG  11
#define SET_HOLIDAY_MSG  12
#define SET_THERM_TEMP_MSG  13

#define RESPONSE_TIMEOUT_MS 1000

#define MOTD_FILE "motd.txt"
#define SCHEDULE_FILE "schedule.txt"
#define EXTTEMP_FILE "setExtTemp.txt"
#define SET_TEMP_FILE "setTemp.txt"
#define HOLIDAY_FILE "holiday.txt"

#define MAX_MOTD_SIZE 64
#define MAX_WIND_SIZE 12

//Struct is long word padded...
struct SchedByElem {
    uint16_t day; //= "0" for every day, "0x0100" for Weekday (Mon - Fri), 
                  //"0x0200" for Weekend (Sat, Sun), 1 - Sunday, 2 - Monday, 3 - Tuesday,....7 - Saturday
    uint16_t start; //Start minute (0 - 1440) 
    uint16_t end; //End time minute (0 - 1440)
    int16_t temp; //Set temperature tenths of C, 180 = 18.0C
};

union SchedUnion {
  struct SchedByElem elem;
  uint8_t raw[sizeof(SchedByElem)];
};

struct DateTimeStruct {
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t dayOfWeek;
  uint8_t dayOfMonth;
  uint8_t month;
  uint8_t year;
}; 

struct HolidayDateStr {
  uint8_t hour;
  uint8_t dayOfMonth;
  uint8_t month;
  uint8_t year;
};

struct HolidayByElem {
    struct HolidayDateStr startDate;
    struct HolidayDateStr endDate;
    int16_t holidayTemp;
    uint8_t valid; //Set to 1 if valid
};

union HolidayUnion {
  HolidayByElem elem;
  uint8_t raw[sizeof(HolidayByElem)];
};

union Content {
    struct Status {
      int16_t currentTemp;
      int16_t setTemp;
      uint8_t heatOn;
      uint8_t minsToSet;
      int16_t extTemp;
      uint8_t noOfSchedules;
    } status;
    struct SetTemp {
      int16_t setTemp;
    } setTemp;
    struct SetThermTemp {
      int16_t thermTemp;
    } setTherm;
    struct SetExt {
      int16_t setExt;
      char windStr[MAX_WIND_SIZE];
    } setExt;
    struct AdjSetTimeConstants {
      int16_t degPerHour; //in tenths of a degree - 50 = 5.0
      int16_t extAdjustment; //in tenths
    } adjSetTimeConstants;
    struct MOTD {
      char motdStr[MAX_MOTD_SIZE]; //Message of the day, having max of 64 chars
      unsigned long expiry; //Number of millis after which message expires
    } motd;
    DateTimeStruct dateTime;
    HolidayUnion holiday;
    //Note no update schedule message as is an exact match and must be deleted and inserted to update
    SchedByElem schedule; //supports delete, insert, retreive. 
};

