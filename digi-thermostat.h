// Digital I/O Pins in use
// #define UP_BUTTON 2
// #define DOWN_BUTTON 3
#define ROTARY_A 2
#define ROTARY_B 3
#define BOOST_BUTTON 4
#define THERM_ONE_WIRE_BUS 5
#define RELAY 6
#define GREEN_LED 7
#define RED_LED 8
#define WIFI_RX 10
#define WIFI_TX 11

#define PIR_PIN A0 // Analogue 0

#define ON 1
#define OFF 0
#define LOOP_DELAY 50
#define RTC_READ_INTERVAL 500UL
#define TEMPERATURE_READ_INTERVAL 15000UL // 15 secs
#define LOCAL_TEMP_ADJUST 35              // Local thermister reads 3.5C over - so reduce reading
#define SCHED_CHECK_INTERVAL 1000UL
#define MESSAGE_CHECK_INTERVAL 10000UL // 10 secs
// #define SCROLL_INTERVAL 150UL //Speed at which to scroll message (word shift speed)
#define SCROLL_PAUSE 1200UL        // Pause at screen roll
#define RX_TEMP_INTERVAL 300000UL  // msecs, 5 min timeout of temp from masterstation
#define GET_TIME_INTERVAL 300000UL // Get time from masterstation every 5 mins
#define TEMP_MOTD_TIME 20000UL     // Time to show status msg - 20 secs

#define ANALOGUE_HIGH 600
// Schedule and Temp settings
#define MAX_SCHEDULES 18
#define HYSTERSIS 1              // 10th Degrees over the set temp to drive the current temp to stop output on/off hysteris
#define DEBOUNCE_TIME 200        // switch must be down for 1000us
#define SET_INTERVAL 1           // Amount to increase the temp by, per button press
#define HOLIDAY_SET_TIME 30000UL // 1min to manually set period to go into holiday mode using dial
#define BACKLIGHT_TIME 30000UL   // 30s time to leave on once PIR triggered

#define LCD_ROWS 4
#define LCD_COLS 20
#define TEMP_SIZE 6

#define EEPROM_I2C_ADDR 0x50

// Wifi

#define STATION_NUMBER 1
char LITERAL_STATUS[] = "%s";
char SERVER_STATUS[] = "Server %s";
char MESSAGE_FAIL[] = "Msg %s";
//
#define ESP_AT_BAUD 9600

#define MAX_RESPONSE_TIME 2000 // millis to wait for a response (byte to byte)
#define MAX_STATUS_TIME 500    // millis to wait for a status response
#define RECONNECT_WAIT_TIME 10000

#define MAX_GET_MSG_SIZE 60 // Max size of dynamic get msg with params
#define MAX_MOTD_SIZE 90
#define MAX_MESSAGE_SIZE MAX_MOTD_SIZE
#define BUFF_SIZE MAX_MESSAGE_SIZE + 6

uint8_t buff[BUFF_SIZE];
char nextMessage[MAX_GET_MSG_SIZE];
char getMessage[MAX_GET_MSG_SIZE];
char EOL[] = "EOL";

// Status message format = /message?s=<station number>&rs=<resend messages, e.g. on a reboot>t=<temp>st=<thermostat set temp>&r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
char NEXT_MESSAGE_TEMPLATE[] = "message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d";
char GET_TEMPLATE[] = "*G%02d%s";
char dateTime[] = "datetime";
char thermTemp[] = "temp";
char setTemp[] = "settemp";
char extTempStr[] = "exttemp";
char motdStr[] = "motd";
char holidayStr[] = "holiday";

#define NETWORK_DOWN_LIMIT 30 * 60 * 1000 // 30 mins for Network to come up otherwise reboot
bool networkUp = false;
bool rxInFail = false;

#define REQ_STATUS_MSG 1
#define STATUS_MSG 2
#define SET_TEMP_MSG 3
#define SET_EXT_MSG 4
#define ADJ_SETTIME_CONST_MSG 5
#define MOTD_MSG 6
#define GET_SCHEDULES_MSG 7
#define SCHEDULE_MSG 8
#define DELETE_ALL_SCHEDULES_MSG 9
#define DELETE_SCHEDULE_MSG 10
#define SET_DATE_TIME_MSG 11
#define SET_HOLIDAY_MSG 12
#define SET_THERM_TEMP_MSG 13
#define RESET_MSG 14

#define MAX_WIND_SIZE 12

struct Message
{
  uint8_t id;
  uint8_t len;
  uint16_t crc;
};

struct DateTimeStruct
{
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t dayOfWeek;
  uint8_t dayOfMonth;
  uint8_t month;
  uint8_t year;
};

struct Motd
{
  uint32_t expiry;
  char motdStr[MAX_MOTD_SIZE];
};

struct Temp
{
  int16_t temp;
};

struct SetExt
{
  int16_t temp;
  char windStr[MAX_WIND_SIZE];
};

struct HolidayDateStr
{
  uint8_t hour;
  uint8_t dayOfMonth;
  uint8_t month;
  uint8_t year;
};

struct HolidayStr
{
  HolidayDateStr startDate;
  HolidayDateStr endDate;
  int16_t temp;
  uint8_t valid; // Set to 1 if valid
};

union HolidayUnion
{
  HolidayStr elem;
  uint8_t raw[sizeof(HolidayStr)];
};

// Struct is long word padded...
struct SchedByElem
{
  uint16_t day;   //= "0" for every day, "0x0100" for Weekday (Mon - Fri),
                  //"0x0200" for Weekend (Sat, Sun), 1 - Sunday, 2 - Monday, 3 - Tuesday,....7 - Saturday
  uint16_t start; // Start minute (0 - 1440)
  uint16_t end;   // End time minute (0 - 1440)
  int16_t temp;   // Set temperature tenths of C, 180 = 18.0C
};

union SchedUnion
{
  struct SchedByElem elem;
  uint8_t raw[sizeof(SchedByElem)];
};
