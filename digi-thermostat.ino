
#include <Wire.h>
#include <RTClib.h>
//Include the this lib to write to eeprom on RTC board
#include <uRTCLib.h>
  
#include <DallasTemperature.h>
#include <OneWire.h>

#include <LiquidCrystal.h>

//I/O Pins in use
#define THERM_ONE_WIRE_BUS 2
#define LCD_RS  12
#define LCD_E   11
#define LCD_D4  10
#define LCD_D5  9
#define LCD_D6  8
#define LCD_D7  7

#define BOILER_ON  "ON"
#define BOILER_OFF  "OFF"
//Each schedule is: DSSSSEEEETTT
//D = "0" for every day, "D" for Weekday (Mon - Fri), "E" for Weekend (Sat, Sun), 1 - Monday, 2 - Tuesday,....7 - Sunday
//SSSS = Start minute (0 - 1440)
//EEEE = End time minute (0 - 1440)
//TTT = Set temperature tenths of C, 180 = 18.0C
#define SCHEDULE_LEN 12
#define MAX_SCHEDULES 25

struct schedByElem {
    char day;
    char start[4];
    char end[4];
    char temp[3];
} ;

union SchedUnion {
  char raw[SCHEDULE_LEN];
  struct schedByElem elem;
};

//Thermometer variables
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(THERM_ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

//RTC
DS1307 rtc;
//EEPROM
uRTCLib eeprom;

//LCD
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

//Global and stuff to initate once
uint32_t delayMS;
float currentTemp = -1;
char schedules[MAX_SCHEDULES][SCHEDULE_LEN];

String getDateStr(const DateTime& dt) {
    return String(dt.year()) + "/" + String(dt.month()) + "/" + String(dt.day(), DEC);
}

String getTimeStr(const DateTime& dt) {
  String time = "";
  if (dt.hour() < 10) {
    time += "0" + String(dt.hour(),DEC);
  } else {
    time += dt.hour();
  }
  time += ":";
  if (dt.minute() < 10) {
    time += "0" + String(dt.minute(),DEC);
  } else {
    time += dt.minute();
  }
  time += ":";
  if (dt.second() < 10) {
    time += "0" + String(dt.second(),DEC);
  } else {
    time += dt.second();
  }
  return time;
}

void setup() {
  Serial.begin(9600);
  delayMS = 500; 
  temp_sensor.begin();

  Wire.begin();
  rtc.begin();
//  rtc.adjust(DateTime(2017, 1, 23, 7, 58, 0));
 
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

  //Read schedule from EEPROM
  //Note: Max position: 32767
  //First position is number of schedules
  int noOfSchedules = eeprom.eeprom_read(0);
  int cnt = 1;
  for (int i=0; i<noOfSchedules; i++) {
    for (int j=0; j<SCHEDULE_LEN; j++) {
      schedules[i][j] = eeprom.eeprom_read(cnt);
      cnt++;
    }
  }
  if (noOfSchedules == 0) {
    //Add a default schedule
    struct schedByElem elem = { '0', {'0','0','0','0'}, {'0','0','0','0'},{'1','3','0'}};
    union SchedUnion sched;
    sched.elem = elem;
    schedules[0] = sched.raw;
  }
}

void loop() {
  float setTemp = 18.5;
  String boilerStatStr = BOILER_OFF;
  
  //Read RTC
  DateTime now = rtc.now();
//  Serial.print(getDateStr(now));
//  Serial.print(" ");
//  Serial.print(getTimeStr(now));
//  Serial.print("\n");

  //Check PIR sensor -> if someone near turn on blacklight for 30 seconds

  //Read button inputs
  
  //Read thermometer every 15 seconds?
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  currentTemp = temp_sensor.getTempCByIndex(0);
  String currTempStr = String(currentTemp, 1);

//  Serial.print("Temperature for Device 1 is: " + currTempStr);
//  Serial.print("\n");
    
  //Check temperature against current set point in schedule
  
  //Turn boiler on or off
  if (setTemp > currentTemp) {
    boilerStatStr = BOILER_ON;
  }

  //Display current status
  //Date + Time (19chars)  OR Time to reach set temp
  //Current temp, set temp, external temp
  
  lcd.setCursor(0, 0);
  String dateTimeStr = getTimeStr(now);
  lcd.print(dateTimeStr + " " + currTempStr + "C");
  lcd.setCursor(0, 1);
  lcd.print("Set:" + String(setTemp, 1) + "C " + boilerStatStr);

  //Check for any commands from in-station
  
  //Report back current status to in-station via RF transmitter every minute
  
  delay(delayMS); //temporary delay in loop

}


