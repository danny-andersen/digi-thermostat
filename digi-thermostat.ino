#include <Wire.h>
#include <RTClib.h>
  
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

//Thermometer variables
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(THERM_ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

//RTC
DS1307 rtc;

//LCD
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

//Global and stuff to initate once
uint32_t delayMS;
float currentTemp = -1;

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
  
  //Report back current status and actions to in-station via RF transmitter
  
  delay(delayMS); //temporary delay in loop

}


