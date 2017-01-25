
#include <Wire.h>
//#include <RTClib.h>
//Include the this lib to write to eeprom on RTC board
#include <uRTCLib.h>
  
#include <DallasTemperature.h>
#include <OneWire.h>

#include <LiquidCrystal.h>

//Digital I/O Pins in use
#define THERM_ONE_WIRE_BUS 2
#define RELAY 3
#define GREEN_LED 4
#define RED_LED 5
#define UP_BUTTON 6
#define DOWN_BUTTON 13
#define BOOST_BUTTON 1
#define LCD_RS  12
#define LCD_E   11
#define LCD_D4  10
#define LCD_D5  9
#define LCD_D6  8
#define LCD_D7  7

#define ON  1
#define OFF  0
#define TEMPERATURE_INTERVAL 15000UL
#define RTC_READ_INTERVAL 1000UL

#define LCD_ROWS 2
#define LCD_COLS 20

//Each schedule is: DDSSEETT
//byte D = "0" for every day, "0x0100" for Weekday (Mon - Fri), "0x0200" for Weekend (Sat, Sun), 1 - Monday, 2 - Tuesday,....7 - Sunday
//uint16_t SS = Start minute (0 - 1440)
//uint16_t EE = End time minute (0 - 1440)
//uint16_t  TT = Set temperature tenths of C, 180 = 18.0C
#define MAX_SCHEDULES 50
#define DEBOUNCE_TIME 50

//Thermometer variables
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(THERM_ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

//RTC
//DS1307 rtc;
//EEPROM
uRTCLib rtc;

//LCD
LiquidCrystal lcd(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

//Struct is long word padded...
struct SchedByElem {
    uint16_t day;
    uint16_t start;
    uint16_t end;
    uint16_t temp;
};

union SchedUnion {
  struct SchedByElem elem;
  byte raw[sizeof(SchedByElem)];
};

struct Debounce {
  int buttonState = LOW;             // the current reading from the input pin
  int lastButtonState = LOW;   // the previous reading from the input pin
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
};

//Global and stuff to initate once
uint32_t delayMS = 500;
float currentTemp = -1;
boolean heat_on = FALSE;
int noOfSchedules = 0;
unsigned long lastRTCRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastInStationUpdate = 0;
unsigned long boilerOnTime = 0;
struct Debounce upButton;
struct Debounce downButton;
struct Debounce boostButton;

byte schedules[MAX_SCHEDULES][sizeof(SchedByElem)];

void setup() {
  Serial.begin(9600);
  //Digi outs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  //Digi ins
  pinMode(UP_BUTTON, INPUT);
  pinMode(DOWN_BUTTON, INPUT);
  pinMode(BOOST_BUTTON, INPUT);
  
  //power on indicator
  digitalWrite(RED_LED, HIGH);

  temp_sensor.begin();

  Wire.begin();
//  rtc.adjust(DateTime(2017, 1, 23, 7, 58, 0));
 
  // set up the LCD's number of columns and rows:
  lcd.begin(LCD_COLS, LCD_ROWS);

  //Read schedule from EEPROM
  //Note: Max position: 32767
  //First byte is number of schedules
  noOfSchedules = rtc.eeprom_read(0);
  int cnt = 1;
  for (int i=0; i<noOfSchedules; i++) {
    for (int j=0; j<sizeof(SchedByElem); j++) {
      schedules[i][j] = rtc.eeprom_read(cnt);
      cnt++;
    }
  }
  if (noOfSchedules == 0) {
    //Add a default schedule - in mem only
    struct SchedByElem elem;
    elem.day = 0;
    elem.start = 0;
    elem.end = 1440;
    elem.temp = 130;
    union SchedUnion sched;
    sched.elem = elem;
    memcpy(&schedules[0], &sched.raw, sizeof(SchedByElem));
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastRTCRead > RTC_READ_INTERVAL) {
    //Read RTC
    rtc.refresh();
    lastRTCRead = currentMillis;
  }
//  Serial.print(getDateStr(now));
//  Serial.print(" ");
//  Serial.print(getTimeStr(now));
//  Serial.print("\n");

  //Check PIR sensor -> if someone near turn on blacklight for 30 seconds

  //Read button inputs
  debounceSwitch(UP_BUTTON, upButton); 
  debounceSwitch(DOWN_BUTTON, downButton); 
  debounceSwitch(BOOST_BUTTON, downButton); 

  //Read thermometer
  if (currentMillis - lastTempRead > TEMPERATURE_INTERVAL) {
    temp_sensor.requestTemperatures(); // Send the command to get temperatures
    currentTemp = temp_sensor.getTempCByIndex(0);
    lastTempRead = currentMillis;
  }
//  Serial.print("Temperature for Device 1 is: " + currTempStr);
//  Serial.print("\n");
    
  //Retreive current set point in schedule
  float setTemp = getSetPoint();
  
  //Turn heating on or off depending on temp
  if (setTemp > currentTemp) {
    if (!heat_on) {
      switchHeat(true);
    }
  } else if (heat_on) {
    //Reached set temperature, turn heating off
    switchHeat(false);
  }

  //Display current status
  //Date + Time (19chars)  OR Time to reach set temp
  //Current temp, set temp, external temp
  
  lcd.setCursor(0, 0);
  String dateTimeStr = getTimeStr();
  String currTempStr = String(currentTemp, 1);
  lcd.print(dateTimeStr + " " + currTempStr + "C");
  lcd.setCursor(0, 1);
  String boilerStatStr = heat_on ? "ON" : "OFF";
  lcd.print("Set:" + String(setTemp, 1) + "C " + boilerStatStr);

  //Check for any commands from in-station
  
  //Report back current status to in-station via RF transmitter every minute
  
  delay(delayMS); //temporary delay in loop

}

float getSetPoint() {
  //Get mins in day
  uint16_t mins = rtc.hour() * 60 + rtc.minute();
  //Find matching schedules and take one with the most specific day that matches
  union SchedUnion sched;
  int priority = 0;
  int temp = 0;
  int currDay = rtc.dayOfWeek();
  for (int i=0; i<noOfSchedules; i++) {
    memcpy(&sched.raw, &schedules[0], sizeof(SchedByElem));
    
    if (sched.elem.start <= mins && sched.elem.end > mins) {
      if (sched.elem.day == 0 && priority <= 1) {
        //All days match and not found higher
          priority = 1;
          temp = sched.elem.temp; 
      }
      if (sched.elem.day == 0x200 && (currDay == 6 || currDay == 7) && priority <= 2) {
        //Its the weekend and not found higher
          priority = 2;
          temp = sched.elem.temp; 
      }
      if (sched.elem.day == 0x100 && (currDay >= 1 || currDay < 6) && priority <= 2) {
        //Its a weekday and not found higher
          priority = 2;
          temp = sched.elem.temp; 
      }
      if (sched.elem.day == currDay && priority <= 3) {
        //Found a specific day - cant get higher
          priority = 3;
          temp = sched.elem.temp; 
      }
    }
  }
}

String getDateStr() {
    return String(rtc.year()) + "/" + String(rtc.month()) + "/" + String(rtc.day(), DEC);
}

String getTimeStr() {
  String time = "";
  if (rtc.hour() < 10) {
    time += "0" + String(rtc.hour(),DEC);
  } else {
    time += rtc.hour();
  }
  time += ":";
  if (rtc.minute() < 10) {
    time += "0" + String(rtc.minute(),DEC);
  } else {
    time += rtc.minute();
  }
  time += ":";
  if (rtc.second() < 10) {
    time += "0" + String(rtc.second(),DEC);
  } else {
    time += rtc.second();
  }
  return time;
}

void switchHeat(boolean on) {
  if (on) {
   digitalWrite(RED_LED, LOW);
   digitalWrite(GREEN_LED, HIGH);
   digitalWrite(RELAY, HIGH);
   heat_on = true;
  } else {
   digitalWrite(RED_LED, HIGH);
   digitalWrite(GREEN_LED, LOW);
   digitalWrite(RELAY, LOW);
   heat_on = false;
  }
}

void debounceSwitch(int pin, struct Debounce *button) {
  // read the state of the switch into a local variable:
  int reading = digitalRead(pin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH),  and you've waited
  // long enough since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != button->lastButtonState) {
    // reset the debouncing timer
    button->lastDebounceTime = millis();
  }

  if ((millis() - button->lastDebounceTime) > DEBOUNCE_TIME) {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != button->buttonState) {
      button->buttonState = reading;
    }
  }
}

