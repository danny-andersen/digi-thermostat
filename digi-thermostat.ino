#include <Wire.h>
#include <uRTCLib.h>
  
#include <DallasTemperature.h>
#include <OneWire.h>

//#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

//Debounce switches
#include <Bounce2.h>

//Digital I/O Pins in use
#define THERM_ONE_WIRE_BUS 2
#define UP_BUTTON 3
#define DOWN_BUTTON 4
#define BOOST_BUTTON 5
#define RELAY 6
#define GREEN_LED 7
#define RED_LED 8

#define ON  1
#define OFF  0
//#define SERIAL_DEBUG
#ifdef SERIAL_DEBUG
#define LOOP_DELAY 5000
#else
#define LOOP_DELAY 50
#endif
#define DEBOUNCE_TIME 50
#define BUTTON_HOLD_TIME 400 //Time button is held down to increment or decrement
#define BOOST_TIME 30*60000 //Length of time to turn on heat regardless of set temperature
#define RTC_READ_INTERVAL 500UL
#define TEMPERATURE_READ_INTERVAL 15000UL

//Schedule and Temp settings
#define MAX_SCHEDULES 50
#define HYSTERSIS 0.5  //Degrees over the set temp to drive the current temp to stop output jitter
#define SET_INTERVAL 0.1 //Amount to increase the temp by per button press

#define LCD_ROWS 4
#define LCD_COLS 20

#define EEPROM_I2C_ADDR 0x50

//Each schedule is: DDSSEETT
//byte D = "0" for every day, "0x0100" for Weekday (Mon - Fri), "0x0200" for Weekend (Sat, Sun), 1 - Monday, 2 - Tuesday,....7 - Sunday
//uint16_t SS = Start minute (0 - 1440)
//uint16_t EE = End time minute (0 - 1440)
//uint16_t  TT = Set temperature tenths of C, 180 = 18.0C

//Thermometer variables
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(THERM_ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

//RTC
uRTCLib rtc;

//LCD
LiquidCrystal_I2C lcd(0x27,20,4);

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

//Global and stuff to initate once
float currentTemp = -1;
float lastScheduledTemp = 0;
float currentSetTemp = 0;
float degPerHour = 5.0;
float extAdj = 0.2;

boolean heat_on = FALSE;
byte noOfSchedules = 0;
unsigned long lastRTCRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastInStationUpdate = 0;
unsigned long boilerOnTime = 0;
unsigned long lastBoostAdjTime = 0;
char* dayNames[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

Bounce upButton = Bounce();
unsigned long upButtonDownTime = 0;
Bounce downButton = Bounce();
unsigned long downButtonDownTime = 0;
Bounce boostButton = Bounce();
unsigned long boostTimer = 0;

byte schedules[MAX_SCHEDULES][sizeof(SchedByElem)];

float extTemp = 100.0; //This will be set by remote command

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  while (!Serial); 
#endif      
  //Digi outs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  //Digi ins
  pinMode(UP_BUTTON, INPUT_PULLUP);
  upButton.attach(UP_BUTTON);
  upButton.interval(DEBOUNCE_TIME);
  pinMode(DOWN_BUTTON, INPUT_PULLUP);
  downButton.attach(DOWN_BUTTON);
  downButton.interval(DEBOUNCE_TIME);
  pinMode(BOOST_BUTTON, INPUT_PULLUP);
  boostButton.attach(BOOST_BUTTON);
  boostButton.interval(DEBOUNCE_TIME);
  
  //turn on power indicator - set orange until initialised
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  temp_sensor.begin();
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  currentTemp = temp_sensor.getTempCByIndex(0);

  Wire.begin();

//  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
//  rtc.set(0,19, 19, 7, 29, 1, 17);
//  eepromWrite(0,0x00);
  
  // set up the LCD's number of columns and rows:
  lcd.init();
  lcd.backlight();
  
  //Read schedule from EEPROM
  //Note: Max position: 32767
  //First byte is number of schedules
  noOfSchedules = eepromRead(0);
#ifdef SERIAL_DEBUG
  Serial.println("No of scheds: " + String(noOfSchedules, HEX));
#endif
  if (noOfSchedules > MAX_SCHEDULES) {
    noOfSchedules = MAX_SCHEDULES;
  }
  int cnt = 1;
  for (int i=0; i<noOfSchedules; i++) {
    for (int j=0; j<sizeof(SchedByElem); j++) {
      schedules[i][j] = eepromRead(cnt);
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
    noOfSchedules = 1;
  }
  //Get the time and current set temp
  rtc.refresh();
  lastScheduledTemp = getSetPoint();
  currentSetTemp = lastScheduledTemp;
  //Done set up
#ifdef SERIAL_DEBUG
  Serial.println("go");
#endif
  //Turn power led RED to indicate running
  digitalWrite(GREEN_LED, LOW);
  lcd.backlight();
}


void loop() {
  unsigned long currentMillis = millis();
  boolean changedState = false;
  boolean bigChangeOfState = false;

  if (currentMillis - lastRTCRead > RTC_READ_INTERVAL) {
    //Read RTC
    rtc.refresh();
    lastRTCRead = currentMillis;
    changedState = true;
  }
#ifdef SERIAL_DEBUG
  Serial.print(getDateStr());
  Serial.print(" ");
  Serial.print(getTimeStr());
  Serial.print("\n");
#endif

  if (boostTimer > 0) {
    unsigned long adj = currentMillis - lastBoostAdjTime;
    if (boostTimer > adj) {
      boostTimer -= adj;
      lastBoostAdjTime = currentMillis;
    } else {
      boostTimer = 0; 
    }
  }

  //Check PIR sensor -> if someone near turn on blacklight for 30 seconds
  //lcd.noBacklight();
//    lcd.backlight();


  //Read thermometer
  if (currentMillis - lastTempRead > TEMPERATURE_READ_INTERVAL) {
    temp_sensor.requestTemperatures(); // Send the command to get temperatures
    currentTemp = temp_sensor.getTempCByIndex(0);
    lastTempRead = currentMillis;
    changedState = true;
  }
#ifdef SERIAL_DEBUG
  Serial.println("Temperature for Device 1 is: " + String(currentTemp, 1));
#endif
    
  //Retreive current set point in schedule
  float schedTemp = getSetPoint();
#ifdef SERIAL_DEBUG
  Serial.println("Scheduled set point: " + String(currentTemp, 1));
#endif
  if (lastScheduledTemp != schedTemp) {
    currentSetTemp = schedTemp;
    lastScheduledTemp = schedTemp;
  }
  //Read button inputs
  if (upButton.update() && upButton.fell()) {
    //increase the set temp
    currentSetTemp += SET_INTERVAL;
    changedState = true;
  }
  if (upButton.read() == LOW && currentMillis - upButtonDownTime > BUTTON_HOLD_TIME) {
    //increase the set temp
    currentSetTemp += SET_INTERVAL;
    upButtonDownTime = currentMillis;
    changedState = true;
  }
  if (downButton.update() && downButton.fell()) {
    //decrease the set temp
    currentSetTemp -= SET_INTERVAL;
    changedState = true;
  }
  if (downButton.read() == LOW && currentMillis - downButtonDownTime > BUTTON_HOLD_TIME) {
    //decrease the set temp
    currentSetTemp -= SET_INTERVAL;
    downButtonDownTime = currentMillis;
    changedState = true;
  }
  if (boostButton.update() && boostButton.fell()) {
    if (boostTimer == 0) {
      //turn heating on for a bit, regardless of set temp
      switchHeat(true);
      boostTimer = BOOST_TIME;
      lastBoostAdjTime = currentMillis;
      changedState = true;
    } else {
      //boost already running - treat as cancel boost
      boostTimer = 0;
      changedState = true;
    }
    bigChangeOfState = true;
  }

  if (boostTimer == 0) {
    //Turn heating on or off depending on temp
    if ((currentSetTemp > currentTemp) && !heat_on) {
      switchHeat(true);
      changedState = true;
      bigChangeOfState = true;
    }
    if (heat_on && (currentTemp > (currentSetTemp + HYSTERSIS))) {
      //Reached set temperature, turn heating off
      //Note: Add in hystersis to stop flip flopping
      switchHeat(false);
      changedState = true;
      bigChangeOfState = true;
    }
  }

  //Check for any commands from in-station
  
  //Report back current status to in-station via RF transmitter regularly

  //Date + Time (19chars)  OR Time to reach set temp
  //Current temp, set temp, external temp

  if (changedState) {
    //Display current status
    if (bigChangeOfState) {
      lcd.clear();
    }
    lcd.setCursor(0, 0);
    String dateTimeStr = getTimeStr();
    String currTempStr = String(currentTemp, 1);
    lcd.print(dateTimeStr + "   " + currTempStr + "C");
    lcd.setCursor(0, 1);
    int runTime = calcRunTime(currentTemp, currentSetTemp + HYSTERSIS, extTemp);
    String runTimeStr = "Run: " + (boostTimer > 0 ? getMinSec(boostTimer) : getMinSec(runTime));
    lcd.print(runTimeStr + " Set:" + String(currentSetTemp, 1) + "C ");
    lcd.setCursor(0, 2);
    String boilerStatStr = heat_on ? "ON    " : "OFF   ";
    lcd.print("Heat:" + boilerStatStr + "Ext:" + (extTemp == 100.0 ? "??.?" : String(extTemp, 1)) + "C");
    lcd.setCursor(0, 3);
    lcd.print("This is the MOTD");
  }


#ifdef SERIAL_DEBUG
  Serial.println("End loop");
#endif
  delay(LOOP_DELAY); //temporary delay in loop

}

void eepromWrite(unsigned int eeaddress, byte data ) {
  int rdata = data;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
}

byte eepromRead(unsigned int eeaddress ) {
  byte rdata = 0xFF;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDR,1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
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
    memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
//    Serial.println("Sched: " + sched.elem.day + String(" ") + sched.elem.temp);

    if (sched.elem.start <= mins && sched.elem.end > mins) {
      if (sched.elem.day == 0 && priority <= 1) {
        //All days match and not found higher
          priority = 1;
          temp = sched.elem.temp; 
          Serial.println("Set 0: " + temp);
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
  return temp / 10.0;
}

//String getDateStr() {
//    return String(rtc.year()) + "/" + String(rtc.month()) + "/" + String(rtc.day(), DEC);
//}

//Calculate the number of ms to reach set temp
int calcRunTime(float tempNow, float tempSet, float tempExt) {
  int noMs = 0;
  if (tempNow < tempSe  t) {
    float deg = degPerHour;
    if (tempExt != 100.0) {
      //Reduce based on outside temp
      deg -= (tempNow - tempExt) * extAdj;
    }
    float noHours = (tempSet - tempNow) / deg;
    noMs = (int)(noHours * 3600 * 1000);
  }
  return noMs;
}

String getMinSec(unsigned long timeMs) {
  unsigned long tsecs = timeMs/1000;
  unsigned long mins = tsecs/60;
  unsigned long secs = tsecs % 60;
  String timeStr = "";
  if (mins < 10) {
    timeStr += "0";
  }
  timeStr += String(mins, DEC) + ":";
  if (secs < 10) {
    timeStr += "0";
  }
  timeStr += String(secs, DEC);
  return timeStr;
}

String getTimeStr() {
  String time = "";
  time += dayNames[rtc.dayOfWeek() - 1];
  time += " ";
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

