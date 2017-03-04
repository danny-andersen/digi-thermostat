#include <Wire.h>
#include <uRTCLib.h>
  
#include <DallasTemperature.h>
#include <OneWire.h>

//#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

//Debounce switches
#include <Bounce2.h>

//Radio network includes
#include <SPI.h>
#include <RF24.h>
#include <RF24Network.h>
#include <printf.h>

#include "digi-thermostat.h"

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

//Global variables and stuff to initate once
int16_t currentTemp = 1000;
int16_t lastScheduledTemp = 0;
volatile int16_t currentSetTemp = 0;
int16_t degPerHour = 50; //Default heating power - 5C per hour increase
int16_t extAdjustment = 50; //Weighting of difference between external and internal temp, e.g. if 10 deg diff (100/50) = -2deg

boolean heatOn = FALSE;
byte noOfSchedules = 0;
boolean defaultSchedule = false;

unsigned long currentMillis = 0;
unsigned long lastRTCRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastInStationUpdate = 0;
unsigned long lastGetSched = 0;
unsigned long boilerOnTime = 0;
unsigned long lastLoopTime = 0;
unsigned long lastScrollTime = 0;
unsigned long loopDelta = 0;
unsigned long lastRunTime = 0;
unsigned long backLightTimer = BACKLIGHT_TIME;
unsigned long motdExpiryTimer = 0;

char* dayNames[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char* monNames[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

//Rotary encoder
byte rotaryA = 0;
byte rotaryB = 0;
unsigned long lastTriggerTimeA = 0;
unsigned long lastTriggerTimeB = 0;

//Radio stuff
RF24 radio(RADIO_CE,RADIO_CS);
RF24Network network(radio);

//Bounce upButton = Bounce();
//unsigned long upButtonDownTime = 0;
//Bounce downButton = Bounce();
//unsigned long downButtonDownTime = 0;
Bounce boostButton = Bounce();
unsigned long boostTimer = 0;

byte schedules[MAX_SCHEDULES][sizeof(SchedByElem)];

int16_t extTemp = 1000; //This will be set by remote command
char motd[MAX_MOTD_SIZE]; //Will be set by remote command
char defaultMotd[] = "V:"__DATE__" S:"; //Show build date and number of schedules
char motdScrolled[LCD_COLS+1]; //Buffer used to scroll motd on LCD
int8_t scrollPos = 0;

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  printf_begin();
  while (!Serial); 
#endif      
  Serial.begin(115200);
  printf_begin();
  while (!Serial); 
  //Digi outs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  //Digi ins
//  pinMode(UP_BUTTON, INPUT_PULLUP);
//  upButton.attach(UP_BUTTON);
//  upButton.interval(DEBOUNCE_TIME);
//  pinMode(DOWN_BUTTON, INPUT_PULLUP);
//  downButton.attach(DOWN_BUTTON);
//  downButton.interval(DEBOUNCE_TIME);
  pinMode(BOOST_BUTTON, INPUT_PULLUP);
  digitalWrite(BOOST_BUTTON, HIGH);
  boostButton.attach(BOOST_BUTTON);
  boostButton.interval(DEBOUNCE_TIME);

  //Rotary encoder
  pinMode(ROTARY_A, INPUT); 
  pinMode(ROTARY_B, INPUT);
  digitalWrite(ROTARY_A, HIGH);
  digitalWrite(ROTARY_B, HIGH);
  rotaryA=digitalRead(ROTARY_A);
  rotaryB=digitalRead(ROTARY_B);
  attachInterrupt(digitalPinToInterrupt(ROTARY_A), intHandlerRotaryA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_B), intHandlerRotaryB, CHANGE);
  
  //turn on power indicator - set orange until initialised
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  temp_sensor.begin();
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  currentTemp = (int16_t)(temp_sensor.getTempCByIndex(0) * 10);

  Wire.begin();

//  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
//  rtc.set(0,45, 15, 5, 3, 2, 17);
//  eepromWrite(0,0x00);
  
  // set up the LCD's number of columns and rows:
  lcd.init();
  lcd.backlight();
  
  // Connect to the network
//  Serial.println(F("Connecting to the network..."));
  SPI.begin();
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(RADIO_CHANNEL);
  delay(5);
  network.begin(RADIO_CHANNEL, DIGI_THERM_NODE_ID);
//  radio.printDetails();

  //Read schedule from EEPROM
  //Note: Max position: 32767
  //First byte is number of schedules
  noOfSchedules = eepromRead(0);
//  Serial.println("No of scheds: " + String(noOfSchedules, HEX));
  if (noOfSchedules > MAX_SCHEDULES) {
    //Assume eprom corrupted
    eepromWrite(0,0x00);
    noOfSchedules = 0;
  }
  int cnt = 1;
  for (int i=0; i<noOfSchedules; i++) {
    for (int j=0; j<sizeof(SchedByElem); j++) {
      schedules[i][j] = eepromRead(cnt);
      cnt++;
    }
  }
  if (noOfSchedules == 0) {
    addDefaultSchedule();
  }

  setDefaultMotd();
  
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
}


void loop() {
  uint8_t changedState = 0;
  lastLoopTime = currentMillis;
  currentMillis = millis();
  loopDelta = currentMillis - lastLoopTime;

  if (currentMillis - lastRTCRead > RTC_READ_INTERVAL) {
    //Read RTC
    rtc.refresh();
    lastRTCRead = currentMillis;
    changedState = 1;
  }
#ifdef SERIAL_DEBUG
  Serial.print(getDateStr());
  Serial.print(" ");
  Serial.print(getTimeStr());
  Serial.print("\n");
#endif

  if (boostTimer > 0) {
    if (boostTimer > loopDelta) {
      boostTimer -= loopDelta;
    } else {
      boostTimer = 0; 
    }
  }

  //Read thermometer
  if (currentMillis - lastTempRead > TEMPERATURE_READ_INTERVAL) {
    temp_sensor.requestTemperatures(); // Send the command to get temperatures
    currentTemp = (int16_t)(temp_sensor.getTempCByIndex(0) * 10);
    lastTempRead = currentMillis;
    changedState = 1;
  }
#ifdef SERIAL_DEBUG
  Serial.println("Temperature for Device 1 is: " + String(currentTemp, 1));
#endif
 
  //Retreive current set point in schedule
  if (currentMillis - lastGetSched > SCHED_CHECK_INTERVAL) {
    float schedTemp = getSetPoint();
#ifdef SERIAL_DEBUG
    Serial.println("Scheduled set point: " + String(currentTemp, 1));
#endif
  
    if (lastScheduledTemp != schedTemp) {
      //Only override set temp if there is a schedule change (cos it may have been manually set)
      currentSetTemp = schedTemp;
      lastScheduledTemp = schedTemp;
      changedState = 1;
    }
  }

  //Read switch inputs
  changedState = readInputs(changedState);

  //Check for any commands from in-station
  changedState = checkMasterMessages(changedState);

  //Report back current status to in-station via RF transmitter regularly

  //If not in boost mode, turn heating on or off depending on temp
  if (boostTimer == 0) {
    if ((currentSetTemp > currentTemp) && !heatOn) {
      switchHeat(true);
      changedState = 2;
    }
    if (heatOn && (currentTemp > (currentSetTemp + HYSTERSIS))) {
      //Reached set temperature, turn heating off
      //Note: Add in hystersis to stop flip flopping
      switchHeat(false);
      changedState = 2;
     }
  }

  //Check if motd has expired
  if (motdExpiryTimer > 0) {
    if (motdExpiryTimer > loopDelta) {
      motdExpiryTimer -= loopDelta;
    } else {
      motdExpiryTimer = 0;
      setDefaultMotd();
    }
  }

  if (checkBackLight()) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
  }

  if (changedState > 0) {
    displayState(changedState);
  }

  if (strlen(motd) > LCD_COLS && currentMillis - lastScrollTime > SCROLL_PAUSE) {
    //Need to scroll the 4th line of the display 
    lastScrollTime = currentMillis;
    int charsToCopy = LCD_COLS;
    int blankChars = 0;
    int motdLen = strlen(motd);
    if (scrollPos + LCD_COLS > motdLen - 1) {
      charsToCopy = motdLen - scrollPos;
      if (charsToCopy == 0) {
        //Right at the end
        charsToCopy = LCD_COLS;
        scrollPos = 0;
      } else {
        blankChars = LCD_COLS - charsToCopy;
      }
    }
    strncpy(&motdScrolled[0], &motd[scrollPos], charsToCopy);
    if (blankChars > 0) {
      //Add blanks to the end of the message for a full screen
       memset(&motdScrolled[charsToCopy],' ', blankChars);
       scrollPos = 0;
    } else {
      //advance to the next screen
      int newPos = scrollPos;
      int lastPos;
      int lastSpace = strrchr(&motd[0], ' ');
      //Find last word to use as first in next msg
      while (newPos < lastSpace && newPos <= scrollPos + LCD_COLS && newPos < motdLen) {
        lastPos = newPos;
        do {
          newPos++;
        } while (motd[newPos] != ' ' && newPos < lastSpace && newPos < motdLen);
      }
      if (newPos != scrollPos + LCD_COLS) {
        newPos = lastPos + 1;
      }
      if (newPos > motdLen) newPos = 0;
      scrollPos = newPos;
    }
    motdScrolled[LCD_COLS] = '\0';
    lcd.setCursor(0, 3);
    lcd.print(motdScrolled);
//    Serial.println(motdScrolled);
  }
  
#ifdef SERIAL_DEBUG
  Serial.println("End loop");
#endif
  delay(LOOP_DELAY); //temporary delay in loop

}

// Interrupt on A changing state
void intHandlerRotaryA() {
  unsigned long t = micros();
  if (t - lastTriggerTimeA > DEBOUNCE_TIME) {
    rotaryA=digitalRead(ROTARY_A);
    //Only inc state on rising edge of A and B is off (CW rotation)
    if (rotaryA && !rotaryB) 
        currentSetTemp = currentSetTemp + SET_INTERVAL;
  }
  lastTriggerTimeA = t;
} 

// Interrupt on B changing state
void intHandlerRotaryB() {
  unsigned long t = micros();
  if (t - lastTriggerTimeB > DEBOUNCE_TIME) {
    rotaryB=digitalRead(ROTARY_B);
    //Only inc state on rising edge of B and A is off (CCW rotation)
    if (rotaryB && !rotaryA)
      currentSetTemp = currentSetTemp - SET_INTERVAL; 
  }
  lastTriggerTimeB = t;
}

uint8_t readInputs(uint8_t changedState) {
  if (boostButton.update() && boostButton.fell()) {
    if (boostTimer == 0) {
      //turn heating on for a bit, regardless of set temp
      switchHeat(true);
      boostTimer = BOOST_TIME;
    } else {
      //boost already running - treat as cancel boost
      boostTimer = 0;
    }
    changedState = 2;
  }
  return changedState;
}

void addDefaultSchedule() {
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
  defaultSchedule = true;
}

uint8_t checkMasterMessages(uint8_t changedState) {
  network.update();
  while (network.available()) {
    RF24NetworkHeader header;
    RF24NetworkHeader respHeader(MASTER_NODE_ID);
    Content payload;
    Content response;
    boolean sendStatus = false;
    union SchedUnion sched;
    network.read(header, &payload, sizeof(Content)); //Read message
//    Serial.print("Header: ");
//    Serial.println(header.toString());
    flickerLED();
    switch((byte)header.type) {
      case (REQ_STATUS_MSG):
          sendStatus = true;
        break;
      case (STATUS_MSG): 
          sendStatus = true;
        break;
      case (SET_TEMP_MSG):
          currentSetTemp = payload.setTemp.setTemp;
          changedState = 2;
          sendStatus = true;
        break;
      case (SET_EXT_MSG):
          extTemp = payload.setExt.setExt;
          changedState = 2;
          sendStatus = true;
        break;
      case (ADJ_SETTIME_CONST_MSG): 
          degPerHour = payload.adjSetTimeConstants.degPerHour;
          extAdjustment = payload.adjSetTimeConstants.extAdjustment;
          changedState = 1;
          sendStatus = true;
        break;
      case (MOTD_MSG):
          motdExpiryTimer = payload.motd.expiry;
          motd[0] = '\0';
          strncat(&motd[0], &payload.motd.motdStr[0], MAX_MOTD_SIZE);
          changedState = 2;
        break;
      case (GET_SCHEDULES_MSG):
          //Send each schedule in the list
          respHeader.type = SCHEDULE_MSG;
          for (int i=0; i<noOfSchedules; i++) {
            memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
            response.schedule.day = sched.elem.day;
            response.schedule.start = sched.elem.start;
            response.schedule.end = sched.elem.end;
            response.schedule.temp = sched.elem.temp;
            //Send to master
            if (!network.write(respHeader, &response, sizeof(Content))) {
              Serial.println("Send of sched failed");
            }
            delay(100);
          }
        break;
      case (SCHEDULE_MSG):
          //Insert schedule
          struct SchedByElem elem;
          elem.day = payload.schedule.day;
          elem.start = payload.schedule.start;
          elem.end = payload.schedule.end;
          elem.temp = payload.schedule.temp;
          sched.elem = elem;
          if (defaultSchedule) {
            //Overwrite the default
            noOfSchedules = 0;
            defaultSchedule = false;
          }
          memcpy(&schedules[noOfSchedules], &sched.raw, sizeof(SchedByElem));
          writeSchedule(noOfSchedules, &sched.raw[0]);
          noOfSchedules++;
          //Write number of schedules at start address
          eepromWrite(0, noOfSchedules);
        break;
      case (DELETE_ALL_SCHEDULES_MSG):
          noOfSchedules = 0;
          eepromWrite(0, noOfSchedules);
          addDefaultSchedule();
        break;
      case (DELETE_SCHEDULE_MSG):
          int schedToDelete;
          schedToDelete = noOfSchedules+1;
          for (int i=0; i<noOfSchedules; i++) {
            memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
            //Find matching schedule
            if (sched.elem.day == payload.schedule.day 
               && sched.elem.start == payload.schedule.start
               && sched.elem.end == payload.schedule.end
               && sched.elem.temp == payload.schedule.temp) {
                schedToDelete = i;
            }
          }
          //Delete matching schedule by shuffling them up
          for (int i=schedToDelete; i<(noOfSchedules-1); i++) {
            memcpy(&schedules[i], &schedules[i++], sizeof(SchedByElem));
          }
        break;
      case (SET_DATE_TIME_MSG):
        //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
        rtc.set(payload.dateTime.sec,
                payload.dateTime.min, 
                payload.dateTime.hour, 
                payload.dateTime.dayOfWeek, 
                payload.dateTime.dayOfMonth, 
                payload.dateTime.month, 
                payload.dateTime.year);
//        Serial.println("Rx Time: " + payload.dateTime.hour + 
//                    payload.dateTime.min + payload.dateTime.sec + payload.dateTime.year);
        rtc.refresh();
//        Serial.println("Clk Time: " + rtc.hour() + 
//                    rtc.minute() + rtc.second());
        break;

    }
    flickerLED();
    if (sendStatus) {
      response.status.currentTemp = currentTemp;
      response.status.setTemp = currentSetTemp;
      response.status.heatOn = (byte)heatOn;
      response.status.minsToSet = (uint16_t) (getRunTime() / 60000);
//      Serial.println("Sending status msg");
      respHeader.type = STATUS_MSG;
      if (!network.write(respHeader, &response, sizeof(Content))) {
//        Serial.println("Send failed");
      }
      flickerLED();
    }
  }
  return changedState;
}

void displayState(uint8_t changedState) {
    //Display current status
    String runTimeStr;
    unsigned long runTime = getRunTime();
    if (runTime != 0) {
      unsigned long threeDigit = 6000000;
      runTimeStr = "Run:";
      if (runTime < threeDigit) {
        runTimeStr += " ";
      }
      runTimeStr += getMinSec(runTime);
      if ((lastRunTime < threeDigit && runTime >= threeDigit) || (lastRunTime >= threeDigit && runTime < threeDigit)) {
        changedState = 2;
      }
  //    Serial.println("Runtime:" + String(runTime) + " 3digi: " + String(threeDigit) + " runTimeStr: " + runTimeStr + " big: " + String(bigChangeOfState));
      lastRunTime = runTime;
    } else {
      //Show date
      runTimeStr = getDateStr();
    }
    if (changedState > 1) {
      lcd.clear();
    }
    lcd.setCursor(0, 0);
    String dateTimeStr = getTimeStr();
    String currTempStr;
    currTempStr = String((float)(currentTemp / 10.0), 1) + "C";
    if (currentTemp <= 100 && currentTemp > 0) {
       currTempStr = "0" + currTempStr;
    } else if (currentTemp == 0) {
       currTempStr = " 0.0C";
    }
    lcd.print(dateTimeStr + "   " + currTempStr);
    lcd.setCursor(0, 1);
    String setTempStr;
    setTempStr = String((float)(currentSetTemp / 10.0), 1) + "C";
    if (currentSetTemp <= 100 && currentSetTemp > 0) {
       setTempStr = "0" + setTempStr;
    } else if (currentSetTemp == 0) {
       setTempStr = " 0.0C";
    }
    lcd.print(runTimeStr + " Set:" + setTempStr);
    lcd.setCursor(0, 2);
    String boilerStatStr = heatOn ? "ON    " : "OFF   ";
    String extTempStr = "??.?C";
    if (extTemp != 1000) {
       extTempStr = String((float)(extTemp / 10.0), 1) + "C";
       if (extTemp < 100 && extTemp != 0) {
          extTempStr = "0" + extTempStr;
       } else if (extTemp == 0) {
          extTempStr = " 0.0C";
       }
    }
    lcd.print("Heat:" + boilerStatStr + "Ext:" + extTempStr);
    if (strlen(motd) <= LCD_COLS) {
      lcd.setCursor(0, 3);
      lcd.print(motd); //Only print here if motd fits, otherwise needs to scroll
    }
}

boolean checkBackLight() {
  if (backLightTimer > 0) {
    if (backLightTimer > loopDelta) {
      backLightTimer -= loopDelta;
    } else {
      backLightTimer = 0;
    }
  }
  //PIR set to repeat trigger so if output is high then set backLightTimer to BACKLIGHT_TIME
  //This keeps the backlight on LCD until people have left the area
  int pirValue = analogRead(PIR_PIN);
  if (pirValue >= ANALOGUE_HIGH) {
    backLightTimer = BACKLIGHT_TIME;
  }
  return (backLightTimer > 0);
}

unsigned long getRunTime() {
  unsigned long runTime = 0;
  if (boostTimer > 0) {
    runTime = boostTimer;
  } else if (heatOn) {
    runTime = calcRunTime(currentTemp, currentSetTemp + HYSTERSIS, extTemp);
  }
  return runTime;
}

void writeSchedule(int schedNum, byte * sched) {
  int cnt;
  cnt = 1 + (schedNum * sizeof(SchedByElem));
  for (int j=0; j<sizeof(SchedByElem); j++) {
      eepromWrite(cnt, *sched++);
      cnt++;
  }
}

void eepromWrite(unsigned int eeaddress, byte data ) {
  int rdata = data;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  delay(5);
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

  
int16_t getSetPoint() {
  //Get mins in day
  uint16_t mins = rtc.hour() * 60 + rtc.minute();
  //Find matching schedules and take one with the most specific day that matches
  union SchedUnion sched;
  int priority = 0;
  int temp = 90;
  int currDay = rtc.dayOfWeek();
  for (int i=0; i<noOfSchedules; i++) {
    memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
//    Serial.println("Sched: " + sched.elem.day + String(" ") + sched.elem.temp);

    if (sched.elem.start <= mins && sched.elem.end > mins) {
      if (sched.elem.day == 0 && priority <= 1) {
        //All days match and not found higher
          priority = 1;
          temp = sched.elem.temp; 
//          Serial.println("Set 0: " + temp);
      }
      if (sched.elem.day == 0x200 && (currDay == 7 || currDay == 1) && priority <= 2) {
        //Its the weekend and not found higher
          priority = 2;
          temp = sched.elem.temp; 
      }
      if (sched.elem.day == 0x100 && (currDay >= 2 || currDay <= 6) && priority <= 2) {
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
  return temp;
}

//Calculate the number of ms to reach set temp
unsigned long calcRunTime(int16_t tempNow, int16_t tempSet, int16_t tempExt) {
  unsigned long noMs = 0;
  if (tempNow < tempSet) {
    int16_t deg = degPerHour;
    if (tempExt != 1000 && tempExt < tempNow) {
      //Reduce based on outside temp
      int16_t adj = (tempNow - tempExt) / extAdjustment;
      if (adj > 3) adj = 3;
      deg -= adj;
    }
    float noSecs = (tempSet - tempNow) * 3600.0 / deg;
    noMs = (unsigned long)(noSecs * 1000);
//    Serial.println("Set: " + String(tempSet,1) + " Now: " + String(tempNow, 1) + " Degph: " + deg + " noMs: " + noMs);
//    Serial.println(getMinSec(noMs));
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

const char * getDateStr() {
    char dateStr[11];
    sprintf(dateStr,"%02d %s %02d ",rtc.day(),monNames[rtc.month()-1],rtc.year());
    return dateStr;
}


void switchHeat(boolean on) {
  if (on) {
   digitalWrite(RED_LED, LOW);
   digitalWrite(GREEN_LED, HIGH);
   digitalWrite(RELAY, LOW);
   heatOn = true;
  } else {
   digitalWrite(RED_LED, HIGH);
   digitalWrite(GREEN_LED, LOW);
   digitalWrite(RELAY, HIGH);
   heatOn = false;
  }
}

void flickerLED() {
  int ledToFlick;
  if (heatOn) {
    ledToFlick = RED_LED;
  } else {
    ledToFlick = GREEN_LED;
  }
  digitalWrite(ledToFlick, HIGH);
  delay(50);
  digitalWrite(ledToFlick, LOW);
}

void setDefaultMotd() {
  //Set up default motd string;
  motd[0] = '\0';
  strncat(motd, &defaultMotd[0], MAX_MOTD_SIZE);
  sprintf(&motd[strlen(defaultMotd)],"%d", noOfSchedules);
}

