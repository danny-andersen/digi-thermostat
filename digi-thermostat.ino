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
#include <RF24Mesh.h>

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
int16_t extAdjustment = 2; //Weighting of difference between external and internal temp

boolean heatOn = FALSE;
byte noOfSchedules = 0;

unsigned long currentMillis = 0;
unsigned long lastRTCRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastInStationUpdate = 0;
unsigned long boilerOnTime = 0;
unsigned long lastLoopTime = 0;
unsigned long lastRunTime = 0;
unsigned long backLightTimer = 0;
char* dayNames[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};

//Rotary encoder
byte rotaryA = 0;
byte rotaryB = 0;
unsigned long lastTriggerTimeA = 0;
unsigned long lastTriggerTimeB = 0;

//Radio stuff
RF24 radio(RADIO_CE,RADIO_CS);
RF24Network network(radio);
RF24Mesh mesh(radio,network);

//Bounce upButton = Bounce();
//unsigned long upButtonDownTime = 0;
//Bounce downButton = Bounce();
//unsigned long downButtonDownTime = 0;
Bounce boostButton = Bounce();
unsigned long boostTimer = 0;

byte schedules[MAX_SCHEDULES][sizeof(SchedByElem)];

int16_t extTemp = 1000; //This will be set by remote command
String motd =  "V:" + String(__DATE__ ); //Will be set by remote command

void setup() {
#ifdef SERIAL_DEBUG
  Serial.begin(9600);
  while (!Serial); 
#endif      
  Serial.begin(9600);
  while (!Serial); 
  //Digi outs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
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
  pinMode(ROTARY_A, INPUT_PULLUP); 
  pinMode(ROTARY_B, INPUT_PULLUP);
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
  
  //Set node id on radio mesh
  mesh.setNodeID(DIGI_THERM_NODE_ID);
  // Connect to the mesh
  Serial.println("Connecting to the mesh...");
  mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS, 5*1000);
  //Read schedule from EEPROM
  //Note: Max position: 32767
  //First byte is number of schedules
  noOfSchedules = eepromRead(0);
#ifdef SERIAL_DEBUG
  Serial.println("No of scheds: " + String(noOfSchedules, HEX));
#endif
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
  uint8_t changedState = 0;
  lastLoopTime = currentMillis;
  currentMillis = millis();
  unsigned long loopDelta = currentMillis - lastLoopTime;

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

  if (changedState > 0) {
    displayState(changedState);
  }

#ifdef SERIAL_DEBUG
  Serial.println("End loop");
#endif
  delay(LOOP_DELAY); //temporary delay in loop

}

// Interrupt on A changing state
void intHandlerRotaryA() {
  unsigned long t = millis();
  if (t - lastTriggerTimeB > DEBOUNCE_TIME && t - lastTriggerTimeA > DEBOUNCE_TIME) {
    rotaryA=digitalRead(ROTARY_A);
    if (rotaryB) {
      currentSetTemp = rotaryA ?  currentSetTemp + SET_INTERVAL : currentSetTemp - SET_INTERVAL;
    }
//    digitalWrite(GREEN_LED, rotaryA);
  }
  lastTriggerTimeA = t;
} 

// Interrupt on B changing state
void intHandlerRotaryB() {
  unsigned long t = millis();
  if (t - lastTriggerTimeB > DEBOUNCE_TIME && t - lastTriggerTimeA > DEBOUNCE_TIME) {
    rotaryB=digitalRead(ROTARY_B);
    if(!rotaryA) { 
      currentSetTemp = rotaryB ? currentSetTemp + SET_INTERVAL : currentSetTemp - SET_INTERVAL;
    }
//    digitalWrite(RELAY, rotaryB);
  }
  lastTriggerTimeB = t;
}

uint8_t readInputs(uint8_t changedState) {
//    if (upButton.update() && upButton.fell()) {
//    //increase the set temp
//    currentSetTemp += SET_INTERVAL;
//    upButtonDownTime = currentMillis;
//    changedState = 1;
//  }
//  if (upButton.read() == LOW && currentMillis - upButtonDownTime > BUTTON_HOLD_TIME) {
//    //increase the set temp
//    currentSetTemp += SET_INTERVAL * 2.0;
//    upButtonDownTime = currentMillis;
//    changedState = 1;
//  }
//  if (downButton.update() && downButton.fell()) {
//    //decrease the set temp
//    currentSetTemp -= SET_INTERVAL;
//    downButtonDownTime = currentMillis;
//    changedState = 1;
//  }
//  if (downButton.read() == LOW && currentMillis - downButtonDownTime > BUTTON_HOLD_TIME) {
//    //decrease the set temp
//    currentSetTemp -= SET_INTERVAL * 2.0;
//    downButtonDownTime = currentMillis;
//    changedState = 1;
//  }
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

uint8_t checkMasterMessages(uint8_t changedState) {
  mesh.update();
  while (network.available()) {
    RF24NetworkHeader header;
    RF24NetworkHeader respHeader;
    respHeader.to_node = MASTER_NODE_ID;
    respHeader.type = STATUS_MSG;
    Content payload;
    Content response;
    boolean sendStatus = false;
    union SchedUnion sched;
    network.read(header, &payload, sizeof(Content)); //Read message
    Serial.print("Received packet #");
    Serial.println(header.type);
    switch((byte)header.type) {
      case (REQ_STATUS_MSG):
          Serial.println("Request status message");
          sendStatus = true;
        break;
      case (STATUS_MSG): 
          Serial.println("Status message rx - shouldn't get this?");
          sendStatus = true;
        break;
      case (SET_TEMP_MSG):
          currentSetTemp = payload.setTemp.setTemp;
          changedState = 1;
          sendStatus = true;
        break;
      case (SET_EXT_MSG):
          extTemp = payload.setExt.setExt;
          changedState = 1;
          sendStatus = true;
        break;
      case (ADJ_SETTIME_CONST_MSG): 
          degPerHour = payload.adjSetTimeConstants.degPerHour;
          extAdjustment = payload.adjSetTimeConstants.extAdjustment;
          changedState = 1;
          sendStatus = true;
        break;
      case (MOTD_MSG):
          motd = payload.motd.motdStr; 
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
              reconnect();
            }
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
          memcpy(&schedules[noOfSchedules], &sched.raw, sizeof(SchedByElem));
          noOfSchedules++;
        break;
      case (DELETE_SCHEDULE_MSG):
          int schedToDelete = -1;
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
    }
    if (sendStatus) {
      response.status.currentTemp = currentTemp;
      response.status.setTemp = currentSetTemp;
      response.status.heatOn = (byte)heatOn;
      response.status.minsToSet = (uint16_t) (getRunTime() / 60000);
      if (!network.write(respHeader, &response, sizeof(Content))) {
        reconnect();
      }
    }
  }
  return changedState;
}

void reconnect() {
    if ( ! mesh.checkConnection() ) {
      //refresh the network address
      Serial.println("Renewing Address");
      mesh.renewAddress();
    } else {
      Serial.println("Send fail, but still connected");
    }
}

void displayState(uint8_t changedState) {
    if (checkBackLight()) {
      lcd.backlight();
    } else {
      lcd.noBacklight();
    }

    //Display current status
    unsigned long runTime = getRunTime();
    String runTimeStr = "Run:";
    unsigned long threeDigit = 6000000;
    if (runTime < threeDigit) {
      runTimeStr += " ";
    }
    runTimeStr += getMinSec(runTime);
    if ((lastRunTime < threeDigit && runTime >= threeDigit) || (lastRunTime >= threeDigit && runTime < threeDigit)) {
      changedState = 2;
    }
//    Serial.println("Runtime:" + String(runTime) + " 3digi: " + String(threeDigit) + " runTimeStr: " + runTimeStr + " big: " + String(bigChangeOfState));
    lastRunTime = runTime;
    if (changedState > 1) {
      lcd.clear();
    }
    lcd.setCursor(0, 0);
    String dateTimeStr = getTimeStr();
    String currTempStr = String((float)(currentTemp / 10.0), 1);
    lcd.print(dateTimeStr + "   " + currTempStr + "C");
    lcd.setCursor(0, 1);
    lcd.print(runTimeStr + " Set:" + String((float)(currentSetTemp / 10.0), 1) + "C");
    lcd.setCursor(0, 2);
    String boilerStatStr = heatOn ? "ON    " : "OFF   ";
    lcd.print("Heat:" + boilerStatStr + "Ext:" + (extTemp == 1000 ? "??.?" : String((float)(extTemp / 10.0), 1)) + "C");
    lcd.setCursor(0, 3);
    lcd.print(motd);
}

boolean checkBackLight() {
  if (backLightTimer > 0) {
    unsigned long loopDelta = currentMillis - lastLoopTime;
    if (backLightTimer > loopDelta) {
      backLightTimer -= loopDelta;
    } else {
      backLightTimer = 0;
    }
  }
  //Check PIR sensor -> if someone near turn on blacklight for 30 seconds
  backLightTimer = BACKLIGHT_TIME;
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

void eepromWrite(unsigned int eeaddress, byte data ) {
  int rdata = data;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((int)(eeaddress >> 8)); // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  Wire.endTransmission();
}

void writeSchedules() {
  int cnt = 1;
  for (int i=0; i<noOfSchedules; i++) {
    for (int j=0; j<sizeof(SchedByElem); j++) {
      eepromWrite(cnt, schedules[i][j]);
      cnt++;
    }
  }
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
//          Serial.println("Set 0: " + temp);
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
  return temp;
}

#ifdef SERIAL_DEBUG
String getDateStr() {
    return String(rtc.year()) + "/" + String(rtc.month()) + "/" + String(rtc.day(), DEC);
}
#endif

//Calculate the number of ms to reach set temp
unsigned long calcRunTime(int16_t tempNow, int16_t tempSet, int16_t tempExt) {
  unsigned long noMs = 0;
  if (tempNow < tempSet) {
    int16_t deg = degPerHour;
    if (tempExt != 1000 && tempExt < tempNow) {
      //Reduce based on outside temp
      deg -= (tempNow - tempExt) * extAdjustment;
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

void switchHeat(boolean on) {
  if (on) {
   digitalWrite(RED_LED, LOW);
   digitalWrite(GREEN_LED, HIGH);
   digitalWrite(RELAY, HIGH);
   heatOn = true;
  } else {
   digitalWrite(RED_LED, HIGH);
   digitalWrite(GREEN_LED, LOW);
   digitalWrite(RELAY, LOW);
   heatOn = false;
  }
}

