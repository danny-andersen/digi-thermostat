
#include <Wire.h>
#include <uRTCLib.h>

#include <DallasTemperature.h>
#include <OneWire.h>

//#include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>

//Debounce switches
#include <Bounce2.h>

// #include <printf.h>
#include <SoftwareSerial.h>
#include <AceCRC.h>

using namespace ace_crc::crc16ccitt_nibble;

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
LiquidCrystal_I2C lcd(0x27, 20, 4);

//WIFI Port
SoftwareSerial wifiSerial(WIFI_RX, WIFI_TX); // RX, TX

//Global variables and stuff to initate once
int16_t currentTemp = 1000;
int16_t currentSentThermTemp = 1000;
int16_t lastScheduledTemp = 0;
volatile int16_t currentSetTemp = 0;
byte noOfSchedules = 0;
boolean isDefaultSchedule = false;
SchedUnion defaultSchedule;
SchedUnion currentSched;
SchedUnion nextSched;
HolidayUnion holiday;
boolean onHoliday = false;
boolean heatOn = false;
int16_t degPerHour = 50; //Default heating power - 5C per hour increase
int16_t extAdjustment = 50; //Weighting of difference between external and internal temp, e.g. if 10 deg diff (100/50) = -2deg

unsigned long currentMillis = 0;
unsigned long lastRTCRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastGetSched = 0;
unsigned long boilerOnTime = 0;
unsigned long lastLoopTime = 0;
unsigned long lastScrollTime = 0;
unsigned long loopDelta = 0;
unsigned long backLightTimer = BACKLIGHT_TIME;
unsigned long motdExpiryTimer = 0;
unsigned long lastThermTempTime = 0; //Time at which rx last thermometer temp
unsigned long lastMessageCheck = 0;
unsigned long lastRTCTime = 0;
//unsigned long networkDownTime = 0;

char* dayNames[7] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
char* monNames[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

//Rotary encoder
byte rotaryA = 0;
byte rotaryB = 0;
unsigned long lastTriggerTimeA = 0;
unsigned long lastTriggerTimeB = 0;

Bounce boostButton = Bounce();
unsigned long boostTimer = 0;

byte schedules[MAX_SCHEDULES][sizeof(SchedByElem)];

int16_t extTemp = 1000; //This will be set by remote command
char motd[MAX_MOTD_SIZE]; //Will be set by remote command
char windStr[MAX_WIND_SIZE]; //Will be set by remote command
char defaultMotd[] = "V:" __DATE__ " S:"; //Show build date and number of schedules
char lcdBuff[LCD_COLS + 1]; //Buffer used to display and scroll motd on LCD
int8_t scrollPos = 0;
uint8_t changedState = 0;  //Whether a state change has happened that should be display (1=display, 2=clear screen before displaying)

//bool networkDown = true;
//bool serverDown = true;
//bool msgFail = true;
bool resendMessages = true;

void setup() {
  Serial.begin(9600);
  //  printf_begin();
  //  while (!Serial);
  currentMillis = 0;
  lastRTCRead = 0;
  lastTempRead = 0;
  lastGetSched = 0;
  boilerOnTime = 0;
  lastLoopTime = 0;
  lastScrollTime = 0;
  loopDelta = 0;
  backLightTimer = BACKLIGHT_TIME;
  motdExpiryTimer = 0;
  lastThermTempTime = 0; //Time at which rx last thermometer temp
  lastMessageCheck = 0;
  lastRTCTime = 0;
  //Digi outs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  //Digi ins
  pinMode(BOOST_BUTTON, INPUT_PULLUP);
  digitalWrite(BOOST_BUTTON, HIGH);
  boostButton.attach(BOOST_BUTTON);
  boostButton.interval(DEBOUNCE_TIME);

  //Rotary encoder
  pinMode(ROTARY_A, INPUT);
  pinMode(ROTARY_B, INPUT);
  digitalWrite(ROTARY_A, HIGH);
  digitalWrite(ROTARY_B, HIGH);
  rotaryA = digitalRead(ROTARY_A);
  rotaryB = digitalRead(ROTARY_B);
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

  //Read schedule from EEPROM
  //Note: Max position: 32767
  //First byte is number of schedules
  noOfSchedules = eepromRead(0);
  //  Serial.println("No of scheds: " + String(noOfSchedules, HEX));
  if (noOfSchedules > MAX_SCHEDULES) {
    //Assume eprom corrupted
    eepromWrite(0, 0x00);
    noOfSchedules = 0;
  }
  int cnt = 1;
  for (int i = 0; i < noOfSchedules; i++) {
    for (int j = 0; j < sizeof(SchedByElem); j++) {
      schedules[i][j] = eepromRead(cnt);
      cnt++;
    }
  }
  if (noOfSchedules == 0) {
    addDefaultSchedule();
  }

  holiday.elem.valid = 0;
  //Go to end of schedule storage area and see if a holiday has been stored
  cnt = 1 + (MAX_SCHEDULES * sizeof(SchedByElem));
  if (eepromRead(cnt) == 1) {
    //Holiday has been set
    cnt++;
    holiday.raw[0] = eepromRead(cnt);
    cnt++;
  }
  //  Serial.println("holiday? " + String(holiday.elem.valid) + " start day: " + String(holiday.elem.startDate.dayOfMonth));

  setDefaultMotd();

  //Get the time and current set temp
  rtc.refresh();
  currentSched.elem.start = 1500;
  currentSched.elem.temp = 100;
  //Done set up

  // Connect to the network
  //Start WifiPort
  wifiSerial.begin(ESP_AT_BAUD);
  while (!wifiSerial);

  delay(RECONNECT_WAIT_TIME); //Give time for wifi to connect to AP
  //Turn power led RED to indicate running
  digitalWrite(GREEN_LED, LOW);

}


void loop() {
  changedState = 0;
  lastLoopTime = currentMillis;
  currentMillis = millis();
  loopDelta = currentMillis - lastLoopTime;

  if (currentMillis - lastRTCRead > RTC_READ_INTERVAL) {
    //Read RTC
    rtc.refresh();
    lastRTCRead = currentMillis;
    changedState = 1;
  }
  //  Serial.print(getDateStr());
  //  Serial.print(" ");
  //  Serial.print(getTimeStr());
  //  Serial.print("\n");

  if (boostTimer > 0) {
    if (boostTimer > loopDelta) {
      boostTimer -= loopDelta;
    } else {
      boostTimer = 0;
    }
  }

  //Read thermometer if not got remote reading
  if (currentMillis - lastThermTempTime > RX_TEMP_INTERVAL && currentSentThermTemp != 100) {
    if (currentMillis - lastTempRead > TEMPERATURE_READ_INTERVAL) {
      temp_sensor.requestTemperatures(); // Send the command to get temperatures
      currentTemp = (int16_t)(temp_sensor.getTempCByIndex(0) * 10);
      //  Serial.println("Temperature for Device 1 is: " + String(currentTemp, 1));
      lastTempRead = currentMillis;
      changedState = 1;
    }
  } else {
    //Use the sent temperature, not the one read internally
    currentTemp = currentSentThermTemp;
  }

  //Retreive current set point in schedule
  if (currentMillis - lastGetSched > SCHED_CHECK_INTERVAL) {
    lastGetSched = currentMillis;
    int16_t currentSchedTemp;
    //Check if on hols
    onHoliday = checkOnHoliday();
    if (onHoliday) {
      if (lastScheduledTemp != holiday.elem.temp) {
        //Override set temp with holiday temp. Note that this can be overriden manually
        currentSetTemp = holiday.elem.temp;
        lastScheduledTemp = holiday.elem.temp;
        changedState = 1;
      }
    } else {
      SchedUnion lastSched;
      memcpy(&lastSched.raw, &currentSched.raw, sizeof(SchedByElem));
      uint16_t mins = rtc.hour() * 60 + rtc.minute();
      getSetPoint(&currentSched, mins, rtc.dayOfWeek(), false);
      if (memcmp(&currentSched.raw, &lastSched.raw, sizeof(SchedByElem)) == 0) {
        //Schedule has changed - find next schedule
        uint16_t minsNext = currentSched.elem.end;
        if (currentSched.elem.day == 0 || currentSched.elem.end == 0) {
          //Currently in default sched
          minsNext = mins;
        }
        getSetPoint(&nextSched, minsNext, rtc.dayOfWeek(), true);
        if (memcmp(&nextSched.raw, &defaultSchedule.raw, sizeof(SchedByElem)) == 0) {
          //No more schedules for today (as returned the default schedule)
          //Get first one tomorrow
          int nextDay = rtc.dayOfWeek() + 1;
          if (nextDay > 7) nextDay = 1;
          getSetPoint(&nextSched, 0, nextDay, true);
        }
        currentSchedTemp = currentSched.elem.temp;
      }
      //      Serial.println("Scheduled set point: " + String(currentTemp, 1));
      if (lastScheduledTemp != currentSchedTemp) {
        //Only override set temp if there is a schedule change (cos it may have been manually set)
        currentSetTemp = currentSched.elem.temp;
        lastScheduledTemp = currentSched.elem.temp;
        changedState = 1;
      }

    }
  }

  //Read switch input
  readInputs();

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
      //Forecast has expired, as has external temp and wind
      //Request new ones
      motdExpiryTimer = 0;
      bool result = false;
      if (networkUp) {
        if (getMotd()) {
          result = true;
        }
        drainSerial();
      }
      if (!result) {
        setDefaultMotd();
      }
      result = false;
      if (networkUp) {
        if (getExtTemp()) {
          result = true;
        }
      }
      if (!result) {
        windStr[0] = '\0';
        extTemp = 1000;
      }
      changedState = 2;
    }
  }

  if (checkBackLight()) {
    lcd.backlight();
  } else {
    lcd.noBacklight();
  }


  //Check for any messages
  if (currentMillis - lastMessageCheck > MESSAGE_CHECK_INTERVAL) {
    if (networkUp) {
      getNextMessage();
      drainSerial();
    }
  }

  //Check if need time update
  if ((currentMillis - lastRTCTime) > GET_TIME_INTERVAL) {
    //    Serial.println("Get datetime");
    if (networkUp) {
      getDateTime();
      drainSerial();
    }
  }

  if (!networkUp) {
    uint8_t stat = checkNetworkUp(false);
    if (stat == 0) {
      networkUp = true;
    }
    if (stat == 1) {
      //Got full status report
      //        debugSerial.print("Network back up. New motd:");
      snprintf(motd, MAX_MOTD_SIZE, (char *)buff);
      //        debugSerial.println(motd);
    } else if (stat == 2) {
      //        debugSerial.print("Network down. New motd:");
      snprintf(motd, MAX_MOTD_SIZE, (char *)"No response from wifi card");
      //        debugSerial.println(motd);
    }
  }

  if (changedState > 0) {
    displayState();
  }

  unsigned long nowMillis = millis(); //calc whether to scroll based on time now, as if server down, can be a long pause
  
  if (strlen(motd) > LCD_COLS && (nowMillis - lastScrollTime) > SCROLL_PAUSE) {
    //Need to scroll the 4th line of the display
    lastScrollTime = nowMillis;
    scrollLcd();
  }

  delay(LOOP_DELAY);
}

// Interrupt on A changing state
void intHandlerRotaryA() {
  unsigned long t = micros();
  if (t - lastTriggerTimeA > DEBOUNCE_TIME) {
    rotaryA = digitalRead(ROTARY_A);
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
    rotaryB = digitalRead(ROTARY_B);
    //Only inc state on rising edge of B and A is off (CCW rotation)
    if (rotaryB && !rotaryA)
      currentSetTemp = currentSetTemp - SET_INTERVAL;
  }
  lastTriggerTimeB = t;
}

void readInputs(void) {
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
}

void addDefaultSchedule() {
  //Add a default schedule - in mem only
  defaultSchedule.elem.day = 0;
  defaultSchedule.elem.start = 0;
  defaultSchedule.elem.end = 1440;
  defaultSchedule.elem.temp = 110;
  memcpy(&schedules[0], &defaultSchedule.raw, sizeof(SchedByElem));
  noOfSchedules = 1;
  isDefaultSchedule = true;
}

void setTempMotd(char templateStr[], char param[]) {
  snprintf(motd, MAX_MOTD_SIZE, templateStr, param);
  motdExpiryTimer = TEMP_MOTD_TIME;
  scrollPos = 0;
}

//bool checkAndOpenTcp() {
//  bool openAndReady = false;
//  if (checkNetworkUp()) {
//    if (networkDown) {
//      setTempMotd(NETWORK_STATUS, "Up");
//      networkDown = false;
//    }
//    networkDownTime = 0;
//    if (openTCP()) {
//      if (serverDown) {
//        setTempMotd(SERVER_STATUS, "Up");
//        serverDown = false;
//      }
//      openAndReady = true;
//    } else {
//      serverDown = true;
//      setTempMotd(SERVER_STATUS, "Down");
//    }
//  } else {
//    setTempMotd(NETWORK_STATUS, "Down");
//    networkDown = true;
//    serverDown = true;
//  }
//  if (networkDown || serverDown) {
//    msgFail = true;
//    resendMessages = true;
//    networkDownTime += LOOP_DELAY;
//    // Serial.print("Net down: ");
//    // Serial.println(networkDownTime);
//    if (networkDownTime >= RECONNECT_WAIT_TIME*12) {
//      //Reset wifi adapter every 2 mins, if not connected
//      connectToAP();
//      networkDownTime = 0;
//    }
//  }
//  return openAndReady;
//}

//bool connectToAP() {
////   Serial.println("Reset wifi");
//  drainSerial();
//  drainSerial();
//  wifiSerial.println(WIFI_RESET);
//  drainSerial();
//
//  delay(RECONNECT_WAIT_TIME); //Let it connect and get an IP
//  drainSerial();
//
//  //Set STA mode
//  // debugSerial.println("STA mode");
//  wifiSerial.println(SET_STA_MODE);
//  if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
//    // printBuff(64);
//    return false;
//  }
//  drainSerial();
//
//  if (checkNetworkUp()) {
//    //Get local IP
//    //  debugSerial.println("Retrieve IP: ");
//    wifiSerial.println(GET_IP_CMD);
//    if (not checkResponse(OK, 2, NO_IP, 5)) {
////       printBuff(64);
//      return false;
//    }
//    drainSerial();
//  } else {
//    return false;
//  }
//  return true;
//}
//
void zeroSendBuffer() {
  for (int i = 0; i < MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
}

//uint8_t sendMsgBuffer() {
//  //Count the message len
//  int msglen = 0;
//  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
//    msglen++;
//    if (getMessage[i] == 0) {
//      break;
//    }
//  }
//  return sendMessage(getMessage, msglen+3); //newlines
//}

//bool getNextMessage() {
//  zeroSendBuffer();
//  //Status message format = /message?rs=resend messages, t=<temp> st=<thermostat set temp>&r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
//  unsigned int runTime = getRunTime() / 1000; //No of secs
//  //Send current status as params in request"message?s=1&rs=%d&t=%d&st=%d&r=%d&p=%d"
//  strcat(getMessage, GET);
//  snprintf(&getMessage[sizeof(GET)-1], MAX_GET_MSG_SIZE, GET_MESSAGE_TEMPLATE, (int)resendMessages, (int)currentTemp, (int)currentSetTemp, runTime, (backLightTimer == BACKLIGHT_TIME));
//  strcat(getMessage, HTTP);
//  uint8_t msgId = sendMsgBuffer();
//  return processMessage(msgId);
//}

bool getNextMessage() {
  for (int i = 0; i < MAX_GET_MSG_SIZE; i++) {
    nextMessage[i] == 0;
  }
  //Send current status as params in request
  //Status message format = /message?s=station&rs=resend init messages&t=<temp>st=<thermostat set temp>  &r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
  //GET /message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d
  snprintf(nextMessage, MAX_GET_MSG_SIZE, NEXT_MESSAGE_TEMPLATE, 1, resendMessages, (int)currentTemp, (int)currentSetTemp, 3, 1);
  //Count the message len
  int msglen = 0;
  for (int i = 0; i < MAX_GET_MSG_SIZE; i++) {
    if (nextMessage[i] == 0) {
      break;
    }
    msglen++;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, msglen, nextMessage);
  uint8_t msgId = sendMessage( getMessage);
  resendMessages = 0;
  return processMessage(msgId);
}

bool getDateTime() {
  zeroSendBuffer();
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 8, dateTime);
  uint8_t msgId = sendMessage(getMessage);
  return processMessage(msgId);
}

//bool getThermTemp() {
//  zeroSendBuffer();
//  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 4, thermTemp);
//  uint8_t msgId = sendMessage(getMessage);
//  return processMessage(msgId);
//}
//
//bool getSetTemp() {
//  zeroSendBuffer();
//  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, setTemp);
//  uint8_t msgId = sendMessage(getMessage);
//  return processMessage(msgId);
//}
//

bool getExtTemp() {
  zeroSendBuffer();
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, extTempStr);
  uint8_t msgId = sendMessage(getMessage);
  return processMessage(msgId);
}

bool getMotd() {
  zeroSendBuffer();
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 4, motdStr);
  uint8_t msgId = sendMessage(getMessage);
  return processMessage(msgId);
}

//void getHoliday() {
//  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
//    getMessage[i] == 0;
//  }
//  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, holiday);
//  uint8_t msgId = sendMessage(getMessage);
//  processMessage(msgId);
//}


//OLD STYLE STARTS HERE:
//bool getDateTime() {
//  zeroSendBuffer();
//  snprintf(getMessage, MAX_GET_MSG_SIZE, dateTimeStr, GET, HTTP);
//  uint8_t msgId = sendMsgBuffer();
////  Serial.println("Got datetime msg: " + msgId);
//  return processMessage(msgId);
//}
//
//bool getMotd() {
//  zeroSendBuffer();
//  snprintf(getMessage, MAX_GET_MSG_SIZE, motdStrStr, GET, HTTP);
//  uint8_t msgId = sendMsgBuffer();
//  return processMessage(msgId);
//}
//
//bool getExtTemp() {
//  zeroSendBuffer();
//  snprintf(getMessage, MAX_GET_MSG_SIZE, extTempStr, GET, HTTP);
//  uint8_t msgId = sendMsgBuffer();
//  return processMessage(msgId);
//}

//void getSetTemp() {
//  zeroSendBuffer();
//  snprintf(getMessage, MAX_GET_MSG_SIZE, setTempStr, GET, HTTP);
//  return processMessage(msgId);
//}

//void getThermTemp() {
//  uint8_t msgId = sendMessage(thermTempStr, sizeof(thermTempStr)+3);
//  processMessage(msgId);
//}
//
//void getHoliday() {
//  uint8_t msgId = sendMessage(holidayStr, sizeof(holidayStr)+3);
//  processMessage(msgId);
//}
//

bool processMessage(uint8_t msgId) {
  bool msgRx = true;
  switch (msgId) {
    case 0:
      //No message
      lastMessageCheck = currentMillis;
      msgRx = false; //No actual message returned
      //This is the default message that is sent if no other
      break;
    case SET_DATE_TIME_MSG:
      //Process Date time
      setDateTime();
      break;
    case MOTD_MSG:
      //Process Date time
      setMotd();
      break;
    case SET_THERM_TEMP_MSG:
      //Process thermometer temp
      setThermTemp();
      //Only this and no message set the lastMessageCheck time
      //this means that next message will be requested on the next loop
      lastMessageCheck = currentMillis;
      break;
    case SET_TEMP_MSG:
      //Process set temp msg
      setSetTemp();
      break;
    case SET_EXT_MSG:
      //Process external temp
      setExtTemp();
      break;
    case SET_HOLIDAY_MSG:
      //Process Date time
      setHoliday();
      break;
    case SCHEDULE_MSG:
      //Process Date time
      setSchedule();
      break;
    case DELETE_ALL_SCHEDULES_MSG:
      deleteAllSchedules();
      break;
  }
  resendMessages = false;
  flickerLED();
  return msgRx;
}

void setMotd() {
  Motd *p = (Motd *)&buff[4]; //Start of content
  motdExpiryTimer = p->expiry;
  motd[0] = '\0';
  strncat(&motd[0], p->motdStr, MAX_MOTD_SIZE);
  scrollPos = 0;
  changedState = 2;
}

void setThermTemp() {
  Temp *tp = (Temp *)&buff[4]; //Start of content
  currentSentThermTemp = tp->temp;
  lastThermTempTime = currentMillis;
  changedState = 2;
}

void setSetTemp() {
  Temp *tp = (Temp *)&buff[4]; //Start of content
  currentSetTemp = tp->temp;
  changedState = 2;
}

void setExtTemp() {
  SetExt *tp = (SetExt *)&buff[4]; //Start of content
  extTemp = tp->temp;
  windStr[0] = '\0';
  strncat(&windStr[0], tp->windStr, MAX_WIND_SIZE);
  changedState = 2;
}

void setDateTime() {
  lastRTCTime = currentMillis;
  DateTimeStruct *dtp = (DateTimeStruct *)&buff[4]; //Start of content
  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
  rtc.set(dtp->sec,
          dtp->min,
          dtp->hour,
          dtp->dayOfWeek,
          dtp->dayOfMonth,
          dtp->month,
          dtp->year);
  rtc.refresh();
  changedState = 2;
}

void setHoliday() {
  HolidayUnion *dtp = (HolidayUnion *)&buff[4]; //Start of content
  //    HolidayUnion *dtp = (HolidayUnion *)&buff[4]; //Start of content
  memcpy(&holiday.raw[0], &dtp->raw[0], sizeof(HolidayStr));
  //Save in eeprom
  int cnt;
  cnt = 1 + (MAX_SCHEDULES * sizeof(SchedByElem));
  eepromWrite(cnt, 1); //Set holiday as valid
  cnt++;
  byte *hols;
  hols = &dtp->raw[0];
  eepromWrite(cnt, *hols++);
  cnt++;
  changedState = 2;
}

void deleteAllSchedules() {
  noOfSchedules = 0;
  eepromWrite(0, noOfSchedules);
  addDefaultSchedule();
}

void setSchedule() {
  SchedByElem *dtp = (SchedByElem *)&buff[4]; //Start of content
  //Insert schedule
  SchedByElem elem;
  SchedUnion sched;
  elem.day = dtp->day;
  elem.start = dtp->start;
  elem.end = dtp->end;
  elem.temp = dtp->temp;
  sched.elem = elem;
  if (isDefaultSchedule) {
    //Overwrite the default
    noOfSchedules = 0;
    isDefaultSchedule = false;
  }
  memcpy(&schedules[noOfSchedules], &sched.raw, sizeof(SchedByElem));
  writeSchedule(noOfSchedules, &sched.raw[0]);
  noOfSchedules++;
  if (motdExpiryTimer == 0) {
    setDefaultMotd(); //Show the number of schedules written as it is in the default motd
  }
  //Write number of schedules at start address
  eepromWrite(0, noOfSchedules);
}

// case (ADJ_SETTIME_CONST_MSG):
//     degPerHour = payload.adjSetTimeConstants.degPerHour;
//     extAdjustment = payload.adjSetTimeConstants.extAdjustment;
//     changedState = 1;
//   break;
// case (MOTD_MSG):
//   break;
// case (GET_SCHEDULES_MSG):
//     //Send each schedule in the list
//     respHeader.type = SCHEDULE_MSG;
//     for (int i=0; i<noOfSchedules; i++) {
//       memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
//       response.schedule.day = sched.elem.day;
//       response.schedule.start = sched.elem.start;
//       response.schedule.end = sched.elem.end;
//       response.schedule.temp = sched.elem.temp;
//       //Send to master
//       network.write(respHeader, &response, sizeof(Content));
//       delay(100);
//     }
//   break;
// case (DELETE_SCHEDULE_MSG):
//     int schedToDelete;
//     schedToDelete = noOfSchedules+1;
//     for (int i=0; i<noOfSchedules; i++) {
//       memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
//       //Find matching schedule
//       if (sched.elem.day == payload.schedule.day
//          && sched.elem.start == payload.schedule.start
//          && sched.elem.end == payload.schedule.end
//          && sched.elem.temp == payload.schedule.temp) {
//           schedToDelete = i;
//       }
//     }
//     //Delete matching schedule by shuffling them up
//     for (int i=schedToDelete; i<(noOfSchedules-1); i++) {
//       memcpy(&schedules[i], &schedules[i++], sizeof(SchedByElem));
//     }
//   break;

void displayState() {
  //Display current status
  if (changedState > 1) {
    lcd.clear();
  }
  char tempStr[TEMP_SIZE];
  lcdBuff[0] = '\0';
  //Display Row 1 - Current time and temperature
  lcd.setCursor(0, 0);
  int pos = getTimeStr(&lcdBuff[0]);
  String currTempStr;
  if (currentTemp == 1000) {
    currTempStr = "??.?";
  } else if (currentTemp == 0) {
    currTempStr = " 0.0";
  } else {
    currTempStr = String((float)(currentTemp / 10.0), 1); //Only way to generate a float str in Arduino
  }
  if (currentTemp < 100 && currentTemp > 0) {
    currTempStr = "0" + currTempStr;
  }
  currTempStr.toCharArray(tempStr, TEMP_SIZE);
  sprintf(lcdBuff, "%s   %sC", lcdBuff, tempStr);
  Serial.println(lcdBuff);
  lcd.print(lcdBuff);

  //Display row 2 - This is one of: Boiler on time + Set temp, Current date + Set Temp, Sched end or start time + Set Temp
  lcd.setCursor(0, 1);
  //    lcdBuff[0] = '\0';
  char sp[] = " ";
  char runTimeStr[11]; //10 chars on LCD + end of str
  runTimeStr[0] = '\0';
  if (rtc.second() % 2) {
    //Show schedule end or next sched start
    if (onHoliday) {
      strncat(runTimeStr, "On hols!!!", 10);
    } else if (currentSched.elem.day == 0) {
      //Currently in the default schedule valid for all times, so show next sched start
      char on[] = "ON @ HHMM";
      getHoursMins(nextSched.elem.start, &on[5]);
      strncat(&runTimeStr[0], &on[0], strlen(on));
      strncat(&runTimeStr[9], sp, 1);
    } else {
      //Show end time of current sched
      char off[] = "OFF @ HHMM";
      getHoursMins(currentSched.elem.end, &off[6]);
      strncat(runTimeStr, &off[0], strlen(off));
    }
  } else {
    //Show boiler runtime or date
    unsigned long runTime = getRunTime();
    if (runTime != 0) {
      //Boiler is on - display how long for on row 2
      char run[] = "Run:MM:SS";
      getMinSec(runTime, &run[4]);
      strncat(runTimeStr, &run[0], strlen(run));
      strncat(&runTimeStr[8], sp, 1);
      //    Serial.println("Runtime:" + String(runTime) + " 3digi: " + String(threeDigit) + " runTimeStr: " + runTimeStr + " big: " + String(bigChangeOfState));
    } else {
      //Boiler not on - Show date
      getDateStr(runTimeStr);
      // strncat(&runTimeStr[0], dt, strlen(dt));
    }
  }
  String setTempStr;
  if (currentSetTemp == 0) {
    setTempStr = " 0.0";
  } else {
    setTempStr = String((float)(currentSetTemp / 10.0), 1); //Only way to generate a float str in Arduino
  }
  if (currentSetTemp < 100 && currentSetTemp > 0) {
    setTempStr = "0" + setTempStr;
  }
  setTempStr.toCharArray(tempStr, TEMP_SIZE);
  sprintf(&lcdBuff[0], "%s Set:%sC", runTimeStr, tempStr);
  //    lcdBuff[LCD_COLS] = '\0';
  Serial.println(lcdBuff);
  delay(100);
  lcd.print(lcdBuff);
  // lcd.print(String(runTimeStr) + " Set:" + tempStr);

  lcd.setCursor(0, 2);
  int len = 0;
  if (strlen(windStr) != 0 && rtc.second() % 2) {
    //display wind speed
    len = snprintf(lcdBuff, MAX_WIND_SIZE, "%s", windStr);
  } else {
    // char bsbuf[MAX_WIND_SIZE];
    char bs[] = "Heat: %s";
    char on[] = "ON";
    char off[] = "OFF";
    // bsbuf[0] = '\0';
    // strncat(&bsbuf[0], &bs[0], strlen(bs));
    if (heatOn) {
      len = snprintf(lcdBuff, MAX_WIND_SIZE, bs, on);
      // strncat(&bsbuf[strlen(bsbuf)-1], &on[0], strlen(on));
    } else {
      len = snprintf(lcdBuff, MAX_WIND_SIZE, bs, off);
      // strncat(&bsbuf[strlen(bsbuf)-1], &off[0], strlen(off));
    }
  }
  lcdBuff[len] = '\0';
  // int i;
  // i = strlen(boilerStatStr);
  while (strlen(lcdBuff) < MAX_WIND_SIZE - 1) {
    lcdBuff[len] = ' ';
    len++;
    lcdBuff[len] = '\0';
  }
  String extTempStr;
  if (extTemp == 1000) {
    extTempStr = "??.?";
  } else if (extTemp == 0) {
    extTempStr = " 0.0";
  } else {
    extTempStr = String((float)(extTemp / 10.0), 1); //Only way to generate a float str in Arduino
  }
  if (extTemp < 100 && extTemp > 0) {
    currTempStr = "0" + currTempStr;
  }
  extTempStr.toCharArray(tempStr, TEMP_SIZE);
  if (extTemp >= 0) {
    sprintf(&lcdBuff[len - 1], " Ext:%sC", tempStr);
  } else {
    sprintf(&lcdBuff[len - 1], " Ext:%s", tempStr);
  }
  Serial.println(lcdBuff);
  delay(100);
  lcd.print(lcdBuff);
  // lcd.print(String(boilerStatStr) + " Ext:" + extTempStr);
  if (strlen(motd) <= LCD_COLS) {
    lcd.setCursor(0, 3);
    Serial.println(motd);
    delay(100);
    lcd.print(motd); //Only print here if motd fits, otherwise needs to scroll
  }
}

void scrollLcd() {
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
  strncpy(&lcdBuff[0], &motd[scrollPos], charsToCopy);
  if (blankChars > 0) {
    //Add blanks to the end of the message for a full screen
    memset(&lcdBuff[charsToCopy], ' ', blankChars);
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
  lcdBuff[LCD_COLS] = '\0';
  lcd.setCursor(0, 3);
  lcd.print(lcdBuff);
  Serial.println(lcdBuff);
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
    if (backLightTimer == 0) {
      //Force a message check, which flags the PIR status to the masterstation
      lastMessageCheck = 0;
    }
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
  for (int j = 0; j < sizeof(SchedByElem); j++) {
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
  Wire.requestFrom(EEPROM_I2C_ADDR, 1);
  if (Wire.available()) rdata = Wire.read();
  return rdata;
}


//Return the schedule ptr to the current schedule. or the next sched if asked for
void getSetPoint(SchedUnion *schedule, uint16_t mins, int currDay, bool nextSched) {
  //Find matching schedules and take one with the most specific day that matches
  //if next sched is true look for the schedule with a start time closest to the current
  union SchedUnion sched;
  int priority = 0;
  memcpy(&schedule->raw, &defaultSchedule.raw, sizeof(SchedByElem));
  uint16_t nextMins = 1440;
  for (int i = 0; i < noOfSchedules; i++) {
    memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
    //    Serial.println("Sched: " + sched.elem.day + String(" ") + sched.elem.temp);
    if (!nextSched && sched.elem.start == 0000 && sched.elem.end == 0000
        && sched.elem.day == 0 && priority == 0) {
      //This sched matches everything
      memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
    } else if ((!nextSched && sched.elem.start <= mins && sched.elem.end > mins) ||
               (nextSched && sched.elem.start > mins && sched.elem.start < nextMins)) {
      if (sched.elem.day == 0 && priority <= 1) {
        //All days match and not found higher
        priority = 1;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
      if (sched.elem.day == 0x200 && (currDay == 7 || currDay == 1) && priority <= 2) {
        //Its the weekend and not found higher
        priority = 2;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
      if (sched.elem.day == 0x100 && (currDay >= 2 || currDay <= 6) && priority <= 2) {
        //Its a weekday and not found higher
        priority = 2;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
      if (sched.elem.day == currDay && priority <= 3) {
        //Found a specific day - cant get higher
        priority = 3;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
    }
  }
}

boolean checkOnHoliday() {
  HolidayDateStr *hols;
  boolean afterStart = false;
  boolean beforeEnd = false;
  if (holiday.elem.valid == 1) {
    hols = &(holiday.elem.startDate);
    if (rtc.year() > hols->year) {
      afterStart = true;
    } else if (rtc.year() == hols->year) {
      if (rtc.month() > hols->month) {
        afterStart = true;
      } else if (rtc.month() == hols->month) {
        if (rtc.day() > hols->dayOfMonth) {
          afterStart = true;
        } else if (rtc.day() == hols->dayOfMonth) {
          if (rtc.hour() >= hols->hour) {
            afterStart = true;
          }
        }
      }
    }
    hols = &(holiday.elem.endDate);
    if (rtc.year() < hols->year) {
      beforeEnd = true;
    } else if (rtc.year() == hols->year) {
      if (rtc.month() < hols->month) {
        beforeEnd = true;
      } else if (rtc.month() == hols->month) {
        if (rtc.day() < hols->dayOfMonth) {
          beforeEnd = true;
        } else if (rtc.day() == hols->dayOfMonth) {
          if (rtc.hour() < hols->hour) {
            beforeEnd = true;
          }
        }
      }
    }
    //    Serial.println("after start:" + String(afterStart) + " before end:" + String(beforeEnd));
    if (!beforeEnd) {
      //Holiday is over - remove it
      holiday.elem.valid = 0;
      int cnt;
      cnt = 1 + (MAX_SCHEDULES * sizeof(SchedByElem));
      eepromWrite(cnt, 0); //Set holiday as invalid
    }
  }
  return (afterStart && beforeEnd);
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

void getHoursMins(unsigned long tmins, char *charBuf) {
  unsigned long hours = tmins / 60;
  unsigned long mins = tmins % 60;
  sprintf(charBuf, "%02d", hours);
  sprintf(charBuf + 2, "%02d", mins);
}

void getMinSec(unsigned long timeMs, char *charBuf) {
  unsigned long tsecs = timeMs / 1000;
  unsigned long mins = tsecs / 60;
  unsigned long secs = tsecs % 60;
  if (mins < 60) {
    sprintf(charBuf, "%02d:", mins);
    sprintf(charBuf + 3, "%02d", secs);
  } else {
    unsigned long hours = mins / 60;
    unsigned long remMins = mins % 60;
    sprintf(charBuf, "%02d:", hours);
    sprintf(charBuf + 3, "%02d", remMins);
  }
}

int getTimeStr(char *ptr) {
  int dayofweek = 1;
  int h = 12;
  int m = 13;
  int s = 31;
  return sprintf(ptr, "%s %02d:%02d:%02d", dayNames[dayofweek - 1], h, m, s );
  //  return sprintf(ptr, "%s %02d:%02d:%02d", dayNames[rtc.dayOfWeek() - 1],rtc.hour(), rtc.minute(), rtc.second() );
}

void getDateStr(char *ptr) {
  int d = 24;
  int m = 6;
  int y = 2022;
  return sprintf(ptr, "%02d %s %02d ", d, monNames[m], y);
  //  return sprintf(ptr, "%02d %s %02d ",rtc.day(),monNames[rtc.month()-1],rtc.year());
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
  sprintf(&motd[strlen(defaultMotd)], "%d", noOfSchedules);
}

uint8_t checkNetworkUp(bool statusOnly) {
  int8_t networkStat = 2;
  uint8_t retries = 0;
  while (retries < 2 && networkStat == 2) {
    //  Check network up
    drainSerial();
    //    debugSerial.println("Check net: ");
    if (statusOnly) {
      wifiSerial.println("*N");
    } else {
      wifiSerial.println("*S");
    }
    wifiSerial.println();
    networkStat = getStatusResponse(statusOnly);
    retries++;
  }
  //  if (networkStat != 0) {
  //    debugSerial.print("Net resp: ");
  //    debugSerial.println(networkStat);
  //    debugSerial.print("Buff:");
  //    printBuff(60);
  //  }
  return networkStat;
}

uint8_t sendMessage(char msg[]) {
  uint8_t retries = 0;
  int8_t msgId = -1; //Signifies message failed
  while (retries < 2 && msgId == -1) {
    drainSerial();
    wifiSerial.println(msg);
    wifiSerial.println();
    int16_t msgLen = waitForGetResponse();
    if (msgLen > 0) {
      //Got HTTP response - extract content part
      Message *msgp = (Message *)&buff[0];
      msgId = msgp->id;
      uint16_t rxCrc = msgp->crc;
      msgp->crc = 0;
      crc_t crc = crc_calculate(&buff[0], msgp->len);
      if (crc != rxCrc) {
        setTempMotd(MESSAGE_FAIL, "CRC failed");
        msgId = -1;
      } else if (rxInFail) {
        rxInFail = false;
        setTempMotd(SERVER_STATUS, "Now Up");
      }
    } else if (msgLen == 0 ) {
      setTempMotd(LITERAL_STATUS, (char *)buff);
      rxInFail = true;
      break;
    }
    retries++;
  }
  return msgId;
}

//Return:
//0 - Network up / Connected
//1 - Network down and got a response
//2 - No response from Wifi board
int8_t getStatusResponse(bool upOnly) {
  unsigned long start = millis();
  uint16_t bufPos = 0;
  unsigned long waitTime = 0;
  bool gotResponse = false;
  int8_t success = -1;
  while (waitTime < MAX_STATUS_TIME && !gotResponse) {
    uint16_t len = receiveData(bufPos);
    if (len > 0) {
      bufPos += len;
      if (success == -1) {
        success = buff[0] - 48;
        shiftBuffDown(1, bufPos); //Remove status byte
        bufPos--;
        if (success != 0 && success != 1) {
          //wrong status byte, try again
          success = -1;
          // Serial.print(buff[0], HEX);
          // Serial.print(" ");
        }
      }
      if (success != -1) {
        if (upOnly) {
          gotResponse = true;
        } else {
          //Look for EOL
          int16_t pos = findStringInBuff(buff, EOL, sizeof(EOL) - 1, bufPos);
          if (pos != -1) {
            gotResponse = true;
            buff[pos] = '\0';
          }
        }
      }
    }
    if (!gotResponse) {
      //Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - start;
  }
  if (!gotResponse) {
    //    Serial.print(" Failed to get net status, len: ");
    //    Serial.println(bufPos);
    //    printBuff(bufPos);
    success = 2;//Indicates response timed out
  }
  return success;
}

int16_t waitForGetResponse() {
  unsigned long start = millis();
  unsigned long lastByteTime = start;
  uint16_t bufPos = 0;
  unsigned long waitTime = 0;
  unsigned long totalWaitTime = 0;
  int16_t msgLen = 0;
  bool gotMsg = false;
  int8_t success = -1;
  while (waitTime < MAX_RESPONSE_TIME && !gotMsg) {
    uint16_t len = receiveData(bufPos);
    if (len > 0) {
      lastByteTime = millis();
      bufPos += len;
      if (success == -1) {
        success = buff[0] - 48;
        shiftBuffDown(1, bufPos); //Remove status byte
        bufPos--;
        if (success < 0 || success > 2) {
          //wrong status byte, try again
          success = -1;
        }
      }
      if (success > 0) {
        //Fail -> Look for EOL
        //        debugSerial.print("Send Fail: got:");
        //        debugSerial.print(bufPos);
        int16_t pos = findStringInBuff(buff, EOL, sizeof(EOL) - 1, bufPos);
        //        debugSerial.print(" EOL:");
        //        debugSerial.println(pos);
        if (pos != -1) {
          buff[pos] = '\0';
          gotMsg = true;
        }
      } else if (success == 0) {
        //Valid message response - Look for message len
        if (bufPos > 2) {
          msgLen = buff[1];
          if (bufPos >= msgLen) {
            gotMsg = true;
          }
        }
      }
    }
    if (!gotMsg) {
      //Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - lastByteTime; //Timeout if not received a byte for max wait time
  }
  if (waitTime >= MAX_RESPONSE_TIME && !gotMsg) {
    //    debugSerial.print("Rx timeout:");
    //    debugSerial.print(msgLen);
    //    debugSerial.print(" Got:");
    //    debugSerial.print(bufPos);
    //    printBuff(bufPos);
    snprintf(motd, MAX_MOTD_SIZE, "Msg Timeout Rx %d of %d", bufPos, msgLen);
    // Serial.println(motd);
    motdExpiryTimer = TEMP_MOTD_TIME;
    scrollPos = 0;
    rxInFail = true;    
    msgLen = -1;
  }
  return msgLen;
}

void shiftBuffDown(uint16_t pos, uint16_t maxPos) {
  //  debugSerial.print("Downshift by ");
  //  debugSerial.println(pos);
  //  printBuff(MAX_MESSAGE_SIZE);
  uint16_t i = 0;
  for (; pos < maxPos; pos++, i++) {
    buff[i] = buff[pos];
  }
  //  debugSerial.println("After:");
  //  printBuff(MAX_MESSAGE_SIZE);
}

uint16_t receiveData(uint16_t startPosition) {
  uint16_t len = 0;
  uint16_t pos = startPosition;
  while (wifiSerial.available() > 0 && pos < BUFF_SIZE) {
    uint8_t b = wifiSerial.read();
    buff[pos++] = b;
    len++;
    // debugSerial.print(b, HEX);
  }
  return len;
}

uint16_t findStringInBuff(uint8_t bf[], char str[], uint8_t strLen, uint16_t maxPos) {
  //  debugSerial.print("Looking for ");
  //  debugSerial.print(str);
  //  debugSerial.print(" Size:");
  //  debugSerial.print(strLen);
  //  debugSerial.print(" MaxPos:");
  //  debugSerial.println(maxPos);
  //  printBuff(maxPos);
  uint16_t startPos = -1;
  for (int i = 0; (i + strLen) < maxPos; i++) {
    if (bf[i] == str[0]) {
      startPos = i;
      uint16_t strPos = 0;
      uint16_t endStr = i + strLen;
      int j = i;
      for (; j < endStr && j < maxPos; j++) {
        if (bf[j] != str[strPos++]) {
          break;
        }
      }
      if (j >= endStr) { //All chars matched
        break;
      } else {
        startPos = -1;
      }
    }
  }
  return startPos;
}

void drainSerial() {
  delay(50);
  int total = 0;
  while (uint16_t len = receiveData(0) > 0) {
    total += len;
    delay(10);
  }
  //  if (total > 0) {
  //    debugSerial.print("Drained: ");
  //    debugSerial.println(total);
  //  }
  resetBuffer();

}

void resetBuffer() {
  //Reset receive buffer
  for (uint32_t i = 0; i < BUFF_SIZE; i++) {
    buff[i] = 0;
  }
}


//bool checkNetworkUp() {
//  //Check network up
//  bool networkUp = true;
//  drainSerial();
////  Serial.println("wifi");
//  wifiSerial.println(QUERY_AP);
//  if (not checkResponse(OK, 2, NO_AP, 5)) {
//      networkUp = false;
//  }
//  drainSerial();
//  return networkUp;
//}

//bool openTCP() {
//  bool connected = true;
//  drainSerial();
////  Serial.println("tcp");
//  wifiSerial.println(OPEN_TCP);
//  if (not checkResponse(CONNECTED, 9, NO_IP, 5)) {
//      connected = false;
//  }
//  return connected;
//}
//
//bool closeTCP() {
//  //  debugSerial.println("Close TCP: ");
//  wifiSerial.println(CLOSE_TCP);
//  if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
//     return false;
//  }
//  drainSerial();
//}

//uint8_t sendMessage(char msg[], uint8_t len) {
////  debugSerial.print("Send msg len:");
////  debugSerial.println(len);
////  Serial.println(msg);
//  drainSerial();
//  wifiSerial.print(SEND_LEN);
//  wifiSerial.println(len);
//  if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
//    //  debugSerial.println("Fail send");
//     return 0;
//  }
//  drainSerial();
////  debugSerial.println("Sending HTTPGET");
//  //Send http request
//  wifiSerial.println(msg);
//  wifiSerial.println();
//  wifiSerial.println();
//  wifiSerial.println();
//  wifiSerial.println();
//  wifiSerial.println();
//  flickerLED();
//  uint8_t msgId = 0; //Signifies no message
//  uint16_t respLen = waitForHttpResponse();
//  if (respLen > 0) {
//    //Got HTTP response - extract content part
//    uint16_t msgLen = extractMessage(respLen);
//    if (msgLen > 0){
//      Message *msgp = (Message *)&buff[0];
//      msgId = msgp->id;
//      uint16_t rxCrc = msgp->crc;
//      msgp->crc = 0;
//      crc_t crc = crc_calculate(&buff[0], msgp->len);
//      if (crc != rxCrc) {
//        setTempMotd(MESSAGE_FAIL, "CRC");
//        msgFail = true;
//        msgId = 0;
//      }
////    snprintf(motd, MAX_MOTD_SIZE, "Rx msg %d", msgId);
//      if (msgFail) {
//          setTempMotd(MESSAGE_FAIL, "OK");
//          msgFail = false;
//      }
//    } else {
//        setTempMotd(MESSAGE_FAIL, "No MLen");
//        msgFail = true;
//        msgId = 0;
//    }
//  } else {
////    printBuff(64);
//    setTempMotd(MESSAGE_FAIL, "No HLen");
//    // debugSerial.println("Fail rx:");
//    msgFail = true;
//    msgId = 0;
//  }
//  return msgId;
//}
//
//uint16_t extractMessage(uint16_t msgLen) {
//  uint16_t contentLen = 0;
//  bool gotLen = false;
//  //Look for Content length in header part of response
//  uint16_t pos = findStringInBuff(buff, CONTENT_LENGTH, sizeof(CONTENT_LENGTH)-1, msgLen);
//  if (pos != -1) {
//    pos += sizeof(CONTENT_LENGTH);
//    //Retrieve content length
//    char lenStr[4];
//    uint16_t endPos = pos;
//    for (; endPos<(pos + 3); endPos++) {
//      if (int(buff[endPos]) == 13) {
//        endPos++;
//        break; //Reached end of line
//      }
//      lenStr[endPos-pos] = buff[endPos];
//    }
//    lenStr[endPos-pos] = '\0'; //null terminate
//    contentLen = atoi(&lenStr[0]);
//    if (contentLen > 0 ) {
//      endPos += 3; //0xd, 0xa, 0xd, 0xa at end of content str and before content
//      //shift bytes down in buff
//      shiftBuffDown(endPos, msgLen); //Now we should just have the message content in the buff..
//    }
//  }
//  return contentLen;
//}

//void resetBuffer() {
//  //Reset receive buffer
//  for (uint32_t i = 0; i<MAX_MESSAGE_SIZE; i++) {
//    buff[i] = 0;
//  }
//}

//bool checkResponse(char *str, uint8_t lenStr, char *failStr, uint8_t lenFail) {
//  unsigned long start = millis();
//  uint16_t bufPos = 0;
//  uint16_t pos = -1;
//  unsigned long waitTime = 0;
//  bool gotResponse = false;
//  bool success = false;
//  while (waitTime < MAX_RESPONSE_TIME && !gotResponse) {
//    uint16_t len = receiveData(bufPos);
//    if (len > 0) {
//      bufPos += len;
//      //Look for specific fail response
//      pos = findStringInBuff(buff, failStr, 5, lenFail);
//      if (pos != -1) {
//        gotResponse = true;
//        success = false;
//        break;
//      }
//      //Look for specific success response
//      pos = findStringInBuff(buff, str, lenStr, bufPos);
//      if (pos != -1) {
//        gotResponse = true;
//        success = true;
//        break;
//      }
//      //Then look for generic error response
//      pos = findStringInBuff(buff, ERROR_RESP, 5, bufPos);
//      if (pos != -1) {
//        gotResponse = true;
//        success = false;
//        break;
//      }
//      //Look for generic OK response
//      pos = findStringInBuff(buff, OK, 2, bufPos);
//      if (pos != -1) {
//        gotResponse = true;
//        success = true;
//        break;
//      }
//    }
//    if (!gotResponse) {
//      //Wait for a bit for the next bytes to arrive
//      delay(10);
//    }
//    waitTime = millis() - start;
//  }
////  Serial.println("Check:");
////  printBuff(64);
//  return success;
//}
//
//uint16_t waitForHttpResponse() {
//  unsigned long start = millis();
//  uint16_t bufPos = 0;
//  unsigned long waitTime = 0;
//  uint16_t msgLen = 0;
//  bool sent = false;
//  bool gotIPD = false;
//  bool gotMsgLen = false;
//  bool gotMsg = false;
//  while (waitTime < MAX_RESPONSE_TIME && !gotMsg) {
//    uint16_t len = receiveData(bufPos);
//    if (len > 0) {
//      bufPos += len;
//      if (!sent) {
//        //Check for SEND OK
//        uint16_t pos = findStringInBuff(buff, sendOK, sizeof(sendOK)-1, bufPos);
//        if (pos != -1) {
//          //Send went OK
//          //shift bytes down in buff
//          uint16_t endPos = pos+sizeof(sendOK)-1;
//          shiftBuffDown(endPos, bufPos);
//          bufPos -= endPos;
//          sent = true;
//        }
//      }
//      if (sent && !gotIPD) {
//        //Look for IPD
//        uint16_t pos = findStringInBuff(buff, ipd, sizeof(ipd)-1, bufPos);
//        if (pos != -1) {
//          //shift bytes down in buff
//          uint16_t endPos = pos+sizeof(ipd)-1;
//          shiftBuffDown(endPos, bufPos);
//          bufPos -= endPos;
//          gotIPD = true;
//        }
//      }
//      if (sent && gotIPD && !gotMsgLen) {
//        //Find the colon
//        uint16_t pos = findStringInBuff(buff, colon, sizeof(colon)-1, bufPos);
//        if (pos != -1) {
//          //Retrieve msglength
//          char lenStr[4];
//          for (int i=0; i<pos && i<3; i++) {
//            lenStr[i] = buff[i];
//          }
//          lenStr[pos] = '\0'; //null terminate
//          msgLen = atoi(&lenStr[0]);
//          //shift bytes down in buff
//          uint16_t endPos = pos+1;
//          shiftBuffDown(endPos, bufPos);
//          bufPos -= endPos;
//          gotMsgLen = true;
//          if (msgLen > BUFF_SIZE) {
//            msgFail = true;
//            msgLen = BUFF_SIZE;
//            setTempMotd(MESSAGE_FAIL, "Max Len");
////            Serial.print("Max Rx Msg Len:");
////            Serial.println(msgLen);
//          }
//        }
//      }
//      if (sent && gotIPD && gotMsgLen && !gotMsg) {
//        if (bufPos >= msgLen) {
//          //got all of the message
//          gotMsg = true;
//        }
//      }
//    }
//    if (!gotMsg) {
//      //Flicker LED to wait for a bit for the next bytes to arrive
//      flickerLED();
//    }
//    waitTime = millis() - start;
//  }
//  if (waitTime >= MAX_RESPONSE_TIME && !gotMsg) {
//    snprintf(motd, MAX_MOTD_SIZE, SERVER_STATUS, " TO");
//  }
//  return msgLen;
//}
//
//void shiftBuffDown(uint8_t pos, uint16_t maxPos) {
//  uint8_t p = pos;
//  for (int i=0; i<maxPos; i++) {
//    buff[i] = buff[p++];
//  }
//}
//
//uint16_t receiveData(uint16_t startPosition){
//  uint16_t len = 0;
//  uint16_t pos = startPosition;
//  while(wifiSerial.available() > 0 && pos < BUFF_SIZE) {
//      uint8_t b = wifiSerial.read();
//      buff[pos++] = b;
//      len++;
//      // debugSerial.print(b, HEX);
//  }
//  return len;
//}
//
////void printBuff(uint32_t pos) {
////    Serial.print("Hex:");
////    for (uint32_t i = 0; i<pos && i<MAX_MESSAGE_SIZE; i++) {
////      Serial.print((uint8_t)buff[i], HEX);
////      Serial.print(',');
////    }
////    Serial.print("\nText:");
////    for (uint32_t i = 0; i<pos && i<MAX_MESSAGE_SIZE; i++) {
////      Serial.print((char) buff[i]);
////    }
////    Serial.println();
////}
//
//
//uint16_t findStringInBuff(uint8_t bf[], char str[], uint8_t strLen, uint16_t maxPos) {
//  uint16_t startPos = -1;
//  for (int i = 0; (i+strLen)<maxPos; i++) {
//    if (bf[i] == str[0]) {
//      startPos = i;
//      uint16_t strPos = 0;
//      uint16_t endStr = i+strLen;
//      int j = i;
//      for (;j < endStr && j<maxPos; j++) {
//        if (bf[j] != str[strPos++]) {
//          break;
//        }
//      }
//      if (j >= endStr) { //All chars matched
//        break;
//      } else {
//        startPos = -1;
//      }
//    }
//  }
//  return startPos;
//}
