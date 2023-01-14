#include <Wire.h>
#include <uRTCLib.h>

#include <DallasTemperature.h>
#include <OneWire.h>

// #include <LiquidCrystal.h>
// #include <LiquidCrystal_I2C.h>
#include <LiquidCrystal_PCF8574.h>

// Debounce switches
#include <Bounce2.h>

// #include <printf.h>
#include <SoftwareSerial.h>
#include <AceCRC.h>

using namespace ace_crc::crc16ccitt_nibble;

#include "digi-thermostat.h"

// Thermometer variables
//  Setup a oneWire instance to communicate with any OneWire devices
//  (not just Maxim/Dallas temperature ICs)
OneWire oneWire(THERM_ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

// RTC
uRTCLib rtc;

// LCD
// LiquidCrystal_I2C lcd(0x27, 20, 4);
LiquidCrystal_PCF8574 lcd(0x27); // set the LCD address to 0x27

// WIFI Port
SoftwareSerial wifiSerial(WIFI_RX, WIFI_TX); // RX, TX

// Global variables and stuff to initate once
int16_t currentTemp = 1000;
int16_t currentSentThermTemp = 1000;
int16_t manHolidayTemp = 1000;
int16_t lastScheduledTemp = 1000;
volatile int16_t currentSetTemp = 0;
byte noOfSchedules = 0;
boolean isDefaultSchedule = false;
SchedUnion defaultSchedule;
SchedUnion currentSched;
SchedUnion nextSched;

HolidayUnion holiday;
boolean onHoliday = false;
Bounce holidayButton = Bounce();
long holidaySetTimer = 0;
long holidayTime = 0;

int16_t degPerHour = 50;    // Default heating power - 5C per hour increase
int16_t extAdjustment = 50; // Weighting of difference between external and internal temp, e.g. if 10 deg diff (100/50) = -2deg

unsigned long currentMillis = 0;
unsigned long lastRTCRead = 0;
unsigned long lastTempRead = 0;
unsigned long lastGetSched = 0;
unsigned long boilerRunTime = 0;
boolean heatOn = false;

unsigned long lastLoopTime = 0;
unsigned long lastScrollTime = 0;
unsigned long loopDelta = 0;
unsigned long backLightTimer = BACKLIGHT_TIME;
uint8_t pirStatus = 0; // 0 = off, 1 = on (triggered)
unsigned long motdExpiryTimer = 0;
unsigned long lastThermTempTime = 0; // Time at which rx last thermometer temp
unsigned long lastMessageCheck = 0;
unsigned long lastNetworkCheckTime = 0;
unsigned long lastRTCTime = 0UL;
unsigned long networkUpTime = 0; // Time at which the network was last up

char *dayNames[7] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
char *monNames[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

// Rotary encoder
byte rotaryA = 0;
byte rotaryB = 0;
unsigned long lastTriggerTimeA = 0;
unsigned long lastTriggerTimeB = 0;

byte schedules[MAX_SCHEDULES][sizeof(SchedByElem)];

int16_t extTemp = 1000;                   // This will be set by remote command
char motd[MAX_MOTD_SIZE + 1];             // Will be set by remote command
char windStr[MAX_WIND_SIZE];              // Will be set by remote command
char defaultMotd[] = "V:" __DATE__ " S:"; // Show build date and number of schedules
char lcdBuff[LCD_COLS + 1];               // Buffer used to display and scroll motd on LCD
int8_t scrollPos = 0;
uint8_t changedState = 0; // Whether a state change has happened that should be display (1=display, 2=clear screen before displaying)

// bool networkDown = true;
// bool serverDown = true;
// bool msgFail = true;
bool resendMessages = true;

void (*resetFunc)(void) = 0; // declare reset fuction at address 0

void setup()
{
  // Serial.begin(9600);
  //  printf_begin();
  // while (!Serial)
  // ;
  currentMillis = millis();
  lastRTCRead = currentMillis;
  lastTempRead = 0;
  lastGetSched = currentMillis;
  lastScrollTime = currentMillis;
  backLightTimer = BACKLIGHT_TIME;
  motdExpiryTimer = TEMP_MOTD_TIME + RECONNECT_WAIT_TIME; // Show the default message for temp period then request the latest motd
  lastThermTempTime = 0;                                  // Time at which rx last thermometer temp
  lastMessageCheck = 0;
  lastRTCTime = currentMillis;
  // Digi outs
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  pinMode(WIFI_RESET_PIN, OUTPUT);
  digitalWrite(WIFI_RESET_PIN, HIGH);
  // Digi ins
  pinMode(BOOST_BUTTON, INPUT_PULLUP);
  digitalWrite(BOOST_BUTTON, HIGH);
  holidayButton.attach(BOOST_BUTTON);
  holidayButton.interval(DEBOUNCE_TIME);

  // Rotary encoder
  pinMode(ROTARY_A, INPUT);
  pinMode(ROTARY_B, INPUT);
  digitalWrite(ROTARY_A, HIGH);
  digitalWrite(ROTARY_B, HIGH);
  rotaryA = digitalRead(ROTARY_A);
  rotaryB = digitalRead(ROTARY_B);
  attachInterrupt(digitalPinToInterrupt(ROTARY_A), intHandlerRotaryA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ROTARY_B), intHandlerRotaryB, CHANGE);

  // turn on power indicator - set orange until initialised
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);

  Wire.begin();

  temp_sensor.begin();
  readLocalTemp();

  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
  //  rtc.set(0,45, 15, 5, 3, 2, 17);
  //  eepromWrite(0,0x00);

  // set up the LCD's number of columns and rows:
  // lcd.init();
  // lcd.backlight();
  lcd.begin(20, 4);
  lcd.clear();
  lcd.display();
  lcd.setBacklight(255);

  // Read schedule from EEPROM
  // Note: Max position: 32767
  // First byte is number of schedules
  noOfSchedules = eepromRead(0);
  //  Serial.println("No of scheds: " + String(noOfSchedules, HEX));
  if (noOfSchedules > MAX_SCHEDULES)
  {
    // Assume eprom corrupted
    eepromWrite(0, 0x00);
    noOfSchedules = 0;
  }
  int cnt = 1;
  for (int i = 0; i < noOfSchedules; i++)
  {
    for (int j = 0; j < sizeof(SchedByElem); j++)
    {
      schedules[i][j] = eepromRead(cnt);
      cnt++;
    }
  }
  if (noOfSchedules == 0)
  {
    addDefaultSchedule();
  }

  holiday.elem.valid = 0;
  // Go to end of schedule storage area and see if a holiday has been stored
  cnt = 1 + (MAX_SCHEDULES * sizeof(SchedByElem));
  if (eepromRead(cnt) == 1)
  {
    // Holiday has been set
    cnt++;
    holiday.raw[0] = eepromRead(cnt);
    cnt++;
  }
  //  Serial.println("holiday? " + String(holiday.elem.valid) + " start day: " + String(holiday.elem.startDate.dayOfMonth));

  setDefaultMotd();

  // Get the time and current set temp
  rtc.refresh();
  currentSched.elem.start = 1500;
  currentSched.elem.temp = 100;
  // Done set up

  // Connect to the network
  // Start WifiPort
  wifiSerial.begin(ESP_AT_BAUD);
  while (!wifiSerial)
    ;

  delay(RECONNECT_WAIT_TIME); // Give time for wifi to connect to AP
  networkUp = true;           // Assume network is up unless shown otherwise
  networkUpTime = millis();
  lastNetworkCheckTime = networkUpTime;
  // Turn power led RED to indicate running

  digitalWrite(GREEN_LED, LOW);
}

void loop()
{
  changedState = 0;
  lastLoopTime = currentMillis;
  currentMillis = millis();
  loopDelta = currentMillis - lastLoopTime;

  if (currentMillis - lastRTCRead > RTC_READ_INTERVAL)
  {
    // Read RTC
    rtc.refresh();
    lastRTCRead = currentMillis;
    changedState = 1;
  }
  //  Serial.print(getDateStr());
  //  Serial.print(" ");
  //  Serial.print(getTimeStr());
  //  Serial.print("\n");

  if (holidaySetTimer > 0)
  {
    // Countdown timer for setting instant holiday time
    if (holidaySetTimer > loopDelta)
    {
      holidaySetTimer -= loopDelta;
    }
    else
    {
      holidaySetTimer = 0;
    }
  }
  else if (holidayTime > 0) // Only start count down holiday timer if still not setting time
  {
    // Countdown timer for instant holiday time
    if (holidayTime > loopDelta)
    {
      holidayTime -= loopDelta;
    }
    else
    {
      holidayTime = 0;
    }
  }

  // Read thermometer if not got remote reading
  if (currentMillis - lastThermTempTime > RX_TEMP_INTERVAL)
  {
    if (currentMillis - lastTempRead > TEMPERATURE_READ_INTERVAL)
    {
      readLocalTemp();
      //  Serial.println("Temperature for Device 1 is: " + String(currentTemp, 1));
      lastTempRead = currentMillis;
      changedState = 1;
      boilerRunTime = getRunTime();
    }
  }
  else
  {
    // Use the sent temperature, not the one read internally
    currentTemp = currentSentThermTemp;
  }

  // Retreive current set point in schedule
  if (currentMillis - lastGetSched > SCHED_CHECK_INTERVAL)
  {
    lastGetSched = currentMillis;
    int16_t currentSchedTemp;
    // Check if on hols
    onHoliday = checkOnHoliday();
    if (onHoliday)
    {
      // Override set temp with holiday temp. to prevent any manual local overrides
      // First check remote override...
      if (holidayTime > 0)
      {
        // In manual holiday mode - allow remote override
        currentSetTemp = manHolidayTemp;
      }
      else
      {
        currentSetTemp = holiday.elem.temp;
      }
    }
    else
    {
      uint16_t mins = rtc.hour() * 60 + rtc.minute();
      // Get current schedule
      getSetPoint(&currentSched, mins, rtc.dayOfWeek(), false);
      currentSchedTemp = currentSched.elem.temp;
      // uint16_t minsNext = currentSched.elem.end;
      // if (currentSched.elem.day == 0 || currentSched.elem.end == 0) {
      //   //Currently in default sched
      //   minsNext = mins;
      // }
      // Get next schedule
      getSetPoint(&nextSched, mins, rtc.dayOfWeek(), true);
      if (memcmp(&nextSched.raw, &defaultSchedule.raw, sizeof(SchedByElem)) == 0)
      {
        // No more schedules for today (as returned the default schedule)
        // Get first one tomorrow
        int nextDay = rtc.dayOfWeek() + 1;
        if (nextDay > 7)
          nextDay = 1;
        getSetPoint(&nextSched, 0, nextDay, true);
      }
      if (lastScheduledTemp != currentSchedTemp)
      {
        // Only override set temp if there is a schedule change (cos it may have been manually set)
        currentSetTemp = currentSched.elem.temp;
        lastScheduledTemp = currentSched.elem.temp;
        changedState = 1;
      }
    }
  }

  // Read switch input
  readInputs();

  // Turn heating on or off depending on temp
  if ((currentSetTemp > currentTemp) && !heatOn)
  {
    switchHeat(true);
    changedState = 1;
  }
  if (heatOn && (currentTemp > (currentSetTemp + HYSTERSIS)))
  {
    // Reached set temperature, turn heating off
    // Note: Add in hystersis to stop flip flopping
    switchHeat(false);
    changedState = 1;
  }

  // Check if motd has expired
  if (motdExpiryTimer > 0)
  {
    if (motdExpiryTimer >= loopDelta)
    {
      motdExpiryTimer -= loopDelta;
    }
    else
    {
      motdExpiryTimer = 0;
    }
  }
  if (networkUp && motdExpiryTimer == 0)
  {
    // Forecast has expired, as has external temp and wind
    // Request new ones
    bool result = false;
    if (networkUp && !rxInFail)
    {
      if (getMotd())
      {
        result = true;
      }
      drainSerial();
    }
    if (!result)
    {
      setDefaultMotd();
    }
    result = false;
    if (networkUp && !rxInFail)
    {
      if (getExtTemp())
      {
        result = true;
      }
    }
    if (!result)
    {
      windStr[0] = '\0';
      extTemp = 1000;
    }
    changedState = 1;
  }

  // Check for any messages
  if (networkUp && (currentMillis - lastMessageCheck) > MESSAGE_CHECK_INTERVAL)
  {
    getNextMessage();
    drainSerial();
  }

  if (!networkUp && (currentMillis - lastNetworkCheckTime) > NETWORK_CHECK_INTERVAL)
  {
    lastNetworkCheckTime = currentMillis;
    uint8_t stat = checkNetworkUp(false);
    if (stat == 0)
    {
      networkUp = true;
      networkUpTime = currentMillis;
    }
    else if (stat == 1)
    {
      // Got full status report
      //         debugSerial.print("Network still down. New motd:");
      //  snprintf(&motd[0], MAX_MOTD_SIZE, (char *)buff);
      setTempMotd(LITERAL_STATUS, (char *)&buff[0]);
      //        debugSerial.println(motd);
    }
    else if (stat == 2)
    {
      //        debugSerial.print("Network down. New motd:");
      // snprintf(&motd[0], MAX_MOTD_SIZE, (char *)"No response from wifi card");
      setTempMotd(LITERAL_STATUS, (char *)"No response from wifi card");
      //        debugSerial.println(motd);
    }
  }

  if ((currentMillis - networkUpTime) > NETWORK_DOWN_LIMIT)
  {
    resetWifi();
  }

  if (checkBackLight())
  {
    // lcd.backlight();
    lcd.display();
    lcd.setBacklight(255);
    pirStatus = 1;
    // lcd.on();
  }
  else
  {
    // lcd.noBacklight();
    lcd.noDisplay();
    lcd.setBacklight(0);
    pirStatus = 0;
    // lcd.off();
  }

  // Check if need time update
  if ((currentMillis - lastRTCTime) > GET_TIME_INTERVAL)
  {
    //    Serial.println("Get datetime");
    if (networkUp && !rxInFail)
    {
      getDateTime();
      drainSerial();
    }
  }

  if (changedState > 0)
  {
    displayState();
  }

  unsigned long nowMillis = millis(); // calc whether to scroll based on time now, as if server down, can be a long pause

  if (strlen(motd) > LCD_COLS && (nowMillis - lastScrollTime) > SCROLL_PAUSE)
  {
    // Need to scroll the 4th line of the display
    lastScrollTime = nowMillis;
    scrollLcd();
  }

  delay(LOOP_DELAY);
}

// Interrupt on A changing state
void intHandlerRotaryA()
{
  unsigned long t = micros();
  if (t - lastTriggerTimeA > DEBOUNCE_TIME)
  {
    rotaryA = digitalRead(ROTARY_A);
    // Only inc state on rising edge of A and B is off (CW rotation)
    if (rotaryA && !rotaryB)
    {
      if (holidaySetTimer > 0) // Setting holiday time
      {
        if (holidayTime > 0)
        {
          holidayTime += 15 * 60000; // Add 15 mins to holiday time
        }
        else
        {
          holidayTime = calcHolidayTime(); // Reset to next boudary
        }
        lastScheduledTemp = holiday.elem.temp; // Force new currentSetTemp when holiday over
        manHolidayTemp = holiday.elem.temp;    // Use this temp during holiday time - allows remote override by overriding this value
      }
      else // This is manually increasing the temp
      {
        currentSetTemp = currentSetTemp + SET_INTERVAL;
      }
    }
  }
  lastTriggerTimeA = t;
}

// Interrupt on B changing state
void intHandlerRotaryB()
{
  unsigned long t = micros();
  if (t - lastTriggerTimeB > DEBOUNCE_TIME)
  {
    rotaryB = digitalRead(ROTARY_B);
    // Only inc state on rising edge of B and A is off (CCW rotation)
    if (rotaryB && !rotaryA)
    {
      if (holidaySetTimer > 0) // Setting holiday time
      {
        holidayTime -= 15 * 60000; // Take 15mins off holiday time
        if (holidayTime < 0)
          holidayTime = 0;
      }
      else // This is manually decreasing the temp
      {
        currentSetTemp = currentSetTemp - SET_INTERVAL;
      }
    }
  }
  lastTriggerTimeB = t;
}

void readInputs(void)
{
  if (holidayButton.update() && holidayButton.fell())
  {
    if (holidaySetTimer == 0)
    {
      holidaySetTimer = HOLIDAY_SET_TIME;
      holidayTime = calcHolidayTime();
    }
    else
    {
      // Treat as set holiday time
      holidaySetTimer = 0;
    }
    changedState = 1;
  }
}

void resetWifi()
{
  //Drive LED orange to show resetting wifi
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  // Reset the wifi processor to reset the wifi card
  digitalWrite(WIFI_RESET_PIN, LOW);
  delay(500);
  digitalWrite(WIFI_RESET_PIN, HIGH);
  // Allow time for wifi card to restart
  delay(RECONNECT_WAIT_TIME); // Give time for wifi to connect to AP
  networkUp = true;           // Assume network is up unless shown otherwise
  networkUpTime = millis();
  lastNetworkCheckTime = networkUpTime;  
  lastMessageCheck = millis();
  //Set LED back to original setting
  switchHeat(heatOn);
}

long calcHolidayTime()
{
  uint8_t currMins = rtc.minute();
  uint8_t mins = 15 * (1 + (currMins / 15)); // Get  next 15 min boundary
  mins -= currMins;                          // Mins to next boundary
  return (mins * 60000);                     // Assign to min start time
}

void readLocalTemp()
{
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  int16_t readTemp = (int16_t)(temp_sensor.getTempCByIndex(0) * 10);
  if (readTemp > 10 && readTemp < 600)
  {
    // Temp looks sensible, use it (i.e. between 1 and 60 C)
    currentTemp = readTemp - LOCAL_TEMP_ADJUST; // Adjust for local temp reading compared to masterstation
  }
}

void addDefaultSchedule()
{
  // Add a default schedule - in mem only
  defaultSchedule.elem.day = 0;
  defaultSchedule.elem.start = 0;
  defaultSchedule.elem.end = 1440;
  defaultSchedule.elem.temp = 110;
  memcpy(&schedules[0], &defaultSchedule.raw, sizeof(SchedByElem));
  noOfSchedules = 1;
  isDefaultSchedule = true;
}

void setTempMotd(char templateStr[], char param[])
{
  snprintf(motd, MAX_MOTD_SIZE, templateStr, param);
  motdExpiryTimer = TEMP_MOTD_TIME;
  scrollPos = 0;
  changedState = 2;
}

void zeroSendBuffer()
{
  for (int i = 0; i < MAX_GET_MSG_SIZE; i++)
  {
    getMessage[i] == 0;
  }
}

bool getNextMessage()
{
  for (int i = 0; i < MAX_GET_MSG_SIZE; i++)
  {
    nextMessage[i] == 0;
  }
  // Send current status as params in request
  // Status message format = /message?s=station&rs=resend init messages&t=<temp>st=<thermostat set temp>  &r=<mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
  // GET /message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d
  int runMins = boilerRunTime / 60000;
  snprintf(nextMessage, MAX_GET_MSG_SIZE, NEXT_MESSAGE_TEMPLATE, STATION_NUMBER, resendMessages, (int)currentTemp, (int)currentSetTemp, runMins, pirStatus);
  // Count the message len
  int msglen = 0;
  for (int i = 0; i < MAX_GET_MSG_SIZE; i++)
  {
    if (nextMessage[i] == 0)
    {
      break;
    }
    msglen++;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, msglen, nextMessage);
  uint8_t msgId = sendMessage(getMessage);
  if (msgId > 0)
  {
    resendMessages = 0;
  }
  return processMessage(msgId);
}

bool getDateTime()
{
  zeroSendBuffer();
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 8, dateTime);
  uint8_t msgId = sendMessage(getMessage);
  return processMessage(msgId);
}

// bool getThermTemp() {
//   zeroSendBuffer();
//   snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 4, thermTemp);
//   uint8_t msgId = sendMessage(getMessage);
//   return processMessage(msgId);
// }
//
// bool getSetTemp() {
//   zeroSendBuffer();
//   snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, setTemp);
//   uint8_t msgId = sendMessage(getMessage);
//   return processMessage(msgId);
// }
//

bool getExtTemp()
{
  zeroSendBuffer();
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, extTempStr);
  uint8_t msgId = sendMessage(getMessage);
  return processMessage(msgId);
}

bool getMotd()
{
  zeroSendBuffer();
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 4, motdStr);
  uint8_t msgId = sendMessage(getMessage);
  return processMessage(msgId);
}

// void getHoliday() {
//   for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
//     getMessage[i] == 0;
//   }
//   snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, holiday);
//   uint8_t msgId = sendMessage(getMessage);
//   processMessage(msgId);
// }

bool processMessage(uint8_t msgId)
{
  bool msgRx = false;
  if (msgId >= 0)
  {
    networkUpTime = currentMillis;
    lastNetworkCheckTime = currentMillis;
  }
  switch (msgId)
  {
  case 0:
    // No message
    lastMessageCheck = currentMillis;
    // This is the default message that is sent if no other
    break;
  case SET_DATE_TIME_MSG:
    // Process Date time
    msgRx = true;
    setDateTime();
    break;
  case MOTD_MSG:
    // Process Date time
    msgRx = true;
    setMotd();
    break;
  case SET_THERM_TEMP_MSG:
    // Process thermometer temp
    msgRx = true;
    setThermTemp();
    // Only this and no message set the lastMessageCheck time
    // this means that next message will be requested on the next loop
    lastMessageCheck = currentMillis;
    break;
  case SET_TEMP_MSG:
    // Process set temp msg
    msgRx = true;
    setSetTemp();
    break;
  case SET_EXT_MSG:
    // Process external temp
    msgRx = true;
    setExtTemp();
    break;
  case SET_HOLIDAY_MSG:
    // Process Date time
    msgRx = true;
    setHoliday();
    break;
  case SCHEDULE_MSG:
    // Process Date time
    msgRx = true;
    setSchedule();
    break;
  case DELETE_ALL_SCHEDULES_MSG:
    msgRx = true;
    deleteAllSchedules();
    break;
  case RESET_MSG:
    resetWifi();
    resetFunc();
    break;
  }
  flickerLED();
  return msgRx;
}

void setMotd()
{
  Motd *p = (Motd *)&buff[4]; // Start of content
  motdExpiryTimer = p->expiry;
  motd[0] = '\0';
  strncat(&motd[0], p->motdStr, MAX_MOTD_SIZE);
  scrollPos = 0;
  changedState = 2;
}

void setThermTemp()
{
  Temp *tp = (Temp *)&buff[4]; // Start of content
  if (tp->temp > 10 && tp->temp < 500)
  {
    // Rx Temp looks sensible, use it
    currentSentThermTemp = tp->temp;
  }
  lastThermTempTime = currentMillis;
  boilerRunTime = getRunTime();
  changedState = 1;
}

void setSetTemp()
{
  Temp *tp = (Temp *)&buff[4]; // Start of content
  currentSetTemp = tp->temp;
  manHolidayTemp = tp->temp;
  changedState = 1;
}

void setExtTemp()
{
  SetExt *tp = (SetExt *)&buff[4]; // Start of content
  extTemp = tp->temp;
  windStr[0] = '\0';
  strncat(&windStr[0], tp->windStr, MAX_WIND_SIZE);
  changedState = 1;
}

void setDateTime()
{
  lastRTCTime = currentMillis;
  DateTimeStruct *dtp = (DateTimeStruct *)&buff[4]; // Start of content
  //  RTCLib::set(byte second, byte minute, byte hour, byte dayOfWeek, byte dayOfMonth, byte month, byte year)
  rtc.set(dtp->sec,
          dtp->min,
          dtp->hour,
          dtp->dayOfWeek,
          dtp->dayOfMonth,
          dtp->month,
          dtp->year);
  rtc.refresh();
  changedState = 1;
}

void setHoliday()
{
  HolidayUnion *dtp = (HolidayUnion *)&buff[4]; // Start of content
  //    HolidayUnion *dtp = (HolidayUnion *)&buff[4]; //Start of content
  memcpy(&holiday.raw[0], &dtp->raw[0], sizeof(HolidayStr));
  // Save in eeprom
  int cnt;
  cnt = 1 + (MAX_SCHEDULES * sizeof(SchedByElem));
  eepromWrite(cnt, 1); // Set holiday as valid
  cnt++;
  byte *hols;
  hols = &dtp->raw[0];
  eepromWrite(cnt, *hols++);
  cnt++;
  changedState = 1;
}

void deleteAllSchedules()
{
  noOfSchedules = 0;
  eepromWrite(0, noOfSchedules);
  addDefaultSchedule();
}

void setSchedule()
{
  SchedByElem *dtp = (SchedByElem *)&buff[4]; // Start of content
  // Insert schedule
  SchedByElem elem;
  SchedUnion sched;
  elem.day = dtp->day;
  elem.start = dtp->start;
  elem.end = dtp->end;
  elem.temp = dtp->temp;
  sched.elem = elem;
  if (isDefaultSchedule)
  {
    // Overwrite the default
    noOfSchedules = 0;
    isDefaultSchedule = false;
  }
  memcpy(&schedules[noOfSchedules], &sched.raw, sizeof(SchedByElem));
  writeSchedule(noOfSchedules, &sched.raw[0]);
  noOfSchedules++;
  if (motdExpiryTimer == 0)
  {
    setDefaultMotd(); // Show the number of schedules written as it is in the default motd
  }
  // Write number of schedules at start address
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

void displayState()
{
  // Display current status
  if (changedState > 1)
  {
    lcd.clear();
  }
  char tempStr[TEMP_SIZE];
  // Display Row 1 - Current time and temperature
  lcd.setCursor(0, 0);
  lcdBuff[0] = '\0';
  char runTimeStr[13]; // 10 chars on LCD + end of str
  runTimeStr[0] = '\0';
  if (holidaySetTimer == 0)
  {
    int pos = getTimeStr(&runTimeStr[0], 13);
    String currTempStr = getTempStr(currentTemp);
    currTempStr.toCharArray(tempStr, TEMP_SIZE);
    snprintf(lcdBuff, LCD_COLS + 1, "%s   %sC", runTimeStr, tempStr);
  }
  else
  {
    // Setting instant holiday time
    getFutureTime(holidayTime, &runTimeStr[0], 10);
    snprintf(lcdBuff, LCD_COLS + 1, "Off until: %s", &runTimeStr[0]);
  }
  // Serial.println(lcdBuff);
  lcd.print(lcdBuff);

  // Display row 2 - This is one of: Boiler on time + Set temp, Current date + Set Temp, Sched end or start time + Set Temp
  lcd.setCursor(0, 1);
  runTimeStr[0] = '\0';
  //    lcdBuff[0] = '\0';
  // char sp[] = " ";
  String setTempStr;
  if (rtc.second() % 2)
  {
    // Show hoiday or next sched start
    if (onHoliday && holidayTime == 0)
    {
      // On scheduled holiday
      strncat(runTimeStr, "On hols!!!", 11);
    }
    else if (holidayTime > 0)
    {
      // Currently off due to instant away time set - show when on
      char on[5];
      getFutureTime(holidayTime, &on[0], 5);
      snprintf(&runTimeStr[0], 11, "ON @ %s ", &on[0]);
      // strncat(&runTimeStr[9], sp, 1);
    }
    else
    {
      // Show next schedule temp and time
      char nextTime[] = "HHMM";
      getHoursMins(nextSched.elem.start, &nextTime[0]);
      setTempStr = getTempStr(nextSched.elem.temp);
      setTempStr.toCharArray(tempStr, TEMP_SIZE);
      // strncat(&on[0], &tempStr[0], 4);
      // strncat(&runTimeStr[0], &on[0], strlen(on));
      snprintf(&runTimeStr[0], 11, "%sC@%s", tempStr, nextTime);
    }
  }
  else
  {
    // Show date
    getDateStr(&runTimeStr[0], 11);
    // char * dt = &(getDateStr())[0];
    // strncat(&runTimeStr[0], dt, strlen(dt));
  }
  setTempStr = getTempStr(currentSetTemp);
  setTempStr.toCharArray(tempStr, TEMP_SIZE);
  snprintf(&lcdBuff[0], LCD_COLS + 1, "%s Set:%sC", runTimeStr, tempStr);
  //    lcdBuff[LCD_COLS] = '\0';
  // Serial.println(lcdBuff);
  // delay(100);
  lcd.print(lcdBuff);
  // lcd.print(String(runTimeStr) + " Set:" + tempStr);

  lcd.setCursor(0, 2);
  uint8_t len = 0;
  uint8_t wlen = strlen(windStr);
  char run[] = "Run: MM:SS ";
  if (wlen != 0 && boilerRunTime != 0)
  {
    if (rtc.second() % 2)
    {
      // display wind speed
      len = snprintf(lcdBuff, MAX_WIND_SIZE, "%s", windStr);
    }
    else
    {
      // Boiler is on - display how long for on row 2
      getMinSec(boilerRunTime, &run[5]);
      // strncat(runTimeStr, &run[0], strlen(run));
      len = snprintf(lcdBuff, 11, "%s", run);
      // strncat(&runTimeStr[8], sp, 1);
      //    Serial.println("Runtime:" + String(runTime) + " 3digi: " + String(threeDigit) + " runTimeStr: " + runTimeStr + " big: " + String(bigChangeOfState));
    }
  }
  else if (wlen != 0)
  {
    // display wind speed
    len = snprintf(lcdBuff, MAX_WIND_SIZE, "%s", windStr);
  }
  else if (boilerRunTime != 0)
  {
    getMinSec(boilerRunTime, &run[5]);
    len = snprintf(lcdBuff, 11, "%s", run);
  }
  else
  {
    char bs[] = "Heat : %s";
    char on[] = "ON  ";
    char off[] = "OFF ";
    if (heatOn)
    {
      len = snprintf(lcdBuff, MAX_WIND_SIZE, bs, on);
    }
    else
    {
      len = snprintf(lcdBuff, MAX_WIND_SIZE, bs, off);
    }
  }
  // Pad with spaces
  lcdBuff[len] = '\0';
  uint8_t iLen = MAX_WIND_SIZE - 1;
  while (strlen(lcdBuff) < iLen)
  {
    lcdBuff[len] = ' ';
    len++;
    lcdBuff[len] = '\0';
  }
  // int i;
  // i = strlen(boilerStatStr);
  // while (strlen(lcdBuff) < MAX_WIND_SIZE - 1) {
  //   lcdBuff[len] = ' ';
  //   len++;
  //   lcdBuff[len] = '\0';
  // }
  String extTempStr = getTempStr(extTemp);
  extTempStr.toCharArray(tempStr, TEMP_SIZE);
  sprintf(&lcdBuff[len], "Ext:%sC", tempStr);

  // Serial.println(lcdBuff);
  // delay(100);
  lcd.print(lcdBuff);
  // lcd.print(String(boilerStatStr) + " Ext:" + extTempStr);
  if (strlen(motd) <= LCD_COLS)
  {
    lcd.setCursor(0, 3);
    // Serial.println(motd);
    // delay(100);
    lcd.print(motd); // Only print here if motd fits, otherwise needs to scroll
  }
}

void scrollLcd()
{
  int charsToCopy = LCD_COLS;
  int blankChars = 0;
  int motdLen = strlen(motd);
  if (scrollPos + LCD_COLS > motdLen - 1)
  {
    charsToCopy = motdLen - scrollPos;
    if (charsToCopy == 0)
    {
      // Right at the end
      charsToCopy = LCD_COLS;
      scrollPos = 0;
    }
    else
    {
      blankChars = LCD_COLS - charsToCopy;
    }
  }
  strncpy(&lcdBuff[0], &motd[scrollPos], charsToCopy);
  if (blankChars > 0)
  {
    // Add blanks to the end of the message for a full screen
    memset(&lcdBuff[charsToCopy], ' ', blankChars);
    scrollPos = 0;
  }
  else
  {
    // advance to the next screen
    int newPos = scrollPos;
    int lastPos;
    int lastSpace = strrchr(&motd[0], ' ');
    // Find last word to use as first in next msg
    while (newPos < lastSpace && newPos <= scrollPos + LCD_COLS && newPos < motdLen)
    {
      lastPos = newPos;
      do
      {
        newPos++;
      } while (motd[newPos] != ' ' && newPos < lastSpace && newPos < motdLen);
    }
    if (newPos != scrollPos + LCD_COLS)
    {
      newPos = lastPos + 1;
    }
    if (newPos > motdLen)
      newPos = 0;
    scrollPos = newPos;
  }
  lcdBuff[LCD_COLS] = '\0';
  lcd.setCursor(0, 3);
  lcd.print(lcdBuff);
  // Serial.println(lcdBuff);
}

boolean checkBackLight()
{
  if (backLightTimer > 0)
  {
    if (backLightTimer > loopDelta)
    {
      backLightTimer -= loopDelta;
    }
    else
    {
      backLightTimer = 0;
    }
  }
  // PIR set to repeat trigger so if output is high then set backLightTimer to BACKLIGHT_TIME
  // This keeps the backlight on LCD until people have left the area
  int pirValue = analogRead(PIR_PIN);
  if (pirValue >= ANALOGUE_HIGH)
  {
    if (backLightTimer == 0)
    {
      // Force a message check, which flags the PIR status change to the masterstation
      lastMessageCheck = 0;
    }
    backLightTimer = BACKLIGHT_TIME;
  }
  return (backLightTimer > 0);
}

unsigned long getRunTime()
{
  unsigned long runTime = 0;
  if (heatOn)
  {
    runTime = calcRunTime(currentTemp, currentSetTemp + HYSTERSIS, extTemp);
  }
  return runTime;
}

void writeSchedule(int schedNum, byte *sched)
{
  int cnt;
  cnt = 1 + (schedNum * sizeof(SchedByElem));
  for (int j = 0; j < sizeof(SchedByElem); j++)
  {
    eepromWrite(cnt, *sched++);
    cnt++;
  }
}

void eepromWrite(unsigned int eeaddress, byte data)
{
  int rdata = data;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.write(rdata);
  delay(5);
  Wire.endTransmission();
}

byte eepromRead(unsigned int eeaddress)
{
  byte rdata = 0xFF;
  Wire.beginTransmission(EEPROM_I2C_ADDR);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
  Wire.requestFrom(EEPROM_I2C_ADDR, 1);
  if (Wire.available())
    rdata = Wire.read();
  return rdata;
}

// Return the schedule ptr to the current schedule. or the next sched if asked for
void getSetPoint(SchedUnion *schedule, uint16_t mins, int currDay, bool nextSched)
{
  // Find matching schedules and take one with the most specific day that matches
  // if next sched is true look for the schedule with a start time closest to the current
  union SchedUnion sched;
  int priority = 0;
  memcpy(&schedule->raw, &defaultSchedule.raw, sizeof(SchedByElem));
  uint16_t nextMins = 1440;
  for (int i = 0; i < noOfSchedules; i++)
  {
    memcpy(&sched.raw, &schedules[i], sizeof(SchedByElem));
    //    Serial.println("Sched: " + sched.elem.day + String(" ") + sched.elem.temp);
    if (!nextSched && sched.elem.start == 0000 && sched.elem.end == 0000 && sched.elem.day == 0 && priority == 0)
    {
      // This sched matches everything
      memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
    }
    else if ((!nextSched && sched.elem.start <= mins && sched.elem.end > mins) ||
             (nextSched && sched.elem.start > mins && sched.elem.start < nextMins))
    {
      if (sched.elem.day == 0 && priority <= 1)
      {
        // All days match and not found higher
        priority = 1;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
      if (sched.elem.day == 0x200 && (currDay == 6 || currDay == 7) && priority <= 2)
      {
        // Its the weekend and not found higher
        priority = 2;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
      if (sched.elem.day == 0x100 && (currDay >= 1 && currDay <= 5) && priority <= 2)
      {
        // Its a weekday and not found higher
        priority = 2;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
      if (sched.elem.day == currDay && priority <= 3)
      {
        // Found a specific day - cant get higher
        priority = 3;
        nextMins = sched.elem.start;
        memcpy(&schedule->raw, &schedules[i], sizeof(SchedByElem));
      }
    }
  }
}

boolean checkOnHoliday()
{
  boolean afterStart = false;
  boolean beforeEnd = false;
  if (holidayTime > 0)
  {
    // On manual holiday
    afterStart = true;
    beforeEnd = true;
    holiday.elem.temp = 100; // Set to default holiday temp of 10C
  }
  else if (holiday.elem.valid == 1)
  {
    HolidayDateStr *hols;
    hols = &(holiday.elem.startDate);
    if (rtc.year() > hols->year)
    {
      afterStart = true;
    }
    else if (rtc.year() == hols->year)
    {
      if (rtc.month() > hols->month)
      {
        afterStart = true;
      }
      else if (rtc.month() == hols->month)
      {
        if (rtc.day() > hols->dayOfMonth)
        {
          afterStart = true;
        }
        else if (rtc.day() == hols->dayOfMonth)
        {
          if (rtc.hour() >= hols->hour)
          {
            afterStart = true;
          }
        }
      }
    }
    hols = &(holiday.elem.endDate);
    if (rtc.year() < hols->year)
    {
      beforeEnd = true;
    }
    else if (rtc.year() == hols->year)
    {
      if (rtc.month() < hols->month)
      {
        beforeEnd = true;
      }
      else if (rtc.month() == hols->month)
      {
        if (rtc.day() < hols->dayOfMonth)
        {
          beforeEnd = true;
        }
        else if (rtc.day() == hols->dayOfMonth)
        {
          if (rtc.hour() < hols->hour)
          {
            beforeEnd = true;
          }
        }
      }
    }
    //    Serial.println("after start:" + String(afterStart) + " before end:" + String(beforeEnd));
    if (!beforeEnd)
    {
      // Holiday is over - remove it
      holiday.elem.valid = 0;
      int cnt;
      cnt = 1 + (MAX_SCHEDULES * sizeof(SchedByElem));
      eepromWrite(cnt, 0); // Set holiday as invalid
    }
  }
  return (afterStart && beforeEnd);
}

// Calculate the number of ms to reach set temp
unsigned long calcRunTime(int16_t tempNow, int16_t tempSet, int16_t tempExt)
{
  unsigned long noMs = 0;
  if (tempNow < tempSet)
  {
    int16_t deg = degPerHour;
    if (tempExt != 1000 && tempExt < tempNow)
    {
      // Reduce based on outside temp
      int16_t adj = (tempNow - tempExt) / extAdjustment;
      if (adj > 3)
        adj = 3;
      deg -= adj;
    }
    float noSecs = (tempSet - tempNow) * 3600.0 / deg;
    noMs = (unsigned long)(noSecs * 1000);
    //    Serial.println("Set: " + String(tempSet,1) + " Now: " + String(tempNow, 1) + " Degph: " + deg + " noMs: " + noMs);
    //    Serial.println(getMinSec(noMs));
  }
  return noMs;
}

// Calculate a time in the future using current time + delta in ms
void getFutureTime(long tms, char *charBuf, uint8_t len)
{
  uint8_t tmins = (uint8_t)(tms / 60000UL);
  uint8_t thours = (uint8_t)(tmins / 60);
  tmins -= (thours * 60);
  uint8_t hours = rtc.hour() + thours;
  uint8_t mins = rtc.minute() + tmins;
  if (mins >= 60)
  {
    hours++;
    mins -= 60;
  }
  if (hours >= 24)
  {
    hours -= 24;
  }
  snprintf(charBuf, len, "%02d%02d  ", hours, mins);
}

void getHoursMins(unsigned long tmins, char *charBuf)
{
  unsigned long hours = tmins / 60;
  unsigned long mins = tmins % 60;
  sprintf(charBuf, "%02d", hours);
  sprintf(charBuf + 2, "%02d", mins);
}

void getMinSec(unsigned long timeMs, char *charBuf)
{
  unsigned long tsecs = timeMs / 1000;
  unsigned long mins = tsecs / 60;
  unsigned long secs = tsecs % 60;
  if (mins < 60)
  {
    sprintf(charBuf, "%02d:", mins);
    sprintf(charBuf + 3, "%02d", secs);
  }
  else
  {
    unsigned long hours = mins / 60;
    unsigned long remMins = mins % 60;
    sprintf(charBuf, "%02d:", hours);
    sprintf(charBuf + 3, "%02d", remMins);
  }
}

String getTempStr(int16_t temp)
{
  String tempStr;
  if (temp == 1000)
  {
    tempStr = "??.?";
  }
  else if (temp == 0)
  {
    tempStr = " 0.0";
  }
  else if (temp > -100) // For temps more than -10 get "-1.0C" over 10 get "10.0C"
  {
    tempStr = String((float)(temp / 10.0), 1); // Only way to generate a float str in Arduino
  }
  else // For neg temps less than -10, drop the decimal place and pad with a space (" -10C")
  {
    tempStr = String((float)(temp / 10.0), 0); // Only way to generate a float str in Arduino
    tempStr = " " + tempStr;
  }

  if (temp < 100 && temp >= 10) // Add a leading 0 for temps between 1 and 10: "01.0C"
  {
    tempStr = "0" + tempStr;
  }
  else if (temp > 0 && temp < 10) // Need to pad with a space if temp between 0 and 1: " 0.5C"
  {
    tempStr = " " + tempStr;
  }
  return tempStr;
}

int getTimeStr(char *ptr, int len)
{
  // int dayofweek = 1;
  // int h = 12;
  // int m = 13;
  // int s = 31;
  // return sprintf(ptr, "%s %02d:%02d:%02d", dayNames[dayofweek - 1], h, m, s );
  return snprintf(ptr, len, "%s %02d:%02d:%02d", dayNames[rtc.dayOfWeek() - 1], rtc.hour(), rtc.minute(), rtc.second());
}

int getDateStr(char *ptr, int len)
{
  // int d = 24;
  // int m = 6;
  // int y = 2022;
  // return sprintf(ptr, "%02d %s %02d ", d, monNames[m], y);
  return snprintf(ptr, len, "%02d %s %02d ", rtc.day(), monNames[rtc.month() - 1], rtc.year());
}

void switchHeat(boolean on)
{
  if (on)
  {
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    digitalWrite(RELAY, LOW);
    heatOn = true;
  }
  else
  {
    digitalWrite(RED_LED, HIGH);
    digitalWrite(GREEN_LED, LOW);
    digitalWrite(RELAY, HIGH);
    heatOn = false;
  }
}

void flickerLED()
{
  int ledToFlick;
  if (heatOn)
  {
    ledToFlick = RED_LED;
  }
  else
  {
    ledToFlick = GREEN_LED;
  }
  digitalWrite(ledToFlick, HIGH);
  delay(50);
  digitalWrite(ledToFlick, LOW);
}

void setDefaultMotd()
{
  // Set up default motd string;
  motd[0] = '\0';
  strncat(motd, &defaultMotd[0], MAX_MOTD_SIZE);
  sprintf(&motd[strlen(defaultMotd)], "%d", noOfSchedules);
}

uint8_t checkNetworkUp(bool statusOnly)
{
  int8_t networkStat = 2;
  uint8_t retries = 0;
  while (retries < 2 && networkStat == 2)
  {
    //  Check network up
    drainSerial();
    //    debugSerial.println("Check net: ");
    if (statusOnly)
    {
      wifiSerial.println("*N");
    }
    else
    {
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

uint8_t sendMessage(char msg[])
{
  uint8_t retries = 0;
  int8_t msgId = -1; // Signifies message failed
  drainSerial();
  wifiSerial.println(msg);
  wifiSerial.println();
  int16_t msgLen = waitForGetResponse();
  if (msgLen > 0)
  {
    // Got HTTP response - extract content part
    Message *msgp = (Message *)&buff[0];
    msgId = msgp->id;
    uint16_t rxCrc = msgp->crc;
    msgp->crc = 0;
    crc_t crc = crc_calculate(&buff[0], msgp->len);
    if (crc != rxCrc)
    {
      setTempMotd(MESSAGE_FAIL, "CRC failed");
      msgId = -1;
      rxInFail = true;
    }
    else if (rxInFail)
    {
      rxInFail = false;
      setTempMotd(SERVER_STATUS, "Now Up");
    }
  }
  return msgId;
}

// Return:
// 0 - Network up / Connected
// 1 - Network down and got a response
// 2 - No response from Wifi board
int8_t getStatusResponse(bool upOnly)
{
  unsigned long start = millis();
  uint16_t bufPos = 0;
  unsigned long waitTime = 0;
  bool gotResponse = false;
  int8_t success = -1;
  while (waitTime < MAX_STATUS_TIME && !gotResponse)
  {
    uint16_t len = receiveData(bufPos);
    if (len > 0)
    {
      bufPos += len;
      if (success == -1)
      {
        success = buff[0] - 48;
        shiftBuffDown(1, bufPos); // Remove status byte
        bufPos--;
        if (success != 0 && success != 1)
        {
          // wrong status byte, try again
          success = -1;
          // Serial.print(buff[0], HEX);
          // Serial.print(" ");
        }
      }
      if (success != -1)
      {
        if (upOnly)
        {
          gotResponse = true;
        }
        else
        {
          // Look for EOL
          int16_t pos = findStringInBuff(buff, EOL, sizeof(EOL) - 1, bufPos);
          if (pos != -1)
          {
            gotResponse = true;
            buff[pos] = '\0';
          }
        }
      }
    }
    if (!gotResponse)
    {
      // Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - start;
  }
  if (!gotResponse)
  {
    //    Serial.print(" Failed to get net status, len: ");
    //    Serial.println(bufPos);
    //    printBuff(bufPos);
    success = 2; // Indicates response timed out
  }
  return success;
}

int16_t waitForGetResponse()
{
  unsigned long start = millis();
  unsigned long lastByteTime = start;
  uint16_t bufPos = 0;
  unsigned long waitTime = 0;
  unsigned long totalWaitTime = 0;
  int16_t msgLen = 0;
  bool gotMsg = false;
  bool fail = false;
  int8_t success = -1;
  while (waitTime < MAX_RESPONSE_TIME && !gotMsg)
  {
    uint16_t len = receiveData(bufPos);
    if (len > 0)
    {
      lastByteTime = millis();
      bufPos += len;
      if (success == -1)
      {
        success = buff[0] - 48;
        shiftBuffDown(1, bufPos); // Remove status byte
        bufPos--;
        if (success < 0 || success > 2)
        {
          // wrong status byte, try again
          success = -1;
        }
      }
      if (success > 0)
      {
        // Fail -> Look for EOL
        //         debugSerial.print("Send Fail: got:");
        //         debugSerial.print(bufPos);
        int16_t pos = findStringInBuff(buff, EOL, sizeof(EOL) - 1, bufPos);
        //        debugSerial.print(" EOL:");
        //        debugSerial.println(pos);
        if (pos != -1)
        {
          buff[pos] = '\0';
          gotMsg = true;
        }
      }
      else if (success == 0)
      {
        // Valid message response - Look for message len
        if (bufPos > 2)
        {
          msgLen = buff[1];
          if (bufPos >= msgLen)
          {
            gotMsg = true;
          }
        }
      }
    }
    if (!gotMsg)
    {
      // Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - lastByteTime; // Timeout if not received a byte for max wait time
  }
  if (gotMsg && msgLen == 0)
  {
    // Didnt receive a valid message - buffer contains error message from wifi card
    setTempMotd(LITERAL_STATUS, (char *)&buff[0]);
    fail = true;
  }
  if (waitTime >= MAX_RESPONSE_TIME && !gotMsg)
  {
    //    debugSerial.print("Rx timeout:");
    //    debugSerial.print(msgLen);
    //    debugSerial.print(" Got:");
    //    debugSerial.print(bufPos);
    //    printBuff(bufPos);
    snprintf(&motd[0], MAX_MOTD_SIZE, "Msg TO Rx %d of %d", bufPos, msgLen);
    // Serial.println(motd);
    motdExpiryTimer = TEMP_MOTD_TIME;
    scrollPos = 0;
    changedState = 1;
    fail = true;
  }
  if (fail)
  {
    lastMessageCheck = currentMillis; // reset message check timer
    rxInFail = true;
    if (networkUp)
    {
      networkUp = false;
    }
  }

  return msgLen;
}

void shiftBuffDown(uint16_t pos, uint16_t maxPos)
{
  //  debugSerial.print("Downshift by ");
  //  debugSerial.println(pos);
  //  printBuff(MAX_MESSAGE_SIZE);
  uint16_t i = 0;
  for (; pos < maxPos; pos++, i++)
  {
    buff[i] = buff[pos];
  }
  //  debugSerial.println("After:");
  //  printBuff(MAX_MESSAGE_SIZE);
}

uint16_t receiveData(uint16_t startPosition)
{
  uint16_t len = 0;
  uint16_t pos = startPosition;
  while (wifiSerial.available() > 0 && pos < BUFF_SIZE)
  {
    uint8_t b = wifiSerial.read();
    buff[pos++] = b;
    len++;
    // debugSerial.print(b, HEX);
  }
  return len;
}

uint16_t findStringInBuff(uint8_t bf[], char str[], uint8_t strLen, uint16_t maxPos)
{
  //  debugSerial.print("Looking for ");
  //  debugSerial.print(str);
  //  debugSerial.print(" Size:");
  //  debugSerial.print(strLen);
  //  debugSerial.print(" MaxPos:");
  //  debugSerial.println(maxPos);
  //  printBuff(maxPos);
  uint16_t startPos = -1;
  for (int i = 0; (i + strLen) < maxPos; i++)
  {
    if (bf[i] == str[0])
    {
      startPos = i;
      uint16_t strPos = 0;
      uint16_t endStr = i + strLen;
      int j = i;
      for (; j < endStr && j < maxPos; j++)
      {
        if (bf[j] != str[strPos++])
        {
          break;
        }
      }
      if (j >= endStr)
      { // All chars matched
        break;
      }
      else
      {
        startPos = -1;
      }
    }
  }
  return startPos;
}

void drainSerial()
{
  delay(50);
  int total = 0;
  while (uint16_t len = receiveData(0) > 0)
  {
    total += len;
    delay(10);
  }
  //  if (total > 0) {
  //    debugSerial.print("Drained: ");
  //    debugSerial.println(total);
  //  }
  resetBuffer();
}

void resetBuffer()
{
  // Reset receive buffer
  for (uint32_t i = 0; i < BUFF_SIZE; i++)
  {
    buff[i] = 0;
  }
}
