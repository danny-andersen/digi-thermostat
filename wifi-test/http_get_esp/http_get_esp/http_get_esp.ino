#include <SoftwareSerial.h>
#include <AceCRC.h>

using namespace ace_crc::crc16ccitt_nibble;

// Your board <-> ESP_AT baud rate:
#define ESP_AT_BAUD       9600

SoftwareSerial wifi(10, 11); // RX, TX

#define debugSerial Serial
#define wifiSerial wifi  //Change this to Serial if a problem, i.e. no interrupt support

#define MAX_RESPONSE_TIME 2000 // millis to wait for a GET response
#define MAX_STATUS_TIME 500 // millis to wait for a status response
#define MAX_GET_MSG_SIZE 60 //Max size of dynamic get msg with params
#define MAX_MOTD_SIZE 128
#define MAX_MESSAGE_SIZE MAX_MOTD_SIZE
#define BUFF_SIZE MAX_MESSAGE_SIZE+6

uint8_t buff[BUFF_SIZE];
char nextMessage[MAX_GET_MSG_SIZE];
char getMessage[MAX_GET_MSG_SIZE];
char motd[MAX_MOTD_SIZE];
char EOL[] = "EOL";

char NEXT_MESSAGE_TEMPLATE[] = "message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d";
char GET_TEMPLATE[] = "*G%02d%s";
char dateTime[] = "datetime";
char thermTemp[] = "temp";
char setTemp[] = "settemp";
char extTemp[] = "exttemp";
char motdStr[] = "motd";
char holiday[] = "holiday";
//Status message format = /message?t=<temp>st=<thermostat set temp>&r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>

int16_t currentSetTemp = 100;
int16_t currentTemp = 100;
unsigned long networkDownTime = 0;
bool gotInitMessages = false;
uint8_t resendMessages = 1;
bool networkUp = false;

#define LOOP_DELAY 5000
#define RECONNECT_WAIT_TIME 10000

#define MAX_WIND_SIZE 12

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

struct Message {
  uint8_t id;
  uint8_t len;
  uint16_t crc;
};

struct DateTimeStruct {
  uint8_t sec;
  uint8_t min;
  uint8_t hour;
  uint8_t dayOfWeek;
  uint8_t dayOfMonth;
  uint8_t month;
  uint16_t year;
}; 

struct Motd {
    uint32_t expiry;
    char motdStr[MAX_MOTD_SIZE];
};

struct Temp {
  int16_t temp;
};

struct SetExt {
  int16_t temp;
  char windStr[MAX_WIND_SIZE];
};

struct HolidayDateStr {
      uint8_t hour;
      uint8_t dayOfMonth;
      uint8_t month;
      uint8_t year;
};

struct HolidayStr {
        HolidayDateStr startDate;
        HolidayDateStr endDate;
        int16_t temp;
};

struct SchedByElem {
    uint16_t day; //= "0" for every day, "0x0100" for Weekday (Mon - Fri), 
                  //"0x0200" for Weekend (Sat, Sun), 1 - Sunday, 2 - Monday, 3 - Tuesday,....7 - Saturday
    uint16_t start; //Start minute (0 - 1440) 
    uint16_t end; //End time minute (0 - 1440)
    int16_t temp; //Set temperature tenths of C, 180 = 18.0C
};

int cnt = 0;

void setup(void)
{
  debugSerial.begin(9600);
//  while (!debugSerial); // wait for serial port to connect
  debugSerial.println("St:");
  wifiSerial.begin(ESP_AT_BAUD);
  while (!wifiSerial);
  delay(RECONNECT_WAIT_TIME); //wait for wifi to connect
  drainSerial(); //Remove any wifi start up messages

//  Serial.println("Done Setup");

//  debugSerial.println("Check Connect options:");
//  wifiSerial.println("AT+CIPSTART=?");
//  while (!wifiSerial.available());
//  debugResponse();

}

void loop(void)
{
  if (networkUp) {
    while (getNextMessage());
  }
  if (!networkUp) {
      uint8_t stat = checkNetworkUp(false);
      if (stat == 0) {
        networkUp = true;
      }
      if (stat == 1) {
        //Got full status report
        debugSerial.print("Network back up. New motd:");
        snprintf(motd, MAX_MOTD_SIZE, (char *)buff);
        debugSerial.println(motd);
      } else if (stat == 2) {
        debugSerial.print("Network down. New motd:");
        snprintf(motd, MAX_MOTD_SIZE, (char *)"No response from wifi card");
        debugSerial.println(motd);
    }
  }
//    //Cycle between messages
//    switch (cnt++) {
//      case 0:
//        getThermTemp();
//        break;
//      case 1:
//        getSetTemp();
//        break;
//      case 2:
//        getExtTemp();
//        break;
//      case 3:
//        getMotd();
//        break;
//      case 4:
//        getHoliday();
//        break;
//      case 5:
//        getDateTime();
//        break;
//      case 6:
//        getNextMessage();
//        break;
//    }
//    if (cnt == 7) cnt = 0;
//  } 
  delay(LOOP_DELAY);
}

bool getNextMessage() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    nextMessage[i] == 0;
  }
  //Send current status as params in request
  //Status message format = /message?s=station&rs=resend init messages&t=<temp>st=<thermostat set temp>  &r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
  //GET /message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d
  snprintf(nextMessage, MAX_GET_MSG_SIZE, NEXT_MESSAGE_TEMPLATE, 1, resendMessages, (int)currentTemp, (int)currentSetTemp, 3, 1);
  //Count the message len
  int msglen = 0;
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
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

void getDateTime() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 8, dateTime);
  uint8_t msgId = sendMessage(getMessage);
  processMessage(msgId);
}

void getThermTemp() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 4, thermTemp);
  uint8_t msgId = sendMessage(getMessage);
  processMessage(msgId);
}

void getSetTemp() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, setTemp);
  uint8_t msgId = sendMessage(getMessage);
  processMessage(msgId);
}

void getExtTemp() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, extTemp);
  uint8_t msgId = sendMessage(getMessage);
  processMessage(msgId);
}

void getMotd() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 4, motdStr);
  uint8_t msgId = sendMessage(getMessage);
  processMessage(msgId);
}

void getHoliday() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_TEMPLATE, 7, holiday);
  uint8_t msgId = sendMessage(getMessage);
  processMessage(msgId);
}

bool processMessage(int8_t msgId) {
  bool anotherMsg = false;
  switch (msgId) {
   case 0:
    //No message
    debugSerial.println("No msg");
    break;
   case SET_DATE_TIME_MSG:
    //Process Date time
    setDateTime();
    anotherMsg = true;
    break;
   case MOTD_MSG:
    //Process Date time
    setMotd();
    anotherMsg = true;
    break;
   case SET_THERM_TEMP_MSG:
    //Process thermometer temp
    setThermTemp();
    break;
   case SET_TEMP_MSG:
    //Process set temp msg
    setSetTemp();
    anotherMsg = true;
    break;
   case SET_EXT_MSG:
    //Process external temp
    setExtTemp();
    anotherMsg = true;
    break;
   case SET_HOLIDAY_MSG:
    //Process Date time
    setHoliday();
    anotherMsg = true;
    break;
   case SCHEDULE_MSG:
    //Process Date time
    setSchedule();
    anotherMsg = true;
    break;
   case DELETE_ALL_SCHEDULES_MSG:
    deleteAllSchedules();
    anotherMsg = true;
    break;
   default:
    networkUp = false;
    debugSerial.println("Failed to Rx message");
  }
  return anotherMsg;
}

void setMotd() {
//    printBuff(sizeof(Motd)+4);
    Motd *p = (Motd *)&buff[4]; //Start of content
    debugSerial.print("Motd: ");
    debugSerial.println(p->motdStr);
    debugSerial.print("Exp: ");
    debugSerial.println(p->expiry);
}

void setThermTemp() {
    Temp *tp = (Temp *)&buff[4]; //Start of content
    debugSerial.print("Rx therm temp: ");
    debugSerial.println(tp->temp);
    currentTemp = tp->temp;
}

void setSetTemp() {
    Temp *tp = (Temp *)&buff[4]; //Start of content
    debugSerial.print("Rx set temp: ");
    debugSerial.println(tp->temp);
    currentSetTemp = tp->temp;
}

void setExtTemp() {
    SetExt *tp = (SetExt *)&buff[4]; //Start of content
    debugSerial.print("Rx ext temp: ");
    debugSerial.println(tp->temp);
    debugSerial.print("Wind: ");
    debugSerial.println(tp->windStr);
}

void setDateTime() {
    DateTimeStruct *dtp = (DateTimeStruct *)&buff[4]; //Start of content
    debugSerial.print("Time: ");
    debugSerial.print(dtp->hour);
    debugSerial.print(":");
    debugSerial.print(dtp->min);
    debugSerial.print(":");
    debugSerial.println(dtp->sec);
    debugSerial.print("Date: ");
    debugSerial.print(dtp->dayOfWeek);
    debugSerial.print(" ");
    debugSerial.print(dtp->dayOfMonth);
    debugSerial.print("-");
    debugSerial.print(dtp->month);
    debugSerial.print("-");
    debugSerial.println(dtp->year);
}

void setHoliday() {
    HolidayStr *dtp = (HolidayStr *)&buff[4]; //Start of content
    debugSerial.print("Holiday start: "); 
    debugSerial.print(dtp->startDate.dayOfMonth);
    debugSerial.print("-");
    debugSerial.print(dtp->startDate.month);
    debugSerial.print("-");
    debugSerial.print(dtp->startDate.year);
    debugSerial.print(":");
    debugSerial.println(dtp->startDate.hour);
    debugSerial.print("Holiday End: "); 
    debugSerial.print(dtp->endDate.dayOfMonth);
    debugSerial.print("-");
    debugSerial.print(dtp->endDate.month);
    debugSerial.print("-");
    debugSerial.print(dtp->endDate.year);
    debugSerial.print(":");
    debugSerial.println(dtp->endDate.hour);
    debugSerial.print("Temp: ");
    debugSerial.println(dtp->temp);
}

void deleteAllSchedules() {
    debugSerial.println("Del schedules..."); 
}

void setSchedule() {
    SchedByElem *dtp = (SchedByElem *)&buff[4]; //Start of content
    debugSerial.print("Sch Day: "); 
    debugSerial.print(dtp->day);
    debugSerial.print(" St: ");
    debugSerial.print(dtp->start);
    debugSerial.print(" E: ");
    debugSerial.print(dtp->end);
    debugSerial.print(" T:");
    debugSerial.println(dtp->temp);
}

uint8_t checkNetworkUp(bool statusOnly) {
  int8_t networkStat = 2;
  uint8_t retries = 0;
  while (retries < 2 && networkStat == 2) {
    //  Check network up
    drainSerial();
//    debugSerial.println("Check net: ");
    if (statusOnly){
      wifiSerial.println("*N");
    } else {
      wifiSerial.println("*S");
    }
    wifiSerial.println();
    networkStat = getStatusResponse(statusOnly);
    retries++;
  }
  if (networkStat != 0) {
    debugSerial.print("Net resp: ");
    debugSerial.println(networkStat);
    debugSerial.print("Buff:");
    printBuff(60);
  }
  return networkStat;
}

uint8_t sendMessage(char msg[]) {
  uint8_t retries = 0;
  int8_t msgId = -1; //Signifies message failed
  while (retries < 2 && msgId == -1) {
    drainSerial();
    wifiSerial.println(msg);
    wifiSerial.println();
    uint16_t msgLen = waitForGetResponse();
    debugSerial.print("Sent:");
    debugSerial.println(msg);
    if (msgLen > 0) {
  //    debugSerial.print("Rx resp:");
  //    debugSerial.println(msgLen);
  //    printBuff(msgLen);
       
      //Got HTTP response - extract content part
      Message *msgp = (Message *)&buff[0];
      msgId = msgp->id;
      uint16_t rxCrc = msgp->crc;
      msgp->crc = 0;
      crc_t crc = crc_calculate(&buff[0], msgp->len);
      if (crc != rxCrc) {
        debugSerial.println("CRC Failed");
        printBuff(msgp->len);
        msgId = -1;
      }
    } else if (msgLen == -1 ) {
      debugSerial.println("Rx TIMEOUT");
    } else {
      debugSerial.print("Rx fail: Motd: ");
      snprintf(motd, MAX_MOTD_SIZE, (char *)buff);
      debugSerial.println(motd);
      break;
    }
    retries++;
  }
  return msgId;
}

void debugResponse() {
  if (wifiSerial.available() > 0) {
    debugSerial.print("Resp: ");
    debugSerial.println(wifiSerial.readString());
  } else {
    debugSerial.println("No resp");
  }
}

void printBuff(uint32_t pos) {
    debugSerial.print("Hex:");
    for (uint32_t i = 0; i<pos && i<MAX_MESSAGE_SIZE; i++) {
      debugSerial.print((uint8_t)buff[i], HEX);
      debugSerial.print(',');
    }
    debugSerial.print("\nText:");
    for (uint32_t i = 0; i<pos && i<MAX_MESSAGE_SIZE; i++) {
      debugSerial.print((char) buff[i]);
    }
    debugSerial.println();
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
  for (uint32_t i = 0; i<BUFF_SIZE; i++) {
    buff[i] = 0;
  }
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
          Serial.print(buff[0], HEX);
          Serial.print(" ");
        }
      }
      if (success != -1) {
        if (upOnly) {
          gotResponse = true;
        } else {
          //Look for EOL
          int16_t pos = findStringInBuff(buff, EOL, sizeof(EOL)-1, bufPos);
          if (pos != -1) {
            gotResponse = true;
  //          debugSerial.print("End of status text: ");
  //          debugSerial.println(pos);
  //          printBuff(40);
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
    Serial.print(" Failed to get net status, len: ");
    Serial.println(bufPos);
    printBuff(bufPos);
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
    Serial.println(motd);
    msgLen = -1;
  }
  return msgLen;
}

void shiftBuffDown(uint16_t pos, uint16_t maxPos) {
//  debugSerial.print("Downshift by ");
//  debugSerial.println(pos);
//  printBuff(MAX_MESSAGE_SIZE);
  uint16_t i=0;
  for (; pos<maxPos; pos++,i++) {
    buff[i] = buff[pos];
  }
//  debugSerial.println("After:");
//  printBuff(MAX_MESSAGE_SIZE);
}

uint16_t receiveData(uint16_t startPosition){
  uint16_t len = 0;
  uint16_t pos = startPosition;
  while(wifiSerial.available() > 0 && pos < BUFF_SIZE) {
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
  for (int i = 0; (i+strLen)<maxPos; i++) {
    if (bf[i] == str[0]) {
      startPos = i;
      uint16_t strPos = 0;
      uint16_t endStr = i+strLen;
      int j = i;
      for (;j < endStr && j<maxPos; j++) {
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
