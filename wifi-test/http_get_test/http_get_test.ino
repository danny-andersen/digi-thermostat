#include <SoftwareSerial.h>
#include <AceCRC.h>

using namespace ace_crc::crc16ccitt_nibble;

#define SSID        "\"HeyHo\","
#define PASSWORD    "\"DADADA4566C59EFFEBBEA3690D\""
#define GET_FW_VERS_CMD "AT+GMR"
#define SET_STA_MODE  "AT+CWMODE=1"
#define JOIN_AP_CMD "AT+CWJAP="
#define QUERY_AP "AT+CWJAP?"
#define GET_IP_CMD "AT+CIFSR"
#define SINGLE_CONN "AT+CIPMUX=0"
#define TIMEOUT "AT+CIPSTO=10"  //10 seconds
//#define OPEN_TCP  "AT+CIPSTART=\"TCP\",\"192.168.1.189\",5000"
#define OPEN_TCP  "AT+CIPSTART=\"TCP\",\"192.168.1.219\",5000"
//#define OPEN_TCP  "AT+CIPSTART=\"TCP\",\"192.168.1.30\",5000"
#define CLOSE_TCP  "AT+CIPCLOSE"
#define SEND_LEN  "AT+CIPSEND="
#define CHECK_TCP_STATUS "AT+CIPSTATUS"
#define WIFI_RESET "AT+RST"

// Your board <-> ESP_AT baud rate:
#define ESP_AT_BAUD       9600

SoftwareSerial wifi(10, 11); // RX, TX

#define debugSerial Serial
#define wifiSerial wifi  //Change this to Serial if a problem, i.e. no interrupt support

#define MAX_RESPONSE_TIME 10000 // millis to wait for a response
#define MAX_GET_MSG_SIZE 60 //Max size of dynamic get msg with params
#define MAX_MESSAGE_SIZE 900
#define BUFF_SIZE MAX_MESSAGE_SIZE+6
uint8_t buff[BUFF_SIZE];
char getMessage[MAX_GET_MSG_SIZE];
char GET_MESSAGE_TEMPLATE[] = "GET /message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d HTTP/0.9";
char dateTime[] = "GET /datetime HTTP/0.9";
char thermTemp[] = "GET /temp HTTP/0.9";
char setTemp[] = "GET /settemp HTTP/0.9";
char extTemp[] = "GET /exttemp HTTP/1.0";
char motdStr[] = "GET /motd HTTP/0.9";
char holiday[] = "GET /holiday HTTP/1.1";
//Status message format = /message?t=<temp>st=<thermostat set temp>&r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>

char sendOK[] = "SEND OK";
char ipd[] = "+IPD,";
char colon[] = ":";
char OK[] = "OK";
char CONNECTED[] =  "CONNECTED";
char ERROR_RESP[] = "ERROR";
char READY[] = "ready";
char NO_AP[] = "No AP";
char NO_IP[] = "No ip";
char NOT_VALID[] = "not valid";
char CONTENT_LENGTH[] = "-Length: ";

int16_t currentSetTemp = 100;
int16_t currentTemp = 100;
unsigned long networkDownTime = 0;
bool gotInitMessages = false;
uint8_t resendMessages = 1;

#define LOOP_DELAY 5000
#define RECONNECT_WAIT_TIME 10000
#define RESET_TIME 180000 //3 mins

#define MAX_MOTD_SIZE 64
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
  drainSerial();
  if (checkNetworkUp()) {
    networkDownTime = 0;
//    if (!gotInitMessages) {
//      if (openTCP()) {
//        getDateTime();
//        drainSerial();
//      }
//    }
    if (openTCP()) {
      getNextMessage();
      drainSerial();
    } 
  } else {
    networkDownTime += LOOP_DELAY;
    debugSerial.print("Net down: ");
    debugSerial.println(networkDownTime);
    if (networkDownTime >= RESET_TIME) {
      //Reset wifi adapter every 5 mins, if not connected
      connectToAP();
      networkDownTime = 0;
    }
  }
  delay(LOOP_DELAY);
}

void getDateTime() {
  uint8_t msgId = sendMessage(dateTime, sizeof(dateTime)+3);
  processMessage(msgId);
}

bool getNextMessage() {
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    getMessage[i] == 0;
  }
  //Send current status as params in request
  //Status message format = /message?s=station&rs=resend init messages&t=<temp>st=<thermostat set temp>  &r=< mins to set temp, 0 off>&p=<1 sensor triggered, 0 sensor off>
  //GET /message?s=%d&rs=%d&t=%d&st=%d&r=%d&p=%d
  snprintf(getMessage, MAX_GET_MSG_SIZE, GET_MESSAGE_TEMPLATE, 1, resendMessages, (int)currentTemp, (int)currentSetTemp, 3, 1);
  //Count the message len
  int msglen = 0;
  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
    msglen++;
    if (getMessage[i] == 0) {
      break;
    }
  }
  uint8_t msgId = sendMessage(getMessage, msglen+3); //newlines
  resendMessages = 0;
  return processMessage(msgId);
}

void getThermTemp() {
  uint8_t msgId = sendMessage(thermTemp, sizeof(thermTemp)+3); //newlines
  processMessage(msgId);
}

void getSetTemp() {
  uint8_t msgId = sendMessage(setTemp, sizeof(setTemp)+3); //newlines
  processMessage(msgId);
}

void getExtTemp() {
  uint8_t msgId = sendMessage(extTemp, sizeof(extTemp)+3);
  processMessage(msgId);
}

void getMotd() {
  uint8_t msgId = sendMessage(motdStr, sizeof(motdStr)+3);
  processMessage(msgId);
}

void getHoliday() {
  uint8_t msgId = sendMessage(holiday, sizeof(holiday)+3);
  processMessage(msgId);
}

bool processMessage(uint8_t msgId) {
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
  }

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

bool checkNetworkUp() {
  //Check network up
  drainSerial();
//  debugSerial.println("Check conn: ");
  wifiSerial.println(QUERY_AP);
  bool networkUp = true;
  if (not checkResponse(OK, 2, NO_AP, 5)) {
    printBuff(32);
    networkUp = false;
  }
//  if (networkUp) {
//      debugSerial.println("Connected to AP");
//  }
  drainSerial();
  return networkUp;
}

bool openTCP() {
  bool connected = true;
  //  debugSerial.println("Open TCP ");
  wifiSerial.println(OPEN_TCP);
  if (not checkResponse(CONNECTED, 9, NO_IP, 5)) {
      debugSerial.println("Fail Server");
      printBuff(128);
      connected = false;
  }
  return connected;
}

bool closeTCP() {
  //  debugSerial.println("Close TCP: ");
  wifiSerial.println(CLOSE_TCP);
  if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
     debugSerial.println("Fail Close TCP");
     return false;
  }
  drainSerial();
}

uint8_t sendMessage(char msg[], uint8_t len) {
//  debugSerial.print("Send msg len:");
//  debugSerial.println(len);
  debugSerial.println(msg);

  wifiSerial.print(SEND_LEN);
  wifiSerial.println(len);
  if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
     debugSerial.println("Fail send");
     return 0;
  }
  drainSerial();
//  debugSerial.println("Sending HTTPGET");
  //Send http request
  wifiSerial.println(msg);
  wifiSerial.println();
  wifiSerial.println();
  wifiSerial.println();
  wifiSerial.println();
  wifiSerial.println();
  uint8_t msgId = 0; //Signifies no message
  uint16_t respLen = waitForHttpResponse();
  if (respLen > 0) {
//    debugSerial.print("Received response:");
//    debugSerial.println(respLen);
//    printBuff(respLen);
     
    //Got HTTP response - extract content part
    uint16_t msgLen = extractMessage(respLen);
    if (msgLen > 0){
      Message *msgp = (Message *)&buff[0];
    //    debugSerial.print("Msg Type: ");
    //    debugSerial.print(msgp->id);
    //    debugSerial.print(" Len: ");
    //    debugSerial.println(msgp->len);
      msgId = msgp->id;
      uint16_t rxCrc = msgp->crc;
      msgp->crc = 0;
      crc_t crc = crc_calculate(&buff[0], msgp->len);
    //    crc_t crc = crc_init();
    //    crc = crc_update(crc, &buff[0], 2);
    //    crc = crc_update(crc, &buff[4], (sizeof(DateTimeStruct)+4));
    //    crc = crc_finalize(crc);
      // debugSerial.print(" Rx CRC: ");
      // debugSerial.print(rxCrc, HEX);
      // debugSerial.print(" Calc CRC: ");
      // debugSerial.println(crc, HEX);
      if (crc != rxCrc) {
        debugSerial.println("CRC Failed - ignoring response");
        printBuff(msgp->len);
        msgId = 0;
      }
    } else {
        debugSerial.println("Fail msglen");
        printBuff(respLen);
        msgId = 0;
    }
  } else {
    debugSerial.println("Fail rx:");
    printBuff(MAX_MESSAGE_SIZE);
    msgId = 0;
  }
  return msgId;
}

uint16_t extractMessage(uint16_t msgLen) {
  uint16_t contentLen = 0;
  bool gotLen = false;
  //Look for Content length in header part of response
  uint16_t pos = findStringInBuff(buff, CONTENT_LENGTH, sizeof(CONTENT_LENGTH)-1, msgLen);
  if (pos != -1) {
    pos += sizeof(CONTENT_LENGTH);
    //Retrieve content length
    char lenStr[4];
    uint16_t endPos = pos;
    for (; endPos<(pos + 3); endPos++) {
      if (int(buff[endPos]) == 13) {
        endPos++;
        break; //Reached end of line
      }
      lenStr[endPos-pos] = buff[endPos];
    }
    lenStr[endPos-pos] = '\0'; //null terminate
    contentLen = atoi(&lenStr[0]);
//    debugSerial.print("Len:");
//    debugSerial.println(contentLen);
    if (contentLen > 0 ) {
      endPos += 3; //0xd, 0xa, 0xd, 0xa at end of content str and before content
      //shift bytes down in buff
      shiftBuffDown(endPos, msgLen); //Now we should just have the message content in the buff..
    }
  }
//  debugSerial.print("contentLen: ");
//  debugSerial.println(contentLen);
//  printBuff(contentLen);
  return contentLen;
}


bool connectToAP() {
  debugSerial.println("Reset wifi");
  wifiSerial.println(WIFI_RESET);
  debugResponse();
  drainSerial();

  delay(RECONNECT_WAIT_TIME); //Let it connect and get an IP

//  //Join AP
//  debugSerial.println("Join AP: ");
//  wifiSerial.print(JOIN_AP_CMD);
//  wifiSerial.print(SSID);
//  wifiSerial.println(PASSWORD);
//  if (not checkResponse(READY, 5, NO_AP, 5)) {
//    printBuff(64);
//    return false;
//  }
//  delay(RECONNECT_WAIT_TIME); //Let it connect and get an IP
//  drainSerial();

  drainSerial();
  drainSerial();
  //Join AP
  debugSerial.println("Joining AP");
  wifiSerial.println(JOIN_AP_CMD + String("\"") + SSID + String("\",\"") + PASSWORD + String("\"") );
  debugResponse();
  delay(RECONNECT_WAIT_TIME);
  debugResponse();
  drainSerial();
  drainSerial();

  if (checkNetworkUp()) {
    //Set STA mode 
    debugSerial.println("STA mode");
    wifiSerial.println(SET_STA_MODE);
    printBuff(64);
    if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
  //    printBuff(64);
      return false;
    }
    drainSerial();
    drainSerial();

    //Get local IP
    //  debugSerial.println("Retrieve IP: ");
    wifiSerial.println(GET_IP_CMD);
    printBuff(64);
    if (not checkResponse(OK, 2, NO_IP, 5)) {
//      printBuff(64);
      return false;
    }
    drainSerial();
  } else {
    return false;
  }
  
  //  debugSerial.println("Single connecton: ");
  //  wifiSerial.println(SINGLE_CONN);
  //  if (not checkResponse(OK, 2, ERROR_RESP, 5)) {
  //    return false;
  //  }
  //  drainSerial();
  
  //  debugSerial.println("Set timeout: ");
  //  wifiSerial.println(TIMEOUT);
  //  while (!wifiSerial.available());
  //  debugResponse();

  return true;  
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
  delay(100);
  int total = 0;
  while (uint16_t len = receiveData(0) > 0) {
    total += len;
    delay(100);
  }
//  if (total > 0) {
//    debugSerial.print("Drained: ");
//    debugSerial.println(total);
//  }
  resetBuffer();
   
}

void resetBuffer() {
  //Reset receive buffer
  for (uint32_t i = 0; i<MAX_MESSAGE_SIZE; i++) {
    buff[i] = 0;
  }
}

bool checkResponse(char *str, uint8_t lenStr, char *failStr, uint8_t lenFail) {
  unsigned long start = millis();
  uint16_t bufPos = 0;
  uint16_t pos = -1;
  unsigned long waitTime = 0;
  bool gotResponse = false;
  bool success = false;
  while (waitTime < MAX_RESPONSE_TIME && !gotResponse) {
    uint16_t len = receiveData(bufPos);
    if (len > 0) {
      bufPos += len;
      //Look for fail response
      pos = findStringInBuff(buff, failStr, lenFail, bufPos);
      if (pos != -1) {
        gotResponse = true;
        success = false;
        break;
      } 
      //Look for specific success response
      pos = findStringInBuff(buff, str, lenStr, bufPos);
      if (pos != -1) {
        gotResponse = true;
        success = true;
        break;
      }
      //Then look for generic error response
      pos = findStringInBuff(buff, ERROR_RESP, 5, bufPos);
      if (pos != -1) {
        gotResponse = true;
        success = false;
        break;
      } 
      //Look for generic OK response
      pos = findStringInBuff(buff, OK, 2, bufPos);
      if (pos != -1) {
        gotResponse = true;
        success = true;
        break;
      } 
    }
    if (!gotResponse) {
      //Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - start;
  }
  if (!success) {
    printBuff(bufPos);
  }
  return success;
}

uint16_t waitForHttpResponse() {
  unsigned long start = millis();
  uint16_t bufPos = 0;
  unsigned long waitTime = 0;
  uint16_t msgLen = 0;
  bool sent = false;
  bool gotIPD = false;
  bool gotMsgLen = false;
  bool gotMsg = false;
  while (waitTime < MAX_RESPONSE_TIME && !gotMsg) {
    uint16_t len = receiveData(bufPos);
    if (len > 0) {
      bufPos += len;
      if (!sent) {
        //Check for SEND OK 
        uint16_t pos = findStringInBuff(buff, sendOK, sizeof(sendOK)-1, bufPos);
        if (pos != -1) {
          //Send went OK
          //shift bytes down in buff
          uint16_t endPos = pos+sizeof(sendOK)-1;
          shiftBuffDown(endPos, bufPos);
          bufPos -= endPos;
          sent = true;
        }
      }
      if (sent && !gotIPD) {
        //Look for IPD
        uint16_t pos = findStringInBuff(buff, ipd, sizeof(ipd)-1, bufPos);
        if (pos != -1) {
          //shift bytes down in buff
          uint16_t endPos = pos+sizeof(ipd)-1;
          shiftBuffDown(endPos, bufPos);
          bufPos -= endPos;
          gotIPD = true;
        }
      }
      if (sent && gotIPD && !gotMsgLen) {
        //Find the colon
        uint16_t pos = findStringInBuff(buff, colon, sizeof(colon)-1, bufPos);
        if (pos != -1) {
          //Retrieve msglength
          char lenStr[4];
          for (int i=0; i<pos && i<3; i++) {
            lenStr[i] = buff[i];
          }
          lenStr[pos] = '\0'; //null terminate
          msgLen = atoi(&lenStr[0]);
          //shift bytes down in buff
          uint16_t endPos = pos+1;
          shiftBuffDown(endPos, bufPos);
          bufPos -= endPos;
          gotMsgLen = true;
        }
      }
      if (sent && gotIPD && gotMsgLen && !gotMsg) {
        if (bufPos >= msgLen) {
          //got all of the message
          gotMsg = true;
        }
      } 
    }
    if (!gotMsg) {
      //Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - start;
  }
  if (waitTime >= MAX_RESPONSE_TIME && !gotMsg) {
    debugSerial.print("Msg Rx timeout:");
    debugSerial.print(msgLen);
    debugSerial.print(" Got:");
    debugSerial.print(bufPos);
    printBuff(bufPos);
  }
  return msgLen;
}

void shiftBuffDown(uint8_t pos, uint16_t maxPos) {
//  debugSerial.print("Downshift by ");
//  debugSerial.println(pos);
//  printBuff(MAX_MESSAGE_SIZE);
  uint8_t p = pos;
  for (int i=0; i<maxPos; i++) {
    buff[i] = buff[p++];
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

uint16_t findHexInBuff(uint8_t bf[], uint16_t startPos, uint8_t hex, uint16_t maxPos) {
//  debugSerial.print("Looking for ");
//  debugSerial.println(hex);
//  debugSerial.print(" MaxPos:");
//  debugSerial.println(maxPos);
//  printBuff(maxPos);
  uint16_t match = -1;
  for (int i = startPos; i<maxPos; i++) {
    if (bf[i] == hex) {
      match = i;
      break;
    }
  }
  return match;
}
