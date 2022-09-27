#include <SoftwareSerial.h>

#define SSID        String("HeyHo")
#define PASSWORD    String("DADADA4566C59EFFEBBEA3690D")
#define GET_FW_VERS_CMD String("AT+GMR")
#define SET_STA_MODE  String("AT+CWMODE=1")
#define JOIN_AP_CMD String("AT+CWJAP=\"HeyHo\",\"DADADA4566C59EFFEBBEA3690D\"")
#define GET_IP_CMD String("AT+CIFSR")
//#define SET_BAUD String("AT+CIOBAUD=9600")
#define SET_BAUD String("AT+UART_DEF=9600,8,1,0,0")

// Your board <-> ESP_AT baud rate:
#define ESP_AT_BAUD_INIT       115200
#define ESP_AT_BAUD       9600

SoftwareSerial wifi(10, 11); // RX, TX

#define debugSerial Serial
#define wifiSerial wifi  //Change this to Serial if a problem, i.e. no interrupt support

void setup(void)
{
  debugSerial.begin(9600);
  while (!debugSerial); // wait for serial port to connect
  debugSerial.println("Start:");
//  wifiSerial.begin(ESP_AT_BAUD_INIT);
  debugSerial.println("Setting baud rate to 9600");
  wifiSerial.begin(ESP_AT_BAUD);
//  wifiSerial.println(SET_BAUD);
//  delay(1000);
//  debugResponse();
//  debugSerial.println("Do a Reset: ");
//  wifiSerial.println("AT+RST");
//  debugSerial.println("Sleep for 5: ");
//  delay(5000);
//  debugResponse();
  debugSerial.println("Waiting for wifi module..");
  delay(10000);
  debugSerial.println("Check wifi module serial port..");
  while (!wifiSerial);
  debugResponse();

  //Get Firmware version
  debugSerial.println("Getting firmware version");
  wifiSerial.println(GET_FW_VERS_CMD);
//  delay(2000);
  while (!wifiSerial.available());
  debugResponse();

  //Join AP
  debugSerial.println(String("Joining AP: ") + SSID);
//  wifiSerial.println(JOIN_AP_CMD + String("\"") + SSID + String("\",\"") + PASSWORD + String("\"") );
  wifiSerial.println(JOIN_AP_CMD);
  delay(5000);
  debugResponse();

//Set STA mode 
  debugSerial.println("Set STA mode");
  wifiSerial.println(SET_STA_MODE);
  delay(2000);
  debugResponse();
  
  debugSerial.println("Query access point: ");
  wifiSerial.println("AT+CWJAP?");
  while (!wifiSerial.available());
//  delay(2000);
  debugResponse();

  debugSerial.println("Query access mode: ");
  wifiSerial.println("AT+CWMODE?");
//  delay(2000);
  while (!wifiSerial.available());
  debugResponse();

  //Get local IP
  
  debugSerial.println("Retrieve IP: ");
  wifiSerial.println(GET_IP_CMD);
  while (!wifiSerial.available());
//  delay(2000);
  debugResponse();

//  debugSerial.println("Starting OTA update: ");
//  wifiSerial.println("AT+CIUPDATE");
//  while (!wifiSerial.available());
//  debugResponse();
//  delay(30000);
//  debugResponse();
//  debugSerial.println("Do a Reset: ");
//  wifiSerial.println("AT+RST");
//  while (!wifiSerial.available());
//  debugSerial.println("Sleep for 5: ");
//  delay(5000);
//  debugResponse();
  
  debugSerial.println("List Access Points: ");
  wifiSerial.println("AT+CWLAP");
  while (!wifiSerial.available());
//  delay(2000);
//  while (!wifiSerial.available());
  debugResponse();

//  debugSerial.println("Query access Point params: ");
//  wifiSerial.println("AT+CWSAP?");
////  delay(2000);
//  while (!wifiSerial.available());
//  debugResponse();


  debugSerial.println("Done");
}

void loop(void) {
  while (!wifiSerial.available());
  debugResponse();
}

void debugResponse() {
  bool gotResponse = false;
  int maxWait = 3000;
  while (maxWait >= 0) {
    if (wifiSerial.available()) {
      String str = wifiSerial.readString();
      debugSerial.println(String("Response: ") + str);
      gotResponse = true;
    }
    delay(10);
    maxWait -= 10;
  } 
  if (!gotResponse) {
    debugSerial.println("No response!");
  }
}
