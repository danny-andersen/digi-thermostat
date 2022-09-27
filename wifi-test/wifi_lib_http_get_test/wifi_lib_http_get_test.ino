#include "ESP_AT_Lib.h"
#include <SoftwareSerial.h>

/* Comment this out to disable prints and save space */
#define ESP_AT_DEBUG_OUTPUT     Serial

#define _ESP_AT_LOGLEVEL_       4

#define USE_ESP32_AT      true
#define BOARD_NAME      "MBED NANO_RP2040_CONNECT"
//#define EspSerial       Serial1
#if defined(SERIAL_TX_BUFFER_SIZE)
  #undef SERIAL_TX_BUFFER_SIZE
  #define SERIAL_TX_BUFFER_SIZE     256
#endif

#if defined(SERIAL_TX_BUFFER_SIZE)
  #undef SERIAL_TX_BUFFER_SIZE
  #define SERIAL_TX_BUFFER_SIZE     256
#endif

#if defined(SERIAL_RX_BUFFER_SIZE)
  #undef SERIAL_RX_BUFFER_SIZE
  #define SERIAL_RX_BUFFER_SIZE     256
#endif

#define SSID        "SSID"
#define PASSWORD    "password"

// Your board <-> ESP_AT baud rate:
#define ESP_AT_BAUD       9600

#define HOST_NAME   "arduino.cc"        //"www.yahoo.com"
#define HOST_PORT   (80)

SoftwareSerial espSerial(10, 11); // RX, TX
ESP8266 wifi(&espSerial);

void setup(void)
{
  Serial.begin(115200);
  while (!Serial);

  delay(1000);

#if defined(BOARD_NAME)
  Serial.println("\nStart HTTPGET on " + String(BOARD_NAME));
#else
  Serial.println("\nStart HTTPGET");
#endif

  Serial.println(ESP_AT_LIB_VERSION);

  // initialize serial for ESP module
  espSerial.begin(ESP_AT_BAUD);

  Serial.print("FW Version:");
  Serial.println(wifi.getVersion().c_str());

  Serial.print("Set AP/STA Mode ");
  
  if (wifi.setOprToStationSoftAP())
  {
    Serial.println("OK");
  }
  else
  {
    Serial.println("failed");
  }

  if (wifi.joinAP(SSID, PASSWORD))
  {
    Serial.println("Connect to WiFi OK");
    Serial.print("IP: ");
    Serial.println(wifi.getLocalIP().c_str());
  }
  else
  {
    Serial.println("Connect to WiFi failed");
  }

  Serial.print("disableMUX ");
  
  if (wifi.disableMUX()) 
  {
    Serial.println("OK");
  } 
  else 
  {
    Serial.println("failed");
  }

  Serial.println("Done");
}

uint8_t buffer[2048] = {0};

void loop(void)
{
  Serial.print("Create TCP ");
  
  if (wifi.createTCP(HOST_NAME, HOST_PORT)) 
  {
    Serial.println("OK");
  } 
  else 
  {
    Serial.println("failed");
  }

  //char hello[] = "GET / HTTP/1.1\r\nHost: www.yahoo.com\r\nConnection: close\r\n\r\n";
  char hello[] = "GET /asciilogo.txt HTTP/1.1\r\nHost: arduino.cc\r\nConnection: close\r\n\r\n";
  
  wifi.send((const uint8_t*)hello, strlen(hello));

  uint32_t len = wifi.recv(buffer, sizeof(buffer), 10000);
  
  if (len > 0) 
  {
    Serial.println("=========================Received============================");
    
    for (uint32_t i = 0; i < len; i++) 
    {
      Serial.print((char) buffer[i]);
    }
    
    Serial.println("\n============================================================");
  }
  
  if (wifi.releaseTCP()) 
  {
    Serial.println("Release TCP OK");
  } 

  //while (1);
  delay(10000);
}
