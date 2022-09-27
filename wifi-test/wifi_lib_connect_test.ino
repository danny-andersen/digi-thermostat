#include "ESP_AT_Lib.h"

/* Comment this out to disable prints and save space */
#define ESP_AT_DEBUG_OUTPUT     Serial

#define _ESP_AT_LOGLEVEL_       4

#define USE_ESP32_AT      true
#define BOARD_NAME      "MBED NANO_RP2040_CONNECT"
#define EspSerial       wifi
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

SoftwareSerial wifi(10, 11); // RX, TX
ESP8266 wifi(&EspSerial);

void setup(void)
{
  Serial.begin(115200);
  while (!Serial);

  delay(1000);

#if defined(BOARD_NAME)
  Serial.println("\nStart ConnectWiFi on " + String(BOARD_NAME));
#else
  Serial.println("\nStart ConnectWiFi");
#endif

  Serial.println(ESP_AT_LIB_VERSION);

  // initialize serial for ESP module
  EspSerial.begin(ESP_AT_BAUD);

  Serial.print("FW Version: ");
  Serial.println(wifi.getVersion().c_str());


  if (wifi.setOprToStation()) 
  {
    Serial.println("Set STA Mode OK");
  } 
  else 
  {
    Serial.println("Set STA Mode failed");
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

  Serial.println("Done");
}

void loop(void)
{
}