
#include <U8x8lib.h>
#include <U8g2lib.h>

#include <Wire.h>
#include <RTClib.h>
  
#include <DallasTemperature.h>
#include <OneWire.h>

//I/O Pins in use
#define ONE_WIRE_BUS 2

//Thermometer variables
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature temp_sensor(&oneWire);

//OLED
//U8X8_SH1106_128X64_VCOMH0_HW_I2C oled;
U8G2_SH1106_128X64_VCOMH0_1_HW_I2C oled(U8G2_R0);

//RTC
DS1307 rtc;

//Global and stuff to initate once
uint32_t delayMS;
float current_temp = -1;

String getDateStr(const DateTime& dt) {
    return String(dt.year()) + "/" + String(dt.month()) + "/" + String(dt.day(), DEC);
}

String getTimeStr(const DateTime& dt) {
  String time = "";
  if (dt.hour() < 10) {
    time += "0" + String(dt.hour(),DEC);
  } else {
    time += dt.hour();
  }
  time += ":";
  if (dt.minute() < 10) {
    time += "0" + String(dt.minute(),DEC);
  } else {
    time += dt.minute();
  }
  time += ":";
  if (dt.second() < 10) {
    time += "0" + String(dt.second(),DEC);
  } else {
    time += dt.second();
  }
  return time;
}

void setup() {
  Serial.begin(9600);
  delayMS = 2000; 
  temp_sensor.begin();

//  Wire.begin();
 
  oled.begin();
  oled.enableUTF8Print();    // enable UTF8 support for the Arduino print() function

  rtc.begin();
//  rtc.adjust(DateTime(2017, 1, 22, 19, 59, 0));
}

void loop() {
  
  //Read RTC
  DateTime now = rtc.now();  
  Serial.print(getDateStr(now));
  Serial.print(" ");
  Serial.print(getTimeStr(now));
  Serial.print("\n");

  //Check PIR sensor -> if someone near turn on blacklight for 30 seconds

  //Read button inputs
  
  //Read thermometer every 15 seconds?
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  current_temp = temp_sensor.getTempCByIndex(0);
  String cur_temp_str = String(current_temp, 2);

  Serial.print("Temperature for Device 1 is: " + cur_temp_str);
  Serial.print("\n");
    
  //Check temperature against current set point in schedule

  //Turn boiler on or off

  //Display current status
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_ncenB14_tr);
    oled.setCursor(0, 15);
    oled.print(getTimeStr(now));
    oled.setCursor(0, 30);
    oled.print("Temp: " + cur_temp_str + "C");
  } while ( oled.nextPage() );
    
  //Report back current status and actions to in-station via RF transmitter
  
  delay(delayMS); //temporary delay in loop

}


