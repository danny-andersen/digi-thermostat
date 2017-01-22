#include <Wire.h>

#include <U8x8lib.h>
#include <U8g2lib.h>

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

//Global and stuff to initate once
uint32_t delayMS;
float current_temp = -1;


void setup() {
  delayMS = 2000; 

  temp_sensor.begin();

  oled.begin();
  oled.enableUTF8Print();    // enable UTF8 support for the Arduino print() function

}

void loop() {
  
  //Read RTC
  
  //Check PIR sensor -> if someone near turn on blacklight for 30 seconds

  //Read button inputs
  
  //Read thermometer 
  temp_sensor.requestTemperatures(); // Send the command to get temperatures
  current_temp = temp_sensor.getTempCByIndex(0);
  String cur_temp_str = String(current_temp, 2);
  
//  Serial.print("Temperature for Device 1 is: " + cur_temp_str);
 // Serial.print(current_temp);
//  Serial.print("\n");
    
  //Display current status
  oled.firstPage();
  do {
    oled.setFont(u8g2_font_ncenB14_tr);
    oled.setCursor(0, 15);
    oled.print("Hi Danny!");
    oled.setCursor(0, 30);
    oled.print("Temp: " + cur_temp_str + "C");
  } while ( oled.nextPage() );
  
  delay(delayMS); //temporary delay in loop

}


