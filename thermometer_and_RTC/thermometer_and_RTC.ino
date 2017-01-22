
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


//RTC
DS1307 rtc;

//Global and stuff to initate once
uint32_t delayMS;
float current_temp = -1;

String getDateStr(const DateTime& dt) {
    return String(dt.year()) + "/" + String(dt.month()) + "/" + String(dt.day(), DEC);
}

String getTimeStr(const DateTime& dt) {
    return String(dt.hour(), DEC) + ":" + String(dt.minute(), DEC) + ":" + String(dt.second(), DEC);
}

void setup() {
  Serial.begin(9600);
  delayMS = 2000; 
  temp_sensor.begin();

  Wire.begin();
  
  rtc.begin();
  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(__DATE__, __TIME__));
  }
  
  //rtc.adjust(DateTime(2017, 1, 22, 18, 23, 0));


}

void loop() {
  
  //Read RTC
//  DateTime now = rtc.now();  
//  Serial.print(getDateStr(now));
//  Serial.print(" ");
//  Serial.print(getTimeStr(now));
//  Serial.print("\n");

//    Serial.print(now.year(), DEC);
//    Serial.print('/');
//    Serial.print(now.month(), DEC);
//    Serial.print('/');
//    Serial.print(now.day(), DEC);
//    Serial.print(" (");
//    Serial.print(") ");
//    Serial.print(now.hour(), DEC);
//    Serial.print(':');
//    Serial.print(now.minute(), DEC);
//    Serial.print(':');
//    Serial.print(now.second(), DEC);
//    Serial.println();
//    
//    Serial.print(" since midnight 1/1/1970 = ");
//    Serial.print(now.unixtime());
//    Serial.print("s = ");
//    Serial.print(now.unixtime() / 86400L);
//    Serial.println("d");
//
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
   
  //Report back current status and actions to in-station via RF transmitter
  
  delay(delayMS); //temporary delay in loop

}


