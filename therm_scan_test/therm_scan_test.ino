// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

// Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_BUS 2

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

void setup(void)
{
    // start serial port
    Serial.begin(9600);
    Serial.println("Dallas Temperature IC Control Library Demo");
    // Start up the library
    sensors.begin();
}

void loop(void)
{
    // call sensors.requestTemperatures() to issue a global temperature
    // request to all devices on the bus
    Serial.print("Requesting temperatures...");
    sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.println("DONE");

    // We use the function ByIndex, and as an example get the temperature from the first sensor only.
    int numOfSensors = sensors.getDeviceCount();
    Serial.print("Number of devices on bus: ");
    Serial.println(numOfSensors);
    for (int i = 0; i < numOfSensors; i++)
    {
        float tempC = sensors.getTempCByIndex(i);
        uint8_t addr[8];
        sensors.getAddress(&addr[0], i);
        uint8_t resol = sensors.getResolution(&addr[0]);
        // Check if reading was successful
        if (tempC != DEVICE_DISCONNECTED_C)
        {
            Serial.print("Temperature for device: ");
            Serial.print(i);
            Serial.print(" addr: ");
            for(int j=0; j<8; j++) 
            {
              Serial.print(addr[j]);
            }
            Serial.print(" resol: ");
            Serial.print(resol);
            Serial.print(" is: ");
            Serial.println(tempC);
        }
        else
        {
            Serial.print("Error: Could not read temperature data from device ");
            Serial.println(i);
        }
    }
    delay(1000);
}
