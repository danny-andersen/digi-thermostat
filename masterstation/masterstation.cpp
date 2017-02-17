/**
 * Masterstation
 *
 * Used to send commands, schedules and messages to a digital thermostat
 * Also receives updated status 
 */

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <RF24/RF24.h>
#include <RF24Network.h>

#include "digi-thermostat.h"

using namespace std;

//
// Hardware configuration
// Configure the appropriate pins for your connections

/****************** Raspberry Pi ***********************/

// Radio CE Pin, CSN Pin, SPI Speed

//RPi Alternate, with SPIDEV - Note: Edit RF24/arch/BBB/spi.cpp and  set 'this->device = "/dev/spidev0.0";;' or as listed in /dev
RF24 radio(22,0);
RF24Network network(radio);


/********************************/

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipes[][6] = {"1Node","2Node"};


int main(int argc, char** argv){

  // Setup and configure rf radio
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(RADIO_CHANNEL);
  // Dump the configuration of the rf unit for debugging
  radio.printDetails();

  // Connect to the network
  printf("Connecting to the network...\n");
  network.begin(RADIO_CHANNEL, MASTER_NODE_ID);

  while(1) {
    RF24NetworkHeader header;
    header.to_node = DIGI_THERM_NODE_ID;
    header.type = REQ_STATUS_MSG;
    Content payload;
    if (!network.write(header, &payload, sizeof(Content))) {
	printf("Failed to write status req message\n");
    } else {
	unsigned long started_waiting_at = millis();
        bool timeout = false;
        while ( ! radio.available() && ! timeout ) {
	    if (millis() - started_waiting_at > RESPONSE_TIMEOUT_MS )
		timeout = true;
	    delay(10);
        }
        if (!timeout) {
	    radio.read( &payload, sizeof(Content) );
	    printf("Received response!\n");
	    printf("Current temp: %ul\n", payload.status.currentTemp); 
	    printf("Current set temp: %ul\n", payload.status.setTemp); 
	    printf("Heat on? %s\n", payload.status.heatOn == 0 ? "No" : "Yes"); 
	    printf("Mins to set temp: %ul\n", payload.status.minsToSet); 
        } else {
    	    printf("No response received in timeout - receiver down...\n");
        }
    }
    sleep(10);
  } // forever loop

  return 0;
}

