/**
 * Masterstation
 *
 * Used to send commands, schedules and messages to a digital thermostat
 * Also receives updated status 
 */

#include <cstdlib>
#include <sys/stat.h>
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

RF24 radio(RPI_V2_GPIO_P1_15, BCM2835_SPI_CS0, BCM2835_SPI_SPEED_8MHZ);

//RPi Alternate, with SPIDEV - Note: Edit RF24/arch/BBB/spi.cpp and  set 'this->device = "/dev/spidev0.0";;' or as listed in /dev
//RF24 radio(22,0);

RF24Network network(radio);


/********************************/

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipes[][6] = {"1Node","2Node"};


int main(int argc, char** argv){

  // Setup and configure rf radio
  radio.begin();

  // optionally, increase the delay between retries & # of retries
//  radio.setRetries(400UL,1);
  radio.setPALevel(RF24_PA_HIGH);
  radio.enableDynamicPayloads();
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(RADIO_CHANNEL);
  // Dump the configuration of the rf unit for debugging
  radio.printDetails();

  // Connect to the network
  printf("Connecting to the network...\n");
  network.begin(RADIO_CHANNEL, MASTER_NODE_ID);

  int motdTime = 0;  
  while(1) {
    network.update();
    RF24NetworkHeader header(DIGI_THERM_NODE_ID);
    header.type = REQ_STATUS_MSG;
    Content payload;
    if (!network.write(header, &payload, sizeof(Content))) {
	printf("Failed to write status req message\n");
    } else {
	unsigned long started_waiting_at = millis();
        bool timeout = false;
        while ( ! network.available() && ! timeout ) {
	    if (millis() - started_waiting_at > RESPONSE_TIMEOUT_MS )
		timeout = true;
	    delay(10);
        }
        if (!timeout) {
	    RF24NetworkHeader rxheader;        // If so, grab it and print it out
	    network.read(rxheader, &payload, sizeof(Content) );
	    printf("Received response!\n");
	    printf("Current temp: %ul\n", payload.status.currentTemp); 
	    printf("Current set temp: %ul\n", payload.status.setTemp); 
	    printf("Heat on? %s\n", payload.status.heatOn == 0 ? "No" : "Yes"); 
	    printf("Mins to set temp: %ul\n", payload.status.minsToSet); 
        } else {
    	    printf("No response received in timeout - receiver down...\n");
        }
    }
    delay(100);
    network.update();
    //Send motd
  
    //snprintf(&motdPayload.motd.motdStr[0], sizeof(motdPayload.motd.motdStr),
    //			"Hello world! %d", header.id);
    //Read motd file if changed
    struct stat fileStat;
    if(stat(MOTD_FILE,&fileStat) >= 0) {    
	if (fileStat.st_mtime > motdTime) {
	    motdTime = fileStat.st_mtime;
	    FILE *fmotd;
	    fmotd = fopen(MOTD_FILE, "r");
	    if (fmotd != NULL) {
		RF24NetworkHeader motdHeader(DIGI_THERM_NODE_ID);
		Content motdPayload;
		if (fgets(&motdPayload.motd.motdStr[0], sizeof(motdPayload.motd.motdStr), fmotd) != NULL) {
		    printf("Sending motd: %s\n", motdPayload.motd.motdStr);
		    motdHeader.type = MOTD_MSG;
		    if (!network.write(motdHeader, &motdPayload, sizeof(Content))) {
			printf("Failed to write motd message\n");
		    } 
		}
	    }
	}
    }
    sleep(2);
  } // forever loop

  return 0;
}

