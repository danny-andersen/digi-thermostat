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
#include <RF24Mesh.h>

#include "../digithermostat.h"

using namespace std;

//
// Hardware configuration
// Configure the appropriate pins for your connections

/****************** Raspberry Pi ***********************/

// Radio CE Pin, CSN Pin, SPI Speed

//RPi Alternate, with SPIDEV - Note: Edit RF24/arch/BBB/spi.cpp and  set 'this->device = "/dev/spidev0.0";;' or as listed in /dev
RF24 radio(22,0);
RF24Network network(radio);
RF24Mesh mesh(radio,network);


/********************************/

// Radio pipe addresses for the 2 nodes to communicate.
const uint8_t pipes[][6] = {"1Node","2Node"};


int main(int argc, char** argv){

  // Setup and configure rf radio
  radio.begin();

  // optionally, increase the delay between retries & # of retries
  radio.setRetries(15,15);
  // Dump the configuration of the rf unit for debugging
  radio.printDetails();

  mesh.setNodeID(MASTER_NODE_ID);
  // Connect to the mesh
  cout << "Connecting to the mesh...";
  mesh.begin(MESH_DEFAULT_CHANNEL, RF24_250KBPS, 60*1000);

  while(1) {
    RF24NetworkHeader header;
    header.to_node = MASTER_NODE_ID;
    header.type = STATUS_REQ_MSG;
    Content payload;
    if (!mesh.write(header, &payload, sizeof(Content)) {
	cout << "Failed to write status req message";
	if (! mesh.checkConnection()) {
	   cout << "Disconnected from mesh: Renewing Address";
      	   mesh.renewAddress();
	} else {
           cout << "Send fail, but still connected";
    }
    unsigned long started_waiting_at = millis();
    bool timeout = false;
    while ( ! radio.available() && ! timeout ) {
	if (millis() - started_waiting_at > RESPONSE_TIMEOUT_MS )
		timeout = true;
	delay(10);
    }
    if (!timeout) {
	radio.read( &payload, sizeof(Content) );
	cout << "Received response!";
	printf("Current temp: %ul\n", payload.status.currentTemp); 
	printf("Current set temp: %ul\n", payload.status.setTemp); 
	printf("Heat on? %s\n", payload.status.heatOn == 0 ? "No" : "Yes"); 
	printf("Mins to set temp: %ul\n", payload.status.minsToSet); 
    } else {
	cout << "No response received in timeout - receiver down...";
    }
    sleep(10);
  } // forever loop

  return 0;
}

