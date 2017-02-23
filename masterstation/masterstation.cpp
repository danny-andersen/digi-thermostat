/**
 * Masterstation
 *
 * Used to send commands, schedules and messages to a digital thermostat
 * Also receives updated status 
 */

#include <cstdlib>
#include <sys/stat.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <unistd.h>
#include <time.h>
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
  int schedSentTime = 0;  
  while(1) {
    network.update();
    RF24NetworkHeader header(DIGI_THERM_NODE_ID);
    header.type = REQ_STATUS_MSG;
    Content payload;
    bool networkUp = false;
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
	    printf("Current temp: %ul\n", payload.status.currentTemp); 
	    printf("Current set temp: %ul\n", payload.status.setTemp); 
	    printf("Heat on? %s\n", payload.status.heatOn == 0 ? "No" : "Yes"); 
	    printf("Mins to set temp: %ul\n", payload.status.minsToSet); 
	    networkUp = true;
        } else {
    	    printf("No response received in timeout - receiver down...\n");
        }
    }
    //Send motd
    //snprintf(&motdPayload.motd.motdStr[0], sizeof(motdPayload.motd.motdStr),
    //			"Hello world! %d", header.id);
    //Read motd file if changed
    struct stat fileStat;
    if(stat(MOTD_FILE,&fileStat) >= 0) {    
        if (fileStat.st_mtime > motdTime) {
	    delay(100);
	    network.update();
	    FILE *fmotd;
	    fmotd = fopen(MOTD_FILE, "r");
	    if (fmotd != NULL) {
		RF24NetworkHeader motdHeader(DIGI_THERM_NODE_ID);
		Content motdPayload;
		if (fgets(&motdPayload.motd.motdStr[0], sizeof(motdPayload.motd.motdStr), fmotd) != NULL) {
		    motdPayload.motd.motdStr[strlen(motdPayload.motd.motdStr)-1] = '\0';
		    printf("Sending motd: %s\n", motdPayload.motd.motdStr);
		    motdHeader.type = MOTD_MSG;
		    if (!network.write(motdHeader, &motdPayload, sizeof(Content))) {
			printf("Failed to write motd message\n");
			networkUp = false;
		    } else {
	    		motdTime = fileStat.st_mtime;
		    }
		}
	    }
	}
    }
    //Send schedules if changed
    if (networkUp && stat(SCHEDULE_FILE,&fileStat) >= 0) {    
        if (fileStat.st_mtime > schedSentTime) {
	    delay(100);
	    network.update();
	    schedSentTime = fileStat.st_mtime;
	    //We are not sure which has changed, so delete all schedules
	    RF24NetworkHeader schedHeader(DIGI_THERM_NODE_ID);
  	    Content schedPayload;
	    schedHeader.type = DELETE_ALL_SCHEDULES_MSG;
	    if (!network.write(schedHeader, &schedPayload, sizeof(Content))) {
		printf("Failed to send delete schedule message\n");
	    } else {
	        //Send schedules, one at a time
	    	FILE *fsched;
	    	fsched = fopen(SCHEDULE_FILE, "r");
	        char * line = NULL;
	        size_t len = 0;
	        ssize_t read;
		//Read line, split by "," and then fill payload and send
		while ((read = getline(&line, &len, fsched)) != -1) {
		    printf("%s", line);
		    char part[32];
		    int pos = strcspn(line, ",");
		    strncpy(part, line, pos);
		    part[pos] = '\0';
		    if (strcmp(part, "Mon-Sun") == 0) {
		        schedPayload.schedule.day = 0x00;
		    } else if (strcmp(part, "Mon-Fri") == 0) {
		        schedPayload.schedule.day = 0x0100;
		    } else if (strcmp(part, "Sat-Sun") == 0) {
		        schedPayload.schedule.day = 0x0200;
		    } else if (strcmp(part, "Mon") == 0) {
		        schedPayload.schedule.day = 0x0001;
		    } else if (strcmp(part, "Tue") == 0) {
		        schedPayload.schedule.day = 0x0002;
		    } else if (strcmp(part, "Wed") == 0) {
		        schedPayload.schedule.day = 0x0003;
		    } else if (strcmp(part, "Thu") == 0) {
		        schedPayload.schedule.day = 0x0004;
		    } else if (strcmp(part, "Fri") == 0) {
		        schedPayload.schedule.day = 0x0005;
		    } else if (strcmp(part, "Sat") == 0) {
		        schedPayload.schedule.day = 0x0006;
		    } else if (strcmp(part, "Sun") == 0) {
		        schedPayload.schedule.day = 0x0007;
		    } else {
		       printf("Unidentified Day specified in schedule: %s", part);
		    } 
		    pos++;
		    //Start and stop times must be 4 digits
		    int shours, smins, ehours, emins;
		    float temp;
		    sscanf(&line[pos], "%2d%2d,%2d%2d,%f",&shours,&smins,&ehours,&emins,&temp);
		    schedPayload.schedule.start = shours * 60 + smins;
		    schedPayload.schedule.end = ehours * 60 + emins;
		    schedPayload.schedule.temp = temp * 10;
		    printf("Day: %x, Start: %d, End: %d, Temp: %d\n", schedPayload.schedule.day, schedPayload.schedule.start, schedPayload.schedule.end, schedPayload.schedule.temp);
		    delay(100);
		    network.update();
	    	    RF24NetworkHeader header(DIGI_THERM_NODE_ID);
	    	    header.type = SCHEDULE_MSG;
	    	    if (!network.write(header, &schedPayload, sizeof(Content))) {
			printf("Failed to send new schedule message\n");
	            }
		}
		free(line);
	    }
	}
    }
    //time_t mytime;
    //mytime = time(NULL);
    sleep(2);
  } // forever loop

  return 0;
}

