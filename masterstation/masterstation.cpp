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

bool readMessage();
bool sendMotd();
bool sendSched();
bool getStatus();
bool sendExtTemp();
bool sendSetTemp();
bool sendHoliday();
bool sendTime(time_t secs);

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
  int extSentTime = 0;
  int tempSentTime = 0;
  time_t sentTime = 0;
  time_t statusTime = 0;
  time_t holidaySentTime = 0;
  bool networkUp = false;
  sleep(1);
  while(1) {
    network.update();
    if (network.available()) {
	if (readMessage() && !networkUp) {
	    printf("Network is now up!\n");
	    networkUp = true;
	}
    } else {
	//Check which command to send, but only do one
        struct stat fileStat;
        if (networkUp && stat(MOTD_FILE,&fileStat) >= 0 
		&& fileStat.st_mtime > motdTime) {
            //Read motd file if changed
	    if (sendMotd()) {
		motdTime = fileStat.st_mtime;
		networkUp = true;
	    } else {
		networkUp = false;
	    }
	} else if (networkUp && stat(SCHEDULE_FILE,&fileStat) >= 0 
		&& fileStat.st_mtime > schedSentTime) {
	    if (sendSched()) {
	        schedSentTime = fileStat.st_mtime;
		networkUp = true;
	    } else {
		networkUp = false;
	    }
	} else if (networkUp && stat(EXTTEMP_FILE,&fileStat) >= 0 
		&& fileStat.st_mtime > extSentTime) {
	    if (sendExtTemp()) {
	        extSentTime = fileStat.st_mtime;
		networkUp = true;
	    } else {
		networkUp = false;
	    }
	} else if (networkUp && stat(SET_TEMP_FILE,&fileStat) >= 0 
		&& fileStat.st_mtime > tempSentTime) {
	    if (sendSetTemp()) {
	        tempSentTime = fileStat.st_mtime;
		networkUp = true;
	    } else {
		networkUp = false;
	    }
	} else if (networkUp && stat(HOLIDAY_FILE,&fileStat) >= 0 
		&& fileStat.st_mtime > holidaySentTime) {
	    if (sendHoliday()) {
	        holidaySentTime = fileStat.st_mtime;
		networkUp = true;
	    } else {
		networkUp = false;
	    }
	} else {
	    time_t secs = time(NULL);
	    if (networkUp && abs(secs - sentTime) > (int)SEND_TIME_INTERVAL) {
	        if (sendTime(secs)) {
		    networkUp = true;
		    sentTime = time(NULL);
		} else {
	    	    networkUp = false;
	        }
	    } else if (abs(secs - statusTime) > (int)STATUS_INTERVAL) {
	       //Req status
	       if (getStatus()) {
		   statusTime = time(NULL);
	       } else {
	      	   networkUp = false;
	       }
	   }
	}
    }
    delay(200);
  }
}

bool getSched() {
    //Get current scheds
    bool status = false;
    RF24NetworkHeader schedHeader(DIGI_THERM_NODE_ID);
    Content schedPayload;
    schedHeader.type = GET_SCHEDULES_MSG;
    if (!network.write(schedHeader, &schedPayload, sizeof(Content))) {
	printf("Failed to send req schedule message\n");
    } else {
	status = true;
    }
    return status;
}

bool sendMotd() {
    FILE *fmotd;
    fmotd = fopen(MOTD_FILE, "r");
    bool status = false;
    if (fmotd != NULL) {
	RF24NetworkHeader motdHeader(DIGI_THERM_NODE_ID);
	Content motdPayload;
	char motdbuf[512];
	if (fgets(&motdbuf[0], sizeof(motdbuf), fmotd) != NULL) {
	    motdPayload.motd.motdStr[0] = '\0';
	    strncat(&motdPayload.motd.motdStr[0], &motdbuf[0], MAX_MOTD_SIZE);
	    motdPayload.motd.motdStr[strlen(motdPayload.motd.motdStr)-1] = '\0';
	    fscanf(fmotd,"%lu\n",&motdPayload.motd.expiry);
	    printf("Sending motd: %s\nExpiry:%lu\n", 
			motdPayload.motd.motdStr,motdPayload.motd.expiry);
	    motdHeader.type = MOTD_MSG;
	    if (!network.write(motdHeader, &motdPayload, sizeof(Content))) {
		printf("Failed to write motd message\n");
	    } else {
	        //Delete file, so it doesn't get resent if we restart
	        remove(MOTD_FILE);
		status = true;
	    }	 
	}
    }
    return status;
}

bool sendTime(time_t secs) {
    struct tm *t = localtime(&secs);
    bool status = false;
    RF24NetworkHeader header(DIGI_THERM_NODE_ID);
    header.type = SET_DATE_TIME_MSG;
    Content payload;
    payload.dateTime.sec = t->tm_sec;
    payload.dateTime.min = t->tm_min;
    payload.dateTime.hour = t->tm_hour;
    payload.dateTime.dayOfMonth = t->tm_mday;
    payload.dateTime.month = t->tm_mon + 1;
    payload.dateTime.year = t->tm_year - 100;
    payload.dateTime.dayOfWeek = t->tm_wday + 1;
    printf("Sending set time, wday: %d, yr: %d\n",t->tm_wday, t->tm_year);
    if (!network.write(header, &payload, sizeof(Content))) {
	printf("Failed to write time message\n");
	status = false;
    } else {
	status = true;
    }
    return status;
}

bool sendExtTemp() {
    FILE *fext;
    fext = fopen(EXTTEMP_FILE, "r");
    bool status = true;
    char tempStr[16];
    char windStr[MAX_WIND_SIZE];
    if (fext != NULL && fgets(&tempStr[0], sizeof(tempStr), fext) != NULL) {
	RF24NetworkHeader header(DIGI_THERM_NODE_ID);
        header.type = SET_EXT_MSG;
	Content payload;
	float tempFl;
        sscanf(&tempStr[0],"%f\n",&tempFl); 
	if (fgets(&windStr[0], sizeof(windStr), fext) != NULL) {
	    payload.setExt.windStr[0] = '\0';
	    strncat(&payload.setExt.windStr[0],&windStr[0], MAX_WIND_SIZE);
	    //Remove newline
	    payload.setExt.windStr[strlen(payload.setExt.windStr)-1] = '\0';
	}
        payload.setExt.setExt  = (int16_t)(tempFl * 10);
        printf("Sending Ext Temp: %d\n", payload.setExt.setExt);
        printf("Sending Wind: %s\n", payload.setExt.windStr);
	if (!network.write(header, &payload, sizeof(Content))) {
	    printf("Failed to write set External Temp message\n");
	    status = false;
	} else {
	    //Delete file, so it doesn't get resent if we restart
	    remove(EXTTEMP_FILE);
	    status = true;
        }	 
    }
    return status;
}

bool sendSetTemp() {
    FILE *fext;
    fext = fopen(SET_TEMP_FILE, "r");
    bool status = true;
    char tempStr[16];
    if (fext != NULL && fgets(&tempStr[0], sizeof(tempStr), fext) != NULL) {
	RF24NetworkHeader header(DIGI_THERM_NODE_ID);
        header.type = SET_TEMP_MSG;
	Content payload;
	float tempFl;
        sscanf(&tempStr[0],"%f",&tempFl); 
        payload.setTemp.setTemp  = (int16_t)(tempFl * 10);
        printf("Sending Set Temp: %d\n", payload.setTemp.setTemp);
	if (!network.write(header, &payload, sizeof(Content))) {
	    printf("Failed to write setTemp message\n");
	    status = false;
	} else {
	    //Delete file, so it doesn't get resent if we restart
	    remove(SET_TEMP_FILE);
	    status = true;
        }	 
    }
    return status;
}

bool sendHoliday() {
    bool status = true;
    FILE *fhols;
    fhols = fopen(HOLIDAY_FILE, "r");
    char * line = NULL;
    size_t len = 0;
    ssize_t read;
    //Read line, split by "," and then fill payload and send
    Content msg;
    int lineCnt = 0;
    while ((read = getline(&line, &len, fhols)) != -1) {
        printf("%s", line);
        char part[32];
	//Get first comma
        int pos = strcspn(line, ",");
        strncpy(part, line, pos);
        part[pos] = '\0';
	pos++;
	bool start = false;
        bool end = false;
	if (strcmp(part, "Start") == 0) start = true;
	else if (strcmp(part, "End") == 0) end = true;
	if (start || end) {
	    int syear, sday, smonth, shour;
	    sscanf(&line[pos], "%02d,%02d,%02d,%02d",&syear,&smonth,&sday,&shour);
	    HolidayDateStr *dt;
	    if (start) dt = &msg.holiday.elem.startDate;
	    else dt = &msg.holiday.elem.endDate;
	    dt->year = syear;
 	    dt->month = smonth;
	    dt->dayOfMonth = sday;
	    dt->hour = shour;
	    printf("start: %d, %d-%d-%d %d\n",start,dt->year,dt->month,dt->dayOfMonth,dt->hour);
	} else if (strcmp(part, "Temp") == 0) {
	    //temp line
	    float temp;
	    sscanf(&line[pos], "%f",&temp);
	    printf("temp: %f\n",temp);
	    msg.holiday.elem.holidayTemp = temp * 10;
	}
	lineCnt++;
    }
    if (lineCnt >= 3) {
	msg.holiday.elem.valid = 1;
	printf("Sending holiday msg temp: %d\n",msg.holiday.elem.holidayTemp);
	//Send holiday
	RF24NetworkHeader header(DIGI_THERM_NODE_ID);
	header.type = SET_HOLIDAY_MSG;
	if (!network.write(header, &msg, sizeof(Content))) {
	    printf("Failed to send holiday message\n");
	    status = false;
	} else {
	    status = true;
        }
    }	
    free(line);
    return status;
}

bool sendSched() {
    //We are not sure which has changed, so delete all schedules
    RF24NetworkHeader delHeader(DIGI_THERM_NODE_ID);
    Content schedPayload;
    delHeader.type = DELETE_ALL_SCHEDULES_MSG;
    bool status = true;
    if (!network.write(delHeader, &schedPayload, sizeof(Content))) {
	printf("Failed to send delete schedule message\n");
	status = false;
    } else {
	delay(200);
	//Send schedules, one at a time
	FILE *fsched;
	fsched = fopen(SCHEDULE_FILE, "r");
	char * line = NULL;
	size_t len = 0;
	ssize_t read;
	//Read line, split by "," and then fill payload and send
	while ((read = getline(&line, &len, fsched)) != -1) {
	    network.update();
	    printf("%s", line);
	    char part[32];
	    int pos = strcspn(line, ",");
	    strncpy(part, line, pos);
	    part[pos] = '\0';
	    schedPayload.schedule.day = 0xFF;
	    if (strcmp(part, "Mon-Sun") == 0) {
		schedPayload.schedule.day = 0x0000;
	    } else if (strcmp(part, "Mon-Fri") == 0) {
		schedPayload.schedule.day = 0x0100;
	    } else if (strcmp(part, "Sat-Sun") == 0) {
		schedPayload.schedule.day = 0x0200;
	    } else if (strcmp(part, "Mon") == 0) {
		schedPayload.schedule.day = 0x0002;
	    } else if (strcmp(part, "Tue") == 0) {
		schedPayload.schedule.day = 0x0003;
	    } else if (strcmp(part, "Wed") == 0) {
		schedPayload.schedule.day = 0x0004;
	    } else if (strcmp(part, "Thu") == 0) {
		schedPayload.schedule.day = 0x0005;
	    } else if (strcmp(part, "Fri") == 0) {
		schedPayload.schedule.day = 0x0006;
	    } else if (strcmp(part, "Sat") == 0) {
		schedPayload.schedule.day = 0x0007;
	    } else if (strcmp(part, "Sun") == 0) {
		schedPayload.schedule.day = 0x0001;
	    } else {
	       printf("Unidentified Day specified in schedule: %s", part);
	    } 
	    if (schedPayload.schedule.day != 0xFF) {
	        pos++;
	        //Start and stop times must be 4 digits
	        int shours, smins, ehours, emins;
	        float temp;
	        sscanf(&line[pos], "%2d%2d,%2d%2d,%f",&shours,&smins,&ehours,&emins,&temp);
	        schedPayload.schedule.start = shours * 60 + smins;
	        schedPayload.schedule.end = ehours * 60 + emins;
	        schedPayload.schedule.temp = temp * 10;
	        printf("Day: %x, Start: %d, End: %d, Temp: %d\n", schedPayload.schedule.day, schedPayload.schedule.start, schedPayload.schedule.end, schedPayload.schedule.temp);
    		RF24NetworkHeader header(DIGI_THERM_NODE_ID);
		header.type = SCHEDULE_MSG;
	    	if (!network.write(header, &schedPayload, sizeof(Content))) {
	 	    printf("Failed to send new schedule message\n");
		    status = false;
		    break; //quit while
	        } else {
		    status = true;
  		}
		//Give it time to write it to EEPROM
		delay(400);
	    }
	} //end while
	free(line);
    }
    return status;
}

bool readMessage() {
    RF24NetworkHeader rxheader;        // If so, grab it and print it out
    Content payload;
    network.read(rxheader, &payload, sizeof(Content) );
    //printf("Received message: %s\n", rxheader.toString());
    bool retStatus = false;
    switch ((int)rxheader.type) {
       case (STATUS_MSG):
	    retStatus = true;
	    FILE *fs;
	    fs = fopen("status.txt", "w+");
	    fprintf(fs,"Current temp: %.2f\n", payload.status.currentTemp/10.0); 
	    fprintf(fs,"Current set temp: %.2f\n", payload.status.setTemp/10.0); 
	    fprintf(fs,"Heat on? %s\n", payload.status.heatOn == 0 ? "No" : "Yes"); 
	    fprintf(fs,"Mins to set temp: %d\n", payload.status.minsToSet); 
	    if (payload.status.extTemp < 1000) {
	    	fprintf(fs,"External temp: %.2f\n", payload.status.extTemp/10.0); 
	    } else {
	    	fprintf(fs,"External temp: Not Set\n");
	    }
	    fprintf(fs,"No of Schedules: %d\n", payload.status.noOfSchedules); 
	    fclose(fs);
	break;
      case (SCHEDULE_MSG):
	    retStatus = true;
	    printf("Sched Msg: %d,%d,%d,%d\n", payload.schedule.day,
					payload.schedule.start,
					payload.schedule.end,
					payload.schedule.temp);
	break;
     default:
	    printf("Received unknown message type: %d\n", rxheader.type);
	break;
    }
    return retStatus;
}

bool getStatus() {
    RF24NetworkHeader header(DIGI_THERM_NODE_ID);
    header.type = REQ_STATUS_MSG;
    Content payload;
    bool status = true;
    if (!network.write(header, &payload, sizeof(Content))) {
	printf("Failed to send status req message\n");
	status = false;
    }
    return status;
}

