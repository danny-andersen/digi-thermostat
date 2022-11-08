//#define GET_FW_VERS_CMD "AT+GMR"
//#define SINGLE_CONN "AT+CIPMUX=0"
//#define TIMEOUT "AT+CIPSTO=10"  //10 seconds
//#define CHECK_TCP_STATUS "AT+CIPSTATUS"
//#define JOIN_AP_CMD "AT+CWJAP="

#define SET_STA_MODE  "AT+CWMODE=1"
#define QUERY_AP "AT+CWJAP?"
#define GET_IP_CMD "AT+CIFSR"
#define OPEN_TCP  "AT+CIPSTART=\"TCP\",\"192.168.1.189\",5000"
//#define OPEN_TCP  "AT+CIPSTART=\"TCP\",\"192.168.1.219\",5000"
//#define OPEN_TCP  "AT+CIPSTART=\"TCP\",\"192.168.1.30\",5000"
#define CLOSE_TCP  "AT+CIPCLOSE"
#define SEND_LEN  "AT+CIPSEND="
#define WIFI_RESET "AT+RST"

// Your board <-> ESP_AT baud rate:
#define ESP_AT_BAUD       9600

// #define RESPONSE_TIMEOUT_MS 1000
#define MAX_RESPONSE_TIME 10000 // millis to wait for a response
#define RECONNECT_WAIT_TIME 10000

//#define debugSerial Serial

char sendOK[] = "SEND OK";
char ipd[] = "+IPD,";
char colon[] = ":";
char OK[] = "OK";
char CONNECTED[] =  "CONNECTED";
char ERROR_RESP[] = "ERROR";
char NO_AP[] = "No AP";
char NO_IP[] = "No IP";
//char NOT_VALID[] = "not valid";
char CONTENT_LENGTH[] = "-Length: ";
