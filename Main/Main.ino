/*
 Name:		Main.ino
 Created:	12/14/2021 10:19:58 PM
 Authors:	Aar�n I. V�lez Ram�rez and Irving J. Garc�a L�pez
*/





//////////
// Pins //
//////////
// Pins 11, 12 and 13 are used for WiFi feather wing
const int SD_CS_PIN = 10;				// SD on Addlogger M0 Feather 
const int R_Chamber_Lamp_PIN = 14;		// Power Actinic Lamp in Red chamber
const int FR_Chamber_Lamp_PIN = 15;		// Power Actinic Lamp in Farred chamber
const int I2C_Select_0_PIN = 9;		// I2C multiplexer digital select line
const int FR_1_LEDs_PIN = 16;			// Power Farred LEDs, circuit 1
const int FR_2_LEDs_PIN = 17;			// Power Farred LEDs, circuit 2
const int FR_1_LEDs_PWM_PIN = 18;		// PWM  line to dim FarRed LEDs, circuit 1
const int FR_2_LEDs_PWM_PIN = 19;		// PWM  line to dim FarRed LEDs, circuit 2
const int R_Chamber_Fan_PIN = 5;		// Power Fan in Red Chamber
const int FR_Chamber_Fan_PIN = 6;		// Power Fan in Farred Chamber





//////////////////////////////////////////////////////////////////
////// Libraries and its associated constants and variables //////
//////////////////////////////////////////////////////////////////


////// Credentials_R-FR_Chamber.h is a user-created library containing paswords, IDs and credentials
#include "Credentials_R-FR_Chamber.h"
const char ssid[] = WIFI_SSID;
const char password[] = WIFI_PASSWD;
const char iot_server[] = IoT_SERVER;
const char iot_user[] = IoT_USER;
const char iot_device[] = IoT_DEVICE;
const char iot_credential[] = IoT_CREDENTIAL;
const char iot_data_bucket[] = IoT_DATA_BUCKET;


////// Comunication libraries
#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>
#define SPIWIFI       SPI  // The SPI port
#define SPIWIFI_SS    13   // Chip select pin
#define ESP32_RESETN  12   // Reset pin
#define SPIWIFI_ACK   11   // a.k.a BUSY or READY pin
#define ESP32_GPIO0   -1
#include <WiFiUdp.h>
WiFiUDP ntpUDP;


////// Iot Thinger
#define THINGER_SERIAL_DEBUG		// Uncomment for debugging connection to Thinger
#define THINGER_SERVER iot_server   // Delete this line if using a free thinger account 
//#define _DISABLE_TLS_				// Uncoment if needed (port 25202 closed, for example)
#include <ThingerWiFiNINA.h>
ThingerWiFiNINA thing(iot_user, iot_device, iot_credential);
/// <summary>
/// IMPORTANT
/// in file \thinger.io\src\ThingerClient.h at Line 355, the function handle_connection() was modified
/// to prevent the thinger handler to agresively try to reconnect to WiFi in case of a lost connection
/// This allows the device to keep monitoring environmental variables even if there is no network connection
/// </summary>


////// SD fat library used in order to use exFAT file system
#include <SdFat.h>
const int SD_FAT_TYPE = 2;	// SD_FAT_TYPE = 2 for exFAT
const int SD_SCK_MHZ = 10;	// it works till 12 MHz in M0 board
SdExFat sd;
ExFile LogFile;
ExFile root;
char line[250];
String str = "";
int position = 0;


////// Time libraries
#include <RTClib.h>
RTC_PCF8523 rtc;
#include <TimeLib.h>
#include <NTPClient.h>
NTPClient timeClient(ntpUDP, "north-america.pool.ntp.org", 0, 300000); // For details, see https://github.com/arduino-libraries/NTPClient
// Time zone library
#include <Timezone.h>
//  Central Time Zone (Mexico City)
TimeChangeRule mxCDT = { "CDT", First, Sun, Apr, 2, -300 };
TimeChangeRule mxCST = { "CST", Last, Sun, Oct, 2, -360 };
Timezone mxCT(mxCDT, mxCST);


////// Library for SHT31 Temperature and Humidity Sensor
#include <DFRobot_SHT3x.h>
DFRobot_SHT3x sht3x(&Wire,/*address=*/0x44,/*RST=*/4); // secondary I2C address 0x44
DFRobot_SHT3x::sRHAndTemp_t sht3x_data;





//////////////////////////////////////////
////// User Constants and Variables //////
//////////////////////////////////////////


////// Station IDs & Constants
const int StaNum = 7;
String StaType = F("M0_WiFiNA_RTC-PCF8523_Thinger");
String StaName = F("R-FR Chamber LANGEBIO");
String Firmware = F("v1.0.0");
//const float VRef = 3.3;
bool debug = true;


////// Log File & Headers
String FileName = "";
const String Headers = F("UTC_UNIX_t\tLocal_UNIX_t\tyear\tmonth\tday\thour\tminute\tsecond\t\
R_AirTemp\tR_AirRH\t\
FR_AirTemp\tFR_AirRH\t\
R_Actinic_ON\tFR_Actinic_ON\t\
R_Fan_ON\tFR_Fan_ON\t\
FR_1_LEDs_ON\tFR_2_LEDs_ON\t\
FR_1_LEDs_PWM\tFR_2_LEDs_PWM\t\
SensorsOK\t\
SentIoT");
const int HeaderN = 21;	// cero indexed
String LogString = "";


////// M0 ADC constants
const int n = 100; // measure n times the ADC input for averaging
float sum = 0; // shift register to hold ADC data


////// Time variables
DateTime RTCnow;    // UTC Date-Time class from RTClib
time_t LastNTP;     // Last UTP time that the RTC was updated form NTP
time_t UTC_t;       // UTC UNIX time stamp
time_t local_t;     // Local time with DST adjust in UNIX time stamp format
time_t SD_local_t;    // Recorded UNIX time stamp
int s = -1;		    // Seconds
int m = -1;		    // Minutes
int h = -1;		    // Hours
int dy = -1;	    // Day
int mo = -1;	    // Month
int yr = -1;	    // Year
// Time for IoT payload
int mIoT = -1;
int hIoT = -1;
int dyIoT = -1;
int moIoT = -1;
int yrIoT = -1;


////// State machine Shift Registers
//int LastSec = -1;           // Last second register (to update clock display each second)
int LastSum = -1;			// Last minute that variables were measured added to the sum for later averaging
int SumNum = 0;				// Number of times a variable value has beed added to the sum for later averaging
int LastLog = -1;			// Last minute that variables were loged to the SD card
bool PayloadRdy = false;	// Payload ready to send to IoT

time_t t_WiFiCnxTry = 0;      // Last time a (re)connection to internet happened
const int WiFiCnx_frq = 30;  // (re)connection to internet frequency in seconds

byte SensorsOK = B00000000;     // Byte variable to store real time sensor status
byte SensorsOKAvg = B00001111;  // Byte variable to store SD card average sensor status
int SensorsOKIoT = 0;           // Variable to send sensor status in decimal format

time_t t_DataBucket = 0;             // Last time Data was sent to bucket (in UNIX time format) 
const int DataBucket_frq = 150;       // Data bucket update frequency in seconds (must be more than 60)


////// Lamps, LEDs and Fan control variables
/// Variables for Actinic Lamp in Red Chamber
bool R_Lamp_Manual_Ctrl = false;
bool R_Lamp_Manual_ON = false;
bool R_Lamp_ON = false;
bool R_Lamp_Period_1 = true;
bool R_Lamp_Period_2 = false;
bool R_Lamp_Period_3 = false;
unsigned int R_Lamp_Period_1_ON = 0;
unsigned int R_Lamp_Period_2_ON = 0;
unsigned int R_Lamp_Period_3_ON = 0;
unsigned int R_Lamp_Period_1_OFF = 0;
unsigned int R_Lamp_Period_2_OFF = 0;
unsigned int R_Lamp_Period_3_OFF = 0;
/// Variables for Actinic Lamp in Farred Chamber
bool FR_Lamp_Manual_Ctrl = false;
bool FR_Lamp_Manual_ON = false;
bool FR_Lamp_ON = false;
bool FR_Lamp_Period_1 = true;
bool FR_Lamp_Period_2 = false;
bool FR_Lamp_Period_3 = false;
unsigned int FR_Lamp_Period_1_ON = 0;
unsigned int FR_Lamp_Period_2_ON = 0;
unsigned int FR_Lamp_Period_3_ON = 0;
unsigned int FR_Lamp_Period_1_OFF = 0;
unsigned int FR_Lamp_Period_2_OFF = 0;
unsigned int FR_Lamp_Period_3_OFF = 0;
/// Variables for FarRed LEDs
// Circuit 1
bool FR_1_LEDs_Manual_Ctrl = false;
bool FR_1_LEDs_Manual_ON = false;
bool FR_1_LEDs_ON = false;
int FR_1_LEDs_PWM_Duty_Cycle = 0;
bool FR_1_LEDs_Period_1 = true;
bool FR_1_LEDs_Period_2 = false;
bool FR_1_LEDs_Period_3 = false;
unsigned int FR_1_LEDs_Period_1_ON = 0;
unsigned int FR_1_LEDs_Period_2_ON = 0;
unsigned int FR_1_LEDs_Period_3_ON = 0;
unsigned int FR_1_LEDs_Period_1_OFF = 0;
unsigned int FR_1_LEDs_Period_2_OFF = 0;
unsigned int FR_1_LEDs_Period_3_OFF = 0;
// Circuit 2
bool FR_2_LEDs_Manual_Ctrl = false;
bool FR_2_LEDs_Manual_ON = false;
bool FR_2_LEDs_ON = false;
int FR_2_LEDs_PWM_Duty_Cycle = 0;
bool FR_2_LEDs_Period_1 = true;
bool FR_2_LEDs_Period_2 = false;
bool FR_2_LEDs_Period_3 = false;
unsigned int FR_2_LEDs_Period_1_ON = 0;
unsigned int FR_2_LEDs_Period_2_ON = 0;
unsigned int FR_2_LEDs_Period_3_ON = 0;
unsigned int FR_2_LEDs_Period_1_OFF = 0;
unsigned int FR_2_LEDs_Period_2_OFF = 0;
unsigned int FR_2_LEDs_Period_3_OFF = 0;
/// Variables for Fans
bool R_Fan_Manual_Ctrl = false;
bool R_Fan_Manual_ON = false;
bool R_Fan_ON = false;
bool FR_Fan_Manual_Ctrl = false;
bool FR_Fan_Manual_ON = false;
bool FR_Fan_ON = false;
/// Variables from SD card
// Actinic lamps
bool SD_R_Lamp_Manual_Ctrl = false;
bool SD_R_Lamp_ON = false;
bool SD_FR_Lamp_Manual_Ctrl = false;
bool SD_FR_Lamp_ON = false;
// Farred LEDs
bool SD_FR_1_LEDs_Manual_Ctrl = false;
bool SD_FR_1_LEDs_ON = false;
int SD_FR_1_LEDs_PWM_Duty_Cycle = 0;
bool SD_FR_2_LEDs_Manual_Ctrl = false;
bool SD_FR_2_LEDs_ON = false;
int SD_FR_2_LEDs_PWM_Duty_Cycle = 0;
// Fans
bool SD_R_Fan_Manual_Ctrl = false;
bool SD_R_Chamber_Fan_ON = false;
bool SD_FR_Fan_Manual_Ctrl = false;
bool SD_FR_Chamber_Fan_ON = false;


////// Measured instantaneous variables
float Temp_R = -1;        // Air temperature RED chamber
float RH_R = -1;          // Air RH value RED chamber
float Temp_FR = -1;        // Air temperature FARRED chamber 
float RH_FR = -1;          // Air RH value FARRED chamber


////// Variables to store sum for eventual averaging
double TempSum_R = 0;
double RHSum_R = 0;
double TempSum_FR = 0;
double RHSum_FR = 0;
double R_Fan_ON_Sum = 0;
double FR_Fan_ON_Sum = 0;


////// Values to be logged. They will be the average over the last 5 minutes
float TempAvg_R = 0;
float RHAvg_R = 0;
float TempAvg_FR = 0;
float RHAvg_FR = 0;
float R_Fan_ON_Avg = 0;
float FR_Fan_ON_Avg = 0;






//////////////////////////////////////////////////////////////////////////
// the setup function runs once when you press reset or power the board //
//////////////////////////////////////////////////////////////////////////
void setup() {

	////// Start General Libraries
	Serial.begin(9600);
	delay(2500);
	if (debug = true) { Serial.print(F("\nSetup start\n")); }
	Wire.begin();
	SPI.begin();
	

	////// Start WiFi
	if (debug = true) { Serial.print(F("Conecting to WiFi")); }
	WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
	// Test for 10 seconds if there is WiFi connection; if not, continue to loop
	for (int i = 0; i <= 10; i++) {
		if (WiFi.status() != WL_CONNECTED) {
			WiFi.begin(WIFI_SSID, WIFI_PASSWD);
			Serial.print(".");
			delay(1000);
		}
		else {
			Serial.println(F("\nConnected to internet!"));
			Serial.print(F("IP address: "));
			Serial.println(WiFi.localIP());
			break;
		}
		if (i == 10) {
			Serial.println(F("\nNo internet connection"));
			WiFi.disconnect();  // if no internet, disconnect. This prevents the board to be busy only trying to connect.
		}
	}
	

	////// Set pin modes of pins not associated with libraries
	//analogReadResolution(12);
	// Actinic light
	pinMode(R_Chamber_Lamp_PIN, OUTPUT);
	digitalWrite(R_Chamber_Lamp_PIN, LOW);
	pinMode(FR_Chamber_Lamp_PIN, OUTPUT);
	digitalWrite(FR_Chamber_Lamp_PIN, LOW);
	//I2C multiplexer
	pinMode(I2C_Select_0_PIN, OUTPUT);
	digitalWrite(I2C_Select_0_PIN, LOW);
	// Farred LEDs
	pinMode(FR_1_LEDs_PIN, OUTPUT);
	digitalWrite(FR_1_LEDs_PIN, LOW);
	pinMode(FR_2_LEDs_PIN, OUTPUT);
	digitalWrite(FR_2_LEDs_PIN, LOW);
	pinMode(FR_1_LEDs_PWM_PIN, OUTPUT);
	analogWrite(FR_1_LEDs_PWM_PIN, 0);
	pinMode(FR_2_LEDs_PWM_PIN, OUTPUT);
	analogWrite(FR_2_LEDs_PWM_PIN, 0);
	// Fans
	pinMode(R_Chamber_Fan_PIN, OUTPUT);
	digitalWrite(R_Chamber_Fan_PIN, LOW);
	pinMode(FR_Chamber_Fan_PIN, OUTPUT);
	digitalWrite(FR_Chamber_Fan_PIN, LOW);


	////// Initialize SD card
	if (debug) { Serial.println(F("Setting SD card...")); }
	sd.begin(SD_CS_PIN, SD_SCK_MHZ);
	// Reserve RAM memory for large and dynamic String object
	// used in SD file write/read
	// (prevents heap RAM framgentation)
	LogString.reserve(HeaderN * 7);
	str.reserve(HeaderN * 7);

	
	////// Configure IoT
	if (debug) { Serial.println(F("Configuring IoT...")); }
	thing.add_wifi(ssid, password);
	
	// Define input resources
	thing["Debug"] << [](pson& in) {
		if (in.is_empty()) { in = debug; }
		else { debug = in; }
	};
	thing["R_Lamp_Ctrl"] << [](pson& in) {
		if (in.is_empty()) {
			in["R_Lamp_Manual_Ctrl"] = R_Lamp_Manual_Ctrl;
			in["R_Lamp_Manual_ON"] = R_Lamp_Manual_ON;
			in["R_Lamp_Period_1"] = R_Lamp_Period_1;
			in["R_Lamp_Period_2"] = R_Lamp_Period_2;
			in["R_Lamp_Period_3"] = R_Lamp_Period_3;
			in["R_Lamp_Period_1_ON"] = R_Lamp_Period_1_ON;
			in["R_Lamp_Period_2_ON"] = R_Lamp_Period_2_ON;
			in["R_Lamp_Period_3_ON"] = R_Lamp_Period_3_ON;
			in["R_Lamp_Period_1_OFF"] = R_Lamp_Period_1_OFF;
			in["R_Lamp_Period_2_OFF"] = R_Lamp_Period_2_OFF;
			in["R_Lamp_Period_3_OFF"] = R_Lamp_Period_3_OFF;
		}
		else {
			R_Lamp_Manual_Ctrl = in["R_Lamp_Manual_Ctrl"];
			R_Lamp_Manual_ON = in["R_Lamp_Manual_ON"];
			R_Lamp_Period_1 = in["R_Lamp_Period_1"];
			R_Lamp_Period_2 = in["R_Lamp_Period_2"];
			R_Lamp_Period_3 = in["R_Lamp_Period_3"];
			R_Lamp_Period_1_ON = in["R_Lamp_Period_1_ON"];
			R_Lamp_Period_2_ON = in["R_Lamp_Period_2_ON"];
			R_Lamp_Period_3_ON = in["R_Lamp_Period_3_ON"];
			R_Lamp_Period_1_OFF = in["R_Lamp_Period_1_OFF"];
			R_Lamp_Period_2_OFF = in["R_Lamp_Period_2_OFF"];
			R_Lamp_Period_3_OFF = in["R_Lamp_Period_3_OFF"];
		}
	};
	thing["FR_Lamp_Ctrl"] << [](pson& in) {
		if (in.is_empty()) {
			in["FR_Lamp_Manual_Ctrl"] = FR_Lamp_Manual_Ctrl;
			in["FR_Lamp_Manual_ON"] = FR_Lamp_Manual_ON;
			in["FR_Lamp_Period_1"] = FR_Lamp_Period_1;
			in["FR_Lamp_Period_2"] = FR_Lamp_Period_2;
			in["FR_Lamp_Period_3"] = FR_Lamp_Period_3;
			in["FR_Lamp_Period_1_ON"] = FR_Lamp_Period_1_ON;
			in["FR_Lamp_Period_2_ON"] = FR_Lamp_Period_2_ON;
			in["FR_Lamp_Period_3_ON"] = FR_Lamp_Period_3_ON;
			in["FR_Lamp_Period_1_OFF"] = FR_Lamp_Period_1_OFF;
			in["FR_Lamp_Period_2_OFF"] = FR_Lamp_Period_2_OFF;
			in["FR_Lamp_Period_3_OFF"] = FR_Lamp_Period_3_OFF;
		}
		else {
			FR_Lamp_Manual_Ctrl = in["FR_Lamp_Manual_Ctrl"];
			FR_Lamp_Manual_ON = in["FR_Lamp_Manual_ON"];
			FR_Lamp_Period_1 = in["FR_Lamp_Period_1"];
			FR_Lamp_Period_2 = in["FR_Lamp_Period_2"];
			FR_Lamp_Period_3 = in["FR_Lamp_Period_3"];
			FR_Lamp_Period_1_ON = in["FR_Lamp_Period_1_ON"];
			FR_Lamp_Period_2_ON = in["FR_Lamp_Period_2_ON"];
			FR_Lamp_Period_3_ON = in["FR_Lamp_Period_3_ON"];
			FR_Lamp_Period_1_OFF = in["FR_Lamp_Period_1_OFF"];
			FR_Lamp_Period_2_OFF = in["FR_Lamp_Period_2_OFF"];
			FR_Lamp_Period_3_OFF = in["FR_Lamp_Period_3_OFF"];
		}
	};
	thing["FR_1_LEDs_Ctrl"] << [](pson& in) {
		if (in.is_empty()) {
			in["FR_1_LEDs_Manual_Ctrl"] = FR_1_LEDs_Manual_Ctrl;
			in["FR_1_LEDs_Manual_ON"] = FR_1_LEDs_Manual_ON;
			in["FR_1_LEDs_PWM_Duty_Cycle "] = FR_1_LEDs_PWM_Duty_Cycle;
			in["FR_1_LEDs_Period_1"] = FR_1_LEDs_Period_1;
			in["FR_1_LEDs_Period_2"] = FR_1_LEDs_Period_2;
			in["FR_1_LEDs_Period_3"] = FR_1_LEDs_Period_3;
			in["FR_1_LEDs_Period_1_ON"] = FR_1_LEDs_Period_1_ON;
			in["FR_1_LEDs_Period_2_ON"] = FR_1_LEDs_Period_2_ON;
			in["FR_1_LEDs_Period_3_ON"] = FR_1_LEDs_Period_3_ON;
			in["FR_1_LEDs_Period_1_OFF"] = FR_1_LEDs_Period_1_OFF;
			in["FR_1_LEDs_Period_2_OFF"] = FR_1_LEDs_Period_2_OFF;
			in["FR_1_LEDs_Period_3_OFF"] = FR_1_LEDs_Period_3_OFF;
		}
		else {
			FR_1_LEDs_Manual_Ctrl = in["FR_1_LEDs_Manual_Ctrl"];
			FR_1_LEDs_Manual_ON = in["FR_1_LEDs_Manual_ON"];
			FR_1_LEDs_PWM_Duty_Cycle = in["FR_1_LEDs_PWM_Duty_Cycle"];
			FR_1_LEDs_Period_1 = in["FR_1_LEDs_Period_1"];
			FR_1_LEDs_Period_2 = in["FR_1_LEDs_Period_2"];
			FR_1_LEDs_Period_3 = in["FR_1_LEDs_Period_3"];
			FR_1_LEDs_Period_1_ON = in["FR_1_LEDs_Period_1_ON"];
			FR_1_LEDs_Period_2_ON = in["FR_1_LEDs_Period_2_ON"];
			FR_1_LEDs_Period_3_ON = in["FR_1_LEDs_Period_3_ON"];
			FR_1_LEDs_Period_1_OFF = in["FR_1_LEDs_Period_1_OFF"];
			FR_1_LEDs_Period_2_OFF = in["FR_1_LEDs_Period_2_OFF"];
			FR_1_LEDs_Period_3_OFF = in["FR_1_LEDs_Period_3_OFF"];
		}
	};
	thing["FR_2_LEDs_Ctrl"] << [](pson& in) {
		if (in.is_empty()) {
			in["FR_2_LEDs_Manual_Ctrl"] = FR_2_LEDs_Manual_Ctrl;
			in["FR_2_LEDs_Manual_ON"] = FR_2_LEDs_Manual_ON;
			in["FR_2_LEDs_PWM_Duty_Cycle "] = FR_2_LEDs_PWM_Duty_Cycle;
			in["FR_2_LEDs_Period_1"] = FR_2_LEDs_Period_1;
			in["FR_2_LEDs_Period_2"] = FR_2_LEDs_Period_2;
			in["FR_2_LEDs_Period_3"] = FR_2_LEDs_Period_3;
			in["FR_2_LEDs_Period_1_ON"] = FR_2_LEDs_Period_1_ON;
			in["FR_2_LEDs_Period_2_ON"] = FR_2_LEDs_Period_2_ON;
			in["FR_2_LEDs_Period_3_ON"] = FR_2_LEDs_Period_3_ON;
			in["FR_2_LEDs_Period_1_OFF"] = FR_2_LEDs_Period_1_OFF;
			in["FR_2_LEDs_Period_2_OFF"] = FR_2_LEDs_Period_2_OFF;
			in["FR_2_LEDs_Period_3_OFF"] = FR_2_LEDs_Period_3_OFF;
		}
		else {
			FR_2_LEDs_Manual_Ctrl = in["FR_2_LEDs_Manual_Ctrl"];
			FR_2_LEDs_Manual_ON = in["FR_2_LEDs_Manual_ON"];
			FR_2_LEDs_PWM_Duty_Cycle = in["FR_2_LEDs_PWM_Duty_Cycle"];
			FR_2_LEDs_Period_1 = in["FR_2_LEDs_Period_1"];
			FR_2_LEDs_Period_2 = in["FR_2_LEDs_Period_2"];
			FR_2_LEDs_Period_3 = in["FR_2_LEDs_Period_3"];
			FR_2_LEDs_Period_1_ON = in["FR_2_LEDs_Period_1_ON"];
			FR_2_LEDs_Period_2_ON = in["FR_2_LEDs_Period_2_ON"];
			FR_2_LEDs_Period_3_ON = in["FR_2_LEDs_Period_3_ON"];
			FR_2_LEDs_Period_1_OFF = in["FR_2_LEDs_Period_1_OFF"];
			FR_2_LEDs_Period_2_OFF = in["FR_2_LEDs_Period_2_OFF"];
			FR_2_LEDs_Period_3_OFF = in["FR_2_LEDs_Period_3_OFF"];
		}
	};
	
	// Define output resources
	thing["RT_R_Lamp_ON"] >> [](pson& out) { out = R_Lamp_ON; };
	thing["RT_FR_Lamp_ON"] >> [](pson& out) { out = FR_Lamp_ON; };
	thing["RT_FR_1_LEDs_ON"] >> [](pson& out) { out = FR_1_LEDs_ON; };
	thing["RT_FR_2_LEDs_ON"] >> [](pson& out) { out = FR_2_LEDs_ON; };
	thing["RT_R_Fan_ON "] >> [](pson& out) { out = R_Fan_ON; };
	thing["RT_FR_Fan_ON "] >> [](pson& out) { out = FR_Fan_ON; };
	thing["RT_Temp_Red"] >> [](pson& out) { out = Temp_R; };
	thing["RT_RH_Red"] >> [](pson& out) { out = RH_R; };
	thing["RT_Temp_FarRed"] >> [](pson& out) { out = Temp_FR; };
	thing["RT_RH_FarRed"] >> [](pson& out) { out = RH_FR; };
	
	thing["Avg_Data"] >> [](pson& out) {
		out["Time_Stamp"] = SD_local_t;
		out["Red_Chamber_Lamp_Manual_Ctrl"] = SD_R_Lamp_Manual_Ctrl;
		out["Red_Chamber_Lamp_ON"] = SD_R_Lamp_ON;
		out["FarRed_Chamber_Lamp_Manual_Ctrl"] = SD_FR_Lamp_Manual_Ctrl;
		out["FarRed_Chamber_Lamp_ON"] = SD_FR_Lamp_ON;
		out["FarRed_1_LEDs_Manual_Ctrl"] = SD_FR_1_LEDs_Manual_Ctrl;
		out["FarRed_1_LEDs_ON"] = SD_FR_1_LEDs_ON;
		out["FarRed_1_LEDs_PWM_Duty_Cycle"] = SD_FR_1_LEDs_PWM_Duty_Cycle;
		out["FarRed_2_LEDs_Manual_Ctrl"] = SD_FR_2_LEDs_Manual_Ctrl;
		out["FarRed_2_LEDs_ON"] = SD_FR_2_LEDs_ON;
		out["FarRed_2_LEDs_PWM_Duty_Cycle"] = SD_FR_2_LEDs_PWM_Duty_Cycle;
		out["Red_Chamber_Fan_Manual_Ctrl"] = SD_R_Fan_Manual_Ctrl;
		out["Red_Chamber_Chamber_Fan_ON"] = SD_R_Chamber_Fan_ON;
		out["FarRed_Chamber_Fan_Manual_Ctrl"] = SD_FR_Fan_Manual_Ctrl;
		out["Red_Chamber_Chamber_Fan_ON"] = SD_FR_Chamber_Fan_ON;
		out["Temperature_Red_Chamber"] = TempAvg_R;
		out["Relative_Humidity_Red_Chamber"] = RHAvg_R;
		out["Temperature_FarRed_Chamber"] = TempAvg_FR;
		out["Relative_Humidity_FarRed_Chamber"] = RHAvg_FR;
		out["Sensors_OK"] = SensorsOKIoT;
	};
	
	

	////// Start RTC
	if (debug) { Serial.println(F("Starting RTC...")); }
	if (!rtc.begin()) {
		if (debug) { Serial.println(F("RTC start fail")); }
	}
	

	//////// If internet, start NTP client engine
	//////// and update RTC and system time
	//////// If no internet, get time form RTC
	if (debug) { Serial.println(F("Starting NTP client engine...")); }
	timeClient.begin();
	Serial.print(F("Trying to update NTP time..."));
	if (!GetNTPTime()) {
		GetRTCTime();
		Serial.println(F("\nTime updated from RTC"));
	}
	else { Serial.println(F("\nTime updated from NTP")); }





	////// Start SHT31 Temp and RH sensor
	// Red Chamber sensor
	if (debug) { Serial.println(F("Starting Temp/RH sensor Red...")); }
	digitalWrite(I2C_Select_0_PIN, LOW);
	if (sht3x.begin() != 0) {
		if (debug) { Serial.println(F("Failed to initialize Red sensor, please confirm wire connection")); }
	}
	if (debug) {
		Serial.print(F("Red sensor serial number: "));
		Serial.println(sht3x.readSerialNumber());
	}
	if (!sht3x.softReset()) {
		if (debug) { Serial.println(F("Failed to initialize Red sensor....")); }
	}
	
	// Far red Chamber sensor
	if (debug) { Serial.println(F("Starting Temp/RH sensor Far red...")); }
	digitalWrite(I2C_Select_0_PIN, HIGH);
	if (sht3x.begin() != 0) {
		if (debug) { Serial.println(F("Failed to initialize Far red sensor, please confirm wire connection")); }
	}
	if (debug) {
		Serial.print(F("Far red sensor serial number: "));
		Serial.println(sht3x.readSerialNumber());
	}
	if (!sht3x.softReset()) {
		if (debug) { Serial.println(F("Failed to initialize Far red sensor....")); }
	}
	digitalWrite(I2C_Select_0_PIN, LOW);

	if (debug) { Serial.println(F("Setup done!")); }
}





//////////////////////////////////////////////////////////////////////////
// The loop function runs over and over again until power down or reset //
//////////////////////////////////////////////////////////////////////////
void loop() {
	if (debug) { Serial.println(F("Loop start")); }


	////// State 0. Test internet connection; if not, try to connect.
	if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(); }
	if (WiFi.status() != WL_CONNECTED &&
		UTC_t - t_WiFiCnxTry > WiFiCnx_frq) {
		Serial.println(F("Loop reconect try"));
		WiFi.begin(ssid, password);

		t_WiFiCnxTry = UTC_t;
	}

	
	////// State 1. Keep the Iot engine runing
	thing.handle();


	////// State 2. Get time from RTC time
	GetRTCTime();
	if (debug) { Serial.println((String)"Time: " + h + ":" + m + ":" + s); }


	////// State 3. Update RTC time from NTP server at midnight
	if (h == 0 &&
		m == 0 &&
		s == 0 &&
		UTC_t != LastNTP) {
		GetNTPTime();
	}


	////// State 4. Test if it is time to read Temp, RH and Fan values
	////// AND record sensor values for 5-minute averages (each 20 seconds)
	if ((s % 20 == 0) && (s != LastSum)) {
		// Read Red chamber sensor
		digitalWrite(I2C_Select_0_PIN, LOW);
		sht3x_data = sht3x.readTemperatureAndHumidity(sht3x.eRepeatability_High);
		if (sht3x_data.ERR == 0) {
			Temp_R = sht3x_data.TemperatureC;
			RH_R = sht3x_data.Humidity;
			bitWrite(SensorsOK, 0, 1);
			bitWrite(SensorsOK, 1, 1);
		}
		else {
			Temp = -1;
			RH = -1;
			bitWrite(SensorsOK, 0, 0);
			bitWrite(SensorsOK, 1, 0);
		}

		// Read Farred chamber sensor
		digitalWrite(I2C_Select_0_PIN, HIGH);
		sht3x_data = sht3x.readTemperatureAndHumidity(sht3x.eRepeatability_High);
		if (sht3x_data.ERR == 0) {
			Temp_FR = sht3x_data.TemperatureC;
			RH_FR = sht3x_data.Humidity;
			bitWrite(SensorsOK, 2, 1);
			bitWrite(SensorsOK, 3, 1);
		}
		else {
			Temp = -1;
			RH = -1;
			bitWrite(SensorsOK, 2, 0);
			bitWrite(SensorsOK, 3, 0);
		}
		digitalWrite(I2C_Select_0_PIN, LOW);
		
		// Record if sensor reads were OK
		SensorsOKAvg = SensorsOKAvg & SensorsOK;
		if ( debug && (!bitRead(SensorsOK, 0) || !bitRead(SensorsOK, 1) || !bitRead(SensorsOK, 2) || !bitRead(SensorsOK, 3)) ) {
			Serial.println(F("At least 1 sensor read failed"));
			Serial.print(F("SensorOK byte: "));
			Serial.println(SensorsOK, BIN);
			Serial.print(F("R_Temp: "));
			Serial.println(Temp_R);
			Serial.print(F("R_RH: "));
			Serial.println(RH_R);
			Serial.print(F("FR_Temp: "));
			Serial.println(Temp_FR);
			Serial.print(F("FR_RH: "));
			Serial.println(RH_FR);
		}

		// Add new values to sum
		TempSum_R += Temp_R;
		RHSum_R += RH_R;
		TempSum_FR += Temp_FR;
		RHSum_FR += RH_FR;
		R_Fan_ON_Sum += (int)R_Fan_ON;
		FR_Fan_ON_Sum += (int)FR_Fan_ON;

		// Update Shift registers
		LastSum = m;
		SumNum += 1;
	}


	delay(1000);
	printWifiStatus();


}

void printWifiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print your board's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");
}