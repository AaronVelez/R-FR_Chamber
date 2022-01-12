/*
 Name:		Main.ino
 Created:	12/14/2021 10:19:58 PM
 Authors:	Aarón I. Vélez Ramírez and Irving J. García López
*/





//////////
// Pins //
//////////
const int SD_CS_PIN = 10;				// SD on Addlogger M0 Feather 
const int R_Chamber_Lamp_PIN = 16;		// Power Actinic Lamp in Red chamber
const int FR_Chamber_Lamp_PIN = 17;		// Power Actinic Lamp in Farred chamber
const int I2C_Select_0_PIN = 18;		// I2C multiplexer digital select line
const int I2C_Select_1_PIN = 19;		// I2C multiplexer digital select line
const int FR_1_LEDs_PIN = 9;			// Power Farred LEDs, circuit 1
const int FR_2_LEDs_PIN = 11;			// Power Farred LEDs, circuit 2
const int FR_1_LEDs_PWM_PIN = 12;		// PWM  line to dim FarRed LEDs, circuit 1
const int FR_2_LEDs_PWM_PIN = 13;		// PWM  line to dim FarRed LEDs, circuit 2
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
#define THINGER_SERVER iot_server   // Delete this line if using a free thinger account 
#define _DEBUG_   // Uncomment for debugging connection to Thinger
#define _DISABLE_TLS_     // Uncoment if needed (port 25202 closed, for example)
//#include <ThingerESP32.h>
//ThingerESP32 thing(iot_user, iot_device, iot_credential);
#include <ThingerWifi.h>
ThingerWifi thing(iot_user, iot_device, iot_credential);
/// <summary>
/// IMPORTANT
/// in file \thinger.io\src\ThingerClient.h at Line 355, the function handle_connection() was modified
/// to prevent the thinger handler to agresively try to reconnect to WiFi in case of a lost connection
/// This allows the device to keep monitoring environmental variables even if there is no network connection
/// </summary>


////// SD fat linrary used in order to use exFAT file system
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
String StaType = F("M0_WiFiNA_Thinger");
String StaName = F("R-FR Chamber CINVESTAV");
String Firmware = F("v1.0.0");
//const float VRef = 3.3;
bool debug = true;


////// Log File & Headers
String FileName = "";
const String Headers = F("UNIX_t\tyear\tmonth\tday\thour\tminute\tsecond\t\
AirTemp\tAirRH\t\
SoilVWC\tSoilTemp\tSoilEC\t\
LeafWetV\t\
DownThermocoupleV\tDownSapTemp\tUpThermocoupleV\tUpSapTemp\tSapFlow\t\
DendrometerV\tStemRadious\t\
ADCTemp\t\
SentLoRa");
const int HeaderN = 21;	// cero indexed
String LogString = "";


////// M0 ADC constants
const int n = 100; // measure n times the ADC input for averaging
float sum = 0; // shift register to hold ADC data


////// Time variables
//DateTime RTCnow;    // UTC Date-Time class from RTC DS3231 (not used here)
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
int LastSec = -1;           // Last second register (to update clock display each second)
int LastSum = -1;			// Last minute that variables were measured added to the sum for later averaging
int SumNum = 0;				// Number of times a variable value has beed added to the sum for later averaging
int LastLog = -1;			// Last minute that variables were loged to the SD card
bool PayloadRdy = false;	// Payload ready to send to IoT

time_t t_WiFiCnxTry = 0;      // Last time a (re)connection to internet happened
const int WiFiCnx_frq = 30;  // (re)connection to internet frequency in seconds

byte SensorsOK = B00000000;     // Byte variable to store real time sensor status
byte SensorsOKAvg = B00011111;  // Byte variable to store SD card average sensor status
int SensorsOKIoT = 0;           // Variable to send sensor status in decimal format

time_t t_DataBucket = 0;             // Last time Data was sent to bucket (in UNIX time format) 
const int DataBucket_frq = 150;       // Data bucket update frequency in seconds (must be more than 60)


////// Lamps, LEDs and Fan control variables
/// IoT control variables for Actinic Lamp in Red Chamber
bool R_Lamp_Manual_Ctrl = false;
bool R_Lamp_Manual_ON = false;
bool R_Lamp_Period_1 = true;
bool R_Lamp_Period_2 = false;
bool R_Lamp_Period_3 = false;
unsigned int R_Lamp_Period_1_ON = 0;
unsigned int R_Lamp_Period_2_ON = 0;
unsigned int R_Lamp_Period_3_ON = 0;
unsigned int R_Lamp_Period_1_OFF = 0;
unsigned int R_Lamp_Period_2_OFF = 0;
unsigned int R_Lamp_Period_3_OFF = 0;
/// IoT control variables for Actinic Lamp in Farred Chamber
bool FR_Lamp_Manual_Ctrl = false;
bool FR_Lamp_Manual_ON = false;
bool FR_Lamp_Period_1 = true;
bool FR_Lamp_Period_2 = false;
bool FR_Lamp_Period_3 = false;
unsigned int FR_Lamp_Period_1_ON = 0;
unsigned int FR_Lamp_Period_2_ON = 0;
unsigned int FR_Lamp_Period_3_ON = 0;
unsigned int FR_Lamp_Period_1_OFF = 0;
unsigned int FR_Lamp_Period_2_OFF = 0;
unsigned int FR_Lamp_Period_3_OFF = 0;
/// IoT control variables for FarRed LEDs
// Circuit 1
bool FR_1_LEDs_Manual_Ctrl = false;
int FR_1_LEDs_Manual_PWM = 0;
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
int FR_2_LEDs_Manual_PWM = 0;
bool FR_2_LEDs_Period_1 = true;
bool FR_2_LEDs_Period_2 = false;
bool FR_2_LEDs_Period_3 = false;
unsigned int FR_2_LEDs_Period_1_ON = 0;
unsigned int FR_2_LEDs_Period_2_ON = 0;
unsigned int FR_2_LEDs_Period_3_ON = 0;
unsigned int FR_2_LEDs_Period_1_OFF = 0;
unsigned int FR_2_LEDs_Period_2_OFF = 0;
unsigned int FR_2_LEDs_Period_3_OFF = 0;
/// IoT control variables for Fans
bool R_Fan_Manual_Ctrl = false;
bool R_Fan_Manual_ON = false;
bool FR_Fan_Manual_Ctrl = false;
bool FR_Fan_Manual_ON = false;
/// RT Control variables
bool R_Chamber_Lamp_Ctrl = false;
bool FR_Chamber_Lamp_Ctrl = false;
bool FR_1_LEDs_Ctrl = false;
bool FR_2_LEDs_Ctrl = false;
int FR_1_LEDs_PWM_Duty_Ctrl = 0;
int FR_2_LEDs_PWM_Duty_Ctrl = 0;
bool R_Chamber_Fan_Ctrl = false;
bool FR_Chamber_Fan_Ctrl = false;
/// Variables from SD card
bool SD_R_Chamber_Lamp_Ctrl = false;
bool SD_FR_Chamber_Lamp_Ctrl = false;
bool SD_FR_1_LEDs_Ctrl = false;
bool SD_FR_2_LEDs_Ctrl = false;
int SD_FR_1_LEDs_PWM_Duty_Ctrl = 0;
int SD_FR_2_LEDs_PWM_Duty_Ctrl = 0;
bool SD_R_Chamber_Fan_Ctrl = false;
bool SD_FR_Chamber_Fan_Ctrl = false;


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


////// Values to be logged. They will be the average over the last 5 minutes
float TempAvg_R = 0;
float RHAvg_R = 0;
float TempAvg_FR = 0;
float RHAvg_FR = 0;







//////////////////////////////////////////////////////////////////////////
// the setup function runs once when you press reset or power the board //
//////////////////////////////////////////////////////////////////////////
void setup() {

	////// Start General Libraries
	Serial.begin(115200);
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
	pinMode(I2C_Select_1_PIN, OUTPUT);
	digitalWrite(I2C_Select_1_PIN, LOW);
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
	thing["FR_LEDs_Ctrl"] << [](pson& in) {
		if (in.is_empty()) {
			in["FR_LEDs_Manual_Ctrl"] = FR_LEDs_Manual_Ctrl;
			in["FR_LEDs_Manual_PWM"] = FR_LEDs_Manual_PWM;
			in["FR_LEDs_Period_1"] = FR_LEDs_Period_1;
			in["FR_LEDs_Period_2"] = FR_LEDs_Period_2;
			in["FR_LEDs_Period_3"] = FR_LEDs_Period_3;
			in["FR_LEDs_Period_1_ON"] = FR_LEDs_Period_1_ON;
			in["FR_LEDs_Period_2_ON"] = FR_LEDs_Period_2_ON;
			in["FR_LEDs_Period_3_ON"] = FR_LEDs_Period_3_ON;
			in["FR_LEDs_Period_1_OFF"] = FR_LEDs_Period_1_OFF;
			in["FR_LEDs_Period_2_OFF"] = FR_LEDs_Period_2_OFF;
			in["FR_LEDs_Period_3_OFF"] = FR_LEDs_Period_3_OFF;
		}
		else {
			FR_LEDs_Manual_Ctrl = in["FR_LEDs_Manual_Ctrl"];
			FR_LEDs_Manual_PWM = in["FR_LEDs_Manual_PWM"];
			FR_LEDs_Period_1 = in["FR_LEDs_Period_1"];
			FR_LEDs_Period_2 = in["FR_LEDs_Period_2"];
			FR_LEDs_Period_3 = in["FR_LEDs_Period_3"];
			FR_LEDs_Period_1_ON = in["FR_LEDs_Period_1_ON"];
			FR_LEDs_Period_2_ON = in["FR_LEDs_Period_2_ON"];
			FR_LEDs_Period_3_ON = in["FR_LEDs_Period_3_ON"];
			FR_LEDs_Period_1_OFF = in["FR_LEDs_Period_1_OFF"];
			FR_LEDs_Period_2_OFF = in["FR_LEDs_Period_2_OFF"];
			FR_LEDs_Period_3_OFF = in["FR_LEDs_Period_3_OFF"];
		}
	};
	
	// Define output resources
	thing["RT_Temp_Red"] >> [](pson& out) { out = Temp_R; };
	thing["RT_RH_Red"] >> [](pson& out) { out = RH_R; };
	thing["RT_Temp_FarRed"] >> [](pson& out) { out = Temp_FR; };
	thing["RT_RH_FarRed"] >> [](pson& out) { out = RH_FR; };
	thing["Avg_Data"] >> [](pson& out) {
		out["Time_Stamp"] = SD_local_t;
		out["Temperature_Red_Chamber"] = TempAvg_R;
		out["Relative_Humidity_Red_Chamber"] = RHAvg_R;
		out["Temperature_FarRed_Chamber"] = TempAvg_FR;
		out["Relative_Humidity_FarRed_Chamber"] = RHAvg_FR;
		out["Red_Chamber_Lamp_ON"] = SD_R_Chamber_Lamp_ON;
		out["FarRed_Chamber_Lamp_ON"] = SD_FR_Chamber_Lamp_ON;
		out["FarRed_LEDs_PWM_Duty_Cycle"] = SD_FR_LEDs_PWM_Duty;
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

	if (debug) { Serial.println(F("Setup done!")); }
}





//////////////////////////////////////////////////////////////////////////
// The loop function runs over and over again until power down or reset //
//////////////////////////////////////////////////////////////////////////
void loop() {

}