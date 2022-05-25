/*
 Name:		Main.ino
 Created:	12/14/2021 10:19:58 PM
 Author:	Aarón I. Vélez Ramírez
*/





//////////
// Pins //
//////////
// Pins 11, 12 and 13 are used for WiFi feather wing
const int SD_CS_PIN = 10;				// SD on Addlogger M0 Feather 
const int R_Chamber_Actinic_PIN = 14;		// Power Actinic Lamp in Red chamber. Level shifter channel 1.
const int FR_Chamber_Actinic_PIN = 15;		// Power Actinic Lamp in Farred chamber.  Level shifter channel 2.
const int I2C_Select_0_PIN = 9;		// I2C multiplexer digital select line
const int FR_1_LEDs_PIN = 16;			// Power Farred LEDs, circuit 1.  Level shifter channel 3.
const int FR_2_LEDs_PIN = 17;			// Power Farred LEDs, circuit 2.  Level shifter channel 4.
const int R_Chamber_Fan_PIN = 18;		// Power Fan in Red Chamber.  Level shifter channel 5.
const int FR_Chamber_Fan_PIN = 19;		// Power Fan in Farred Chamber.  Level shifter channel 6.
const int FR_1_LEDs_PWM_PIN = 5;		// PWM  line to dim FarRed LEDs, circuit 1.  Level shifter channel 7.
const int FR_2_LEDs_PWM_PIN = 6;		// PWM  line to dim FarRed LEDs, circuit 2.  Level shifter channel 8.






//////////////////////////////////////////////////////////////////
////// Libraries and its associated constants and variables //////
//////////////////////////////////////////////////////////////////


////// Credentials_R-FR_Chamber.h is a user-created library containing paswords, IDs and credentials
#include "Credentials_R-FR_Chamber.h"
#ifdef WiFi_SSID_is_HEX
const bool ssidIsHex = true;
const char ssidHEX[] = WIFI_SSID_HEX;
char ssid[64];
#else
const bool ssidIsHex = false;
const char ssidHEX[] = "";
const char ssid[] = WIFI_SSID;
#endif
const char password[] = WIFI_PASSWD;
const char iot_server[] = IoT_SERVER;
const char iot_user[] = IoT_USER;
const char iot_device[] = IoT_DEVICE;
const char iot_credential[] = IoT_CREDENTIAL;
const char iot_data_bucket[] = IoT_DATA_BUCKET;


////// Comunication libraries
#include <Wire.h>
#include <SPI.h>
#include <WiFiNINA.h>		// USE ADAFRUIT VERSSION!
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
char line[500];
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
//#include <DFRobot_SHT3x.h>
//DFRobot_SHT3x sht3x(&Wire,/*address=*/0x44,/*RST=*/4); // secondary I2C address 0x44
//DFRobot_SHT3x::sRHAndTemp_t sht3x_data;



////// Library for I2C ADC
#include <SparkFun_ADS1015_Arduino_Library.h>
ADS1015 adcSensor;



////// PID library
#include <PID_v1.h>






//////////////////////////////////////////
////// User Constants and Variables //////
//////////////////////////////////////////


////// Station IDs & Constants
const int StaNum = 7;
String StaType = F("M0_WiFiNA_RTC-PCF8523_Thinger");
String StaName = F("R-FR Chamber LANGEBIO");
String Firmware = F("v1.0.0");
//const double VRef = 3.3;
bool debug = true;


////// Log File & Headers
const char* FileName[] = { "2020.txt", "2021.txt", "2022.txt", "2023.txt", "2024.txt", "2025.txt", "2026.txt", "2027.txt", "2028.txt", "2029.txt",
			"2030.txt", "2031.txt", "2032.txt", "2033.txt", "2034.txt", "2035.txt", "2036.txt", "2037.txt", "2038.txt", "2039.txt",
			"2040.txt", "2041.txt", "2042.txt", "2043.txt", "2044.txt", "2045.txt", "2046.txt", "2047.txt", "2048.txt", "2049.txt",
			"2050.txt", "2051.txt", "2052.txt", "2053.txt", "2054.txt", "2055.txt", "2056.txt", "2057.txt", "2058.txt", "2059.txt",
			"2060.txt", "2061.txt", "2062.txt", "2063.txt", "2064.txt", "2065.txt", "2066.txt", "2067.txt", "2068.txt", "2069.txt",
			"2070.txt", "2071.txt", "2072.txt", "2073.txt", "2074.txt", "2075.txt", "2076.txt", "2077.txt", "2078.txt", "2079.txt",
			"2080.txt", "2081.txt", "2082.txt", "2083.txt", "2084.txt", "2085.txt", "2086.txt", "2087.txt", "2088.txt", "2089.txt",
			"2090.txt", "2091.txt", "2092.txt", "2093.txt", "2094.txt", "2095.txt", "2096.txt", "2097.txt", "2098.txt", "2099.txt",
			"2100.txt" };
const String Headers = F("UTC_UNIX_t\tLocal_UNIX_t\tyear\tmonth\tday\thour\tminute\tsecond\t\
AC_OK\tBatVolt\t\
R_Actinic_ON\tFR_Actinic_ON\t\
R_Fan_ON\tFR_Fan_ON\t\
FR_1_LEDs_ON\tFR_2_LEDs_ON\t\
FR_1_LEDs_PWM\tFR_2_LEDs_PWM\t\
R_Actinic_Manual_Ctrl\tFR_Actinic_Manual_Ctrl\t\
FR_1_LEDs_Manual_Ctrl\tFR_2_LEDs_Manual_Ctrl\t\
R_Fan_Manual_Ctrl\tFR_Fan_Manual_Ctrl\t\
SensorsOK\t\
SentIoT");
const int HeaderN = 26;	// cero indexed
String LogString = "";


////// M0 ADC constants
const int n = 100; // measure n times the ADC input for averaging
double sum = 0; // shift register to hold ADC data


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
const int WiFiCnx_frq = 150;  // (re)connection to internet frequency in seconds

byte SensorsOK = B00000000;     // Byte variable to store real time sensor status
byte SensorsOKAvg = B00000001;  // Byte variable to store SD card average sensor status
int SensorsOKIoT = 0;           // Variable to send sensor status in decimal format

time_t t_DataBucket = 0;             // Last time Data was sent to bucket (in UNIX time format) 
const int DataBucket_frq = 150;       // Data bucket update frequency in seconds (must be more than 60)


////// Lamps, LEDs and Fan control variables
/// Variables for Actinic Lamp in Red Chamber
bool R_Actinic_Manual_Ctrl = false;
bool R_Actinic_Manual_ON = false;
bool R_Actinic_ON = false;
bool R_Actinic_Period_1 = true;
bool R_Actinic_Period_2 = false;
//bool R_Actinic_Period_3 = false;
unsigned int R_Actinic_Period_1_ON = 0;
unsigned int R_Actinic_Period_2_ON = 0;
//unsigned int R_Actinic_Period_3_ON = 0;
unsigned int R_Actinic_Period_1_OFF = 0;
unsigned int R_Actinic_Period_2_OFF = 0;
//unsigned int R_Actinic_Period_3_OFF = 0;
/// Variables for Actinic Lamp in Farred Chamber
bool FR_Actinic_Manual_Ctrl = false;
bool FR_Actinic_Manual_ON = false;
bool FR_Actinic_ON = false;
bool FR_Actinic_Period_1 = true;
bool FR_Actinic_Period_2 = false;
//bool FR_Actinic_Period_3 = false;
unsigned int FR_Actinic_Period_1_ON = 0;
unsigned int FR_Actinic_Period_2_ON = 0;
//unsigned int FR_Actinic_Period_3_ON = 0;
unsigned int FR_Actinic_Period_1_OFF = 0;
unsigned int FR_Actinic_Period_2_OFF = 0;
//unsigned int FR_Actinic_Period_3_OFF = 0;
/// Variables for environmental control
bool Temp_Ctrl = true;	// If false, control is over Relative humidity
double R_Temp_Set = 20;
double FR_Temp_Set = 20;
double R_RH_Set = 70;
double FR_RH_Set = 70;
/// Variables for FarRed LEDs
// Circuit 1
bool FR_1_LEDs_Manual_Ctrl = false;
bool FR_1_LEDs_Manual_ON = false;
bool FR_1_LEDs_ON = false;
int FR_1_LEDs_PWM_Duty_Cycle = 0;
bool FR_1_LEDs_Period_1 = true;
bool FR_1_LEDs_Period_2 = false;
//bool FR_1_LEDs_Period_3 = false;
unsigned int FR_1_LEDs_Period_1_ON = 0;
unsigned int FR_1_LEDs_Period_2_ON = 0;
//unsigned int FR_1_LEDs_Period_3_ON = 0;
unsigned int FR_1_LEDs_Period_1_OFF = 0;
unsigned int FR_1_LEDs_Period_2_OFF = 0;
//unsigned int FR_1_LEDs_Period_3_OFF = 0;
// Circuit 2
bool FR_2_LEDs_Manual_Ctrl = false;
bool FR_2_LEDs_Manual_ON = false;
bool FR_2_LEDs_ON = false;
int FR_2_LEDs_PWM_Duty_Cycle = 0;
bool FR_2_LEDs_Period_1 = true;
bool FR_2_LEDs_Period_2 = false;
//bool FR_2_LEDs_Period_3 = false;
unsigned int FR_2_LEDs_Period_1_ON = 0;
unsigned int FR_2_LEDs_Period_2_ON = 0;
//unsigned int FR_2_LEDs_Period_3_ON = 0;
unsigned int FR_2_LEDs_Period_1_OFF = 0;
unsigned int FR_2_LEDs_Period_2_OFF = 0;
//unsigned int FR_2_LEDs_Period_3_OFF = 0;
/// Variables for Fans
bool R_Fan_Manual_Ctrl = false;
bool R_Fan_Manual_ON = false;
double R_Fan_ON_t = 0;
bool R_Fan_ON = false;
bool FR_Fan_Manual_Ctrl = false;
bool FR_Fan_Manual_ON = false;
double FR_Fan_ON_t = 0;
bool FR_Fan_ON = false;


////// Measured instantaneous variables
double IoT_Temp_R = -1;        // Air temperature RED chamber
double IoT_RH_R = -1;          // Air RH value RED chamber
double IoT_Temp_FR = -1;        // Air temperature FARRED chamber 
double IoT_RH_FR = -1;          // Air RH value FARRED chamber
bool AC_OK = true;			// Monitor AC power supply OK
double USBVolt = 0;			// USB voltage


////// Variables to store sum for eventual averaging
//Environmental
int AC_OK_Sum = 0;
double USBVolt_Sum = 0;
// Actinic lamps
int R_Actinic_Manual_Ctrl_Sum = 0;
int R_Actinic_ON_Sum = 0;
int FR_Actinic_Manual_Ctrl_Sum = 0;
int FR_Actinic_ON_Sum = 0;
// Farred LEDs
int FR_1_LEDs_Manual_Ctrl_Sum = 0;
int FR_1_LEDs_ON_Sum = 0;
int FR_1_LEDs_PWM_Duty_Cycle_Sum = 0;
int FR_2_LEDs_Manual_Ctrl_Sum = 0;
int FR_2_LEDs_ON_Sum = 0;
int FR_2_LEDs_PWM_Duty_Cycle_Sum = 0;
// Fans
int R_Fan_Manual_Ctrl_Sum = 0;
int FR_Fan_Manual_Ctrl_Sum = 0;
int R_Fan_ON_Sum = 0;
int FR_Fan_ON_Sum = 0;


////// Values to be logged. They will be the average over the last 5 minutes
//Environmental
double AC_OK_Avg = 0;
double USBVolt_Avg = 0;
// Actinic lamps
double R_Actinic_Manual_Ctrl_Avg = 0;
double R_Actinic_ON_Avg = 0;
double FR_Actinic_Manual_Ctrl_Avg = 0;
double FR_Actinic_ON_Avg = 0;
// Farred LEDs
double FR_1_LEDs_Manual_Ctrl_Avg = 0;
double FR_1_LEDs_ON_Avg = 0;
double FR_1_LEDs_PWM_Duty_Cycle_Avg = 0;
double FR_2_LEDs_Manual_Ctrl_Avg = 0;
double FR_2_LEDs_ON_Avg = 0;
double FR_2_LEDs_PWM_Duty_Cycle_Avg = 0;
// Fans
double R_Fan_Manual_Ctrl_Avg = 0;
double FR_Fan_Manual_Ctrl_Avg = 0;
double R_Fan_ON_Avg = 0;
double FR_Fan_ON_Avg = 0;


////// PID variables and controls
const int WindowSize = 600;		// Windows size in seconds. Each n seconds the PID sets how much of that time the fan is ON, remainder is OFF.
const int MinOnTime = 60;		// Fan is ON at least this number of seconds regardless PID
const int MinOffTime = 6;		// Fan is not turned OFF if OFF time will be less than this value in seconds
time_t windowStartTime = 0;

const double Kp = 2; // 
const double Ki = 5; //
const double Kd = 1; //

PID R_Temp_PID(&IoT_Temp_R, &R_Fan_ON_t, &R_Temp_Set, Kp, Ki, Kd, AUTOMATIC, REVERSE);
PID FR_Temp_PID(&IoT_Temp_FR, &FR_Fan_ON_t, &FR_Temp_Set, Kp, Ki, Kd, AUTOMATIC, REVERSE);
PID R_RH_PID(&IoT_RH_R, &R_Fan_ON_t, &R_RH_Set, Kp, Ki, Kd, AUTOMATIC, REVERSE);
PID FR_RH_PID(&IoT_RH_FR, &FR_Fan_ON_t, &FR_RH_Set, Kp, Ki, Kd, AUTOMATIC, REVERSE);

//////////////////////////////////////////////////////////////////////////
// the setup function runs once when you press reset or power the board //
//////////////////////////////////////////////////////////////////////////
void setup() {

	////// Start General Libraries
	Serial.begin(115200);
	delay(6000);
	if (debug = true) { Serial.print(F("\nSetup start\n")); }
	Wire.begin();
	SPI.begin();


	////// Start WiFi
	if (debug = true) { Serial.print(F("Conecting to WiFi")); }
	WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
	
#ifdef WiFi_SSID_is_HEX
	String ssidStr = HexString2ASCIIString(ssidHEX);
	ssidStr.toCharArray(ssid, sizeof(ssid) + 1);
#endif
	
	if (debug) {
		Serial.print(F("SSID name: "));
		Serial.println(ssid);
	}
	
	// Try to connect to WiFi; if not successful, reset WiFi board
	if (password == "") { WiFi.begin(ssid); }
		else { WiFi.begin(ssid, password); }
		delay(1000);
		Serial.print(F("WiFi status: "));
		Serial.println(WiFi.status());
	
	if (WiFi.status() != WL_CONNECTED) {
		WiFiConnectTry();
	}
	else {
		Serial.println(F("Connected to Internet!!!"));
	}
	


	////// Set pin modes of pins not associated with libraries
	//analogReadResolution(12);
	// Actinic light
	pinMode(R_Chamber_Actinic_PIN, OUTPUT);
	digitalWrite(R_Chamber_Actinic_PIN, LOW);
	pinMode(FR_Chamber_Actinic_PIN, OUTPUT);
	digitalWrite(FR_Chamber_Actinic_PIN, LOW);
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
	if (debug) {
		Serial.print(F("Starting SD card: "));
		Serial.println( sd.begin(SD_CS_PIN, SD_SCK_MHZ) );
	}
	else {
		sd.begin(SD_CS_PIN, SD_SCK_MHZ);
	}
	// Reserve RAM memory for large and dynamic String object
	// used in SD file write/read
	// (prevents heap RAM framgentation)
	LogString.reserve(HeaderN * 7);
	str.reserve(HeaderN * 7);


	////// Load Control variables from SD card
	if (true) { // (just a container to collapse/organize code)
		debug = (bool)Get_Setpoint("Debug.txt");
		// Red Actinic Light
		R_Actinic_Manual_Ctrl = (bool)Get_Setpoint("R_Actinic_Manual_Ctrl.txt");
		R_Actinic_Manual_ON = (bool)Get_Setpoint("R_Actinic_Manual_ON.txt");
		R_Actinic_Period_1 = (bool)Get_Setpoint("R_Actinic_Period_1.txt");
		R_Actinic_Period_2 = (bool)Get_Setpoint("R_Actinic_Period_2.txt");
		//R_Actinic_Period_3 = (bool)Get_Setpoint("R_Actinic_Period_3.txt");
		R_Actinic_Period_1_ON = (int)Get_Setpoint("R_Actinic_Period_1_ON.txt");
		R_Actinic_Period_2_ON = (int)Get_Setpoint("R_Actinic_Period_2_ON.txt");
		//R_Actinic_Period_3_ON = (int)Get_Setpoint("R_Actinic_Period_3_ON.txt");
		R_Actinic_Period_1_OFF = (int)Get_Setpoint("R_Actinic_Period_1_OFF.txt");
		R_Actinic_Period_2_OFF = (int)Get_Setpoint("R_Actinic_Period_2_OFF.txt");
		//R_Actinic_Period_3_OFF = (int)Get_Setpoint("R_Actinic_Period_3_OFF.txt");
		// Far-red Actinic Light
		FR_Actinic_Manual_Ctrl = (bool)Get_Setpoint("FR_Actinic_Manual_Ctrl.txt");
		FR_Actinic_Manual_ON = (bool)Get_Setpoint("FR_Actinic_Manual_ON.txt");
		FR_Actinic_Period_1 = (bool)Get_Setpoint("FR_Actinic_Period_1.txt");
		FR_Actinic_Period_2 = (bool)Get_Setpoint("FR_Actinic_Period_2.txt");
		//FR_Actinic_Period_3 = (bool)Get_Setpoint("FR_Actinic_Period_3.txt");
		FR_Actinic_Period_1_ON = (int)Get_Setpoint("FR_Actinic_Period_1_ON.txt");
		FR_Actinic_Period_2_ON = (int)Get_Setpoint("FR_Actinic_Period_2_ON.txt");
		//FR_Actinic_Period_3_ON = (int)Get_Setpoint("FR_Actinic_Period_3_ON.txt");
		FR_Actinic_Period_1_OFF = (int)Get_Setpoint("FR_Actinic_Period_1_OFF.txt");
		FR_Actinic_Period_2_OFF = (int)Get_Setpoint("FR_Actinic_Period_2_OFF.txt");
		//FR_Actinic_Period_3_OFF = (int)Get_Setpoint("FR_Actinic_Period_3_OFF.txt");
		// Environmental control
		Temp_Ctrl = (bool)Get_Setpoint("Temp_Ctrl.txt");
		R_Temp_Set = Get_Setpoint("R_Temp_Set.txt");
		FR_Temp_Set = Get_Setpoint("FR_Temp_Set.txt");
		R_RH_Set = Get_Setpoint("R_RH_Set.txt");
		FR_RH_Set = Get_Setpoint("FR_RH_Set.txt");
		// Far-red LEDs 1
		FR_1_LEDs_Manual_Ctrl = (bool)Get_Setpoint("FR_1_LEDs_Manual_Ctrl.txt");
		FR_1_LEDs_Manual_ON = (bool)Get_Setpoint("FR_1_LEDs_Manual_ON.txt");
		FR_1_LEDs_PWM_Duty_Cycle = (int)Get_Setpoint("FR_1_LEDs_PWM_Duty_Cycle.txt");
		FR_1_LEDs_Period_1 = (bool)Get_Setpoint("FR_1_LEDs_Period_1.txt");
		FR_1_LEDs_Period_2 = (bool)Get_Setpoint("FR_1_LEDs_Period_2.txt");
		//FR_1_LEDs_Period_3 = (bool)Get_Setpoint("FR_1_LEDs_Period_3.txt");
		FR_1_LEDs_Period_1_ON = (int)Get_Setpoint("FR_1_LEDs_Period_1_ON.txt");
		FR_1_LEDs_Period_2_ON = (int)Get_Setpoint("FR_1_LEDs_Period_2_ON.txt");
		//FR_1_LEDs_Period_3_ON = (int)Get_Setpoint("FR_1_LEDs_Period_3_ON.txt");
		FR_1_LEDs_Period_1_OFF = (int)Get_Setpoint("FR_1_LEDs_Period_1_OFF.txt");
		FR_1_LEDs_Period_2_OFF = (int)Get_Setpoint("FR_1_LEDs_Period_2_OFF.txt");
		//FR_1_LEDs_Period_3_OFF = (int)Get_Setpoint("FR_1_LEDs_Period_3_OFF.txt");
		// Far-red LEDs 2
		FR_2_LEDs_Manual_Ctrl = (bool)Get_Setpoint("FR_2_LEDs_Manual_Ctrl.txt");
		FR_2_LEDs_Manual_ON = (bool)Get_Setpoint("FR_2_LEDs_Manual_ON.txt");
		FR_2_LEDs_PWM_Duty_Cycle = (int)Get_Setpoint("FR_2_LEDs_PWM_Duty_Cycle.txt");
		FR_2_LEDs_Period_1 = (bool)Get_Setpoint("FR_2_LEDs_Period_1.txt");
		FR_2_LEDs_Period_2 = (bool)Get_Setpoint("FR_2_LEDs_Period_2.txt");
		//FR_2_LEDs_Period_3 = (bool)Get_Setpoint("FR_2_LEDs_Period_3.txt");
		FR_2_LEDs_Period_1_ON = (int)Get_Setpoint("FR_2_LEDs_Period_1_ON.txt");
		FR_2_LEDs_Period_2_ON = (int)Get_Setpoint("FR_2_LEDs_Period_2_ON.txt");
		//FR_2_LEDs_Period_3_ON = (int)Get_Setpoint("FR_2_LEDs_Period_3_ON.txt");
		FR_2_LEDs_Period_1_OFF = (int)Get_Setpoint("FR_2_LEDs_Period_1_OFF.txt");
		FR_2_LEDs_Period_2_OFF = (int)Get_Setpoint("FR_2_LEDs_Period_2_OFF.txt");
		//FR_2_LEDs_Period_3_OFF = (int)Get_Setpoint("FR_2_LEDs_Period_3_OFF.txt");
	}
	

	////// Configure IoT
	if (debug) { Serial.println(F("Configuring IoT...")); }
	thing.add_wifi(ssid, password);


	////// Define input resources
	if (true) {	// (just a container to collapse/organize code)
		// Debug
		thing["Debug"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("Debug.txt"); }
			else {
				Set_Setpoint("Debug.txt", (float)in);
				debug = in;
			}
		};
		// Environmental IoT Data
		thing["R_Env_IoT_Data_in"] << [](pson& in) {
			IoT_Temp_R = in["IoT_Temp_R"];
			IoT_RH_R = in["IoT_RH_R"];
		};
		thing["FR_Env_IoT_Data_in"] << [](pson& in) {
			IoT_Temp_FR = in["IoT_Temp_FR"];
			IoT_RH_FR = in["IoT_RH_FR"];
		};
		// Red Actinic Light
		thing["R_Actinic_Manual_Ctrl"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("R_Actinic_Manual_Ctrl.txt"); }
			else {
				Set_Setpoint("R_Actinic_Manual_Ctrl.txt", (float)in);
				R_Actinic_Manual_Ctrl = in;
			}
		};
		thing["R_Actinic_Manual_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("R_Actinic_Manual_ON.txt"); }
			else {
				Set_Setpoint("R_Actinic_Manual_ON.txt", (float)in);
				R_Actinic_Manual_ON = in;
			}
		};
		thing["R_Actinic_Period_1"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("R_Actinic_Period_1.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_1.txt", (float)in);
				R_Actinic_Period_1 = in;
			}
		};
		thing["R_Actinic_Period_2"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("R_Actinic_Period_2.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_2.txt", (float)in);
				R_Actinic_Period_2 = in;
			}
		};
		/*
		thing["R_Actinic_Period_3"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("R_Actinic_Period_3.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_3.txt", (float)in);
				R_Actinic_Period_3 = in;
			}
		};
		*/
		thing["R_Actinic_Period_1_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("R_Actinic_Period_1_ON.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_1_ON.txt", (float)in);
				R_Actinic_Period_1_ON = in;
			}
		};
		thing["R_Actinic_Period_2_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("R_Actinic_Period_2_ON.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_2_ON.txt", (float)in);
				R_Actinic_Period_2_ON = in;
			}
		};
		/*
		thing["R_Actinic_Period_3_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("R_Actinic_Period_3_ON.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_3_ON.txt", (float)in);
				R_Actinic_Period_3_ON = in;
			}
		};
		*/
		thing["R_Actinic_Period_1_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("R_Actinic_Period_1_OFF.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_1_OFF.txt", (float)in);
				R_Actinic_Period_1_OFF = in;
			}
		};
		thing["R_Actinic_Period_2_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("R_Actinic_Period_2_OFF.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_2_OFF.txt", (float)in);
				R_Actinic_Period_2_OFF = in;
			}
		};
		/*
		thing["R_Actinic_Period_3_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("R_Actinic_Period_3_OFF.txt"); }
			else {
				Set_Setpoint("R_Actinic_Period_3_OFF.txt", (float)in);
				R_Actinic_Period_3_OFF = in;
			}
		};
		*/
		// Far-red Actinic Light
		thing["FR_Actinic_Manual_Ctrl"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_Actinic_Manual_Ctrl.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Manual_Ctrl.txt", (float)in);
				FR_Actinic_Manual_Ctrl = in;
			}
		};
		thing["FR_Actinic_Manual_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_Actinic_Manual_ON.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Manual_ON.txt", (float)in);
				FR_Actinic_Manual_ON = in;
			}
		};
		thing["FR_Actinic_Period_1"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_Actinic_Period_1.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_1.txt", (float)in);
				FR_Actinic_Period_1 = in;
			}
		};
		thing["FR_Actinic_Period_2"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_Actinic_Period_2.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_2.txt", (float)in);
				FR_Actinic_Period_2 = in;
			}
		};
		/*
		thing["FR_Actinic_Period_3"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_Actinic_Period_3.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_3.txt", (float)in);
				FR_Actinic_Period_3 = in;
			}
		};
		*/
		thing["FR_Actinic_Period_1_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_Actinic_Period_1_ON.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_1_ON.txt", (float)in);
				FR_Actinic_Period_1_ON = in;
			}
		};
		thing["FR_Actinic_Period_2_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_Actinic_Period_2_ON.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_2_ON.txt", (float)in);
				FR_Actinic_Period_2_ON = in;
			}
		};
		/*
		thing["FR_Actinic_Period_3_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_Actinic_Period_3_ON.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_3_ON.txt", (float)in);
				FR_Actinic_Period_3_ON = in;
			}
		};
		*/
		thing["FR_Actinic_Period_1_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_Actinic_Period_1_OFF.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_1_OFF.txt", (float)in);
				FR_Actinic_Period_1_OFF = in;
			}
		};
		thing["FR_Actinic_Period_2_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_Actinic_Period_2_OFF.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_2_OFF.txt", (float)in);
				FR_Actinic_Period_2_OFF = in;
			}
		};
		/*
		thing["FR_Actinic_Period_3_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_Actinic_Period_3_OFF.txt"); }
			else {
				Set_Setpoint("FR_Actinic_Period_3_OFF.txt", (float)in);
				FR_Actinic_Period_3_OFF = in;
			}
		};
		*/
		// Environmental control
		thing["Temp_Ctrl"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("Temp_Ctrl.txt"); }
			else {
				Set_Setpoint("Temp_Ctrl.txt", (float)in);
				Temp_Ctrl = in;
			}
		};
		thing["R_Temp_Set"] << [](pson& in) {
			if (in.is_empty()) { in = Get_Setpoint("R_Temp_Set.txt"); }
			else {
				Set_Setpoint("R_Temp_Set.txt", (float)in);
				R_Temp_Set = in;
			}
		};
		thing["FR_Temp_Set"] << [](pson& in) {
			if (in.is_empty()) { FR_Temp_Set = IoT_Temp_R; }
			else {
				FR_Temp_Set = IoT_Temp_R;
			}
		};
		/*
		thing["FR_Temp_Set"] << [](pson& in) {
			if (in.is_empty()) { in = Get_Setpoint("FR_Temp_Set.txt"); }
			else {
				Set_Setpoint("FR_Temp_Set.txt", (float)in);
				FR_Temp_Set = in;
			}
		};
		*/

		thing["R_RH_Set"] << [](pson& in) {
			if (in.is_empty()) { in = Get_Setpoint("R_RH_Set.txt"); }
			else {
				Set_Setpoint("R_RH_Set.txt", (float)in);
				R_RH_Set = in;
			}
		};
		thing["FR_RH_Set"] << [](pson& in) {
			if (in.is_empty()) { in = Get_Setpoint("FR_RH_Set.txt"); }
			else {
				Set_Setpoint("FR_RH_Set.txt", (float)in);
				FR_RH_Set = in;
			}
		};
		// Far-red LEDs 1
		thing["FR_1_LEDs_Manual_Ctrl"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_1_LEDs_Manual_Ctrl.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Manual_Ctrl.txt", (float)in);
				FR_1_LEDs_Manual_Ctrl = in;
			}
		};
		thing["FR_1_LEDs_Manual_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_1_LEDs_Manual_ON.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Manual_ON.txt", (float)in);
				FR_1_LEDs_Manual_ON = in;
			}
		};
		thing["FR_1_LEDs_PWM_Duty_Cycle"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_PWM_Duty_Cycle.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_PWM_Duty_Cycle.txt", (float)in);
				FR_1_LEDs_PWM_Duty_Cycle = in;
			}
		};
		thing["FR_1_LEDs_Period_1"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_1_LEDs_Period_1.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_1.txt", (float)in);
				FR_1_LEDs_Period_1 = in;
			}
		};
		thing["FR_1_LEDs_Period_2"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_1_LEDs_Period_2.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_2.txt", (float)in);
				FR_1_LEDs_Period_2 = in;
			}
		};
		/*
		thing["FR_1_LEDs_Period_3"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_1_LEDs_Period_3.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_3.txt", (float)in);
				FR_1_LEDs_Period_3 = in;
			}
		};
		*/
		thing["FR_1_LEDs_Period_1_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_Period_1_ON.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_1_ON.txt", (float)in);
				FR_1_LEDs_Period_1_ON = in;
			}
		};
		thing["FR_1_LEDs_Period_2_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_Period_2_ON.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_2_ON.txt", (float)in);
				FR_1_LEDs_Period_2_ON = in;
			}
		};
		/*
		thing["FR_1_LEDs_Period_3_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_Period_3_ON.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_3_ON.txt", (float)in);
				FR_1_LEDs_Period_3_ON = in;
			}
		};
		*/
		thing["FR_1_LEDs_Period_1_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_Period_1_OFF.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_1_OFF.txt", (float)in);
				FR_1_LEDs_Period_1_OFF = in;
			}
		};
		thing["FR_1_LEDs_Period_2_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_Period_2_OFF.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_2_OFF.txt", (float)in);
				FR_1_LEDs_Period_2_OFF = in;
			}
		};
		/*
		thing["FR_1_LEDs_Period_3_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_1_LEDs_Period_3_OFF.txt"); }
			else {
				Set_Setpoint("FR_1_LEDs_Period_3_OFF.txt", (float)in);
				FR_1_LEDs_Period_3_OFF = in;
			}
		};
		*/
		// Far-red LEDs 2
		thing["FR_2_LEDs_Manual_Ctrl"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_2_LEDs_Manual_Ctrl.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Manual_Ctrl.txt", (float)in);
				FR_2_LEDs_Manual_Ctrl = in;
			}
		};
		thing["FR_2_LEDs_Manual_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_2_LEDs_Manual_ON.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Manual_ON.txt", (float)in);
				FR_2_LEDs_Manual_ON = in;
			}
		};
		thing["FR_2_LEDs_PWM_Duty_Cycle"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_PWM_Duty_Cycle.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_PWM_Duty_Cycle.txt", (float)in);
				FR_2_LEDs_PWM_Duty_Cycle = in;
			}
		};
		thing["FR_2_LEDs_Period_1"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_2_LEDs_Period_1.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_1.txt", (float)in);
				FR_2_LEDs_Period_1 = in;
			}
		};
		thing["FR_2_LEDs_Period_2"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_2_LEDs_Period_2.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_2.txt", (float)in);
				FR_2_LEDs_Period_2 = in;
			}
		};
		/*
		thing["FR_2_LEDs_Period_3"] << [](pson& in) {
			if (in.is_empty()) { in = (bool)Get_Setpoint("FR_2_LEDs_Period_3.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_3.txt", (float)in);
				FR_2_LEDs_Period_3 = in;
			}
		};
		*/
		thing["FR_2_LEDs_Period_1_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_Period_1_ON.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_1_ON.txt", (float)in);
				FR_2_LEDs_Period_1_ON = in;
			}
		};
		thing["FR_2_LEDs_Period_2_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_Period_2_ON.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_2_ON.txt", (float)in);
				FR_2_LEDs_Period_2_ON = in;
			}
		};
		/*
		thing["FR_2_LEDs_Period_3_ON"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_Period_3_ON.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_3_ON.txt", (float)in);
				FR_2_LEDs_Period_3_ON = in;
			}
		};
		*/
		thing["FR_2_LEDs_Period_1_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_Period_1_OFF.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_1_OFF.txt", (float)in);
				FR_2_LEDs_Period_1_OFF = in;
			}
		};
		thing["FR_2_LEDs_Period_2_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_Period_2_OFF.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_2_OFF.txt", (float)in);
				FR_2_LEDs_Period_2_OFF = in;
			}
		};
		/*
		thing["FR_2_LEDs_Period_3_OFF"] << [](pson& in) {
			if (in.is_empty()) { in = (int)Get_Setpoint("FR_2_LEDs_Period_3_OFF.txt"); }
			else {
				Set_Setpoint("FR_2_LEDs_Period_3_OFF.txt", (float)in);
				FR_2_LEDs_Period_3_OFF = in;
			}
		};
		*/
	}


	////// Define output resources
	thing["RT_R_Actinic_ON"] >> [](pson& out) { out = R_Actinic_ON; };
	thing["RT_FR_Actinic_ON"] >> [](pson& out) { out = FR_Actinic_ON; };
	thing["RT_FR_1_LEDs_ON"] >> [](pson& out) { out = FR_1_LEDs_ON; };
	thing["RT_FR_2_LEDs_ON"] >> [](pson& out) { out = FR_2_LEDs_ON; };
	thing["RT_R_Fan_ON "] >> [](pson& out) { out = R_Fan_ON; };
	thing["RT_FR_Fan_ON "] >> [](pson& out) { out = FR_Fan_ON; };
	thing["RT_AC_OK"] >> [](pson& out) { out = AC_OK; };
	thing["RT_USB_Voltage"] >> [](pson& out) { out = USBVolt; };
	
	thing["Avg_Data"] >> [](pson& out) {
		out["Time_Stamp"] = SD_local_t;
		out["Red_Chamber_Actinic_Manual_Ctrl"] = R_Actinic_Manual_Ctrl_Avg;
		out["Red_Chamber_Actinic_ON"] = R_Actinic_ON_Avg;
		out["FarRed_Chamber_Actinic_Manual_Ctrl"] = FR_Actinic_Manual_Ctrl_Avg;
		out["FarRed_Chamber_Actinic_ON"] = FR_Actinic_ON_Avg;
		out["FarRed_1_LEDs_Manual_Ctrl"] = FR_1_LEDs_Manual_Ctrl_Avg;
		out["FarRed_1_LEDs_ON"] = FR_1_LEDs_ON_Avg;
		out["FarRed_1_LEDs_PWM_Duty_Cycle"] = FR_1_LEDs_PWM_Duty_Cycle_Avg;
		out["FarRed_2_LEDs_Manual_Ctrl"] = FR_2_LEDs_Manual_Ctrl_Avg;
		out["FarRed_2_LEDs_ON"] = FR_2_LEDs_ON_Avg;
		out["FarRed_2_LEDs_PWM_Duty_Cycle"] = FR_2_LEDs_PWM_Duty_Cycle_Avg;
		out["Red_Chamber_Fan_Manual_Ctrl"] = R_Fan_Manual_Ctrl_Avg;
		out["Red_Chamber_Fan_ON"] = R_Fan_ON_Avg;
		out["FarRed_Chamber_Fan_Manual_Ctrl"] = FR_Fan_Manual_Ctrl_Avg;
		out["FarRed_Chamber_Fan_ON"] = FR_Fan_ON_Avg;
		out["Sensors_OK"] = SensorsOKIoT;
		out["AC_OK"] = AC_OK_Avg;
		out["USB_Voltage"] = USBVolt_Avg;

	};



	////// Start RTC
	if (debug) { Serial.println(F("Starting RTC...")); }
	if (rtc.begin()) {
		if (debug) { Serial.println(F("RTC start OK")); }
	}
	else {
		Serial.println(F("RTC start fail"));
	}
	delay(250); // Wait to I2C device to release bus. It helps to prevent I2C bus getting stuck in noisy line


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


	// Start I2C ADC
	if (debug) { Serial.println(F("Starting I2C ADC...")); }
	adcSensor.begin();							// default address is 0x48
	adcSensor.setGain(ADS1015_CONFIG_PGA_1);	// 1x gain   +/- 4.096V  1 bit = 2mV		
	if (debug) {
		Serial.print(F("ADC conected: "));
		Serial.println(adcSensor.isConnected());
	}
	delay(250); // Wait to I2C device to release bus. It helps to prevent I2C bus getting stuck in noisy line

	/*
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
	delay(250); // Wait to I2C device to release bus. It helps to prevent I2C bus getting stuck in noisy line

	// Far red Chamber sensor
	if (debug) { Serial.println(F("Starting Temp/RH sensor Far red...")); }
	digitalWrite(I2C_Select_0_PIN, HIGH);
	delay(1000);
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
	delay(250); // Wait to I2C device to release bus. It helps to prevent I2C bus getting stuck in noisy line
	digitalWrite(I2C_Select_0_PIN, LOW);
	*/

	////// Start PIDs
	windowStartTime = local_t;
	// Set a minimum ON Fan time of 150 seconds
	R_Temp_PID.SetOutputLimits(0, WindowSize);
	FR_Temp_PID.SetOutputLimits(0, WindowSize);
	R_RH_PID.SetOutputLimits(0, WindowSize);
	FR_RH_PID.SetOutputLimits(0, WindowSize);
	// Turn PID ON
	R_Temp_PID.SetMode(AUTOMATIC);
	R_RH_PID.SetMode(AUTOMATIC);
	FR_Temp_PID.SetMode(AUTOMATIC);
	FR_RH_PID.SetMode(AUTOMATIC);


	if (debug) { Serial.println(F("Setup done!")); }
}





//////////////////////////////////////////////////////////////////////////
// The loop function runs over and over again until power down or reset //
//////////////////////////////////////////////////////////////////////////
void loop() {
	if (debug) { Serial.println(F("Loop start")); }


	////// State 0. Test internet connection; if not, try to connect.
	//if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(); }
	if (WiFi.status() != WL_CONNECTED &&
		UTC_t - t_WiFiCnxTry > WiFiCnx_frq) {
		Serial.println(F("Disconnecting Wifi"));
		Serial.print(F("WiFi Status at disconnect: "));
		Serial.println( WiFi.disconnect() );
		Serial.println(F("Loop reconect try:"));
		WiFiConnectTry();

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


	////// State 4. Test if it is time to read Temp, RH values
	// 	   ADN test if AC current is OK
	////// AND record sensor values for 5-minute averages (each 20 seconds)
	if ((s % 20 == 0) && (s != LastSum)) {
		if (debug) { Serial.println(F("Time to read sensors")); }

		// Debug Temp and RH IoT values
		if (debug) {
			Serial.print(F("Red IoT Temp: "));
			Serial.println(IoT_Temp_R, 2);
			Serial.print(F("Red IoT RH: "));
			Serial.println(IoT_RH_R, 2);

			Serial.print(F("Far-Red IoT Temp: "));
			Serial.println(IoT_Temp_FR, 2);
			Serial.print(F("Far-Red IoT RH: "));
			Serial.println(IoT_RH_FR, 2);
		}

		
		// Read I2C ADC
		if (debug) { Serial.println(F("Reading I2C ADC")); }
		// With gain 1x, 1 bit  = 2mV
		// IF AC is OK, supplu voltage is 5V, and after voltage divider is 2.5V.
		// Threshold to consider AC OK is 4.75V
		USBVolt = adcSensor.getSingleEnded(0) * adcSensor.getMultiplier();
		if (USBVolt > 2375) {
			// M0 supply voltage is higher than 4.75V (2375 mV x 2)
			// Mark AC supply OK
			AC_OK = true;
		}
		else { AC_OK = false; }
		// Record if ADC sensor is OK
		if (adcSensor.isConnected() == 1) {
			bitWrite(SensorsOK, 0, 1);
		}
		else {
			bitWrite(SensorsOK, 0, 0);
		}
		delay(250); // Wait to I2C device to release bus. It helps to prevent I2C bus getting stuck in noisy line


		// Record if sensor reads were OK
		SensorsOKAvg = SensorsOKAvg & SensorsOK;
		if (debug && !bitRead(SensorsOK, 0)) {
			Serial.println(F("At least 1 sensor read failed"));
			Serial.print(F("SensorOK byte: "));
			Serial.println(SensorsOK, BIN);
			Serial.print(F("ADC_USBVolt: "));
			Serial.println(USBVolt);
		}

		// Add new values to sum
		// Environment
		AC_OK_Sum += AC_OK;
		USBVolt_Sum += USBVolt;
		// Actinic
		R_Actinic_Manual_Ctrl_Sum += (int)R_Actinic_Manual_Ctrl;
		R_Actinic_ON_Sum += (int)R_Actinic_ON;
		FR_Actinic_Manual_Ctrl_Sum += (int)FR_Actinic_Manual_Ctrl;
		FR_Actinic_ON_Sum += (int)FR_Actinic_ON;
		// Farred LEDs
		FR_1_LEDs_Manual_Ctrl_Sum += (int)FR_1_LEDs_Manual_Ctrl;
		FR_1_LEDs_ON_Sum += (int)FR_1_LEDs_ON;
		FR_1_LEDs_PWM_Duty_Cycle_Sum += FR_1_LEDs_PWM_Duty_Cycle;
		FR_2_LEDs_Manual_Ctrl_Sum += (int)FR_2_LEDs_Manual_Ctrl;
		FR_2_LEDs_ON_Sum += (int)FR_2_LEDs_ON;
		FR_2_LEDs_PWM_Duty_Cycle_Sum += FR_2_LEDs_PWM_Duty_Cycle;
		// Fans
		R_Fan_Manual_Ctrl_Sum += (int)R_Fan_Manual_Ctrl;
		FR_Fan_Manual_Ctrl_Sum += (int)FR_Fan_Manual_Ctrl;
		R_Fan_ON_Sum += (int)R_Fan_ON;
		FR_Fan_ON_Sum += (int)FR_Fan_ON;



		// Update Shift registers
		LastSum = m;
		SumNum += 1;
	}


	////// State 5. PID control over Temp or RH.
	////// As Fan control is a relay, the PID output is transformed into a "control window" ON time percentage
	/// Test if it is time to shift control window
	if (local_t - windowStartTime > WindowSize) {
		windowStartTime = local_t;
	}
	/// Red Chamber Fan control
	if (R_Fan_Manual_Ctrl) {
		digitalWrite(R_Chamber_Fan_PIN, R_Fan_Manual_ON);
		R_Fan_ON = R_Fan_Manual_ON;
	}
	else {
		// Execute PID algorithms
		if (Temp_Ctrl) { R_Temp_PID.Compute(); }
		else { R_RH_PID.Compute(); }
		// Execute PID output for Red chamber
		if (!R_Fan_ON && local_t - windowStartTime <= R_Fan_ON_t && R_Fan_ON_t >= MinOnTime) {
			digitalWrite(R_Chamber_Fan_PIN, HIGH);
			R_Fan_ON = true;
		}
		else if (R_Fan_ON && local_t - windowStartTime >= R_Fan_ON_t + MinOffTime) {
			digitalWrite(R_Chamber_Fan_PIN, LOW);
			R_Fan_ON = false;
		}
	}
	/// Farred Chamber Fan control
	if (FR_Fan_Manual_Ctrl) {
		digitalWrite(FR_Chamber_Fan_PIN, FR_Fan_Manual_ON);
		FR_Fan_ON = FR_Fan_Manual_ON;
	}
	else {
		// Execute PID algorithms
		if (Temp_Ctrl) { FR_Temp_PID.Compute(); }
		else { FR_RH_PID.Compute(); }
		// Execute PID output for Farred chamber
		if (!FR_Fan_ON && local_t - windowStartTime <= FR_Fan_ON_t && FR_Fan_ON_t >= MinOnTime) {
			digitalWrite(FR_Chamber_Fan_PIN, HIGH);
			FR_Fan_ON = true;
		}
		else if (FR_Fan_ON && local_t - windowStartTime >= FR_Fan_ON_t + MinOffTime) {
			digitalWrite(FR_Chamber_Fan_PIN, LOW);
			FR_Fan_ON = false;
		}
	}



	////// State 6. Set IoT Control over Actinic Lamps and LEDs
	// Red Chamber Actinic Lamp
	if (R_Actinic_Manual_Ctrl) { R_Actinic_ON = R_Actinic_Manual_ON; }
	else {
		bool Period_1 = false;
		if (R_Actinic_Period_1) { Period_1 = SetLights(R_Actinic_Period_1_ON, R_Actinic_Period_1_OFF); }
		bool Period_2 = false;
		if (R_Actinic_Period_2) { Period_2 = SetLights(R_Actinic_Period_2_ON, R_Actinic_Period_2_OFF); }
		/*
		bool Period_3 = false;
		if (R_Actinic_Period_3) { Period_3 = SetLights(R_Actinic_Period_3_ON, R_Actinic_Period_3_OFF); }
		*/
		if (Period_1 || Period_2 /*|| Period_3*/) {
			R_Actinic_ON = true;
		}
		else {
			R_Actinic_ON = false;
		}
	}
	digitalWrite(R_Chamber_Actinic_PIN, R_Actinic_ON);

	// Farred Chamber Actinic Lamp
	if (FR_Actinic_Manual_Ctrl) { FR_Actinic_ON = FR_Actinic_Manual_ON; }
	else {
		bool Period_1 = false;
		if (FR_Actinic_Period_1) { Period_1 = SetLights(FR_Actinic_Period_1_ON, FR_Actinic_Period_1_OFF); }
		bool Period_2 = false;
		if (FR_Actinic_Period_2) { Period_2 = SetLights(FR_Actinic_Period_2_ON, FR_Actinic_Period_2_OFF); }
		/*
		bool Period_3 = false;
		if (FR_Actinic_Period_3) { Period_3 = SetLights(FR_Actinic_Period_3_ON, FR_Actinic_Period_3_OFF); }
		*/
		if (Period_1 || Period_2 /*|| Period_3*/) {
			FR_Actinic_ON = true;
		}
		else {
			FR_Actinic_ON = false;
		}
	}
	digitalWrite(FR_Chamber_Actinic_PIN, FR_Actinic_ON);

	// Farred LEDs Circuit 1
	if (FR_1_LEDs_Manual_Ctrl) { FR_1_LEDs_ON = FR_1_LEDs_Manual_ON; }
	else {
		bool Period_1 = false;
		if (FR_1_LEDs_Period_1) { Period_1 = SetLights(FR_1_LEDs_Period_1_ON, FR_1_LEDs_Period_1_OFF); }
		bool Period_2 = false;
		if (FR_1_LEDs_Period_2) { Period_2 = SetLights(FR_1_LEDs_Period_2_ON, FR_1_LEDs_Period_2_OFF); }
		/*
		bool Period_3 = false;
		if (FR_1_LEDs_Period_3) { Period_3 = SetLights(FR_1_LEDs_Period_3_ON, FR_1_LEDs_Period_3_OFF); }
		*/
		if (Period_1 || Period_2 /*|| Period_3*/) {
			FR_1_LEDs_ON = true;
		}
		else {
			FR_1_LEDs_ON = false;
		}
	}
	digitalWrite(FR_1_LEDs_PIN, FR_1_LEDs_ON);
	if (FR_1_LEDs_ON) { analogWrite(FR_1_LEDs_PWM_PIN, FR_1_LEDs_PWM_Duty_Cycle); }
	else { analogWrite(FR_1_LEDs_PWM_PIN, 0); }

	// Farred LEDs Circuit 2
	if (FR_2_LEDs_Manual_Ctrl) { FR_2_LEDs_ON = FR_2_LEDs_Manual_ON; }
	else {
		bool Period_1 = false;
		if (FR_2_LEDs_Period_1) { Period_1 = SetLights(FR_2_LEDs_Period_1_ON, FR_2_LEDs_Period_1_OFF); }
		bool Period_2 = false;
		if (FR_2_LEDs_Period_2) { Period_2 = SetLights(FR_2_LEDs_Period_2_ON, FR_2_LEDs_Period_2_OFF); }
		/*
		bool Period_3 = false;
		if (FR_2_LEDs_Period_3) { Period_3 = SetLights(FR_2_LEDs_Period_3_ON, FR_2_LEDs_Period_3_OFF); }
		*/
		if (Period_1 || Period_2 /*|| Period_3*/) {
			FR_2_LEDs_ON = true;
		}
		else {
			FR_2_LEDs_ON = false;
		}
	}
	digitalWrite(FR_2_LEDs_PIN, FR_2_LEDs_ON);
	if (FR_2_LEDs_ON) { analogWrite(FR_2_LEDs_PWM_PIN, FR_2_LEDs_PWM_Duty_Cycle); }
	else { analogWrite(FR_2_LEDs_PWM_PIN, 0); }







	////// State 7. Test if it is time to compute  averages and record in SD card (each 5 minutes)
	if (((m % 5) == 0) && (m != LastLog) && (SumNum > 0)) {
		if (debug) { Serial.println(F("Time to calculate averages and log to SD card")); }
		// Calculate averages
		AC_OK_Avg = AC_OK_Sum / SumNum;
		USBVolt_Avg = USBVolt_Sum / SumNum;
		// Actinic
		R_Actinic_Manual_Ctrl_Avg = R_Actinic_Manual_Ctrl_Sum / SumNum;
		R_Actinic_ON_Avg = R_Actinic_ON_Sum / SumNum;
		FR_Actinic_Manual_Ctrl_Avg = FR_Actinic_Manual_Ctrl_Sum / SumNum;
		FR_Actinic_ON_Avg = FR_Actinic_ON_Sum / SumNum;
		// Farred LEDs
		FR_1_LEDs_Manual_Ctrl_Avg = FR_1_LEDs_Manual_Ctrl_Sum / SumNum;
		FR_1_LEDs_ON_Avg = FR_1_LEDs_ON_Sum / SumNum;
		FR_1_LEDs_PWM_Duty_Cycle_Avg = FR_1_LEDs_PWM_Duty_Cycle_Sum / SumNum;
		FR_2_LEDs_Manual_Ctrl_Avg = FR_2_LEDs_Manual_Ctrl_Sum / SumNum;
		FR_2_LEDs_ON_Avg = FR_2_LEDs_ON_Sum / SumNum;
		FR_2_LEDs_PWM_Duty_Cycle_Avg = FR_2_LEDs_PWM_Duty_Cycle_Sum / SumNum;
		// Fans
		R_Fan_Manual_Ctrl_Avg = R_Fan_Manual_Ctrl_Sum / SumNum;
		FR_Fan_Manual_Ctrl_Avg = FR_Fan_Manual_Ctrl_Sum / SumNum;
		R_Fan_ON_Avg = R_Fan_ON_Sum / SumNum;
		FR_Fan_ON_Avg = FR_Fan_ON_Sum / SumNum;


		// Open Year LogFile (create if not available)
		if (!sd.exists(FileName[yr - 2020])) {
			LogFile.open((FileName[yr - 2020]), O_RDWR | O_CREAT); // Create file

			// Add Metadata
			LogFile.println(F("Start position of last line send to IoT:\t1"));
			// Tabs added to prevent line ending with 0. Line ending with 0 indicates that line needs to be sent to IoT.
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("Metadata:"));
			LogFile.println((String)"Station Number\t" + StaNum + "\t\t\t");
			LogFile.println((String)"Station Name\t" + StaName + "\t\t\t");
			LogFile.println((String)"Station Type\t" + StaType + "\t\t\t");
			LogFile.println((String)"Firmware\t" + Firmware + "\t\t\t");
			LogFile.println((String)"Log file creation local UNIX time:\t" + (unsigned long)local_t + "\t\t\t");
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("\t\t\t"));
			LogFile.println(F("\t\t\t"));

			LogFile.println(Headers); // Add Headers
		}
		else {
			LogFile.close();
			LogFile.open(FileName[yr - 2020], O_RDWR); // Open file
			LogFile.seekEnd(); // Set position to end of file
		}


		// Log to SD card
		LogString = (String)(unsigned long)UTC_t + "\t" + (unsigned long)local_t + "\t" + yr + "\t" + mo + "\t" + dy + "\t" + h + "\t" + m + "\t" + s + "\t" +
			String(AC_OK_Avg, 2) + "\t" + String(USBVolt_Avg, 4) + "\t" +
			String(R_Actinic_ON_Avg, 2) + "\t" + String(FR_Actinic_ON_Avg, 2) + "\t" +
			String(R_Fan_ON_Avg, 2) + "\t" + String(FR_Fan_ON_Avg, 2) + "\t" +
			String(FR_1_LEDs_ON_Avg, 2) + "\t" + String(FR_2_LEDs_ON_Avg, 2) + "\t" +
			String(FR_1_LEDs_PWM_Duty_Cycle) + "\t" + String(FR_2_LEDs_PWM_Duty_Cycle) + "\t" +
			String(R_Actinic_Manual_Ctrl_Avg, 2) + "\t" + String(FR_Actinic_Manual_Ctrl_Avg, 2) + "\t" +
			String(FR_1_LEDs_Manual_Ctrl_Avg, 2) + "\t" + String(FR_2_LEDs_Manual_Ctrl_Avg, 2) + "\t" +
			String(R_Fan_Manual_Ctrl_Avg, 2) + "\t" + String(R_Fan_Manual_Ctrl_Avg, 2) + "\t" +
			String(SensorsOKAvg, DEC) + "\t" +
			"0";
		LogFile.println(LogString); // Prints Log string to SD card file "LogFile.txt"
		LogFile.close(); // Close SD card file to save changes


		// Reset Shift Registers
		LastLog = m;

		AC_OK_Sum = 0;
		USBVolt_Sum = 0;
		
		R_Actinic_Manual_Ctrl_Sum = 0;
		R_Actinic_ON_Sum = 0;
		FR_Actinic_Manual_Ctrl_Sum = 0;
		FR_Actinic_ON_Sum = 0;
		
		FR_1_LEDs_Manual_Ctrl_Sum = 0;
		FR_1_LEDs_ON_Sum = 0;
		FR_1_LEDs_PWM_Duty_Cycle_Sum = 0;
		FR_2_LEDs_Manual_Ctrl_Sum = 0;
		FR_2_LEDs_ON_Sum = 0;
		FR_2_LEDs_PWM_Duty_Cycle_Sum = 0;
		
		R_Fan_Manual_Ctrl_Sum = 0;
		FR_Fan_Manual_Ctrl_Sum = 0;
		R_Fan_ON_Sum = 0;
		FR_Fan_ON_Sum = 0;

		SumNum = 0;
		SensorsOKAvg = B00011111;
	}



	/*
	Serial.print(F("Payload ready: "));
	Serial.println(PayloadRdy);

	Serial.print(F("UTC: "));
	Serial.println((int)UTC_t);

	Serial.print(F("t_DataBucket: "));
	Serial.println((int)t_DataBucket);

	Serial.print(F("DataBucket_frq: "));
	Serial.println(DataBucket_frq);

	Serial.print(F("Logic test: "));
	Serial.println(!PayloadRdy &&
		UTC_t - t_DataBucket > DataBucket_frq - 15);

	Serial.print(F("Root open: "));
	Serial.println(root.open("/"));

	root.rewind();

	Serial.print(F("Open file, try 1: "));
	Serial.println(LogFile.openNext(&root, O_RDWR));
	LogFile.printName(&Serial);
	Serial.print(F("Open file, try 2: "));
	Serial.println(LogFile.openNext(&root, O_RDWR));
	LogFile.printName(&Serial);
	Serial.println();

	root.close();
	LogFile.close();
	*/

	////// State 8. Test if there is data available to be sent to IoT cloud
	// Only test if Payload is not ready AND
	// the next DataBucket upload oportunity is in 15 sec 
	if (!PayloadRdy &&
		UTC_t - t_DataBucket > DataBucket_frq - 15) {
		if (debug) { Serial.println(F("Time to look in LogFiles for a Payload")); }
		for (int i = yr - 1; i <= yr; i++) {
			if (debug) {
				Serial.println(F("For loop start"));
				Serial.print(F("Looking for year LogFile: "));
				Serial.println(i);
			}
			if (PayloadRdy) { break; }
			if (LogFile.open(FileName[i - 2020])) {
				if(debug) {
					Serial.print(F("Opened file: "));
					LogFile.printName(&Serial);
					Serial.println();
				}
				LogFile.rewind();
				LogFile.fgets(line, sizeof(line));     // Get first line
				str = String(line);
				if (debug) {
					LogFile.printName(&Serial);
					Serial.println();
					Serial.print(F("File first line: "));
					Serial.println(str.substring(0, str.indexOf("\r")));
				}
				str = str.substring(str.indexOf("\t"), str.indexOf("\r"));
				if (str == "Done") {	// Skips file if year data is all sent to IoT
					LogFile.close();
				}
				else {
					position = str.toInt();	// Sets file position to start of last line sent to IoT
					LogFile.seekSet(position);	// Set position to last line sent to LoRa
					// Read each line until a line not sent to IoT is found
					while (LogFile.available()) {
						position = LogFile.curPosition();  // START position of current line
						int len = LogFile.fgets(line, sizeof(line));
						if (line[len - 2] == '0') {
							str = String(line); // str is the payload, next state test if there is internet connection to send payload to IoT
							if (debug) {
								Serial.println("Loteria, data to send to IoT found!");
								Serial.print(F("Data: "));
								Serial.println(str.substring(0, str.indexOf("\r")));
							}
							PayloadRdy = true;
							LogFile.close();
							break;
						}
					}
					LogFile.close();
				}
			}
			LogFile.close();	// 
		}
	}



	////// State 9. Test if there is Internet and a Payload to sent SD data to IoT
	if (true) {
		if (debug) {
			Serial.print(F("Thing connected, payload ready and enought time has enlapsed: "));
			Serial.println(thing.is_connected() &&
				PayloadRdy &&
				UTC_t - t_DataBucket > DataBucket_frq);
		}
		if (thing.is_connected() &&
			PayloadRdy &&
			UTC_t - t_DataBucket > DataBucket_frq) {
			t_DataBucket = UTC_t; // Record Data Bucket update TRY; even if it is not succesfful
			// extract data from payload string (str)
			if (debug) {
				Serial.print(F("RAW payload string: "));
				Serial.println(str);
			}
			for (int i = 0; i < HeaderN; i++) {
				String buffer = str.substring(0, str.indexOf('\t'));
				if (i != 7) { 	// Do not read seconds info
					if (i == 0) {   // UTC UNIX Time
						// Do not send to IoT
					}
					else if (i == 1) {  // Local UNIX time
						SD_local_t = buffer.toInt();
					}
					else if (i == 2) {
						yrIoT = buffer.toInt();
					}
					else if (i == 3) {
						moIoT = buffer.toInt();
					}
					else if (i == 4) {
						dyIoT = buffer.toInt();
					}
					else if (i == 5) {
						hIoT = buffer.toInt();
					}
					else if (i == 6) {
						mIoT = buffer.toInt();
					}
					else if (i == 8) {	// AC_OK
						AC_OK_Avg = buffer.toFloat();
					}
					else if (i == 9) {	// Battery voltage
						USBVolt_Avg = buffer.toFloat();
					}
					else if (i == 10) { // Red chamber Actinic lamp ON?
						R_Actinic_ON_Avg = buffer.toInt();
					}
					else if (i == 11) { // Farred chamber Actinic lamp ON?
						FR_Actinic_ON_Avg = buffer.toInt();
					}
					else if (i == 12) { // Red chamber Fan ON?
						R_Fan_ON_Avg = buffer.toInt();
					}
					else if (i == 13) { // Farred chamber Fan ON?
						FR_Fan_ON_Avg = buffer.toInt();
					}
					else if (i == 14) { // FR LEDs 1 ON?
						FR_1_LEDs_ON_Avg = buffer.toInt();
					}
					else if (i == 15) { // FR LEDs 2 ON?
						FR_2_LEDs_ON_Avg = buffer.toInt();
					}
					else if (i == 16) { // FR LEDs 1 PWM
						FR_1_LEDs_PWM_Duty_Cycle_Avg = buffer.toInt();
					}
					else if (i == 17) { // FR LEDs 2 PWM
						FR_2_LEDs_PWM_Duty_Cycle_Avg = buffer.toInt();
					}
					else if (i == 18) { // Manual Actinic Ctrl in red chamber
						R_Actinic_Manual_Ctrl_Avg = buffer.toInt();
					}
					else if (i == 19) { // Manual Actinic Ctrl in farred chamber
						FR_Actinic_Manual_Ctrl_Avg = buffer.toInt();
					}
					else if (i == 20) { // Manual LEDs 1 Ctrl in red chamber
						FR_1_LEDs_Manual_Ctrl_Avg = buffer.toInt();
					}
					else if (i == 21) { // Manual LEDs 1 Ctrl in farred chamber
						FR_2_LEDs_Manual_Ctrl_Avg = buffer.toInt();
					}
					else if (i == 22) { // Manual Fan Ctrl in red chamber
						R_Fan_Manual_Ctrl_Avg = buffer.toInt();
					}
					else if (i == 23) { // Manual Fan Ctrl in farred chamber
						FR_Fan_Manual_Ctrl_Avg = buffer.toInt();
					}
					else if (i == 24) { // SensorsOK
						SensorsOKIoT = buffer.toInt();
					}
				}
				str = str.substring(str.indexOf('\t') + 1);
			}
			// send data to IoT. If succsessful, rewrite line in log File
			if (thing.write_bucket(iot_data_bucket, "Avg_Data", true) == 1) {
				if (debug) { Serial.println(F("Loteria, data on Cloud!!!")); }
				// Update line sent to IoT status
				if (debug) {
					Serial.print(F("IoT year: "));
					Serial.println(yrIoT);
					Serial.print(F("File name: "));
					Serial.println(FileName[yrIoT - 2020]);
				}
				LogFile.open(FileName[yrIoT - 2020], O_RDWR); // Open file containing the data just sent to IoT
				str = String(line);                     // Recover complete payload from original line
				str.setCharAt(str.length() - 2, '1');   // Replace 0 with 1, last characters are always "\r\n"
				LogFile.seekSet(position);              // Set position to start of line to be rewritten
				LogFile.println(str.substring(0, str.length() - 1));    // Remove last character ('\n') to prevent an empty line below rewritten line
				// Test if this line is last line of year Log File, if so, write "Done" at the end of first line
				if (moIoT == 12 && dyIoT == 31 && hIoT == 23 && mIoT == 55) {
					LogFile.rewind();
					LogFile.fgets(line, sizeof(line));     // Get first line
					str = String(line);
					str = str.substring(0, str.indexOf("\t"));
					str = (String)str + "\t" + "Done";
					LogFile.rewind();
					LogFile.println(str.substring(0, str.length() - 1));    // Remove last character ('\n') to prevent an empty line below rewritten line
					LogFile.close();
				}
				else {
					// Update start position of last line sent to IoT
					LogFile.rewind();
					LogFile.fgets(line, sizeof(line));     // Get first line
					str = String(line);
					str = str.substring(0, str.indexOf("\t"));
					str = (String)str + "\t" + position;
					LogFile.rewind();
					LogFile.println(str);
					LogFile.close();
				}
				PayloadRdy = false;		// Payload in cloud, str String destroyed in the process
			}
			PayloadRdy = false;		// Payload upload NOT succesful, yet str String destroyed in the process. Need to look for payload in SD card again
		}
	}



	if (debug) {
		printWifiStatus();
	}

}


// Function for WiFi SSID with non-ASCII characters
String HexString2ASCIIString(String hexstring) {
	String temp = "", sub = "", result;
	char buf[3];
	for (int i = 0; i < hexstring.length(); i += 2) {
		sub = hexstring.substring(i, i + 2);
		sub.toCharArray(buf, 3);
		char b = (char)strtol(buf, 0, 16);
		if (b == '\0')
			break;
		temp += b;
	}
	return temp;
}


// Function to connect to WiFi
void WiFiConnectTry() {
	
	// Reset WiFi board
	Serial.println(F("Resseting WiFi board"));
	digitalWrite(ESP32_RESETN, LOW);
	delay(500);
	digitalWrite(ESP32_RESETN, HIGH);
	delay(500);

	if (debug) {
		Serial.print(F("SSID name: "));
		Serial.println(ssid);
	}

	for (int i = 0; i <= 5; i++) {
		if (WiFi.status() != WL_CONNECTED) {

			if (password == "") { WiFi.begin(ssid); }
			else { WiFi.begin(ssid, password); }
			
			delay(1000);
			
			Serial.print(F("WiFi status: "));
			Serial.println(WiFi.status());			
		}
		else {
			Serial.println(F("\nConnected to internet!"));
			Serial.print(F("IP address: "));
			Serial.println(WiFi.localIP());
			break;
		}
		if (i == 5) {
			Serial.println(F("\nNo internet connection"));
			WiFi.disconnect();  // if no internet, disconnect. This prevents some boards to be busy only trying to connect.
		}
	}
}


// Function to monitor WiFi status
void printWifiStatus() {
	// print the SSID of the network you're attached to:
	Serial.print("SSID: ");
	Serial.println(WiFi.SSID());

	// print WiFi status
	Serial.print(F("WiFi status: "));
	Serial.println(WiFi.status());

	// Test 
	Serial.print(F("Test WiFi status results: "));
	Serial.println(WiFi.status() == WL_CONNECTED);

	// print your board's IP address:
	IPAddress ip = WiFi.localIP();
	Serial.print("IP Address: ");
	Serial.println(ip);

	// print the received signal strength:
	long rssi = WiFi.RSSI();
	Serial.print("signal strength (RSSI):");
	Serial.print(rssi);
	Serial.println(" dBm");

	// print Thing IoT connect status
	Serial.print(F("Thing IoT connect status: "));
	Serial.println(thing.is_connected());

}

