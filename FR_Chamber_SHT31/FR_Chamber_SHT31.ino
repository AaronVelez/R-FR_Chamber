/*
 Name:		FR_Chamber_SHT31.ino
 Created:	5/11/2022 12:11:00 PM
 Author:	aivel
*/


//////////////////////////////////////////////////////////////////
////// Libraries and its associated constants and variables //////
//////////////////////////////////////////////////////////////////

////// Board library
#include "M5Core2.h"


////// GFX Free Fonts https://rop.nl/truetype2gfx/
#include "FreeSans5pt7b.h"
#include "FreeSans6pt7b.h"
#include "FreeSans7pt7b.h"
#include "FreeSans8pt7b.h"
//#include "FreeSans9pt7b.h" already defined, it can be used
#include "FreeSans10pt7b.h"
//#include "FreeSans12pt7b.h" already defined, it can be used
#include "FreeSans15pt7b.h"
#include "FreeSans20pt7b.h"


////// Credentials_Gas_Alarm_Photo_Lab.h is a user-created library containing paswords, IDs and credentials
#include "Credentials_FR_Chamber_Environment.h"
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
#include <WiFi.h>
#include <WiFiUdp.h>
WiFiUDP ntpUDP;


////// Iot Thinger
#define THINGER_SERVER iot_server   // Delete this line if using a free thinger account 
#define _DEBUG_   // Uncomment for debugging connection to Thinger
#define _DISABLE_TLS_     // Uncoment if needed (port 25202 closed, for example)
#include <ThingerESP32.h>
ThingerESP32 thing(iot_user, iot_device, iot_credential);
/// <summary>
/// IMPORTANT
/// in file \thinger.io\src\ThingerClient.h around Line 426, the function handle_connection() was modified
/// to prevent the thinger handler to agresively try to reconnect to WiFi in case of a lost connection
/// This allows the alarn to keep monitoring gas levels even if there is no network connection
/// </summary>


////// SD fat library used in order to use exFAT file system
////// Adapted to M5 stack acording to https://github.com/ArminPP/sdFAT-M5Stack
#include <SdFat.h>
#define SPI_SPEED SD_SCK_MHZ(25)
#define SD_CONFIG SdSpiConfig(TFCARD_CS_PIN, SHARED_SPI, SPI_SPEED) // TFCARD_CS_PIN is defined in M5Stack Config.h (Pin 4)
/// <summary>
/// IMPORTANT
/// in file SdFat\src\SdFatConfig.h at Line 100 set to cero for NON dedicated SPI:
/// #define ENABLE_DEDICATED_SPI 0
/// </summary>

SdExFat sd;
ExFile LogFile;
ExFile root;

char line[250];
String str = "";
unsigned int position = 0;


////// Time libraries
RTC_TimeTypeDef RTCtime;
RTC_DateTypeDef RTCDate;


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
DFRobot_SHT3x sht3x(&Wire,/*address=*/0x44,/*RST=*/4);
DFRobot_SHT3x::sRHAndTemp_t sht3x_data;





//////////////////////////////////////////
////// User Constants and Variables //////
//////////////////////////////////////////

////// Station IDs & Constants
const int Station_Number = 10;
String Processor = F("ESP32");
String IoT_Hardware = F("ESP32 WiFi");
String IoT_Software = F("Thinger ESP32");
String RTC_Hardware = F("BM8563");
String IoT_Asset_Type = F("Environment_Monitor");
String IoT_Group = F("LANGEBIO");
String IoT_Station_Name = F("FarRed Chamber Environment");
String Firmware = F("v1.0.0");

bool debug = false;

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
AirTemp\tAirRH\t\
USBVolt\tBatVolt\t\
SensorsOK\t\
SentIoT");
const int HeaderN = 13;	// Number of items in header (columns), Also used as a cero-indexed header index
String LogString = "";


////// Time variables
//DateTime RTCnow;    // UTC Date-Time class from RTC DS3231 (not used here)
time_t LastNTP;     // Last UTP time that the RTC was updated form NTP
time_t UTC_t;       // UTC UNIX time stamp
time_t local_t;     // Local time with DST adjust in UNIX time stamp format
time_t SD_local_t;  // Recorded UNIX time stamp
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
int LastSec = -1;           // Last second that gas sensor values where measured
int LastLcd = -1;           // Last time the screen was updated
int LastSum = -1;			// Last minute that variables were added to the sum for later averaging
int SumNum = 0;				// Number of times a variable value has beed added to the sum for later averaging
int LastLog = -1;			// Last minute that variables were loged to the SD card
bool PayloadRdy = false;	// Payload ready to send to IoT

time_t t_WiFiCnxTry = 0;      // Last time a (re)connection to internet happened
const int WiFiCnx_frq = 30;  // (re)connection to internet frequency in seconds

byte SensorsOK = B00000000;     // Byte variable to store real time sensor status
byte SensorsOKAvg = B00000001;  // Byte variable to store SD card average sensor status
int SensorsOKIoT = 0;           // Variable to send sensor status in decimal format

time_t t_DataBucket = 0;             // Last time Data was sent to bucket (in UNIX time format) 
const int DataBucket_frq = 150;       // Data bucket update frequency in seconds (must be more than 60)


////// Measured instantaneous variables
float Temp = -1;        // Air temperature read each minute
float RH = -1;          // Air RH value read each minute
float USBVolt = 0;		// USB voltage (VBUS in AXP192)
float BatVolt = 0;		// Battery voltage


////// Variables to store sum for eventual averaging
float TempSum = 0;
float RHSum = 0;
float USBVoltSum = 0;
float BatVoltSum = 0;


////// Values to be logged. They will be the average over the last 5 minutes
float TempAvg = 0;
float RHAvg = 0;
float USBVoltAvg = 0;
float BatVoltAvg = 0;





// the setup function runs once when you press reset or power the board
void setup() {
    ////// Initialize and setup M5Stack
    //M5.begin(true, true, true, true, kMBusModeOutput);
    M5.begin();
    Wire.begin(32, 33);
    M5.Lcd.println(F("M5 started"));

#ifdef WiFi_SSID_is_HEX
    String ssidStr = HexString2ASCIIString(ssidHEX);
    ssidStr.toCharArray(ssid, sizeof(ssid) + 1);
#endif

    M5.Lcd.print(F("SSID name: "));
    M5.Lcd.println(ssid);

    if (password == "") { WiFi.begin(ssid); }
    else { WiFi.begin(ssid, password); }

    WiFi.setAutoReconnect(false);
    M5.Lcd.print(F("Connecting to internet..."));
    // Test for 10 seconds if there is WiFi connection;
    // if not, continue to loop in order to monitor gas levels
    for (int i = 0; i <= 10; i++) {
        if (WiFi.status() != WL_CONNECTED) {
            M5.Lcd.print(".");
            delay(1000);
        }
        else {
            M5.Lcd.println(F("\nConnected to internet!"));
            break;
        }
        if (i == 10) {
            M5.Lcd.println(F("\nNo internet connection"));
            WiFi.disconnect();  // if no internet, disconnect. This prevents the board to be busy only trying to connect.
        }
    }


        ////// Initialize SD card
    M5.Lcd.println(F("Setting SD card..."));
    sd.begin(SD_CONFIG);
    // Reserve RAM memory for large and dynamic String object
    // used in SD file write/read
    // (prevents heap RAM framgentation)
    LogString.reserve(HeaderN * 7);
    str.reserve(HeaderN * 7);



    ////// Configure IoT
    M5.Lcd.println(F("Configuring IoT..."));
    if (password == "") { thing.add_wifi(ssid); }
    else { thing.add_wifi(ssid, password); }
    // Define input resources




    // Define output resources
    thing["FR_Env_IoT_Data_out"] >> [](pson& out) {
        out["IoT_Temp_FR"] = Temp;
        out["IoT_RH_FR"] = RH;
    };

    thing["RT_USB_Voltage"] >> [](pson& out) { out = USBVolt; };
    thing["RT_Battery_Voltage"] >> [](pson& out) { out = BatVolt; };
    thing["RT_SensorOK"] >> [](pson& out) { out = SensorsOK; };

    thing["Avg_Data"] >> [](pson& out) {
        out["Time_Stamp"] = SD_local_t;
        out["Temperature"] = TempAvg;
        out["Relative_Humidity"] = RHAvg;
        out["USB_Voltage"] = USBVoltAvg;
        out["Battery_Voltage"] = BatVoltAvg;
        out["Sensors_OK"] = SensorsOKIoT;
    };


    //////// If internet, start NTP client engine
    //////// and update RTC and system time
    //////// If no internet, get time form RTC
    M5.Lcd.println(F("Starting NTP client engine..."));
    timeClient.begin();
    M5.Lcd.print(F("Trying to update NTP time..."));
    // Try to update NTP time.
    // If not succesful , get RTc time and continue to loop in order to monitor gas levels
    if (!GetNTPTime()) {
        GetRTCTime();
        M5.Lcd.println(F("\nTime updated from RTC"));
    }
    else { M5.Lcd.println(F("\nTime updated from NTP")); }


    // Start SHT31 Temp and RH sensor
    M5.Lcd.println(F("Starting Temp/RH sensor..."));
    if (sht3x.begin() != 0) {
        M5.Lcd.println(F("Failed to initialize the chip, please confirm the wire connection"));
        delay(1000);
    }
    M5.Lcd.print(F("Chip serial number: "));
    M5.Lcd.println(sht3x.readSerialNumber());
    if (!sht3x.softReset()) {
        M5.Lcd.print(F("Failed to reset the chip...."));
    }

    // Setup finish message
    M5.Lcd.println(F("Setup done!"));
    M5.Lcd.println(F("\nStarting in "));
    for (int i = 0; i <= 5; i++) {
        M5.Lcd.println(5 - i);
        if (i != 5) { delay(1000); }
        else { delay(250); }
    }
    M5.Lcd.clear(BLACK);
}

// the loop function runs over and over again until power down or reset
void loop() {
    if (debug) {
        M5.Lcd.clear(BLACK);
        M5.Lcd.setTextDatum(TL_DATUM);
        M5.Lcd.setCursor(0, 10);
        M5.Lcd.setFreeFont(&FreeSans6pt7b);
        M5.Lcd.print(F("Loop start at: "));
        M5.Lcd.println(millis());
    }


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
    if (debug) { M5.Lcd.println((String)"Time: " + h + ":" + m + ":" + s); }


    ////// State 3. Update RTC time from NTP server at midnight
    if (h == 0 &&
        m == 0 &&
        s == 0 &&
        UTC_t != LastNTP) {
        GetNTPTime();
    }


    ////// State 4. Test if it is time to read Temp and RH values
    ////// AND record sensor values for 5-minute averages (each 20 seconds)
    if ((s % 20 == 0) && (s != LastSum)) {
        sht3x_data = sht3x.readTemperatureAndHumidity(sht3x.eRepeatability_High);
        if (sht3x_data.ERR == 0) {
            Temp = sht3x_data.TemperatureC;
            RH = sht3x_data.Humidity;
            bitWrite(SensorsOK, 0, 1);
        }
        else {
            Temp = -1;
            RH = -1;
            bitWrite(SensorsOK, 0, 0);
        }

        // Send data to IoT device "R_FR_Chamber"
        thing.call_device("R_FR_Chamber", "FR_Env_IoT_Data_in", thing["FR_Env_IoT_Data_out"]);

        // Record if sensor reads were OK
        SensorsOKAvg = SensorsOKAvg & SensorsOK;
        if (!bitRead(SensorsOK, 0)) {
            M5.Lcd.println(F("At least 1 sensor read failed"));
            M5.Lcd.print(F("SensorOK byte: "));
            M5.Lcd.println(SensorsOK, BIN);
            M5.Lcd.print(F("Temp: "));
            M5.Lcd.println(Temp);
            M5.Lcd.print(F("RH: "));
            M5.Lcd.println(RH);
        }

        // Read USB and battery voltages
        USBVolt = M5.Axp.GetVBusVoltage();
        BatVolt = M5.Axp.GetBatVoltage();

        // Add new values to sum
        TempSum += Temp;
        RHSum += RH;
        USBVoltSum += USBVolt;
        BatVoltSum += BatVolt;

        // Update Shift registers
        LastSum = m;
        SumNum += 1;
    }


    ////// State 5. Test if it is time to compute  averages and record in SD card (each 5 minutes)
    if (((m % 5) == 0) && (m != LastLog) && (SumNum > 0)) {
        // Calculate averages
        TempAvg = TempSum / SumNum;
        RHAvg = RHSum / SumNum;
        USBVoltAvg = USBVoltSum / SumNum;
        BatVoltAvg = BatVoltSum / SumNum;


        // Open Year LogFile (create if not available)
        if (!sd.exists(FileName[yr - 2020])) {
            LogFile.open((FileName[yr - 2020]), O_RDWR | O_CREAT); // Create file

            // Add Metadata
            LogFile.println(F("Start position of last line send to IoT:\t1"));
            // Tabs added to prevent line ending with 0. Line ending with 0 indicates that line needs to be sent to IoT.
            LogFile.println(F("\t\t\t"));
            LogFile.println(F("Metadata:"));
            LogFile.println((String)"Station Number\t" + Station_Number + "\t\t\t");
            LogFile.println((String)"Station Name\t" + IoT_Station_Name + "\t\t\t");
            LogFile.println((String)"Station Asset Type\t" + IoT_Asset_Type + "\t\t\t");
            LogFile.println((String)"Station Group\t" + IoT_Group + "\t\t\t");
            LogFile.println((String)"Processor\t" + Processor + "\t\t\t");
            LogFile.println((String)"IoT Hardware\t" + IoT_Hardware + "\t\t\t");
            LogFile.println((String)"IoT Software\t" + IoT_Software + "\t\t\t");
            LogFile.println((String)"RTC Hardware\t" + RTC_Hardware + "\t\t\t");
            LogFile.println((String)"Firmware\t" + Firmware + "\t\t\t");
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
            LogFile.open(FileName[yr - 2020], O_RDWR); // Open file
            LogFile.seekEnd(); // Set position to end of file
        }


        // Log to SD card
        LogString = (String)UTC_t + "\t" + local_t + "\t" + yr + "\t" + mo + "\t" + dy + "\t" + h + "\t" + m + "\t" + s + "\t" +
            String(TempAvg, 4) + "\t" + String(RHAvg, 4) + "\t" +
            String(USBVoltAvg, 4) + "\t" + String(BatVoltAvg, 4) + "\t" +
            String(SensorsOKAvg, DEC) + "\t" +
            "0";
        LogFile.println(LogString); // Prints Log string to SD card file "LogFile.txt"
        LogFile.close(); // Close SD card file to save changes


        // Reset Shift Registers
        LastLog = m;

        TempSum = 0;
        RHSum = 0;
        USBVoltSum = 0;
        BatVoltSum = 0;

        SumNum = 0;
        SensorsOKAvg = B00000001;
    }

    ////// State 6. Test if there is data available to be sent to IoT cloud
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
                if (debug) {
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


    ////// State 7. Test if there is Internet and a Payload to sent SD data to IoT
    if (true) {
        if (debug) {
            M5.Lcd.print(F("Thing connected, payload ready and enought time has enlapsed: "));
            M5.Lcd.println(thing.is_connected() &&
                PayloadRdy &&
                UTC_t - t_DataBucket > DataBucket_frq);
        }
        if (thing.is_connected() &&
            PayloadRdy &&
            UTC_t - t_DataBucket > DataBucket_frq) {
            t_DataBucket = UTC_t; // Record Data Bucket update TRY; even if it is not succesfful
            // extract data from payload string (str)
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
                    else if (i == 8) {  // Temp
                        TempAvg = buffer.toFloat();
                    }
                    else if (i == 9) { // RH
                        RHAvg = buffer.toFloat();
                    }
                    else if (i == 10) { // USB votlage
                        USBVoltAvg = buffer.toFloat();
                    }
                    else if (i == 11) { // Battery voltage
                        BatVoltAvg = buffer.toFloat();
                    }
                    else if (i == 12) { // SensorsOK
                        SensorsOKIoT = buffer.toInt();
                    }
                }
                str = str.substring(str.indexOf('\t') + 1);
            }
            // send data to IoT. If succsessful, rewrite line in log File
            if (thing.write_bucket(iot_data_bucket, "Avg_Data", true) == 1) {
                if (debug) { M5.Lcd.println(F("Loteria, data on Cloud!!!")); }
                // Update line sent to IoT status
                if (debug) {
                    M5.Lcd.print(F("IoT year: "));
                    M5.Lcd.println(yrIoT);
                    M5.Lcd.print(F("File name: "));
                    M5.Lcd.println(FileName[yrIoT - 2020]);
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


    ////// State 8. Update Screen
    if (!debug && (s != LastLcd)) {
        // Top left, Temp
        M5.Lcd.fillRect(0, 0, 160, 100, M5.Lcd.color565(255, 0, 0));
        M5.Lcd.setTextColor(M5.Lcd.color565(255, 255, 255));
        M5.Lcd.setFreeFont(&FreeSans7pt7b);
        M5.Lcd.setTextDatum(TL_DATUM);
        M5.Lcd.drawString(F("Air Temp (C):"), 10, 10);
        M5.Lcd.setFreeFont(&FreeSans20pt7b);
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.drawString(String(Temp), 80, 50);

        // Top right, Relative humidity
        M5.Lcd.fillRect(160, 0, 160, 100, M5.Lcd.color565(0, 0, 255));
        M5.Lcd.setTextColor(M5.Lcd.color565(255, 255, 255));
        M5.Lcd.setFreeFont(&FreeSans7pt7b);
        M5.Lcd.setTextDatum(TL_DATUM);
        M5.Lcd.drawString(F("Air RH (%):"), 10 + 160, 10);
        M5.Lcd.setFreeFont(&FreeSans20pt7b);
        M5.Lcd.setTextDatum(MC_DATUM);
        M5.Lcd.drawString(String(RH), 80 + 160, 50);

        // Bottom left, Aproximate VPD
        M5.Lcd.fillRect(0, 100, 160, 100, M5.Lcd.color565(0, 255, 0));
        M5.Lcd.setTextColor(M5.Lcd.color565(255, 255, 255));
        M5.Lcd.setFreeFont(&FreeSans7pt7b);
        M5.Lcd.setTextDatum(TL_DATUM);
        M5.Lcd.drawString(F("Aprox VPD (kPa):"), 10, 10 + 100);
        M5.Lcd.setFreeFont(&FreeSans20pt7b);
        M5.Lcd.setTextDatum(MC_DATUM);
        // VPD is aproximate beacuse air temp is used instead of leaf temperature to calculate water vapour pressure inside the leaf
        M5.Lcd.drawString(String(CalculateVPD(CalculateVP(Temp, RH), CalculateVP(Temp, 100))), 80, 50 + 100);


        // Bottom right, Info
        M5.Lcd.fillRect(160, 100, 160, 100, M5.Lcd.color565(128, 128, 128));
        M5.Lcd.setTextColor(M5.Lcd.color565(255, 255, 255));
        M5.Lcd.setFreeFont(&FreeSans15pt7b);
        M5.Lcd.setTextDatum(TL_DATUM);
        M5.Lcd.drawString(F("Far-Red"), 10 + 160, 10 + 100);
        M5.Lcd.drawString(F("Chamber"), 10 + 160, 10 + 100 + 50);

        // Date and time
        M5.Lcd.fillRect(0, 200, 320, 240, M5.Lcd.color565(0, 0, 0));
        M5.Lcd.setTextColor(M5.Lcd.color565(255, 255, 255));
        M5.Lcd.setFreeFont(&FreeSans12pt7b);
        M5.Lcd.setTextDatum(ML_DATUM);
        M5.Lcd.drawString((String)dy + "/" + mo + "/" + yr + "  " + h + ":" + m + ":" + s, 10, 220);


        // IoT connection
        if (thing.is_connected()) {
            M5.Lcd.setFreeFont(&FreeSans12pt7b);
            M5.Lcd.setTextDatum(MR_DATUM);
            M5.Lcd.drawString(F("IoT"), 320 - 10, 220);
        }
        else {
            M5.Lcd.setFreeFont(&FreeSans12pt7b);
            M5.Lcd.setTextDatum(MR_DATUM);
            M5.Lcd.drawString(F("No WiFi"), 320 - 10, 220);
        }
        LastLcd = s;
    }


    if (debug) {
        M5.Lcd.print(F("Loop end at: "));
        M5.Lcd.println(millis());
        M5.Lcd.println(F("Press button A to continue"));
        while (true) {
            thing.handle(); // Keep IoT engine runningat all times
            M5.update();    // Update buyyom state
            if (M5.BtnA.wasPressed()) {
                break;
            }
        }
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



float CalculateVP(float Temp, float RH) {
    float result = 0;
    result = 6.1121 * exp((18.678 - (Temp / 234.5)) * (Temp / (257.14 + Temp)));  // Arden Buck equation (in hPa)
    result = result * RH / 100;     // Adjust for RH
    result = result * 0.1;          // Convert to kPa
    return result;
}


float CalculateVPD(float AirVP, float LeafVP) {
    float result = 0;
    result = LeafVP - AirVP;
    return result;
}

////// Get time from NTP an dupdate RTC and local time variables
// For M5 BM8563 RTC
bool GetNTPTime() {
    if (timeClient.forceUpdate()) {
        UTC_t = timeClient.getEpochTime();
        // set system time to UTC unix timestamp
        setTime(UTC_t);
        // Set RTC time to UTC time from system time
        RTCDate.Year = year();
        RTCDate.Month = month();
        RTCDate.Date = day();
        RTCtime.Hours = hour();
        RTCtime.Minutes = minute();
        RTCtime.Seconds = second();
        M5.Rtc.SetTime(&RTCtime);
        M5.Rtc.SetDate(&RTCDate);
        // Convert to local time
        local_t = mxCT.toLocal(UTC_t);
        // Set system time lo local time
        setTime(local_t);
        LastNTP = UTC_t;
        if (debug) {
            Serial.println(F("NTP client update success!"));
            Serial.print(F("UTC time from NTP is: "));
            Serial.println((unsigned long)UTC_t);
        }
        return true;
    }
    else {
        if (debug) { Serial.println(F("NTP update not succesfull")); }
        return false;
    }
}


////// Get time from RTC and update UTC and local time variables
// For M5 BM8563 RTC
void GetRTCTime() {
    M5.Rtc.GetTime(&RTCtime);   // Get UTC time from M5 RTC.
    M5.Rtc.GetDate(&RTCDate);   // Get UC date from M5 RTC

    setTime(RTCtime.Hours,                   // Set system time to UTC time
        RTCtime.Minutes,
        RTCtime.Seconds,
        RTCDate.Date,
        RTCDate.Month,
        RTCDate.Year);
    UTC_t = now();                      // Get UTC time from system time in UNIX format


    local_t = mxCT.toLocal(UTC_t);      // Calculate local time in UNIX format
    setTime(local_t);                   // Set system time to local
    s = second();                       // Set time variables to local time from system time
    m = minute();
    h = hour();
    dy = day();
    mo = month();
    yr = year();
}
