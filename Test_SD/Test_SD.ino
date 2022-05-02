/*
 Name:    Test_SD.ino
 Created: 5/1/2022 9:53:42 PM
 Author:  aivel
*/



const int SD_CS_PIN = 10;       // SD on Addlogger M0 Feather 

#include <SPI.h>

////// SD fat library used in order to use exFAT file system
#include <SdFat.h>
const int SD_FAT_TYPE = 2;  // SD_FAT_TYPE = 2 for exFAT
const int SD_SCK_MHZ = 10;  // it works till 12 MHz in M0 board
SdExFat sd;
ExFile LogFile;
ExFile root;
char line[500];
String str = "";
int position = 0;

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
R_AirTemp\tR_AirRH\t\
FR_AirTemp\tFR_AirRH\t\
R_Actinic_ON\tFR_Actinic_ON\t\
R_Fan_ON\tFR_Fan_ON\t\
FR_1_LEDs_ON\tFR_2_LEDs_ON\t\
FR_1_LEDs_PWM\tFR_2_LEDs_PWM\t\
R_Actinic_Manual_Ctrl\tFR_Actinic_Manual_Ctrl\t\
FR_1_LEDs_Manual_Ctrl\tFR_2_LEDs_Manual_Ctrl\t\
R_Fan_Manual_Ctrl\tFR_Fan_Manual_Ctrl\t\
SensorsOK\t\
SentIoT");
const int HeaderN = 29; // cero indexed
String LogString = "";



int yr = 2022;



// the setup function runs once when you press reset or power the board
void setup() {

    ////// Start General Libraries
    Serial.begin(115200);
    delay(2500);
    if (debug = true) { Serial.print(F("\nSetup start\n")); }
    SPI.begin();

    ////// Initialize SD card
    if (debug) {
        Serial.print(F("Starting SD card: "));
        Serial.println(sd.begin(SD_CS_PIN, SD_SCK_MHZ));
    }
    else {
        sd.begin(SD_CS_PIN, SD_SCK_MHZ);
    }
    // Reserve RAM memory for large and dynamic String object
    // used in SD file write/read
    // (prevents heap RAM framgentation)
    LogString.reserve(HeaderN * 7);
    str.reserve(HeaderN * 7);





}

// the loop function runs over and over again until power down or reset
void loop() {

    Serial.println();

    Serial.print(F("Log file exists? "));
    Serial.println(sd.exists(FileName[yr - 2020]));     // FIX!!! (use sd instead of LogFile)


    Serial.print(F("Close test: "));
    Serial.println(LogFile.close());        // FIX!!! (close file)

    Serial.print(F("Open test: "));
    Serial.println(LogFile.open((FileName[yr - 2020]), O_RDWR));

    LogFile.rewind();


    LogFile.fgets(line, sizeof(line));
    Serial.print(F("First line: "));
    Serial.println(line);




    delay(1000);

}
