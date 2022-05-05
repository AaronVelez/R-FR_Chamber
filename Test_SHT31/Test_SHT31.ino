/*
 Name:		Test_SHT31.ino
 Created:	5/3/2022 4:37:08 PM
 Author:	aivel
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


////// Comunication libraries
#include <Wire.h>
#include <SPI.h>



////// Library for SHT31 Temperature and Humidity Sensor
#include <DFRobot_SHT3x.h>
DFRobot_SHT3x sht3x(&Wire,/*address=*/0x44,/*RST=*/4); // secondary I2C address 0x44
DFRobot_SHT3x::sRHAndTemp_t sht3x_data;



////// M0 ADC constants
const int n = 100; // measure n times the ADC input for averaging
double sum = 0; // shift register to hold ADC data


byte SensorsOK = B00000000;     // Byte variable to store real time sensor status
byte SensorsOKAvg = B00011111;  // Byte variable to store SD card average sensor status
int SensorsOKIoT = 0;           // Variable to send sensor status in decimal format

////// Measured instantaneous variables
double Temp_R = -1;        // Air temperature RED chamber
double RH_R = -1;          // Air RH value RED chamber
double Temp_FR = -1;        // Air temperature FARRED chamber 
double RH_FR = -1;          // Air RH value FARRED chamber
bool AC_OK = true;			// Monitor AC power supply OK
double BatVolt = 0;			// Batery voltage


bool debug = true;


// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(115200);
	delay(2500);
	if (debug = true) { Serial.print(F("\nSetup start\n")); }
	Wire.begin();
	SPI.begin();


	//I2C multiplexer
	pinMode(I2C_Select_0_PIN, OUTPUT);
	digitalWrite(I2C_Select_0_PIN, LOW);



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



}

// the loop function runs over and over again until power down or reset
void loop() {

	Serial.println(F("Loop start"));
	Serial.println(F("Reading Red sensor"));

	// Read Red chamber Temp/RH sensor
	digitalWrite(I2C_Select_0_PIN, LOW);

	Serial.print(F("Red sensor serial number: "));
	Serial.println(sht3x.readSerialNumber());

	sht3x_data = sht3x.readTemperatureAndHumidity(sht3x.eRepeatability_High);
	if (sht3x_data.ERR == 0) {
		Temp_R = sht3x_data.TemperatureC;
		RH_R = sht3x_data.Humidity;
		bitWrite(SensorsOK, 0, 1);
		bitWrite(SensorsOK, 1, 1);
		Serial.print(F("Red Temp: "));
		Serial.println(Temp_R);
		Serial.print(F("Red RH: "));
		Serial.println(RH_R);
	}
	else {
		Serial.print(F("Sensor error"));
		Temp_R = -1;
		RH_R = -1;
		bitWrite(SensorsOK, 0, 0);
		bitWrite(SensorsOK, 1, 0);
	}




	Serial.println();
	Serial.println(F("Reading Far red sensor"));
	// Read Farred chamber Temp/RH sensor
	digitalWrite(I2C_Select_0_PIN, HIGH);

	Serial.print(F("Far red sensor serial number: "));
	Serial.println(sht3x.readSerialNumber());

	sht3x_data = sht3x.readTemperatureAndHumidity(sht3x.eRepeatability_High);
	if (sht3x_data.ERR == 0) {
		Temp_FR = sht3x_data.TemperatureC;
		RH_FR = sht3x_data.Humidity;
		bitWrite(SensorsOK, 2, 1);
		bitWrite(SensorsOK, 3, 1);
		Serial.print(F("Far red Temp: "));
		Serial.println(Temp_FR);
		Serial.print(F("Far red RH: "));
		Serial.println(RH_FR);
	}
	else {
		Serial.print(F("Sensor error"));
		Temp_FR = -1;
		RH_FR = -1;
		bitWrite(SensorsOK, 2, 0);
		bitWrite(SensorsOK, 3, 0);
	}
	digitalWrite(I2C_Select_0_PIN, LOW);


	Serial.println();
	Serial.println();
	Serial.println();
	delay(1000);
  
}
