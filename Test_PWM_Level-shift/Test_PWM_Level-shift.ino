/*
 Name:		Test_PWM_Level_shift.ino
 Created:	5/6/2022 4:45:13 PM
 Author:	aivel
*/


////// To be used at CIO on the 11 May 2022 for lamp calibration


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

// the setup function runs once when you press reset or power the board
void setup() {


	// Farred LEDs
	pinMode(FR_1_LEDs_PIN, OUTPUT);
	digitalWrite(FR_1_LEDs_PIN, HIGH);
	pinMode(FR_2_LEDs_PIN, OUTPUT);
	digitalWrite(FR_2_LEDs_PIN, HIGH);
	pinMode(FR_1_LEDs_PWM_PIN, OUTPUT);
	analogWrite(FR_1_LEDs_PWM_PIN, 64);
	pinMode(FR_2_LEDs_PWM_PIN, OUTPUT);
	analogWrite(FR_2_LEDs_PWM_PIN, 190);

}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
