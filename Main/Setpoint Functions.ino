// Function to get setpoints from txt files form SD card
float Get_Setpoint(const char* Setpoint_File) {
	ExFile SetpointsFile;
	char read[25];
	String read_str;
	float setpoint;
	
	Serial.print(F("File name: "));
	Serial.println(Setpoint_File);

	
	
	Serial.print(F("File open success: "));
	Serial. println( SetpointsFile.open(Setpoint_File, O_RDWR) );


	SetpointsFile.rewind();
	SetpointsFile.fgets(read, sizeof(read));     // Get first line
	SetpointsFile.close();

	Serial.print(F("SD raw line: "));
	Serial.println(read);


	read_str = String(read);
	read_str = read_str.substring(0, read_str.indexOf("\r"));
	
	Serial.print(F("SD processed line: "));
	Serial.println(read_str);

	setpoint = read_str.toFloat();
	
	
	Serial.print(F("SD setpoint: "));
	Serial.println(setpoint);
	
	return setpoint;
}



void Set_Setpoint(const char* Setpoint_File, float setpoint) {
	ExFile SetpointsFile;

	SetpointsFile.remove(Setpoint_File);
	SetpointsFile.open(Setpoint_File, O_RDWR | O_CREAT);

	SetpointsFile.println(setpoint);
	SetpointsFile.close();
}











