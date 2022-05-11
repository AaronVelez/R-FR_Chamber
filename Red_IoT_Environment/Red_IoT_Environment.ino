/*
 Name:		Red_IoT_Environment.ino
 Created:	5/9/2022 5:58:13 PM
 Author:	aivel
*/

// the setup function runs once when you press reset or power the board
void setup() {




	thing["R_Env_IoT_Data_out"] >> [](pson& out) {
		out["IoT_Temp_R"] = 3;
		out["IoT_RH_R"] = 43.1;
	};

}

// the loop function runs over and over again until power down or reset
void loop() {




	thing.call_device("deviceA", "R_Env_IoT_Data_in", thing["R_Env_IoT_Data_out"]);
  
}
