#include <stdbool.h>
#include "Arduino.h"
#include "mcp_can.h"


#define EV_RV_RD_data_ID    0x0C1B3049 
#define Ego_Acceleration_ID	0x0D4001B0


#define DLC			8
#define EXT_FRAME 	1


#define BUFF_MAX 	10
#define BUFF_MIN 	00
volatile int		buffer = BUFF_MAX;


float Ego_ace       = 0;
float Lead_ace      = 0;
float Relative_velo = 0;
float Lead_pos      = 50.0;                            //Initial lead car position (m)
float Ego_pos       = 10.0;                            //Initial ego car position  (m)
float Lead_velo 	= 90.0/3.6;                        //Initial lead car position (m/s)
float Ego_velo  	= 70.0/3.6;                        //Initial ego car position  (m/s)
float interval  	= 0.001;
int   counter   	= 0;
float Relative_distance_pres = 10;
float Relative_distance_past = 5;
unsigned long timePrevious = 0;
unsigned long timeCurrent = 0;
unsigned long timeVariation = 0;

char  Relative_distance_pres_send[4];
char  Ego_Speed_send[4];
char  Relative_Speed_send[4];


unsigned char Send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


unsigned char mDATA[8];
unsigned char mDLC    = 0;
long unsigned int mID = 0;
byte ret = 0;


MCP_CAN CAN1(10);


void setup() 
{
	float Relative_distance_pres = Lead_pos - Ego_pos;     //Relative distance between Ego and Lead Car.
	float Relative_distance_past = Relative_distance_pres; //Past value of relative distance.
	
	Serial.begin(115200);
	
	//Start CAN controller : baudrate = 250K, clock=8MHz
	while (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        delay(200);        
    }
	Serial.println("MCP2515 can_send Started!");
	//Defines operation mode
	CAN1.setMode(MCP_NORMAL);
	pinMode(2, INPUT);
	
}

void loop()
{
	timeCurrent = millis();
	timeVariation = timeCurrent-timePrevious;
  
	if (timeVariation >= 30){
		
	if(!digitalRead(2)){
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		if((mID & Ego_Acceleration_ID) == Ego_Acceleration_ID){
			Ego_ace = (mDATA[1] << 8) | mDATA[2];
			Ego_ace = (Ego_ace-5)*0.01;
			Serial.println("RECEBEU ACC");
		}else{
			Ego_ace = Ego_ace;
		}
	}
	
	Relative_distance_past = Relative_distance_pres; //Storage of the last value of Relative Distance
	
	//Simulation of vehicles behavior
	Ego_velo = Ego_velo + interval*Ego_ace;
	Lead_velo = Lead_velo + interval*Lead_ace;

	//Calculation of Ego Car position
	Ego_pos += (Ego_velo*interval);

	//Calculation of Lead Car position
	Lead_pos += (Lead_velo*interval);

	Relative_distance_pres = Lead_pos - Ego_pos;
	Relative_velo = Lead_velo - Ego_velo;

	if(Relative_distance_pres > 200){
		Relative_distance_pres = 200;
	}else{
		Relative_distance_pres = Relative_distance_pres;
	}

	//counter++;
	

		
		timePrevious = timeCurrent;
	/*	
		Serial.print("Rebendo ---- Ego_ace: ");
		Serial.print(Ego_ace);
		Serial.println("; ");
	
		Serial.print("Enviando ---- Relative Distance: ");
		Serial.print(Relative_distance_pres);
		Serial.print("; ");
		
		Serial.print("EGO SPEED: ");
		Serial.print(Ego_velo);
		Serial.print("; ");

		Serial.print("Relative Speed: ");
		Serial.print(Relative_velo);
		Serial.println("; ");

		Serial.print("Time Current for this Cycle: ");
		Serial.print(timeCurrent);
		Serial.println("; ");

    
		Serial.print("Time Previous for this Cycle: ");
		Serial.print(timePrevious);
		Serial.println("; ");

		Serial.print("Time MILIS for this Cycle: ");
		Serial.print(millis());
		Serial.println("; ");
	*/	
		//CAN Message Sender
	
		sprintf(Ego_Speed_send , "%04X", (int)(Ego_velo*256));
		Send_data[1] = strtol(Ego_Speed_send , NULL, 16) >> 8;
		Send_data[2] = strtol(Ego_Speed_send  + 2, NULL, 16);
	
		sprintf(Relative_distance_pres_send , "%04X", (int)Relative_distance_pres);
		Send_data[3] = strtol(Relative_distance_pres_send , NULL, 16) >> 8;
		Send_data[4] = strtol(Relative_distance_pres_send  + 2, NULL, 16);
		
		sprintf(Relative_Speed_send , "%04X", (int)(Relative_velo*256));
		Send_data[5] = strtol(Relative_Speed_send , NULL, 16) >> 8;
		Send_data[6] = strtol(Relative_Speed_send  + 2, NULL, 16);
	
		ret = CAN1.sendMsgBuf(EV_RV_RD_data_ID,EXT_FRAME,DLC,Send_data);

		if (ret == CAN_OK){
		Serial.println("SIMU ARDUINO SENT"); 
	}
		ret = 0;


	}	

}