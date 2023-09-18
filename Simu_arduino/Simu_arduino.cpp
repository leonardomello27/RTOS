#include <stdbool.h>
#include "Arduino.h"
#include "mcp_can.h"
#include "tpl_os.h"


float Ego_ace       = 0;
float Lead_ace      = 0;
float Relative_velo = 0;
float Lead_pos      = 50.0;                            //Initial lead car position (m)
float Ego_pos       = 10.0;                            //Initial ego car position  (m)
float Relative_distance_pres = Lead_pos - Ego_pos;     //Relative distance between Ego and Lead Car.
float Relative_distance_past = Relative_distance_pres; //Past value of relative distance.
float Lead_velo = 90.0/3.6;                            //Initial lead car position (m/s)
float Ego_velo  = 70.0/3.6;                            //Initial ego car position  (m/s)
float interval  = 0.001;
int   counter   = 0;
char Relative_distance_pres_send[4];
char Ego_Speed_send[4];
char Relative_Speed_send[4];


#define Ego_Speed_ID			0xFF1B0007
#define Relative_Distance_ID	0x0A1B8008 
#define Ego_Acceleration_ID		0x18EFE210
#define Relative_Speed_ID		0x0C1B3049 

#define DLC			8
#define EXT_FRAME 	1

unsigned char Ego_Speed_data[8]         = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char Relative_Distance_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char mSim3_data[8] 			= {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char Relative_Speed_data[8]    = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char mDATA[8];
unsigned char mDLC = 0;
long unsigned int mID = 0;
static byte ret = 0;

MCP_CAN CAN1(10);

void setup() {
	//serial: baudrate = 115200
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
	
	if(!digitalRead(2)){
	CAN1.readMsgBuf(&mID, &mDLC, mDATA);
	if((mID & Ego_Acceleration_ID) == Ego_Acceleration_ID){
		Ego_ace = mDATA[1];
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

	counter++;

	Relative_distance_pres = Lead_pos - Ego_pos;
	Relative_velo = Lead_velo - Ego_velo;

	if(Relative_distance_pres > 200){
		Relative_distance_pres = 200;
	}else{
		Relative_distance_pres = Relative_distance_pres;
	}
		
	counter++;
	
		if (counter == 500000){
			Serial.print("Ego_ace: ");
			Serial.print(Ego_ace);
			Serial.print("; ");
		
			Serial.print("Relative Distance: ");
			Serial.print(Relative_distance_pres);
			Serial.print("; ");
		
		
			Serial.print("EGO SPEED: ");
			Serial.print(Ego_velo);
			Serial.print("; ");
	

			Serial.print("Relative Speed: ");
			Serial.print(Relative_velo);
			Serial.println("; ");
			TerminateTask();
	

			//Relative Distance CAN Message Sender
	
			sprintf(Ego_Speed_send , "%04X", (int)(Ego_velo*256));
			Ego_Speed_data[1] = strtol(Ego_Speed_send , NULL, 16) >> 8;
			Ego_Speed_data[2] = strtol(Ego_Speed_send  + 2, NULL, 16);
		
	
			ret = CAN1.sendMsgBuf(Ego_Speed_ID,EXT_FRAME,DLC,Ego_Speed_data);

		
			//Ego Speed CAN Message Sender
			//Relative_distance_pres = Relative_distance_pres;
	
			sprintf(Relative_distance_pres_send , "%04X", (int)Relative_distance_pres);
			Relative_Distance_data[1] = strtol(Relative_distance_pres_send , NULL, 16) >> 8;
			Relative_Distance_data[2] = strtol(Relative_distance_pres_send  + 2, NULL, 16);
		
		
			ret = CAN1.sendMsgBuf(Relative_Distance_ID,EXT_FRAME,DLC,Relative_Distance_data);


			//Relative Speed CAN Message Sender
		
		
			sprintf(Relative_Speed_send , "%04X", (int)(Relative_velo*256));
			Relative_Speed_data[1] = strtol(Relative_Speed_send , NULL, 16) >> 8;
			Relative_Speed_data[2] = strtol(Relative_Speed_send  + 2, NULL, 16);
		
			ret = CAN1.sendMsgBuf(Relative_Speed_ID,EXT_FRAME,DLC,Relative_Speed_data);
		
		}
//		
//			exit(0);
//		
}