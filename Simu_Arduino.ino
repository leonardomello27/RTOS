//////////////////////////////////////////////////
// Universidade Federal de Pernambuco - UFPE	//	  
// Residência Tecnológica em Software Automotivo//      			  
// Final Project -  Team 1                      //
//////////////////////////////////////////////////

#include <stdbool.h>
#include "Arduino.h"
#include "mcp_can.h"

//ID CAN Receive
#define Ego_acceleration_ID	0x0D4001B0

//Id mensagens CAN sand
#define EV_RV_RD_data_ID    0x0C1B3049 

//Macros send
#define DLC			8
#define EXT_FRAME 	1

//Variables received CAN Frame
unsigned char     mDATA[8];
unsigned char     mDLC  = 0;
long unsigned int mID   = 0;
static            byte ret = 0;


//Definition of receive buffer size
#define BUFF_MAX 	10
#define BUFF_MIN 	00
volatile int		buffer = BUFF_MAX;

//Variables send by CAN network

float Ego_speed  			 = 70.0/3.6;  //Initial ego car position  (m/s)
float Relative_speed    	 = 0;
float Relative_distance_pres = 10;

//Variables received by CAN network
float Ego_acceleration	= 0;

//Environment variables
float Lead_acceleration = 0;
float Lead_position     = 50.0;      //Initial lead car position (m)
float Ego_position      = 10.0;      //Initial ego car position  (m)
float Lead_speed 		= 55.0/3.6;  //Initial lead car position (m/s)
float interval  		= 0.01;
int   counter   		= 0;
float Relative_distance_past = 0;
unsigned long timePrevious 	 = 0;
unsigned long timeCurrent 	 = 0;
unsigned long timeVariation  = 0;

char  Ego_Speed_send[4];
char  Relative_Speed_send[4];
char  Relative_distance_pres_send[4];

//CAN FRAME_DATA START 
unsigned char Send_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//MPC_CAN Object with Chip selector as digital pin 10
MCP_CAN CAN1(10);

void setup() 
{
	float Relative_distance_pres = Lead_position - Ego_position;   //Relative distance between Ego and Lead Car.
	float Relative_distance_past = Relative_distance_pres; 		   //Past value of relative distance.
	
	Serial.begin(115200);
	
	//Start CAN controller : baudrate = 250K, clock=8MHz
	while (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        delay(200);        
    }
	//Serial.println("MCP2515 can_send Started!");
	//Defines operation mode
	CAN1.setMode(MCP_NORMAL);
	pinMode(2, INPUT);
	
}

void loop()
{
	timeCurrent = millis();
	timeVariation = timeCurrent-timePrevious;
  
	if (timeVariation >= 15){
		
		if(!digitalRead(2)){
			CAN1.readMsgBuf(&mID, &mDLC, mDATA);
			if((mID & Ego_acceleration_ID) == Ego_acceleration_ID){
				Ego_acceleration = (mDATA[1] << 8) | mDATA[2];
				Ego_acceleration = (Ego_acceleration-5)*0.01;
				//Serial.println("RECEBEU ACC");
			}else{
				Ego_acceleration = Ego_acceleration;
			}
		}
	
		Relative_distance_past = Relative_distance_pres; //Storage of the last value of Relative Distance
	
		//Simulation of vehicles behavior
		Ego_speed = Ego_speed + interval*Ego_acceleration;
		
		Lead_speed = Lead_speed + interval*Lead_acceleration;

		//Calculation of Ego Car position
		Ego_position += (Ego_speed*interval);

		//Calculation of Lead Car position
		Lead_position += (Lead_speed*interval);

		Relative_distance_pres = Lead_position - Ego_position;
		Relative_speed = Lead_speed - Ego_speed;

		if(Relative_distance_pres > 200){
			Relative_distance_pres = 200;
		}else{
			Relative_distance_pres = Relative_distance_pres;
		}
		
		timePrevious = timeCurrent;
		
		Serial.print("Ego_acceleration:");
		Serial.print(Ego_acceleration);
		Serial.print(",");
	
		Serial.print("Relative Distance:");
		Serial.print(Relative_distance_pres);
		Serial.print(",");
		
		Serial.print("EGO SPEED:");
		Serial.print(Ego_speed);
		Serial.print(",");

		Serial.print("Rspeed:");
		Serial.println(Relative_speed);
		
		//CAN Message Sender
		sprintf(Ego_Speed_send , "%04X", (int)(Ego_speed*256));
		Send_data[1] = strtol(Ego_Speed_send , NULL, 16) >> 8;
		Send_data[2] = strtol(Ego_Speed_send  + 2, NULL, 16);
	
		sprintf(Relative_distance_pres_send , "%04X", (int)Relative_distance_pres);
		Send_data[3] = strtol(Relative_distance_pres_send , NULL, 16) >> 8;
		Send_data[4] = strtol(Relative_distance_pres_send  + 2, NULL, 16);
		
		sprintf(Relative_Speed_send , "%04X", (int)(Relative_speed*256));
		Send_data[5] = strtol(Relative_Speed_send , NULL, 16) >> 8;
		Send_data[6] = strtol(Relative_Speed_send  + 2, NULL, 16);
	
		ret = CAN1.sendMsgBuf(EV_RV_RD_data_ID,EXT_FRAME,DLC,Send_data);
	}	
}