//////////////////////////////////////////////////
// Universidade Federal de Pernambuco - UFPE		  
// Residência Tecnológica em Software Automotivo      			  
// PROJETO FINAL 
//////////////////////////////////////////////////


#include "tpl_os.h"
#include "Arduino.h"


long unsigned int 	mID;
unsigned char 		mDATA[8];
unsigned char 		mDLC  = 0;

#define mEEC1_DLC			8
#define EXT_FRAME			1


#define ACC_speed_set_ID	0x18F00503 //ACC_speed_set
#define ACC_input_ID		0x0CF00400 //Acc_input
#define Ego_speed_ID 		0x0C43A1B4 //Ego_speed
#define ACC_enabled_ID 		0x18FEF100 //Acc_enabled


//bool   Fault_signal = 0; // FAULT_SIGNAL
bool  ACC_input   = 1;
float Set_speed     = 80/3.6; // setspeed
char Set_speed_send[4];
bool  ACC_enabled = 0;
float Ego_speed = 0;  

byte M   = 0;
byte ret = 0;


// STORE FRAME_DATA
unsigned char ACC_input_Data[8]     = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char ACC_speed_set_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


//Construct an MCP_CAN object and configure the selector chip for pin 10.
MCP_CAN CAN1(10); 

void setup()
{
	// Initialize the serial interface: baudrate = 115200
	Serial.begin(115200);
	
	//Initialize the can controller: baudrate = 250K, clock=8MHz
	while(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) 
    {         
        delay(200);        
    }
//	Serial.println("MCP2515 can_send initialized successfully!");
	//Changes to normal operating mode
	CAN1.setMode(MCP_NORMAL);
	pinMode(2, INPUT);
}


TASK(ACC_speed_set_send)
{
	
	sprintf(Set_speed_send , "%04X", (int)(Set_speed*256));
	ACC_speed_set_Data[1] = strtol(Set_speed_send , NULL, 16) >> 8;
	ACC_speed_set_Data[2] = strtol(Set_speed_send  + 2, NULL, 16);

	M = CAN1.sendMsgBuf(ACC_speed_set_ID, EXT_FRAME, mEEC1_DLC, ACC_speed_set_Data);
	
//	if (M == CAN_OK){
//		Serial.println("can_send ACC_speed_set OK"); 
//	}
	TerminateTask();
}


TASK(ACC_input_send)
{
	ACC_input_Data[1] = (char)ACC_input; //WRITES TO CAN MESSAGE DATA FIELD

	ret = CAN1.sendMsgBuf(ACC_input_ID, EXT_FRAME, mEEC1_DLC, ACC_input_Data);

//	if (ret == CAN_OK){
//		Serial.println("can_send ACC_input OK"); 
//	}
	TerminateTask();
}


TASK(Receive)
{
	if(!digitalRead(2)){
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		if((mID & Ego_speed_ID) == Ego_speed_ID){
			Ego_speed = (mDATA[1] << 8) | mDATA[2]; //Check data dictionary
			Ego_speed = Ego_speed / (256);
			Serial.println("RECEBEU EGO CAN----------");
		}
		if((mID & ACC_enabled_ID) == ACC_enabled_ID){
			ACC_enabled = mDATA[1];
		}			
	}
	TerminateTask();
}


TASK(print)
{
	GetResource(res1);
	
	Serial.print("ACC_input: ");
	Serial.print(ACC_input);
	Serial.print("; ");
	
	Serial.print("Set_speed: ");
	Serial.print(Set_speed);
	Serial.println(";");
	
	Serial.print("Ego_speed: ");
	Serial.print(Ego_speed);
	Serial.print("; ");

	ReleaseResource(res1);
	TerminateTask();
}