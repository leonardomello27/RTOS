//////////////////////////////////////////////////
// Universidade Federal de Pernambuco - UFPE	//	  
// Residência Tecnológica em Software Automotivo//      			  
// Final Project                                //
//////////////////////////////////////////////////

#include "tpl_os.h"
#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>


#define BUFF_MAX 	10
#define BUFF_MIN 	00
volatile int		buffer = BUFF_MAX;


//Macros send
#define DLC_ACC				8
#define EXT_FRAME 			1
static 						byte M  = 0;
static 						byte M1 = 0;

//Variables received
long unsigned int 	mID;
unsigned char 		mDATA[8];
unsigned char 		mDLC  = 0;


//Id CAN Receive
#define ACC_speed_set_ID	0x18F00503 //Data dictionary
#define ACC_input_ID		0x0CF00400 //Data dictionary
#define EV_RV_RD_data_ID    0x0C1B3049 


//Id mensagens CAN sand
#define ID_ACC_Acceleration		0x0D4001B0 //Data dictionary
#define ACC_enabled_ID			0x18FEF100 //Data dictionar


//Calibration Variables
const float D_default  = 10;
float  Time_gap_base   = 3;
const float Kverr_gain = 0.5;
const float Kxerr_gain = 0.2;
const float Kvx_gain   = 0.04;
const float Ego_acceleration_min  = -5;
const float Ego_acceleration_max  = 1.47;


//Variables received by CAN network
bool  ACC_input         = 1; 
float Ego_speed         = 0;
float Relative_distance = 0;
float Relative_speed    = 0;
float ACC_speed_set     = 0;
float Acceleration_aux     = 0;


//Variables Logic Block
bool ACC_enabled  = 0;
bool Brake_pedal  = 0;
bool Gas_pedal    = 0;
bool Fault_signal = 0;
bool aux          = 0;


//ACC Variables
float Time_gap = 0;
bool  Rain          = 0;
float Acceleration  = 0;
char  Acceleration_send[5];
float Safe_distance = 15;
float Control_v     = 0;
float Control_x     = 0;
float SafeD_relD	= 0;


//CAN FRAME_DATA START 
unsigned char ACC_Acceleration_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
unsigned char ACC_enabled_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//MPC_CAN Object with Chip selector as digital pin 10
MCP_CAN CAN1(10);  

void setup() {
	//serial: baudrate = 115200
	Serial.begin(115200);
	
	//Start CAN controller : baudrate = 250K, clock=8MHz
	while (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) {
        delay(200);        
    }
	//Defines operation mode
	CAN1.setMode(MCP_NORMAL);
	pinMode(2, INPUT);
}


TASK(Can_Receive)
{
	if(!digitalRead(2)){  
		GetResource(res1);
		//Read can frame: mID = Identifier, mDLC = Data lenght, mDATA = data frame
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		
		if((mID & EV_RV_RD_data_ID) == EV_RV_RD_data_ID) {
			//Extracting Ego speed
			Serial.println("RECEBEU MSG SIMUARDUINO");
			Ego_speed = (mDATA[1] << 8) | mDATA[2]; //Check data dictionary
			Ego_speed = Ego_speed / (256);
			
			
			//Extracting Relative Distance
			Relative_distance = (mDATA[3] << 8) | mDATA[4]; //Check data dictionary
			
			////Extracting Relative Speed
			Relative_speed = (mDATA[5] << 8) | mDATA[6]; //Check data dictionary
			Relative_speed = Relative_speed / (256);
			
			
			ReleaseResource(res1);
			TerminateTask();
		}
		
		if((mID & ACC_input_ID) == ACC_input_ID) {
			ACC_input = mDATA[0]; //Check data dictionary
			
			ReleaseResource(res1);
			TerminateTask();
		}
		
		if((mID & ACC_speed_set_ID) == ACC_speed_set_ID) {
			ACC_speed_set = (mDATA[1] << 8) | mDATA[2]; //Check data dictionary
			ACC_speed_set = ACC_speed_set / (256);
			
			ReleaseResource(res1);
			TerminateTask();
		}
			ReleaseResource(res1);
	}

	TerminateTask();
}


TASK(Logic_block)
{
	GetResource(res1);
	if(aux == 0 && ACC_input == 1 && Fault_signal == 0 && Ego_speed >= 11 && Gas_pedal == 0 && Brake_pedal == 0){
		aux = 1;
		ACC_enabled = 1;
	}else {
		if (ACC_input == 1 && aux == 1 && Fault_signal == 0 && Gas_pedal == 0 && Brake_pedal == 0){
			ACC_enabled = 1;
		}else {
			ACC_enabled = 0;
			aux = 0;
		}
	}
	ACC_enabled_Data[1] = (char)ACC_enabled;
	
	ReleaseResource(res1);
	
	M = CAN1.sendMsgBuf(ACC_enabled_ID, EXT_FRAME, DLC_ACC, ACC_enabled_Data);	
	
	TerminateTask();
	
}


TASK(Calculate_ACC_Acceleration) 
{
	
	if(Rain){
		Time_gap = 2*Time_gap_base;
	}else{
		Time_gap = Time_gap_base;
	}
	
	GetResource(res1);
	
	if(ACC_enabled)
	{
		Safe_distance = (Ego_speed * Time_gap) + D_default;
		SafeD_relD = Safe_distance - Relative_distance;
		Control_x = (Relative_speed * Kvx_gain) - ((Safe_distance - Relative_distance) * Kxerr_gain);
		Control_v = (ACC_speed_set - Ego_speed) * Kverr_gain;

		if (SafeD_relD > 0){
			
			Acceleration = (Relative_speed * Kvx_gain) - (SafeD_relD * Kxerr_gain);
			
		}else{
			Acceleration = (Control_x < Control_v) ? Control_x : Control_v;
			
		}
		Acceleration = (Acceleration < Ego_acceleration_min) ? Ego_acceleration_min : (Acceleration > Ego_acceleration_max) ? Ego_acceleration_max : Acceleration;
		sprintf(Acceleration_send , "%04X", (int)(((Acceleration)/0.01)+5));
		ACC_Acceleration_Data[1] = strtol(Acceleration_send , NULL, 16) >> 8;
		ACC_Acceleration_Data[2] = strtol(Acceleration_send  + 2, NULL, 16);
	
		ReleaseResource(res1);
	
		M1 = CAN1.sendMsgBuf(ID_ACC_Acceleration, EXT_FRAME, DLC_ACC, ACC_Acceleration_Data);
	}	
	TerminateTask();
}


TASK(print)
{
	GetResource(res1);
	
	Serial.print("Enviando ---- ACC_enabled: ");
	Serial.print(ACC_enabled);
	Serial.print("; ");
	
	Serial.print("Aceleração: ");
	Acceleration_aux  = (ACC_Acceleration_Data[1] << 8) | ACC_Acceleration_Data[2];
	Serial.print((Acceleration_aux-5)*0.01);
	Serial.println(";");
	
	
	Serial.print("Chegando ICM ---- ACC_input: ");
	Serial.print(ACC_input);
	Serial.print("; ");
	
	Serial.print("Set_speed: ");
	Serial.print(ACC_speed_set);
	Serial.println("; ");
	
	Serial.print("Chegando Simu_arduino ---- Ego_speed: ");
	Serial.print(Ego_speed);
	Serial.print("; ");
	
	Serial.print("Relative_distance: ");
	Serial.print(Relative_distance);
	Serial.print("; ");
	
	Serial.print("Relative_speed: ");
	Serial.print(Relative_speed);
	Serial.println("; ");
	
	ReleaseResource(res1);
	TerminateTask();
}