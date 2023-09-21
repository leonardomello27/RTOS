//////////////////////////////////////////////////
// Universidade Federal de Pernambuco - UFPE	//	  
// Residência Tecnológica em Software Automotivo//      			  
// Final Project -  Team 1                      //
//////////////////////////////////////////////////

#include "tpl_os.h"
#include "Arduino.h"
#include <stdlib.h>
#include <stdio.h>

//ID CAN Receive used to receive CAN messages (ICM and simulation). Used to compare ID field of received CAN FRAME.
#define ICM_ID	0x18F00503 //Data dictionary. Receives ICM ECU information: Speed set and ACC enable
#define Simulation_Data_ID    0x0C1B3049 //Data dictionary. Receives Simulation data, such as speed, distance, etc.

//Id mensagens CAN used to send Data on CAN bus. Used ID field of CAN FRAME
#define ACC_acceleration_ID		0x0D4001B0 //Data dictionary. Used to send ACC output to others system's node.
#define ACC_Status_ID			0x18FEF100 //Data dictionar. Used to send ACC status to ICM ECU.

//Macros used to send and receive CAN data.
#define DLC_ACC		8
#define EXT_FRAME 	1
static 				byte M  = 0; 
static 				byte M1 = 0;

//Variables received
long unsigned int 	mID; //Used to store, compare and/or write on ID field (CAN message frame)
unsigned char 		mDATA[8]; //Used to store DATA to be sent on CAN bus. Represents the CAN data field.
unsigned char 		mDLC  = 0; //Represents how much bytes of the DATA field are actual transmited data.

//Definition of receive buffer size for CAN MCP 2515 module. It has a limit of reception and transmit buffers.
//Check datasheet for more details
#define BUFF_MAX 	10
#define BUFF_MIN 	00
volatile int		buffer = BUFF_MAX;

//Calibration Variables
const float D_default  = 10;  //Default distance between ACC (ego) car and front (lead) car, regardless of time gap.
const float Kverr_gain = 0.5; //gain used on ACC calculation.
const float Kxerr_gain = 0.2; //gain used on ACC calculation.
const float Kvx_gain   = 0.04; //gain used on ACC calculation.
float       Time_gap_base         = 3; //Base time gap (seconds) between ego and lead car. Used to define Time_Gap based on context
const float Ego_acceleration_min  = -5; //Maximum brake value on m/s^2
const float Ego_acceleration_max  = 1.47;//Maximum acceleration allowed for the ACC, in m/s^2

//Variables received by CAN network
bool  ACC_input         = 0; //Represents the ACC input by the user. Represents the user desire to enable or disable ACC. Sent by ICM ECU.
float Ego_speed         = 0; //Represents ACC(ego) current speed. Sent by Simulation ECU.
float Relative_distance = 0; //Represents the relative distance between cars. Represents the distance sensor value. Sent by Simulation ECU.
float Relative_speed    = 0; //Represents the relative speed between cars. Sent by Simulation ECU.
float ACC_speed_set     = 0; //Represents the desired speed (user input, setpoint) for the acc to reach, if the context allows it. Sent by ICM ECU.

//Variables Logic Block
bool ACC_enabled  = 0; //Represents if the ACC was enabled or disabled.
bool Brake_pedal  = 0; //Represents if the brake pedal was pressed.
bool Gas_pedal    = 0; //Represents if the gas pedal was pressed.
bool Fault_signal = 0; //Represents if a faulty sensor was detected.
bool aux          = 0; //Auxiliary variable to detect context of ACC (If was disabled and became enabled, if was enabled and became disabled, etc)

//ACC Variables
float Time_gap    = 0; //Calculated time gap based on Time_gap_base and context(rain). If it is raining, Time_gap =Time_gap_base *2
bool  Rain        = 0; //Represents if the system detected rain.

//CAN FRAME_DATA START. Used to store data and send it via CAN bus in their respective CAN frame messages. Check data dictionary for more information of sending and receiving it.
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
	pinMode(2, INPUT); //Defines digital pin 2 as input
}


TASK(Can_Receive)
{
	if(!digitalRead(2)){  
		GetResource(res1);
		//Read can frame: mID = Identifier, mDLC = Data lenght, mDATA = data frame
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		
		if((mID & Simulation_Data_ID) == Simulation_Data_ID) {
			Ego_speed = (mDATA[1] << 8) | mDATA[2]; //Check data dictionary
			Ego_speed = Ego_speed / (256);
			
			Relative_distance = (mDATA[3] << 8) | mDATA[4]; //Check data dictionary
			
			//Extracting Relative Speed
			Relative_speed = (mDATA[5] << 8) | mDATA[6]; //Check data dictionary
			Relative_speed = Relative_speed / (256);
			
			
			ReleaseResource(res1);
			TerminateTask();
		}
		
		if((mID & ICM_ID) == ICM_ID) {
			
			Serial.println("CAN ICM message received");
			
			//Extracting ACC set speed
			ACC_speed_set = (mDATA[1] << 8) | mDATA[2]; //Check data dictionary
			ACC_speed_set = ACC_speed_set / (256);
			
			//Extracting ACC Input
			ACC_input = mDATA[3];
			
			ReleaseResource(res1);
			TerminateTask();
		}
		
		ReleaseResource(res1);
	}
	TerminateTask();
}

//Logic_block task is used to detect the context and decide if the acc should be enabled or disabled.
//The conditions follows the requirements.
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
	ACC_enabled_Data[2] = (char)Fault_signal;
	ACC_enabled_Data[3] = (char)Gas_pedal;
	ACC_enabled_Data[4] = (char)Brake_pedal;
	
	//Send CAN message to ICM ECU detailing current ACC situation. Used to inform the user.
	
	M = CAN1.sendMsgBuf(ACC_Status_ID, EXT_FRAME, DLC_ACC, ACC_enabled_Data);	
	ReleaseResource(res1);
	Serial.println("CAN message sent to ICM ECU");
	
	TerminateTask();
	
}


TASK(Calculate_ACC_Acceleration) 
{
	float Acceleration  = 0; //Acceleration to be calculated by ACC ECU. Meters per second^2
	float Safe_distance = 0; //Calculated safe distance. Safe_distance = D_default + (Time_gap*Ego_speed). Unit: Meters
	float Control_v     = 0; //Used to calculate acceleration when the context is "Control velocity mode"
	float Control_x     = 0; //Used to calculate acceleration when the context is "Control distance mode"
	float SafeD_relD	= 0; //Difference between calculated safe distance and measured distance (relative distance)
	char  Acceleration_send[5];
	
	//Defines Time_gap value based on rain signal.
	if(Rain){
		Time_gap = 2*Time_gap_base;
	}else{
		Time_gap = Time_gap_base;
	}
	
	GetResource(res1);
	
	if(ACC_enabled) //If ACC is enabled, do the following.
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
		
		//Limits acceleration values based on defined range
		Acceleration = (Acceleration < Ego_acceleration_min) ? Ego_acceleration_min : (Acceleration > Ego_acceleration_max) ? Ego_acceleration_max : Acceleration;
		
		sprintf(Acceleration_send , "%04X", (int)(((Acceleration)/0.01)+5));
		ACC_Acceleration_Data[1] = strtol(Acceleration_send , NULL, 16) >> 8;
		ACC_Acceleration_Data[2] = strtol(Acceleration_send  + 2, NULL, 16);
	
		ReleaseResource(res1);
	
		M1 = CAN1.sendMsgBuf(ACC_acceleration_ID, EXT_FRAME, DLC_ACC, ACC_Acceleration_Data);
		
		if (M1 == CAN_OK){
			Serial.println("Acceleration sent"); 
			M1 = 0;
		}
	}	
	TerminateTask();
}


TASK(print)
{
	GetResource(res1);
	
	Serial.print("Sent ---- ACC_enabled: ");
	Serial.print(ACC_enabled);
	Serial.print("; ");
	
	Serial.print("Acceleration: ");
	Serial.print((((ACC_Acceleration_Data[1] << 8) | ACC_Acceleration_Data[2])-5)*0.01);
	Serial.println(";");
	
	
	Serial.print("ICM ---- ACC_input: ");
	Serial.print(ACC_input);
	Serial.print("; ");
	
	Serial.print("Set_speed: ");
	Serial.print(ACC_speed_set);
	Serial.println("; ");
	
	Serial.print("Simu_arduino ---- Ego_speed: ");
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