//////////////////////////////////////////////////
// Universidade Federal de Pernambuco - UFPE	//	  
// Residência Tecnológica em Software Automotivo//      			  
// Final Project -  Team 1                      //
//////////////////////////////////////////////////

#include "tpl_os.h"
#include "Arduino.h"

//Variables received by CAN bus
#define Ego_speed_ID 		0x0C43A1B4
#define ACC_enabled_ID 		0x18FEF100
#define EV_RV_RD_data_ID	0x0C1B3049

//Id mensagens CAN send
#define ICM_ID	0x18F00503

//Macros send, used to construct CAN message frames
#define DLC_ACC		8
#define EXT_FRAME	1

//Variables received
long unsigned int 	mID;
unsigned char 		mDATA[8];
unsigned char 		mDLC    = 0;
static 				byte M  = 0;

//Definition of receive buffer size
#define BUFF_MAX 	10
#define BUFF_MIN 	00
volatile int		buffer = BUFF_MAX;

//Variable to be sent on CAN bus. Received by the ACC ECU. They have initial values.
bool  ACC_input    = 1; //User input used to enable or disable ACC, as the requirements demands.
float Set_speed    = 80/3.6;// User iput used to define desired speed for ACC, as the requirements demands.
char  Set_speed_send[4]; // Variable used to store and send CAN speed set. Received by the ACC ECU

//Variables received by the CAN bus. Used to warn the user about the ACC current situation.
bool  ACC_enabled  = 0; //Represents if the ACC as enabled by the ACC ECU. 
float Ego_speed    = 0; //Represents the vehicle current speed.
bool  Brake_pedal  = 0; //Represents if the brake pedal was pressed.
bool  Gas_pedal    = 0; //Represents if the gas pedal was pressed.
bool  Fault_signal = 0; //Represents if a faulty sensor was detected.

//STORE FRAME_DATA
unsigned char ICM_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Construct an MCP_CAN object and configure the selector chip for pin 10.
MCP_CAN CAN1(10); 

void setup()
{
	//Limits starting set speed
	if (Set_speed < 11) Set_speed = 11;
	else if (Set_speed > 33) Set_speed = 33;
	
	//Initialize the serial interface: baudrate = 115200
	Serial.begin(115200);
	
	//Initialize the can controller: baudrate = 250K, clock=8MHz
	while(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) 
    {         
        delay(200);        
    }
	
	//Changes to normal operating mode
	CAN1.setMode(MCP_NORMAL);
	
	pinMode(2, INPUT); //Defines the interrupt pin as digital pin 2.
}


TASK(ACC_speed_set_send) //Task responsible to send ACC input and set speed in the same CAN message frame
{
	
	if (Serial.available() > 0){ // Check if there is user input to change speed set on serial buffer.
		Set_speed = Serial.parseInt();  // Converter all "numbers" chars to integer and store it.
		if (Set_speed < 40){ // Check if set speed is between acceptable values.
			
            Serial.println("Speed set must have a value greater than 40 km/h");
			Set_speed = 40;
			
        }
		else if (Set_speed >120)
		{
			Serial.println("Speed set must have a value lesser than 120 km/h");
			Set_speed = 120;
		}	
			sprintf(Set_speed_send , "%04X", (int)((Set_speed/3.6)*256));
			ICM_Data[1] = strtol(Set_speed_send , NULL, 16) >> 8;
			ICM_Data[2] = strtol(Set_speed_send  + 2, NULL, 16);

    }
	Serial.read(); //Clears serial buffer for next iteraction
	
	ICM_Data[3] = (char)ACC_input;  //Stores ACC input on CAN frame (data field)

	M = CAN1.sendMsgBuf(ICM_ID, EXT_FRAME, DLC_ACC, ICM_Data);
	
	Serial.println("ICM HAS SENT CAN MESSAGE");

	TerminateTask();
}


TASK(Receive)
{
	if(!digitalRead(2)){
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		if((mID & EV_RV_RD_data_ID) == EV_RV_RD_data_ID){
			Ego_speed = (mDATA[1] << 8) | mDATA[2];
			Ego_speed = Ego_speed / (256);
			Serial.println("Ego speed CAN message received");
		}
		if((mID & ACC_enabled_ID) == ACC_enabled_ID){
			ACC_enabled = mDATA[1];
			Fault_signal = mDATA[2];
			Gas_pedal = mDATA[3];
			Brake_pedal = mDATA[4];
			Serial.println("ACC CAN message received");
		}	
	}
	TerminateTask();
}


TASK(print) //Task used to print values during run time. Used for analyses during runtime.
{
	//this ENTIRE task can be commented, except "terminate task". It is used only to show data on serial terminal.
	GetResource(res1);
	
	Serial.print("Recebendo ---- Ego_speed: ");
	Serial.print(Ego_speed);
	Serial.print("; ");
	
	Serial.print("ACC_enabled: ");
	Serial.print(ACC_enabled);
	Serial.println("; ");
	
	Serial.print("Enviando ---- ACC_input: ");
	Serial.print(ICM_Data[3]);
	Serial.print("; ");
	
	Serial.print("Set_speed: ");
	Serial.print((((ICM_Data[1] << 8) | ICM_Data[2]) / 256));
	Serial.println(";");

	ReleaseResource(res1);
	TerminateTask();
}