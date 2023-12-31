/*
Universidade Federal de Pernambuco - UFPE	
Technological Residency in Automotive Software
Team 1 : Adaptive Cruise Control 
Authors: Gabriel Lopes Lomeu Reis Oliveira;  
		 Leonardo de Melo Abreu 
		 Lucas de Carvalho Sobral; 
		 Matheus Henrique de Souza Passos; 
		 Vitor Fassanaro Cortez de Carvalho. 
*/

#include "tpl_os.h"
#include "Arduino.h"

//ID CAN Receive used to receive CAN messages (ACC_ECU and Simu_Arduino). Used to compare ID field of received CAN FRAME.
#define Ego_speed_ID 		0x0C43A1B4 //Data dictionary
#define ACC_enabled_ID 		0x18FEF100 //Data dictionary
#define EV_RV_RD_data_ID	0x0C1B3049 //Data dictionary

//Id mensagens CAN used to send Data on CAN bus. Used ID field of CAN FRAME
#define ICM_ID	0x18F00503 //Data dictionary

//Macros used to send CAN data
#define DLC_ACC		8
#define EXT_FRAME	1

//Variables received
long unsigned int 	mID;         //Used to store, compare and/or write on ID field (CAN message frame)
unsigned char 		mDATA[8];    //Used to store DATA received from the CAN bus. Represents the CAN data field
unsigned char 		mDLC    = 0; //Represents the number of bytes present in the received data field

static 				byte M  = 0; //You can use it to check the status of the CAN message. If M == CAN_OK, the message was transmitted successfully.

//Definition of receive buffer size for CAN MCP 2515 module. It has a limit of reception and transmit buffers.
//Check datasheet for more details
#define BUFF_MAX 	10
#define BUFF_MIN 	00
volatile int		buffer = BUFF_MAX;

//Sent variables
bool  ACC_input    = 1;  //Represents the ACC input by the user. Represents the user desire to enable or disable ACC. Sent by ICM ECU.
float Set_speed    = 80; //Represents the desired speed (user input, setpoint) for the acc to reach, if the context allows it. Sent by ICM ECU.
char  Set_speed_send[4]; //Used to store separate the transmission bits from the Set Speed value before sending.

//Variables received
bool  ACC_enabled  = 0; //Represents if the ACC was enabled or disabled. Recived from ACC_ECU.
float Ego_speed    = 0; //Represents ACC(ego) current speed. Recived from Simu_Arduino.
bool  Brake_pedal  = 0; //Represents if the brake pedal was pressed. Recived from ACC_ECU.
bool  Gas_pedal    = 0; //Represents if the gas pedal was pressed. Recived from ACC_ECU.
bool  Fault_signal = 0; //Represents if a faulty sensor was detected. Recived from ACC_ECU.

//STORE FRAME_DATA
//Used to store data and send it via CAN bus in their respective CAN frame messages. Check data dictionary for more information of sending and receiving it.
unsigned char ICM_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Construct an MCP_CAN object and configure the selector chip for pin 10.
MCP_CAN CAN1(10); 

void setup()
{
	
	//It checks if the Set Speed value entered by the user is within the operational limits for the ACC.
	if (Set_speed < 40) Set_speed = 40;
	else if (Set_speed > 120) Set_speed = 120;
	
	//Initialize the serial interface: baudrate = 115200.
	Serial.begin(115200);
	
	//Initialize the can controller: baudrate = 250K, clock=8MHz.
	while(CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) != CAN_OK) 
    {         
        delay(200);        
    }
	
	CAN1.setMode(MCP_NORMAL); //Changes to normal operating mode.
	
	pinMode(2, INPUT); //Defines digital pin 2 as input.
}


TASK(ACC_speed_set_send)
{
	
	if (Serial.available() > 0){ //It checks if there is data available on the serial port.
		Set_speed = Serial.parseInt();  //It reads the value received from the serial port.
		//It checks if the values entered by the user are within the operational limits.
		if (Set_speed < 40){
            Serial.println("Set Speed velocity must be greater than 40 km//h");
			Set_speed =40;
        }else if(Set_speed > 120){
			Set_speed = 120; //If the value is greater than 120 km/h, it will be limited to 120 km/h.
		}

    }
	//It separates the bytes and allocates them within the CAN message to be sent.
	sprintf(Set_speed_send , "%04X", (int)((Set_speed/3.6)*256)); 
	ICM_Data[1] = strtol(Set_speed_send , NULL, 16) >> 8;
	ICM_Data[2] = strtol(Set_speed_send  + 2, NULL, 16);
	Serial.read();
	
	ICM_Data[3] = (char)ACC_input;
	
	//Send the CAN message
	M = CAN1.sendMsgBuf(ICM_ID, EXT_FRAME, DLC_ACC, ICM_Data);

	TerminateTask();
}

//This task is responsible for receiving data from Ego Speed, ACC Enable, Fault Signal, Gas Pedal, and Brake Pedal.
TASK(Receive)
{
	if(!digitalRead(2)){
		//Read can frame: mID = Identifier, mDLC = Data lenght, mDATA = data frame
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		
		if((mID & EV_RV_RD_data_ID) == EV_RV_RD_data_ID){ //Verify ID
			//Extracting Ego Speed
			Ego_speed = (mDATA[1] << 8) | mDATA[2];
			Ego_speed = Ego_speed / (256);
		}
		if((mID & ACC_enabled_ID) == ACC_enabled_ID){//Verify ID
			//Extracting ACC Enabled
			ACC_enabled = mDATA[1];
			//Extracting Fault Signal
			Fault_signal = mDATA[2];
			//Extracting Gas Pedal
			Gas_pedal = mDATA[3];
			//Extracting Brake Pedal
			Brake_pedal = mDATA[4];
		}	
	}
	TerminateTask();
}

//This task is responsible for displaying important system data on the serial interface.

TASK(print)
{
	GetResource(res1);
	
	Serial.print("Ego_speed: ");
	Serial.print(Ego_speed);
	Serial.print("; ");
	
	Serial.print("Set_speed: ");
	Serial.print((((ICM_Data[1] << 8) | ICM_Data[2]) / 256));
	Serial.println(";");
	
	if(ACC_enabled == 1){
		Serial.print("ACC ON");
	}else{
		Serial.print("ACC OFF");
	}
	
	if(Fault_signal == 1){
		Serial.print("Error in the ACC system, it will be turned off");
	}
	
	if(Gas_pedal == 1){
		Serial.print("Gas pedal pressed ACC turned off");
	}
	
	if(Brake_pedal == 1){
		Serial.print("Brake pedal pressed ACC turned off");
	}

	ReleaseResource(res1);
	TerminateTask();
}