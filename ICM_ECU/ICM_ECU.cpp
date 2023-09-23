//////////////////////////////////////////////////
// Universidade Federal de Pernambuco - UFPE	//	  
// Residência Tecnológica em Software Automotivo//      			  
// Final Project -  Team 1                      //
//////////////////////////////////////////////////

#include "tpl_os.h"
#include "Arduino.h"

//Variables received
#define Ego_speed_ID 		0x0C43A1B4
#define ACC_enabled_ID 		0x18FEF100
#define EV_RV_RD_data_ID	0x0C1B3049

//Id mensagens CAN sand
#define ICM_ID	0x18F00503

//Macros send
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

//Sent variables
bool  ACC_input    = 1;
float Set_speed    = 80;
char  Set_speed_send[4];

//Variables received
bool  ACC_enabled  = 0;
float Ego_speed    = 0;  
bool  Brake_pedal  = 0;
bool  Gas_pedal    = 0;
bool  Fault_signal = 0;

//STORE FRAME_DATA
unsigned char ICM_Data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

//Construct an MCP_CAN object and configure the selector chip for pin 10.
MCP_CAN CAN1(10); 

void setup()
{
	//Define set speed
	Set_speed = Set_speed/3.6;
	
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
	
	pinMode(2, INPUT);
}


TASK(ACC_speed_set_send)
{
	
	if (Serial.available() > 0){ // Verifica se há dados disponíveis na porta serial
		Set_speed = Serial.parseInt();  // Lê o valor recebido da porta serial
		if (Set_speed < 40){
			
            Serial.println("Velocidade de Set Speed deve ser maior que 40 km//h");
			
        } else{
			
			sprintf(Set_speed_send , "%04X", (int)((Set_speed/3.6)*256));
			ICM_Data[1] = strtol(Set_speed_send , NULL, 16) >> 8;
			ICM_Data[2] = strtol(Set_speed_send  + 2, NULL, 16);
        }
    }
	Serial.read();
	
	ICM_Data[3] = (char)ACC_input;

	M = CAN1.sendMsgBuf(ICM_ID, EXT_FRAME, DLC_ACC, ICM_Data);
	
	Serial.println("ENVIOU ICM");

	TerminateTask();
}


TASK(Receive)
{
	Serial.println("ENTROU NA TASK");
	if(!digitalRead(2)){
		CAN1.readMsgBuf(&mID, &mDLC, mDATA);
		if((mID & EV_RV_RD_data_ID) == EV_RV_RD_data_ID){
			Ego_speed = (mDATA[1] << 8) | mDATA[2];
			Ego_speed = Ego_speed / (256);
			Serial.println("RECEBEU EGO CAN----------");
		}
		if((mID & ACC_enabled_ID) == ACC_enabled_ID){
			ACC_enabled = mDATA[1];
			Fault_signal = mDATA[2];
			Gas_pedal = mDATA[3];
			Brake_pedal = mDATA[4];
		}	
	}
	TerminateTask();
}


TASK(print)
{
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