
 #define ADDR_NODE_ID_H          1
#define ADDR_NODE_ID_L          2
#define ADDR_USER_BASE          3
 
uint8_t ucBoardAddrHi = DEFAULT_BOARD_ADDR_HI;  //board address high; 
uint8_t ucBoardAddrLo = DEFAULT_BOARD_ADDR_LO;  //board address low; default = 128*1 +89 = 217
uint16_t FullBoardAddress;   //the full board address ... 

uint8_t ucAddrHiSen = DEFAULT_SENSOR_ADDR_HI;    //sensor address high
uint8_t ucAddrLoSen = DEFAULT_SENSOR_ADDR_LO;    //sensor address low
uint8_t ucSenType = 0x0F; //input
uint16_t uiAddrSenFull[10];

 
 //// Debug settings
#define _SERIAL_DEBUG  1
#define _SERIAL_SUBS_DEBUG 1

#define BlueLed 2 // same as PIN D4

/// Variables for OPC_Peer Xfer
uint8_t OPCSV;
uint8_t OPCSub;
uint8_t OPCdata;
uint8_t OPCSRCL;
uint8_t OPCRXTX;




 //servo settings   /// need to be here so that Subroutines.h knows about them...

 //one for each port and one for each of the  switchable positions  (avoiding address 0 for the moment), 
int SDemand[5];
int SDelay[5];
int SloopDelay = 2;  //100 is about 4 seconds with 50ms delay in loop
int ServoOffDelay[5];
int OffDelay = 200;  //about 6 seconds with Delay = 20ms in main loop

Servo myservo0;  // create servo object to control a servo  
Servo myservo1;  // create servo object to control a servo  
Servo myservo2;  // create servo object to control a servo  
Servo myservo3;  // create servo object to control a servo  
Servo MyLocoDrive;       // create servo object to control a servo type loco 

 int buttonState[4] ;
 int lastButtonState[4];
 int DebounceTime[4]  ;
 int DebounceDelay  =  6 ; //equals 60ms (6 cycles of Void loop at 10ms each)



uint8_t Motor_Speed =0 ;
uint8_t Motor_Servo =0 ;

uint8_t DIRF =0 ;
uint8_t SND =0 ;
uint8_t DEST ;



  uint8_t sendMessage[128]; 







 

