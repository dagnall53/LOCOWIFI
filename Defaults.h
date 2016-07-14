
 /* PIN References... Also defined somewhere else in the esp included code so do not unhide this section!!!...


static const uint8_t D0   = 16;  and Red Led on NodeMcu V2 (not present on NodeMCU v3)
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;  and Blue Led on SP8266
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

*/ 
  /** Typical pin layout used:
 * -----------------------------------------------------------------------------------------
 *             MFRC522      ESP866            NodeMCU     NUMBER
 *             Reader/PCD   
 * Signal      Pin          Pin          
 * -----------------------------------------------------------------------------------------
 * RST/Reset   RST          GPIO4              D2          4
 * SPI SS      SDA(SS)      GPIO5              D1          5
 * SPI MOSI    MOSI         GPIO13             D7          13
 * SPI MISO    MISO         GPIO12             D6          12
 * SPI SCK     SCK          GPIO14             D5          14
 *                          GPIO9              SD2         
 *                          GPIO10             SD3
 */





#define BlueLed 2 // same as PIN D4!
 #define ADDR_NODE_ID_H          1
#define ADDR_NODE_ID_L          2
#define ADDR_USER_BASE          3
 
uint8_t ucBoardAddrHi = DEFAULT_BOARD_ADDR_HI;  //board address high; 
uint8_t ucBoardAddrLo = DEFAULT_BOARD_ADDR_LO;  //board address low; default = 128*1 +89 = 217
uint16_t FullBoardAddress;   //the full board address ... 

uint8_t ucAddrHiSen = DEFAULT_SENSOR_ADDR_HI;    //sensor address high
uint8_t ucAddrLoSen = DEFAULT_SENSOR_ADDR_LO;    //sensor address low
uint8_t ucSenType = 0x0F; //input


 
 //// Debug settings
#define _SERIAL_DEBUG  1
#define _SERIAL_SUBS_DEBUG 1



/// Variables for OPC_Peer Xfer
uint16_t OPCSV;
uint8_t OPCSub;
uint8_t OPCdata;
uint8_t OPCSRCL;
uint8_t OPCRXTX;




 //servo settings   /// need to be here so that Subroutines.h knows about them...

 //one for each port and one for each of the  switchable positions  (avoiding address 0 for the moment), 
int SDemand[8];
int SDelay[8];
int SloopDelay = 2;  //100 is about 4 seconds with 50ms delay in loop
int ServoOffDelay[8];
int OffDelay = 200;  //about 6 seconds with Delay = 20ms in main loop

Servo myservo0;  // create servo object to control a servo  
Servo myservo1;  // create servo object to control a servo  
Servo myservo2;  // create servo object to control a servo  
Servo myservo3;  // create servo object to control a servo  
Servo myservo4;  // create servo object to control a servo  
Servo myservo5;  // create servo object to control a servo  
Servo myservo6;  // create servo object to control a servo  
Servo myservo7;  // create servo object to control a servo  
Servo myservo8;  // create servo object to control a servo  
Servo myservo9;  // create servo object to control a servo
Servo myservo10;  // create servo object to control a servo



 int buttonState[8] ;
 int lastButtonState[8];
 int DebounceTime[8]  ;
 int DebounceDelay  =  6 ; //equals 60ms (6 cycles of Void loop at 10ms each)

int CRD_SENSOR ;

uint8_t Motor_Speed =0 ;
uint8_t Motor_Servo =0 ;

uint8_t DIRF =0 ;
uint8_t SND =0 ;
uint16_t MyLocoLAddr ;
uint8_t MyLocoAddr ;


  uint8_t recMessage[128]; 
  uint8_t sendMessage[128]; 
byte OPC_Data[10];
uint8_t Message_Length;

uint8_t uiLnSendLength = 14; //14 bytes
uint8_t uiLnSendCheckSumIdx = 13; //last byte is CHK_SUMM
#define UID_LEN         7

boolean DealtWith  ;

boolean LOCO  = 1; // for LOCO use

int RFIDSTATE ;
boolean  Data_Updated;
int EEprom_Counter; 

//describe the subroutines..

 

