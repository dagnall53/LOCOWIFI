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
//#if defined(ARDUINO_ESP8266_ESP01)
  #define RST_PIN         D2           // Configurable, see typical pin layout above
  #define SS_PIN          D1           // Configurable, see typical pin layout above
 // Configurable, see typical pin layout above    //setup
//RFID stuff***************************************************************
#define NR_OF_PORTS     1
#define NR_OF_SVS       NR_OF_PORTS * 3 + 3

#define NR_OF_EXT_SVS   100 + NR_OF_PORTS * 3

MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.

MFRC522::MIFARE_Key key;




void setMessageHeader(uint8_t *SendPacketSensor);
uint8_t processXferMess(uint8_t *LnRecMsg, uint8_t *LnSendMsg);
uint8_t lnCalcCheckSumm(uint8_t *cMessage, uint8_t cMesLen);


uint8_t uiLnSendMsbIdx = 12;
uint8_t uiStartChkSen;

uint8_t oldUid[UID_LEN] = {0};

uint8_t SendPacketSensor[16];
boolean bReaderActive = false;

//#############################################################

  unsigned long uiStartTime;
  unsigned long uiActTime;
  bool delaying;
  unsigned char i=0;
  unsigned char j=0;  
  uint16_t uiDelayTime = 1000;
 #define PICC_REQIDL           0x26    
  int RFID_status;
  int RFID_read = 0;    
boolean  SensorOutput_Inactive ;


void SendUID (uint8_t *UID, uint8_t PHASE,int where ){  //phase 1 =on 0=off
uint8_t SendMsg[4];
  int tempaddr; 
  int addr; 
       addr= UID[1]+(UID[2]*256);
       addr = addr & 2047; 
     #if _SERIAL_SUBS_DEBUG
         Serial.print("RFID step: ");
         Serial.print (where);
                 Serial.print(" Addr:");        
                 Serial.print (addr);
                 Serial.print (" Val:");
                 Serial.println(PHASE);
                #endif
        if (PHASE==1) {
                   SensorOutput_Inactive = false;       
                   setOPCMsg(SendMsg, addr-1, 0); }
        if (PHASE==0){ 
                   SensorOutput_Inactive = true; 
                   setOPCMsg(SendMsg, addr-1, 1);  
              oldUid[1]=0;
              oldUid[2]=0; 
        }
  
       #if _SERIAL_SUBS_DEBUG
       //    Serial.print(" msg ");
       //    dump_byte_array(SendMsg,4);
       //   Serial.println(); 
          #endif
          UDPSEND(SendMsg,4,2);
          //  UDP.beginPacket(ipBroad, port);
          //  UDP.write(SendMsg,4);
          //  UDP.endPacket();

                         
}




