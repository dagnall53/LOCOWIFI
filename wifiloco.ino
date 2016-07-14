
#include <SPI.h>
#include <MFRC522.h> 

#define SS_PIN         D1           // Configurable, see typical pin layout above

  
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <Servo.h> 
#include <EEPROM.h>
#include "SV.h";
#include "Defaults.h";
#include "OPC_Codes.h";
#include "Secrets.h"; 

String wifiSSID = SSID_RR;
String wifiPassword = PASS_RR;  
int wifiaddr;

WiFiUDP UDP;
IPAddress ipBroad; 
const int port = 1235;

#include "Subroutines.h"; 

///HTML server
WiFiServer server(80);


//otherstuff
int LoopCount ;
byte udpTestPacket0[] = {0xB2, 0x02, 0x50, 0x1F};
byte udpTestPacket1[] = {0xB2, 0x02, 0x40, 0x0F};
byte UDPSendPacket[16]  ;
boolean Phase = 0 ;



#include "Motor.h"; 
#include "RFID_Subs.h";
void setup() {
 LOCO = 1;
  Serial.begin(115200);
  Serial.print(F("Initialising. Trying to connect to:"));
  Serial.println(SSID_RR);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  while (WiFi.status() != WL_CONNECTED) {delay(500);Serial.print(".");}
  Serial.println();
  Serial.println(WiFi.localIP());
    ipBroad=WiFi.localIP();
    Serial.print(" last byte:");
    wifiaddr=ipBroad[3];
  Serial.println(wifiaddr);
  ipBroad[3]=255; //Set broadcast to local broadcast ip e.g. 192.168.0.255

  // Start the server
    server.begin();
    Serial.println("Server started");
    // start UDP server
    UDP.begin(port);
  

//  Setup addresses


        Data_Updated= false; 
        EEprom_Counter = 0;
    EEPROM.begin(512);
      //SetDefaultSVs();
      // WriteSV2EPROM();   ///  comment out later....use if needed to reset eprom
      //   ShowSVS();  
      //EEPROM.commit();
    if((EEPROM.read(255) == 0xFF) && (EEPROM.read(256) == 0xFF)){ //eeprom empty, first run * comment outthis and 
      Serial.println(" ******* EPROM EMPTY....Setting Default EEPROM values *****");
      SetDefaultSVs();
      WriteSV2EPROM(); 
       Data_Updated= true; 
       EEprom_Counter=0;
      //EEPROM.commit();
      delay(100);
   } //if
     

     EPROM2SV();
     SV[2]=wifiaddr;
   ShowSVS();  
   MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);
    MyLocoAddr= CV[1];/// 
  FullBoardAddress=AddrFull(SV[2],SV[1]);

//   * Setup rfidstuff *************************

  SPI.begin();        // Init SPI bus
    mfrc522.PCD_Init(); // Init MFRC522 card
    byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    if((readReg == 0x00) || (readReg == 0xFF)){ //missing reader
      Serial.println("Reader absent");
    } else {
      Serial.println("Reader present");
      bReaderActive = true;  
      //If you set Antenna Gain to Max it will increase reading distance
   mfrc522.PCD_SetAntennaGain(mfrc522.RxGain_max);
    }









  
  Serial.print("LocoIO Board address :");
  Serial.print(SV[1]);
  Serial.print("-");
  Serial.println(SV[2]);
  Serial.print(" Servos ");
   for ( int i=1;i<=8;i++) {
    if (((SV[3*i]& 0x38) ==0x18)){
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
    Serial.print(SensorAddress(i));
    }
   }
  Serial.println("   ");
  Serial.print(" Switches ");
   for ( int i=1;i<=8;i++) {
      if ((SV[(3*i)]& 0x38) == 0x00)    {   // only do this if this port is a "Switch Report" Serial.print(" :"); 
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
     Serial.print(SensorAddress(i));
      }
   }
  Serial.println("   ");
  Serial.print(" Using Long 'Locomotive Address' is");
  Serial.print (CV[18]+((CV[17]&0x3F)*256));
  Serial.print(" CV17=");
  Serial.print(CV[17]); 
  Serial.print(" CV18=");
  Serial.print(CV[18]); 
  Serial.print(" Short 'Locomotive Address' is");
  Serial.print (CV[1]);
  Serial.println();
  SetMyPorts();
 if(LOCO==0){
 for (int i=1 ; i<=8; i++) {
    SDelay[i]=1000;
    SetServo(i,1);
      }
 }
  digitalWrite (BlueLed, LOW) ;  /// turn On  


 ReadInputPorts();
 for (int i=0 ; i<=8; i++) {
   lastButtonState[i]= buttonState[i]+25;  // is not boolean, so simpy make it different
 }
  
if ( LOCO == 1 ) {
  Serial.println("......... Setting Loco defaults");
   Motor_Speed=0;
   Motor_Servo=90;
   myservo8.attach(D8);
   myservo8.write(Motor_Servo);
   ServoOffDelay[8]=1;
   SDelay[8]= 1000;
}
LoopCount = 0;

SensorOutput_Inactive = true;

}  /// end of setup ///////




void loop() {
  uint8_t recLen = UDP.parsePacket();


   if (Data_Updated) {
      EEprom_Counter=EEprom_Counter+1;
      if (EEprom_Counter >=1000) {
        Data_Updated=false;
        Serial.println("Commiting EEPROM");
        EEPROM.commit();
        }
      }
//connect wifi if not connected
    while (WiFi.status() != WL_CONNECTED) {
    delay(10);
    Serial.print("~");
    WiFi.mode(WIFI_STA);
    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
    }
    digitalWrite (BlueLed ,HIGH) ;  ///   turn OFF
    MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);
    MyLocoAddr=CV[1];/// 
    LoopCount=LoopCount+1;
    delay(2);  // add a small delay
    FullBoardAddress=AddrFull(SV[2],SV[1]);
  
    Message_Length= recLen;  // test to try to avoid calling parsepacket multiple times inside loop..

  if(Message_Length!=0){    // Check if a rocrail client has connected
          digitalWrite (BlueLed, LOW) ;  /// turn On 
          UDP.read(recMessage, Message_Length);
         // UDP.flush();
       
        // show all messages for debug
        Show_MSG();
        if (recMessage[0] == OPC_SW_REQ){//+++++++++++servo setting ++++++++++++++++++++++++++++++
            // ********** Look for set messages for my servo addresses  
            for (int i=1; i<=8;i++){
               if ((CommandFor(recMessage)== SensorAddress(i))&& ((SV[3*i]& 0x38) ==0x18)) { //if its a ", Sensor Request", its a SERVO driver
                  #if _SERIAL_DEBUG
                         Serial.print(F("Set Servo: "));
                  #endif   
                  SetServo(i ,CommandData(recMessage, Message_Length));   
             }    }    }  ////****** end of if (CommandOpCode  = OPC_SW_REQ)
             //+++++++++++++++++++++++++++++++servo setting +++++++++++++++++++++++++++++++++++++++++++++++++++
       if (recMessage[0]== OPC_PEER_XFER){
        if (((CommandFor(recMessage)== FullBoardAddress)  ||(recMessage[3]== 0))){  // my address or broadcast        
            OPCPeerRX(recMessage);
            if(recMessage[3]== 0){
                 delay(wifiaddr*20);
               }
          OPCSRCL = recMessage[3];
          //OPCRXTX= OPC_Data[1]; // OPCSV= OPC_Data[2];  // OPCSub =OPC_Data[5];
         
          OPCdata = OPC_Data[4];    //&0x7F; ?? // limit as rocrail ??
        
          #if _SERIAL_DEBUG
              Serial.print(" Peer xfr :");
              Serial.print(" SV[");
              Serial.print( OPC_Data[2]);
          #endif  
          if (OPC_Data[1]==CMD_READ) {  // This is a Request for data 
              OPCResponse(sendMessage,OPC_Data[1],SV[2],SV[1],OPC_Data[2],SV[OPC_Data[2]],SV[OPC_Data[2]+1],SV[OPC_Data[2]+2]);       
            #if _SERIAL_DEBUG
               Serial.print("] Read");
               Serial.print(" Response :"); 
               Serial.print(" SV[");
               Serial.print( OPC_Data[2]);          
               Serial.print("]=");
               Serial.print(SV[OPC_Data[2]]); 
               Serial.println(" ");
           #endif 
             UDPSEND(sendMessage,16,1000);
                                }  
         if (OPC_Data[1]==CMD_WRITE) {  // This is a Write data message
              SV[OPC_Data[2]]=OPCdata;      
              WriteSV2EPROM(); 
              Data_Updated= true;
              EEprom_Counter=0;
              //EEPROM.commit();
              OPCResponse(sendMessage,OPC_Data[1],SV[2],SV[1],OPC_Data[2],SV[OPC_Data[2]],SV[OPC_Data[2]+1],SV[OPC_Data[2]+2]);
              
          #if _SERIAL_DEBUG 
             Serial.print("] Write ");
             Serial.print("  data =");
             Serial.print(OPCdata) ; 
             Serial.print("d   (");
             dump_byte(OPCdata);
             Serial.print("H ");
             Serial.print("  written and Responded   :"); 
             Serial.print(" SV[");
             Serial.print( OPC_Data[2]);          
             Serial.print("]   =");
             Serial.print(SV[OPC_Data[2]]); 
             Serial.print("   MSG= :");
             dump_byte_array(sendMessage, 16);  
             Serial.println("");
             #endif 
          UDPSEND(sendMessage,16,1000);  //delay  before sending?
                         }
          }
//+++++++++++++++++++++++++++++++++++++++++++++++++ opc peer xfer++++++++++++
}  //recMessage[0]== OPC_PEER_XFER
  motorcommands(recMessage); 
  }
 
  // ++++++++++++++++++++++++++++++++++++++++++++++++end if Message_Length
  else{ //if not receiving, do other things...
  doPeriodicServo();
  ReadInputPorts();
 
 for (int i=1 ; i<8; i++) { 
  // make this  range 1 to 8 (1=port D1)  later when we can sort an automatic port i/o setup section routine 
    
  if ((SV[(3*i)]& 0x38) == 0x00)    {   // only do this if this port is a "Switch Report" SV3 .. bits 4 5 are 0 =Switch and Request (bit 3) is also 0 
      if (Debounce(i) == i) {
          lastButtonState[i] = buttonState[i];
          DebounceTime[i] =0 ;
          
          Serial.print ("Change on IO port : ");
          Serial.print(i);
          Serial.print(" state");
          Serial.print(buttonState[i]);
          Serial.print (" Config (sv[");
          Serial.print(((3*i))); 
          Serial.print ("]) = ");
          Serial.print(SV[((3*i))]);
          Serial.print ("  Sensor Address:(");
          Serial.print(SensorAddress(i));
          setOPCMsg(UDPSendPacket, SensorAddress(i)-1, buttonState[i]);   //setOPCMsg (uint8_t *SendMsg, int i, uint16_t addr, int InputState )
          Serial.print(")  ");
           dump_byte_array(UDPSendPacket,4);
          Serial.println(); 
         
            UDP.beginPacket(ipBroad, port);
            UDP.write(UDPSendPacket,4);
            UDP.endPacket();
            //delay(CV[100]);
            //UDP.beginPacket(ipBroad, port);
            //UDP.write(UDPSendPacket,4);
            //UDP.endPacket();
           }
      }
 }


//                +++++++++++++++++RFID Stuff 
 if(bReaderActive){
   
     if ( mfrc522.PICC_IsNewCardPresent() && mfrc522.PICC_ReadCardSerial()){
            if(!delaying){   //Avoid to many/to fast reads of the same tag
                 if(!compareUid( mfrc522.uid.uidByte, oldUid, mfrc522.uid.size)){// NEW  NOT same UID 
          copyUid(mfrc522.uid.uidByte, oldUid, mfrc522.uid.size);
          SendUID(mfrc522.uid.uidByte,1,1);
          uiStartTime = millis();
          delaying = true;
                                                                    }//  same 
                 RFIDSTATE=0;  //is same 
                 uiStartTime = millis();  // same, continue delay
                 delaying = true;
          } else { //if(!delaying)
         uiActTime = millis();  
         delaying = false;
         if(compareUid( mfrc522.uid.uidByte, oldUid, mfrc522.uid.size)){//same UID  
                       RFIDSTATE=0;  //is same 
            if((uiActTime - uiStartTime) < uiDelayTime){
                      delaying = true;
           } //if((uiActTime
         }
     } //else 
            if ( !SensorOutput_Inactive){
              if(!compareUid( mfrc522.uid.uidByte, oldUid, mfrc522.uid.size)){// !same UID    
              SendUID(oldUid,0,2); 
              RFIDSTATE=8;}}
                                            }//ifmrfc
     else{   //nowt happening..
      RFIDSTATE=RFIDSTATE+1;
      if (RFIDSTATE==6){     
      SendUID(oldUid,0,3);
      RFIDSTATE=8;
      } 
      if (RFIDSTATE>=8){
          RFIDSTATE=8;       
          }
         }  
  } //if(bReaderActive)   
         
  //+++++++++++++++++++ RFID STUFF+++++++++++++++++++++
  }  
} //void loop


void ReadInputPorts(){
//buttonState[1]= digitalRead(D1);
//buttonState[2]= digitalRead(D2);
buttonState[3]= digitalRead(D3);
buttonState[4]= digitalRead(D4);
//buttonState[5]= digitalRead(D5);
//buttonState[6]= digitalRead(D6);
//buttonState[7]= digitalRead(D7);
//buttonState[8]= digitalRead(D8);


    
}
void SetMyPorts(){
  pinMode(BlueLed, OUTPUT);  //is also D4...
  //myservo0.attach(D0);  // attaches the servo on Dxx to the servo object 
  //  pinMode(D0, INPUT);
  //myservo1.attach(D1);  // Now done in  doPeriodicServo() and SetServo(does attache detach) subroutines..
 //pinMode(D1, INPUT);
 // myservo2.attach(D2);
 //pinMode(D2, INPUT); //myservo3.attach(D3);  
 pinMode(D3, INPUT);
// pinMode(D4, INPUT);
//  pinMode(D5, INPUT);
 // pinMode(D6, INPUT);
 // pinMode(D7, INPUT);
 // pinMode(D8, INPUT);
 // pinMode(D9, INPUT);
 // pinMode(D10, INPUT);  // stops serial if set...
    
  Serial.println("Ports set");
}
//------


