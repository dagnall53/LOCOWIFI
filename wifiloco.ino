#define _LOCO 1 
#define _STATIC 1


#include <SPI.h>
#include <MFRC522.h> //   mine is in C:\Users\Richard\Documents\Arduino\libraries\MFRC522 and .cpp is modified to give 10Mhz with ESP  #define SPI_CLOCK_DIV_4 10000000
 
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
WiFiUDP UDP2;  

IPAddress ipBroad; 
const int port = 1235;

#include "Subroutines.h"; 

///HTML server
//WiFiServer server(80);


//otherstuff

byte udpTestPacket0[] = {0xB2, 0x02, 0x50, 0x1F};
byte udpTestPacket1[] = {0xB2, 0x02, 0x40, 0x0F};
//byte UDPSendPacket[16]  ;




#include "Motor.h"; 


#include "RFID_Subs.h";

void setup() {
  POWERON=true;
  Ten_Sec= 10000;
  LOCO= false;
#if _LOCO
 LOCO = true ;
#endif
// set my indexed port range for indexed use later
           NodeMcuPortD[0]=D0;
           NodeMcuPortD[1]=D1;
           NodeMcuPortD[2]=D2;
           NodeMcuPortD[3]=D3;
           NodeMcuPortD[4]=D4;   
           NodeMcuPortD[5]=D5;
           NodeMcuPortD[6]=D6;
           NodeMcuPortD[7]=D7;
           NodeMcuPortD[8]=D8;
           NodeMcuPortD[9]=D9;
           NodeMcuPortD[10]=D10;



  
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
  //  server.begin();
  //  Serial.println("Server started");
    // start UDP server
    UDP.begin(port);
    UDP2.begin(port);

//  Setup addresses


        Data_Updated= false; 
        EEPROM.begin(512);

    if((EEPROM.read(255) == 0xFF) && (EEPROM.read(256) == 0xFF)){ //eeprom empty, first run. Can also set via CV[8]=8
                      Serial.println(" ******* EPROM EMPTY....Setting Default EEPROM values *****");
                      SetDefaultSVs();
                      WriteSV2EPROM(); 
                      Data_Updated= true; 
                      EPROM_Write_Delay=millis()+Ten_Sec;
                      //EEPROM.commit();
                      //delay(100);
                                } //if
     

      EPROM2SV();     //Write to the SV and CV registers
      SV[2]=wifiaddr;  //overwrite this one though!!
     // ShowSVS();  
      MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);
      MyLocoAddr= CV[1];/// 
      FullBoardAddress=AddrFull(SV[2],SV[1]);
      // check addresses: 
 for ( int i=0;i<=10;i++) {
  Serial.print(" Port D[");
  Serial.print(i);
  Serial.print("] is ");
  Serial.print (NodeMcuPortD[i]);
  Serial.println();
 }


 
 
#if  _LOCO  
//   * Setup rfidstuff *************************
   
    SPI.begin();        // Init SPI bus// 
    mfrc522.PCD_Init(); // Init MFRC522 card
    mfrc522.PCD_SetRegisterBitMask(mfrc522.RFCfgReg, (0x07<<4));  //https://github.com/miguelbalboa/rfid/issues/43 //If you set Antenna Gain to Max it will increase reading distance
 
    byte readReg = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
    if((readReg == 0x00) || (readReg == 0xFF)){ //missing reader
                                Serial.println("Reader absent");
                                } else {
                                         Serial.println("Reader present");
                                          bReaderActive = true;  
                                        }

  //++++++++++++++++++++Print Debug and Current setup information stuff    +++++++++++++++++++++++++++++


  Serial.println("   ");
  Serial.print(" Short 'Locomotive Address' is");
  Serial.print (CV[1]);
  Serial.println();
  Serial.print(" Long 'Locomotive Address' is");
  Serial.print (CV[18]+((CV[17]&0x3F)*256));
  Serial.print(" CV17=");
  Serial.print(CV[17]); 
  Serial.print(" CV18=");
  Serial.println(CV[18]); 
  
  Serial.println("......... Setting Loco defaults");
   Motor_Speed=0;
   SPEED=0;
   Loco_motor_servo_demand=90; 
   if (!myservo8.attached()) {myservo8.attach(D8); }// re attach loco motor servos in case they have been switched off
   myservo8.write(Loco_motor_servo_demand);
   //ServoOffDelay[8]=1;
   //SDelay[8]= 1000;
   servoDelay[8]=millis();
 //  SendWiFiDebug(00);
 pinMode(BACKLight, OUTPUT);   // D0 and D3 for direction lights
 pinMode(FRONTLight, OUTPUT);  
 pinMode(BlueLed, OUTPUT);  //is also D4...
  digitalWrite (FRONTLight,1);   //Turn off direction lights
  digitalWrite (BACKLight,1);  //Turn off direction lights
 CV[8]= 0x0D; // DIY MFR code
 CV[7] = 0x03; //ver 
#endif  


 #if _STATIC
  Serial.print("LocoIO Board address :");
  Serial.print(SV[1]);
  Serial.print("-");
  Serial.println(SV[2]);
  Serial.print(" Servos on addresses: ");
  for ( int i=1;i<=8;i++) {
    if ((((SV[3*i]& 0x88) ==0x88))){   //OUTPUT & PULSE  TYPE == SERVO
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
    Serial.print(SensorAddress(i));
     } }
   Serial.println("   "); 
   Serial.print(" Switches  on addressses:");
   for ( int i=1;i<=8;i++) {
      if ((SV[(3*i)]& 0x88) == 0x00)    {   // only do this if this port is a "Switch Report" == INPUT
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
    Serial.print(SensorAddress(i));
      } }
   
   Serial.println("   ");
   Serial.print(" Outputs on addresses:");          
   for ( int i=1;i<=8;i++) {
      if ((SV[(3*i)]& 0x88) == 0x80)    {   // "OUTUT and NOT PULSE" == normal led type output
    Serial.print(" D"); 
    Serial.print(i);
    Serial.print(":=");
    Serial.print(SensorAddress(i));
      }  }
    Serial.println("   "); 
#endif
 
  for (int i=1 ; i<=8; i++) {
    SDelay[i]=1000;
    SetServo(i,1);
      }

  SetMyPorts();
  digitalWrite (BlueLed, LOW) ;  /// turn On  
   
  ReadInputPorts();
    for (int i=0 ; i<=8; i++) {
        lastButtonState[i]= digitalRead(NodeMcuPortD[i]);  // just set them all at this stage
         }
 
  
 POWERON=true;
 SPEED=0;
 WaitUntill = millis();

  SensorOutput_Inactive = true;

  RFIDCycle=millis();
  LoopTimer=millis();
  LocoCycle=millis();
  LenDebugMessage=1;
  DebugMessage[0]=0xFF; 
}  /// end of setup ///////




void loop() {
//Switch these on and set port 1 = output to check timing via oscilloscope use CV[20] as a switch
if ((CV[20]&0x80)==0x80) { //bit 7 = phase test
Phase=!Phase;
digitalWrite(D1,Phase);
               }
LoopTimer=micros();   
delay(2);   // min time between messages is circa 1500us  (loconet)
   digitalWrite (BlueLed ,HIGH) ;  ///   turn OFF
 

   //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ 

   
  // commit the changes writes to the  Eprom and change the ports if they have been altered..
    if ((millis()>= EPROM_Write_Delay) && (Data_Updated)){                // commit EEPROM only when needed..
                     Data_Updated=false; 
                       #if _SERIAL_DEBUG
                         Serial.println("Commiting EEPROM");
                       #endif   

                     SetMyPorts();  //make any port direction changes..

                     FullBoardAddress=AddrFull(SV[2],SV[1]);        // +++++++++  Set up things that may have been changed...+++++++++
                     MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);
                     MyLocoAddr=CV[1];/// 
                     EEPROM.commit();     
                     Show_Port_Settings();
                     delay(50);
                     //ESP.reset() ;
                     }    // soft reset? ?? why does doing this eprom commit stop the servo output???
                      
  //re-connect wifi if not connected
    while (WiFi.status() != WL_CONNECTED)                        {
                    delay(10);
                       #if _SERIAL_DEBUG
                         Serial.print("~");
                       #endif
                    WiFi.mode(WIFI_STA);
                    WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str()); }
              

 //periodic updates and checks
  
      
    if ((millis()>= LocoCycle) && (millis()>=WaitUntill))  {doLocoServo();}   // do at periodic interval and avoid doing anything when doing cv or sv read writes...
    
    UDPFetch(recMessage); //have we recieved data??    
      
         switch (Message_Length){
          case 0: {  // no rx data to work with //if not receiving, do other things... 
                   if ((WiFiDebug) && (millis() >=WaitUntill) && (LenDebugMessage>=2)) {  SendWiFiDebug(0xA3);} // if the debug message gets long, send it..
                     #if _STATIC
                   if ((POWERON)) {ReadInputPorts();}
                     #endif
                         if ( (POWERON) && (millis()>= RFIDCycle)  &&(SPEED>=1) && (bReaderActive)) {      // remove wait for rfidcycle untill i can get the check rfid below 10ms...if we are doing CVSv or loco stuff wait before doing any CheckRFID 
                               checkRFID(); 
                               digitalWrite (BlueLed ,HIGH) ;  ///   turn OFF
                       // Serial.print(micros()-LoopTimer);
                       // Serial.println("us :RFID");
                                                        }
                          else{  // not reading, but perhaps turn off tag?
                             if((!SensorOutput_Inactive)&& (millis()>= RFIDCycle + uiDelayTime)){ 
                  //           Serial.println("Nothing read for a while line 308"); // wait delay before declaring the card gone
                                 SendUID(oldUid,0,3);
                                 bRFIDquiet=true; } }                               
                          if ((POWERON) && (!myservo8.attached()) ) {myservo8.attach(D8); }
                  }    // end case 0
          break;             
 
         case 2:
                if ((recMessage[0]==0x82) && (recMessage[1]==0x7D)){   //  power off
                   POWERON=false;
                   Motor_Speed=0;
                   myservo8.write(90);
                   delay(150);
                   myservo8.detach(); }
                if ((recMessage[0]==0x85) && (recMessage[1]==0x7A)){   //  emergency stops
                   POWERON=false;
                   Motor_Speed=0;
                   myservo8.write(90);
                   delay(150);
                   myservo8.detach(); }
                if ((recMessage[0]==0x83) && (recMessage[1]==0x7C)){ 
                   POWERON=true;
                   myservo8.attach(D8);}
         
         break;
           case 4:
                Len4commands(recMessage); 
       // Serial.print(micros()-LoopTimer);
       // Serial.println("us :4commands");
                break;
         case 14:
                Len14Commands(recMessage);
                break;
         case 16:
                Len16Commands(recMessage);
                break;
         default:
            //Show_MSG();
                break;
  } // end switch cases

} //void loop


void READ_PORT( int i){
   if ((SV[(3*i)]& 0x88) == 0x00)    {   // only do this if this port is an "INPUT"
 /*
          Serial.print (" D1:");
           Serial.print(digitalRead(D1));
            Serial.print (" D2:");
           Serial.print(digitalRead(D2));
            Serial.print (" D3:");
           Serial.print(digitalRead(D3));
           Serial.print (" ---'D1'=");  
           Serial.print(D1); 
           Serial.print ("-- nodemcu[1]=");  
           Serial.print(NodeMcuPortD[1]); 
           


          Serial.println(" ");
   */ 
           if (Debounce(i)) { // debounce is true if switch changed 
                 
            #if _SERIAL_DEBUG   
          Serial.print ("Change on IO port : ");
          Serial.print(i); 
          Serial.print ("  > Sensor Address:");
          Serial.print(SensorAddress(i));  
          Serial.print(" State");
          Serial.print(digitalRead(NodeMcuPortD[i]));
          Serial.print (" Config (SV[");
          Serial.print(((3*i))); 
          Serial.print ("]) = ");
          Serial.println(SV[((3*i))]);
         
          #endif  
          setOPCMsg(sendMessage, SensorAddress(i)-1, digitalRead(NodeMcuPortD[i]));   //setOPCMsg (uint8_t *SendMsg, int i, uint16_t addr, int InputState )
         // Serial.print(")  ");//  dump_byte_array(sendMessage,4);// Serial.println(); 
          UDPSEND(sendMessage,4,2); }}
}

void ReadInputPorts(){

// Check for port changes +++++++++++++++++++++++++++++++++++++
if (!bReaderActive) {
     // Serial.println(" Reader Absent ....all ports available");
    for (int i=1 ; i<=8; i++) {  READ_PORT(i);  }}
         else{    READ_PORT(1);
      // only port 1  is fully user available. 
             }
                    }

void PortMode(int i){
           Serial.println();
           Serial.print ("Port :");
           Serial.print (i);
                   if (i==4){Serial.println (" is BlueLed and Output");
                                           pinMode(BlueLed, OUTPUT); } 
     else{
     if ((SV[(3*i)]& 0x88) == 0x80)  { pinMode(NodeMcuPortD[i], OUTPUT);
          Serial.println (" is Output");}  // "OUTUT and NOT PULSE" 
          if ((SV[(3*i)]& 0x88) == 0x00)  { pinMode(NodeMcuPortD[i], INPUT);
          Serial.println (" is Input");}  // "INPUT
          if ((SV[(3*i)]& 0x88) == 0x88)  { myservo8.attach(NodeMcuPortD[i]);
          Serial.println (" is Servo");}  // Servo
           if ((SV[(3*i)]& 0x88) == 0x08)  { 
          Serial.println (" OTHER, set by something else");}  // else
}}

void SetMyPorts(){
  int i;
  if (!bReaderActive) {
    Serial.println("");
    Serial.println(" Reader Absent ....all ports available");
    for (int i=1 ; i<=8; i++) { 

           PortMode(i);
                              }
                      }
  else{   
      // only port 1 is user available. 
        PortMode(1); 
      }
Serial.println("all Ports set");
               }
//------


