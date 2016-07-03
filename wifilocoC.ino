

#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <WiFiUDP.h>
#include <Servo.h> 
#include <EEPROM.h>
#include "SV.h";
#include "Defaults.h";
#include "OPC_Codes.h";

#include "Subroutines.h"; 

#include "Secrets.h"; 

String wifiSSID = SSID_RR;
String wifiPassword = PASS_RR;  


WiFiUDP UDP;
IPAddress ipBroad; 
const int port = 1235;


///HTML server
WiFiServer server(80);


//otherstuff
int LoopCount ;
byte udpTestPacket0[] = {0xB2, 0x02, 0x50, 0x1F};
byte udpTestPacket1[] = {0xB2, 0x02, 0x40, 0x0F};
byte UDPSendPacket[16]  ;
boolean Phase = 0 ;


#include "Motor.h"; 
void setup() {
 
  Serial.begin(115200);
  Serial.print(F("Initialising. Trying to connect to:"));
  Serial.println(SSID_RR);
  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str());
  while (WiFi.status() != WL_CONNECTED) {delay(500);Serial.print(".");}
  Serial.println();
  Serial.println(WiFi.localIP());
  ipBroad=WiFi.localIP();
  ipBroad[3]=255; //Set broadcast to local broadcast ip e.g. 192.168.0.255

  // Start the server
    server.begin();
    Serial.println("Server started");
    // start UDP server
    UDP.begin(port);

//  Setup addresses


   SetDefaultSVs();
  
    EEPROM.begin(512);
    WriteSV2EPROM();   ///  comment out later....use if needed to reset eprom
   //   ShowSVS();  
 
    if((EEPROM.read(ADDR_NODE_ID_H) == 0xFF) && (EEPROM.read(ADDR_NODE_ID_L) == 0xFF)){ //eeprom empty, first run * comment outthis and 
      SetDefaultSVs();
      WriteSV2EPROM(); 
      EEPROM.commit();
   } //if
     

     EPROM2SV();
   ShowSVS();  
   DEST=CV[18];
  FullBoardAddress=AddrFull(SV[2],SV[1]);
  Serial.print("Board Base address for commands is:");
  Serial.print(FullBoardAddress);
  Serial.print(" onwards");
  Serial.println(" ");
  Serial.print(" Will send sensor data from addresses :");
   for ( int i=0;i<=3;i++){
  Serial.print(SensorAddress(i+1));
  Serial.print(" , "); }
  Serial.println(" ");

 SetMyPorts();
 for (int i=0 ; i<=4; i++) {
    SDelay[i]=1000;
    SetServo(i,1);
      }
  digitalWrite (BlueLed, LOW) ;  /// turn On  


 ReadInputPorts();
 for (int i=0 ; i<=4; i++) {
   lastButtonState[i]= buttonState[i]+25;  // is not boolean, so simpy make it different
 }

LoopCount = 0;
}

void loop() {
  digitalWrite (BlueLed ,HIGH) ;  ///   turn OFF
  
  LoopCount=LoopCount+1;
  delay(20);  // add a small delay
  FullBoardAddress=AddrFull(SV[2],SV[1]);

  uint8_t recLen = UDP.parsePacket();
  uint8_t recMessage[128]; 
  uint8_t sendMessage[128]; 
 //periodic functions
  doPeriodicServo();
  

  
  
  
  
  
  if(recLen){    // Check if a rocrail client has connected

        UDP.read(recMessage, recLen);
        UDP.flush();
    
        // show all messages for debug
    #if _SERIAL_DEBUG
         
         Serial.println();
         Serial.print(F("This Message is for:"));
         Serial.print(CommandFor(recMessage, recLen));
               Serial.print(F(" Our Board address is:"));
         Serial.print(FullBoardAddress); 
          Serial.print(F(" Our DEST is:"));
         Serial.print(DEST); 
            Serial.print("  and our LOCOaddr is CV[18]:");
         Serial.print( CV[18]); 
         Serial.print(" , CV[17]:");
         Serial.println( CV[17]);       
         Serial.print(" (AddrHi, SV[2]:");
        Serial.print(SV[2]); 
        Serial.print("  AddrLo, SV[1]:");
        Serial.print( SV[1]);   
        //  Serial.print("  LOCOaddr, CV[18]:");
        // Serial.print( CV[18]); 
        // Serial.print(" , CV[17]:");
        // Serial.print( CV[17]);         
         Serial.print("  Full Message is:");     
         dump_byte_array(recMessage, recLen);
         Serial.println( );
     #endif   


        motorcommands(recMessage, recLen);




        if (recMessage[0] == OPC_SW_REQ){//+++++++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
           if ((CommandFor(recMessage, recLen)>= FullBoardAddress) & (CommandFor(recMessage, recLen)<= (FullBoardAddress+3))) {
           digitalWrite (BlueLed, LOW) ;  /// turn On 
    #if _SERIAL_DEBUG
          Serial.print(F("Servo Message for ME "));
     #endif   
           SetServo((CommandFor(recMessage, recLen)-FullBoardAddress+1),CommandData(recMessage, recLen));                     }
                                        }  ////****** end of if (CommandOpCode  = OPC_SW_REQ)





       if (recMessage[0]== OPC_PEER_XFER){
          OPCPeerRX(recMessage);
 
        if (((CommandFor(recMessage, recLen)== FullBoardAddress)  ||(OPCSRCL== 0))){  // my address or broadcast
       
         if (SV[1]==81) {delay (random(20,100));}
         else {if (OPCSRCL== 0){ delay(SV[1]*5);}}
          
          digitalWrite (BlueLed, LOW) ;  /// turn On  
          #if _SERIAL_DEBUG
        Serial.print(" Peer xfr message for me :");
    //    Serial.print(" Addr:");
    //    Serial.print( OPCSRCL);          
    //    Serial.print(" sub  :");
    //    Serial.print(OPCSub);                // no need for printing this now, code works ok..
        Serial.print(" SV[");
        Serial.print( OPCSV);
        if (OPCRXTX==CMD_READ){Serial.print("] Read");}
        else{Serial.print("]Write ");
             Serial.print("  data =");
        Serial.print(OPCdata) ; }
    #endif  
          
          
          if (OPCRXTX==CMD_READ) {  // This is a Request for data 
              OPCResponse(sendMessage,OPCRXTX,SV[2],SV[1],OPCSV,SV[OPCSV],SV[OPCSV+1],SV[OPCSV+2]);
  #if _SERIAL_DEBUG
        Serial.print("   Response Message"); 
        Serial.print(" SV[");
        Serial.print( OPCSV);          
        Serial.print("]=");
        Serial.print(SV[OPCSV]); 
        Serial.print("   Full Message =");
               dump_byte_array(sendMessage, 16);  
        Serial.println("");
  #endif 
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 16);
              UDP.endPacket();
}  
         if (OPCRXTX==CMD_WRITE) {  // This is a Write data message
              SV[OPCSV]=OPCdata;
              OPCResponse(sendMessage,OPCRXTX,SV[2],SV[1],OPCSV,0x00,0x00,SV[OPCSV]);
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 16);
              UDP.endPacket();
   #if _SERIAL_DEBUG
        Serial.print("   Ack  Message"); 
        Serial.print(" SV[");
        Serial.print( OPCSV);          
        Serial.print("]   =");
        Serial.print(SV[OPCSV]); 
        Serial.print("   MSG= :");
               dump_byte_array(sendMessage, 16);  
        Serial.println("");
  #endif 
            }
          }
      WriteSV2EPROM(); 
      EEPROM.commit();
}  //recMessage[0]== OPC_PEER_XFER





   
  }// end if reclen




 ReadInputPorts();
 for (int i=0 ; i<4; i++) {
      if (Debounce(i) == i) {
          lastButtonState[i] = buttonState[i];
          DebounceTime[i] =0 ;
           digitalWrite (BlueLed ,LOW) ;  /// turn On  
          Serial.print ("Change on IO port : ");
          Serial.print(i+1);
          Serial.print ("  detected = Sensor Address(: ");
          Serial.print(SensorAddress(i+1));
          setOPCMsg(UDPSendPacket, i , SensorAddress(i+1)-1, buttonState[i]);   //setOPCMsg (uint8_t *SendMsg, int i, uint16_t addr, int InputState )
          Serial.print(")  ");
           dump_byte_array(UDPSendPacket,4);
          Serial.println(); 
            UDP.beginPacket(ipBroad, port);
            UDP.write(UDPSendPacket,4);
            UDP.endPacket();
            delay(10);
            UDP.beginPacket(ipBroad, port);
            UDP.write(UDPSendPacket,4);
            UDP.endPacket();
           }
      }




//Serial.print(".");
/*           ///////   Hard wired test of send
  if (LoopCount > 300) {   // don't do this test every time!...
  UDP.beginPacket(ipBroad, port);
  if(Phase){
    UDP.write(udpTestPacket0,sizeof(udpTestPacket0));
     Phase=!Phase;
  } else {
    UDP.write(udpTestPacket1,sizeof(udpTestPacket1));  
    Phase=!Phase;
  }
  UDP.endPacket();
  Serial.print(Phase);
  Serial.println(F("  UDP Packet sent"));
   LoopCount= 0;
}  // if LoopCount
*/     // end of ********* hard test of send



} //void loop


void ReadInputPorts(){
buttonState[0]= digitalRead(D5);
buttonState[1]= digitalRead(D6);
buttonState[2]= digitalRead(D7);
buttonState[3]= digitalRead(D9);
//buttonState[4]= digitalRead(D9);
//buttonState[5]= digitalRead(D10);

    
}
void SetMyPorts(){

  //myservo0.attach(D0);  // attaches the servo on Dxx to the servo object 
  //myservo1.attach(D1);  // Now done in  doPeriodicServo() and SetServo subroutines..
 // myservo2.attach(D2);
  //myservo3.attach(D3);  
  MyLocoDrive.attach(D1);
  pinMode(BlueLed, OUTPUT);  //note is D4...
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
 // pinMode(D8, INPUT);
  pinMode(D9, INPUT);
 // pinMode(D10, INPUT);  // stops serial if set...
    
  Serial.println("Ports set");
}
//------

