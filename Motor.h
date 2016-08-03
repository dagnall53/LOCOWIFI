// motor stuff

void setOPC_SL_RD_DATA_Msg (uint8_t *SendMsg, uint8_t SLOT, uint8_t STAT, uint8_t ADR, uint8_t SPD, uint8_t DIRF, uint8_t TRK, uint8_t SS2, uint8_t ADR2, uint8_t SND, uint8_t ID1, uint8_t ID2)  
          {
  unsigned char k = 0;
  uint16_t tempaddr; 
   SendMsg[0] = 0xE7 ; //OP code Slot data read
       
     SendMsg[1] = 0x0E;  // count
      SendMsg[2] = SLOT; 
       SendMsg[3] = STAT; 
        SendMsg[4] = ADR; 
         SendMsg[5] = SPD; 
          SendMsg[6] = DIRF; 
           SendMsg[7] = TRK; 
            SendMsg[8] = SS2; 
             SendMsg[9] = ADR2; 
              SendMsg[10] = SND; 
               SendMsg[11] = ID1; 
                SendMsg[12] = ID2; 
        SendMsg[13]=0xFF;
        for(k=0; k<13;k++){                                   //Make checksum for this three byte message
        SendMsg[13] ^= SendMsg[k];
                         }
}

void setOPC_LONG_ACK_Msg (uint8_t *SendMsg, uint8_t LOPC, uint8_t ACK1 )  
          {
  unsigned char k = 0;
  uint16_t tempaddr; 
   SendMsg[0] = 0xB4 ; //Long Ack
    SendMsg[1] = LOPC;  // count
     SendMsg[2] = ACK1; 
        SendMsg[3]=0xFF;
        for(k=0; k<3;k++){                                   //Make checksum for this three byte message
        SendMsg[3] ^= SendMsg[k];
                         }
}

void SL_RD_Data ( uint8_t *sendMessage)   {


    #if _SERIAL_DEBUG 
          Serial.print(F(" Sending SL_RD_Data :"));     
     #endif 
 // MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);  /// will be only the short address used here..  ?
    MyLocoAddr=CV[1];/// should be recMsg[2]; ///FORCE KEEP MyLocoAddr 
    
    /*         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
SLOT, =1 - 119 active loco
STAT1, D6 D3 00 = Free loco  D5 D4 Busy/ Active  00 = free slot  D2,D1,D0 010=14 step speed
ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2
DIRF  loco direction and functions(4) 0,0,DIR,F0,F4,F3,F2,F1
    setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,        STAT1, ADR,       SPD,       DIRF,    TRK,  SS2,   ADR2,  SND,  ID1,  ID2)  
 */   

    setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x03, MyLocoAddr, Motor_Speed,  DIRF, 0x03,  0x01, 0x00,  SND,  0x00,  0x00)  ;          
    UDPSEND(sendMessage,14,2);  // 2ms delay!


}


void Len4commands (uint8_t *RECMsg) {
   uint16_t CVRequest;
   uint8_t CV_Data;

 switch (RECMsg[0]){
    case  0xA0:    //OPC_LOCO_SPD  //loco speed.. ++++++++++++++++++++++++++++++++++++
          if ((CommandFor(RECMsg)== MyLocoAddr) ){       
                     digitalWrite (BlueLed, LOW) ;  /// turn On        
                     Motor_Speed= (RECMsg[2]*90)/127;            
                     LocoUpdate(0);              }  // //****** end of if (CommandOpCode  = OPC_LOCO_SPD)
    break;
   case  0xA1:    //OPC_LOCO_DIRF== OPC_LOCO_DIRF){  //loco DIRF.. ++++++++++++++++++++++++++++++++++++
        if ((CommandFor(RECMsg)== MyLocoAddr) ){   
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                    DIRF=RECMsg[2];      
                    LocoUpdate(1);              }              
     // IF "LOCO" the code will drive servo 8, (in do periodicupdate   motorspeed  set SV's for 8 to Servo, but address to ??
     // will need to set motorspeed change as acc and dec CV functions and vary motor speed by loco cvs...
     break;
   case  0xA2:    //OPC_LOCO_SND   //loco snd.. ++++++++++++++++++++++++++++++++++++
        if ((CommandFor(RECMsg)== MyLocoAddr) ){ 
                     digitalWrite (BlueLed, LOW) ;  /// turn On 
                     SND= RECMsg[2];  
                       LocoUpdate(2);            }     
    break;
 
   case  0xB0:    //OPC SW REQ B0 {//+++++++++++servo setting ++++++++++++++++++++++++++++++
            // ********** Look for set messages for my servo addresses  
            for (int i=1; i<=8;i++){
               if ((CommandFor(recMessage)== SensorAddress(i))&& (((SV[3*i]& 0x88) ==0x88))) { //if its a ", Sensor Request", its a SERVO driver
                  #if _SERIAL_DEBUG
                         Serial.print(F("Set Servo: "));
                  #endif   
                  SetServo(i ,CommandData(recMessage, Message_Length));   
             }    }   // ..CommandOpCode  = OPC_SW_REQ)                    
     
    break;
    case 0xB2:
     // OPC_INPUT_REP  0xB2  General sensor input codes  
     // no need to do anything with sensor codes..
    break;
     case 0xB4:
     // OPC_LONG_ACK  0xB4  Long acknowledge
     // no need to do anything with other peoples acks..
    break;
   case  0xBA:    //OPC_MOVE_Slots                      //move slots.. ++++++++++++++++++++++++++++++++++++
         if ((CommandFor(RECMsg)== MyLocoAddr) ) {    // ********** Look for messages in my range  
            digitalWrite (BlueLed, LOW) ;  /// turn On   
        #if _SERIAL_DEBUG
          Serial.println(F(" OPC_MOVE_SLOTS Message")) ;     
        #endif  
            SL_RD_Data(sendMessage);  // deals with message, updates MyLocoAddr
                                        }             ////****** end of if (CommandOpCode  = OPC_MOVE_SLOTSQ)
    break;

    case 0xBB: // OPC_RQ_SL_DATA){  BB //REQUEST SLOT DATA+++++++++++++++++++++
        if ((CommandFor(RECMsg)== MyLocoAddr) ){
          digitalWrite (BlueLed, LOW) ;  /// turn On 
                    #if _SERIAL_DEBUG
                        Serial.print(F("This Loco slot request"));     
                    #endif  
          SL_RD_Data(sendMessage);
                                                }
   break;
   case  0xBF:    //OPC_LOCO_ADR  
   if ((CommandFor(RECMsg)== MyLocoAddr) ) {    // ********** Look for   Request switch function messages in my range  
          #if _SERIAL_DEBUG
                   Serial.print(F("OPC_LOCO_ADR for me:"));
          #endif  
           digitalWrite (BlueLed, LOW) ;  /// turn On 
           SL_RD_Data(sendMessage);  // deals with message, updates MyLocoAddr
           #if _SERIAL_DEBUG
                 Serial.println(F(" Sending.."));
                 //dump_byte_array(sendMessage, 14);  
           #endif
                          }
    break;
}   //end of case statements
} // end len4 commands

void Len14Commands (uint8_t *RECMsg) {
   uint16_t CVRequest;
   uint8_t CV_Data;
if ((RECMsg[0] == OPC_WR_SL_DATA)&(RECMsg[2]==0x7C)){ //"Write PT slot data"   read write cv data.  
  if ((CommandFor(RECMsg)== MyLocoAddr)||(CommandFor(RECMsg)== MyLocoLAddr)||(CommandFor(RECMsg)== 0)) {  // ?? Full check of possibilities? May need to add a CV29  bit to select L and S addr messages..
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                    CVRequest=(RECMsg[9]&0x7F)+(128*bitRead(RECMsg[8],0))+(256*bitRead(RECMsg[8],4))+(512*bitRead(RECMsg[8],5));
                    CV_Data=(RECMsg[10]&0x7F)+(128*bitRead(RECMsg[8],1));
                   //REQUEST CV  DATA+++++++++++++++++++++
                   //5<HOPSA>Operations Mode Programming- 7 High address bits of Loco to program, 0 if Service Mode
                   //6<LOPSA>Operations Mode Programming- 7 Low address bits of Loco to program, 0 if Service Mode
                   //7<TRK> Normal Global Track status for this Master, Bit 3 also is 1 WHEN Service Mode track is BUSY
                   //8<CVH> High 3 BITS of CV#, and ms bit of DATA.7 <0,0,CV9,CV8 - 0,0, D7,CV7>
                   //9<CVL> Low 7 bits of 10 bit CV address. <0,CV6,CV5,CV4-CV3,CV2,CV1,CV0>
                   //10<DATA7>Low 7 BITS OF data to WR or RD COMPARE <0,D6,D5,D4 - D3,D2,D1,D0> ms bit is at CVH bit 1 position.                      
            if (bitRead(RECMsg[3],6)==0) {  // read
  #if _SERIAL_DEBUG
          Serial.print(F("Read request for CV["));
          Serial.print(CVRequest+1);
          Serial.print(F("]  ="));
          Serial.print(CV[CVRequest+1]);
     #endif  
            setOPC_LONG_ACK_Msg (sendMessage, 0x7F,  0x01)  ; // Task accepted , will send <E7> (Slot data response.)reply at completion
 #if _SERIAL_DEBUG
          Serial.println(F(" Sending OPC_LONG_ACK_Msg line 168 "));
     #endif   
      UDPSEND(sendMessage,4,2);
      //  <0xEF>,<0E>,<7C>,                     <PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
      BuildProgrammerResponseMessage(sendMessage,RECMsg[3], RECMsg[5],RECMsg[6],RECMsg[7],RECMsg[8],RECMsg[9],CV[CVRequest+1]);   // basically a copy of the command but with the requested data...
  #if _SERIAL_DEBUG
        Serial.print("  Sending response message"); 
        Serial.print(" CV[");
        Serial.print( CVRequest+1);          
        Serial.print("]=");
        Serial.print(CV[CVRequest+1]); 
       // Serial.print("   Full Message =");
       //        dump_byte_array(sendMessage, 14);  
        Serial.println("");
  #endif 
             UDPSEND(sendMessage,14,2);
                                      } // end of READ
                    
      else{  // its a write...
                
   #if _SERIAL_DEBUG
          Serial.print(F("Write CV["));
          Serial.print(CVRequest+1);
          Serial.print(F("]  ="));
          Serial.print(CV_Data);
     #endif  
          if ((CVRequest+1==8)){ // set defaults with CV8=8
            if((CV_Data==8)){
            SetDefaultSVs();
               }}
               else{
          CV[CVRequest+1]=CV_Data; 
                    }
          WriteSV2EPROM();  // update EEPROM
           Data_Updated= true; 
           EEprom_Counter=millis();
          //EEPROM.commit();  // done on timer now
          EPROM2SV();  // update the SV table
          setOPC_LONG_ACK_Msg (sendMessage, 0x6F,  0x01)  ; // acknowledge
     #if _SERIAL_DEBUG
          Serial.print(F("  Sending OPC_LONG_ACK_Msg line 208"));
          //dump_byte_array(sendMessage, 0x04);
          Serial.println();
     #endif     
          UDPSEND(sendMessage,4,2);
  
                    }  // end of write cv
                      }//end our MyLocoAddr
                 } //end request cv data
}  // endLen14Commands 

void Len16Commands (uint8_t *RECMsg) {
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
               Serial.print(" +1:");
                 Serial.print(SV[OPC_Data[2]+1]); 
              Serial.print(" +2:"); 
                 Serial.print(SV[OPC_Data[2]+2]); 
            //   Serial.print("   MSG= :");
            // dump_byte_array(sendMessage, 16);  
             Serial.println("");
           #endif 
             UDPSEND(sendMessage,16,2);
                                }  
         if (OPC_Data[1]==CMD_WRITE) {  // This is a Write data message
              SV[OPC_Data[2]]=OPCdata;      
              WriteSV2EPROM(); 
              Data_Updated= true;
              EEprom_Counter=millis();
              //EEPROM.commit();
              OPCResponse(sendMessage,OPC_Data[1],SV[2],SV[1],OPC_Data[2],SV[OPC_Data[2]],SV[OPC_Data[2]+1],SV[OPC_Data[2]+2]);
              
          #if _SERIAL_DEBUG 
             Serial.print("] Write ");
             Serial.print("  data =");
             Serial.print(OPCdata) ; 
             Serial.print("d   (");
             dump_byte(OPCdata);
             Serial.print(") ");
             Serial.print("  Written and Responded   :"); 
             Serial.print(" SV[");
             Serial.print( OPC_Data[2]);          
             Serial.print("]   =");
             Serial.print(SV[OPC_Data[2]]); 
             Serial.print(" +1:");
             Serial.print(SV[OPC_Data[2]+1]); 
             Serial.print(" +2:"); 
             Serial.print(SV[OPC_Data[2]+2]); 
          //   Serial.print("   MSG= :");
          //   dump_byte_array(sendMessage, 16);  
             Serial.println("");
             #endif 
             UDPSEND(sendMessage,16,2);  //delay  before sending?
                         }
          } // my address or broadcast 
//+++++++++++++++++++++++++++++++++++++++++++++++++ opc peer xfer++++++++++++
}  //recMessage[0]== OPC_PEER_XFER

} //end len16 commands


