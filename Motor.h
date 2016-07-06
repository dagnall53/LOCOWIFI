// motor stuff

void setOPC_SL_RD_DATA_Msg (uint8_t *SendMsg, uint8_t SLOT, uint8_t STAT1, uint8_t ADR, uint8_t SPD, uint8_t DIRF, uint8_t TRK, uint8_t SS2, uint8_t ADR2, uint8_t SND, uint8_t ID1, uint8_t ID2)  
          {
  unsigned char k = 0;
  uint16_t tempaddr; 
   SendMsg[0] = 0xE7 ; //OPC S LRD DATA
       
     SendMsg[1] = 0x0E;  // count
      SendMsg[2] = SLOT; 
       SendMsg[3] = STAT1; 
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

void MoveSlots ( uint8_t *recMsg)   {


    #if _SERIAL_DEBUG
          Serial.print(F("Slot change request :"));
         Serial.print(MyLocoAddr);
          Serial.print(F(" to MyLocoAddr :"));
         Serial.println( recMsg[2]);     
     #endif 
  MyLocoAddr= CV[18]+((CV[17]-192)*256);  /// should be recMsg[2]; ///FORCE KEEP MyLocoAddr = CV18 ?

   if (recMsg[2]==MyLocoAddr) { //only reply if we are target, else it could be another loco...
           setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;          
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
   }                  





  

}


void motorcommands (uint8_t *RECMsg,uint8_t Len) {
   uint16_t CVRequest;
   uint8_t CV_Data;
      if (RECMsg[0] == OPC_LOCO_ADR){  //loco request ++++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range  
            #if _SERIAL_DEBUG
          Serial.print(F("This Loco Message is for:"));
         Serial.print(CommandFor(RECMsg, Len));
          Serial.print(F("my addr is:"));
         Serial.println( MyLocoAddr);     
     #endif  
           if ((CommandFor(RECMsg, Len)== MyLocoAddr) ) { //.
           digitalWrite (BlueLed, LOW) ;  /// turn On 
    
 /*         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
SLOT, =1 - 119 active loco
STAT1, D6 D3 00 = Free loco  D5 D4 Busy/ Active  00 = free slot  D2,D1,D0 010=14 step speed
ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2
DIRF  loco direction and functions(4) 0,0,DIR,F0,F4,F3,F2,F1


         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
 */
          setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;                     }
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
                                       
                                        }  ////****** end of if (CommandOpCode  = OPC_SW_REQ)

    if (RECMsg[0] == OPC_MOVE_SLOTS){  //move slots.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                    MoveSlots(RECMsg);  // deals with message, updates MyLocoAddr
                                        
                                        }  ////****** end of if (CommandOpCode  = OPC_MOVE_SLOTSQ)

   if (RECMsg[0] == OPC_LOCO_SPD){  //loco speed.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){ 
                       Motor_Speed= (RECMsg[2]*90)/127;
                    
                    
    #if _SERIAL_DEBUG
          Serial.print(F("Dirn "));
          Serial.print(bitRead(DIRF, 5 )); 
          Serial.print(F(" request speed "));
          Serial.print(RECMsg[2]);
          Serial.print(F("    loco speed setting"));
          Serial.println(Motor_Speed);
     #endif   
                                         }  // if my dest addr..
                                     }  ////****** end of if (CommandOpCode  = OPC_LOCO_SPD)

 if (RECMsg[0] == OPC_LOCO_DIRF){  //loco DIRF.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){ 
                    DIRF=RECMsg[2]; 

                     if (bitRead(DIRF, 5 )) {
                       Motor_Servo= Motor_Speed +90; }
                     else {
                      Motor_Servo= 90-Motor_Speed;
                     }
    #if _SERIAL_DEBUG
          Serial.print(F("Dirn "));
          Serial.print(bitRead(DIRF, 5 ));
          Serial.print(F("   SERVO would be "));
          Serial.println(Motor_Servo);
     #endif  
     MyLocoDrive.write(Motor_Servo);
                                         }
                                     }  ////****** end of if (CommandOpCode  = OPC_LOCO_DIRF)


if (RECMsg[0] == OPC_RQ_SL_DATA){  //REQUEST SLOT DATA+++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){ 
           setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;          
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
                    
                   }
                 }

                 
if (RECMsg[0] == OPC_WR_SL_DATA){  //REQUEST CV  DATA+++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                    CVRequest=RECMsg[9]+(128*bitRead(RECMsg[8],0))+(256*bitRead(RECMsg[8],4))+(512*bitRead(RECMsg[8],5));
                    CV_Data=RECMsg[10]+(128*bitRead(RECMsg[8],1));
                   if (RECMsg[6]==MyLocoAddr){  // may need to improve 
                    if (bitRead(RECMsg[3],6)==0) {  // read
  #if _SERIAL_DEBUG
          Serial.print(F(" Read CV "));
          Serial.print(CVRequest+1);
          Serial.print(F("  ="));
          Serial.println(CV[CVRequest+1]);
     #endif  
  //send long ack first?
  
           delay(10);
           setOPC_LONG_ACK_Msg (sendMessage, 0x6F,  0x01)  ; // acknowledge
 #if _SERIAL_DEBUG
          Serial.print(F(" OPC_LONG_ACK_Msg ="));
          dump_byte_array(sendMessage, 0x04);
          Serial.println();
     #endif        
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 4);
              UDP.endPacket();
    //    ///Long ack sent             
     
              
              //  send the data.....
              // Flags for the 7th data Byte from LocoNet.Cpp line 1529
                 //#define LNCV_FLAG_PRON 0x80
                 //#define LNCV_FLAG_PROFF 0x40
                 //#define LNCV_FLAG_RO 0x01
delay(10);   
             //      msg       ,cmd, src       ,dstl,dsth, D1  ,d2          ,d3 ,d4 ,d5 ,d6 , d7  , d8
      BuildMessage(sendMessage,0xE5,MyLocoAddr,0x50,0x01,0x01 ,CVRequest+1 ,0  ,0  ,0  ,0  ,0x01 ,CV[CVRequest+1]);
// BuildUE5Message(uint8_t *SendMsg, uint8_t SRC, uint8_t DSTL,uint8_t DSTH,uint16_t Class, uint16_t LNCV, uint16_t LNCVAL, uint8_t FLAGS)
       //   BuildUE5Message(sendMessage,MyLocoAddr,0x50,0x01,                 0x01 ,CVRequest+1 ,CV[CVRequest+1],0x00);
      
  #if _SERIAL_DEBUG
        Serial.print("   Response Message"); 
        Serial.print(" CV[");
        Serial.print( CVRequest+1);          
        Serial.print("]=");
        Serial.print(CV[CVRequest+1]); 
        Serial.print("   Full Message =");
               dump_byte_array(sendMessage, 16);  
        Serial.println("");
  #endif 
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 16);
              UDP.endPacket();





              // data sent
                    } // end of READ
                    
                    else{  // its a write...
   #if _SERIAL_DEBUG
          Serial.print(F(" Write CV "));
          Serial.print(CVRequest+1);
          Serial.print(F("  ="));
          Serial.println(CV_Data);
     #endif  
        
          if ((CVRequest+1==8) && (CV_Data==8)){
            SetDefaultSVs();
               }
               else{
          CV[CVRequest+1]=CV_Data; 
                    }
          WriteSV2EPROM();  // update EEPROM
          EEPROM.commit();
          delay(100);
          EPROM2SV();
            setOPC_LONG_ACK_Msg (sendMessage, 0x6F,  0x01)  ; // acknowledge
 #if _SERIAL_DEBUG
          Serial.print(F(" OPC_LONG_ACK_Msg ="));
          dump_byte_array(sendMessage, 0x04);
          Serial.println();
     #endif        
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 4);
              UDP.endPacket();

     
                    }  // end of write cv





                    
                   }//end our MyLocoAddr
                 } //end request cv data



} // end motorcommands

