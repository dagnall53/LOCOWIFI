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
         Serial.print(DEST);
          Serial.print(F(" to DEST :"));
         Serial.println( recMsg[2]);     
     #endif 
  DEST= CV[18];  /// should be recMsg[2]; ///FORCE KEEP DEST = CV18 ?

   if (recMsg[2]==CV[18]) { //only reply if we are target, else it could be another loco...
           setOPC_SL_RD_DATA_Msg (sendMessage, CV[18],  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;          
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
   }                  





  

}


void motorcommands (uint8_t *RECMsg,uint8_t Len) {
  
      if (RECMsg[0] == OPC_LOCO_ADR){  //loco request ++++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range  
            #if _SERIAL_DEBUG
          Serial.print(F("This Loco Message is for:"));
         Serial.print(CommandFor(RECMsg, Len));
          Serial.print(F("my addr is:"));
         Serial.println( CV[18]);     
     #endif  
           if ((CommandFor(RECMsg, Len)== CV[18]) ) { // may need to include CV[17] later...
           digitalWrite (BlueLed, LOW) ;  /// turn On 
    
 /*         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
SLOT, =1 - 119 active loco
STAT1, D6 D3 00 = Free loco  D5 D4 Busy/ Active  00 = free slot  D2,D1,D0 010=14 step speed
ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2
DIRF  loco direction and functions(4) 0,0,DIR,F0,F4,F3,F2,F1


         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
 */
          setOPC_SL_RD_DATA_Msg (sendMessage, CV[18],  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;                     }
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
                                       
                                        }  ////****** end of if (CommandOpCode  = OPC_SW_REQ)

    if (RECMsg[0] == OPC_MOVE_SLOTS){  //move slots.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                    MoveSlots(RECMsg);  // deals with message, updates DEST
                                        
                                        }  ////****** end of if (CommandOpCode  = OPC_MOVE_SLOTSQ)

   if (RECMsg[0] == OPC_LOCO_SPD){  //loco speed.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==DEST){ 
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
                   if (RECMsg[1]==DEST){ 
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
                   if (RECMsg[1]==DEST){ 
           setOPC_SL_RD_DATA_Msg (sendMessage, DEST,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;          
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
                    
                   }



}




} // end motorcommands

