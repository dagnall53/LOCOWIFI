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
 // MyLocoLAddr=CV[18]+((CV[17]&0x3F)*256);  /// will be only the short address used here..  ?
  MyLocoAddr=CV[1];/// should be recMsg[2]; ///FORCE KEEP MyLocoAddr 
   if (recMsg[2]==MyLocoAddr) { //only reply if we are target, else it could be another loco...
         #if _SERIAL_DEBUG  
         Serial.print (F(" Responding :"));  
         Serial.println();
             #endif  
           setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;          
            dump_byte_array(sendMessage, 10);
           delay(CV[100]);
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
   }                  





  

}


void motorcommands (uint8_t *RECMsg) {
   uint16_t CVRequest;
   uint8_t CV_Data;
      if (RECMsg[0] == OPC_LOCO_ADR){  //loco request ++++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range  
     #if _SERIAL_DEBUG
          Serial.print(F("This Loco Message is for:"));
          Serial.print(CommandFor(RECMsg));
          Serial.print(F("my addr is:"));
          Serial.println( MyLocoAddr);     
     #endif  
           if ((CommandFor(RECMsg)== MyLocoAddr) ) { //.
           digitalWrite (BlueLed, LOW) ;  /// turn On 
    
 /*         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
SLOT, =1 - 119 active loco
STAT1, D6 D3 00 = Free loco  D5 D4 Busy/ Active  00 = free slot  D2,D1,D0 010=14 step speed
ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2
DIRF  loco direction and functions(4) 0,0,DIR,F0,F4,F3,F2,F1


         setOPC_SL_RD_DATA_Msg (sendMessage, SLOT,  STAT1, ADR, SPD,  DIRF, TRK,  SS2, ADR2,  SND,  ID1,  ID2)  
 */
          setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;                     }
              delay(CV[100]);
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
                                       
                                        }  ////****** end of if (CommandOpCode  = OPC_SW_REQ)

    if (RECMsg[0] == OPC_MOVE_SLOTS){  //move slots.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On   
     #if _SERIAL_DEBUG
          Serial.print(F("This Loco Mov slots Message")) ;     
     #endif  
                    MoveSlots(RECMsg);  // deals with message, updates MyLocoAddr
                                        
                                        }  ////****** end of if (CommandOpCode  = OPC_MOVE_SLOTSQ)

   if (RECMsg[0] == OPC_LOCO_SPD){  //loco speed.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){             
                       Motor_Speed= (RECMsg[2]*90)/127;            }  // if my dest addr..
                    if (bitRead(DIRF, 5 )) { Motor_Servo= Motor_Speed +90; }
                     else { Motor_Servo= 90-Motor_Speed;  }   
                                 }  ////****** end of if (CommandOpCode  = OPC_LOCO_SPD)
  if (RECMsg[0] == OPC_LOCO_DIRF){  //loco DIRF.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){ DIRF=RECMsg[2];       }
                    if (bitRead(DIRF, 5 )) { Motor_Servo= Motor_Speed +90; }   // need to ex or with the cv29 bit....
                     else { Motor_Servo= 90-Motor_Speed;  }          
     // IF LOCO will use servo 8, (in do periodicupdate   motorspeed  set SV's for 8 to Servo, but address to ??
     // will need to set motorspeed change as acc and dec CV functions and vary motor speed by loco cvs...
                                    }  ///
    if (RECMsg[0] == OPC_LOCO_SND){  //loco snd.. ++++++++++++++++++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){ SND= RECMsg[2];}  // if my dest addr..
                                LOCOINFO();       
                                }  ////****** end of if (CommandOpCode  = OPC_LOCO_SPD)

 //****** end of if (CommandOpCode  = OPC_LOCO_DIRF)


if (RECMsg[0] == OPC_RQ_SL_DATA){  //REQUEST SLOT DATA+++++++++++++++++++++
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                   if (RECMsg[1]==MyLocoAddr){     
                    #if _SERIAL_DEBUG
          Serial.print(F("This Loco slot request"));     
     #endif  
           setOPC_SL_RD_DATA_Msg (sendMessage, MyLocoAddr,  0x05, CV[18], Motor_Speed,  DIRF, 0x00,  0x00, CV[17],  SND,  0x01,  0x00)  ;          
              delay(CV[100]);
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
                    
                   }
                 }

                 
if (RECMsg[0] == OPC_WR_SL_DATA){  //REQUEST CV  DATA+++++++++++++++++++++
//<HOPSA>Operations Mode Programming- 7 High address bits of Loco to program, 0 if Service Mode
//<LOPSA>Operations Mode Programming- 7 Low address bits of Loco to program, 0 if Service Mode
//<TRK> Normal Global Track status for this Master, Bit 3 also is 1 WHEN Service Mode track is BUSY
//<CVH> High 3 BITS of CV#, and ms bit of DATA.7 <0,0,CV9,CV8 - 0,0, D7,CV7>
//<CVL> Low 7 bits of 10 bit CV address. <0,CV6,CV5,CV4-CV3,CV2,CV1,CV0>
//<DATA7>Low 7 BITS OF data to WR or RD COMPARE <0,D6,D5,D4 - D3,D2,D1,D0> ms bit is at CVH bit 1 position.
  //< 0  >,<1>, <2>,  <3>,  <4>,<5>,    <6>,    <7>,  <8>,  <9>,  <10>,   <11>,<12>,<CHK>
  //<0xEF>,<0E>,<7C>,<PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
            // ********** Look for messages in my range 
                    digitalWrite (BlueLed, LOW) ;  /// turn On 
                    CVRequest=(RECMsg[9]&0x7F)+(128*bitRead(RECMsg[8],0))+(256*bitRead(RECMsg[8],4))+(512*bitRead(RECMsg[8],5));
                    CV_Data=(RECMsg[10]&0x7F)+(128*bitRead(RECMsg[8],1));
                   if ((CommandFor(RECMsg)== MyLocoAddr)||(CommandFor(RECMsg)== MyLocoLAddr)||(CommandFor(RECMsg)== 0))   
                                                         {  // ?? Full check of possibilities? May need to add a CV29  bit to select L and S addr messages..
                    if (bitRead(RECMsg[3],6)==0) {  // read
  #if _SERIAL_DEBUG
          Serial.print(F(" Read CV "));
          Serial.print(CVRequest+1);
          Serial.print(F("  ="));
          Serial.println(CV[CVRequest+1]);
     #endif  
           delay(CV[100]);
           setOPC_LONG_ACK_Msg (sendMessage, 0x7F,  0x01)  ; // Task accepted , <E7> reply at completion
 #if _SERIAL_DEBUG
          Serial.print(F(" OPC_LONG_ACK_Msg ="));
          dump_byte_array(sendMessage, 0x04);
          Serial.println();
     #endif      
     delay(CV[100]);  
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 4);
              UDP.endPacket();

     delay(CV[100]);   
      //  <0xEF>,<0E>,<7C>,                     <PCMD>,<0>,<HOPSA>,<LOPSA>,<TRK>;<CVH>,<CVL>,<DATA7>,<0>,<0>,<CHK>
      BuildProgrammerResponseMessage(sendMessage,RECMsg[3], RECMsg[5],RECMsg[6],RECMsg[7],RECMsg[8],RECMsg[9],CV[CVRequest+1]);   // basically a copy of the command but with the requested data...
  #if _SERIAL_DEBUG
        Serial.print("   Response Message"); 
        Serial.print(" CV[");
        Serial.print( CVRequest+1);          
        Serial.print("]=");
        Serial.print(CV[CVRequest+1]); 
        Serial.print("   Full Message =");
               dump_byte_array(sendMessage, 14);  
        Serial.println("");
  #endif 
             delay(CV[100]);
             UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 14);
              UDP.endPacket();
              // data sent
                    } // end of READ
                    
                    else{  // its a write...
                
   #if _SERIAL_DEBUG
          Serial.print(F(" Write CV "));
          Serial.print(CVRequest+1);
          Serial.print(F("  ="));
          Serial.print(CV_Data);

       //   Serial.print(F(" TEST rec8 bit 0 "));
       //   Serial.print((bitRead(RECMsg[8],0)));  
       //   Serial.print(F(" rec8 bit1 ="));
       //   Serial.print((bitRead (RECMsg[8],1)));
       //   Serial.print(F("  rec8 bit3 ="));
       //   Serial.println((bitRead (RECMsg[8],3)));
          
     #endif  
        
          if ((CVRequest+1==8)){ 
            if((CV_Data==8)){
            SetDefaultSVs();
               }}
               else{
          CV[CVRequest+1]=CV_Data; 
                    }
          WriteSV2EPROM();  // update EEPROM
           Data_Updated= true; 
           EEprom_Counter=0;
          //EEPROM.commit();
          delay(100);
          EPROM2SV();
            setOPC_LONG_ACK_Msg (sendMessage, 0x6F,  0x01)  ; // acknowledge
 #if _SERIAL_DEBUG
          Serial.print(F(" OPC_LONG_ACK_Msg ="));
          dump_byte_array(sendMessage, 0x04);
          Serial.println();
     #endif    
     delay(CV[100]);    
              UDP.beginPacket(ipBroad, port);
              UDP.write(sendMessage, 4);
              UDP.endPacket();

     
                    }  // end of write cv





                    
                   }//end our MyLocoAddr
                 } //end request cv data



} // end motorcommands

