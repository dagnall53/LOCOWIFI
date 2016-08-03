/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
       Serial.print(buffer[i] < 0x10 ? " 0" : " ");
       Serial.print(buffer[i], HEX);
    }
}
void dump_byte(uint8_t buffer) {
 
       Serial.print(buffer < 0x10 ? " 0" : " ");
       Serial.print(buffer, HEX);
       Serial.print("H ");

}


uint8_t lnCalcCheckSumm(uint8_t *cMessage, uint8_t cMesLen) {
    unsigned char ucIdx = 0;
    char cLnCheckSum = 0;

    for (ucIdx = 0; ucIdx < cMesLen - 1; ucIdx++) //check summ is the last byte of the message
    {
        cLnCheckSum ^= cMessage[ucIdx];
    }

    return (~cLnCheckSum);
}


void setOPCMsg (uint8_t *SendMsg,uint16_t addr, int InputState )  
{
  unsigned char k = 0;
  uint16_t tempaddr; 
   SendMsg[0] = OPC_INPUT_REP; //OPC_INPUT_REP         
        bitClear (SendMsg[1],7);
        bitWrite (SendMsg[1],6, bitRead(addr,7));
        bitWrite (SendMsg[1],5, bitRead(addr,6));
        bitWrite (SendMsg[1],4, bitRead(addr,5));
        bitWrite (SendMsg[1],3, bitRead(addr,4));
        bitWrite (SendMsg[1],2, bitRead(addr,3));
        bitWrite (SendMsg[1],1, bitRead(addr,2));
        bitWrite (SendMsg[1],0, bitRead(addr,1));

        bitClear (SendMsg[2],7);
        bitSet   (SendMsg[2],6);  //X=1
        bitWrite (SendMsg[2],5, bitRead(addr,0));
        bitWrite (SendMsg[2],4, !InputState);   
        bitWrite (SendMsg[2],3, bitRead(addr,11));  
        bitWrite (SendMsg[2],2, bitRead(addr,10)); 
        bitWrite (SendMsg[2],1, bitRead(addr,9));
        bitWrite (SendMsg[2],0, bitRead(addr,8));
       
      
     
          
        SendMsg[3]=0xFF;
        for(k=0; k<3;k++){                                   //Make checksum for this 4 byte message
        SendMsg[3] ^= SendMsg[k];
                         }
}

uint16_t AddrFull (uint8_t HI, uint8_t LO) {
   uint16_t FullAddress;
   FullAddress= 128*(HI & 0x0F) + LO;
   return (FullAddress);
}




uint16_t CommandFor (uint8_t *RECMsg) {
   uint16_t Address;
   Address= 255;
   switch (RECMsg[0]){       
    case 0xA0: // OPC_LOCO_SND)
       Address= ( RECMsg[1]); 
        break;      
    case 0xA1: // OPC_LOCO_SND)
       Address= ( RECMsg[1]); 
        break;    
    case 0xA2: // OPC_LOCO_SND)
       Address= ( RECMsg[1]); 
        break; 
    case 0xB0://OPC_SW_REQ) 
       Address= ((128*(RECMsg[2]&0x0F)+RECMsg[1])+1); 
    break;
    case 0xBB: //OPC_RQ_SL_DATA)
       Address= ( RECMsg[1]); 
        break; 
     case 0xBF: // OPC_LOCO_ADR) 
        Address= ( (128*(RECMsg[1])+RECMsg[2])); 
       break;
    case 0xE5: // OPC_PEER_XFER)
        Address= ((128*(RECMsg[11]&0x0F)+RECMsg[3])); 
       break;
    case 0xEF: // OPC_WR_SL_DATA)
        Address= ( RECMsg[6]+(128*RECMsg[5])); 
        break;
    default:
        Address=255;
    break;
   }
 return (Address);
}

uint8_t CommandData (uint8_t *RECMsg, uint8_t len) {
   uint8_t Data;
   bitWrite (Data, 1, bitRead(RECMsg[2],5));
   bitWrite (Data, 0, bitRead(RECMsg[2],4)); 
  
 return (Data);
}
uint8_t CommandOpCode (uint8_t *RECMsg, uint8_t len) {
   uint8_t Data;
    Data = RECMsg[0];
 return (Data);
}


void SetServo (int i, int pos){  // expects i = 1 to 8   maps back to "D1- D8" 

 if ((((SV[3*i]& 0x88) ==0x88)) ||(i==8)){  // only if this port is a servo...(case 8 has its own check  

#if _SERIAL_SUBS_DEBUG
         Serial.print("Servo on address set at Port:");
         Serial.print(i);
         Serial.print(" using speed SV[");
         Serial.print(100+(3*i));
         Serial.print("] ");  
         Serial.print(" outputs to ");
      #endif  
switch (pos){  // evaluate request  
  case 0:            //never seen this case, but It may be possible? 
   SDemand[i]= (SV[98+(3*i)]+SV[99+(3*i)])/2;
    break;
  case 1:
  SDemand[i]= SV[98+(3*i)];    //servoThrown[
    break;
  case 2:
    SDemand[i]= (SV[98+(3*i)]+SV[99+(3*i)])/2;
    break;
  case 3:
 SDemand[i]= SV[99+(3*i)]; //servoStraight[i];
    break; 
  case 254:
  SDemand[i]= Loco_motor_servo_demand/2; // special case for loco!
    break;
  }

  SDemand[i]=SDemand[i]*2; // Range variable are 1-127 so double them to give sensible range for servo
if (SDemand[i]>=180) {SDemand[i]=180;}  //limit at 180 degrees
   SDelay[i]=1000;          // set so next doPeriodicservo will do its stuff stuff now
   ServoOffDelay[i]=0;   //SET THE DELAY COUNTER FOR THIS SERVO

switch  (i){  // we sort out which servo to do immediately now .. case 1= the Base Address
  case 0:
   if (!myservo0.attached()) {myservo0.attach(D0); }// re attach servos in case they have been switched off
   // if (SV[100+(3*i)]==0) {myservo0.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D0 ");
               #endif 
   break;
  case 1:
   if (!myservo1.attached()) {myservo1.attach(D1); }// re attach servos in case they have been switched off
   // if (SV[100+(3*i)]==0) {myservo1.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
     #if _SERIAL_SUBS_DEBUG
         Serial.print(" D1 ");
               #endif 
             
  break;
  case 2:

   if (!myservo2.attached()) {myservo2.attach(D2); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo2.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D2 ");
               #endif 
          
  break;
    case 3:
   if (!myservo3.attached()) {myservo3.attach(D3); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo3.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D3 ");
               #endif 
  break;
  case 4:
   if (!myservo4.attached()) {myservo4.attach(D4); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo4.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D4 ");
               #endif 
  break;
    case 5:
if (!myservo5.attached()) {myservo5.attach(D5); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo1.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D5 ");
               #endif 
             
  break;
    case 6:
 if (!myservo6.attached()) {myservo6.attach(D6); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo6.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D6 ");
               #endif 
             
  break;
    case 7:
   if (!myservo7.attached()) {myservo7.attach(D7); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo7.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D7 ");
               #endif 
  break;    
  case 8:
   if (!myservo8.attached()) {myservo8.attach(D8); }// re attach servos in case they have been switched off
 //   if (SV[100+(3*i)]==0) {myservo8.write(SDemand[i]);} // if 1 instant..elsewill be slowly run in doperiodicservo
    #if _SERIAL_SUBS_DEBUG
         Serial.print(" D8 ");
               #endif 
  break;
     }   
/////SDemand (i) is now set, leave it to doPeriodic Servo...
     #if _SERIAL_SUBS_DEBUG
         Serial.print(" requests state:");
         Serial.print(pos);
         Serial.print(" to be set to position :");
         Serial.print(SDemand[i]);
         Serial.print(" at speed '"); 
         Serial.println (SV[100+(3*i)]);
      #endif  

} // only if this port is a servo
  }  // set servo



uint8_t Debounce (int i) {  // Tests for inputs having changed, returns value of input changed if true
  uint8_t SwitchSensed=255;     //AND 256 IF NO CHANGE
 /*
 #if _SERIAL_SUBS_DEBUG
         Serial.print(F("Debounce: "));
         Serial.print(i);
          Serial.print(" Button state:");
         Serial.println(buttonState[i]);
            Serial.print(" Last state :");
         Serial.println(lastButtonState[i]);      
         Serial.print(" Debounce Time :");
         Serial.println(DebounceTime[i]);
         Serial.println("   ");
#endif  
*/
   if (lastButtonState[i] != buttonState[i]) { 
    DebounceTime[i]= DebounceTime[i]+1;
                                            }
    if (DebounceTime[i] >= DebounceDelay ) {
     SwitchSensed = i;  
                    
    }
                    
                        
 return (SwitchSensed);
  }



uint8_t OPCPeerRX(uint8_t *RECMsg){
 uint8_t PXCT1;
  uint8_t PXCT2;

PXCT1= RECMsg[5];
PXCT2= RECMsg[10];
OPC_Data[1]=RECMsg[6]+128*bitRead(PXCT1,0);
OPC_Data[2]=RECMsg[7]+128*bitRead(PXCT1,1);
OPC_Data[3]=RECMsg[8]+128*bitRead(PXCT1,2);
OPC_Data[4]=RECMsg[9]+128*bitRead(PXCT1,3);
OPC_Data[5]=RECMsg[11]+128*bitRead(PXCT1,0);
OPC_Data[6]=RECMsg[12]+128*bitRead(PXCT1,1);
OPC_Data[7]=RECMsg[13]+128*bitRead(PXCT1,2);
OPC_Data[8]=RECMsg[14]+128*bitRead(PXCT1,3);
//for (int i=1;i<9;i++){
 // Serial.print("OPC_Data ");
 // Serial.print(i);
 // Serial.print("  =");
//  dump_byte(OPC_Data[i]);
//}
//Serial.println(" ");
  //PXCT1 = x x x x ...then MSB of .. [9] [8] [7] [6]
    //PXCT2 = x x x x ...then MSB of .. [14] [13] [12] [11]
OPCSRCL = RECMsg[3];
OPCRXTX= OPC_Data[1];
OPCSV= OPC_Data[2];
OPCdata = OPC_Data[4]&0x7F;  // limit as rocrail ??
OPCSub =OPC_Data[5];

}
void OPCResponse(uint8_t *SendMsg,uint8_t RXTX, uint8_t ADHI, uint8_t ADLO,uint8_t SV, uint8_t Data1, uint8_t Data2, uint8_t Data3)
{
  int k;
  uint8_t PXCT1;
  uint8_t PXCT2;

 SendMsg[0]= OPC_PEER_XFER ;
 SendMsg[1]=  0x10;     //  Message length
 SendMsg[2]=  ADLO;    // LocoIO low address
 SendMsg[3]=  0x50;     // always 50 ow address byte of Locobuffer
 SendMsg[4]=  0x01;     // always 01 high address byte of Locobuffer

 SendMsg[6]=  (RXTX &0x7F);    // LOCOIO_SV_WRITE Original sent command
 SendMsg[7]=  (SV & 0x7F);      //SV number requested
 SendMsg[8]=  0x7F ;    //// Lower 7 bits of LocoIO version
 SendMsg[9]=  0x00;     // 00
 PXCT1 = 0;
   //PXCT1 = x x x x ...then MSB of .. [9] [8] [7] [6]
    //PXCT2 = x x x x ...then MSB of .. [14] [13] [12] [11]
 bitWrite (PXCT1, 0, bitRead(RXTX,7));
 bitWrite (PXCT1, 1, bitRead(SV,7));
 bitWrite (PXCT1, 2, bitRead(SendMsg[8],7));
 bitWrite (PXCT1, 3, bitRead(SendMsg[9],7));
 SendMsg[5]=  PXCT1;     //// PXCT1 high order bits 
 SendMsg[11]=  (ADHI & 0x7F);     //LocoIO Sub-address hi
 SendMsg[12]=  (Data1 & 0x7F);   //requested data
 SendMsg[13]=  (Data2 & 0x7F);  //data +1
 SendMsg[14]=  (Data3 & 0x7F);  // data +2 or written data if "Write"
 PXCT2 = 0;
 bitWrite (PXCT2, 0, bitRead(ADHI,7));
 bitWrite (PXCT2, 1, bitRead(Data1,7));
 bitWrite (PXCT2, 2, bitRead(Data2,7));
 bitWrite (PXCT2, 3, bitRead(Data3,7));
  SendMsg[10]=  PXCT2;    // // High order bit of requested dataMSB requested data 

  SendMsg[15]=0xFF;  //checksum
        for(k=0; k<15;k++){                                   //Make checksum for this three byte message
        SendMsg[15] ^= SendMsg[k]; 
}
}


void WriteSV2EPROM(){
           
for (int i=0; i<=255 ;i++){
  EEPROM.write(i,SV[i]);
  EEPROM.write(i+256,CV[i]);
                          }
}

void EPROM2SV(){
  for (int i=0; i<=255 ;i++){
  SV[i]=EEPROM.read(i);
  CV[i]=EEPROM.read(i+256);
                          }
}

uint16_t SensorAddress(uint8_t i){
uint16_t Addr;
 
Addr = SV[1+(3*i)];
Addr= Addr*2;
Addr= Addr + bitRead(SV[2+(3*i)],5);
Addr=Addr+(256*(SV[2+(3*i)]&0x0F));
Addr=Addr+1; //to match rocrail numbering..
/*
#if _SERIAL_SUBS_DEBUG
          Serial.print(F(" Sensor Address from SV:"));
          Serial.print(i);
          Serial.print(" SV[");
          Serial.print(2+(3*i));
          Serial.print("] = ");
          Serial.print( SV[(2+(3*i))]);
          Serial.print("   SV[");
          Serial.print(1+(3*i));
          Serial.print("] = ");
          Serial.print( SV[(1+(3*i))]);
          Serial.print("   Computes to: ");
          Serial.println( Addr+1);
       
     #endif  
*/





return Addr;
  
}




void doPeriodicServo() {  // attaches and detatches servos
  int ServoPositionNow;
  int Servodemand;
  int Servoattached;
  int offset;
 for (int i=1 ; i<=8; i++) {
  if ((((SV[3*i]& 0x88) ==0x88)) ||((i==8)&(LOCO==1))){  // only if this port is a servo... or i = 8 and loco
 
     SDelay[i]=SDelay[i]+1;
  if (SDelay[i] >= SloopDelay){   // only do this at intervals...
                   SDelay[i]=0;    

   
switch  (i){  // we sort out which servo to operate here.. case 1= the Base Address
  case 0:
    ServoPositionNow= myservo0.read();
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > SV[100+(3*i)]){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo0.write(ServoPositionNow+offset);
    Servoattached= myservo0.attached();
      if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo0.attached()) { myservo0.detach();  } }
    break;
  case 1:
    ServoPositionNow= myservo1.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo1.write(ServoPositionNow+offset);
    Servoattached= myservo1.attached();
     if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo1.attached()) { myservo1.detach();  } }

  break;
  case 2:
    ServoPositionNow= myservo2.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo2.write(ServoPositionNow+offset);
    Servoattached= myservo2.attached();
   if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo2.attached()) { myservo2.detach();  } }

  break;
    case 3:
    ServoPositionNow= myservo3.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo3.write(ServoPositionNow+offset);
    Servoattached= myservo3.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo3.attached()) { myservo3.detach();  } }

  break;
    case 4:
    ServoPositionNow= myservo4.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo4.write(ServoPositionNow+offset);
    Servoattached= myservo3.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo4.attached()) { myservo4.detach();  } }

  break;  
   case 5:
    ServoPositionNow= myservo5.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo5.write(ServoPositionNow+offset);
    Servoattached= myservo5.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo5.attached()) { myservo5.detach();  } }

  break;    
  case 6:
    ServoPositionNow= myservo6.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo6.write(ServoPositionNow+offset);
    Servoattached= myservo6.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo6.attached()) { myservo6.detach();  } }

  break;   
  case 7:
    ServoPositionNow= myservo7.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo7.write(ServoPositionNow+offset);
    Servoattached= myservo7.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo7.attached()) { myservo7.detach();  } }

  break;    
  case 8:
    if (LOCO==1){
    // this is the srvo for the loco throttle 
    ServoPositionNow= myservo8.read(); 
    SDemand[i]=Loco_motor_servo_demand;
    offset= Loco_motor_servo_demand-ServoPositionNow;
    if (abs(offset) > (CV[3]+1)){  // can be Cv's later...
      offset= (offset*(CV[3]+1))/abs(offset);
    }
    if ((ServoPositionNow+offset<=180) && ((ServoPositionNow+offset) >=1)){
      myservo8.write(ServoPositionNow+offset);    // if you play with the CV values this can go temporarily crazy...
    }
   // if (Loco_motor_servo_demand==90){
   //  myservo8.write(Loco_motor_servo_demand);  // set mid position immediately!
   // }
    Servoattached= myservo8.attached();
    }
    else{
       if (((SV[3*i]& 0x88) ==0x88)) {  // only if this port is a servo...
    ServoPositionNow= myservo8.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > (SV[100+(3*i)]+1)){
      offset= (offset*(SV[100+(3*i)]+1))/abs(offset);
    }
    myservo8.write(ServoPositionNow+offset);
    Servoattached= myservo8.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
       if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo8.attached()) { myservo8.detach();  } }
               
   }
      
      }
   

   break;      }
if (SloopDelay >=20){  // only send debug if updating slowly... 
    #if _SERIAL_DEBUG
         Serial.println();
         Serial.print(F("Servo:"));
         Serial.print(i); 
         Serial.print("  attached="), 
           Serial.print(Servoattached); 
         Serial.print(" Position now=");
         Serial.print(ServoPositionNow); 
          Serial.print(" demand is=");
         Serial.print(SDemand[i]);    
         Serial.print(" delay is=");
         Serial.print(ServoOffDelay[i]);
         Serial.println( );
     #endif  
  }
 
   }
 }
  } }
  
  void BuildMessage(uint8_t *SendMsg,uint8_t Command, uint8_t SRC, uint8_t DSTL,uint8_t DSTH,uint8_t D1, uint8_t D2, uint8_t D3, uint8_t D4, uint8_t D5, uint8_t D6, uint8_t D7, uint8_t D8)
{
  int k;
  uint8_t PXCT1;
  uint8_t PXCT2;

 SendMsg[0]= Command ;
 SendMsg[1]=  0x10;     //  Message length
 SendMsg[2]=  SRC;    // Arg1
 SendMsg[3]=  DSTL;     // Arg2
 SendMsg[4]=  DSTH;     // arg3
                         // arg4 = PXCT1
 SendMsg[6]=  (D1 & 0x7F);    // Arg 5
 SendMsg[7]=  (D2 & 0x7F);      // Arg 6
 SendMsg[8]= (D3 & 0x7F) ;    ////  Arg 7
 SendMsg[9]=  (D4 & 0x7F);     // Arg 8
                              //Arg 9
 SendMsg[11]=  (D5 & 0x7F);     //Arg10
 SendMsg[12]=  (D6 & 0x7F);   //Arg 11
 SendMsg[13]=  (D7 & 0x7F);  //Arg 12
 SendMsg[14]=  (D8 & 0x7F);  // Arg 13
 
 PXCT1 = 0;
   //PXCT1 = x x x x ...then MSB of .. [9] [8] [7] [6]
    //PXCT2 = x x x x ...then MSB of .. [14] [13] [12] [11]
 bitWrite (PXCT1, 0, bitRead(D1,7));
 bitWrite (PXCT1, 1, bitRead(D2,7));
 bitWrite (PXCT1, 2, bitRead(D3,7));
 bitWrite (PXCT1, 3, bitRead(D4,7));
 SendMsg[5]=  PXCT1;     //// PXCT1 high order bits 
 
 PXCT2 = 0;
 bitWrite (PXCT2, 0, bitRead(D5,7));
 bitWrite (PXCT2, 1, bitRead(D6,7));
 bitWrite (PXCT2, 2, bitRead(D7,7));
 bitWrite (PXCT2, 3, bitRead(D8,7));
   SendMsg[10]=  PXCT2;    // // High order bit of requested dataMSB requested data 

  SendMsg[15]=0xFF;  //checksum
        for(k=0; k<15;k++){                                   //Make checksum for this three byte message
        SendMsg[15] ^= SendMsg[k]; 
}
}

void BuildUE5Message(uint8_t *SendMsg, uint8_t SRC, uint8_t DSTL,uint8_t DSTH,uint16_t CLASS, uint16_t LNCV, uint16_t LNCVAL, uint8_t FLAGS)
{
  uint8_t PXCT1;
Serial.println("in BuildEU5 message");

 SendMsg[0]= 0xE5 ;  // Uhlengbock E5 message, see https://groups.yahoo.com/neo/groups/loconet_hackers/conversations/messages/9288
                        //Opcode  OPC_PEER_XFER (0E5h) - for replies (or 'spontaneous messages')
                        //OPC_IMM_PACKET (0EDh) - for messages which expect a reply
 SendMsg[1]=  0x0F;     //  Message length
 SendMsg[2]=  SRC;      // Arg1
 SendMsg[3]=  DSTL;     // Arg2
 SendMsg[4]=  DSTH;     // arg3
 SendMsg[5]=  0x21;    // CMD  ==readLNCV                        

 SendMsg[7]=  ((CLASS&0xFF00)/256);      // D1
 SendMsg[8]=  (CLASS&0xFF) ;          // D2
 SendMsg[9]=  ((LNCV&0xFF00)/256);     // D3
 SendMsg[10]=  (LNCV&0xFF);        //D4
 SendMsg[11]=  ((LNCVAL&0xFF00)/256);   //D5
 SendMsg[12]=  (LNCVAL&0xFF);  //D6
 SendMsg[13]=  (FLAGS);  // D7
 
 PXCT1 = 0;// PXCT1   0, D7.7, D6.7, D5.7, D4.7, D3.7, D2.7, D1.7
 bitWrite (PXCT1, 0, bitRead(SendMsg[7],7));
 bitWrite (PXCT1, 1, bitRead(SendMsg[8],7));
 bitWrite (PXCT1, 2, bitRead(SendMsg[9],7));
 bitWrite (PXCT1, 3, bitRead(SendMsg[10],7));
 bitWrite (PXCT1, 4, bitRead(SendMsg[11],7));
 bitWrite (PXCT1, 5, bitRead(SendMsg[12],7));
 bitWrite (PXCT1, 6, bitRead(SendMsg[13],7));
 SendMsg[6]=  PXCT1;    // PXCT1   0, D7.7, D6.7, D5.7, D4.7, D3.7, D2.7, D1.7
   // // remove the High order bit of requested dataMSB requested data 
   Serial.println("in BuildEU5 message  decapitating data");
   for (int i=7;i<=13;i++){
    SendMsg[i]= SendMsg[i]&0x7F;
   }
  SendMsg[14]=0xFF;  //checksum
        for(int k=0; k<14;k++){                                   //Make checksum for this  message
        SendMsg[14] ^= SendMsg[k]; 
}
}

   // <0xEF>,<0E>,<7C>,                     <PCMD>,<0>,   <HOPSA>,  <LOPSA>,<TRK>,    <CVH>,    <CVL>,    <DATA7>,<0>,<0>,<CHK>
   //  BuildProgrammerResponseMessage(sendMessage,RECMsg[3], RECMsg[5],RECMsg[6],RECMsg[7],RECMSG[8],RECMsg[9],CV[CVRequest+1]);
void  BuildProgrammerResponseMessage(uint8_t *SendMsg, uint8_t PCMD, int8_t HOPSA,uint8_t LOPSA, uint8_t TRK, uint8_t CVH, uint8_t CVL, uint8_t DATA7)
{
//Serial.print("in Build Programmer Response Message  DATA7=");
//Serial.print(DATA7);
//  Serial.print();
//    Serial.println();
 SendMsg[0]= 0xE7 ;  // 
 SendMsg[1]=  0x0E;     //  Message length
 SendMsg[2]=  0x7C;      // Arg1
 SendMsg[3]=  PCMD;
 SendMsg[4]=  0x00;     // PSTAT = 0 no errors...
 SendMsg[5]=  HOPSA;     // arg3
 SendMsg[6]=  LOPSA;    // CMD  ==readLNCV  
 SendMsg[7]=  TRK;      // D1   
 SendMsg[8]=  CVH ;          // D2                    
 bitWrite (SendMsg[8],1, bitRead(DATA7,7));  // msb of data7 goes in CVH bit 1
  SendMsg[9]=  CVL;     // D3
 SendMsg[10]= (DATA7 &0x7F) ;        //D4
 SendMsg[11]=  0x00;   //D5
 SendMsg[12]=  0x00;  //D6
 //SendMsg[13]=  (FLAGS);  // checksum

  SendMsg[13]=0xFF;  //checksum
        for(int k=0; k<13;k++){                                   //Make checksum for this  message
        SendMsg[13] ^= SendMsg[k]; 
}
}



void setMessageHeader(uint8_t *SendPacketSensor){
    unsigned char k = 0;
    SendPacketSensor[0] = 0xE4; //OPC - variable length message 
    SendPacketSensor[1] = uiLnSendLength; //14 bytes length
    SendPacketSensor[2] = 0x41; //report type 
    SendPacketSensor[3] = (FullBoardAddress >> 7) & 0x7F; //ucAddrHiSen; //sensor address high
    SendPacketSensor[4] = FullBoardAddress & 0x7F;; //ucAddrLoSen; //sensor address low 
   
    SendPacketSensor[uiLnSendCheckSumIdx]=0xFF;
    for(k=0; k<5;k++){
      SendPacketSensor[uiLnSendCheckSumIdx] ^= SendPacketSensor[k];
    }
}

void copyUid (byte *buffIn, byte *buffOut, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
        buffOut[i] = buffIn[i];
    }
    if(bufferSize < UID_LEN){
       for (byte i = bufferSize; i < UID_LEN; i++) {
           buffOut[i] = 0;
       }    
    }
}

void calcAddrBytes(uint16_t uiFull, uint8_t *uiLo, uint8_t *uiHi){
    *uiHi = (((uiFull - 1)  >> 8) & 0x0F) + (((uiFull-1) & 1) << 5);
    *uiLo = ((uiFull - 1) & 0xFE) / 2;  
}

/**
 * Function used to compare two RFID UIDs
 */
bool compareUid(byte *buffer1, byte *buffer2, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
       if(buffer1[i] != buffer2[i]){
         return(false);
       }
    }
  return(true);
}

void LocoUpdate( byte Ref){   
  
  #if _SERIAL_DEBUG
  Serial.print(Ref);
  if(bitRead(CV[29],0)){            // need to account for the  cv29 bit 0....
          if(bitRead(DIRF, 5 )){
             Serial.print(" Backwards"); 
             Loco_motor_servo_demand= Motor_Speed +90;}
          else{
             Serial.print(" Forwards"); 
             Loco_motor_servo_demand= 90-Motor_Speed;}
                      }
                 else {
    if(bitRead(DIRF, 5 )){
            Serial.print(" Forwards"); 
            Loco_motor_servo_demand= 90-Motor_Speed;}
          else{
            Serial.print(" Backwards");
            Loco_motor_servo_demand= Motor_Speed +90; }
                      }  
          Serial.print(F(" @ Speed :"));
          Serial.print(Motor_Speed);
          Serial.print(F(" Servo :"));
          Serial.print(Loco_motor_servo_demand);
          Serial.print(F(" Functions: "));
          Serial.println(DIRF&0x1F);
     #endif  
}
void UDPSEND (uint8_t *SendMsg, uint8_t Len,int DELAY){

   delay(DELAY);
   
              UDP2.beginPacket(ipBroad, port);
              UDP2.write(SendMsg, Len);
              UDP2.endPacket(); 
}
void UDPFetch (uint8_t *recMessage){
   uint8_t recLen = UDP.parsePacket();
   Message_Length= recLen;  // test to try to avoid calling parsepacket multiple times inside loop..
   if(Message_Length!=0){    // Check if a rocrail client has connected
          digitalWrite (BlueLed, LOW) ;  /// turn On 
          UDP.read(recMessage, Message_Length);
          UDP.flush();
     
  }
}
void Show_MSG(){
   #if _SERIAL_DEBUG      
         Serial.println(); 
         Serial.print(F("From:"));
         IPAddress remote = UDP.remoteIP();
         Serial.print(remote);
         Serial.print(F(" Msg to :"));
         Serial.print(CommandFor(recMessage));
         Serial.print(F(" Len:"));
         Serial.print(Message_Length); 
         Serial.print(F(" This board address:"));
         Serial.print(FullBoardAddress); 
         Serial.print(F(" This Loco (Short)Addr :"));
         Serial.print(MyLocoAddr); 
         Serial.print(F("  '(Long)Addr' is:"));
         Serial.println(MyLocoLAddr); 
         Serial.print(" Full Msg:");     
         dump_byte_array(recMessage, Message_Length);
         Serial.println( );
     #endif   
}

