/**
 * Helper routine to dump a byte array as hex values to Serial.
 */
void dump_byte_array(byte *buffer, byte bufferSize) {
    for (byte i = 0; i < bufferSize; i++) {
       Serial.print(buffer[i] < 0x10 ? " 0" : " ");
       Serial.print(buffer[i], HEX);
    }
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


void setOPCMsg (uint8_t *SendMsg, int i, uint16_t addr, int InputState )  
{
  unsigned char k = 0;
  uint16_t tempaddr; 
   SendMsg[0] = OPC_INPUT_REP; //OPC_INPUT_REP 
        tempaddr = addr;
        SendMsg[1] = tempaddr & 0xFE;
      SendMsg[1] = SendMsg[1] >> 1;
        bitClear (SendMsg[1],7);
        bitSet (SendMsg[2],6);  //X=1
        bitWrite (SendMsg[2],5, bitRead(addr,0));
        bitWrite (SendMsg[2],4, !InputState);
        bitWrite (SendMsg[2],0, bitRead(addr,7));
        bitWrite (SendMsg[2],1, bitRead(addr,8));
        bitWrite (SendMsg[2],2, bitRead(addr,9));
        bitWrite (SendMsg[2],3, bitRead(addr,10));
          
        SendMsg[3]=0xFF;
        for(k=0; k<3;k++){                                   //Make checksum for this three byte message
        SendMsg[3] ^= SendMsg[k];
                         }
}

uint16_t AddrFull (uint8_t HI, uint8_t LO) {
   uint16_t FullAddress;
   FullAddress= 128*(HI & 0x0F) + LO;
   return (FullAddress);
}




uint16_t CommandFor (uint8_t *RECMsg, uint8_t len) {
   uint16_t Address;
   Address= 0;
   if(RECMsg[0] == OPC_SW_REQ){ Address= ((128*(RECMsg[2]&0x0F)+RECMsg[1])+1); }
    if(RECMsg[0] == OPC_PEER_XFER){ Address= ((128*(RECMsg[11]&0x0F)+RECMsg[3])); }
    if(RECMsg[0] == OPC_LOCO_ADR){ Address= ( RECMsg[2]); }
    if(RECMsg[0] == OPC_LOCO_SND){ Address= ( RECMsg[1]); }
    if(RECMsg[0] == OPC_LOCO_DIRF){ Address= ( RECMsg[1]); }
    if(RECMsg[0] == OPC_LOCO_SPD){ Address= ( RECMsg[1]); }
    if(RECMsg[0] == OPC_RQ_SL_DATA){ Address= ( RECMsg[1]); }
    
    
    
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


void SetServo (int i, int pos){
#if _SERIAL_SUBS_DEBUG
         Serial.print(F("Servo on D"));
         Serial.print(i-1);   // if you change ports set this correctly!...
         Serial.print(" requests state:");
         Serial.print(pos);
         Serial.print(" will be set to position :");
     #endif  
switch (pos){
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
  }
#if _SERIAL_SUBS_DEBUG
         Serial.print(SDemand[i]);
          Serial.print(" at speed '"); 
           Serial.print(SV[100+(3*i)]);
         Serial.println("'  {NOTE: 2 deg steps, so Range[1..90] = 2 to 180 degrees}");
         #endif  
SDemand[i]=SDemand[i]*2; // Range variable are 1-127 so double them to give sensible range for servo
if (SDemand[i]>=180) {SDemand[i]=180;}  //limit at 180 degrees
   SDelay[i]=1000;          // set so next doPeriodicservo will do its stuff stuff now
   ServoOffDelay[i]=0; 
   switch  (i){  // we sort out which servo to operate here.. case 1= the Base Address
// re attach servos in case they have been switched off
  case 1:
    if (!myservo0.attached()) {myservo0.attach(D0);}
   break;
  case 2:
    if (!myservo1.attached()) {myservo1.attach(D0);}  /// fiddled to avoid loco deiver for this testt
  break;
  case 3:
     if (!myservo2.attached()) {myservo2.attach(D2);}
  break;
    case 4:
    if (!myservo3.attached()) {myservo3.attach(D3);}
  break;
        }
if (SV[100+(3*i)]==5) {  // if 5 instant..elsewill be slowly run in doperiodicservo
switch  (i){  // we sort out which servo to do immediately now .. case 1= the Base Address
  case 1:
    myservo0.write(SDemand[i]);
   break;
  case 2:
    myservo1.write(SDemand[i]);
  break;
  case 3:
    myservo2.write(SDemand[i]);
  break;
    case 4:
    myservo3.write(SDemand[i]);
  break;
        }
}//IF sv=1
/////SDemand (i) is now set, leav it to do Periodic Servo...



  }



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
  //PXCT1 = x x x x ...then MSB of .. [9] [8] [7] [6]
    //PXCT2 = x x x x ...then MSB of .. [14] [13] [12] [11]
OPCSRCL = RECMsg[3];
OPCRXTX= RECMsg[6]+128*bitRead(PXCT1,0);
OPCSV= RECMsg[7]+128*bitRead(PXCT1,1);
OPCdata = RECMsg[9]+128*bitRead(PXCT1,3);
OPCSub =RECMsg[11]+128*bitRead(PXCT2,3);

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

 SendMsg[6]=  RXTX;    // LOCOIO_SV_WRITE Original sent command
 SendMsg[7]=  (SV & 0x7F);      //SV number requested
 SendMsg[8]=  0x7F ;    //// Lower 7 bits of LocoIO version
 SendMsg[9]=  0x00;     // 00
 PXCT1 = 0;
   //PXCT1 = x x x x ...then MSB of .. [9] [8] [7] [6]
    //PXCT2 = x x x x ...then MSB of .. [14] [13] [12] [11]
 bitWrite (PXCT1, 0, bitRead(SendMsg[6],7));
 bitWrite (PXCT1, 1, bitRead(SendMsg[7],7));
 bitWrite (PXCT1, 2, bitRead(SendMsg[8],7));
 bitWrite (PXCT1, 3, bitRead(SendMsg[9],7));
 SendMsg[5]=  PXCT1;     //// PXCT1 high order bits 
 SendMsg[11]=  (ADHI & 0x7F);     //LocoIO Sub-address hi
 SendMsg[12]=  (Data1 & 0x7F);   //requested data
 SendMsg[13]=  (Data2 & 0x7F);  //data +1
 SendMsg[14]=  (Data3 & 0x7F);  // data +2 or written data if "Write"
 PXCT2 = 0;
 bitWrite (PXCT2, 0, bitRead(SendMsg[11],7));
 bitWrite (PXCT2, 1, bitRead(SendMsg[12],7));
 bitWrite (PXCT2, 2, bitRead(SendMsg[13],7));
 bitWrite (PXCT2, 3, bitRead(SendMsg[14],7));
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
Addr=Addr+(128*(SV[2+(3*i)]&0x0F));
Addr=Addr+1; //to match rocrail numbering..
/*#if _SERIAL_SUBS_DEBUG
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
          Serial.println( Addr);
       
     #endif  
*/





return Addr;
  
}




void doPeriodicServo() {
  int ServoPositionNow;
  int Servodemand;
  int Servoattached;
  int offset;
 for (int i=1 ; i<=4; i++) {
     SDelay[i]=SDelay[i]+1;
  if (SDelay[i] >= SloopDelay){   // only do this at intervals...
   SDelay[i]=0;    
switch  (i){  // we sort out which servo to operate here.. case 1= the Base Address
  case 1:
    ServoPositionNow= myservo0.read();
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > SV[100+(3*i)]){
      offset= (offset*SV[100+(3*i)])/abs(offset);
    }
    myservo0.write(ServoPositionNow+offset);
    Servoattached= myservo0.attached();
      if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo0.attached()) { myservo0.detach();  } }
    break;
  case 2:
    ServoPositionNow= myservo1.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > SV[100+(3*i)]){
      offset= (offset*SV[100+(3*i)])/abs(offset);
    }
    myservo1.write(ServoPositionNow+offset);
    Servoattached= myservo1.attached();
     if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo1.attached()) { myservo1.detach();  } }

  break;
  case 3:
    ServoPositionNow= myservo2.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > SV[100+(3*i)]){
      offset= (offset*SV[100+(3*i)])/abs(offset);
    }
    myservo2.write(ServoPositionNow+offset);
    Servoattached= myservo2.attached();
   if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo2.attached()) { myservo2.detach();  } }

  break;
    case 4:
    ServoPositionNow= myservo3.read(); 
    offset= SDemand[i]-ServoPositionNow;
    if (abs(offset) > SV[100+(3*i)]){
      offset= (offset*SV[100+(3*i)])/abs(offset);
    }
    myservo3.write(ServoPositionNow+offset);
    Servoattached= myservo3.attached();
    if (abs(offset)==0){ServoOffDelay[i]++;}
      if (ServoOffDelay[i] > OffDelay) {
                    ServoOffDelay[i]=1000;
            if (myservo3.attached()) { myservo3.detach();  } }

  break;
        }
if (SloopDelay >=90){  // only send debug if updating slowly... 
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
  }
  



