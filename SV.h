uint8_t SV[256];
uint8_t CV[256];
 #define DEFAULT_BOARD_ADDR_LO          90;
 #define DEFAULT_BOARD_ADDR_HI         1;

 #define DEFAULT_SENSOR_ADDR_HI         0; 
 #define DEFAULT_SENSOR_ADDR_LO         2;

#define _SERIAL_SUBS_DEBUG 1



void SetDefaultSVs(){

  SV[1]=  DEFAULT_BOARD_ADDR_LO;  //ucBoardAddrLo     ;
  SV[2]=   DEFAULT_BOARD_ADDR_HI; //ucBoardAddrHi   ;
  SV[4]= DEFAULT_SENSOR_ADDR_LO;    //sensor address low  ucAddrLoSen;
  SV[5]= DEFAULT_SENSOR_ADDR_HI;  //                  high ucAddrHiSen;
    
//servo settings, Just lo, hi and velocity (not used)
 for (int i=1; i <= 8; i++){ // set servo defaults here for now
    SV[98+(3*i)] = 0;   //(thrown)
    SV[99+(3*i)] = 90;  //Straight)  (90 will become 180 degrees)
    SV[100+(3*i)]= 0x01; }
//SENSOR DEFAULTS
for (int i=1; i<=16; i++) {
  SV[0+(3*i)] = 0x07;  //set all to inputs // "switch report" 
  SV[1+(3*i)] = 10+i ;
  SV[2+(3*i)]= 0x00;}

//cv'S ARE IN eeprom 256 ONWARDS.

    #if _LOCO
  SV[3*8]=0x88;    // servo on 8

    #endif 

CV[18] = 0x03;   //default = 3
CV[17] = 192;
CV[1]= 0x03; 
CV[3]= 0x03;   // default acc/decc
CV[29]= 0x32;  // N0rmal  addressing..  

CV[5]=170;   // default servo settings for + 100
CV[2]=95;    // default servo settings for + lowest speed
CV[6]=85;   // default servo settings for - 100
CV[9]=10;   // default servo settings for -ve lowest speed




CV[8]= 0x0D; // DIY MFR code
CV[7] = 0x02; //ver 
 #if _SERIAL_SUBS_DEBUG
         Serial.print(F("********* Default settings reinstated ***********"));
         Serial.println("   "); 
        
  #endif    

  
} //SetDefaultSVs


void ShowSVS(){
  #if _SERIAL_SUBS_DEBUG
         Serial.print(F("********BOARD Add hi :"));
         Serial.print(SV[2]);
         Serial.print(" Add lo :");
         Serial.print(SV[1]);
         Serial.println(" ********  ");

      
        
Serial.println("   ----------LOW SV's -------------  "); 
 Serial.print(F("SV [0]"));
 Serial.print(SV[0]);
 Serial.print("d  ");
  Serial.print(F("SV [1]"));
 Serial.print(SV[1]);
 Serial.print("d  ");
  Serial.print(F("SV [2]"));
 Serial.print(SV[2]);
 Serial.print("d  ");  
  Serial.print(F("SV [3]"));
 Serial.print(SV[3]);
 Serial.println("d  ");  
     
         Serial.println("  ------Switch settings and addresses----------- "); 
 for (int j=1; j<=16; j++) {
          for (int i=1;i<=3;i++){
         Serial.print(F("SV ["));
         Serial.print(i+(j*3));
         Serial.print("] =");
         Serial.print(SV[i+(j*3)]);
         Serial.print("d  ");
          }
         Serial.println("   "); 
         }
Serial.println("   ---------SERVO settings --------------  "); 
for (int i=1; i<=8; i++) {

         Serial.print(F("SV :"));
         Serial.print(98+(3*i));
         Serial.print("  is :");
         Serial.print(SV[98+(3*i)]);
         Serial.print("   ");
         Serial.print(F("SV :"));
         Serial.print(99+(3*i));
         Serial.print("  is :");
         Serial.print(SV[99+(3*i)]);
         Serial.print("   ");
         Serial.print(F("SV :"));
         Serial.print(100+(3*i));
         Serial.print("  is :");
         Serial.print(SV[100+(3*i)]);
         Serial.println("   ");
}
Serial.println("   ---------CV settings --------------  "); 
   for (int j=0; j<=7; j++) {
          for (int i=0;i<=4;i++){
         Serial.print(F("CV ["));
         Serial.print(i+(j*5));
         Serial.print("] =");
         Serial.print(CV[i+(j*5)]);
         Serial.print("d  ");
          }
         Serial.println("   "); 
         }
#endif  
Serial.println("   -----------------------  "); 


}
