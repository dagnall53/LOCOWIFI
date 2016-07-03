uint8_t SV[256];
uint8_t CV[256];
 #define DEFAULT_BOARD_ADDR_LO          89;
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
 for (int i=1; i <= 4; i++){ // set servo defaults here for now
    SV[98+(3*i)] = 0;   //(thrown)
    SV[99+(3*i)] = 90;  //Straight)  (90 will become 180 degrees)
    SV[100+(3*i)]= 0x01; }
//SENSOR DEFAULTS
for (int i=1; i<=10; i++) {
  SV[0+(3*i)] = 0x00;
  SV[1+(3*i)] = 10+i ;
  SV[2+(3*i)]= 0x00;}

//cv'S ARE IN 256 ONWARDS.

CV[18] = 0x0E;
CV[17] = 0x01;
CV[1]= 0x0E; 
CV[29]= 0x20;  // N0rmal and extended addressing..  0x21 for reversed

CV[8]= 0x0D; // DIY code
CV[7] = 0x01;


  
} //SetDefaultSVs


void ShowSVS(){
  #if _SERIAL_SUBS_DEBUG
         Serial.print(F("Add hi :"));
         Serial.print(SV[2]);
         Serial.print(" Add lo :");
         Serial.print(SV[1]);
         Serial.println("   ");

         for (int i=0; i<=30; i++) {
         Serial.print(F("CV ["));
         Serial.print(i);
         Serial.print("] =");
         Serial.print(CV[i]);
         Serial.println("   "); 
         }
  #endif         

for (int i=0; i<=4; i++) {
#if _SERIAL_SUBS_DEBUG
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
#endif  


}
}
