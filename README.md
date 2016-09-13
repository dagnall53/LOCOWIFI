# LOCOWIFI
WIFI ESP8266 Arduino code for use with Rcorail

This is experimental code that controls Servos and /Switches and reads RFID tags  over WIFI using Loconet UDP.

With the stationary version (set #define_Static) multiple ESP8266's can be used as stationary nodes and sense trains, or be used to swich signals or servos to control points.

A mobile version (set #define _Loco) of the code accepts standard Loconet commands and acts as a single command station and controls the loco motor through Servo output connected to D8. 

This is the generic code, and has been mainly tested as a mobile device
                CODE IS IN DEVELOPMENT......BE careful.. 

The Standard LOCOIO programming interface in Rocrail then sets the addresses and modes The Module address is based on the Wifi IP address, so all nodes will report a different address. 
The basic servo/switch software sets up ports as either servos, Inuts or outputs. 
Servos are set up as config "output and output pulse" (SV3*port number &0x88 == 0x88)
Digital output is Output and not pulse ((SV3*port number &0x88 == 0x80) 
and inputs are "Input" (SV3*port number &0x88 == 0x00)
 Servo "speed" is set by "V" and this and the servo end positions can be changed using the Rocrail LocoIO "servo" tab. 
 Speed "5" is fastest, 1 is slowest.
 Servos turn off after reaching their positions (plus a short additional delay) to save power. Note: The red led will turn on because D0 is used on a V2 module..) 
When a change is sensed on an input it will send a B2 Message to rocrail. (for sensors). The Addressit sends will be the one set in locoio interface.

D4 is connected to the ESP8266 blue led, which is used to indicate message reception on the board or switch change detection.
It should quickly flash when a message is sent or received.

Using the following ports can or will cause issues: 
        D0 is the red led, but we can still use D0 for a servo .. just do not try to use it for Red Led control!... 
        D1 can be selected to give timing pulses for debug and test purposes. 
        D4 is the blue LED .. this is pulsed to indication message reception and is also used for the RFID interface (see later) 



When #define _LOCO 1 (line 1 of the ino) is set, the code sets itsself up as a MOBILE device, 
and expects a RFID reader RC522 to be fitted. The RFID reader  uses D7,D6 and D5 for MOSI, MISO and SCK. 
IT ALSO uses D0 for RST and D4 for SS pin. This latter is also the blue led on the esp8266.
A front light (D3) and Back light (D2) are also defined and will switch on with f0 and changes of direction.
In loco mode this leaves only D1 "free". If not being used for code timing this pin works as per the Static version, being switchable as an output, or as in input, both with addresses set via the Locoio interface.

The Loco control uses CV's 5,2 6 and 9 unconventionally in this version. Cv2 is the start servo setting(ie motor just running)(typically 95 degrees)  forwards, and CV6 is the equivalent for backwards(typically 85 degrees) . CV 5 is the highest servo setting (typically 170 degrees) with CV 9 being the fastest backwards speed (Setting typically 10 degrees). Motor acceleration is set by CV[3]. Higher is faster acceleration...



Notes: The MRFC522 code I used had a bug in the set clock speed, and defaulted to circa 2MHz. I modified mine to 8Mz, and with this the cyce time with the RFID reader was about 15ms. The code is being updated in the Github , so you may wish to check this. 
Without the RFID reader ceheck, the cycle time is sub 1ms, and is dominated by the Delay(2) in the main loop. 
CV[29] motor reverse bit is functional.

To get the fastest response , when a RFIDtag is read, the RFID reader is switched off for a variable RFIDQuietAfterTag, this is set to circa 3 seconds. This allows the code to cycle as fast as possible to get any commands from Rocrail after the tag is seen. 
When a tag has not been seen for uiDelayTime (0.5s) the code should send a Tag clear message. 
Tags send an "address" that is based on their unique ID, but folded into a range 100-227. This allows plenty of addresses, but the code can be changed in RFID_Subs, line 71 if you need more locations on your track.  (I wanted to have the option for two tags out of a batch to have the same address for some test purposes. )



TO DO
The code still sometimes misses A0-A2 commands. Since these are only sent once, this is a critical failure if running automatic. Pressing a f button DOES re-send all three messages, so a Rocrail automat that perhaps blinks f5 at regular intervals might be an answer.  Ideally I would get Rocrail to send these commands repeatedly, as one would expect in a UDP system. 


