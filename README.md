# LOCOWIFI
WIFI ESP8266 Arduino code for use with Rcorail

This is experimental code that adds LOCO speed and direction to my Servo/Switch code..

This basic servo/switch software sets up four servo ports starting at address (SV[2](hi), SV[1](lo).
These two  will be  reported as "Sub" SV[2] and  and "Low" SV[1] in the Rocrail LocoIo General tab, 
They can be modified from there.
Together they produce an "address" that Rocrail can send servo messages and SV set/red commands 
With SV[2]=(1), SV[1]=(2), the first address of the Servos is 130 (Decimal) (128+2).
This controls the Servos on Nudemcu ports:
Port 1 is D0 and then address 130,
Port 2 is D1 and address; port 3 is 132 and controls D2; port 4 is 133 and controls D3.
The Servo "speed" is set by "V" and this and the servo end positions can be changed using the Rocrail LocoIO "servo" tab. 
Speed "5" is fastest, 1 is slowest.
Servos turn off after reaching their positions (plus a short additional delay) to save power. Note: The red led will turn on because D0 is used on a V2 module..) 

The module also has inputs that can send a B2 Message to rocrail. (for sensors).
The Messages these send to are controlled by the SV,s in (port1 )  SV's3,4 and 5  (port2)  SV's6,7,8 .....etc..
Using the Rocrail LocoIo programming tab for I/O, these can have their individual "address" set, 
to determine which address is sent whn the corresponding input port changes.
Port 1 uses D5, Port2 uses D6 Port 3 uses D7 port 4 used D9. All include debounce so a small delay is present before the mesage is sent.
The message is actually sent twice, just in case one of the packets is lost..

D4 is connected to the ESP8266 blue led, which is used to indicate message reception on the board or switch change detection.
It should quickly flash when a message is sent or received.

Using the following ports can or will cause issues: 
     D0 is the red led, but we can still use D0 for a servo .. just do not try to use it for Red Led control!... 
     D8 is used for serial data ? 
