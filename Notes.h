/*  REFERENCE data.. Do not INCLUDE in the main file....

This software sets up four servo ports starting at address (SV[2](hi), SV[1](lo).
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


     

 /* References... Also defined somewhere else in the esp included code so do not unhide this section!!!...


static const uint8_t D0   = 16;  and Red Led on NodeMcu V2 (not present on NodeMCU v3)
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;  and Blue Led on SP8266
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;

*/ 
  


/**
  * Programming SV’s
  *
  *     The SV’s in LocoIO can be programmed using Loconet OPC_PEER_XFER messages.
  *     Commands for setting SV’s
  *
  *     PC to LocoIO loconet message (OPC_PEER_XFER)
  *
  *     Code  LOCOIO_SV_READ      LOCOIO_SV_WRITE
  *     ----  --------------      ---------------
  *     0xE5  OPC_PEER_XFER       OPC_PEER_XFER
  *     0x10  Message length      Message length
  *     SRCL  0x50                0x50                // low address byte of Locobuffer
  *     DSTL  LocoIO low address  LocoIO low address
  *     DSTH  0x01                0x01                // Fixed LocoIO high address
  *     PXCT1
  *     D1    LOCOIO_SV_READ      LOCOIO_SV_WRITE     // Read/Write command
  *     D2    SV number           SV number
  *     D3    0x00                0x00
  *     D4    0x00                Data to Write
  *     PXCT2
  *     D5    LocoIO Sub-address  LocoIO Sub-address
  *     D6    0x00                0x00
  *     D7    0x00                0x00
  *     D8    0x00                0x00
  *     CHK   Checksum            Checksum
  *
  *
  *     LocoIO to PC reply message (OPC_PEER_XFER)
  *
  *     Code  LOCOIO_SV_READ      LOCOIO_SV_WRITE
  *     ----  --------------      ---------------
  *     0xE5  OPC_PEER_XFER       OPC_PEER_XFER
  *     0x10  Message length      Message length
  *     SRCL  LocoIO low address  LocoIO low address
  *     DSTL  0x50                0x50                // low address byte of Locobuffer
  *     DSTH  0x01                0x01                // high address byte of Locobuffer
  *     PXCT1 MSB LocoIO version  MSB LocoIO version  // High order bit of LocoIO version
  *     D1    LOCOIO_SV_READ      LOCOIO_SV_WRITE     // Original Command
  *     D2    SV number requested SV number requested
  *     D3    LSBs LocoIO version LSBs LocoIO version // Lower 7 bits of LocoIO version
  *     D4    0x00                0x00
  *     PXCT2 MSB Requested Data  MSB Requested Data  // High order bit of requested data
  *     D5    LocoIO Sub-address  LocoIO Sub-address
  *     D6    Requested Data      0x00
  *     D7    Requested Data + 1  0x00
  *     D8    Requested Data + 2  Written Data
  *     CHK   Checksum            Checksum
  *
  *
**/

