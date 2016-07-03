 // --------------------------------------------------------
    // OPC_PEER_XFER CMD's
    // --------------------------------------------------------
// and see http://www.locobuffer.com/LocoIO/LocoIO.pdf  
// see also http://www.digitrax.com/static/apps/cms/media/documents/loconet/loconetpersonaledition.pdf
// and https://github.com/rocrail/GCA/blob/master/ln/lio/mgv50/src/loconet.h

#define CMD_WRITE            0x01    /* Write 1 byte of data from D1*/
#define CMD_READ             0x02    /* Initiate read 1 byte of data into D1*/
#define CMD_MASKED_WRITE     0x03    /* Write 1 byte of masked data from D1. D2 contains the mask to be used.*/
#define CMD_WRITE4           0x05    /* Write 4 bytes of data from D1..D4*/
#define CMD_READ4            0x06    /* Initiate read 4 bytes of data into D1..D4*/
#define CMD_DISCOVER         0x07    /* Causes all devices to identify themselves by their MANUFACTURER_ID, DEVELOPER_ID, PRODUCT_ID and Serial Number*/
#define CMD_IDENTIFY         0x08    /* Causes an individual device to identify itself by its MANUFACTURER_ID, DEVELOPER_ID, PRODUCT_ID and Serial Number*/
#define CMD_CHANGE_ADDR      0x09    /* Changes the device address to the values specified in <DST_L> + <DST_H> in the device that matches */
                                        /* the values specified in <ADRL> + <ADRH> + <D1>..<D4> that we in the reply to the Discover or Identify command issued previously*/
#define CMD_RECONFIGURE      0x4F    /* Initiates a device reconfiguration or reset so that any new device configuration becomes active*/

    // Replies
#define CMDR_WRITE           0x41    /* Transfers a write response in D1*/
#define CMDR_READ            0x42    /* Transfers a read response in D1*/
#define CMDR_MASKED_WRITE    0x43    /* Transfers a masked write response in D1*/
#define CMDR_WRITE4          0x45    /* Transfers a write response in D1..D4*/
#define CMDR_READ4           0x46    /* Transfers a read response in D1..D4*/
#define CMDR_DISCOVER        0x47    /* Transfers an Discover response containing the MANUFACTURER_ID, DEVELOPER_ID, PRODUCT_ID and Serial Number*/
#define CMDR_IDENTIFY        0x48    /* Transfers an Identify response containing the MANUFACTURER_ID, DEVELOPER_ID, PRODUCT_ID and Serial Number*/
#define CMDR_CHANGE_ADDR     0x49    /* Transfers a Change Address response.*/
#define CMDR_RECONFIGURE     0x4F    /* Acknowledgement immediately prior to a device reconfiguration or reset*/


#define OPC_LOCO_SPD           0xA0
#define OPC_LOCO_DIRF           0xA1   
#define OPC_LOCO_SND           0xA2 

#define OPC_MOVE_SLOTS       0xBA

#define  OPC_LOCO_ADR         0xBF

#define OPC_SW_ACK            0xBD    //;REQ SWITCH WITH acknowledge function (not DT200) YES LACK
                                      // ;<0xBD>,<SW1>,<SW2>,<CHK> REQ SWITCH function
                                      //<SW1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 input pairs in a DS54
                                      //<SW2> =<0,0,DIR,ON- A10,A9,A8,A7> Control bits and 4 MS adr bits.
                                      //,DIR=1 for Closed,/GREEN, =0 for Thrown/RED
                                      //,ON=1 for Output ON, =0 FOR output OFF
                                      // ;response is <0xB4> <3D><00> if DCS100 FIFO is full,command rejected
                                      //<0xB4><3D><7F> if DCS100 accepted
#define OPC_SW_STATE          0xBC          //;REQ state of SWITCH YES LACK
                                            //;<0xBC>,<SW1>,<SW2>,<CHK> REQ state of SWITCH
#define OPC_SW_REQ            0xB0    //;REQ SWITCH function NO
                                      // ;<0xB0>,<SW1>,<SW2>,<CHK> REQ SWITCH function
                                       //<SW1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 input pairs in a DS54
                                       //<SW2> =<0,0,DIR,ON- A10,A9,A8,A7> Control bits and 4 MS adr bits.
                                       //,DIR=1 for Closed,/GREEN, =0 for Thrown/RED
                                       //,ON=1 for Output, =0 FOR output OFF
                                       //Note-,Immediate response of <0xB4><30><00> if command failed, otherwise no response

#define  OPC_PEER_XFER 0xE5            //;move 8 bytes PEER to PEER, SRC->DST NO resp
                                       // ;<0xE5>,<10>,<SRC>,<DSTL><DSTH>,<PXCT1>,<D1>,<D2>,<D3>,<D4>,
                                       // ; <PXCT2>,<D5>,<D6>,<D7>,<D8>,<CHK>
                                       // ;SRC/DST are 7 bit args. DSTL/H=0 is BROADCAST msg
                                       // ; SRC=0 is MASTER
                                       // ; SRC=0x70-0x7E are reserved
                                       //;SRC=7F is THROTTLE msg xfer, <DSTL><DSTH> encode ID#, <0><0> is THROT B'CAST
                                       // ;<PXCT1>=<0,XC2,XC1,XC0 - D4.7,D3.7,D2.7,D1.7>
                                       // ;XC0-XC2=ADR type CODE-0=7 bit Peer TO Peer adrs; 
                                       //                        1=><D1>is SRC HI,<D2>is DST HI
                                        //;<PXCT2>=<0,XC5,XC4,XC3 - D8.7,D7.7,D6.7,D5.7>
                                       // ;XC3-XC5=data type CODE- 0=ANSI TEXT string,balance RESERVED
#define OPC_INPUT_REP  0xB2            // ; General SENSOR Input codes NO
                                       // ; <0xB2>, <IN1>, <IN2>, <CHK>
                                       // <IN1> =<0,A6,A5,A4- A3,A2,A1,A0>, 7 ls adr bits. A1,A0 select 1 of 4 inputs pairs in a DS54
                                       // <IN2> =<0,X,I,L- A10,A9,A8,A7> Report/status bits and 4 MS adr bits.
                                       // "I"=0 for DS54 "aux" inputs and 1 for "switch" inputs mapped to 4K SENSOR space.
                                       // (This is effectively a least significant adr bit when using DS54 input configuration)
                                       // "L"=0 for input SENSOR now 0V (LO) , 1 for Input sensor >=+6V (HI)
                                       // "X"=1, control bit , 0 is RESERVED for future!
#define OPC_RQ_SL_DATA  0xBB            // send   OPC_SL_RD_DATA      

 #define  OPC_WR_SL_DATA  0xEF 


