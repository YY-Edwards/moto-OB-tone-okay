/*///////
Copyright (c) 2009 M.H.Retzer.  All rights reserved.

Developed by: M.H.Retzer
              Motorola, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
with the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:
  1. Redistributions of source code must retain the above copyright notice,
     this list of conditions and the following disclaimers.
  2. Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimers in the
     documentation and/or other materials provided with the distribution.
  3. Neither the names of M.H.Retzer, Motorola, Inc.
     nor the names of its contributors may be used to endorse
     or promote products derived from this Software without specific prior
     written permission.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE CONTRIBUTORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS WITH THE SOFTWARE.
/////*/





#ifndef XNL_H_
#define XNL_H_

#include "compiler.h"

//______________________________________________________________________
//XNL State Machine.

typedef enum {
  XNL_UNCONNECTEDWAITINGSTATUS,
  XNL_UNCONNECTEDWAITINGAUTHKEY,
  XNL_UNCONNECTEDWAITINGDEVICECONN,
  XNL_CONNECTED
} XNL_States;

typedef struct {
	U16        XNL_MasterAddress;
	U16        XNL_DeviceAddress;
	U16        XNL_DeviceLogicalAddress;
	U16        XNL_TransactionIDBase;
	U16        XNL_3BitRollover;
	XNL_States XNL_State;
	Bool       isIncomingMessage;
}XNL_Ctrlr;



//_________________________________________________________________
//XNL/XCMP Definitions.


#define XNL_MASTER_STATUS_BRDCST     0x0002
#define XNL_DEVICE_MASTER_QUERY      0x0003
#define XNL_DEVICE_AUTH_KEY_REQUEST  0x0004
#define XNL_DEVICE_AUTH_KEY_REPLY    0x0005
#define XNL_DEVICE_CONN_REQUEST      0x0006
#define XNL_DEVICE_CONN_REPLY        0x0007
#define XNL_DEVICE_SYSMAP_REQUEST    0x0008
#define XNL_DEVICE_SYSMAP_BRDCST     0x0009
#define XNL_DATA_MSG                 0x000B
#define XNL_DATA_MSG_ACK             0x000C


#define XNL_PROTO_XNL_CTRL           0x0000
#define XNL_PROTO_XCMP               0x0100



#define DEVICE_GENERIC_BLOCK_COUNT 1
#define DEVICE_GENERIC_U16_COUNT 8
#define DEVICE_GENERIC_MACBYTE_COUNT 0xE
static const U16 DEVICE_GENERIC_PROTO[DEVICE_GENERIC_U16_COUNT] =
{
		0x400E,   //XCMPXNL_DATA | 0x0E Bytecount.
		0x0000,   //Cannot precalculate checksum.
		0x0000,   //Must be filled in.
		0x0000,   //XNL control message | Unused XNL Flags.
		0x0000,   //Must substitute Master address.
		0x0000,   //Unasigned device address.
		0x0000,   //No transaction ID required for this message.
		0x0000    //This message contains no payload.
};

#define DEVICE_MASTER_STATUS_BRDCST_MACBYTE_COUNT 21 //extended length
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.1
U16   MinorXNLVersionNumber;
U16   MajorXNLVersionNumber;
U16   MasterLogicalIdentifier;   //Upper byte is device type. Lower byte is device number.
U8    DataMessageSent;
U8    u8;                        //Pad.
U16   u16[3];
} contentMASTER_STATUS_BRDCST;



//XCMP/XNL Development Guide  5.4.2.
#define DEVICE_MASTER_QUERY_BLOCK_COUNT 1
#define DEVICE_MASTER_QUERY_U16_COUNT 8
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.2
U16   u16[7];                    //This message contains no payload.
} contentDEVICE_MASTER_QUERY;





//XCMP/XNL Development Guide  5.4.3.
#define DEVICE_AUTH_KEY_REQUEST_BLOCK_COUNT 1
#define DEVICE_AUTH_KEY_REQUEST_U16_COUNT 8
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.3
U16   u16[7];                    //This message contains no payload.
} contentDEVICE_AUTH_KEY_REQUEST;





#define DEVICE_AUTH_KEY_REPLY_MACBYTE_COUNT 24 //extended length
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.4
U16   TemporaryXNLAddress;
U8    UnencryptedAuthenticationValue[8];
U16   u16[2];
} contentDEVICE_AUTH_KEY_REPLY;



//XCMP/XNL Development Guide 5.4.5
#define DEVICE_CONN_REQUEST_BLOCK_COUNT 1
#define DEVICE_CONN_REQUEST_U16_COUNT 14
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.5
U16   PreferredXNLAddress;
U8    DeviceType;
U8    AuthenticationIndex;
U8    EncryptedAuthenticationValue[8];
U16   u16;
} contentDEVICE_CONN_REQUEST;

static const U16 DEVICE_CONN_REQUEST_PROTO[DEVICE_CONN_REQUEST_U16_COUNT] =
{
		0x401A,   //XCMPXNL_DATA | 0x1A Bytecount.
		0x0000,   //Cannot precalculate checksum.

		XNL_DEVICE_CONN_REQUEST,   //Opcode.
		0x0000,   //XNL control message | Unused XNL Flags.

		0x0000,   //Must substitute Master address.
		0x0000,   //Must substitute Temporary XNL address.

		0x0000,   //No transaction ID required for this message.
		0x000C,   //This message contains 12 payload bytes.

		0x0000,   //No Preferred XNL Address.
		0x0702,   //XCMP/XNL Development Specification Section 4.5.3.2.1.
		          //Same as in MOTOTRBO?XCMP/XNL Development Specification?
		          //Array aligned at u32[5]
		0x0000,   //Must substitute value encrypted.
		0x0000,   //Must substitute value encrypted.
		0x0000,   //Must substitute value encrypted.
		0x0000    //Must substitute value encrypted.
};


#define DEVICE_CONN_REPLY_MACBYTE_COUNT 28 //Extended count
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.6
U16   Result_Base;
U16   XNLAddress;
U16   LogicalAddress;           //Upper byte is device type. Lower byte is device number.
U8    EncryptedAuthenticationValue[8];
} contentDEVICE_CONN_REPLY;


typedef struct {                 //XCMP/XNL Development Guide Section 5.4.7
U16   u16[7];                    //This message contains no payload.
} contentDEVICE_SYSMAP_REQUEST;


typedef struct {                 //XCMP/XNL Development Guide Section 5.4.8
U16   SizeofSysMaparray;
U8    u8[10];                    //Packed bytes.
U16   u16;
} contentDEVICE_SYSMAP_BRDCST;

#define XCMP_MTMask      0xF000
#define XCMP_requestMT   0x0000
#define XCMP_DEVINITSTS  0xB400   //Device Initialization Status
#define DIC              0x01
#define XCMP_DEVMGMTBCST 0xB428   //Device Management Broadcast
#define XCMP_TONECTRLREQ 0x0409   //Tone Control Request
#define XCMP_TONECTRLREP 0x8409   //Tone Control Reply
#define XCMP_TONECTRLBRDCST 0xB409 //Tone Control Broadcast


typedef struct {                 //XCMP/XNL Development Guide Section 5.4.9
U16  XCNPopcode;
U8   u8[12];                     //Packed Bytes.
} contentXNL_DATA_MSG;


//XCMP/XNL Development Guide 5.4.10
#define XNL_DATA_MSG_ACK_BLOCK_COUNT 1
#define XNL_DATA_MSG_ACK_U16_COUNT 8
typedef struct {                 //XCMP/XNL Development Guide Section 5.4.10
U16   u16[7];                    //This message contains no payload.
} contentXNL_DATA_MSG_ACK;


typedef union {
  contentMASTER_STATUS_BRDCST     ContentMASTER_STATUS_BRDCST;
  contentDEVICE_MASTER_QUERY      ContentDEVICE_MASTER_QUERY;
  contentDEVICE_AUTH_KEY_REQUEST  ContentDEVICE_AUTH_KEY_REQUEST;
  contentDEVICE_AUTH_KEY_REPLY    ContentDEVICE_AUTH_KEY_REPLY;
  contentDEVICE_CONN_REQUEST      ContentDEVICE_CONN_REQUEST;
  contentDEVICE_CONN_REPLY        ContentDEVICE_CONN_REPLY;
  contentDEVICE_SYSMAP_REQUEST    ContentDEVICE_SYSMAP_REQUEST;
  contentDEVICE_SYSMAP_BRDCST     ContentDEVICE_SYSMAP_BRDCST;
  contentXNL_DATA_MSG             ContentXNL_DATA_MSG;
  contentXNL_DATA_MSG_ACK         ContentXNL_DATA_MSG_ACK;
} XNLpayload;

#define TONECTRLREQ_BLOCK_COUNT 1
#define TONECTRLREQ_BYTE_COUNT 8
static const U8 TONECTRLREQPROTO[TONECTRLREQ_BYTE_COUNT] =
{
		//This test uses 0x000C [PRIORITY_BEEP]
		0x01, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00
};

#define DEVINITSTS_BLOCK_COUNT 1
#define DEVINITSTS_BYTE_COUNT 9
static const U8 DEVINITSTSPROTO[DEVINITSTS_BYTE_COUNT] =
{
		0x07, 0x00, 0x00, 0x05, 0x00, 0x07, 0x00, 0x00, 0x00
};




#define XCMPRESULT_SUCCESS 0x00
#define XCMPRESULT_FAILURE 0x01
#define XCMPRESULT_INCORRECTMODE 0x02
#define XCMPRESULT_NOTSUPPORTED 0x03
#define XCMPRESULT_INVALPARAM 0x04
#define XCMPRESULT_TOOBIG 0x05
#define XCMPRESULT_SECLOCK 0x06





//________________________________________________
//Structures defining the SSC physical frame.
typedef union {
  U32     word;
  U16 hword[2];
  U8   byte[4];
} Reserved_Channel;

typedef union {
  U32     word;
  U16 hword[2];
  U8   byte[4];
} XNL_Channel;

typedef union {
  U32  word[2];
  U16 hword[4];
  U8   byte[8];
} Payload_Channel;

typedef struct {
  Reserved_Channel theReserved_Channel;
  XNL_Channel           theXNL_Channel;
  Payload_Channel   thePayload_Channel;
 } SSC_Frame;

#define PHYHEADER32      (U32)0xABCD0000
#define PHYTERMRIGHT     (U32)0x000000BA
#define PHYTERMLEFT      (U32)0x00BA0000
#define XNL_IDLE         (U32)0xABCD5A5A
#define PAYLOADIDLE0     (U32)0xABCD5A5A
#define PAYLOADIDLE1     (U32)0x00000000


//_____________________________________________________________________________
//XNL/SCC Rx Media Controller.

#define RXMEDIABUFFERSIZE 5120
  typedef enum {
    WAITINGABAB,
    READINGARRAYDISCRPT,
    READINGMEDIA,
    BGFORCERESET
  } RxMediaStates;  //enums are 32 bits.
typedef struct{
	RxMediaStates RxMediaState;
	S32           RxMedia_IsFillingNext16;
	S32           RxBytesWaiting;
	S32           ArrayDiscLength;
}RxMediaCtrlr;




//_____________________________________________________________________________
//XNL/SSC Rx Circular Buffer Link Controller.

#define RXCIRBUFFERMAXFRAGS      16
#define RXCIRBUFFERMAX16         2048
#define RXCIRBUFFERFRAGWRAP      0x000F

 typedef enum {
   WAITINGFORHEADER,
   READINGFRAGMENT,
   WAITINGCSUM,
   WAITINGLASTTERM
 } RxLink_State;



typedef struct {
  U16            phy_control;
  U16            checksumspacesaver;
} MAC_Header;

//Option Board ADK Development Guide [9.1.2.4].
//The 0xABCD Header is a constant, so need not be stored.
typedef struct {   //XCMP/XNL Development Guide Section 5.1
  U16            opcode;
  U16            flags;
  U16            destination;
  U16            source;
  U16            transactionID;
  S16            payloadLength;
}XNL_Header;

//Fixed size 256 byte Rx fragment.
typedef struct {
  MAC_Header     theMAC_Header;  // 2  hWords
  XNL_Header     theXNL_Header;  // 6  hWords
  XNLpayload     theXNL_Payload; // 7  hWords (most common messages)
  U16            theRest[113];
} RxTemplate;

typedef union {
  RxTemplate     theRxTemplate;
  U16            RxFragmentElement16[128];
} RxFragment;

typedef union {
  RxFragment     theRxFragment[RXCIRBUFFERMAXFRAGS];
  U16            CirBufferElement16[RXCIRBUFFERMAX16];
} RxCirBuffer;

// "RxCirCtrlr" is the state machine controller for the Rx circular buffer.
typedef struct{
  U32            RxLinkCount;      //Frames received since powerup.
  RxTemplate    *pRxTemplate;
  RxLink_State   theRxLink_State;
  S16            RxLink_Expected;  //Bytes Expected.
  U16            RxLink_CSUM;      //Scratch area frag CSUM.
  S16            RxXNL_IsFillingMessageIndex;
  S16            RxXNL_IsFillingNextU16;
  S16            RxXNL_ProcessWaitingIndex;
} RxCirCtrlr;

//________________________________________________________________________
//XNL/SSC Tx buffers.

#define TXPOOLSIZE        16     //16 phy_block's of 256 Bytes = 4096 Bytes
#define TXBLOCKBOUND      TXPOOLSIZE


typedef struct {
	  MAC_Header     theMAC_Header;                         //  2 hWords
	  XNL_Header     theXNL_Header;                         //  6
	  XNLpayload     theXNLpayload;                         //  7
	  U16            overflow[113];                         //113
}MACXNL_Template;

typedef struct {
	  MAC_Header     theMAC_Header;                         //  2 hWords
	  U16            overflow[126];                         //126
}MACRAW_Template;

typedef union {
	MACXNL_Template   XNL;
	MACRAW_Template   MAC;
	U32           u32[64];
	U16          u16[128];
	U8            u8[256];
}phy_block;






//________________________________________________________________________
//XNL Tx Controller.

#define STANDARDTIMEOUT             4000;     //500mS*8KHz.
#define TXINSTANCESIMPLEMENTED      4
#define TXINSTANCESBOUND            TXINSTANCESIMPLEMENTED
#define NULLINSTANCEBEHAVIOR        0x00000000   //Zero Behavior | Zero Retries.
#define ALLOCATEDINSTANCEBEHAVIOR   0x00010005   //Allocates     | 5 Retries.

//"behavior" word can only be written by BG if FGOWNSBEHAVIOR flag is clear.
//BG must verify that this flag is clear before writing anything to this word.
//In particular, BG cannot deplete this instance if FGOWNSBEHAVIOR is set.
//To schedule an instance for transmission, BG must verify that a transmission
//may be scheduled, NextInstance == TXINSTANCEBOUNT. Then set FGOWNSBEHAVIOR to 1,
//then put instance index into NextInstance.
#define FGHANDSHAKEMASK         0xE0000000
#define FGOWNSBEHAVIOR          0x80000000
#define FGHASSENT               0x40000000
#define CANDEPLETEAFTERSENT     0x20000000
#define OKTOGARBAGECOLLECT      0x60000000 //Un-owned, sent, can deplete.
#define TXINSTANCESTATEMASK     0x0FFF0000
#define TXINSTANCERETRYMASK     0x0000FFFF
#define TXINSTANCESTATE         0x00010000 //Others may be needed

                                           //The retry test decrements count,
                                           //and if equal to Zero, stops retrying.
#define TXXNLCTRLPROTO          0x00010006
#define OWNEDXNLCTRLPRPTO       0x80010005
#define TXXCMPACKPROTO          0x20010002
#define OWNEDXCMPACKPROTO       0xA0010001
#define TXXCMPMSGPROTO          0x00010006
#define OWNEDXCNPMSGPROTO       0x80010005




//Please note that the maximum transfer unit (MTU) size for a Data Session
//is 1500 bytes. Of the MTU size, 28 bytes are reserved for overhead.
// Therefore, the maximum payload is 1472 bytes. Therefore, requires 6 fragments.
#define MAXPHYBLOCKS      6   //Maximum number of fragments possible in a single Instance.
typedef struct {
	U32           behavior;
	U32           RetryTime;
	S32           BlockIndex[MAXPHYBLOCKS];
} TxInstance;


typedef enum {
  IDLEWAITINGSCHEDULE,
  INSTANCETRANSMIT,
  CONTINUINGNEWFRAGMENT,
  SENDINGTERMINATOR32
} TXLINKSTATES;  //enums are 32 bits.

typedef struct {
	S32 AvailableInstanceCount;      //Used to speed up BG.
	S32 AvailableBlockCount;         //Used to speed up BG.
	U32 TxLinkState;                 //Tx Phy machine State.
	S32 BytesRemaining;              //Phy Bytes remaining in this fragment.
	S32 CurrentInstanceIndex;        //CurrentInstanceIndex, CurrentBlockIndex,
	S32 CurrentBlockIndex;           //and Next16TxIndex are writable only by FG.
	S32 Next16TxIndex;               //CurrentInstanceIndex may be read by BG.
	S32 NextWaitingIndex;            //Upon NextWaitingIndex Tx start by FG,
} TxXNL_schedule;                   //NextWaitingIndex is nulled to TXINSTANCESBOUND.
                                    //This indicates a new instance may be scheduled.
                                    //The BG may then write to NextWaitingIndex.
                                    //[Except for initialization], the BG may only
                                    //Write to NextWaitingIndex when it is == TXINSTANCESBOUND.
                                    //The FG must never write to NextWaitingIndex
                                    //while it == TXINSTANCESBOUND. This is the Tx FG/BG
                                    //semiphore.


void Delay(unsigned long  Delay);									
void Delay_ms(unsigned long  Delay_ms);

/****************************************************************************


//typedef struct{
//	  U32            RxLinkCount;      //Frames received since powerup.
//	  RxTemplate    *pRxTemplate;      //Pointer to unrappable extended RxTemplate within Circular Buffer.
//	  RxLink_State   theRxLink_State;
//	  S16            RxLink_Expected;  //Bytes Expected.
//      U16            RxLink_CSUM;      //Scratch area used by FG to verify frag CSUM.
//      S16            RxXNL_IsFillingMessageIndex;
//      S16            RxXNL_IsFillingNextU16;
//  	  S16            RxXNL_ProcessWaitingIndex;
//  } RxCirCtrlr;




typedef struct {
	S16 extended_length;
	U16 checksumspacesaver;
}extended_control;

//typedef struct {
//	U16                 phy_control;
//	U16                 csum;
//} MAC_Header;


//Option Board ADK Development Guide [9.1.2.4]. 260 comprised of:
//      2 Useful/Terminator [Usually constant.]
//    252 Bytes Payload
//      2 Bytes Checksum
//      2 Type/Length
//      2 Header            [This is constant.]
//typedef struct {   //XCMP/XNL Development Guide Section 5.1
//	  U16            opcode;
//	  U16            flags;
//	  U16            destination;
//	  U16            source;
//	  U16            transactionID;
//	  S16            payloadLength;
//}XNL_Header;

typedef struct {
	extended_control theExtendedControl;  // 2  hWords
	XNL_Header       theXNL_Header;       // 6  hWords
	XNLpayload       theXNL_Payload;      // 7  hWords (most common messages)
	U16              overflow;            // 1  hWord  {last hWord guaranteed not to wrap)
} RxTemplate;




#define RXCIRBUFFERSIZE      2048     //2048 hWord = 4096 Bytes
#define RXCIRBUFFERKEEPOUT     15     //Won't start a new reception within last 15 hWords.
#define RXCIRBUFFERMAXSTART  2032     //Maximum possible new message start index.
#define RXCIRBUFFERWRAP      0x07FF

*******************************************************************/

#endif /*XNL_H_*/
