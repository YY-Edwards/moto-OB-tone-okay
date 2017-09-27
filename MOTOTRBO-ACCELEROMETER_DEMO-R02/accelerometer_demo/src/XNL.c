#include "GOB.h"
#include "XNL.h"
#include "my_INTC.h"
#include "accelerometer.h"
#include "key.h"

  volatile U32 bunchofrandomstatusflags;
  // 0x00000001 XCMP Initialization Complete.         0xFFFFFFFC
  // 0x00000002 XCMP Option Board (Channel) Enabled.  0xFFFFFFFD
  // 0x00000020 Accelerometer Double Tap detected.    0xFFFFFFDF
  // 0x00000040 Accelerometer tilt detected.          0xFFFFFFBF

  volatile U32 intStartCount;
  volatile U32 intDuration;
  volatile SSC_Frame RxBuffer[2];
  volatile SSC_Frame TxBuffer[2];
  volatile U8 BufferIndex;

  volatile U8 devflag;

  volatile Bool DontPanic;


  RxMediaCtrlr theRxMediaCtrlr;
  XNL_Ctrlr theXNL_Ctrlr;
  RxCirCtrlr theRxCirCtrlr;
  TxXNL_schedule txSchedule;
  TxInstance TxInstancePool[TXINSTANCESIMPLEMENTED];
  U8  TxBlockReservation[TXPOOLSIZE];
  U16 RxMediaBuffer[RXMEDIABUFFERSIZE];
  RxCirBuffer theRxCirBuffer;
  phy_block TxBufferPool[TXPOOLSIZE];

  void reset_IsFillingNextU16(void)
  {
	  theRxCirCtrlr.RxXNL_IsFillingNextU16 =
		  theRxCirCtrlr.RxXNL_IsFillingMessageIndex << 7;
  }

  void post_message(void)
  {
	  S16 nextpossiblebufferindex;

	  nextpossiblebufferindex = ((theRxCirCtrlr.RxXNL_IsFillingMessageIndex) + 1) & RXCIRBUFFERFRAGWRAP;
	  if ( nextpossiblebufferindex
		!= theRxCirCtrlr.RxXNL_ProcessWaitingIndex ){ //Test for collision, discard.
		  theRxCirCtrlr.RxXNL_IsFillingMessageIndex =
			  nextpossiblebufferindex;
	  }
  }

  void XNL_PhyRx(U32 theXNLRxWord)
  {
		//This is the code for parsing the incoming physical message.
	    switch (theRxCirCtrlr.theRxLink_State){

	    // Note that all segments must align with a 32-bit boundary and beginning of
	    // each XCMP/XNL payload frame must start on slot 3 Thus, segments of odd length
	    // must append a 0x0000 at the end (slot 4) to ensure alignment. [9.1.3]
	    case WAITINGFORHEADER:                  //Waiting for something. Most frequent visit.
	    	if (0xABCD5A5A == theXNLRxWord)   break;                   //Ignore Idles.
	    	if (0xABCD0000 != (theXNLRxWord  & 0xFFFF0000)) break;   //Skip until Header.
	    	if (0x00004000 != (theXNLRxWord  & 0x0000F000)) break;   //Skip non-XCMPXNL_DATA.
                
    //           AVR32_GPIO.port[1].ovrc  =  0x00000008;
			
			theRxCirCtrlr.RxLink_Expected = (theXNLRxWord & 0x000000FF) - 2; //Length excluding CSUM.
	    	if (theRxCirCtrlr.RxLink_Expected <= 0) break;           //Discard degenerate message.
	    	//Do not need to check for buffer wrap here. Should point to clean buffer.
	    	theRxCirBuffer.CirBufferElement16[(theRxCirCtrlr.RxXNL_IsFillingNextU16)++] = theXNLRxWord;
	    	theRxCirBuffer.CirBufferElement16[(theRxCirCtrlr.RxXNL_IsFillingNextU16)++] = (theRxCirCtrlr.RxLinkCount) & 0x0000FFFF; //Time stamp.

	        //This switch tests the fragment type, and adjusts receiver state accordingly.
	        switch (theXNLRxWord & 0x00000F00) {  //Check frag type.
	        case 0x00000000:                      //Only Fragment.
	        case 0x00000100:                      //First of Multifragment.
	        case 0x00000200:                      //Continuing Multifragment.
	        case 0x00000300:                      //Last Multifragment.
	        	theRxCirCtrlr.theRxLink_State = WAITINGCSUM;
	        	break;
	        default:                              //No other values allowed.
	        	reset_IsFillingNextU16();
	        	break;
	        }
	        break;


	    case READINGFRAGMENT:
	    	  theRxCirBuffer.CirBufferElement16[(theRxCirCtrlr.RxXNL_IsFillingNextU16)++] = (theXNLRxWord & 0xFFFF0000) >> 16;
	    	  theRxCirCtrlr.RxLink_CSUM += (theXNLRxWord & 0xFFFF0000) >> 16;
	    	  theRxCirCtrlr.RxLink_Expected -= 2;
	    	  if (theRxCirCtrlr.RxLink_Expected <= 0) {
	    		  //All read in.
	    		  //Terminator should be in 2nd hWord.
	    		  //Shaoqun says useful bits not used. The packet will always end with $00BA. [9.1.2.8]
	    		  if ( (0x000000BA == (theXNLRxWord  & 0x0000FFFF))
	    			  && (theRxCirCtrlr.RxLink_CSUM == 0)  )  {
	    			  post_message();
	    		  }
	    		  reset_IsFillingNextU16();
	    		  theRxCirCtrlr.theRxLink_State = WAITINGFORHEADER;
	    		  break;
	    	  }

	    	  //Have not broken. 2nd hWord contains payload.
	    	  theRxCirBuffer.CirBufferElement16[(theRxCirCtrlr.RxXNL_IsFillingNextU16)++] = (theXNLRxWord & 0x0000FFFF);
	    	  theRxCirCtrlr.RxLink_CSUM  += (theXNLRxWord & 0x0000FFFF);
	    	  theRxCirCtrlr.RxLink_Expected -= 2;
	    	  if (theRxCirCtrlr.RxLink_Expected <= 0) {
	    		  //All read in. Next Word should be 0x00BA0000.
	    		  theRxCirCtrlr.theRxLink_State = WAITINGLASTTERM;
	    	  } //else, next Word contains more payload.
	    	  break;



	    case WAITINGCSUM:   //Gets here on CSUM. Expect at least one hWord payload. Gets here once on every fragment.
	    	theRxCirCtrlr.RxLink_CSUM  = (theXNLRxWord & 0xFFFF0000) >> 16;  //Stores CSUM
	    	theRxCirCtrlr.RxLink_CSUM += (theXNLRxWord & 0x0000FFFF);        //sums in first hWord
	    	theRxCirBuffer.CirBufferElement16[(theRxCirCtrlr.RxXNL_IsFillingNextU16)++] = (theXNLRxWord & 0x0000FFFF);
	    	theRxCirCtrlr.RxLink_Expected -= 2;
	    	if (theRxCirCtrlr.RxLink_Expected > 0) { //Normal case for greater than one byte payloads.
	    		theRxCirCtrlr.theRxLink_State = READINGFRAGMENT;
	    	}else{ //Sort of strange, one byte payload. Should not happen, but follow protocol.
	    		  	//Expect next word 0x00BA0000.
	    			//Note that all segments must align with a 32-bit boundary and beginning
	    			//of each XCMP/XNL payload frame must start on slot 3 Thus, segments
	    			//of odd length must append a 0x0000 at the end (slot 4) to ensure alignment. [9.1.3]
	    			theRxCirCtrlr.theRxLink_State = WAITINGLASTTERM;
	    	}
	    	break;



	    case WAITINGLASTTERM:     //Expecting last terminator 0x00BA0000.
	    	if (  (0x00BA0000 == (theXNLRxWord  & 0x00FF0000)) //Expected found.
	    	   &&  (theRxCirCtrlr.RxLink_CSUM == 0)  ) {       //Good checksum.
	    		post_message();
	    	}
	    	theRxCirCtrlr.theRxLink_State = WAITINGFORHEADER;
	    	reset_IsFillingNextU16();
	    	break;


	    }//End of theRxLink_State switch.
  }


  U32 XNL_PhyTx(void)
  {
	  U32 theReturn;
    //This is the code for handling any outgoing XNL Phy message.
    switch (txSchedule.TxLinkState){
    case IDLEWAITINGSCHEDULE:
    	//Test to see if there is anything to transmit.
    	if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){ //Nothing new to transmit. Send Idle.
    		theReturn = XNL_IDLE;// We're done here.
    	}else{ //A new instance has has been scheduled.

    		txSchedule.CurrentInstanceIndex = txSchedule.NextWaitingIndex; //Begin handling waiting instance.
    		txSchedule.NextWaitingIndex = TXINSTANCESBOUND;                //Allow BG to schedule new instance.
    		txSchedule.CurrentBlockIndex = 0;                              //Handle to first fragment. Assume index to a valid fragment block.
    		//txSchedule.Next16TxIndex = 0;                                //Points to first hWord in fragment block.
    		theReturn = TxBufferPool[TxInstancePool[txSchedule.CurrentInstanceIndex].BlockIndex[0]].XNL.theMAC_Header.phy_control;
    		txSchedule.BytesRemaining = theReturn & 0x000000FF;
    		theReturn |= PHYHEADER32; //Transmit 0xABCD0000 | Type/Length.
    		txSchedule.Next16TxIndex = 1;
    		txSchedule.TxLinkState = INSTANCETRANSMIT;
    		//We're done here. The new transmission has started.
    	}
    	break;

    case INSTANCETRANSMIT:
    	theReturn = TxBufferPool[TxInstancePool[txSchedule.CurrentInstanceIndex].BlockIndex[txSchedule.CurrentBlockIndex]].u16[txSchedule.Next16TxIndex] << 16;
    	txSchedule.Next16TxIndex += 1;
    	txSchedule.BytesRemaining -= 2;
    	if (txSchedule.BytesRemaining <= 0){ //Have written all the bytes (including 16-bit pad).
    		//Must immediately send 0x00BA in Slot 4.
    		theReturn |= PHYTERMRIGHT;
    		txSchedule.CurrentBlockIndex += 1;  //Advance to next fragment in this instance.
    		if (TxInstancePool[txSchedule.CurrentInstanceIndex].BlockIndex[txSchedule.CurrentBlockIndex] == TXBLOCKBOUND){ //All fragments of this instance have been transmitted.
    			TxInstancePool[txSchedule.CurrentInstanceIndex].behavior &= 0x7FFFFFFF; //Release ownership.
    			TxInstancePool[txSchedule.CurrentInstanceIndex].behavior |= FGHASSENT;  //Flag sent.
    			txSchedule.CurrentInstanceIndex = TXINSTANCESBOUND; //Signal for anyone watching.
    			txSchedule.TxLinkState = IDLEWAITINGSCHEDULE;       //Go back to waiting.
    		}else{ //Fragments remain in this instance.
    			//Next interrupt will send Header for next fragment.
    			//txSchedule.CurrentInstanceIndex is unchanged.
    			//txSchedule.CurrentBlockIndex already points to next valid fragment block.
    			//txSchedule.BytesRemaining needs to be initialized.
    			//txSchedule.Next16TxIndex needs to be initialized.
    			txSchedule.TxLinkState = CONTINUINGNEWFRAGMENT;
    		}
    		break; //This fragment finished.
    	}

    	//Have not broken. Transmit 2nd hWord.
    	theReturn |= TxBufferPool[TxInstancePool[txSchedule.CurrentInstanceIndex].BlockIndex[txSchedule.CurrentBlockIndex]].u16[txSchedule.Next16TxIndex];
    	txSchedule.Next16TxIndex += 1;
    	txSchedule.BytesRemaining -= 2;
    	if (txSchedule.BytesRemaining <= 0){ //Have written all the bytes (including 16-bit pad).
    		//Must send 0x00BA0000 next interrupt in Slot 3&4.
    		//txSchedule.CurrentInstanceIndex is unchanged.
			//txSchedule.CurrentBlockIndex needs advancing and checking.
			//txSchedule.BytesRemaining needs to be initialized.
			//txSchedule.Next16TxIndex needs to be initialized.
    		txSchedule.TxLinkState = SENDINGTERMINATOR32;
    	}
    	break;

    case SENDINGTERMINATOR32:
    	theReturn = PHYTERMLEFT;
    	txSchedule.CurrentBlockIndex += 1;  //Advance to next fragment in this instance.
    	if (TxInstancePool[txSchedule.CurrentInstanceIndex].BlockIndex[txSchedule.CurrentBlockIndex] == TXBLOCKBOUND){ //All fragments of this instance have been transmitted.
    		TxInstancePool[txSchedule.CurrentInstanceIndex].behavior &= 0x7FFFFFFF; //Release ownership.
    		TxInstancePool[txSchedule.CurrentInstanceIndex].behavior |= FGHASSENT;  //Flag sent.
    	    txSchedule.CurrentInstanceIndex = TXINSTANCESBOUND; //Signal for anyone watching.
    	    txSchedule.TxLinkState = IDLEWAITINGSCHEDULE;   //Go back to waiting.
    	}else{ //Fragments remain in this instance.
			//Next interrupt will send Header for next fragment.
			//txSchedule.CurrentInstanceIndex is unchanged.
			//txSchedule.CurrentBlockIndex already points to next valid fragment block.
			//txSchedule.BytesRemaining needs to be initialized.
			//txSchedule.Next16TxIndex needs to be initialized.
			txSchedule.TxLinkState = CONTINUINGNEWFRAGMENT;
    	}
    	break; //This fragment finished.

    case CONTINUINGNEWFRAGMENT:
    	theReturn = TxBufferPool[TxInstancePool[txSchedule.CurrentInstanceIndex].BlockIndex[txSchedule.CurrentBlockIndex]].MAC.theMAC_Header.phy_control;
    	txSchedule.BytesRemaining = theReturn & 0x00FF;
    	theReturn |= PHYHEADER32;     //Transmit 0xABCD0000  | Type/Length.
    	txSchedule.Next16TxIndex = 1;
    	txSchedule.TxLinkState = INSTANCETRANSMIT;
    	break; //We're done here. The continuing fragment has started.
    }//End of TxLinkState Switch
    return theReturn;
  }


  void RxPhyMedia(void)
  {
	    //This is the RxMedia Phy Handler.
	    switch (theRxMediaCtrlr.RxMediaState){
	    case WAITINGABAB:
	    	if ( RxBuffer[BufferIndex].thePayload_Channel.word[0] == 0xABCD5A5A) break;                  //Ignore Idles.
	    	if ((RxBuffer[BufferIndex].thePayload_Channel.word[0]  & 0xFFFF0000) != 0xABCD0000) break;   //Skip until Header.
	    	if ((RxBuffer[BufferIndex].thePayload_Channel.word[0]  & 0x0000F000) != 0x00005000) break;   //Skip on non-PAYLOAD_DATA_RX.
	    	theRxMediaCtrlr.RxBytesWaiting = RxBuffer[BufferIndex].thePayload_Channel.word[0] & 0x000000FF;
	    	if ((RxBuffer[BufferIndex].thePayload_Channel.word[0]  & 0x00000F00) <= 1){  //Frag type must process Array Discriptor.
	    		//The first word of the media access payload must be the Array descriptor length. And the
	    	    //unit of the length is in word (16-bit). The length field itself does not count into the length.
	    	    //When there is no array descriptor, the length must be set to zero.[9.1.4.1]
	    		if ((theRxMediaCtrlr.RxBytesWaiting -= 4) <= 0) break;          //Nothing beyond this Phy buffer. Keep looking for Header
	    		theRxMediaCtrlr.ArrayDiscLength = RxBuffer[BufferIndex].thePayload_Channel.hword[2];
	    		switch (theRxMediaCtrlr.ArrayDiscLength){
	    		case 0:          //The usual case. Remaining word in Phy buffer is Audio.
	    			RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[3];
	    			theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
	    			if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	    			theRxMediaCtrlr.RxMediaState = READINGMEDIA;
	    			break;
	    		case 1:          //The next usual case.
	    			//In general case, add code to process single word Array discriptor.
	    			theRxMediaCtrlr.RxMediaState = READINGMEDIA;
	    			break;
	    		default:         //So far, can't happen, but need to code anyway.
	    			//In general, add code to process multiword array discriptor.
	    		    theRxMediaCtrlr.ArrayDiscLength -= 1;
	    		    theRxMediaCtrlr.RxMediaState = READINGARRAYDISCRPT;
	    		}
	    		break;
	    	}
	    	//Code gets here on middle or last fragment. No Array descriptor.
	    	if (theRxMediaCtrlr.RxBytesWaiting < 2) break;          //This shouldn't happen, but must check.
	    	RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.word[1] & 0x0000FFFF;
	    	theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
	    	if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	    	if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0) break;  //This shouldn't happen, but must check;
	    	theRxMediaCtrlr.RxMediaState = READINGMEDIA;
	    	break; //End of WAITINGABAB.

	    case READINGMEDIA:
	    	RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[0];
	    	theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
	    	if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	    	if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0){
	    		theRxMediaCtrlr.RxMediaState = WAITINGABAB;
	    		break;
	    	}

	    	RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[1];
	    	theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
	    	if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	    	if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0){
	    	   theRxMediaCtrlr.RxMediaState = WAITINGABAB;
	    	   break;
	    	}

	    	RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[2];
	    	theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
	    	if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	    	if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0){
	    	    theRxMediaCtrlr.RxMediaState = WAITINGABAB;
	    	    break;
	    	}

	    	RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[3];
	    	theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
	    	if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	    	if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0){
	    	    theRxMediaCtrlr.RxMediaState = WAITINGABAB;
	    	    break;
	    	}
	    	break; //End of READINGMEDIA.


		case READINGARRAYDISCRPT:  //So far, this cannot happen, but needed for forward compatibility.
			//Array descriptorLength is greater than 0 on entry here.
			if(theRxMediaCtrlr.ArrayDiscLength > 4){ //All 4 words are still array discriptor.
				//For now, continue discarding.
				theRxMediaCtrlr.ArrayDiscLength -= 4;
				theRxMediaCtrlr.RxBytesWaiting  -= 8;
				if (theRxMediaCtrlr.RxBytesWaiting <= 0)theRxMediaCtrlr.RxMediaState = WAITINGABAB;
				break;
			}

			switch (theRxMediaCtrlr.ArrayDiscLength){ //1,2, or 3
			case 3:
				if ((theRxMediaCtrlr.RxBytesWaiting -= 6) <= 0) break;  //Throw away 3 hWords.
				RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[3];
				theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
				if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
				theRxMediaCtrlr.RxBytesWaiting -= 2;
				break;
			case 2:
				if ((theRxMediaCtrlr.RxBytesWaiting -= 4) <= 0) break;  //Throw away 2 hWords.
				RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[2];
				theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
				if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
				if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0)break;

				RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[3];
				theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
				if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
				theRxMediaCtrlr.RxBytesWaiting -= 2;
			    break;;
			case 1:
				if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0) break;  //Throw away 1 hWords.
				RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[1];
				theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
				if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
				if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0)break;

				RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[2];
				theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
				if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
				if ((theRxMediaCtrlr.RxBytesWaiting -= 2) <= 0)break;

				RxMediaBuffer[theRxMediaCtrlr.RxMedia_IsFillingNext16] = RxBuffer[BufferIndex].thePayload_Channel.hword[3];
				theRxMediaCtrlr.RxMedia_IsFillingNext16 += 1;
				if (theRxMediaCtrlr.RxMedia_IsFillingNext16 >= RXMEDIABUFFERSIZE)theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
				theRxMediaCtrlr.RxBytesWaiting -= 2;
				break;
			}
			if (theRxMediaCtrlr.RxBytesWaiting <= 0){
				theRxMediaCtrlr.RxMediaState = WAITINGABAB;
			}else{
				theRxMediaCtrlr.RxMediaState = READINGMEDIA;
			}
			break;  //End of READINGARRAYDISCRPT.



	    case BGFORCERESET: //Do nothing.
	    	break;
	    }//End of RxMedia Phy Handler.
  }

  //brief Default interrupt handler.
  __attribute__((__interrupt__))
  static void _unhandled_interrupt(void)
  {
	  // Catch unregistered interrupts.
	  while (TRUE);
  }

  //External (accelerometer) interrupt handler.
  __attribute__((__interrupt__))
  static void _external_interrupt(void)
  {
        //Reserved for Double-Tap Interrupt.
  }

__attribute__((__interrupt__))
static void pdca_int_handler(void)
{
	intStartCount = Get_system_register(AVR32_COUNT);
	//TxBuffer and RxBuffer terminates the Option Card SSC Physical Layer.
	BufferIndex ^= 0x01; //Toggle Index.
	(&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->marr = (U32)(&TxBuffer[BufferIndex].theXNL_Channel.word);
	(&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->tcrr = 3;  //Three words xfered each DMA.


	(&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->marr = (U32)(&RxBuffer[BufferIndex].theXNL_Channel.word);
	(&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->tcrr = 3;  //Three words xfered each DMA.
	(&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->isr;

	theRxCirCtrlr.RxLinkCount += 1;

	XNL_PhyRx(RxBuffer[BufferIndex].theXNL_Channel.word);
	TxBuffer[BufferIndex].theXNL_Channel.word = XNL_PhyTx();
	RxPhyMedia();


    //Dummy code to Idle Tx Media.
    TxBuffer[BufferIndex].thePayload_Channel.word[0] = PAYLOADIDLE0;
    TxBuffer[BufferIndex].thePayload_Channel.word[1] = PAYLOADIDLE1;
    intDuration = Get_system_register(AVR32_COUNT) - intStartCount;

}//End of pdca_int_handler.





// This routine attempts to reserve a TxInstance
//with the number of requested fragment blocks.
//If successful, it will return an Index
//to the Instance (<TXINSTANCESBOUND). If unsuccessful,
//it will return == TXINSTANCESBOUND. The requesting routine
//must check this!
U8 reserveTxInstance(int BlocksNeeded)
{
	U8  i;
	U8  blockIndex;
	U8  instanceIndex;

	//This is really un-necessary, and wastes two bytes of RAM. If the
	//values stored in AvailableInstanceCount or AvailableBlockCount
	//ever drift, there will be a problem. This *may* be useful for
	//automating performance metrics. [I *think* you can remove this test
	//and still work.]
	if ((txSchedule.AvailableInstanceCount < 1)
		| (txSchedule.AvailableBlockCount < BlocksNeeded))
		return TXINSTANCESBOUND;


	for (instanceIndex=0; instanceIndex<TXINSTANCESIMPLEMENTED; instanceIndex++){
		if (TxInstancePool[instanceIndex].behavior == NULLINSTANCEBEHAVIOR){
			//Code gets here when the available instance is found.
			TxInstancePool[instanceIndex].RetryTime = theRxCirCtrlr.RxLinkCount;
			txSchedule.AvailableInstanceCount -= 1;
			for (i=0; i<MAXPHYBLOCKS; i++){
				if (i < BlocksNeeded){   //Still looking.
					for (blockIndex=0; blockIndex<TXPOOLSIZE; blockIndex++){
						if (TxBlockReservation[blockIndex] == TXINSTANCESBOUND){
							TxBlockReservation[blockIndex] = instanceIndex;
							TxInstancePool[instanceIndex].BlockIndex[i] = blockIndex;
							txSchedule.AvailableBlockCount -= 1;
							break;
						}
					}
				}else{                   //Mark end of list.
					TxInstancePool[instanceIndex].BlockIndex[i] = TXBLOCKBOUND;
				}
			}
			//Code gets here when all the BlocksNeeded are allocated.
			return instanceIndex;
		}
	}
	//If using AvailableInstanceCount and AvailableBlockCount, code should never
	//get here unless something is very wrong. If these tests are not used,
	//code will get here if allocation fails. Then should call releaseTxInstance.
	// if (instanceIndex < TXINSTANCESBOUND) releaseTxInstance(instanceIndex);
	//Here, something is very wrong:
	DontPanic = FALSE;
	return TXINSTANCESBOUND;
}

U8 findTxInstance_byOpCode(U16 opcode)
{
	U8 i;

	for (i = 0; i < TXINSTANCESIMPLEMENTED; i++){                    //Check each Instance.
		if (TxInstancePool[i].behavior > 0) {  //For non-Null behavior,
			                                                         //Check for opcode.
			if (TxBufferPool[TxInstancePool[i].BlockIndex[0]].XNL.theXNL_Header.opcode == opcode) return i;
		}
	}
	return 	TXINSTANCESBOUND;  //Return none found.
}

U8 findTxInstance_byTransID(U16 TransID)
{
	U8 i;

	for (i = 0; i < TXINSTANCESIMPLEMENTED; i++){                    //Check each Instance.
		if (TxInstancePool[i].behavior > 0) {  //For non-Null behavior,
			                                                         //Check for TransID.
			if (TxBufferPool[TxInstancePool[i].BlockIndex[0]].XNL.theXNL_Header.transactionID == TransID) return i;
		}
	}
	return 	TXINSTANCESBOUND;  //Return none found.
}

U8 findTxInstance_byTimeout(void)
{
	U8  i;

	for (i = 0; i < TXINSTANCESIMPLEMENTED; i++){                    //Check each Instance.
		if (TxInstancePool[i].behavior > 0) {  //For non-Null behavior,
			//FG writes to RxLinkCount *should* be atomic. If not, could be FG/BG contention.
			if (theRxCirCtrlr.RxLinkCount - TxInstancePool[i].RetryTime < 0x7FFFFFFF) return i;
		}
	}
	return 	TXINSTANCESBOUND;  //Return none found.
}

void releaseTxInstance(U8 instanceIndex)
{
	U8  i;

	if (instanceIndex >= TXINSTANCESBOUND) return;

	//It is remotely possible we could be asked to release an instance
	//while it is still owned by the transmitter. An elegant solution
	//may be to mark this, and handle the deletion through garbage collection.
	//I'd need to set up the FG/BG semiphores differently for this.
	//Right now, I do not want to take the time to do this. Since this
	//really should never happen, and since time delays in BG really should not
	//cause any problems, I'm just using an ugly wait loop here.
	while (((TxInstancePool[instanceIndex].behavior) & FGOWNSBEHAVIOR) == FGOWNSBEHAVIOR);

	for (i=0; i<MAXPHYBLOCKS; i++){
		if (TxInstancePool[instanceIndex].BlockIndex[i] != TXBLOCKBOUND){
			TxBlockReservation[TxInstancePool[instanceIndex].BlockIndex[i]]
			                                            = TXINSTANCESBOUND;
			TxInstancePool[instanceIndex].BlockIndex[i] = TXBLOCKBOUND;
			txSchedule.AvailableBlockCount += 1;
		}else{  //We've removed all allocations.
			break;
		}
	}
	TxInstancePool[instanceIndex].behavior = NULLINSTANCEBEHAVIOR;
	txSchedule.AvailableInstanceCount += 1;
}

void garbageCollect(void)
{
	U8  i;

	for (i = 0; i < TXINSTANCESIMPLEMENTED; i++){ //Check each Instance.
		if (((TxInstancePool[i].behavior) & FGHANDSHAKEMASK)  == OKTOGARBAGECOLLECT) {  //Check for can delete.
			releaseTxInstance(i);
		}
	}
}

void sumTxInstance(U8 instanceIndex)
{
	U32  fragwithinInstance;
	U32  indextofrag;
	S32  hWordswithinFrag;
	U32  indextohWord;
	U16 sumScratch;

	if (instanceIndex >= TXINSTANCESBOUND) return;

	for (fragwithinInstance=0; fragwithinInstance<MAXPHYBLOCKS; fragwithinInstance++){
		indextofrag = TxInstancePool[instanceIndex].BlockIndex[fragwithinInstance];
		if (indextofrag != TXBLOCKBOUND){
			sumScratch = 0;
			hWordswithinFrag = ((TxBufferPool[indextofrag].MAC.theMAC_Header.phy_control) & 0x00FF) - 2;
			hWordswithinFrag =  (hWordswithinFrag + (hWordswithinFrag & 0x0001)) >> 1; //Round up.
			indextohWord = 2;
			while (hWordswithinFrag>0){
				sumScratch += TxBufferPool[indextofrag].u16[indextohWord];
				indextohWord     += 1;
				hWordswithinFrag -= 1;
			}
			TxBufferPool[indextofrag].XNL.theMAC_Header.checksumspacesaver = -sumScratch;
		}else{ //We've summed all blocks.
			break;
		}
	}
}

void depleteAProcessedMessage(void)
{
	theRxCirCtrlr.RxXNL_ProcessWaitingIndex =
		((theRxCirCtrlr.RxXNL_ProcessWaitingIndex) + 1) & RXCIRBUFFERFRAGWRAP;
}

// const U32 authKey[] = { XX, XX, XX, XX };
// NOTE: XNL Authentication key available only to licensed MOTOTRBO developers.
// This code will not compile without a valid value assigned to authKey[].

void encipher(U32 *const v,
		      U32 *const w,
	    const U32 *const k)
{
	 register U32 y=v[0], z=v[1], sum=0;
	 register U32 delta= authDelta;
// NOTE: Delta available only to licensed MOTOTRBO developers.
// This code will not compile without a valid value assigned to delta.
	 register U32 a=k[0], b=k[1], c=k[2], d=k[3];
	 register U32 n=32;

	 while(n-->0)
	 {
	 sum += delta;
	 y += ((z << 4)+a) ^ (z+sum) ^ ((z >> 5)+b);
	 z += ((y << 4)+c) ^ (y+sum) ^ ((y >> 5)+d);
	 }

	 w[0]=y; w[1]=z;
 }

void ResetRxMedia(void)
{
	U32 i;
	theRxMediaCtrlr.RxMediaState = BGFORCERESET;
	theRxMediaCtrlr.RxMedia_IsFillingNext16 = 0;
	for (i=0; i<RXMEDIABUFFERSIZE; i++) RxMediaBuffer[i] = 0;
	theRxMediaCtrlr.RxMediaState = WAITINGABAB;
}



void initXNL(void)
{
	int i;
	U8 masterQuery;

	ResetRxMedia();
	theRxCirCtrlr.theRxLink_State = WAITINGFORHEADER;
	theRxCirCtrlr.RxLinkCount =                  0;
	theRxCirCtrlr.RxXNL_IsFillingMessageIndex =  0;
	theRxCirCtrlr.RxXNL_IsFillingNextU16 =       0;
	theRxCirCtrlr.RxXNL_ProcessWaitingIndex =    0;
	theXNL_Ctrlr.XNL_State = XNL_UNCONNECTEDWAITINGSTATUS;
	theXNL_Ctrlr.isIncomingMessage =         FALSE;


	  //TxBlockReservation has TXPOOLSIZE entries, [0,1,2...TXPOOLSIZE-1],
	  //one for if each available fragment block.
	  //Each entry is assigned to one of the available Instances (threads).
	  //There are TXINSTANCESIMPLEMENTED implemented instances,
	  //enabling TXINSTANCESIMPLEMENTED independent Tx mesage threads.
	  //Initially each TxBlockReservation entry is set to
	  //TXINSTANCESBOUND (== TXINSTANCESIMPLEMENTED), marking the block as available.
	  for (i=0; i<TXPOOLSIZE; i++) TxBlockReservation[i] = TXINSTANCESBOUND;

	  txSchedule.AvailableBlockCount = TXPOOLSIZE;

	  //TxInstancePool has TXINSTANCESIMPLEMENTED entries, [0,1...TXINSTANCESIMPLEMENTED-1],
	  //one for ack available Tx Instance (thread).
	  //Each Instance State is initially set to NULLINSTANCESTATE,
	  //flagging it as available.
	  for (i=0; i<TXINSTANCESIMPLEMENTED; i++)
		  TxInstancePool[i].behavior = NULLINSTANCEBEHAVIOR;

	  txSchedule.AvailableInstanceCount = TXINSTANCESIMPLEMENTED;

	  //On initialization, the Tx scheduler is forced to TXINSTANCESBOUND,
	  //indicating that none of the available threads are scheduled.
	  //The FG will continually transmit the MAC Idle message.
	  txSchedule.CurrentBlockIndex = TXINSTANCESBOUND;
	  txSchedule.NextWaitingIndex  = TXINSTANCESBOUND;
	  txSchedule.TxLinkState = IDLEWAITINGSCHEDULE;


	  // All non-master devices must wait until a MASTER_STATUS_BRDCST is received. If no
	  // MASTER_STATUS_BRDCST message is received within 500 ms, then the non-master
	  // device should send a DEVICE_MASTER_QUERY message. If the master is present,
	  // then it will respond with a MASTER_STATUS_BRDCST message. If a master is not
	  // present, then the device shall re-send the DEVICE_MASTER_QUERY message. This
	  // will handle the differences in power up times, as well as the case when a device is
	  // connected after the rest of the system has already powered up. 5.2.1 XCMP/XNL
	  // Development Guide.
	  //
	  // This code provides for this retry by creating an intance of the DEVICE_MASTER_QUERY
	  // message, but not sending it. Should the MASTER_STATUS_BRDCST not be received
	  // before the retry timout, the DEVICE_MASTER_QUERY will then be sent.
	  masterQuery = reserveTxInstance(DEVICE_MASTER_QUERY_BLOCK_COUNT);
	  if (masterQuery == TXINSTANCESBOUND){ //Good practice to test and recover.
		  DontPanic = FALSE;  //We just started, Panic!
		  return;
	  }

	  for (i=0; i<DEVICE_GENERIC_U16_COUNT; i++)
	     TxBufferPool[TxInstancePool[masterQuery].BlockIndex[0]].u16[i]
	     = DEVICE_GENERIC_PROTO[i];

	  //Insert opcode.
	  TxBufferPool[TxInstancePool[masterQuery].BlockIndex[0]].XNL.theXNL_Header.opcode
	    = XNL_DEVICE_MASTER_QUERY;

	  //fill in checksums.
	  sumTxInstance(masterQuery);

	  //Schedule in future.
	  TxInstancePool[masterQuery].RetryTime += STANDARDTIMEOUT;
	  TxInstancePool[masterQuery].behavior = TXXNLCTRLPROTO;
}

void processXNL_MASTER_STATUS_BRDCST(void)
{
	  U8 i;
	  U8 theInstance;

	   //The XNL_MASTER_STATUS_BRDCST message is sent out by the master device to indicate that the master has been
	  //determined and that non-master devices can now connect. The data payload for this
	  //message will contain the XNL version as well as the logical device identifier for the
	  //master device. The last field in the payload contains a flag that indicates whether or not
	  //an XNL_DATA_MSG has been sent out. This will indicate to a connecting device that it
	  //has missed messages. The XNL header will contain the master’s XNL address. 5.4.1
	  theXNL_Ctrlr.XNL_MasterAddress = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.source;
    //Could extract here Minor XNL Protocol Version Number.
    //Could extract here Major XNL Protocol Version Number.
    //Could extract here Master Logical Identifier.
    //Could extract here Message Sent Boolean.
	  releaseTxInstance(findTxInstance_byOpCode(XNL_DEVICE_MASTER_QUERY));
	  theXNL_Ctrlr.XNL_State = XNL_UNCONNECTEDWAITINGAUTHKEY;

	  //This message is sent by all non-master devices in order to get the authentication key to
	  //be used when establishing a connection. This message contains no payload data.
	  //XCMP/XNL Development Guide 5.4.3
	  theInstance = reserveTxInstance(DEVICE_AUTH_KEY_REQUEST_BLOCK_COUNT);
	  if (theInstance == TXINSTANCESBOUND){ //Good practice to test and recover.
	    DontPanic = FALSE;  //We just started, Panic!
	  	return;
	  }

	  for (i=0; i<DEVICE_GENERIC_U16_COUNT; i++)
	  	    TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u16[i]
	  	    = DEVICE_GENERIC_PROTO[i];

	  //Insert opcode.
	  TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.opcode
	      	  = XNL_DEVICE_AUTH_KEY_REQUEST;

	  //Use actual Master address.
	  TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination
	  = theXNL_Ctrlr.XNL_MasterAddress;

	  //fill in checksums.
	  sumTxInstance(theInstance);

	  //Attempt to schedule transmission.
	  //Scheduling *should* be immediate here as we've just started up.
	  //Failure to get immediate scheduling here may be concern for Panic.
	  if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
		  //Immediate transmission allowed
		  TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
		  TxInstancePool[theInstance].behavior = OWNEDXNLCTRLPRPTO;
		  txSchedule.NextWaitingIndex = theInstance;
	  }else{
		  //Leave RetryTime at current time.
		  TxInstancePool[theInstance].behavior = TXXNLCTRLPROTO;
	  }

}

void processXNL_DEVICE_AUTH_KEY_REPLY(void)
{
    U32 v_vector[2], w_vector[2];
	U8 i, theInstance;

	

  //The payload for XNL_DEVICE_AUTH_KEY_REPLY message is a temporary XNL address to use during the connection
  //process and an unencrypted 8 byte random number generated by the master. This
  //number should be encrypted by the receiving device and will be used to authenticate
  //the connection request. 5.4.4
  //Temporarily use temporary device address
  theXNL_Ctrlr.XNL_DeviceAddress
    = (theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.TemporaryXNLAddress;

  //Get Array of values to be encrypted into an aligned 2X32bits.
  v_vector[0] = ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[0])<<24
               | ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[1])<<16
               | ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[2])<<8
               | ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[3]);
  v_vector[1] = ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[4])<<24
               | ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[5])<<16
               | ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[6])<<8
               | ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_AUTH_KEY_REPLY.UnencryptedAuthenticationValue[7]);

  encipher(&v_vector[0], &w_vector[0], &authKey[0]);

  releaseTxInstance(findTxInstance_byOpCode(XNL_DEVICE_AUTH_KEY_REQUEST));
  	  theXNL_Ctrlr.XNL_State = XNL_UNCONNECTEDWAITINGDEVICECONN;

  	//This message is sent by all non-master devices in order to establish a logical
  	//connection with the master. If a particular XNL address is desired (for fixed address
  	//systems), a preferred address field should be used. Otherwise a value of 0x0000 should
  	//be used. For systems that contain both fixed address and dynamic address
  	//assignments, the preferred address cannot be guaranteed. The payload for this
  	//message also includes a device type value, authentication index, and the encrypted
  	//authentication value. XCMP/XNL Development Guide 5.4.5
  	theInstance = reserveTxInstance(DEVICE_CONN_REQUEST_BLOCK_COUNT);
  		  if (theInstance == TXINSTANCESBOUND){ //Good practice to test and recover.
  		    DontPanic = FALSE;  //We just started, Panic!
  		  	return;
  		  }

  		for (i=0; i<DEVICE_CONN_REQUEST_U16_COUNT; i++)
  			 TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u16[i]
  			 = DEVICE_CONN_REQUEST_PROTO[i];

  		//Use actual Master address.
  		TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination
  		     = theXNL_Ctrlr.XNL_MasterAddress;

  		//Use Temporary address.
  		//There is a possible flaw in the XNL protocol; several Option Cards may
  		//power up at about the same time, XNL_DEVICE_AUTH_KEY_REPLY. Thus their Temporary Address
  		//will also be the same. So, they will be sending the same message here,
  		//and all will receive the same DEVICE_CONN_REPLY. Not real sure what's going
  		//to happen with multiple conrol heads, etc. One suspects the Rocket Scientists will
  		//eventually figure this out, and demand a transaction ID based on Device Type in
  		//the XNL_DEVICE_AUTH_KEY_REQUEST.
  		TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.source
  		  	 = theXNL_Ctrlr.XNL_DeviceAddress;

  		//We know encrypted array happens to be aligned to 32-bit boundary.
  		TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u32[5]
  		                                                = w_vector[0];
  		TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u32[6]
  		  		                                        = w_vector[1];

  		//fill in checksums.
  	    sumTxInstance(theInstance);

  	    //Presently the protocol enables, but does not require the Device to Authenticate the Master.
  	    //The present exchange is vulnerable to playback attack anyway, and only one Master
  	    //can be talking on the physical bus, this is kind of silly. I've included it here as just
  	    //another opportunity to check that the sequence has sanity.
  	    encipher(&w_vector[0], &v_vector[0], &authKey[0]);
  	    //Squirrel away the result for comparison with value returned in DEVICE_CONN_REPLY.
  	    //This is not transmitted. Only using spare instance memory.
  	    TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u32[7]
  	    		                                           = v_vector[0];
  	    TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u32[8]
  	    		  		                                   = v_vector[1];


  	    //Attempt to schedule transmission.
  	  	//Scheduling *should* be immediate here as we've just started up.
  	  	//Failure to get immediate scheduling here may be concern for Panic.
  	  	//In general, a schedule failure can just wait for retry.
  	  	if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
  	  		//Immediate transmission allowed
  	  	    TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
  	  	    TxInstancePool[theInstance].behavior = OWNEDXNLCTRLPRPTO;
  	  		txSchedule.NextWaitingIndex = theInstance;
  	  	}else{
  	  	    //Leave RetryTime at current time.
  	  	    TxInstancePool[theInstance].behavior = TXXNLCTRLPROTO;
  	  	}

}

void processXNL_DEVICE_CONN_REPLY(void)
{
	U8 i, theInstance;

	
	//Bool MasterAuthenticate;
	//The reply to the Device Connection Request contains a result code (connection
	//successful or not) and the XNL address and device logical address to use for all future
	//communications. In addition, the reply will contain a unique 8-bit value that should be
	//used as the upper byte of the transaction ID and an 8-byte encrypted value that the
	//device can use to authenticate the master. XCMP/XNL Development Guide 5.4.6


	//Test result code
	if((((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_CONN_REPLY.Result_Base)
	   & 0x0000FF00) != 0x00000100){
		//Rejected. The device must retry the authentication process at this point by sending out a
		//new AUTH_KEY_REQUEST message. XCMP/XNL Development Guide Section 5.2.3
		  releaseTxInstance(findTxInstance_byOpCode(XNL_DEVICE_CONN_REQUEST));
		  theXNL_Ctrlr.XNL_State = XNL_UNCONNECTEDWAITINGAUTHKEY;
		  theInstance = reserveTxInstance(DEVICE_AUTH_KEY_REQUEST_BLOCK_COUNT);
		  if (theInstance == TXINSTANCESBOUND){ //Good practice to test and recover.
		  	  DontPanic = FALSE;  //Panic!
		  	  return;
		  }

		  for (i=0; i<DEVICE_GENERIC_U16_COUNT; i++)
		  	  TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u16[i]
		  	  	 = DEVICE_GENERIC_PROTO[i];

		  //Insert opcode.
		  TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.opcode
		  	   = XNL_DEVICE_AUTH_KEY_REQUEST;

		  //Use actual Master address.
		  TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination
		  	= theXNL_Ctrlr.XNL_MasterAddress;

		  //fill in checksums.
		  sumTxInstance(theInstance);

		  if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
			  //Immediate transmission allowed.
			  TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
			  TxInstancePool[theInstance].behavior = OWNEDXNLCTRLPRPTO;
			  txSchedule.NextWaitingIndex = theInstance;
		  }else{
			  //Leave RetryTime at current time.
			  TxInstancePool[theInstance].behavior = TXXNLCTRLPROTO;
		  }



	}else{ //connection accepted
		//Record Transaction ID Base
		theXNL_Ctrlr.XNL_TransactionIDBase
		  = (((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_CONN_REPLY.Result_Base) & 0x000000FF) << 8;
		//Record Device Logical Address
		theXNL_Ctrlr.XNL_DeviceLogicalAddress
			      = (theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_CONN_REPLY.LogicalAddress;
		//Record permanent device address
	    theXNL_Ctrlr.XNL_DeviceAddress
	      = (theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_CONN_REPLY.XNLAddress;
	    //Initialize 3Bit  Rollover.
	    theXNL_Ctrlr.XNL_3BitRollover = 0x0000;

	    ////Optionally, authenticate Master.
	    //theInstance = findTxInstance_byOpCode(XNL_DEVICE_CONN_REQUEST));
	    //MasterAuthenticate = PASS;
	    //For (i=0; i<8; i++) {
	    //	if (
	    //((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentDEVICE_CONN_REPLY.EncryptedAuthenticationValue[i])
	    //!= (TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].u8[i+20]))
	    //	MasterAuthenticate = FAIL;
	    //}

	    releaseTxInstance(findTxInstance_byOpCode(XNL_DEVICE_CONN_REQUEST));
	    theXNL_Ctrlr.XNL_State = XNL_CONNECTED;
	}
}

Bool scheduleXNL_ACK(void)
{
	U8 theInstance;

	theInstance = reserveTxInstance(XNL_DATA_MSG_ACK_BLOCK_COUNT);
	if (theInstance == TXINSTANCESBOUND){ //Are we able to ACK?
		  	return FALSE;
		  }
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theMAC_Header.phy_control = 0x400E;

	//Turn around Flags.
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.flags
	         = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.flags;

	//Insert opcode.
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.opcode
		      	  = XNL_DATA_MSG_ACK;

    //ACK Destination Address is Source of XNL_Message.
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination
		  = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.source;

	//ACK Source Address is my address.
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.source
	      = theXNL_Ctrlr.XNL_DeviceAddress;

	//Turn around Transaction ID.
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.transactionID
	      = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.transactionID;

	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.payloadLength
          = 0;

	//fill in checksums.
	sumTxInstance(theInstance);

	if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
		//Immediate transmission allowed.
		TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
	    TxInstancePool[theInstance].behavior = OWNEDXCMPACKPROTO;
		txSchedule.NextWaitingIndex = theInstance;
	}else{
		//Leave RetryTime at current time.
		TxInstancePool[theInstance].behavior = TXXCMPACKPROTO;
	}


	return TRUE;
}

U16 newFlag(void)
{
	U16 flag;
	flag = (theXNL_Ctrlr.XNL_3BitRollover + 1) & 0x0007;
	theXNL_Ctrlr.XNL_3BitRollover = flag;
	return flag;
}

U16 newTransID(void)
{
	U16 TransID;
	TransID = ((theXNL_Ctrlr.XNL_TransactionIDBase & 0x00FF) + 1) & 0x00FF;
	TransID |= theXNL_Ctrlr.XNL_TransactionIDBase & 0xFF00;
	theXNL_Ctrlr.XNL_TransactionIDBase = TransID;
	return TransID;
}

void sendOpcode_Not_Supported(U16 XCMPopcode)
{
	U8 theInstance;

	theInstance = reserveTxInstance(DEVINITSTS_BLOCK_COUNT);
	if (theInstance == TXINSTANCESBOUND){ //Are we able to schedule?
		return;
	}

	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]]
		                                                  .XNL
		                                                  .theXNLpayload
		                                                  .ContentXNL_DATA_MSG
		                                                  .XCNPopcode
		                                               = XCMPopcode | 0x8000;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]]
		                                                  .XNL
		                                                  .theXNLpayload
		                                                  .ContentXNL_DATA_MSG
		                                                  .u8[0]
		                                               = XCMPRESULT_NOTSUPPORTED;

	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theMAC_Header.phy_control = 0x4011;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.opcode = XNL_DATA_MSG;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.flags = 0x0100 | newFlag();
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.source;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.source = theXNL_Ctrlr.XNL_DeviceAddress;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.transactionID = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.transactionID;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.payloadLength = 0x0003;
	sumTxInstance(theInstance);

	if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
			//Immediate transmission allowed.
			TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
			TxInstancePool[theInstance].behavior = OWNEDXCNPMSGPROTO;
			txSchedule.NextWaitingIndex = theInstance;
	}else{
			//Leave RetryTime at current time.
			TxInstancePool[theInstance].behavior = TXXCMPMSGPROTO;
	}
}

Bool sendTONECTRLREQ(void)
{
	U8 theInstance;
	U8 i;

	theInstance = reserveTxInstance(TONECTRLREQ_BLOCK_COUNT);
	if (theInstance == TXINSTANCESBOUND){ //Are we able to schedule?
			return FALSE;
	}

	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]]
		                                              .XNL
		                                              .theXNLpayload
		                                              .ContentXNL_DATA_MSG
		                                              .XCNPopcode = XCMP_TONECTRLREQ;
	for (i=0; i<DEVINITSTS_BYTE_COUNT; i++){
	     TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]]
			                                          .XNL
			                                          .theXNLpayload
			                                          .ContentXNL_DATA_MSG
			                                          .u8[i] = TONECTRLREQPROTO[i];
	}

	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theMAC_Header.phy_control = 0x4018;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.opcode = XNL_DATA_MSG;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.flags = 0x0100 | newFlag();
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination = theXNL_Ctrlr.XNL_MasterAddress;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.source = theXNL_Ctrlr.XNL_DeviceAddress;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.transactionID = newTransID();
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.payloadLength = 0x000A;
	sumTxInstance(theInstance);

	if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
			//Immediate transmission allowed.
			TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
		    TxInstancePool[theInstance].behavior = OWNEDXCNPMSGPROTO;
			txSchedule.NextWaitingIndex = theInstance;
	}else{
			//Leave RetryTime at current time.
			TxInstancePool[theInstance].behavior = TXXCMPMSGPROTO;
	}
	return TRUE;
}


void sendDEVINITSTS(void)
{
	U8 theInstance;
	U8 i;

	theInstance = reserveTxInstance(DEVINITSTS_BLOCK_COUNT);
	if (theInstance == TXINSTANCESBOUND){ //Are we able to schedule?
		return;
	}
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]]
	                                                  .XNL
	                                                  .theXNLpayload
	                                                  .ContentXNL_DATA_MSG
	                                                  .XCNPopcode = XCMP_DEVINITSTS;
	for (i=0; i<DEVINITSTS_BYTE_COUNT; i++){
		TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]]
		                                              .XNL
		                                              .theXNLpayload
		                                              .ContentXNL_DATA_MSG
		                                              .u8[i] = DEVINITSTSPROTO[i];
	}
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theMAC_Header.phy_control = 0x4019;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.opcode = XNL_DATA_MSG;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.flags = 0x0100 | newFlag();
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.destination = 0x0000;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.source = theXNL_Ctrlr.XNL_DeviceAddress;
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.transactionID = newTransID();
	TxBufferPool[TxInstancePool[theInstance].BlockIndex[0]].XNL.theXNL_Header.payloadLength = 0x000B;
	sumTxInstance(theInstance);

	if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){
			//Immediate transmission allowed.
			TxInstancePool[theInstance].RetryTime += STANDARDTIMEOUT;
		    TxInstancePool[theInstance].behavior = OWNEDXCNPMSGPROTO;
			txSchedule.NextWaitingIndex = theInstance;
	}else{
			//Leave RetryTime at current time.
			TxInstancePool[theInstance].behavior = TXXCMPMSGPROTO;
	}
}

void processXNL_DATA_MSG_ACK(void)
{
	U16 DestinationAddress;
	U16 TransactionID;

	DestinationAddress = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.destination;
	if ((theXNL_Ctrlr.XNL_DeviceAddress) == DestinationAddress){
		//The ack is for me.
		TransactionID = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.transactionID;
		//if (instanceIndex >= TXINSTANCESBOUND) return;handled in releaseTxInstance.
		releaseTxInstance(findTxInstance_byTransID(TransactionID));
	}
}

void processXNL_DATA_MSG(void)
{
	U16 DestinationAddress;
	U16 XCMPopcode;
	U16 temp;

	DestinationAddress = (theRxCirCtrlr.pRxTemplate)->theXNL_Header.destination;
	if ((0x000000 == DestinationAddress)
		|| (theXNL_Ctrlr.XNL_DeviceAddress == DestinationAddress)){
        //The message is for me.
		if (scheduleXNL_ACK()){ //Try to schedule ACK.
		//If cannot schedule ACK, just leave without processing message;
		//XNL will retry again, and hopefully our Tx resources will then be free.
		//If ACK has been scheduled. It most likely is already	owned
		//by the transmitter, but possibly is waiting in Queue with immediate
	    //timeout.
		    XCMPopcode = (theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentXNL_DATA_MSG.XCNPopcode;
            switch (XCMPopcode){
            case XCMP_DEVINITSTS:
            	if ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentXNL_DATA_MSG.u8[4] == DIC){
            		bunchofrandomstatusflags |= DIC;  //Need do nothing else.
            	}else if((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentXNL_DATA_MSG.u8[4] != 0x02){
					bunchofrandomstatusflags  &= 0xFFFFFFFC; //Device Init no longer Complete.
					sendDEVINITSTS();			
				//}else{
            		//bunchofrandomstatusflags  &= 0xFFFFFFFC; //Device Init no longer Complete.
            		//sendDEVINITSTS();
            	}
            	break;
            case XCMP_DEVMGMTBCST:
            	temp  = (theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentXNL_DATA_MSG.u8[1] << 8;
            	temp |= (theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentXNL_DATA_MSG.u8[2];
            	if (temp == theXNL_Ctrlr.XNL_DeviceLogicalAddress){
            		if ((theRxCirCtrlr.pRxTemplate)->theXNL_Payload.ContentXNL_DATA_MSG.u8[0] == 0x01){
            			//Enable Option Board
            			bunchofrandomstatusflags |= 0x00000002;
            		}else{
            			//Disable Option Board.
            			bunchofrandomstatusflags &= 0xFFFFFFFD;
            		}
            	}
            	break;
            default:
            	if ((XCMPopcode & XCMP_MTMask) == XCMP_requestMT){
            	   sendOpcode_Not_Supported(XCMPopcode);
            	}
            	break;
            }

		}
	}
}


void process_XNL(void)
{
  U32 temp;
  U8 SomeInstance;

  garbageCollect();

  theXNL_Ctrlr.isIncomingMessage = FALSE;
  if (theRxCirCtrlr.RxXNL_ProcessWaitingIndex != theRxCirCtrlr.RxXNL_IsFillingMessageIndex) {

	  //Align XNL Template with message in circular buffer.
	  theRxCirCtrlr.pRxTemplate = (RxTemplate*)(&(theRxCirBuffer.theRxFragment[theRxCirCtrlr.RxXNL_ProcessWaitingIndex]));
      if ( 0 == (((theRxCirCtrlr.pRxTemplate)->theMAC_Header.phy_control) & 0x0F00)){
    	  theXNL_Ctrlr.isIncomingMessage = TRUE;
      }else{ //This simple implementation throws away multiple fragments.
    	  depleteAProcessedMessage();
      }
  }

  switch (theXNL_Ctrlr.XNL_State) {
  case XNL_UNCONNECTEDWAITINGSTATUS:
    if   (theXNL_Ctrlr.isIncomingMessage) {
      if ( XNL_MASTER_STATUS_BRDCST ==
    	  (theRxCirCtrlr.pRxTemplate)->theXNL_Header.opcode ){
	    processXNL_MASTER_STATUS_BRDCST();
      }
	      depleteAProcessedMessage(); //This state depletes everything.
    }
    break;


  case XNL_UNCONNECTEDWAITINGAUTHKEY:
	  if   (theXNL_Ctrlr.isIncomingMessage) {
		  if (XNL_DEVICE_AUTH_KEY_REPLY ==
			  (theRxCirCtrlr.pRxTemplate)->theXNL_Header.opcode ){
			  processXNL_DEVICE_AUTH_KEY_REPLY();
		  }
	      depleteAProcessedMessage(); //It is possible we could receive another XNL_MASTER_STATUS_BRDCST
	                                  //here. It is possible the Master address could have changed.
	                                  //Strict protocol would deplete any queued DEVICE_AUTH_KEY_REQUEST
	                                  //and process the new XNL_MASTER_STATUS_BRDCST. If the Master address
	                                  //has not changed, this is just a valid repeat of the same STATUS_BRDCST.
	  }
	  break;


  case XNL_UNCONNECTEDWAITINGDEVICECONN:

	  AVR32_GPIO.port[1].ovrc  =  0x00000008;
  	  if   (theXNL_Ctrlr.isIncomingMessage) {
  		  if (XNL_DEVICE_CONN_REPLY ==
  			  (theRxCirCtrlr.pRxTemplate)->theXNL_Header.opcode ){
  			  processXNL_DEVICE_CONN_REPLY();
  		  }
  	      depleteAProcessedMessage(); //It is possible we could receive another XNL_MASTER_STATUS_BRDCST
  	                                  //here.
  	                                  //It is possible we could receive another XNL_DEVICE_AUTH_KEY_REPLY
  	                                  //here, intended for us or someone else (no way to tell), having
  	                                  //a different Unencrypted Authentication Value. Strict protocol
  	                                  //should check for this, deplete any outstanding CONN_REQUESTs, and
  	                                  //process the new AUTH_KEY_REPLY. Since I suspect this will all
  	                                  //change in the future, I'll do nothing here.
  	  }
  	  break;

  case XNL_CONNECTED:

	  
  	  if   (theXNL_Ctrlr.isIncomingMessage) {
  		  switch ((theRxCirCtrlr.pRxTemplate)->theXNL_Header.opcode){
  		  case XNL_DEVICE_SYSMAP_BRDCST:

  			  break;
  		  case XNL_DATA_MSG:
  			  processXNL_DATA_MSG();
  			  break;
  		  case XNL_DATA_MSG_ACK:
  			  processXNL_DATA_MSG_ACK();
  			  break;
		  }//End of switch on XNL Header Opcode
  	      depleteAProcessedMessage();
  	  }
  	  break;


  default:

    break;
  } //End of switch on XNL_State.

  //Need to search for active instances with timeouts.
  SomeInstance = findTxInstance_byTimeout();
  if (SomeInstance != TXINSTANCESBOUND){//Some Instance needs a retry.
	  if (txSchedule.NextWaitingIndex == TXINSTANCESBOUND){//Scheduling allowed
	      //Formally, shouldn't retry something that's stuck in transmitter.
		  //Should in general never fail this test.
		  if (((TxInstancePool[SomeInstance].behavior) & FGOWNSBEHAVIOR) == 0x00000000){
			  temp = ((TxInstancePool[SomeInstance].behavior) & TXINSTANCERETRYMASK) - 1;
			  if (temp == 0){ //All retries exhausted.
				  //In general case should maybe signal some error.
				  releaseTxInstance(SomeInstance);
			  }else{
				  TxInstancePool[SomeInstance].behavior &= 0x3FFF0000; //Clear owned, sent, old count.
				  TxInstancePool[SomeInstance].behavior |= temp;       //Update new count;
				  TxInstancePool[SomeInstance].behavior |= FGOWNSBEHAVIOR;
				  TxInstancePool[SomeInstance].RetryTime = theRxCirCtrlr.RxLinkCount
		  								          + STANDARDTIMEOUT;
				  txSchedule.NextWaitingIndex = SomeInstance;
			  }
		  }

	  }
  }


}//End of process_XNL.





//All calculations in this function suppose that the Osc0 frequency is 12MHz.
//The oscillators are disabled by default after reset.The oscillators can be enabled by writing to
//the OSCnEN bits in MCCTRL. Operation mode (external clock or crystal) is chosen by writing to
//the MODE field in OSCCTRLn. [12.5.2]
//The SSC clock is generated by the power manager. Before using the SSC, the programmer
//must ensure that the SSC clock is enabled in the power manager.
//In the SSC description, Master Clock (MCK) is the bus clock of the peripheral bus to which the
//SSC is connected. [23.6.2]
//Switch to clock Osc0 (crystal)
//start PLL0 and switch main clock to PLL0 output
void local_start_pll0(void)
{
	//pm_switch_to_osc0(pm, 12000000, 3);
	//    pm_enable_osc0_crystal(pm, 12000000);
	//         pm_set_osc0_mode(pm,AVR32_PM_OSCCTRL0_MODE_CRYSTAL_G3);  0x00000007
	//    pm_enable_clk0(pm, 3);
	//         pm_enable_clk0_no_wait(pm, 3);
	               (&AVR32_PM)->oscctrl0 = 0x00000307;
	               (&AVR32_PM)->mcctrl   = 0x00000004;
	//         pm_wait_for_clk0_ready(pm);
	               while (!((&AVR32_PM)->poscsr & AVR32_PM_POSCSR_OSC0RDY_MASK));
	//    pm_switch_to_clock(pm, AVR32_PM_MCSEL_OSC0);
	               (&AVR32_PM)->mcctrl   = 0x00000005;

	//pm_pll_setup(pm,
	//               0,   // use PLL0
	//               7,   // MUL=7 in the formula
	//               1,   // DIV=1 in the formula
	//               0,   // Sel Osc0/PLL0 or Osc1/PLL1
	//               16); // lockcount in main clock for the PLL wait lock
	//pm_pll_set_option(pm, 0, //PLL number 0
	//                        1, //freq Set to 1 for VCO frequency range 80-180MHz
	//                        1, //div2 Divide the PLL output frequency by 2
	//                        0);//0 to enable the Wide-Bandith Mode
	//pm_pll_enable(pm,0);
	               (&AVR32_PM)->pll[0] = 0x1007010D;


	//pm_wait_for_pll0_locked(pm);
	              while (!((&AVR32_PM)->poscsr & AVR32_PM_POSCSR_LOCK0_MASK));

	//pm_cksel(pm, 1,  //Bus A clock divisor enable = 1
	//             0,  //Bus A select = 0 (PBA clock = 48MHz/2 = 24MHz).
	//             0,  //B clock divisor enable = 0
	//             0,  //Bus B select = 0
	//             0,  //HS Bus clock divisor enable = 0
	//             0); //HS Bus select = 0
	              (&AVR32_PM)->cksel = 0x00800000;

	//flashc_set_wait_state(1);
	              AVR32_FLASHC.fcr = 0x00000040;

	//pm_switch_to_clock(pm, AVR32_PM_MCSEL_PLL0);
	              (&AVR32_PM)->mcctrl   = 0x00000006;


	              AVR32_HMATRIX.mcfg[AVR32_HMATRIX_MASTER_CPU_INSTRUCTION] = 0x1;
}



void local_start_SSC(void)
{
//Before using the SSC receiver, the PIO controller must be configured to dedicate the SSC
//receiver I/O lines to the SSC peripheral mode. [23.6.1]
//Before using the SSC transmitter, the PIO controller must be configured to dedicate the SSC
//transmitter I/O lines to the SSC peripheral mode. [23.6.1]
// Assign GPIO to SSC.
	//gpio_enable_module
	//     gpio_enable_module_pin
	    AVR32_GPIO.port[1].pmr0c = 0x00000DC0;
	    AVR32_GPIO.port[1].pmr1c = 0x00000DC0;
	    AVR32_GPIO.port[1].gperc = 0x00000DC0;

   //Software reset SSC
  (&AVR32_SSC)->cr = AVR32_SSC_CR_SWRST_MASK;

  (&AVR32_SSC)->cmr = AVR32_SSC_CMR_DIV_NOT_ACTIVE << AVR32_SSC_CMR_DIV_OFFSET;           //For Slave


  //For Slave
  (&AVR32_SSC)->tcmr =
	            AVR32_SSC_TCMR_CKS_RK_CLOCK             << AVR32_SSC_TCMR_CKS_OFFSET    |
                AVR32_SSC_TCMR_CKO_INPUT_ONLY           << AVR32_SSC_TCMR_CKO_OFFSET    |
                1                                       << AVR32_SSC_TCMR_CKI_OFFSET    |
                AVR32_SSC_TCMR_CKG_NONE                 << AVR32_SSC_TCMR_CKG_OFFSET    |
                4                                       << AVR32_SSC_TCMR_START_OFFSET  |
                32                                      << AVR32_SSC_TCMR_STTDLY_OFFSET |
                63                                      << AVR32_SSC_TCMR_PERIOD_OFFSET;


  //For Slave
  (&AVR32_SSC)->tfmr =
	              31                                    << AVR32_SSC_TFMR_DATLEN_OFFSET |
                  0                                     << AVR32_SSC_TFMR_DATDEF_OFFSET |
                  1                                     << AVR32_SSC_TFMR_MSBF_OFFSET   |
                  2                                     << AVR32_SSC_TFMR_DATNB_OFFSET  |
                  0                                     << AVR32_SSC_TFMR_FSLEN_OFFSET  |
                  AVR32_SSC_TFMR_FSOS_INPUT_ONLY        << AVR32_SSC_TFMR_FSOS_OFFSET   |
                  0                                     << AVR32_SSC_TFMR_FSDEN_OFFSET  |
                  1                                     << AVR32_SSC_TFMR_FSEDGE_OFFSET;

  //For Slave
  (&AVR32_SSC)->rcmr =
	            AVR32_SSC_RCMR_CKS_RK_PIN               << AVR32_SSC_RCMR_CKS_OFFSET    |
                AVR32_SSC_RCMR_CKO_INPUT_ONLY           << AVR32_SSC_RCMR_CKO_OFFSET    |
                0                                       << AVR32_SSC_RCMR_CKI_OFFSET    |
                AVR32_SSC_RCMR_CKG_NONE                 << AVR32_SSC_RCMR_CKG_OFFSET    |
                AVR32_SSC_RCMR_START_DETECT_FALLING_RF  << AVR32_SSC_RCMR_START_OFFSET  |
                0                                       << AVR32_SSC_RCMR_STOP_OFFSET   |
                32                                      << AVR32_SSC_RCMR_STTDLY_OFFSET |
                63                                      << AVR32_SSC_RCMR_PERIOD_OFFSET;

  //For Slave
  (&AVR32_SSC)->rfmr =
	              31                                    << AVR32_SSC_RFMR_DATLEN_OFFSET |
                  0                                     << AVR32_SSC_RFMR_LOOP_OFFSET   |
                  1                                     << AVR32_SSC_RFMR_MSBF_OFFSET   |
                  2                                     << AVR32_SSC_RFMR_DATNB_OFFSET  |
                  0                                     << AVR32_SSC_RFMR_FSLEN_OFFSET  |
                  AVR32_SSC_RFMR_FSOS_INPUT_ONLY        << AVR32_SSC_RFMR_FSOS_OFFSET   |
                  1                                     << AVR32_SSC_RFMR_FSEDGE_OFFSET;


}

void local_start_PDC(void)
{
	  BufferIndex = 1;

	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->idr = AVR32_PDCA_RCZ_MASK | AVR32_PDCA_TRC_MASK | AVR32_PDCA_TERR_MASK;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->isr;                             //Dummy read?
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->mar = (U32)(&RxBuffer[0].theXNL_Channel.word);
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->tcr = 3;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->psr = AVR32_PDCA_PID_SSC_RX;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->marr = (U32)(&RxBuffer[1].theXNL_Channel.word);
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->tcrr = 3;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->mr = AVR32_PDCA_WORD;


	  TxBuffer[0].theXNL_Channel.word = XNL_IDLE;
	  TxBuffer[0].thePayload_Channel.word[0] = PAYLOADIDLE0;
	  TxBuffer[0].thePayload_Channel.word[1] = PAYLOADIDLE1;
	  TxBuffer[1].theXNL_Channel.word = XNL_IDLE;
	  TxBuffer[1].thePayload_Channel.word[0] = PAYLOADIDLE0;
	  TxBuffer[1].thePayload_Channel.word[1] = PAYLOADIDLE1;

	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->idr = AVR32_PDCA_RCZ_MASK | AVR32_PDCA_TRC_MASK | AVR32_PDCA_TERR_MASK; //Atomic!
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->isr;                                                                    //Dummy read?
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->mar = (U32)(&TxBuffer[0].theXNL_Channel.word);
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->tcr = 3;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->psr = AVR32_PDCA_PID_SSC_TX;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->marr = (U32)(&TxBuffer[1].theXNL_Channel.word);
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->tcrr = 3;
	  (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->mr = AVR32_PDCA_WORD;
}

//	SSC Related I/O
//	MAKO_DCLK				AVR32_SSC_TX_CLOCK_0_PIN		NU	[41 PortB Pin  9 00000200 Func 0]
//							AVR32_SSC_RX_CLOCK_0_PIN			[38 PortB Pin  6 00000040 Func 0]
//	Timer Clk				AVR32_TC_CLK0_0_1_PIN				[20 PortA Pin 20 00100000 Func 1]
//	MAKO_FSYNC				AVR32_SSC_TX_FRAME_SYNC_0_PIN		[43 PortB Pin 11 00000800 Func 0]
//							AVR32_SSC_RX_FRAME_SYNC_0_PIN		[40 PortB Pin  8 00000100 Func 0]
//	Timer Sync				AVR32_TC_B0_0_0_PIN					[33 PortB Pin  1 00000002 Func 0]
//	SSC_TX_DATA_ENABLE		AVR32_TC_A0_0_0_PIN					[32 PortB Pin  0 00000001 Func 0]
//	MAKO_TX					AVR32_SSC_TX_DATA_0_PIN				[42 PortB Pin 10 00000400 Func 0]
//	MAKO_RX					AVR32_SSC_RX_DATA_0_PIN				[39 PortB Pin  7 00000080 Func 0]
//

void local_start_timer(void)
{
    //Route CLK to Timer
      AVR32_GPIO.port[0].pmr0s = 0x00100000;
      AVR32_GPIO.port[0].pmr1c = 0x00100000;
      AVR32_GPIO.port[0].gperc = 0x00100000;
      //Route FS and Tri-State to Timer.
      AVR32_GPIO.port[1].pmr0c = 0x00000003;
      AVR32_GPIO.port[1].pmr1c = 0x00000003;
      AVR32_GPIO.port[1].gperc = 0x00000003;

      (&AVR32_TC)->bmr = 4;
      (&AVR32_TC)->channel[0].cmr =
      	                                AVR32_TC_BSWTRG_NONE       << AVR32_TC_BSWTRG_OFFSET   |
                                        AVR32_TC_BEEVT_NONE        << AVR32_TC_BEEVT_OFFSET    |
                                        AVR32_TC_BCPC_NONE         << AVR32_TC_BCPC_OFFSET     |
                                        AVR32_TC_BCPB_NONE         << AVR32_TC_BCPB_OFFSET     |
                                        AVR32_TC_ASWTRG_SET        << AVR32_TC_ASWTRG_OFFSET   |
                                        AVR32_TC_AEEVT_SET         << AVR32_TC_AEEVT_OFFSET    |
                                        AVR32_TC_ACPC_NONE         << AVR32_TC_ACPC_OFFSET     |
                                        AVR32_TC_ACPA_CLEAR        << AVR32_TC_ACPA_OFFSET     |
                                        1                          << AVR32_TC_WAVE_OFFSET     |
                                        AVR32_TC_WAVSEL_UP_NO_AUTO << AVR32_TC_WAVSEL_OFFSET   |
                                        1                          << AVR32_TC_ENETRG_OFFSET   |
                                        AVR32_TC_EEVT_TIOB_INPUT   << AVR32_TC_EEVT_OFFSET     |
                                        AVR32_TC_EEVTEDG_POS_EDGE  << AVR32_TC_EEVTEDG_OFFSET  |
                                        0                          << AVR32_TC_CPCDIS_OFFSET   |
                                        0                          << AVR32_TC_CPCSTOP_OFFSET  |
                                        AVR32_TC_BURST_NOT_GATED   << AVR32_TC_BURST_OFFSET    |
                                        1                          << AVR32_TC_CLKI_OFFSET     |
                                        AVR32_TC_TCCLKS_XC0        << AVR32_TC_TCCLKS_OFFSET;



      (&AVR32_TC)->channel[0].ra = 32;
      (&AVR32_TC)->channel[0].ccr = AVR32_TC_SWTRG_MASK | AVR32_TC_CLKEN_MASK;
}


int main(void)
{
  while(TRUE){
  //	Delay_ms(6000);
    Disable_global_interrupt();
    DontPanic = TRUE;
	//Force SSC_TX_DATA_ENABLE Disabled as soon as possible.
   AVR32_GPIO.port[1].ovrs  =  0x00000009;  //Value will be high.
   AVR32_GPIO.port[1].gpers =  0x00000009;  //Enable as GPIO.
   AVR32_GPIO.port[1].oders =  0x00000009;  //Output Driver will be Enabled.


        //       
	     //       Delay_ms(6000);
	      //        Delay_ms(6000);

    bunchofrandomstatusflags = 0x00000000;

	devflag=0;

    
    local_start_pll0();
    my_init_interrupts();

    //accelerometer_init();


    //Set up PB03 to watch FS.
    //Waits for radio to start making FSYNC.
      AVR32_GPIO.port[1].oderc = 0x00000002;
      AVR32_GPIO.port[1].gpers = 0x00000002;
      while ((AVR32_GPIO.port[1].pvr & 0x00000002) == 0); //Wait for FS High.
      while ((AVR32_GPIO.port[1].pvr & 0x00000002) != 0); //Wait for FS Low.

	 // 	delay(3000);
	//delay(3000);

    local_start_SSC();
    local_start_PDC();

    initXNL();

    //Start the SSC Physical Layer.
    (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->cr = AVR32_PDCA_TEN_MASK;
    (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCTX_EXAMPLE])->cr = AVR32_PDCA_TEN_MASK;
    (&AVR32_SSC)->cr = AVR32_SSC_CR_RXEN_MASK | AVR32_SSC_CR_TXEN_MASK;
    (&AVR32_PDCA.channel[PDCA_CHANNEL_SSCRX_EXAMPLE])->ier = AVR32_PDCA_RCZ_MASK;

    Enable_global_interrupt();

    while ((AVR32_GPIO.port[1].pvr & 0x00000002) == 0); //Wait for FS High.
    while ((AVR32_GPIO.port[1].pvr & 0x00000002) != 0); //Wait for FS Low.
    local_start_timer();

	U8 i;
    while(DontPanic){

    Delay_ms(100);
	
	//sendTONECTRLREQ();
	//if (0x00000003 == (bunchofrandomstatusflags & 0x00000003)){
	if (0x00000001 == (bunchofrandomstatusflags & 0x00000003)){
		//sendTONECTRLREQ();
		//nop();
		for (i=6;i<10; i++)
		{;
		}
	}
	
	if (0x00000003 == (bunchofrandomstatusflags & 0x00000003)){
		sendTONECTRLREQ();
		sendTONECTRLREQ();
	}
	
	
		
/*
//	 AVR32_GPIO.port[1].ovrc  =  0x00000008; 	
      process_XNL();
   //*   processAccelerometer();
      //Test if tilt is detected.

      
      if(0x00000040 == (bunchofrandomstatusflags & 0x00000040)){
          //Test if option board initialized and enabled.
    	  if (0x00000003 == (bunchofrandomstatusflags & 0x00000003)){
              if (sendTONECTRLREQ()){
    	         bunchofrandomstatusflags &= 0xFFFFFFBF;
    	      }
    	  }else{
    	      	 bunchofrandomstatusflags &= 0xFFFFFFBF;
    	  }
      }
      if (0x00000010 == (bunchofrandomstatusflags & 0x00000020)){
          processDoubleClick();
      }
	  */
    }
    //Code will get here, and re-start, upon Panic.
  }
}

const __int_handler interrupt_priority_handlers[4] =
{ &pdca_int_handler,
  &_external_interrupt,
  &_unhandled_interrupt,
  &_unhandled_interrupt
};

//A negative value will yield an unrecoverable exception.
const int priorityMapping[AVR32_INTC_NUM_INT_GRPS] =
{  -1,                      //Group  0 SYSBLOCK
    1,                      //Group  1 EIC, PM, RTC
   -1,                      //Group  2 GPIO
    0,                      //Group  3 DMA Controller
   -1,                      //Group  4 FLASH Controller
   -1,                      //Group  5 UART0
   -1,                      //Group  6 UART1
   -1,                      //Group  7 UART2
   -1,                      //Group  8 Unknown
   -1,                      //Group  9 SPI
   -1,                      //Group 10 Unknown
   -1,                      //Group 11 TWI
   -1,                      //Group 12 PWM
   -1,                      //Group 13 SSC
   -1,                      //Group 14 Timer/Counter
   -1,                      //Group 15 ADC
   -1,                      //Group 16 Unknown
   -1,                      //Group 17 USB
};


void Delay(unsigned long  Delay) 
{   
    unsigned long elasp=0;
    do 
    {
          elasp+=1;
		  process_XNL();
    }
    while (elasp< Delay);
 
}
   
void Delay_ms(unsigned long  Delay_ms) 
{   
    unsigned long elasp_ms=0;
    do 
    {
		Delay(6000); 

		  elasp_ms+=1;
    }
    while (elasp_ms< Delay_ms);
 
}


