#include "GOB.h"
#include "accelerometer.h"


#define LIS302DL_WHO_AM_I_MAGIC		0x3b

enum lis302dl_reg {
	LIS302DL_REG_WHO_AM_I		    = 0x0f,
	LIS302DL_REG_CTRL1		        = 0x20,
	LIS302DL_REG_CTRL2		        = 0x21,
	LIS302DL_REG_CTRL3		        = 0x22,
	LIS302DL_REG_HP_FILTER_RESET	= 0x23,
	LIS302DL_REG_STATUS		        = 0x27,
	LIS302DL_REG_OUT_X		        = 0x29,
	LIS302DL_REG_OUT_Y		        = 0x2b,
	LIS302DL_REG_OUT_Z		        = 0x2d,
	LIS302DL_REG_FF_WU_CFG_1	    = 0x30,
	LIS302DL_REG_FF_WU_SRC_1	    = 0x31,
	LIS302DL_REG_FF_WU_THS_1	    = 0x32,
	LIS302DL_REG_FF_WU_DURATION_1	= 0x33,
	LIS302DL_REG_FF_WU_CFG_2	    = 0x34,
	LIS302DL_REG_FF_WU_SRC_2	    = 0x35,
	LIS302DL_REG_FF_WU_THS_2	    = 0x36,
	LIS302DL_REG_FF_WU_DURATION_2	= 0x37,
	LIS302DL_REG_CLICK_CFG		    = 0x38,
	LIS302DL_REG_CLICK_SRC		    = 0x39,
	LIS302DL_REG_CLICK_THSY_X	    = 0x3b,
	LIS302DL_REG_CLICK_THSZ		    = 0x3c,
	LIS302DL_REG_CLICK_TIME_LIMIT	= 0x3d,
	LIS302DL_REG_CLICK_LATENCY	    = 0x3e,
	LIS302DL_REG_CLICK_WINDOW	    = 0x3f,
};

enum lis302dl_reg_ctrl1 {
	LIS302DL_CTRL1_Xen		= 0x01,
	LIS302DL_CTRL1_Yen		= 0x02,
	LIS302DL_CTRL1_Zen		= 0x04,
	LIS302DL_CTRL1_STM		= 0x08,
	LIS302DL_CTRL1_STP		= 0x10,
	LIS302DL_CTRL1_FS		= 0x20,
	LIS302DL_CTRL1_PD		= 0x40,
	LIS302DL_CTRL1_DR		= 0x80,
};

enum lis302dl_reg_ctrl3 {
	LIS302DL_CTRL3_PP_OD	= 0x40,
	LIS302DL_CTRL3_DATAONLY = 0x04,
	LIS302DL_CTRL3_CLICKandDATA = 0x3C,
};

enum lis302dl_reg_status {
	LIS302DL_STATUS_XDA		= 0x01,
	LIS302DL_STATUS_YDA		= 0x02,
	LIS302DL_STATUS_ZDA		= 0x04,
	LIS302DL_STATUS_XYZDA	= 0x08,
	LIS302DL_STATUS_XOR		= 0x10,
	LIS302DL_STATUS_YOR		= 0x20,
	LIS302DL_STATUS_ZOR		= 0x40,
	LIS302DL_STATUS_XYZOR	= 0x80,
};

enum lis302dl_reg_ffwusrc1 {
	LIS302DL_FFWUSRC1_XL		= 0x01,
	LIS302DL_FFWUSRC1_XH		= 0x02,
	LIS302DL_FFWUSRC1_YL		= 0x04,
	LIS302DL_FFWUSRC1_YH		= 0x08,
	LIS302DL_FFWUSRC1_ZL		= 0x10,
	LIS302DL_FFWUSRC1_ZH		= 0x20,
	LIS302DL_FFWUSRC1_IA		= 0x40,
};

enum lis302dl_reg_cloik_src {
	LIS302DL_CLICKSRC_SINGLE_X	= 0x01,
	LIS302DL_CLICKSRC_DOUBLE_X	= 0x02,
	LIS302DL_CLICKSRC_SINGLE_Y	= 0x04,
	LIS302DL_CLICKSRC_DOUBLE_Y	= 0x08,
	LIS302DL_CLICKSRC_SINGLE_Z	= 0x10,
	LIS302DL_CLICKSRC_DOUBLE_Z	= 0x20,
	LIS302DL_CLICKSRC_IA		= 0x40,
};



#define LIS302DL_F_WUP_FF		0x0001	/* wake up from free fall */
#define LIS302DL_F_WUP_CLICK	0x0002
#define LIS302DL_F_POWER		0x0010
#define LIS302DL_F_FS			0x0020 	/* ADC full scale */

/* lowlevel register access functions */

#define READ_BIT	0x01
#define MS_BIT		0x02
#define ADDR_SHIFT	2

#define ACC_ADDRESS        0x1D        // acc TWI address
#define TWI_SPEED         100000       // Speed of TWI
//#define ADDR_START
#define ADDR_LGT  1

#define READ_OPS  0x3B
#define WRITE_OPS 0x3A

#define POWER_ON_VAL 0x40
#define ACCREADY     0x47


const S32 filtercoffs[16] =
{
		133756,
		213239,
		435727,
		779334,
		1191587,
		1599781,
		1926767,
		2108413,
		2108413,
		1926767,
		1599781,
		1191587,
		779334,
		435727,
		213239,
		133756
};


//theAccStatus
#define ACC_ISINITIALIZED   0x8000     //Is Initialized.
#define ACC_GOODANGLEHIT    0x0001     //Good Angle Hit.
#define ACC_GOODANGLEMASK   0xFFFE     //Good Angle Mask.
#define ACC_MOTIONHIT       0x0002     //Good motion detected.
#define ACC_INITIALVALUE    0x8003
#define ACC_DECPERSEC       25         //25 decimated samples per second
#define ACC_TILTTHRESHOLD   102746112  //60 degree tilt threshold
struct ACCSTATUS {
	U16 theAccStatus;
	S8  seconds;
	S8  mS_40;
}accstatus;
extern   volatile U32 bunchofrandomstatusflags;
S32 filterresults[3];
S32 oldresults[3];
S32 accelerometerIndex;
S8 accsamples[16][4];

Bool wearenottilted(void)
{
	S32 dotproduct;

	dotproduct  = (oldresults[0]>>16)*(filterresults[0]>>16);
	dotproduct += (oldresults[1]>>16)*(filterresults[1]>>16);
	dotproduct += (oldresults[2]>>16)*(filterresults[2]>>16);
	if (ACC_TILTTHRESHOLD < dotproduct){
		return TRUE;    //We are not tilted.
	}else{
		return FALSE;   //We are tilted.
	}
}

void filter(U32 index)
{
	S32 j, k;
	filterresults[0] = 0;
	filterresults[1] = 0;
	filterresults[2] = 0;

	for (j=0;j<16;j++)
	{
		k = (index - j) & 0x0000000F;
		filterresults[0] += (accsamples[k][0])*filtercoffs[j];
		filterresults[1] += (accsamples[k][1])*filtercoffs[j];
		filterresults[2] += (accsamples[k][2])*filtercoffs[j];
	}
}


U32 my_writeabyte(U32 subaddress, U32 datatosend)
{
	U32 TWI_Status = 0;

	AVR32_TWI.cr =   AVR32_TWI_CR_MSEN_MASK | AVR32_TWI_CR_SVDIS_MASK;
	AVR32_TWI.mmr =  ACC_ADDRESS        << AVR32_TWI_MMR_DADR_OFFSET   |
	                 ADDR_LGT           << AVR32_TWI_MMR_IADRSZ_OFFSET |
	                 0                  << AVR32_TWI_MMR_MREAD_OFFSET;
	AVR32_TWI.iadr =  subaddress;

	AVR32_TWI.thr = datatosend;  //Higher order bits discarded.

	do
	{
	  TWI_Status =  AVR32_TWI.sr & 0x00000104;
	}
	while (TWI_Status == 0);
	while ((AVR32_TWI.sr & 0x00000001) == 0x00000000); //Wait for complete.
	return (TWI_Status);
}



U32 my_readabyte(U32 subaddress, S8 *datareceived)
{
	U32 TWI_Status = 0;

	AVR32_TWI.cr   =  AVR32_TWI_CR_MSEN_MASK | AVR32_TWI_CR_SVDIS_MASK;
	AVR32_TWI.mmr =  ACC_ADDRESS        << AVR32_TWI_MMR_DADR_OFFSET   |
	                 ADDR_LGT           << AVR32_TWI_MMR_IADRSZ_OFFSET |
	                 1                  << AVR32_TWI_MMR_MREAD_OFFSET;
	AVR32_TWI.iadr =  subaddress;

	AVR32_TWI.cr   =  AVR32_TWI_START_MASK | AVR32_TWI_STOP_MASK;

	do
	{
	  TWI_Status =  AVR32_TWI.sr & 0x00000102;
	}
	while (TWI_Status == 0);

	if (!(TWI_Status & 0x00000100))
	{
	  *datareceived = AVR32_TWI.rhr;
	}
	while ((AVR32_TWI.sr & 0x00000001) == 0x00000000); //Wait for complete.
	return (TWI_Status);
}

void accelerometer_init(void)
{
  U32 TWI_status;
  U32 temp;
  S8 data;

  accstatus.theAccStatus = 0x00;
  accstatus.seconds = 0;
  accstatus.mS_40 = ACC_DECPERSEC;
  accelerometerIndex = 0;
  for (TWI_status = 0; TWI_status < 16; TWI_status++){
	  for (temp = 0; temp < 4; temp++){
		  accsamples[TWI_status][temp] = 0;
	  }
  }

  //Allocate I/O's to TWI.
    //AVR32_TWI_SDA_0_0_PIN  port=0 pin=A mask=0x00000400
    //AVR32_TWI_SCL_0_0_PIN  port=0 pin=9 mask=0x00000200
    //AVR32_TWI_SCL_0_0_FUNCTION AVR32_TWI_SDA_0_0_FUNCTION functions = 0
    AVR32_GPIO.port[0].pmr0c  = 0x000000600;
    AVR32_GPIO.port[0].pmr1c  = 0x000000600;
    AVR32_GPIO.port[0].gperc  = 0x000000600;
  //  Accelerometer Outputs   GPIO Polling Inputs.
    //INERTIAL INT1           port=0 pin=5 mask= 0x00000020
    //INERTIAL INT2           port=0 pin=6 mask= 0x00000040
    AVR32_GPIO.port[0].oderc = 0x00000060;
    AVR32_GPIO.port[0].gpers = 0x00000060;



  //Init TWI.
	AVR32_TWI.cr   =  AVR32_TWI_CR_SWRST_MASK;  //resets TWI!
	AVR32_TWI.sr;                               // [shouldn't hurt]
	AVR32_TWI.cwgr =  0x0000ECEC;               //100KHz using 24Mhz PBA Clock.

  //For Test, read Who_am_I
  //TWI_status = my_readabyte(LIS302DL_REG_WHO_AM_I, &data);

  //Data Rate 100Hz, Power Active, Full Scale 2.3G, All Axis Enabled.
  TWI_status = my_writeabyte(LIS302DL_REG_CTRL1, ACCREADY);
  //Lines Active High, INT2 (temporarily) disabled, INT1 on Data Ready.
  TWI_status = my_writeabyte(LIS302DL_REG_CTRL3, LIS302DL_CTRL3_DATAONLY);


      //Discard dummy sample.
	  TWI_status = my_readabyte(LIS302DL_REG_OUT_X, &data);
	  TWI_status = my_readabyte(LIS302DL_REG_OUT_Y, &data);
	  TWI_status = my_readabyte(LIS302DL_REG_OUT_Z, &data);
}

	//The code reads the accelerometer status register (to clear the
	//hardware interrupt line), and then reads the three x, y, z samples into
	//a circular array. [The array is of dimension four for alignment.]
	//The filter is lowpass FIR for smoothing. The filter should remove "most"
	//of the fast motion variation, so the acceleration vector should be "mostly"
	//due to gravity. With an accelerometer gain of 0.018G/digit, 1G acceleration
	//should read a maximum of about 56 levels for an aligned axis. The filter
	//coefficients are scaled so that the filter output for an aligned axis
	//is a maximum of about 1G = 939523648. The dot product of two 1G aligned
	//vectors is then 205492225. A threshold of 102746112 is about a 60 degree
	//tilt.
void processAccelerometer(void)
{
	U32 TWI_status;

	if ((AVR32_GPIO.port[0].pvr & 0x00000020) != 0){//else, just leave.



  	TWI_status = my_readabyte(LIS302DL_REG_OUT_X, &accsamples[accelerometerIndex][0]);
  	TWI_status = my_readabyte(LIS302DL_REG_OUT_Y, &accsamples[accelerometerIndex][1]);
  	TWI_status = my_readabyte(LIS302DL_REG_OUT_Z, &accsamples[accelerometerIndex][2]);
    //This should clear INERTIAL INT1.


  	if (0x00000000 == (accelerometerIndex & 0x00000003)){ //Decimation
  		filter(accelerometerIndex);

  		accstatus.mS_40 -= 1;
  		if (ACC_ISINITIALIZED == (accstatus.theAccStatus & ACC_ISINITIALIZED)){
  			if (wearenottilted()){
  				//Any good angle during interval is a good interval.
  				accstatus.theAccStatus |= ACC_GOODANGLEHIT;
  			}

  			if (0 == accstatus.mS_40){

  				if (ACC_GOODANGLEHIT != (accstatus.theAccStatus & ACC_GOODANGLEHIT)){
  					//Angle change detected. Reset and let settle detector.
  					 accstatus.theAccStatus = 0x00;
  					//report angle change.
  					bunchofrandomstatusflags |= 0x00000040;
  				}else{
  					accstatus.theAccStatus &= ACC_GOODANGLEMASK;
  				}
  				accstatus.mS_40 = ACC_DECPERSEC;
  				accstatus.seconds += 1;
  			}

  		}else{
  			if (0 == accstatus.mS_40){//Then initialization interval complete.
  				accstatus.mS_40 = ACC_DECPERSEC;
  				accstatus.seconds = 0;
  				accstatus.theAccStatus = ACC_INITIALVALUE;
  				oldresults[0] = filterresults[0];
  				oldresults[1] = filterresults[1];
  				oldresults[2] = filterresults[2];
  			}
  		}
  	}

  	accelerometerIndex = (accelerometerIndex + 1) & 0x0000000F;
	}
}

void processDoubleClick(void)
{
	bunchofrandomstatusflags &= 0xFFFFFFDF;
}
