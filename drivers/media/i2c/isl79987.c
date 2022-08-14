
#define DEBUG

#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of_graph.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/v4l2-dv-timings.h>
#include <linux/printk.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-fwnode.h>


#define	bk_pr_err(fmt,args...) pr_err(fmt,##args)
#define	bk_pr_debug(fmt,args...) pr_debug(fmt,##args)

/**
 * I2C slave addresses on TW9992 board 
 */
#define ISL79987_I2C_IO   0x44 /* 0x88 on 8bit */
#define ISL79987_CHIP_ID  0x87


enum isl79987_pads {
	ISL79987_AFE_SINK0=0,
	ISL79987_AFE_SINK1,
	ISL79987_AFE_SINK2,
	ISL79987_AFE_SINK3,
	//ISL79987_CSI2_SOURCE0,
	//ISL79987_CSI2_SOURCE1,
	//ISL79987_CSI2_SOURCE2,
	//ISL79987_CSI2_SOURCE3,
	ISL79987_CSI2_SOURCE,
	ISL79987_NR_PADS,
};

#define ISL79987_PORT_MAX ISL79987_NR_PADS

/**
 * isl79987_state
 */

struct isl79987_state {
	struct device *dev;
	struct i2c_client *client;
    struct v4l2_subdev sd;
	struct v4l2_ctrl_handler ctrl_hdl;
	struct mutex mutex;

	struct device_node *endpoints[ISL79987_NR_PADS];
	struct media_pad pads[ISL79987_NR_PADS];

	struct v4l2_mbus_framefmt format;
	struct v4l2_fract aspect_ratio;
	struct v4l2_subdev_frame_interval fi;

	bool streaming;
	v4l2_std_id curr_norm;
	int afe_field;
	unsigned int csi2_input[5];
	unsigned int vc;
	int lanes;  /* MIPI Lanes. max 2 */
	int chs;	/* default: 4Channels */

    int is_dummy;
	int power;

	int pseudo;	/* PseudoFrame 0, 1, 2, 4 */
	int histo;	/* Histogram on PseudoFrame */
	//int nvc;	/* VC on PseudoFrame. use (state->vc & 0x03) */
};


#define isl79987_ctrl_to_state(ctrl) \
	container_of(ctrl->handler, struct isl79987_state, ctrl_hdl)
#define isl79987_sd_to_state(a) container_of(a, struct isl79987_state, sd)
#define isl79987_clinet_to_state(a) container_of(a, struct isl79987_state, client)

/*! @brief This mutex is used to provide mutual exclusion.
 *
 *  Create a mutex that can be used to provide mutually exclusive
 *  read/write access to the globally accessible data structures
 *  and variables that were defined above.
 */
static DEFINE_MUTEX(mutex);


/**
 * I2C read & write -------------------------------------------------
 *
 *          min		Max                                 Timeout
 *          ---     ---                                 -------
 * I2C		none	100k(STD),400k(Fast) 2M(HighSpeed)	none
 * SMBus	10kHz   100kHz                              35ms
 *
 * @see I2C speed is &i2c4 { ..clock-frequency = <400000>; ..} on salvator-xs.dtsi
 */

struct reg_value {
    u8 reg;
    u8 value;
};

static int WriteTW88(struct i2c_client *i2c_client, u8 reg, u8 val)
{
	s32 ret;
	unsigned char data[2] = { reg & 0xff, val };

	ret = i2c_master_send(i2c_client, data, 2);
	if (ret < 2) {
		dev_err(&i2c_client->dev, "%s: i2c write error, reg:0x%x ret:%d\n", __func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}
	return 0;
}

static int WriteTW88S(struct i2c_client *i2c_client, struct reg_value *config, int size)
{
    int i;
	int ret;

    for(i=0; i < size; i++) {
        ret = WriteTW88(i2c_client, config[i].reg, config[i].value);
		if(ret)
			return ret;
	}
    return 0;
}

static int ReadTW88(struct i2c_client *i2c_client, u8 reg, u8 *val)
{
	int ret;
	unsigned char data[1] = { reg & 0xff };

	ret = i2c_master_send(i2c_client, data, 1);
	if (ret < 1) {
		dev_err(&i2c_client->dev, "%s: i2c send error, reg:0x%x ret:%d\n",	__func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}
	ret = i2c_master_recv(i2c_client, val, 1);
	if (ret < 1) {
		dev_err(&i2c_client->dev, "%s: i2c recv error, reg:0x%x ret:%d\n", __func__, reg, ret);
		return ret < 0 ? ret : -EIO;
	}

	return 0;
}


/**
 * download tables
 *  ISL79987 major register description.
 *   R002[7] SW_RST auto clear
 *   R002[4] MIPI_RESET
 *   R002[3:0] En CH4..CH1
 *   R007[1:0] 0:1CH 1:2CHs 2:4CHs
 *   R1xx:CH1 R2xx:CH2 R3xx:CH3 R4xx:CH4 RFxx:All Channels 
 *   R500[7] MIPI_PowerDown
 *   R501[5] FrameMode
 *   R504[7:0] VC R504[7:6]:CH4 R504[5:4]:CH3 R504[3:2]:CH2 R504[1:0]:CH1 def:0xE4=11-10-01-00
 *   R506[0] PseudoFrameMode
 *   R50D[7:4] TestPattern
 *    
 *	 R1E7[0] En CH1 Histogram
 *	 R2E7[0] En CH2 Histogram
 *	 R3E7[0] En CH3 Histogram
 *	 R4E7[0] En CH4 Histogram
 *   R53C[2:0]R53D[7:0] Histogram Lines. 0x0F1:NTSC, 0x121:PAL 
 */

static struct reg_value isl79987_regs_default[] = {
{0xFF,0x00},
	{0x03,0x00},{0x0D,0xC9},{0x0E,0xC9}, 
	{0x10,0x01},{0x11,0x03},{0x12,0x00},{0x13,0x00},{0x14,0x00}, 
{0xFF,0x05},
	{0x00,0x02},{0x01,0x85},{0x02,0xa0},{0x03,0x18},
	{0x04,0xE4},{0x05,0x40},{0x06,0x40}, 
	{0x10,0x05},{0x11,0xa0},
	{0x20,0x00},{0x21,0x0C},{0x22,0x00},{0x23,0x00}, 
	{0x24,0x00},{0x25,0xF0},{0x26,0x00},{0x27,0x00},
	{0x2A,0x00},{0x2B,0x19},
	{0x2C,0x18},{0x2D,0xF1},{0x2E,0x00},{0x2F,0xF1},
	{0x30,0x00},{0x31,0x00},{0x32,0x00},{0x33,0xC0},
	{0x34,0x18},{0x35,0x00},{0x36,0x00}
};

static struct reg_value isl79987_decoder[]={
{0xff,0x0F},
	{0x2F,0xE6},{0x33,0x85},{0x3d,0x08},{0xE7,0x00}
};

static struct reg_value isl79987_regs_decoder_ntsc[]={
{0xFF,0x0F},
	{0x07,0x02},{0x08,0x14/*0x13*/},{0x09,0xf0},{0x0a,0x13/*0x0F*/},{0x0b,0xd0},
	{0x2F,0xE6},{0x33,0x85},{0x3d,0x08},{0xE7,0x00}
};

static struct reg_value isl79987_regs_mipi_ntsc[]={
{0xFF,0x05},
	{0x0F,0x80},
	{0x2C,0x18},{0x2D,0xF1},{0x2E,0x00},{0x2F,0xF1},{0x3C,0x00},{0x3D,0x1F}	
};

/* BKNote191204
	vDelay RF07[7:6]RF08[7:0] To crop the upper area, it needs 0x19. Def:0x12.
	If 0x19, Pseudo+Histo generates 0~287, and PIC_HEIGHT was 288
	If 0x18,                        0~288, and PIC_HEIGHT was 289
	If 0x16,                        0~288, and PIC_HEIGHT was 289
	So, assign 0x16 on Driver, Application needs to crop lines.
 */
struct reg_value isl79987_regs_decoder_pal[]={
{0xFF,0x0F},
	{0x07,0x12},{0x08,0x16},{0x09,0x20},{0x0a,0x0D/*0x08*/},{0x0b,0xd0},
	{0x2F,0xE6},{0x33,0x85},{0x3d,0x08},{0xE7,0x00}
};

struct reg_value isl79987_regs_mipi_pal[]={
{0xFF,0x05},
	{0x0F,0x84},
	{0x2C,0x19},{0x2D,0x21},{0x2E,0x01},{0x2F,0x21},{0x3C,0x01},{0x3D,0x21}	
};  

static struct reg_value isl79987_4Ch2Lane[]={
{0xFF,0x00},
	{0x07,0x12},{0x08,0x1f},{0x09,0x43},{0x0A,0x4f},{0x0B,0x41},
{0xFF,0x05},
	{0x00,0x02},{0x01,0x05},{0x02,0xA0},{0x03,0x10},
	{0x04,0xE4},{0x05,0x00},{0x06,0x00},{0x07,0x2B},
	{0x08,0x02},{0x09,0x00},{0x0A,0x62},{0x0B,0x02},
	{0x0C,0x36},{0x0D,0x00},{0x0E,0x6C},{0x0F,0x80},
	{0x10,0x05},{0x11,0xA0},{0x12,0x77},{0x13,0x17},
	{0x14,0x08},{0x15,0x38},{0x16,0x14},{0x17,0xF6},
	{0x18,0x00},{0x19,0x17},{0x1A,0x0A},{0x1B,0x71},
	{0x1C,0x7A},{0x1D,0x0F},{0x1E,0x8C},
	{0x23,0x0A},{0x26,0x08},
	{0x28,0x01},{0x29,0x0E},{0x2A,0x00},{0x2B,0x00},
	{0x38,0x03},{0x39,0xC0},{0x3A,0x06},{0x3B,0xB3},
	{0x3C,0x00},{0x3D,0xF1}
};

struct reg_value isl79987_4Ch1Lane[]={
{0xFF,0x0},
	{0x07,0x12},{0x08,0x1f},{0x09,0x43},{0x0A,0x4f},{0x0B,0x40},
{0xFF,0x5},
	{0x00,0x01},{0x01,0x05},{0x02,0xA0},{0x03,0x10},
    {0x04,0xE4},{0x05,0x00},{0x06,0x00},{0x07,0x2B},
    {0x08,0x00},{0x09,0x00},{0x0A,0x62},{0x0B,0x02},
    {0x0C,0x36},{0x0D,0x00},{0x0E,0x6C},{0x0F,0x80},
    {0x10,0x05},{0x11,0xA0},{0x12,0x78},{0x13,0x17},
    {0x14,0x07},{0x15,0x36},{0x16,0x10},{0x17,0xF6},
    {0x18,0x00},{0x19,0x17},{0x1A,0x0A},{0x1B,0x71},
    {0x1C,0x7A},{0x1D,0x0F},{0x1E,0x8C},
    {0x23,0x0A},{0x26,0x07},
    {0x28,0x01},{0x29,0x0E},{0x2A,0x00},{0x2B,0x00},
    {0x38,0x03},{0x39,0xC0},{0x3A,0x06},{0x3B,0xB3},
    {0x3C,0x00},{0x3D,0xF1}
};

struct reg_value isl79987_2Ch2Lane[]={
{0xFF,0x0},
	{0x07,0x11},{0x08,0x1f},{0x09,0x47},{0x0A,0x4f},{0x0B,0x42},
{0xFF,0x5},
	{0x00,0x02},{0x01,0x05},{0x02,0xA0},{0x03,0x10},
	{0x04,0xE4},{0x05,0x00},{0x06,0x00},{0x07,0x24},
	{0x08,0x02},{0x09,0x00},{0x0A,0x62},{0x0B,0x02},
	{0x0C,0x36},{0x0D,0x00},{0x0E,0x36},{0x0F,0x80},
	{0x10,0x05},{0x11,0xA0},{0x12,0x34},{0x13,0x0F},
	{0x14,0x06},{0x15,0x24},{0x16,0x11},{0x17,0x70},
	{0x18,0x00},{0x19,0x17},{0x1A,0x06},{0x1B,0x31},
	{0x1C,0x46},{0x1D,0x08},{0x1E,0x57},
	{0x23,0x06},{0x26,0x06},
	{0x28,0x01},{0x29,0x69},{0x2A,0x00},{0x2B,0x00},
	{0x38,0x01},{0x39,0xE0},{0x3A,0x06},{0x3B,0xB3},
	{0x3C,0x00},{0x3D,0xF1}
};

struct reg_value isl79987_2Ch1Lane[]={
{0xFF,0x0},
	{0x07,0x11},{0x08,0x1f},{0x09,0x47},{0x0A,0x4f},{0x0B,0x41},
{0xFF,0x5},
	{0x00,0x01},{0x01,0x05},{0x02,0xA0},{0x03,0x10},
	{0x04,0xE4},{0x05,0x00},{0x06,0x00},{0x07,0x1B},
	{0x08,0x02},{0x09,0x00},{0x0A,0x62},{0x0B,0x02},
	{0x0C,0x36},{0x0D,0x00},{0x0E,0x36},{0x0F,0x80},
	{0x10,0x05},{0x11,0xA0},{0x12,0x34},{0x13,0x07},
	{0x14,0x02},{0x15,0x1E},{0x16,0x0A},{0x17,0x70},
	{0x18,0x00},{0x19,0x17},{0x1A,0x06},{0x1B,0x31},
	{0x1C,0x43},{0x1D,0x08},{0x1E,0x77},
	{0x23,0x03},{0x26,0x02},
	{0x28,0x00},{0x29,0xB4},{0x2A,0x00},{0x2B,0x00},
	{0x38,0x01},{0x39,0xE0},{0x3A,0x06},{0x3B,0xB3},
	{0x3C,0x00},{0x3D,0xF1}
};

struct reg_value isl79987_1Ch1Lane[]={
{0xFF,0x0},
	{0x07,0x00},{0x08,0x1f},{0x09,0x4f},{0x0A,0x4f},{0x0B,0x42},
{0xFF,0x5},
	{0x00,0x01},{0x01,0x05},{0x02,0xA0},{0x03,0x10},
	{0x04,0xE4},{0x05,0x00},{0x06,0x00},{0x07,0x17},
	{0x08,0x00},{0x09,0x00},{0x0A,0x62},{0x0B,0x02},
	{0x0C,0x36},{0x0D,0x00},{0x0E,0x1B},{0x0F,0x80},
	{0x10,0x05},{0x11,0xA0},{0x12,0x12},{0x13,0x05},
	{0x14,0x02},{0x15,0x0E},{0x16,0x08},{0x17,0x37},
	{0x18,0x00},{0x19,0x00},{0x1A,0x02},{0x1B,0x11},
	{0x1C,0x22},{0x1D,0x03},{0x1E,0x22},
	{0x23,0x02},{0x26,0x02},
	{0x28,0x01},{0x29,0x0E},{0x2A,0x00},{0x2B,0x00},
	{0x38,0x00},{0x39,0xF0},{0x3A,0x06},{0x3B,0xB3},
	{0x3C,0x00},{0x3D,0xF1}
};

//./v4l2_test -9987 4 2 –nonstd 4 0 –demux 3
//struct reg_value isl79987_4Ch2Lane_PseudoFrame[] = {
//}
//./v4l2_test -9987 4 2 –nonstd 4 1 –demux 3
//struct reg_value isl79987_4Ch2Lane_PseudoFrame_Histogram[] = {
//}



/**
 *
 * @see R000[] CHIP_ID 0x87
 * @see R001[] REV     0x01
 */
static int isl79987_check_chip_id(struct isl79987_state *state)
{
    struct i2c_client *client = state->client;
    u8 id, rev;

    if(state->is_dummy) {
        pr_info("Renesas isl79987 dummy driver\n");
        return 0;
    }
	WriteTW88(client, 0xFF, 0);
    ReadTW88(client, 0x00, &id);
    if (id != ISL79987_CHIP_ID) {
        pr_err("isl79987 is not found, chip id reg 0x00 = 0x%x.\n", id);
        pr_err(" ***enable dummy to debug***\n");
        state->is_dummy=1;
		return 0; /* or -ENODEV */
    }

    ReadTW88(client, 0x01, &rev);
    pr_info("Renesas isl79987 id:%2X rev:%2X is found.\n",id, rev);     

    return 0;
}

/**
 *
 * @see R002[7] SW_RST auto clear 
 */
static int isl79987_SW_reset(struct isl79987_state *state)
{
    struct i2c_client *client = state->client;
	u8 val;

    if(state->is_dummy) {
        return 0;
	}

	WriteTW88(client, 0xff, 0);
	ReadTW88(client,0x02, &val);
    WriteTW88(client,0x02, val | 0x80);
    return 0; 
}

/**
 *
 * @see R002[4] MIPI_RESET
 * @see R002[3:0] En CH3..CH0
 */
#define MIPI_RESET_CLEAR		0
#define MIPI_RESET_SET			1
#define MIPI_RESET_CH_CLEAR		2
#define MIPI_RESET_ALL_CLEAR	3
#define MIPI_RESET_ALL_SET		4
static int isl79987_mipi_reset(struct isl79987_state *state, int flag_set)
{
    struct i2c_client *client = state->client;
    u8 bTemp;

    if(state->is_dummy)
        return 0;

    WriteTW88(client, 0xff, 0); 
    ReadTW88(client, 0x02, &bTemp);
	switch(flag_set) {
	case MIPI_RESET_CLEAR:		bTemp &= ~0x10; break;
	case MIPI_RESET_SET:		bTemp |=  0x10; break;
	case MIPI_RESET_CH_CLEAR:	bTemp &= ~0x0F; break;
	case MIPI_RESET_ALL_CLEAR:	bTemp &= ~0x1F; break;
	case MIPI_RESET_ALL_SET:	bTemp |=  0x1F; break;
	}
    WriteTW88(client, 0x02, bTemp); 

    return 0; 
}

#define ISL79987_AFE_STATUS_REG	0x03
	#define ISL79987_AFE_STATUS_VDLOSS	0x80	/*Video Loss */
	#define ISL79987_AFE_STATUS_FIELD	0x10	/*EVEN Field*/
	#define ISL79987_AFE_STATUS_DET50	0x01	/*50Hz source*/
	#define ISL79987_AFE_STATUS_LOCK	0x68	/*[6]HLOCK [5]SLOCK 3[VLOCK]*/
	#define ISL79987_AFE_STATUS_NOSIGNAL_MASK (ISL79987_AFE_STATUS_VDLOSS | ISL79987_AFE_STATUS_LOCK)

#define ISL79987_DEC_STD_REG	0x1C
	#define ISL79987_AFE_STD_NTSC     0x00
	#define ISL79987_AFE_STD_PAL      0x10
	#define ISL79987_AFE_STD_SECAM    0x20
	#define ISL79987_AFE_STD_NTSC4    0x30
	#define ISL79987_AFE_STD_PAL_M    0x40 
	#define ISL79987_AFE_STD_PAL_CN   0x50 
	#define ISL79987_AFE_STD_PAL_60   0x60 
	#define ISL79987_AFE_STD_INVALID  0x70 
	#define ISL79987_AFE_STD_SET_AUTO 7
/**
 *
 * @see
 *
 */
static int isl79987_afe_status(struct isl79987_state *state, int pad, u32 *signal, v4l2_std_id *std)
{
    struct i2c_client *client = state->client;
    u8 tmp;

    if(state->is_dummy) {
        if(signal) {
            *signal = 0;
		}
	    if (std) {
            *std = V4L2_STD_NTSC;
		}
        return 0;
    }
	switch(pad) {
	case ISL79987_AFE_SINK0: 	WriteTW88(client, 0xFF, 0x01); break;
	case ISL79987_AFE_SINK1:	WriteTW88(client, 0xFF, 0x02); break;
	case ISL79987_AFE_SINK2:	WriteTW88(client, 0xFF, 0x03); break;
	case ISL79987_AFE_SINK3:	WriteTW88(client, 0xFF, 0x04); break;
	default: 					WriteTW88(client, 0xFF, 0x0F); break;
	}
	ReadTW88(client, ISL79987_AFE_STATUS_REG, &tmp);
    if((tmp & ISL79987_AFE_STATUS_NOSIGNAL_MASK) 
		!= ISL79987_AFE_STATUS_LOCK) {
		if(signal)
			*signal = V4L2_IN_ST_NO_SIGNAL;
        pr_debug("%s NO_SIGNAL\n",__func__);
		if (std) {
			*std = V4L2_STD_UNKNOWN;
			return 0;
		}			
	}
	else {
		if(signal)
	        *signal = 0;
	}
	if (!std)
		return 0;

    ReadTW88(client, ISL79987_DEC_STD_REG, &tmp);
    tmp &= 0x70;
    if(tmp == ISL79987_AFE_STD_PAL 
    || tmp == ISL79987_AFE_STD_SECAM 
    || tmp == ISL79987_AFE_STD_PAL_CN) {
        pr_debug("%s PAL\n",__func__);
        *std = V4L2_STD_PAL;
    }
    else if(tmp == 0x70) {
        pr_debug("%s InvalidSTD\n",__func__);         
        *std = V4L2_STD_UNKNOWN;
    } else {
        pr_debug("%s NTSC\n",__func__);
        *std = V4L2_STD_NTSC;
    }

    return 0;
}

/**
 *
 * @see
 *
 */
static void isl79987_wait_field(struct isl79987_state *state, int f)
{
	struct i2c_client *client = state->client;
	volatile u8 tmp;

    if(state->is_dummy) {
        return;
	}

	/* co-work with Decoder1 */
	WriteTW88(client, 0xFF, 1);
	if(f) {
		while(1) {
        	ReadTW88(client, ISL79987_AFE_STATUS_REG, (u8 *)&tmp);
			if(tmp & ISL79987_AFE_STATUS_VDLOSS)
				return;
			if((tmp & ISL79987_AFE_STATUS_FIELD) ==0)
				break;			
		}
		while(1) {
        	ReadTW88(client, ISL79987_AFE_STATUS_REG, (u8 *)&tmp);
			if(tmp & ISL79987_AFE_STATUS_VDLOSS)
				return;
			if(tmp & ISL79987_AFE_STATUS_FIELD)
				break;			
		}
	}
	else {
		while(1) {
        	ReadTW88(client, ISL79987_AFE_STATUS_REG, (u8 *)&tmp);
			if(tmp & ISL79987_AFE_STATUS_VDLOSS)
				return;
			if(tmp & ISL79987_AFE_STATUS_FIELD)
				break;			
		}
		while(1) {
        	ReadTW88(client, ISL79987_AFE_STATUS_REG, (u8 *)&tmp);
			if(tmp & ISL79987_AFE_STATUS_VDLOSS)
				return;
			if((tmp & ISL79987_AFE_STATUS_FIELD) ==0)
				break;			
		}
	}
}

/**
 *
 * @see R500[7]
 */
static int isl79987_mipi_power(struct isl79987_state *state, bool on)
{
    struct i2c_client *client = state->client;
    u8 reg;

    if(state->is_dummy) {
        return 0;
	}

	WriteTW88(client, 0xff, 5);
    ReadTW88(client, 0x00, &reg);
    if(on) reg &= ~0x80;
    else   reg |=  0x80;
    WriteTW88(client, 0x00, reg);

    return 0;
}

/**
 *
 * @see R504[7:6] VC3
 * @see R504[5:4] VC2
 * @see R504[3:2] VC1
 * @see R504[1:0] VC0
 */
static int isl79987_csi2_set_virtual_channel(struct isl79987_state *state,
					    unsigned int vc)
{
    struct i2c_client *client = state->client;

    if(state->is_dummy)
    {
        return 0;
    }

	WriteTW88(client, 0xff, 5);
	WriteTW88(client, 0x04, (u8)vc);

    return 0;
}

/**
 * Control TestPattern
 * @see R50D[7] Ch1 TestPattern
 * @see R50D[6] Ch2 TestPattern
 * @see R50D[5] Ch3 TestPattern
 * @see R50D[4] Ch4 TestPattern
 */
static void isl79987_csi2_enable_TestPattern(struct isl79987_state *state, u32 pad, int fOn)
{
	struct i2c_client *client = state->client;
	u8 tmp;
	u8 channel;

	switch(pad) {
	case ISL79987_AFE_SINK0:	channel = 0x80; break;			
	case ISL79987_AFE_SINK1:	channel = 0x40; break;
	case ISL79987_AFE_SINK2:	channel = 0x20; break;
	case ISL79987_AFE_SINK3:	channel = 0x10; break;
	default:					channel = 0xF0; break;
	}

	WriteTW88(client, 0xFF, 5);
	ReadTW88(client, 0x0D, &tmp);
	if(fOn) tmp |= channel;
	else    tmp &= ~channel;
	WriteTW88(client, 0x0D, tmp);
}


/**
 * download chip registers
 * 
 * @param state->curr_norm
 *		isl79987_regs_decoder_ntsc, isl79987_regs_mipi_ntsc
 *		isl79987_regs_decoder_pal, isl79987_regs_mipi_pal
 * @param state->chs
 * @param state->lanes
 *    isl79987_4Ch2Lane
 *    isl79987_4Ch1Lane
 *    isl79987_2Ch2Lane
 *    isl79987_2Ch1Lane
 *    isl79987_1Ch1Lane
 */
static int isl79987_download_chip_registers(struct isl79987_state *state)
{
	struct i2c_client *client = state->client;
	int lines;
	u8 tmp;

    if(state->is_dummy) {
        return 0;
	}

    pr_debug("download isl79987 regs. %s %s%s\n",
		state->curr_norm & V4L2_STD_525_60 ? "NTSC":"PAL",
		state->chs==1 ? "1CH" : state->chs==2 ? "2CHs" : "4CHs", 
		state->lanes == 1 ? "1Lane" : "2Lanes");
	if(state->chs==1 && state->lanes == 2)
		bk_pr_err("Err %s state->chs==1 && state->lanes==2\n",__func__);

	isl79987_mipi_reset(state, MIPI_RESET_ALL_SET);               
    WriteTW88S(client, isl79987_regs_default, ARRAY_SIZE(isl79987_regs_default));
	isl79987_mipi_reset(state, MIPI_RESET_CH_CLEAR); 

    WriteTW88S(client, isl79987_decoder, ARRAY_SIZE(isl79987_decoder)); 
	if(state->curr_norm & V4L2_STD_525_60)
	    WriteTW88S(client, isl79987_regs_decoder_ntsc, ARRAY_SIZE(isl79987_regs_decoder_ntsc)); 
	else
	    WriteTW88S(client, isl79987_regs_decoder_pal, ARRAY_SIZE(isl79987_regs_decoder_pal));

	if(state->chs==1) {
		WriteTW88S(client, isl79987_1Ch1Lane, ARRAY_SIZE(isl79987_1Ch1Lane)); 
	}
	else if(state->chs==2) {
		if(state->lanes == 1)
			WriteTW88S(client, isl79987_2Ch1Lane, ARRAY_SIZE(isl79987_2Ch1Lane)); 
		else
			WriteTW88S(client, isl79987_2Ch2Lane, ARRAY_SIZE(isl79987_2Ch2Lane)); 
	}
	else {
		if(state->lanes == 1)
			WriteTW88S(client, isl79987_4Ch1Lane, ARRAY_SIZE(isl79987_4Ch1Lane)); 
		else
			WriteTW88S(client, isl79987_4Ch2Lane, ARRAY_SIZE(isl79987_4Ch2Lane)); 
	}
	if(state->vc) {
		isl79987_csi2_set_virtual_channel(state, state->vc);
	}
	if(state->curr_norm & V4L2_STD_525_60)
	    WriteTW88S(client, isl79987_regs_mipi_ntsc, ARRAY_SIZE(isl79987_regs_mipi_ntsc)); 
	else
	    WriteTW88S(client, isl79987_regs_mipi_pal, ARRAY_SIZE(isl79987_regs_mipi_pal));

#if 0
state->pseudo		En PseudoFrame.	0:Disable [1,2,4]
state->histo		En Histogram when it is PseudoFrame mode
state->curr_norm	V4L2_STD_PAL or V4L2_STD_NTSC
//state->nvc			virtual channel on PseudoFrame mode

R501[5]=0	Disable FrameMode
RFE7[0]=1	En Histogram for all 4channels
R506[6]=1	FIX_LNO
R506[5]=1	En 8HDR
R506[0]=1	En PseudoFrame

R504[7:6]=	VC4
R504[5:4]=	VC3
R504[3:2]=	VC2
R504[1:0]=	VC1

R538[]R539[] = Total PseudoFrame lines
#endif	
	if(state->pseudo) {
		/* first, remove FrameMode; REG501[5] */
        WriteTW88(client, 0xFF, 5);
		ReadTW88(client, 0x01, &tmp);
		WriteTW88(client, 0x01, tmp & ~0x20);

        lines = state->pseudo;
        if(state->curr_norm==V4L2_STD_PAL)	lines *= 288;
        else                                lines *= 240;
        if(state->histo) { /* ?1 */
            lines += state->pseudo;
            WriteTW88(client, 0xff, 0xf);
            WriteTW88(client, 0xE7, 0x01);
        }

        WriteTW88(client, 0xFF, 5);
        WriteTW88(client, 0x06, 0x61);
        switch(state->vc & 0x03) {
        case 0:
            WriteTW88(client, 0x04, 0x00); /* or use REG506[3]=1 */
            break;
        case 1:
            WriteTW88(client, 0x04, 0x55);
            break;
        case 2:
            WriteTW88(client, 0x04, 0xAA);
            break;
        case 3:
            WriteTW88(client, 0x04, 0xFF);
            break;
        default:
            WriteTW88(client, 0x04, 0x00);
            break;
        }

        WriteTW88(client, 0x38, lines >> 8);
        WriteTW88(client, 0x39, lines);        

	}

	isl79987_mipi_reset(state, MIPI_RESET_CLEAR); 

	return 0;
}

/**
 * client NONE.
 */
static int isl79987_initialise_clients(struct isl79987_state *state)
{
	return 0;
}

static void isl79987_unregister_clients(struct isl79987_state *state)
{
	return;
}

static int isl79987_afe_std(v4l2_std_id std) 
{
	if (std == V4L2_STD_PAL_60)
		return ISL79987_AFE_STD_PAL_60;
	if (std == V4L2_STD_NTSC_443)
		return ISL79987_AFE_STD_NTSC4;
	if (std == V4L2_STD_PAL_N)
		return ISL79987_AFE_STD_PAL;
	if (std == V4L2_STD_PAL_M)
		return ISL79987_AFE_STD_PAL_M;
	if (std == V4L2_STD_PAL_Nc)
		return ISL79987_AFE_STD_PAL_CN;
	if (std & V4L2_STD_NTSC)
		return ISL79987_AFE_STD_NTSC;
	if (std & V4L2_STD_PAL)
		return ISL79987_AFE_STD_PAL;
	if (std & V4L2_STD_SECAM)
		return ISL79987_AFE_STD_SECAM;

	return -EINVAL;
}

/**
 * ==============================================================================
 * V4L2 Interface
 * ==============================================================================
 */

/** 
 * -----------------------------------------------------------------------------
 * v4l2_subdev_internal_ops
 * -----------------------------------------------------------------------------
 */

/**
 * @see registered 
 */
static int isl79987_csi2_registered(struct v4l2_subdev *sd)
{
#if 0
//	struct isl79987_state *state = isl79987_sd_to_state(sd);
//	struct v4l2_device *v4l2_dev;
//	struct v4l2_subdev *src;
//	int ret;
//	int enabled;
//	int i;
#endif

	bk_pr_debug("%s Registered %s (%s)", __func__, "TX", sd->name);
#if 0
	v4l2_dev = sd->v4l2_dev;
	src = &state->afe.sd;
	enabled = MEDIA_LNK_FL_ENABLED;	
	enabled |= MEDIA_LNK_FL_NOT_RVIN;
	if (!src->v4l2_dev) {
		ret = v4l2_device_register_subdev(v4l2_dev, src);
		if (ret) {
			pr_err("%s err 1\n",__func__);
			return ret;
		}
	}
	for(i=0; i < 4; i++) {
		ret= media_create_pad_link(
			&src->entity, ISL79987_AFE_SINK0+i,
			&tx->sd.entity, ISL79987_CSI2_SOURCE0+i,
			enabled);
		if(ret) {
			pr_err("%s err %d\n",__func__,2+i);
			return ret;
		}
	}	
#endif	
	return 0;
}

static const struct v4l2_subdev_internal_ops isl79987_csi2_internal_ops = {
	.registered = isl79987_csi2_registered,
};

/** 
 * ------------------------------------------------------
 * Subdev module and controls
 * ------------------------------------------------------
 */

/**
 * set pixelrate for RCAR_CSI2
 */
int isl79987_csi2_set_pixelrate(struct v4l2_subdev *sd, s64 rate)
{
	struct v4l2_ctrl *ctrl;

	pr_debug("%s rate:%lld\n", __func__,rate);

	ctrl = v4l2_ctrl_find(sd->ctrl_handler, V4L2_CID_PIXEL_RATE);
	if (!ctrl)
		return -EINVAL;

	return v4l2_ctrl_s_ctrl_int64(ctrl, rate);
}

static const char * const ctrl_frp_menu[] = {
	"Disabled",
	"TestPattern"
};

/* Contrast Control */
#define ISL79987_IA_CONTRAST_Y    0x11    /* Contrast REG111 */
#define ISL79987_IA_CON_MIN	      0       /* Minimum contrast */
#define ISL79987_IA_CON_DEF	      128     /* Default */
#define ISL79987_IA_CON_MAX	      255     /* Maximum contrast */

/* Saturation Control */
#define ISL79987_IA_CONTRAST_CB   0x13    /* Saturation Cb REG113 */
#define ISL79987_IA_CONTRAST_CR   0x14    /* Saturation Cr REG114 */
#define ISL79987_IA_SAT_MIN       0       /* Minimum saturation */
#define ISL79987_IA_SAT_DEF       128     /* Default */
#define ISL79987_IA_SAT_MAX       255     /* Maximum saturation */

/* Brightness Control */
#define ISL79987_IA_BRIGHTNESS_Y  0x10    /* Brightness REG110 */
#define ISL79987_IA_BRI_MIN       -128    /* Luma is -512d */
#define ISL79987_IA_BRI_DEF       0       /* Luma is 0 */
#define ISL79987_IA_BRI_MAX       127     /* Luma is 508d */

/* Hue Control */
#define ISL79987_IA_HUE           0x15    /* Hue REG115 */
#define ISL79987_IA_HUE_MIN	      0       /* -90 degree */
#define ISL79987_IA_HUE_DEF	      32      /*   0 degree */
#define ISL79987_IA_HUE_MAX	      63      /* +90 degree */

/* Sharpness */
#define ISL79987_IA_SHARPNESS     0x12    /* Sharpness REG112 */
#define ISL79987_IA_SHARP_MIN     0
#define ISL79987_IA_SHARP_DEF     1
#define ISL79987_IA_SHARP_MAX     15

/**
 * control BRIGHTNESS,CONTRAST,SATURATION,HUE and TestPattern.
 * @see s_ctrl 
 * command:
 *	v4l2-ctl -d /dev/v4l-subdev1 -l
 *	v4l2-ctl -d /dev/v4l-subdev1 --set-ctrl test_pattern=1
 */
static int isl79987_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct isl79987_state *state = isl79987_ctrl_to_state(ctrl);
    struct i2c_client *client = state->client;
    u8 tmp;

	if(ctrl->id == V4L2_CID_PIXEL_RATE)
		return 0;

    if(state->is_dummy)
        return 0;
	
    switch (ctrl->id) {
    case V4L2_CID_BRIGHTNESS:
		WriteTW88(client, 0xFF, 0x0F);
        WriteTW88(client, (u8)ISL79987_IA_BRIGHTNESS_Y,ctrl->val);
        break;
    case V4L2_CID_CONTRAST:
		WriteTW88(client, 0xFF, 0x0F);
        WriteTW88(client, (u8)ISL79987_IA_CONTRAST_Y,ctrl->val);
        break;
    case V4L2_CID_SATURATION:
		WriteTW88(client, 0xFF, 0x0F);
        WriteTW88(client, (u8)ISL79987_IA_CONTRAST_CB,ctrl->val);
        WriteTW88(client, (u8)ISL79987_IA_CONTRAST_CR,ctrl->val);
        break;
    case V4L2_CID_HUE:
        tmp = ctrl->val;
        							 //convert SW to HW.
        if(tmp==32)       tmp = 32;  //32    =>32
        else if(tmp < 32) tmp += 33; //0..31 =>33..63
        else              tmp -= 33; //33..63=>0..31 
		WriteTW88(client, 0xFF, 1);
        WriteTW88(client, (u8)ISL79987_IA_HUE, tmp);
        break;
    case V4L2_CID_BLACK_LEVEL:
		WriteTW88(client, 0xFF, 0x0F);
        ReadTW88(client, 0x0C, &tmp);
        if(ctrl->val) tmp |= 0x10;
        else            tmp &= ~0x10;
        WriteTW88(client, 0x0C, tmp);
        break;
    case V4L2_CID_AUTO_WHITE_BALANCE:
		WriteTW88(client, 0xFF, 0x0F);
        ReadTW88(client, 0x80, &tmp);
        if(ctrl->val) tmp |= 0x81;
        else    tmp &= ~0x81;
        WriteTW88(client, 0x80, tmp);
        break;
    case V4L2_CID_SHARPNESS:
		WriteTW88(client, 0xFF, 0x0F);
        ReadTW88(client, 0x12, &tmp);
        tmp &= 0xF0;
        tmp |= ctrl->val;
        WriteTW88(client, 0x12,tmp);
        break;
    case V4L2_CID_TEST_PATTERN:
		isl79987_csi2_enable_TestPattern(state, 0, ctrl->val);
		isl79987_csi2_enable_TestPattern(state, 1, ctrl->val);
		isl79987_csi2_enable_TestPattern(state, 2, ctrl->val);
		isl79987_csi2_enable_TestPattern(state, 3, ctrl->val);
        break;
    case V4L2_CID_DO_WHITE_BALANCE:
    case V4L2_CID_RED_BALANCE:
    case V4L2_CID_BLUE_BALANCE:
    case V4L2_CID_GAMMA:    
    case V4L2_CID_EXPOSURE: 
    case V4L2_CID_AUTOGAIN: 
    case V4L2_CID_GAIN:
    case V4L2_CID_HFLIP:
    case V4L2_CID_VFLIP:
    default:
        pr_debug("%s unknown id:%x\n",__func__,ctrl->id);
        return -EINVAL;
    }

    return 0;
}

static const struct v4l2_ctrl_ops isl79987_ctrl_ops = {
	.s_ctrl = isl79987_s_ctrl,
};

static int isl79987_init_controls(struct isl79987_state *state)
{
    int ret;

	v4l2_ctrl_handler_init(&state->ctrl_hdl, 6);

	v4l2_ctrl_new_std(&state->ctrl_hdl, &isl79987_ctrl_ops,
			  V4L2_CID_PIXEL_RATE, 1, INT_MAX, 1, 1);

	state->ctrl_hdl.lock = &state->mutex;

    v4l2_ctrl_new_std(&state->ctrl_hdl, &isl79987_ctrl_ops,
	    V4L2_CID_BRIGHTNESS, 
        ISL79987_IA_BRI_MIN, ISL79987_IA_BRI_MAX, 
        1, ISL79987_IA_BRI_DEF);
    v4l2_ctrl_new_std(&state->ctrl_hdl, &isl79987_ctrl_ops,
        V4L2_CID_CONTRAST, 
        ISL79987_IA_CON_MIN, ISL79987_IA_CON_MAX, 
        1, ISL79987_IA_CON_DEF);
    v4l2_ctrl_new_std(&state->ctrl_hdl, &isl79987_ctrl_ops,
        V4L2_CID_SATURATION, 
        ISL79987_IA_SAT_MIN, ISL79987_IA_SAT_MAX, 
        1, ISL79987_IA_SAT_DEF);
    v4l2_ctrl_new_std(&state->ctrl_hdl, &isl79987_ctrl_ops,
        V4L2_CID_HUE, 
        ISL79987_IA_HUE_MIN, ISL79987_IA_HUE_MAX, 
        1, ISL79987_IA_HUE_DEF);

	v4l2_ctrl_new_std_menu_items(&state->ctrl_hdl, &isl79987_ctrl_ops,
		V4L2_CID_TEST_PATTERN,
		ARRAY_SIZE(ctrl_frp_menu) - 1,
		0, 0, ctrl_frp_menu);

	state->sd.ctrl_handler = &state->ctrl_hdl;
	if (state->ctrl_hdl.error) {
		v4l2_ctrl_handler_free(&state->ctrl_hdl);
        pr_err("Err state->ctrl_hdl.error\n");
		return state->ctrl_hdl.error;
	}

	ret = v4l2_ctrl_handler_setup(&state->ctrl_hdl);
    if(ret) {
        pr_err("Err v4l2_ctrl_handler_setup\n");
    }
    return ret;
}

/** 
 * -----------------------------------------------------------------------------
 * v4l2_subdev_core_ops
 * -----------------------------------------------------------------------------
 */

#if defined(CONFIG_VIDEO_ADV_DEBUG) || defined(CONFIG_VIDEO_BKADV_DEBUG)
/**
 * @see g_register 
 */
static int isl79987_core_g_register(struct v4l2_subdev *sd, struct v4l2_dbg_register *reg)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
    struct i2c_client *client = state->client;

	if(state->is_dummy) {
		reg->val =0xBF;
		return 0;
	}

    if(reg->match.addr != ISL79987_I2C_IO) {
		bk_pr_debug("%s addr:0x%x name:%s\n",__func__,reg->match.addr,reg->match.name); 
        return -EINVAL;
	}
	if(reg->size != 1) {
		bk_pr_debug("%s size %d\n",__func__, reg->size);
        return -EINVAL;		
	}

    ReadTW88(client, (u8)reg->reg, (u8 *)&reg->val);
	reg->val &= 0x00FF;
    return 0;        
}

/**
 * @see s_register 
 */
static int isl79987_core_s_register(struct v4l2_subdev *sd, const struct v4l2_dbg_register *reg)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
    struct i2c_client *client = state->client;

	if(state->is_dummy)
		return 0;

    if(reg->match.addr != ISL79987_I2C_IO) {
		bk_pr_debug("%s addr:0x%x name:%s\n",__func__,reg->match.addr,reg->match.name); 
        return -EINVAL;
	}
	if(reg->size != 1) {
		bk_pr_debug("%s size %d\n",__func__, reg->size);
        return -EINVAL;		
	}

    WriteTW88(client, (u8)reg->reg, (u8)reg->val);
    return 0;
}
#endif

/**
 * @see s_power 
 */
static int isl79987_core_s_power(struct v4l2_subdev *sd, int on)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);

	bk_pr_debug("%s on:%d\n",__func__, on);

	if(state->is_dummy) {
		return 0;
	}

	isl79987_mipi_power(state, on);
	return 0;
}

static const struct v4l2_subdev_core_ops isl79987_core_ops = {
#if defined(CONFIG_VIDEO_ADV_DEBUG)
	.g_register = isl79987_core_g_register,
	.s_register = isl79987_core_s_register,
#endif
	.s_power	= isl79987_core_s_power,
};

/** 
 * -----------------------------------------------------------------------------
 * v4l2_subdev_video_ops
 * -----------------------------------------------------------------------------
 */

/**
 * @see s_routing 
 * command
 *	v4l2-ctl -d /dev/v4l-subdev1 --s-routing '0 -> 4 [1]'
 *	v4l2-ctl -d /dev/v4l-subdev1 --s-routing '1 -> 4 [1]'
 *	v4l2-ctl -d /dev/v4l-subdev1 --s-routing '2 -> 4 [1]'
 *	v4l2-ctl -d /dev/v4l-subdev1 --s-routing '3 -> 4 [1]'
 */
/* BKTODO   Link to primary decoder
	 00 01 02 03  => 00 01 02 03
	 mapping....
*/
static int isl79987_s_routing(struct v4l2_subdev *sd, u32 input, u32 output, u32 config)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
    //struct i2c_client *client = state->client;
	//u8 tmp;
	int ret = 0;

	mutex_lock(&state->mutex);
	if(input > ISL79987_AFE_SINK3) {
        pr_err("%s input %d too big\n",__func__,input);
        input = ISL79987_AFE_SINK0;
    }	

	//BKTODO180611
	//if(tx->input != input) {
	//	if(state->is_dummy ==0) { 			
	//		WriteTW88(client, 0xFF,0x06); 
	//		ReadTW88(client, 0x60, &tmp);
	//		tmp &= 0xF0;
	//		if(tx->input)
	//			tmp += tx->input;	
	//		WriteTW88(client, 0x60, tmp);
	//	}
	//	
		state->csi2_input[0] = input;		
	//	ret = isl79987_csi2_change_sink_pad_link(sd); 
	//}
	mutex_unlock(&state->mutex);
	return ret;
}

/**
 * @see g_routing 
 * command: v4l2-ctl -d /dev/v4l-subdev1 --g-routing
 */
static int isl79987_g_routing(struct v4l2_subdev *sd, u32 *input, u32 *output, u32 *config)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);

	if(input==NULL
    || output==NULL
    || config==NULL) {
		pr_err("%s: FAIL\n", __func__);
		return -EINVAL;
	}

	*input = state->csi2_input[0];
	*output = ISL79987_CSI2_SOURCE;
	*config = 1;

	return 0;	
}

/**
 * @see g_std 
 * command
 *	v4l2-ctl -d /dev/v4l-subdev1 --get-standard
 */
static int isl79987_afe_g_std(struct v4l2_subdev *sd, v4l2_std_id *norm)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);

	*norm = state->curr_norm;

	bk_pr_debug("%s norm:%x %s\n", 
		__func__,(unsigned int)state->curr_norm, 
		state->curr_norm & V4L2_STD_525_60 ? "60Hz":"50Hz");

	return 0;
}

/**
 * @see s_std 
 * command
 *	v4l2-ctl -d /dev/v4l-subdev1 --set-standard ntsc
 *	v4l2-ctl -d /dev/v4l-subdev1 --set-standard pal
 */
static int isl79987_afe_s_std(struct v4l2_subdev *sd, v4l2_std_id std)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	int afe_std;

	bk_pr_debug("%s std:%x\n",__func__, (unsigned int)std);

	afe_std = isl79987_afe_std(std);
	if (afe_std < 0)
		return afe_std;

	mutex_lock(&state->mutex);
	state->curr_norm = std;
	mutex_unlock(&state->mutex);

	return 0;
}

/**
 * @see querystd 
 * command:
 *	v4l2-ctl -d /dev/v4l-subdev1 --get-detected-standard
 */
static int isl79987_afe_querystd(struct v4l2_subdev *sd, v4l2_std_id *std)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	int ret;

	bk_pr_debug("%s\n",__func__);

	mutex_lock(&state->mutex);
	ret = isl79987_afe_status(state, 0, NULL, std);
	mutex_unlock(&state->mutex);

	return ret;
}

/**
 * @see g_tvnorms 
 */
static int isl79987_afe_g_tvnorms(struct v4l2_subdev *sd, v4l2_std_id *norm)
{
	*norm = V4L2_STD_ALL;
	return 0;
}

/** 
 * @see g_input_status 
 */
static int isl79987_afe_g_input_status(struct v4l2_subdev *sd, u32 *status)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	int ret;

	mutex_lock(&state->mutex);
	ret = isl79987_afe_status(state, 0, status, NULL);
	mutex_unlock(&state->mutex);
	return ret;
}

/**
 * @see s_stream 
 */
static int isl79987_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	struct i2c_client *client = state->client;
	int ret=0;
	u8 tmp;
    
	bk_pr_debug("%s enable:%d state->streaming was %d\n",__func__, enable, state->streaming);

	mutex_lock(&state->mutex);

	if(enable)
    {
		state->streaming = true;
    }
	else 
    {
		if(state->streaming)
        {
			state->streaming = false;
        }
	}

	if(enable && state->streaming==1) {
		ret=isl79987_mipi_reset(state, MIPI_RESET_SET);
		isl79987_wait_field(state, 1);
		ret=isl79987_mipi_reset(state, MIPI_RESET_CLEAR);
		usleep_range(16600,17000); /* 16ms */ 
		usleep_range(16600,17000); /* 16ms */ 

		WriteTW88(client, 0xFF, 1);
        ReadTW88(client, ISL79987_AFE_STATUS_REG, &tmp);
		bk_pr_debug("Field:%d\n",tmp & ISL79987_AFE_STATUS_FIELD ? 1:0);
	}

	mutex_unlock(&state->mutex);

	return ret;
}

/**
 *
 * @see g_pixelaspect
 */
static int isl79987_g_pixelaspect(struct v4l2_subdev *sd,
				     struct v4l2_fract *aspect)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);

	if (state->curr_norm & V4L2_STD_525_60) {
		aspect->numerator = 4;   //11;
		aspect->denominator = 3; //10;
	} else {
		aspect->numerator = 4;   //54;
		aspect->denominator = 3; //59;
	}

	return 0;
}

/**
 * @see g_mbus_config: get supported mediabus configurations 
 * command: 
 *	v4l2-ctl -d /dev/v4l-subdev1 --g-mbus
 */
static int isl79987_g_mbus_config(struct v4l2_subdev *sd,
					struct v4l2_mbus_config *cfg)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);

	memset(cfg, 0, sizeof(struct v4l2_mbus_config));

    cfg->type = V4L2_MBUS_CSI2_DPHY;
    cfg->flags = V4L2_MBUS_CSI2_NONCONTINUOUS_CLOCK;

	switch(state->lanes) {
	case 1:	
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE; 
		break;
	case 2: 
	default:
		cfg->flags |= V4L2_MBUS_CSI2_1_LANE; 
		cfg->flags |= V4L2_MBUS_CSI2_2_LANE; 
		break;
	}
	switch(state->vc & 0x0003) {
	case 1: cfg->flags |= V4L2_MBUS_CSI2_CHANNEL_1; break;
	case 2: cfg->flags |= V4L2_MBUS_CSI2_CHANNEL_2; break;
	case 3: cfg->flags |= V4L2_MBUS_CSI2_CHANNEL_3; break;
	case 0: 
	default:
			cfg->flags |= V4L2_MBUS_CSI2_CHANNEL_0; break;
	}

    return 0;
}

/**
 * @see s_mbus_config: set a certain mediabus configuration. 
 * command
 * v4l2-ctl -d /dev/v4l-subdev1 --s-mbus '4 1'
 * v4l2-ctl -d /dev/v4l-subdev1 --s-mbus '4 3'
 */
static int isl79987_s_mbus_config(struct v4l2_subdev *sd,const struct v4l2_mbus_config *cfg)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	u8 temp;

    if(cfg->type != V4L2_MBUS_CSI2_DPHY) {
        return 0;
	}
	
	state->lanes = 1;
	if(cfg->flags & V4L2_MBUS_CSI2_2_LANE)  state->lanes++;

    /* On which channels it can send video data */
    if     (cfg->flags & V4L2_MBUS_CSI2_CHANNEL_0)  state->vc = 0x00; /* 0xE4 = 11-10-01-00 */
    else if(cfg->flags & V4L2_MBUS_CSI2_CHANNEL_1)  state->vc = 0x39; /* 0x39 = 00-11-10-01 */
    else if(cfg->flags & V4L2_MBUS_CSI2_CHANNEL_2)  state->vc = 0x4E; /* 0x4E = 01-00-11-10 */
    else if(cfg->flags & V4L2_MBUS_CSI2_CHANNEL_3)  state->vc = 0x93; /* 0x93 = 10-01-00-11 */
	else                                            state->vc = 0x00; /* default. 0xE4 */

	temp = (cfg->flags & 0x0C00) >> 10;
	switch(temp) {
	case 0: state->chs = 4; break;
	case 1: state->chs = 1; break;
	case 2: state->chs = 2; break;
	case 3: state->chs = 4;	break;		
	}


	return 0;
}


static const struct v4l2_subdev_video_ops isl79987_video_ops = {
	.s_routing = isl79987_s_routing,
	// .g_routing = isl79987_g_routing,
	.g_std = isl79987_afe_g_std,
	.s_std = isl79987_afe_s_std,
	.querystd = isl79987_afe_querystd,
	.g_tvnorms = isl79987_afe_g_tvnorms,
	.g_input_status = isl79987_afe_g_input_status,
	.s_stream = isl79987_s_stream,
	.g_pixelaspect = isl79987_g_pixelaspect,
	//.g_mbus_config = isl79987_g_mbus_config,
	//.s_mbus_config = isl79987_s_mbus_config,
};


/** 
 * -----------------------------------------------------------------------------
 * v4l2_subdev_pad_ops
 * -----------------------------------------------------------------------------
 */

/**
 * @see enum_mbus_code
 */
static int isl79987_enum_mbus_code(struct v4l2_subdev *sd,
				    struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_UYVY8_1X16;

	return 0;
}

/**
 * @see get_fmt get output format
 * VIDIOC_G_FMT
 * VIDIOC_SUBDEV_G_FMT
 * command: 
 *	media-ctl --get-v4l2 "'isl79987 4-0044 afe':4"    
 *	v4l2-ctl -d /dev/v4l-subdev1 --get-subdev-fmt 4
 */

static int isl79987_get_format(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state,
			      struct v4l2_subdev_format *format)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	struct v4l2_mbus_framefmt *mbusformat;

	return 0;
}


/**
 * @see set_fmt set output format.
 * VIDIOC_S_FMT
 * command: 
 *   media-ctl -V "'isl79987 4-0044 afe':4 [fmt:UYVY8_1X16/720x480 field:interlaced]"
*/


static int isl79987_set_format(struct v4l2_subdev *sd,
			        struct v4l2_subdev_state *sd_state,
				    struct v4l2_subdev_format *sdformat)
{
	struct isl79987_state *state = isl79987_sd_to_state(sd);
	struct v4l2_mbus_framefmt *mbusformat;
	int ret;
	u32 status;


	return 0;
}



static const struct v4l2_subdev_pad_ops isl79987_pad_ops = {
	.enum_mbus_code = isl79987_enum_mbus_code,
	.get_fmt = isl79987_get_format,
	.set_fmt = isl79987_set_format,
};

/** 
 * -----------------------------------------------------------------------------
 * v4l2_subdev_ops
 * -----------------------------------------------------------------------------
 */
static const struct v4l2_subdev_ops isl79987_ops = {
	.core  = &isl79987_core_ops,
	.video = &isl79987_video_ops,
	.pad   = &isl79987_pad_ops,
};



/** 
 * ------------------------------------------------------
 * Media Operations
 * ------------------------------------------------------
 */

static const struct media_entity_operations isl79987_media_ops = {
	.link_validate = v4l2_subdev_link_validate,
};


/** 
 * ------------------------------------------------------
 * Device Tree
 * ------------------------------------------------------
 */
#if 0 //Example
	isl79987,is-dummy = <1>;

	isl79987,vc = <0>;    /* default: 0xE4 */
	isl79987,vc = <228>;  /* 0xE4 = 11-10-01-00 */
	isl79987,vc = <57>;   /* 0x39 = 00-11-10-01 */
	isl79987,vc = <78>;   /* 0x4E = 01-00-11-10 */
	isl79987,vc = <147>;  /* 0x93 = 10-01-00-11 */

	isl79987,chs = <4>;		/* 1,2,4CHs */
#endif
static int isl79987_parse_dt(struct isl79987_state *state)
{
	struct device_node *ep_np = NULL;
	struct of_endpoint ep;
	bool found = false;
	struct v4l2_fwnode_endpoint v4l2_ep;
    struct i2c_client *client = state->client;

    pr_debug("On isl79987_parse_dt\n");

	if (of_property_read_u32(state->dev->of_node, "isl79987,is-dummy", &state->is_dummy)==0)
		bk_pr_debug("%s state->is_dummy is %d\n",__func__, state->is_dummy);
	if (of_property_read_u32(state->dev->of_node, "isl79987,vc", &state->vc)==0)
		bk_pr_debug("%s state->vc is 0x%04X\n",__func__, state->vc);
	if (of_property_read_u32(state->dev->of_node, "isl79987,chs", &state->chs)==0)
		bk_pr_debug("%s state->chs is %d\n",__func__, state->chs);


	for_each_endpoint_of_node(state->dev->of_node, ep_np) {
		of_graph_parse_endpoint(ep_np, &ep);

		v4l2_fwnode_endpoint_parse(of_fwnode_handle(ep_np), &v4l2_ep);

		v4l_info(client, "Endpoint %s on port %d",
				of_node_full_name(ep.local_node),
				ep.port);

		if (ep.port >= ISL79987_PORT_MAX ) {
			v4l_err(client, "Invalid endpoint %s on port %d",
				of_node_full_name(ep.local_node),
				ep.port);
			continue;
		}

		if (state->endpoints[ep.port]) {
			v4l_err(client,
				"Multiple port endpoints are not supported");
			continue;
		}

		of_node_get(ep_np);
		state->endpoints[ep.port] = ep_np;

		if (ep.port == ISL79987_CSI2_SOURCE) {
			state->lanes = v4l2_ep.bus.mipi_csi2.num_data_lanes;
			bk_pr_debug("%s state->lanes is %d\n",__func__, state->lanes);
		}
		found = true;
	}
	return found ? 0 : -ENODEV;
}


static void isl79987_dt_cleanup(struct isl79987_state *state)
{
	unsigned int i;

	for (i = 0; i < ISL79987_PORT_MAX; i++)
		of_node_put(state->endpoints[i]);
}


/** 
 * ------------------------------------------------------
 * probe
 * ------------------------------------------------------
 */

static int isl79987_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
    struct isl79987_state *state;
	int i;
    int ret;

    printk(KERN_ERR "In isl79987_probe\n");

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA)) {
        pr_info("**Renesas isl79987_probe failed**");
        return -EIO;
    }
 
    state = kzalloc(sizeof(struct isl79987_state), GFP_KERNEL);
    if (state == NULL) {
		pr_err("%s ENOMEM\n",__func__);
        return -ENOMEM;
	}
	mutex_init(&state->mutex);

    state->dev = &client->dev;
    state->client = client;
    i2c_set_clientdata(client, state);

    v4l_info(client, "chip find @ 0x%02x (%s)\n",
        client->addr << 1, client->adapter->name);

    state->is_dummy = 0;
    ret = isl79987_check_chip_id(state);
    if (ret) {
        dev_err(&client->dev, "isl79987 check chip id failed\n");
        goto err_free_mutex;
    }

	state->vc = 0x00;
	state->chs = 4; /* 4Channels */

    ret = isl79987_parse_dt(state);
    if (ret) {
        dev_err(&client->dev, "isl79987 parse error\n");
        goto err_free_mutex;
    }
	if(state->chs==1 && state->lanes==2) {
		bk_pr_debug("%s 1CH supports only 1Lane.\n",__func__);
		state->lanes = 1;
	}

    ret = isl79987_initialise_clients(state);
	if (ret) {
		bk_pr_err("Fail initialise_clients()\n");
		goto err_cleanup_dt;
	}

	state->streaming = 0;
	state->curr_norm = V4L2_STD_NTSC;
	state->afe_field = 1;
	state->fi.interval.numerator = 1;
	state->fi.interval.denominator = 60; 
	state->format.width = 720;
	state->format.height = 480;
	state->format.code = MEDIA_BUS_FMT_UYVY8_1X16;
	state->format.field = V4L2_FIELD_INTERLACED;
	state->format.colorspace = V4L2_COLORSPACE_SMPTE170M;
	state->aspect_ratio.numerator = 4;
	state->aspect_ratio.denominator = 3;

	state->pseudo = 0;
	state->histo = 0;
	//state->nvc = 0;

	ret = isl79987_download_chip_registers(state);
    if (ret) {
        bk_pr_err("Fail init_chip_registers()\n");
        goto err_cleanup_dt;
	}

	v4l2_subdev_init(&state->sd, &isl79987_ops);
	state->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	state->sd.owner = state->dev->driver->owner;
	state->sd.dev = state->dev;
	v4l2_set_subdevdata(&state->sd, state);

	snprintf(state->sd.name, sizeof(state->sd.name), 
		"%s %d-%04x %s",
		state->dev->driver->name,
		i2c_adapter_id(state->client->adapter),
		state->client->addr, "afe" /*ident*/);

	state->sd.entity.function = MEDIA_ENT_F_UNKNOWN;		//for CSI2
	state->sd.entity.function = MEDIA_ENT_F_ATV_DECODER; 	//for AFE
	state->sd.entity.ops = &isl79987_media_ops;
	state->sd.internal_ops = &isl79987_csi2_internal_ops;
	bk_pr_debug("state->sd.name:%s\n",state->sd.name);

	for(i = ISL79987_AFE_SINK0; i <= ISL79987_AFE_SINK3; i++) {
		state->csi2_input[i]=i;
		/* Inputs and ports are 1-indexed to match the data sheet */
		if (state->endpoints[i]) {
			state->csi2_input[i] = i;
			bk_pr_debug("%s found state->endpoints[%d]\n",__func__,i);
		}
	}

	state->sd.fwnode = of_fwnode_handle(state->endpoints[ISL79987_CSI2_SOURCE]);

	for(i = ISL79987_AFE_SINK0; i <= ISL79987_AFE_SINK3; i++)
		state->pads[i].flags = MEDIA_PAD_FL_SINK;
	//for (i = ISL79987_CSI2_SOURCE0; i <= ISL79987_CSI2_SOURCE; i++)
	//	state->pads[i].flags = MEDIA_PAD_FL_SOURCE;
	state->pads[ISL79987_CSI2_SOURCE].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&state->sd.entity, 
			ISL79987_PORT_MAX,
			state->pads);
	if (ret) {
        pr_err("Fail media_entity_pads_init()\n");
		goto err_cleanup_clients; 
    }

	ret = isl79987_init_controls(state);
	if (ret) {
        pr_err("Fail init_controls()\n");
		goto err_cleanup_media;
    }
	isl79987_csi2_set_pixelrate(&state->sd, state->chs * 13500000);	

	ret = v4l2_async_register_subdev(&state->sd);
	if (ret) {
        pr_err("Fail v4l2_async_register_subdev()\n");
		goto err_free_ctrl;
    }

    bk_pr_debug("Finish isl79987_probe\n");

    return 0;

err_free_ctrl:
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
err_cleanup_media:
	media_entity_cleanup(&state->sd.entity);
err_cleanup_clients:
	isl79987_unregister_clients(state);
err_cleanup_dt:
	isl79987_dt_cleanup(state);
err_free_mutex:
	mutex_destroy(&state->mutex);
	kfree(state);

	return ret;
}

static int isl79987_remove(struct i2c_client *client)
{
	struct isl79987_state *state = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(&state->sd);
	v4l2_ctrl_handler_free(&state->ctrl_hdl);
	media_entity_cleanup(&state->sd.entity);
	isl79987_unregister_clients(state);
	isl79987_dt_cleanup(state);
	mutex_destroy(&state->mutex);
	kfree(state);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int isl79987_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl79987_state *state = i2c_get_clientdata(client);

    if(state->is_dummy)
        return 0;

    isl79987_mipi_power(state, 0);

	return 0;
}

static int isl79987_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct isl79987_state *state = i2c_get_clientdata(client);

    if(state->is_dummy)
        return 0;

    isl79987_mipi_power(state, 1);
	isl79987_SW_reset(state);

	return 0;
}

static const struct dev_pm_ops isl79987_pm_ops = {
	SET_LATE_SYSTEM_SLEEP_PM_OPS(isl79987_suspend, isl79987_resume)
};
#endif

static const struct i2c_device_id isl79987_id[] = {
	{ "isl79987", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, isl79987_id);

static const struct of_device_id isl79987_of_table[] = {
	{ .compatible = "isl,isl79987", },
	{ }
};
MODULE_DEVICE_TABLE(of, isl79987_of_table);

static struct i2c_driver isl79987_driver = {
	.driver = {
		.name = "isl79987",
#ifdef CONFIG_PM_SLEEP
		.pm = &isl79987_pm_ops,
#endif
		.of_match_table = isl79987_of_table,
	},
	.probe = isl79987_probe,
	.remove = isl79987_remove,
	.id_table = isl79987_id,
};

module_i2c_driver(isl79987_driver);

MODULE_AUTHOR("Brian Kang <brian.kang.ry@renesas.com>");
MODULE_DESCRIPTION("ISL79987 video decoder");
MODULE_LICENSE("GPL v2");