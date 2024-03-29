#ifndef __TOUCHPANEL_H__
#define __TOUCHPANEL_H__

#include <linux/hrtimer.h>
#include <linux/string.h>
#include <linux/vmalloc.h>
//#include <linux/io.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/kthread.h>
#include <linux/bitops.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/byteorder/generic.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/interrupt.h>
#include <linux/time.h>

#include <linux/proc_fs.h>
#include <asm/uaccess.h>


#include <linux/jiffies.h>


/**********************Custom define begin**********************************************/


#define TPD_POWER_SOURCE_CUSTOM         MT6351_POWER_LDO_VLDO28
//#define IIC_PORT                        1 //MT6572: 0  MT6589:1 , Based on the I2C index you choose for TPM

/*
///// ***** virtual key  definition  ***** /////

Below are the recommend  virtual key definition for different resolution TPM.

HVGA  320x480     2key ( (80,530);(240,530) )           3key  ( (80,530);(160;530);(240,530) )          4key   ( (40,530);(120;530);(200,530);(280,530)  )
WVGA  480x800     2key ( (80,900);(400,900) )           3key  ( (80,900);(240,900);(400,900) )          4key   ( (60,900);(180;900);(300,900);(420,900)  )
FWVGA 480x854     2key ( (80,900);(400,900) )           3key  ( (80,900);(240,900);(400,900) )          4key   ( (60,900);(180;900);(300,900);(420,900)  )
QHD   540x960     2key ( (90,1080);(450,1080) )         3key  ( (90,1080);(270,1080);(450,1080) )       4key   ( (90,1080);(180;1080);(360,1080);(450,1080)  )
HD    1280x720    2key ( (120,1350);(600,1350) )        3key  ( (120,1350);(360,1350);(600,1350) )      4key   ( (120,1080);(240;1080);(480,1080);(600,1080)  )
FHD   1920x1080   2key ( (160,2100);(920,2100) )        3key  ( (160,2100);(540,2100);(920,2100) )      4key   ( (160,2100);(320;1080);(600,1080);(920,2100)  )
*/
#if 0
#define TPD_HAVE_BUTTON          // if have virtual key,need define the MACRO
#define TPD_BUTTON_HEIGH        (40)
#define TPD_KEY_COUNT           (1)
#define TPD_KEYS                {KEY_HOME}
#define TPD_KEYS_DIM            {{360,2000,120,TPD_BUTTON_HEIGH}}
#endif
/*********************Custom Define end*************************************************/

#define TPD_NAME    "ft5x46_ts"

/* Pre-defined definition */
#define TPD_TYPE_CAPACITIVE
#define TPD_TYPE_RESISTIVE
#define TPD_POWER_SOURCE

#define TPD_I2C_NUMBER           		0
#define TPD_WAKEUP_TRIAL         		60
#define TPD_WAKEUP_DELAY         		100

//#define VELOCITY_CUSTOM
#define TPD_VELOCITY_CUSTOM_X 			15
#define TPD_VELOCITY_CUSTOM_Y 			20

#define TPD_DELAY                		(2*HZ/100)
//#define TPD_RES_X                		1080
//#define TPD_RES_Y                		1920
#define TPD_CALIBRATION_MATRIX  		{962,0,0,0,1600,0,0,0};

//#define TPD_HAVE_CALIBRATION
//#define TPD_HAVE_TREMBLE_ELIMINATION

/******************************************************************************/
/*Chip Device Type*/
#define IC_FT5X06						0	/*x=2,3,4*/
#define IC_FT5606						1	/*ft5506/FT5606/FT5816*/
#define IC_FT5316						2	/*ft5x16*/
#define IC_FT6208						3  	/*ft6208*/
#define IC_FT6x06     					4	/*ft6206/FT6306*/
#define IC_FT5x06i     					5	/*ft5306i*/
#define IC_FT5x36     					6	/*ft5336/ft5436/FT5436i*/
#define IC_FT5x46     					7	/*ft5346/ft5446*/

/*register address*/
#define FT_REG_CHIP_ID				0xA3    //chip ID 
#define FT_REG_FW_VER				0xA6    //FW version 
#define FT_REG_VENDOR_ID			0xA8    //TP vendor ID 

#define FT_APP_INFO_ADDR	        0xd7f8

/*max point*/
#define TPD_MAX_POINTS_2            2
#define TPD_MAX_POINTS_5            5
#define TPD_MAX_POINTS_10           10

#define FT_MAX_ID	0x0F

/*calibration option*/
#define AUTO_CLB_NEED               1
#define AUTO_CLB_NONEED             0

/*debug fuction*/
#define TPD_SYSFS_DEBUG
#define FTS_CTL_IIC
#define FTS_APK_DEBUG

/*tp power down when suspend*/
//#define TPD_CLOSE_POWER_IN_SLEEP

/*apgrade*/
//#define TPD_AUTO_UPGRADE				// if need upgrade CTP FW when POWER ON,pls enable this MACRO
//#define TPD_HW_REST

/*proximity*/
//#define TPD_PROXIMITY					// if need the PS funtion,enable this MACRO

/*gesture*/
//#define FTS_GESTURE                     // if need gesture funtion,enable this MARCO
#ifdef FTS_GESTURE
#undef TPD_CLOSE_POWER_IN_SLEEP
#define FTS_GESTURE_DBG                 // if define it,open gesture in suspend every time

#define GESTURE_SWITCH_OPEN          0x50
#define GESTURE_SWITCH_CLOSE         0x51
#endif //FTS_GESTURE

/*glove*/
//#define FTS_GLOVE
#ifdef FTS_GLOVE
#define GLOVE_SWITCH_OPEN            0x53
#define GLOVE_SWITCH_CLOSE           0x54
#endif

/*trace*/
#if 1
//#define FTS_DBG_FLAG
#ifdef FTS_DBG_FLAG
#define FTS_DBG(fmt, args...) 				printk("[FTS]" fmt, ## args)
#else
#define FTS_DBG(fmt, args...) 				do{}while(0)
#endif
#endif
/*current chip information for upgrade*/
struct Upgrade_Info
{
	u8  CHIP_ID;
	u8  FTS_NAME[20];
	u8  TPD_MAX_POINTS;
	u8  AUTO_CLB;
	u16 delay_aa;		    /*delay of write FT_UPGRADE_AA */
	u16 delay_55;		    /*delay of write FT_UPGRADE_55 */
	u8  upgrade_id_1;	    /*upgrade id 1 */
	u8  upgrade_id_2;	    /*upgrade id 2 */
	u16 delay_readid;	    /*delay of read id */
	u16 delay_earse_flash;  /*delay of earse flash*/
};
#define CUST_EINTF_TRIGGER_RISING     			1    //High Polarity and Edge Sensitive
#define CUST_EINTF_TRIGGER_FALLING    			2    //Low Polarity and Edge Sensitive
#define CUST_EINTF_TRIGGER_HIGH      				4    //High Polarity and Level Sensitive
#define CUST_EINTF_TRIGGER_LOW       				8    //Low Polarity and Level Sensitive

#define CUST_EINT_DEBOUNCE_DISABLE          0
#define CUST_EINT_DEBOUNCE_ENABLE           1

#define CUST_EINT_TOUCH_PANEL_NUM              1
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_CN      0
#define CUST_EINT_TOUCH_PANEL_TYPE							CUST_EINTF_TRIGGER_FALLING
#define CUST_EINT_TOUCH_PANEL_DEBOUNCE_EN      CUST_EINT_DEBOUNCE_DISABLE

#define GPIO_TOUCH_RESET_PIN GPIO_CTP_RST_PIN
#define GPIO_TOUCH_RESET_PIN_M_GPIO GPIO_CTP_RST_PIN_M_GPIO
#define GPIO_TOUCH_EINT_PIN GPIO_CTP_EINT_PIN
#define GPIO_TOUCH_EINT_PIN_M_EINT GPIO_CTP_EINT_PIN_M_EINT
#define CUST_EINT_TOUCH_INT_NUM CUST_EINT_TOUCH_PANEL_NUM

#endif /* TOUCHPANEL_H__ */
