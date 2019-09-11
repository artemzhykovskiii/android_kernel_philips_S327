
#ifndef LSM6DS3_H
#define LSM6DS3_H
	 
#include <linux/ioctl.h>
#include <hwmsensor.h>
#include <accel.h>
#include <gyroscope.h>
#include <step_counter.h>
#include <cust_acc.h>
#include <cust_gyro.h>
#include <linux/of.h>
#include <linux/of_irq.h>

#define DEBUG 1
#define POWER_NONE_MACRO MT65XX_POWER_NONE
	 
/* LSM6DS3 Register Map  (Please refer to LSM6DS3 Specifications) */
#define LSM6DS3_REG_FUNC_CFG_ACCESS     0x01
#define LSM6DS3_REG_INT1_CTRL           0x0D
#define LSM6DS3_REG_WHO_AM_I			0x0F
#define LSM6DS3_REG_CTRL1_XL			0x10
#define LSM6DS3_REG_CTRL2_G				0x11
#define LSM6DS3_REG_CTRL3_C				0x12
#define LSM6DS3_REG_CTRL4_C				0x13
#define LSM6DS3_REG_CTRL5_C		    	0x14
#define LSM6DS3_REG_CTRL6_C		    	0x15
#define LSM6DS3_REG_CTRL7_G		    	0x16
#define LSM6DS3_REG_CTRL8_XL		    0x17
#define LSM6DS3_REG_CTRL9_XL		    0x18
#define LSM6DS3_REG_CTRL10_C			0x19

#define LSM6DS3_REG_OUTX_L_G 			0x22
#define LSM6DS3_REG_OUTX_H_G 			0x23
#define LSM6DS3_REG_OUTY_L_G 			0x24
#define LSM6DS3_REG_OUTY_H_G 			0x25
#define LSM6DS3_REG_OUTZ_L_G 			0x26
#define LSM6DS3_REG_OUTZ_H_G 			0x27

#define LSM6DS3_REG_OUTX_L_XL 			0x28
#define LSM6DS3_REG_OUTX_H_XL			0x29
#define LSM6DS3_REG_OUTY_L_XL			0x2A
#define LSM6DS3_REG_OUTY_H_XL			0x2B
#define LSM6DS3_REG_OUTZ_L_XL			0x2C
#define LSM6DS3_REG_OUTZ_H_XL			0x2D
#define LSM6DS3_REG_STEP_COUNTER_L      0x4B
#define LSM6DS3_REG_STEP_COUNTER_H      0x4C
#define LSM6DS3_REG_FUNC_SRC            0x53
#define LSM6DS3_REG_TAP_CFG             0x58

/*
*  embeded register
*/
#define LSM6DS3_REG_PEDO_THS                    0x0F
#define LSM6DS3_REG_PEDO_DEB                    0x14

/*
*  register mask bit
*/
#define LSM6DS3_REG_INT1_CTRL_MASK_INT1_STEP_DETECTOR       0x80
#define LSM6DS3_REG_INT1_CTRL_MASK_INT1_SIGN_MOT            0x40

#define LSM6DS3_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN        0x80

#define LSM6DS3_REG_CTRL1_XL_MASK_FS_XL			            0x0C
#define LSM6DS3_REG_CTRL1_XL_MASK_ODR_XL		            0xF0

#define LSM6DS3_REG_CTRL2_G_MASK_FS_G			            0x0C
#define LSM6DS3_REG_CTRL2_G_MASK_ODR_G			            0xF0
#define LSM6DS3_REG_CTRL2_G_MASK_ODR_G			            0xF0

#define LSM6DS3_REG_CTRL10_C_MASK_FUNC_EN                   0x04
#define LSM6DS3_REG_CTRL10_C_MASK_PEDO_RST_STEP             0x02

#define LSM6DS3_REG_TAP_CFG_MASK_PEDO_EN                    0x40
#define LSM6DS3_REG_TAP_CFG_MASK_LIR                        0x01

#define LSM6DS3_REG_FUNC_SRC_MASK_STEP_DETECTED             0x10
#define LSM6DS3_REG_FUNC_SRC_MASK_TILT_IA                   0x20
#define LSM6DS3_REG_FUNC_SRC_MASK_SIGN_MOTION_IA            0x40
/*
*  embeded register mask bit
*/
#define LSM6DS3_REG_PEDO_THS_MASK_PEDO_4G                   0x80
#define LSM6DS3_REG_PEDO_THS_MASK_PEDO_THS_MIN              0x1F
#define LSM6DS3_REG_PEDO_DEB_MASK_DEB_TIME                  0xF8
#define LSM6DS3_REG_PEDO_DEB_MASK_DEB_STEP                  0x07

/*
*  register value
*/
#define LSM6DS3_FIXED_DEVID_LSM6DS3			    0x69
#define LSM6DS3_FIXED_DEVID_LSM6DSM			    0x69
#define LSM6DS3_FIXED_DEVID_LSM6DSL			    0x6A

#define LSM6DS3_REG_FUNC_CFG_ACCESS_ENABLE                  0x80
#define LSM6DS3_REG_FUNC_CFG_ACCESS_DISABLE                 0x00

#define LSM6DS3_REG_CTRL1_XL_ODR_208HZ			0x50
#define LSM6DS3_REG_CTRL1_XL_ODR_104HZ			0x40
#define LSM6DS3_REG_CTRL1_XL_ODR_52HZ			0x30
#define LSM6DS3_REG_CTRL1_XL_ODR_26HZ           0x20
#define LSM6DS3_REG_CTRL1_XL_ODR_12HZ           0x10
#define LSM6DS3_REG_CTRL1_XL_ODR_0HZ			0x00

#define LSM6DS3_REG_CTRL1_XL_FS_2G			                0x00
#define LSM6DS3_REG_CTRL1_XL_FS_16G			                0x03
#define LSM6DS3_REG_CTRL1_XL_FS_4G			                0x08
#define LSM6DS3_REG_CTRL1_XL_FS_8G			                0x0C

#define LSM6DS3_REG_CTRL2_G_ODR_208HZ			            0x50
#define LSM6DS3_REG_CTRL2_G_ODR_104HZ			            0x40
#define LSM6DS3_REG_CTRL2_G_ODR_52HZ			            0x30
#define LSM6DS3_REG_CTRL2_G_ODR_0HZ				            0x00

#define LSM6DS3_REG_CTRL2_G_FS_245DPS			            0x00
#define LSM6DS3_REG_CTRL2_G_FS_500DPS			            0x03
#define LSM6DS3_REG_CTRL2_G_FS_1000DPS			            0x08
#define LSM6DS3_REG_CTRL2_G_FS_2000DPS			            0x0C

#define LSM6DS3_REG_CTRL10_C_FUNC_ENABLE                    0x04
#define LSM6DS3_REG_CTRL10_C_FUNC_DISABLE                   0x00
#define LSM6DS3_REG_CTRL10_C_FUNC_PEDO_RST_STEP             0x02

#define LSM6DS3_REG_TAP_CFG_PEDO_ENABLE                     0x40
#define LSM6DS3_REG_TAP_CFG_PEDO_DISABLE                    0x00
#define LSM6DS3_REG_TAG_CFG_LIR                             0x01

#define LSM6DS3_REG_INT1_CTRL_INT1_STEP_DETECTOR_ENABLE     0x80
#define LSM6DS3_REG_INT1_CTRL_INT1_STEP_DETECTOR_DISABLE    0x00

#define LSM6DS3_REG_INT1_CTRL_INT1_SIGN_MOT_ENABLE          0x40
#define LSM6DS3_REG_INT1_CTRL_INT1_SIGN_MOT_DISABLE         0x00

/*
*  embeded register value
*/
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_128MG            0x84
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_160MG            0x85
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_192MG            0x86
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_224MG            0x87
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_256MG            0x88
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_288MG            0x89
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_320MG            0x8A
#define LSM6DS3_REG_PEDO_THS_PEDO_4G_352MG            0x8B

#define LSM6DS3_REG_PEDO_DEB_TIME_800MS               0x50
#define LSM6DS3_REG_PEDO_DEB_TIME_880MS               0x58
#define LSM6DS3_REG_PEDO_DEB_TIME_960MS               0x60
#define LSM6DS3_REG_PEDO_DEB_TIME_1040MS              0x68
#define LSM6DS3_REG_PEDO_DEB_TIME_1120MS              0x70

#define LSM6DS3_REG_PEDO_DEB_STEP_5STEP               0x05
#define LSM6DS3_REG_PEDO_DEB_STEP_6STEP               0x06
#define LSM6DS3_REG_PEDO_DEB_STEP_7STEP               0x07


/*
*  return value
*/
#define LSM6DS3_SUCCESS					    0
#define LSM6DS3_ERR_I2C					    -1
#define LSM6DS3_ERR_STATUS					-3
#define LSM6DS3_ERR_SETUP_FAILURE			-4
#define LSM6DS3_ERR_GETGSENSORDATA			-5
#define LSM6DS3_ERR_IDENTIFICATION			-6

#define LSM6DS3_BUFSIZE			256

#define LSM6DS3_AXIS_X          0
#define LSM6DS3_AXIS_Y          1
#define LSM6DS3_AXIS_Z          2
#define LSM6DS3_AXES_NUM        3
#define LSM6DS3_DATA_LEN        6
#define LSM6DS3_DEV_NAME        "LSM6DS3"
//#define DEGREE_TO_RAD_1000		17
/* 1 rad = 180/PI degree, MAX_LSB = 131, */
/* 180*131/PI = 7506 */
#define DEGREE_TO_RAD    7506
#define CONFIG_LSM6DS3_LOWPASS   /*apply low pass filter on output*/       

#define CONFIG_LSM6DS3_ACC_DRY


/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI    = 0X08,
    ADX_TRC_INFO    = 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][LSM6DS3_AXES_NUM];
    int sum[LSM6DS3_AXES_NUM];
    int num;
    int idx;
};

/*----------------------------------------------------------------------------*/
struct lsm6ds3_acc {
    //struct i2c_client *client;
	u8     name[32];
    struct acc_hw *lsm6ds3_acc_hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw[LSM6DS3_AXES_NUM];

    /*data*/
    s32                     offset[LSM6DS3_AXES_NUM];
    s16                     data[LSM6DS3_AXES_NUM];
    bool                    lsm6ds3_acc_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LSM6DS3_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lsm6ds3_gyro {
    //struct i2c_client *client;
	u8     name[32];
    struct gyro_hw *lsm6ds3_gyro_hw;
    struct hwmsen_convert   cvt;
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;
    s32                     cali_sw[LSM6DS3_AXES_NUM];

    /*data*/
    s32                     offset[LSM6DS3_AXES_NUM];
    s16                     data[LSM6DS3_AXES_NUM];
    bool                    lsm6ds3_gyro_power;
	int                     odr;
	int                     enabled;
#if defined(CONFIG_LSM6DS3_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
};

struct lsm6ds3_pedo {
	u8     name[32];
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
    atomic_t                filter;

    u32                     data;
    int                     overflow;
    bool                    lsm6ds3_pedo_power;
	int                     odr;
	int                     step_c_enabled;
    int                     step_d_enabled;
    int                     sigmotion_enabled;
};

struct lsm6ds3_data {
	u8     name[32];
	u8     chip_id;
    struct i2c_client *client;
    struct lsm6ds3_acc lsm6ds3_acc_data;
	struct lsm6ds3_gyro lsm6ds3_gyro_data;
    struct lsm6ds3_pedo lsm6ds3_pedo_data;
    u8     reg_addr;
};

#define ST_TAG                  "STMEMS "
#define ST_ERR(fmt, args...)    printk(KERN_ERR ST_TAG "%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define ST_LOG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)

#if defined(DEBUG)
    #define ST_FUN(f)               printk(KERN_INFO ST_TAG "%s\n", __FUNCTION__)
	#define ST_DBG(fmt, args...)    printk(KERN_ERR ST_TAG fmt, ##args)
#else
	#define ST_FUN(f)
	#define ST_DBG(fmt, args...)
#endif

extern struct lsm6ds3_data *obj_i2c_data;

int lsm6ds3_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lsm6ds3_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len);
int lsm6ds3_open_ram_page(struct lsm6ds3_data *obj, u8 value);
int lsm6ds3_embed_func_enable(struct lsm6ds3_data *obj, u8 value);

void dumpReg(struct lsm6ds3_data *obj);
//int lsm6ds3_set_interrupt(struct lsm6ds3_data *obj, u8 intenable);

int lsm6ds3_acc_set_power_mode(struct lsm6ds3_acc *acc_obj, bool enable);
int lsm6ds3_acc_init(struct lsm6ds3_acc *acc_obj, int reset_cali);

extern struct acc_init_info lsm6ds3_acc_init_info;
extern int lsm6ds3_acc_init_flag;

int lsm6ds3_gyro_set_power_mode(struct lsm6ds3_gyro *gyro_obj, bool enable);
int lsm6ds3_gyro_init(struct lsm6ds3_gyro *gyro_obj, int reset_cali);

extern struct gyro_init_info lsm6ds3_gyro_init_info;
extern int lsm6ds3_gyro_init_flag;

int lsm6ds3_pedo_interrupt_step_d_handler(struct lsm6ds3_pedo *pedo_obj);
int lsm6ds3_pedo_interrupt_sigmotion_handler(struct lsm6ds3_pedo *pedo_obj);
int lsm6ds3_pedo_interrupt_tilt_handler(struct lsm6ds3_pedo *pedo_obj);
extern struct step_c_init_info lsm6ds3_pedo_init_info;
extern int lsm6ds3_pedo_init_flag;
#endif
