
#include "lsm6ds3.h"


/*----------------------------------------------------------------------------*/

static struct data_resolution lsm6ds3_acc_data_resolution[] = {
     /* combination by {FULL_RES,RANGE}*/
    {{ 0, 0}, 16393},     // dataformat +/-2g   in 16-bit resolution;  { 1, 0} = 1.0 = (2*2*1000)/(2^16);  1024 = (2^16)/(2*2) 
	{{ 0, 0}, 2049},    // dataformat +/-16g  in 16-bit resolution;  { 3, 9} = 3.9 = (2*16*1000)/(2^16);  128 = (2^16)/(2*16) 
    {{ 0, 0}, 8197},     // dataformat +/-4g   in 16-bit resolution;  { 1, 9} = 1.9 = (2*4*1000)/(2^16);  512 = (2^16)/(2*4) 
    {{ 0, 0}, 4098},     // dataformat +/-8g   in 16-bit resolution;  { 3, 9} = 3.9 = (2*8*1000)/(2^16);  256 = (2^16)/(2*8) 
};

/*----------------------------------------------------------------------------*/
static struct data_resolution lsm6ds3_offset_resolution = {{15, 6}, 64};
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
struct acc_hw lsm6ds3_acc_cust_hw;
int lsm6ds3_acc_init_flag =-1; // 0<==>OK -1 <==> fail
/*----------------------------------------------------------------------------*/


/*For driver get cust info*/
struct acc_hw *lsm6ds3_get_cust_acc_hw(void)
{
    return &lsm6ds3_acc_cust_hw;
}

/*----------------------------------------------------------------------------*/

static int lsm6ds3_acc_set_resolution(struct lsm6ds3_acc *acc_obj)
{
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;
    
    int err;
    u8  dat, reso;

    err = lsm6ds3_i2c_read_block(client, LSM6DS3_REG_CTRL1_XL, &dat, 0x01);

    if(err < 0)
    {
        ST_ERR("write data format fail!!\n");
        return err;
    }

    /*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
    reso = (dat & LSM6DS3_REG_CTRL1_XL_MASK_FS_XL)>>2;
    if(reso >= 0x3)
        reso = 0x3;
    

    if(reso < sizeof(lsm6ds3_acc_data_resolution)/sizeof(lsm6ds3_acc_data_resolution[0]))
    {        
        acc_obj->reso = &lsm6ds3_acc_data_resolution[reso];
        return 0;
    }
    else
    {
        return -EINVAL;
    }
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_read_rawdata(struct lsm6ds3_acc *acc_obj, s16 data[LSM6DS3_AXES_NUM])
{
    
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;
    
    u8 buf[LSM6DS3_DATA_LEN] = {0};
    int err = 0;

    if(NULL == client)
    {
        err = -EINVAL;
    }
    
    else
    {
        if((lsm6ds3_i2c_read_block(client, LSM6DS3_REG_OUTX_L_XL, buf, 0x06))<0)
        {
           ST_ERR("read  G sensor data register err!\n");
             return -1;
        }
		
		data[LSM6DS3_AXIS_X] = (s16)((buf[LSM6DS3_AXIS_X*2+1] << 8) | (buf[LSM6DS3_AXIS_X*2]));
		data[LSM6DS3_AXIS_Y] = (s16)((buf[LSM6DS3_AXIS_Y*2+1] << 8) | (buf[LSM6DS3_AXIS_Y*2]));
		data[LSM6DS3_AXIS_Z] = (s16)((buf[LSM6DS3_AXIS_Z*2+1] << 8) | (buf[LSM6DS3_AXIS_Z*2]));	

        if(atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA)
        {
            ST_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z],
                                       data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z]);
        }

        if(atomic_read(&acc_obj->trace) & ADX_TRC_RAWDATA)
        {
            ST_LOG("[%08X %08X %08X] => [%5d %5d %5d] after\n", data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z],
                                       data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z]);
        }
        
#ifdef CONFIG_LSM6DS3_LOWPASS
        if(atomic_read(&acc_obj->filter))
        {
            if(atomic_read(&acc_obj->fir_en) && !atomic_read(&acc_obj->suspend))
            {
                int idx, firlen = atomic_read(&acc_obj->firlen);   
                if(acc_obj->fir.num < firlen)
                {                
                    acc_obj->fir.raw[acc_obj->fir.num][LSM6DS3_AXIS_X] = data[LSM6DS3_AXIS_X];
                    acc_obj->fir.raw[acc_obj->fir.num][LSM6DS3_AXIS_Y] = data[LSM6DS3_AXIS_Y];
                    acc_obj->fir.raw[acc_obj->fir.num][LSM6DS3_AXIS_Z] = data[LSM6DS3_AXIS_Z];
                    acc_obj->fir.sum[LSM6DS3_AXIS_X] += data[LSM6DS3_AXIS_X];
                    acc_obj->fir.sum[LSM6DS3_AXIS_Y] += data[LSM6DS3_AXIS_Y];
                    acc_obj->fir.sum[LSM6DS3_AXIS_Z] += data[LSM6DS3_AXIS_Z];
                    if(atomic_read(&acc_obj->trace) & ADX_TRC_FILTER)
                    {
                        ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", acc_obj->fir.num,
                            acc_obj->fir.raw[acc_obj->fir.num][LSM6DS3_AXIS_X], acc_obj->fir.raw[acc_obj->fir.num][LSM6DS3_AXIS_Y], acc_obj->fir.raw[acc_obj->fir.num][LSM6DS3_AXIS_Z],
                            acc_obj->fir.sum[LSM6DS3_AXIS_X], acc_obj->fir.sum[LSM6DS3_AXIS_Y], acc_obj->fir.sum[LSM6DS3_AXIS_Z]);
                    }
                    acc_obj->fir.num++;
                    acc_obj->fir.idx++;
                }
                else
                {
                    idx = acc_obj->fir.idx % firlen;
                    acc_obj->fir.sum[LSM6DS3_AXIS_X] -= acc_obj->fir.raw[idx][LSM6DS3_AXIS_X];
                    acc_obj->fir.sum[LSM6DS3_AXIS_Y] -= acc_obj->fir.raw[idx][LSM6DS3_AXIS_Y];
                    acc_obj->fir.sum[LSM6DS3_AXIS_Z] -= acc_obj->fir.raw[idx][LSM6DS3_AXIS_Z];
                    acc_obj->fir.raw[idx][LSM6DS3_AXIS_X] = data[LSM6DS3_AXIS_X];
                    acc_obj->fir.raw[idx][LSM6DS3_AXIS_Y] = data[LSM6DS3_AXIS_Y];
                    acc_obj->fir.raw[idx][LSM6DS3_AXIS_Z] = data[LSM6DS3_AXIS_Z];
                    acc_obj->fir.sum[LSM6DS3_AXIS_X] += data[LSM6DS3_AXIS_X];
                    acc_obj->fir.sum[LSM6DS3_AXIS_Y] += data[LSM6DS3_AXIS_Y];
                    acc_obj->fir.sum[LSM6DS3_AXIS_Z] += data[LSM6DS3_AXIS_Z];
                    acc_obj->fir.idx++;
                    data[LSM6DS3_AXIS_X] = acc_obj->fir.sum[LSM6DS3_AXIS_X]/firlen;
                    data[LSM6DS3_AXIS_Y] = acc_obj->fir.sum[LSM6DS3_AXIS_Y]/firlen;
                    data[LSM6DS3_AXIS_Z] = acc_obj->fir.sum[LSM6DS3_AXIS_Z]/firlen;
                    if(atomic_read(&acc_obj->trace) & ADX_TRC_FILTER)
                    {
                        ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
                        acc_obj->fir.raw[idx][LSM6DS3_AXIS_X], acc_obj->fir.raw[idx][LSM6DS3_AXIS_Y], acc_obj->fir.raw[idx][LSM6DS3_AXIS_Z],
                        acc_obj->fir.sum[LSM6DS3_AXIS_X], acc_obj->fir.sum[LSM6DS3_AXIS_Y], acc_obj->fir.sum[LSM6DS3_AXIS_Z],
                        data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z]);
                    }
                }
            }
        }    
#endif
    }
    return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_reset_calibration(struct lsm6ds3_acc *acc_obj)
{
    memset(acc_obj->cali_sw, 0x00, sizeof(acc_obj->cali_sw));
    return 0;     
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_read_calibration(struct lsm6ds3_acc *acc_obj, int dat[LSM6DS3_AXES_NUM])
{
    dat[acc_obj->cvt.map[LSM6DS3_AXIS_X]] = acc_obj->cvt.sign[LSM6DS3_AXIS_X]*acc_obj->cali_sw[LSM6DS3_AXIS_X];
    dat[acc_obj->cvt.map[LSM6DS3_AXIS_Y]] = acc_obj->cvt.sign[LSM6DS3_AXIS_Y]*acc_obj->cali_sw[LSM6DS3_AXIS_Y];
    dat[acc_obj->cvt.map[LSM6DS3_AXIS_Z]] = acc_obj->cvt.sign[LSM6DS3_AXIS_Z]*acc_obj->cali_sw[LSM6DS3_AXIS_Z];            
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_write_calibration(struct lsm6ds3_acc *acc_obj, int dat[LSM6DS3_AXES_NUM])
{
    int err = 0;

    ST_FUN();
    if(!acc_obj || !dat)
    {
        ST_ERR("null ptr!!\n");
        return -EINVAL;
    }
    else
    {        
        s16 cali[LSM6DS3_AXES_NUM];
        cali[acc_obj->cvt.map[LSM6DS3_AXIS_X]] = acc_obj->cvt.sign[LSM6DS3_AXIS_X]*acc_obj->cali_sw[LSM6DS3_AXIS_X];
        cali[acc_obj->cvt.map[LSM6DS3_AXIS_Y]] = acc_obj->cvt.sign[LSM6DS3_AXIS_Y]*acc_obj->cali_sw[LSM6DS3_AXIS_Y];
        cali[acc_obj->cvt.map[LSM6DS3_AXIS_Z]] = acc_obj->cvt.sign[LSM6DS3_AXIS_Z]*acc_obj->cali_sw[LSM6DS3_AXIS_Z]; 
		
        cali[LSM6DS3_AXIS_X] += dat[LSM6DS3_AXIS_X];
        cali[LSM6DS3_AXIS_Y] += dat[LSM6DS3_AXIS_Y];
        cali[LSM6DS3_AXIS_Z] += dat[LSM6DS3_AXIS_Z];

        acc_obj->cali_sw[LSM6DS3_AXIS_X] += acc_obj->cvt.sign[LSM6DS3_AXIS_X]*dat[acc_obj->cvt.map[LSM6DS3_AXIS_X]];
        acc_obj->cali_sw[LSM6DS3_AXIS_Y] += acc_obj->cvt.sign[LSM6DS3_AXIS_Y]*dat[acc_obj->cvt.map[LSM6DS3_AXIS_Y]];
        acc_obj->cali_sw[LSM6DS3_AXIS_Z] += acc_obj->cvt.sign[LSM6DS3_AXIS_Z]*dat[acc_obj->cvt.map[LSM6DS3_AXIS_Z]];
    } 

    return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_set_full_scale(struct lsm6ds3_acc *acc_obj, u8 dataformat)
{
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;
    u8 buf[10];
    u8 reg = LSM6DS3_REG_CTRL1_XL;
    int res = 0;

    memset(buf, 0, sizeof(u8)*10);

    if((lsm6ds3_i2c_read_block(client, reg, buf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    buf[0] &= ~LSM6DS3_REG_CTRL1_XL_MASK_FS_XL;
    buf[0] |= dataformat;

    res = lsm6ds3_i2c_write_block(client, reg, buf, 0x1);

    if(res < 0)
    {
        return LSM6DS3_ERR_I2C;
    }
	return lsm6ds3_acc_set_resolution(acc_obj);
    
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_set_odr(struct lsm6ds3_acc *acc_obj, u8 odr)
{
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;

    u8 buf[10];
    u8 reg = LSM6DS3_REG_CTRL1_XL;
    int res = 0;

    memset(buf, 0, sizeof(u8)*10);

    reg = LSM6DS3_REG_CTRL1_XL;
    if((lsm6ds3_i2c_read_block(client, reg, buf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    buf[0] &= ~LSM6DS3_REG_CTRL1_XL_MASK_ODR_XL;
    buf[0] |= odr;

    res = lsm6ds3_i2c_write_block(client, reg, buf, 0x1);

    if(res < 0)
    {
        return LSM6DS3_ERR_I2C;
    }
    		
    return LSM6DS3_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
int lsm6ds3_acc_set_power_mode(struct lsm6ds3_acc *acc_obj, bool state)
{ 
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;

    int res = 0;
    u8 addr = 0;
    u8 buf[2] = {0};

    if(state == acc_obj->lsm6ds3_acc_power)
    {
        ST_LOG("Sensor power status is newest!\n");
        return LSM6DS3_SUCCESS;
    }

    if(state == true)
    {
		if(acc_obj->odr == 0)
		{
			acc_obj->odr = LSM6DS3_REG_CTRL1_XL_ODR_104HZ;
		}
		res = lsm6ds3_acc_set_odr(acc_obj, acc_obj->odr);
    }
    else if(state == false)
    {
        addr = LSM6DS3_REG_TAP_CFG;
        
        res = lsm6ds3_i2c_read_block(client, addr, buf, 0x01);
        
        if(res)
        {
            ST_ERR("read LSM6DS3_REG_TAP_CFG register err!\n");
            return LSM6DS3_ERR_I2C;
        }

        // read pedometer status
        if (buf[0]&LSM6DS3_REG_TAP_CFG_MASK_PEDO_EN)
        {
            res = lsm6ds3_acc_set_odr(acc_obj, LSM6DS3_REG_CTRL1_XL_ODR_26HZ);
        }
        else
        {
            res = lsm6ds3_acc_set_odr(acc_obj, LSM6DS3_REG_CTRL1_XL_ODR_0HZ);
        }
        
    }
	else
	{
		ST_ERR("set power state error!\n");
		return LSM6DS3_ERR_SETUP_FAILURE;
	}
	
    if(res < 0)
    {
        ST_ERR("set power mode failed!\n");
        return LSM6DS3_ERR_I2C;
    }
    else if(atomic_read(&acc_obj->trace) & ADX_TRC_INFO)
    {
        ST_LOG("set power mode ok %d!\n", state);
    }
	
	acc_obj->lsm6ds3_acc_power = state;
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
int lsm6ds3_acc_init(struct lsm6ds3_acc *acc_obj, int reset_cali)
{
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;

    int res = 0;
    u8 buf[2] = {0, 0};
    ST_FUN();
    
    buf[0] = 0x00;
    res = lsm6ds3_i2c_write_block(client, LSM6DS3_REG_CTRL1_XL, buf, 0x01);
    if(res < 0)
    {
        ST_ERR("lsm6ds3_acc_init step 1!\n");
        return res;
    }
    
    acc_obj->odr = 0;
    res = lsm6ds3_acc_set_odr(acc_obj, LSM6DS3_REG_CTRL1_XL_ODR_0HZ);//power down
    if(res < 0)
    {
        ST_ERR("lsm6ds3_acc_init step 2!\n");
        return res;
    }

    res = lsm6ds3_acc_set_full_scale(acc_obj, LSM6DS3_REG_CTRL1_XL_FS_4G);
    if(res < 0) 
    {
        ST_ERR("lsm6ds3_acc_init step 3!\n");
        return res;
    }
	
    gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = acc_obj->reso->sensitivity;

    if(0 != reset_cali)
    { 
        //reset calibration only in power on
        res = lsm6ds3_acc_reset_calibration(acc_obj);
        if(res < 0)
        {
            return res;
        }
    }

#ifdef CONFIG_LSM6DS3_LOWPASS
    memset(&acc_obj->fir, 0x00, sizeof(acc_obj->fir));
#endif
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_read_chip_name(struct lsm6ds3_acc *acc_obj, u8 *buf, int bufsize)
{
    sprintf(buf, "%s", acc_obj->name);
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_read_data(struct lsm6ds3_acc *acc_obj, u8 *data, int bufsize)
{
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;

    //u8 databuf[20];
    int acc[LSM6DS3_AXES_NUM];
    int res = 0;
    //memset(databuf, 0, sizeof(u8)*10);

    if(NULL == data)
    {
        return LSM6DS3_ERR_SETUP_FAILURE;
    }
    if(NULL == client)
    {
        *data = 0;
        return LSM6DS3_ERR_SETUP_FAILURE;
    }

    if(atomic_read(&acc_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }
	
    if((res = lsm6ds3_acc_read_rawdata(acc_obj, acc_obj->data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return res;
    }
    else
    {
        acc_obj->data[LSM6DS3_AXIS_X] += acc_obj->cali_sw[LSM6DS3_AXIS_X];
        acc_obj->data[LSM6DS3_AXIS_Y] += acc_obj->cali_sw[LSM6DS3_AXIS_Y];
        acc_obj->data[LSM6DS3_AXIS_Z] += acc_obj->cali_sw[LSM6DS3_AXIS_Z];
        
        /*remap coordinate*/
        acc[acc_obj->cvt.map[LSM6DS3_AXIS_X]] = acc_obj->cvt.sign[LSM6DS3_AXIS_X]*acc_obj->data[LSM6DS3_AXIS_X];
        acc[acc_obj->cvt.map[LSM6DS3_AXIS_Y]] = acc_obj->cvt.sign[LSM6DS3_AXIS_Y]*acc_obj->data[LSM6DS3_AXIS_Y];
        acc[acc_obj->cvt.map[LSM6DS3_AXIS_Z]] = acc_obj->cvt.sign[LSM6DS3_AXIS_Z]*acc_obj->data[LSM6DS3_AXIS_Z];

        //ST_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);

        //Out put the mg
        acc[LSM6DS3_AXIS_X] = acc[LSM6DS3_AXIS_X] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
        acc[LSM6DS3_AXIS_Y] = acc[LSM6DS3_AXIS_Y] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
        acc[LSM6DS3_AXIS_Z] = acc[LSM6DS3_AXIS_Z] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
        

        sprintf(data, "%04x %04x %04x", acc[LSM6DS3_AXIS_X], acc[LSM6DS3_AXIS_Y], acc[LSM6DS3_AXIS_Z]);
        if(atomic_read(&acc_obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
        {
            ST_LOG("gsensor data: %s!\n", data);
            dumpReg(obj);
        }
    }
    
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_read_rawdata_string(struct lsm6ds3_acc *acc_obj, u8 *buf)
{
    struct lsm6ds3_data *obj = container_of(acc_obj, struct lsm6ds3_data, lsm6ds3_acc_data);
	struct i2c_client *client = obj->client;
    int res = 0;

    if (!buf || !client)
    {
        return LSM6DS3_ERR_SETUP_FAILURE;
    }
    
    if((res = lsm6ds3_acc_read_rawdata(acc_obj, acc_obj->data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return res;
    }
    else
    {
        sprintf(buf, "%04x %04x %04x", acc_obj->data[LSM6DS3_AXIS_X], 
        acc_obj->data[LSM6DS3_AXIS_Y], acc_obj->data[LSM6DS3_AXIS_Z]);
    }
    
    return LSM6DS3_SUCCESS;
}


/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    //struct i2c_client *client = lsm6ds3_i2c_client;
    struct lsm6ds3_data *obj = obj_i2c_data;
	struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
    u8 strbuf[LSM6DS3_BUFSIZE];
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    
    lsm6ds3_acc_read_chip_name(acc_obj, strbuf, LSM6DS3_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    return snprintf(buf, PAGE_SIZE, "0x%x\n", obj->chip_id);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
    u8 databuf[LSM6DS3_BUFSIZE];
    
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    lsm6ds3_acc_read_data(acc_obj, databuf, LSM6DS3_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_rawdata_value(struct device_driver *ddri, char *buf)
{   
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
    u8 databuf[LSM6DS3_BUFSIZE];
    
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    lsm6ds3_acc_read_rawdata_string(acc_obj, databuf);
    return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_cali_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 
    int err, len, mul;
    int tmp[LSM6DS3_AXES_NUM];    
    len = 0;

    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    if((err = lsm6ds3_acc_read_calibration(acc_obj, tmp)))
    {
        return -EINVAL;
    }
    else
    {    
        mul = acc_obj->reso->sensitivity/lsm6ds3_offset_resolution.sensitivity;
        len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
            acc_obj->offset[LSM6DS3_AXIS_X], acc_obj->offset[LSM6DS3_AXIS_Y], acc_obj->offset[LSM6DS3_AXIS_Z],
            acc_obj->offset[LSM6DS3_AXIS_X], acc_obj->offset[LSM6DS3_AXIS_Y], acc_obj->offset[LSM6DS3_AXIS_Z]);
        len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
            acc_obj->cali_sw[LSM6DS3_AXIS_X], acc_obj->cali_sw[LSM6DS3_AXIS_Y], acc_obj->cali_sw[LSM6DS3_AXIS_Z]);

        len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
            acc_obj->offset[LSM6DS3_AXIS_X]*mul + acc_obj->cali_sw[LSM6DS3_AXIS_X],
            acc_obj->offset[LSM6DS3_AXIS_Y]*mul + acc_obj->cali_sw[LSM6DS3_AXIS_Y],
            acc_obj->offset[LSM6DS3_AXIS_Z]*mul + acc_obj->cali_sw[LSM6DS3_AXIS_Z],
            tmp[LSM6DS3_AXIS_X], tmp[LSM6DS3_AXIS_Y], tmp[LSM6DS3_AXIS_Z]);
        
        return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{ 
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 
    int err, x, y, z;
    int dat[LSM6DS3_AXES_NUM];

    if(!strncmp(buf, "rst", 3))
    {
        if((err = lsm6ds3_acc_reset_calibration(acc_obj)))
        {
            ST_ERR("reset offset err = %d\n", err);
        }    
    }
    else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
    {
        dat[LSM6DS3_AXIS_X] = x;
        dat[LSM6DS3_AXIS_Y] = y;
        dat[LSM6DS3_AXIS_Z] = z;
        if((err = lsm6ds3_acc_write_calibration(acc_obj, dat)))
        {
            ST_ERR("write calibration err = %d\n", err);
        }        
    }
    else
    {
        ST_ERR("invalid format\n");
    }
    
    return count;
}
/*----------------------------------------------------------------------------*/

static ssize_t lsm6ds3_attr_acc_show_power_status(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct i2c_client *client = obj->client;

    u8 data;

    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    lsm6ds3_i2c_read_block(client, LSM6DS3_REG_CTRL1_XL, &data, 0x01);
    return snprintf(buf, PAGE_SIZE, "%x\n", data);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LSM6DS3_LOWPASS
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 
    if(atomic_read(&acc_obj->firlen))
    {
        int idx, len = atomic_read(&acc_obj->firlen);
        ST_LOG("len = %2d, idx = %2d\n", acc_obj->fir.num, acc_obj->fir.idx);

        for(idx = 0; idx < len; idx++)
        {
            ST_LOG("[%5d %5d %5d]\n", acc_obj->fir.raw[idx][LSM6DS3_AXIS_X], acc_obj->fir.raw[idx][LSM6DS3_AXIS_Y], acc_obj->fir.raw[idx][LSM6DS3_AXIS_Z]);
        }
        
        ST_LOG("sum = [%5d %5d %5d]\n", acc_obj->fir.sum[LSM6DS3_AXIS_X], acc_obj->fir.sum[LSM6DS3_AXIS_Y], acc_obj->fir.sum[LSM6DS3_AXIS_Z]);
        ST_LOG("avg = [%5d %5d %5d]\n", acc_obj->fir.sum[LSM6DS3_AXIS_X]/len, acc_obj->fir.sum[LSM6DS3_AXIS_Y]/len, acc_obj->fir.sum[LSM6DS3_AXIS_Z]/len);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&acc_obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LSM6DS3_LOWPASS
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 
    int firlen;

    if(1 != sscanf(buf, "%d", &firlen))
    {
        ST_ERR("invallid format\n");
    }
    else if(firlen > C_MAX_FIR_LENGTH)
    {
        ST_ERR("exceeds maximum filter length\n");
    }
    else
    { 
        atomic_set(&acc_obj->firlen, firlen);
        if(0 == firlen)
        {
            atomic_set(&acc_obj->fir_en, 0);
        }
        else
        {
            memset(&acc_obj->fir, 0x00, sizeof(acc_obj->fir));
            atomic_set(&acc_obj->fir_en, 1);
        }
    }
#endif    
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 
    int trace;
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&acc_obj->trace, trace);
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }
    
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_status_value(struct device_driver *ddri, char *buf)
{    
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data; 
    ssize_t len = 0;
    
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }    
    
    if(acc_obj->lsm6ds3_acc_hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
                acc_obj->lsm6ds3_acc_hw->i2c_num, acc_obj->lsm6ds3_acc_hw->direction, acc_obj->reso->sensitivity, acc_obj->lsm6ds3_acc_hw->power_id, acc_obj->lsm6ds3_acc_hw->power_vol);   
        dumpReg(obj);
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&acc_obj->trace)); 
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lsm6ds3_acc_init(acc_obj, 0);
    dumpReg(obj);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_show_layout_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;

    if(NULL == obj)
    {
        ST_LOG("lsm6ds3_acc is null!!\n");
        return -1;
    }

    return sprintf(buf, "(%d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
        acc_obj->lsm6ds3_acc_hw->direction, acc_obj->cvt.sign[0], acc_obj->cvt.sign[1],
        acc_obj->cvt.sign[2], acc_obj->cvt.map[0], acc_obj->cvt.map[1], acc_obj->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_acc_store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;

    int layout = 0;

    if(NULL == obj)
    {
        ST_ERR("lsm6ds3_acc is null!!\n");
        return count;
    }

    if(1 == sscanf(buf, "%d", &layout))
    {
        if(!hwmsen_get_convert(layout, &acc_obj->cvt))
        {
            ST_ERR("HWMSEN_GET_CONVERT function error!\r\n");
        }
        else if(!hwmsen_get_convert(acc_obj->lsm6ds3_acc_hw->direction, &acc_obj->cvt))
        {
            ST_LOG("invalid layout: %d, restore to %d\n", layout, acc_obj->lsm6ds3_acc_hw->direction);
        }
        else
        {
            ST_ERR("invalid layout: (%d, %d)\n", layout, acc_obj->lsm6ds3_acc_hw->direction);
            hwmsen_get_convert(0, &acc_obj->cvt);
        }
    }
    else
    {
        ST_LOG("invalid format = '%s'\n", buf);
    }

    return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, lsm6ds3_attr_acc_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lsm6ds3_attr_acc_show_chipid_value,        NULL);
static DRIVER_ATTR(rawdata,              S_IRUGO, lsm6ds3_attr_acc_show_rawdata_value,       NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lsm6ds3_attr_acc_show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, lsm6ds3_attr_acc_show_cali_value,          lsm6ds3_attr_acc_store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, lsm6ds3_attr_acc_show_power_status,        NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, lsm6ds3_attr_acc_show_firlen_value,        lsm6ds3_attr_acc_store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lsm6ds3_attr_acc_show_trace_value,         lsm6ds3_attr_acc_store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, lsm6ds3_attr_acc_show_status_value,        NULL);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lsm6ds3_attr_acc_show_chipinit_value,      lsm6ds3_attr_acc_store_chipinit_value);
static DRIVER_ATTR(layout,     S_IRUGO | S_IWUSR, lsm6ds3_attr_acc_show_layout_value,        lsm6ds3_attr_acc_store_layout_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *lsm6ds3_attr_acc_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_rawdata,      /*dump sensor raw data*/
    &driver_attr_cali,         /*show calibration data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,
    &driver_attr_chipinit,
    &driver_attr_layout,
};
/*----------------------------------------------------------------------------*/
int lsm6ds3_acc_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_acc_list)/sizeof(lsm6ds3_attr_acc_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, lsm6ds3_attr_acc_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6ds3_attr_acc_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
int lsm6ds3_acc_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_acc_list)/sizeof(lsm6ds3_attr_acc_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }
    

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lsm6ds3_attr_acc_list[idx]);
    }
    

    return err;
}

/*----------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lsm6ds3_acc_open_report_data_intf(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lsm6ds3_acc_enable_nodata_intf(int en)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
    int res =0;
    bool power = false;
    
    if(1==en)
    {
        power = true;
    }
    if(0==en)
    {
        power = false;
    }
	acc_obj->enabled = en;
    res = lsm6ds3_acc_set_power_mode(acc_obj, power);
    if(res != LSM6DS3_SUCCESS)
    {
        ST_ERR("lsm6ds3_acc_set_power_mode fail!\n");
        return res;
    }
    ST_LOG("lsm6ds3_acc_enable_nodata_intf OK!\n");
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_acc_set_delay_intf(u64 ns)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;

    int ms =0;
    int odr=0;
    int err;
	
    ms = (int)ns/1000/1000;
    if(ms <= 5)
    {
        odr = LSM6DS3_REG_CTRL1_XL_ODR_208HZ;
    }
    else if(ms <= 10)
    {
        odr = LSM6DS3_REG_CTRL1_XL_ODR_104HZ;
    }
    else
    {
        odr = LSM6DS3_REG_CTRL1_XL_ODR_52HZ;
    }

	acc_obj->odr = odr;
	err = lsm6ds3_acc_set_odr(acc_obj, acc_obj->odr);
    if(err != LSM6DS3_SUCCESS ) //0x2C->BW=100Hz
    {
        ST_ERR("Set delay parameter error!\n");
    }

    if(ms >= 50)
    {
        atomic_set(&acc_obj->filter, 0);
    }
    else
    {                    
        acc_obj->fir.num = 0;
        acc_obj->fir.idx = 0;
        acc_obj->fir.sum[LSM6DS3_AXIS_X] = 0;
        acc_obj->fir.sum[LSM6DS3_AXIS_Y] = 0;
        acc_obj->fir.sum[LSM6DS3_AXIS_Z] = 0;
        atomic_set(&acc_obj->filter, 1);
    }
    
    ST_LOG("lsm6ds3_acc_set_delay_intf (%d)\n",ms);
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_acc_get_data_intf(int* x ,int* y,int* z, int* status)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;

    u8 buff[LSM6DS3_BUFSIZE];
    lsm6ds3_acc_read_data(acc_obj, buff, LSM6DS3_BUFSIZE);
    
    sscanf(buff, "%x %x %x", x, y, z);        
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_acc_open(struct inode *inode, struct file *file)
{
    file->private_data = obj_i2c_data;

    if(file->private_data == NULL)
    {
        ST_ERR("null pointer!!\n");
        return -EINVAL;
    }
    return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long lsm6ds3_acc_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)

{
    struct lsm6ds3_data *obj = (struct lsm6ds3_data*)file->private_data;
	struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
	struct SENSOR_DATA sensor_data;
    
	u8 strbuf[LSM6DS3_BUFSIZE];
    void __user *data;
    long err = 0;
    int cali[3];

    //ST_FUN(f);
    if(_IOC_DIR(cmd) & _IOC_READ)
    {
        err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
    }
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
    {
        err = !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
    }

    if(err)
    {
        ST_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
        return -EFAULT;
    }

    switch(cmd)
    {
        case GSENSOR_IOCTL_INIT:
            lsm6ds3_acc_init(acc_obj, 0);            
            break;

        case GSENSOR_IOCTL_READ_CHIPINFO:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            
            lsm6ds3_acc_read_chip_name(acc_obj, strbuf, LSM6DS3_BUFSIZE);
            if(copy_to_user(data, strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;
            }                 
            break;      

        case GSENSOR_IOCTL_READ_SENSORDATA:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            lsm6ds3_acc_set_power_mode(acc_obj, true);
            lsm6ds3_acc_read_data(acc_obj, strbuf, LSM6DS3_BUFSIZE);
            if(copy_to_user(data, strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;      
            }                 
            break;

        case GSENSOR_IOCTL_READ_GAIN:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }            
            
            if(copy_to_user(data, &gsensor_gain, sizeof(struct GSENSOR_VECTOR3D)))
            {
                err = -EFAULT;
                break;
            }                 
            break;

        case GSENSOR_IOCTL_READ_OFFSET:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            
            if(copy_to_user(data, &gsensor_offset, sizeof(struct GSENSOR_VECTOR3D)))
            {
                err = -EFAULT;
                break;
            }                 
            break;

        case GSENSOR_IOCTL_READ_RAW_DATA:
            data = (void __user *) arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            lsm6ds3_acc_read_rawdata_string(acc_obj, strbuf);
            if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
            {
                err = -EFAULT;
                break;      
            }
            break;      

        case GSENSOR_IOCTL_SET_CALI:
            data = (void __user*)arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            if(copy_from_user(&sensor_data, data, sizeof(sensor_data)))
            {
                err = -EFAULT;
                break;      
            }
            if(atomic_read(&acc_obj->suspend))
            {
                ST_ERR("Perform calibration in suspend state!!\n");
                err = -EINVAL;
            }
            else
            {
                cali[LSM6DS3_AXIS_X] = sensor_data.x * acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
                cali[LSM6DS3_AXIS_Y] = sensor_data.y * acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;
                cali[LSM6DS3_AXIS_Z] = sensor_data.z * acc_obj->reso->sensitivity / GRAVITY_EARTH_1000;              
                err = lsm6ds3_acc_write_calibration(acc_obj, cali);             
            }
            break;

        case GSENSOR_IOCTL_CLR_CALI:
            err = lsm6ds3_acc_reset_calibration(acc_obj);
            break;

        case GSENSOR_IOCTL_GET_CALI:
            data = (void __user*)arg;
            if(data == NULL)
            {
                err = -EINVAL;
                break;      
            }
            if((err = lsm6ds3_acc_read_calibration(acc_obj, cali)))
            {
                break;
            }
            
            sensor_data.x = cali[LSM6DS3_AXIS_X] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
            sensor_data.y = cali[LSM6DS3_AXIS_Y] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
            sensor_data.z = cali[LSM6DS3_AXIS_Z] * GRAVITY_EARTH_1000 / acc_obj->reso->sensitivity;
            if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
            {
                err = -EFAULT;
                break;
            }        
            break;

        default:
            ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
            break;
    }

    return err;
}

#ifdef CONFIG_COMPAT
static long lsm6ds3_acc_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long err = 0;
    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL) {
                err = -EINVAL;
                break;
            }
        
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, (unsigned long)arg32);
            if (err) {
                ST_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
                return err;
            }
            break;
            
        case COMPAT_GSENSOR_IOCTL_SET_CALI:
            if (arg32 == NULL) {
                err = -EINVAL;
                break;
            }
        
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_SET_CALI, (unsigned long)arg32);
            if (err) {
                ST_ERR("GSENSOR_IOCTL_SET_CALI unlocked_ioctl failed.");
                return err;
            }
            break;
            
        case COMPAT_GSENSOR_IOCTL_GET_CALI:
            if (arg32 == NULL) {
                err = -EINVAL;
                break;
            }
        
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_GET_CALI, (unsigned long)arg32);
            if (err) {
                ST_ERR("GSENSOR_IOCTL_GET_CALI unlocked_ioctl failed.");
                return err;
            }
            break;
            
        case COMPAT_GSENSOR_IOCTL_CLR_CALI:
            if (arg32 == NULL) {
                err = -EINVAL;
                break;
            }
        
            err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_CLR_CALI, (unsigned long)arg32);
            if (err) {
                ST_ERR("GSENSOR_IOCTL_CLR_CALI unlocked_ioctl failed.");
                return err;
            }
            break;

        default:
            ST_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;
    }

    return err;
}
#endif


static struct file_operations lsm6ds3_acc_fops = {
    .owner = THIS_MODULE,
    .open = lsm6ds3_acc_open,
    .release = lsm6ds3_acc_release,
    .unlocked_ioctl = lsm6ds3_acc_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = lsm6ds3_acc_compat_ioctl,
#endif
};

static struct miscdevice lsm6ds3_acc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gsensor",
    .fops = &lsm6ds3_acc_fops,
};


/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_local_init(void)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
    int err = 0;
    int retry = 0;
    struct acc_control_path ctl={0};
    struct acc_data_path data={0};    
    const u8 *name = "mediatek,lsm6ds3_acc";	
    ST_FUN();

    acc_obj->lsm6ds3_acc_hw = get_accel_dts_func(name, &lsm6ds3_acc_cust_hw);

    if (!acc_obj->lsm6ds3_acc_hw) {
        ST_ERR("get lsm6ds3 dts info failed\n");
    }

    if((err = hwmsen_get_convert(acc_obj->lsm6ds3_acc_hw->direction, &acc_obj->cvt)))
    {
        ST_ERR("invalid direction: %d\n", acc_obj->lsm6ds3_acc_hw->direction);
        goto exit;
    }

    atomic_set(&acc_obj->trace, 0);
    atomic_set(&acc_obj->suspend, 0);
    
#ifdef CONFIG_LSM6DS3_LOWPASS
    if(acc_obj->lsm6ds3_acc_hw->firlen > C_MAX_FIR_LENGTH)
    {
        atomic_set(&acc_obj->firlen, C_MAX_FIR_LENGTH);
    }    
    else
    {
        atomic_set(&acc_obj->firlen, acc_obj->lsm6ds3_acc_hw->firlen);
    }
    
    if(atomic_read(&acc_obj->firlen) > 0)
    {
        atomic_set(&acc_obj->fir_en, 1);
    }
#endif

    for(retry = 0; retry < 3; retry++){
        if((err = lsm6ds3_acc_init(acc_obj, 1)))
        {
            ST_ERR("lsm6ds3_acc_device init cilent fail time: %d\n", retry);
            continue;
        }
    }
    if(err != 0)
        goto exit_init_failed;

	sprintf(acc_obj->name, "%s_ACC", obj->name);
	
    if((err = misc_register(&lsm6ds3_acc_device)))
    {
        ST_ERR("lsm6ds3_acc_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if((err = lsm6ds3_acc_create_attr(&(lsm6ds3_acc_init_info.platform_diver_addr->driver))))
    {
        ST_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    ctl.is_use_common_factory = false;
    ctl.open_report_data= lsm6ds3_acc_open_report_data_intf;
    ctl.enable_nodata = lsm6ds3_acc_enable_nodata_intf;
    ctl.set_delay  = lsm6ds3_acc_set_delay_intf;
    ctl.is_report_input_direct = false;
    
    err = acc_register_control_path(&ctl);
    if(err)
    {
        ST_ERR("register acc control path err\n");
        goto exit_kfree;
    }

    data.get_data = lsm6ds3_acc_get_data_intf;
    data.vender_div = 1000;
    err = acc_register_data_path(&data);
    if(err) {
        ST_ERR("register acc data path err\n");
        goto exit_kfree;
    }

    ST_LOG("%s: OK\n", __func__);
    lsm6ds3_acc_init_flag = 0;    
    return 0;

exit_create_attr_failed:
    misc_deregister(&lsm6ds3_acc_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
    kfree(obj);
exit:
    ST_ERR("%s: err = %d\n", __func__, err);
    lsm6ds3_acc_init_flag = -1;        
    return lsm6ds3_acc_init_flag;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_acc_local_remove(void)
{
    ST_FUN(); 
    misc_deregister(&lsm6ds3_acc_device);
    lsm6ds3_acc_delete_attr(&(lsm6ds3_acc_init_info.platform_diver_addr->driver));
    
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct acc_init_info lsm6ds3_acc_init_info = {
        .name = "lsm6ds3_acc",
        .init = lsm6ds3_acc_local_init,
        .uninit = lsm6ds3_acc_local_remove,
};


