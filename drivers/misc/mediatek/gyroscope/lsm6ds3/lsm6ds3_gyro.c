
#include "lsm6ds3.h"


/*----------------------------------------------------------------------------*/


static struct data_resolution lsm6ds3_gyro_data_resolution[] = {
     /* combination by {FULL_RES,RANGE}*/
    {{ 0, 0}, 114},    //245dps	
	{{ 0, 0}, 57},    //500dps
    {{ 0, 0}, 29},     //1000dps
    {{ 0, 0}, 14},     //2000dps
};
#if 0
/*----------------------------------------------------------------------------*/
static struct data_resolution lsm6ds3_offset_resolution = {{15, 6}, 64};
static struct GSENSOR_VECTOR3D gsensor_gain, gsensor_offset;
#endif
struct gyro_hw lsm6ds3_gyro_cust_hw;
int lsm6ds3_gyro_init_flag =-1; // 0<==>OK -1 <==> fail
/*----------------------------------------------------------------------------*/


/*For driver get cust info*/
struct gyro_hw *lsm6ds3_get_cust_gyro_hw(void)
{
    return &lsm6ds3_gyro_cust_hw;
}

/*----------------------------------------------------------------------------*/

#if 1
static int lsm6ds3_gyro_set_resolution(struct lsm6ds3_gyro *gyro_obj)
{
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;
    
    int err;
    u8  dat, reso;

    err = lsm6ds3_i2c_read_block(client, LSM6DS3_REG_CTRL2_G, &dat, 0x01);

    if(err < 0)
    {
        ST_ERR("write data format fail!!\n");
        return err;
    }

    /*the data_reso is combined by 3 bits: {FULL_RES, DATA_RANGE}*/
    reso = (dat & LSM6DS3_REG_CTRL2_G_MASK_FS_G)>>2;
    if(reso >= 0x3)
        reso = 0x3;
    

    if(reso < sizeof(lsm6ds3_gyro_data_resolution)/sizeof(lsm6ds3_gyro_data_resolution[0]))
    {        
        gyro_obj->reso = &lsm6ds3_gyro_data_resolution[reso];
        return 0;
    }
    else
    {
        return -EINVAL;
    }
}
#endif
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_read_rawdata(struct lsm6ds3_gyro *gyro_obj, s16 data[LSM6DS3_AXES_NUM])
{
    
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;
    
    u8 buf[LSM6DS3_DATA_LEN] = {0};
    int err = 0;

    if(NULL == client)
    {
        err = -EINVAL;
    }
    
    else
    {
        if((lsm6ds3_i2c_read_block(client, LSM6DS3_REG_OUTX_L_G, buf, 0x06))<0)
        {
           ST_ERR("read  G sensor data register err!\n");
             return -1;
        }
		
		//data[LSM6DS3_AXIS_X] = ( (s16) ( ( (buf[1] << 8) | buf[0] ) ) ) >> 4;
        //data[LSM6DS3_AXIS_Y] = ( (s16) ( ( (buf[3] << 8) | buf[2] ) ) ) >> 4;
        //data[LSM6DS3_AXIS_Z] = ( (s16) ( ( (buf[5] << 8) | buf[4] ) ) ) >> 4;
		
		data[LSM6DS3_AXIS_X] = (s16)((buf[LSM6DS3_AXIS_X*2+1] << 8) | (buf[LSM6DS3_AXIS_X*2]));
		data[LSM6DS3_AXIS_Y] = (s16)((buf[LSM6DS3_AXIS_Y*2+1] << 8) | (buf[LSM6DS3_AXIS_Y*2]));
		data[LSM6DS3_AXIS_Z] = (s16)((buf[LSM6DS3_AXIS_Z*2+1] << 8) | (buf[LSM6DS3_AXIS_Z*2]));	

        if(atomic_read(&gyro_obj->trace) & ADX_TRC_RAWDATA)
        {
            ST_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z],
                                       data[LSM6DS3_AXIS_X], data[LSM6DS3_AXIS_Y], data[LSM6DS3_AXIS_Z]);
        }
        
#ifdef CONFIG_LSM6DS3_LOWPASS
        if(atomic_read(&gyro_obj->filter))
        {
            if(atomic_read(&gyro_obj->fir_en) && !atomic_read(&gyro_obj->suspend))
            {
                int idx, firlen = atomic_read(&gyro_obj->firlen);   
                if(gyro_obj->fir.num < firlen)
                {                
                    gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DS3_AXIS_X] = data[LSM6DS3_AXIS_X];
                    gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DS3_AXIS_Y] = data[LSM6DS3_AXIS_Y];
                    gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DS3_AXIS_Z] = data[LSM6DS3_AXIS_Z];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_X] += data[LSM6DS3_AXIS_X];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_Y] += data[LSM6DS3_AXIS_Y];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_Z] += data[LSM6DS3_AXIS_Z];
                    if(atomic_read(&gyro_obj->trace) & ADX_TRC_FILTER)
                    {
                        ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", gyro_obj->fir.num,
                            gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DS3_AXIS_X], gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DS3_AXIS_Y], gyro_obj->fir.raw[gyro_obj->fir.num][LSM6DS3_AXIS_Z],
                            gyro_obj->fir.sum[LSM6DS3_AXIS_X], gyro_obj->fir.sum[LSM6DS3_AXIS_Y], gyro_obj->fir.sum[LSM6DS3_AXIS_Z]);
                    }
                    gyro_obj->fir.num++;
                    gyro_obj->fir.idx++;
                }
                else
                {
                    idx = gyro_obj->fir.idx % firlen;
                    gyro_obj->fir.sum[LSM6DS3_AXIS_X] -= gyro_obj->fir.raw[idx][LSM6DS3_AXIS_X];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_Y] -= gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Y];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_Z] -= gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Z];
                    gyro_obj->fir.raw[idx][LSM6DS3_AXIS_X] = data[LSM6DS3_AXIS_X];
                    gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Y] = data[LSM6DS3_AXIS_Y];
                    gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Z] = data[LSM6DS3_AXIS_Z];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_X] += data[LSM6DS3_AXIS_X];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_Y] += data[LSM6DS3_AXIS_Y];
                    gyro_obj->fir.sum[LSM6DS3_AXIS_Z] += data[LSM6DS3_AXIS_Z];
                    gyro_obj->fir.idx++;
                    data[LSM6DS3_AXIS_X] = gyro_obj->fir.sum[LSM6DS3_AXIS_X]/firlen;
                    data[LSM6DS3_AXIS_Y] = gyro_obj->fir.sum[LSM6DS3_AXIS_Y]/firlen;
                    data[LSM6DS3_AXIS_Z] = gyro_obj->fir.sum[LSM6DS3_AXIS_Z]/firlen;
                    if(atomic_read(&gyro_obj->trace) & ADX_TRC_FILTER)
                    {
                        ST_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
                        gyro_obj->fir.raw[idx][LSM6DS3_AXIS_X], gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Y], gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Z],
                        gyro_obj->fir.sum[LSM6DS3_AXIS_X], gyro_obj->fir.sum[LSM6DS3_AXIS_Y], gyro_obj->fir.sum[LSM6DS3_AXIS_Z],
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
static int lsm6ds3_gyro_reset_calibration(struct lsm6ds3_gyro *gyro_obj)
{
    memset(gyro_obj->cali_sw, 0x00, sizeof(gyro_obj->cali_sw));
    return 0;     
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_read_calibration(struct lsm6ds3_gyro *gyro_obj, int dat[LSM6DS3_AXES_NUM])
{
    dat[gyro_obj->cvt.map[LSM6DS3_AXIS_X]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_X]*gyro_obj->cali_sw[LSM6DS3_AXIS_X];
    dat[gyro_obj->cvt.map[LSM6DS3_AXIS_Y]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_Y]*gyro_obj->cali_sw[LSM6DS3_AXIS_Y];
    dat[gyro_obj->cvt.map[LSM6DS3_AXIS_Z]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_Z]*gyro_obj->cali_sw[LSM6DS3_AXIS_Z];            
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_write_calibration(struct lsm6ds3_gyro *gyro_obj, int dat[LSM6DS3_AXES_NUM])
{
    int res = 0;

    ST_FUN();
    if(!gyro_obj || !dat)
    {
        ST_ERR("null ptr!!\n");
        return -EINVAL;
    }
    else
    {        
        s16 cali[LSM6DS3_AXES_NUM];
        cali[gyro_obj->cvt.map[LSM6DS3_AXIS_X]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_X]*gyro_obj->cali_sw[LSM6DS3_AXIS_X];
        cali[gyro_obj->cvt.map[LSM6DS3_AXIS_Y]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_Y]*gyro_obj->cali_sw[LSM6DS3_AXIS_Y];
        cali[gyro_obj->cvt.map[LSM6DS3_AXIS_Z]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_Z]*gyro_obj->cali_sw[LSM6DS3_AXIS_Z]; 
		
        cali[LSM6DS3_AXIS_X] += dat[LSM6DS3_AXIS_X];
        cali[LSM6DS3_AXIS_Y] += dat[LSM6DS3_AXIS_Y];
        cali[LSM6DS3_AXIS_Z] += dat[LSM6DS3_AXIS_Z];

        gyro_obj->cali_sw[LSM6DS3_AXIS_X] += gyro_obj->cvt.sign[LSM6DS3_AXIS_X]*dat[gyro_obj->cvt.map[LSM6DS3_AXIS_X]];
        gyro_obj->cali_sw[LSM6DS3_AXIS_Y] += gyro_obj->cvt.sign[LSM6DS3_AXIS_Y]*dat[gyro_obj->cvt.map[LSM6DS3_AXIS_Y]];
        gyro_obj->cali_sw[LSM6DS3_AXIS_Z] += gyro_obj->cvt.sign[LSM6DS3_AXIS_Z]*dat[gyro_obj->cvt.map[LSM6DS3_AXIS_Z]];
    } 

    return res;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_set_full_scale(struct lsm6ds3_gyro *gyro_obj, u8 dataformat)
{
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;
    u8 buf[10];
    u8 reg = LSM6DS3_REG_CTRL2_G;
    int res = 0;

    memset(buf, 0, sizeof(u8)*10);

    if((lsm6ds3_i2c_read_block(client, reg, buf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    buf[0] &= ~LSM6DS3_REG_CTRL2_G_MASK_FS_G;
    buf[0] |= dataformat;

    res = lsm6ds3_i2c_write_block(client, reg, buf, 0x1);

    if(res < 0)
    {
        return LSM6DS3_ERR_I2C;
    }
	return lsm6ds3_gyro_set_resolution(gyro_obj);
	//return LSM6DS3_SUCCESS;
    
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_set_odr(struct lsm6ds3_gyro *gyro_obj, u8 odr)
{
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;

    u8 buf[10];
    u8 reg = LSM6DS3_REG_CTRL2_G;
    int res = 0;

    memset(buf, 0, sizeof(u8)*10);
    
    if((lsm6ds3_i2c_read_block(client, reg, buf, 0x01))<0)
    {
        ST_ERR("read reg_ctl_reg1 register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    buf[0] &= ~LSM6DS3_REG_CTRL2_G_MASK_ODR_G;
    buf[0] |= odr;

    res = lsm6ds3_i2c_write_block(client, reg, buf, 0x1);

    if(res < 0)
    {
        return LSM6DS3_ERR_I2C;
    }
    		
    return LSM6DS3_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
int lsm6ds3_gyro_set_power_mode(struct lsm6ds3_gyro *gyro_obj, bool state)
{ 
    int res = 0;

    if(state == gyro_obj->lsm6ds3_gyro_power)
    {
        ST_LOG("Sensor power status is newest!\n");
        return LSM6DS3_SUCCESS;
    }

    if(state == true)
    {
		if(gyro_obj->odr == 0)
		{
			gyro_obj->odr = LSM6DS3_REG_CTRL2_G_ODR_104HZ;
		}
		res = lsm6ds3_gyro_set_odr(gyro_obj, gyro_obj->odr);
    }
    else if(state == false)
    {
		res = lsm6ds3_gyro_set_odr(gyro_obj, LSM6DS3_REG_CTRL2_G_ODR_0HZ);
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
    else if(atomic_read(&gyro_obj->trace) & ADX_TRC_INFO)
    {
        ST_LOG("set power mode ok %d!\n", state);
    }
	
	gyro_obj->lsm6ds3_gyro_power = state;
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
int lsm6ds3_gyro_init(struct lsm6ds3_gyro *gyro_obj, int reset_cali)
{
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;

    int res = 0;
    u8 buf[2] = {0, 0};
    ST_FUN();
    // first clear reg1
    buf[0] = 0x00;
    res = lsm6ds3_i2c_write_block(client, LSM6DS3_REG_CTRL2_G, buf, 0x01);
    if(res < 0)
    {
        ST_ERR("lsm6ds3_gyro_init step 1!\n");
        return res;
    }
    
    gyro_obj->odr = 0;
    res = lsm6ds3_gyro_set_odr(gyro_obj, LSM6DS3_REG_CTRL2_G_ODR_0HZ);//power down
    if(res < 0)
    {
        ST_ERR("lsm6ds3_gyro_init step 2!\n");
        return res;
    }

    res = lsm6ds3_gyro_set_full_scale(gyro_obj, LSM6DS3_REG_CTRL2_G_FS_2000DPS);//8g or 2G no oher choise
    if(res < 0) 
    {
        ST_ERR("lsm6ds3_gyro_init step 3!\n");
        return res;
    }

    if(0 != reset_cali)
    { 
        //reset calibration only in power on
        res = lsm6ds3_gyro_reset_calibration(gyro_obj);
        if(res < 0)
        {
            return res;
        }
    }

#ifdef CONFIG_LSM6DS3_LOWPASS
    memset(&gyro_obj->fir, 0x00, sizeof(gyro_obj->fir));
#endif
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_read_chip_name(struct lsm6ds3_gyro *gyro_obj, u8 *buf, int bufsize)
{
    sprintf(buf, "%s", gyro_obj->name);
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_read_data(struct lsm6ds3_gyro *gyro_obj, u8 *data, int bufsize)
{
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;

    //u8 databuf[20];
    int gyro[LSM6DS3_AXES_NUM];
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

    if(atomic_read(&gyro_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }
	
    if((res = lsm6ds3_gyro_read_rawdata(gyro_obj, gyro_obj->data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return res;
    }
    else
    {
        gyro_obj->data[LSM6DS3_AXIS_X] += gyro_obj->cali_sw[LSM6DS3_AXIS_X];
        gyro_obj->data[LSM6DS3_AXIS_Y] += gyro_obj->cali_sw[LSM6DS3_AXIS_Y];
        gyro_obj->data[LSM6DS3_AXIS_Z] += gyro_obj->cali_sw[LSM6DS3_AXIS_Z];
        
        /*remap coordinate*/
        gyro[gyro_obj->cvt.map[LSM6DS3_AXIS_X]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_X]*gyro_obj->data[LSM6DS3_AXIS_X];
        gyro[gyro_obj->cvt.map[LSM6DS3_AXIS_Y]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_Y]*gyro_obj->data[LSM6DS3_AXIS_Y];
        gyro[gyro_obj->cvt.map[LSM6DS3_AXIS_Z]] = gyro_obj->cvt.sign[LSM6DS3_AXIS_Z]*gyro_obj->data[LSM6DS3_AXIS_Z];

        //ST_LOG("Mapped gsensor data: %d, %d, %d!\n", gyro[LSM6DS3_AXIS_X], gyro[LSM6DS3_AXIS_Y], gyro[LSM6DS3_AXIS_Z]);

        //Out put the degree/sencond
        gyro[LSM6DS3_AXIS_X] = gyro[LSM6DS3_AXIS_X]*131/gyro_obj->reso->sensitivity;
        gyro[LSM6DS3_AXIS_Y] = gyro[LSM6DS3_AXIS_Y]*131/gyro_obj->reso->sensitivity;
        gyro[LSM6DS3_AXIS_Z] = gyro[LSM6DS3_AXIS_Z]*131/gyro_obj->reso->sensitivity;
        

        sprintf(data, "%04x %04x %04x", gyro[LSM6DS3_AXIS_X], gyro[LSM6DS3_AXIS_Y], gyro[LSM6DS3_AXIS_Z]);
        if(atomic_read(&gyro_obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
        {
            ST_LOG("gyroscope data: %s!\n", data);
            dumpReg(obj);
        }
    }
    
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_read_rawdata_string(struct lsm6ds3_gyro *gyro_obj, u8 *buf)
{
    struct lsm6ds3_data *obj = container_of(gyro_obj, struct lsm6ds3_data, lsm6ds3_gyro_data);
	struct i2c_client *client = obj->client;
    int res = 0;

    if (!buf || !client)
    {
        return LSM6DS3_ERR_SETUP_FAILURE;
    }
    
    if((res = lsm6ds3_gyro_read_rawdata(gyro_obj, gyro_obj->data)))
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return res;
    }
    else
    {
        sprintf(buf, "%04x %04x %04x", gyro_obj->data[LSM6DS3_AXIS_X], 
        gyro_obj->data[LSM6DS3_AXIS_Y], gyro_obj->data[LSM6DS3_AXIS_Z]);
    }
    
    return LSM6DS3_SUCCESS;
}


/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    //struct i2c_client *client = lsm6ds3_i2c_client;
    struct lsm6ds3_data *obj = obj_i2c_data;
	struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
    u8 strbuf[LSM6DS3_BUFSIZE];
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    
    lsm6ds3_gyro_read_chip_name(gyro_obj, strbuf, LSM6DS3_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_chipid_value(struct device_driver *ddri, char *buf)
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
static ssize_t lsm6ds3_attr_gyro_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
    u8 databuf[LSM6DS3_BUFSIZE];
    
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    lsm6ds3_gyro_read_data(gyro_obj, databuf, LSM6DS3_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_rawdata_value(struct device_driver *ddri, char *buf)
{   
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
    u8 databuf[LSM6DS3_BUFSIZE];
    
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    lsm6ds3_gyro_read_rawdata_string(gyro_obj, databuf);
    return snprintf(buf, PAGE_SIZE, "%s\n", databuf);
}
#if 0
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_cali_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 
    int err, len, mul;
    int tmp[LSM6DS3_AXES_NUM];    
    len = 0;

    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    if((err = lsm6ds3_gyro_read_calibration(gyro_obj, tmp)))
    {
        return -EINVAL;
    }
    else
    {    
        mul = gyro_obj->reso->sensitivity/lsm6ds3_offset_resolution.sensitivity;
        len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
            gyro_obj->offset[LSM6DS3_AXIS_X], gyro_obj->offset[LSM6DS3_AXIS_Y], gyro_obj->offset[LSM6DS3_AXIS_Z],
            gyro_obj->offset[LSM6DS3_AXIS_X], gyro_obj->offset[LSM6DS3_AXIS_Y], gyro_obj->offset[LSM6DS3_AXIS_Z]);
        len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
            gyro_obj->cali_sw[LSM6DS3_AXIS_X], gyro_obj->cali_sw[LSM6DS3_AXIS_Y], gyro_obj->cali_sw[LSM6DS3_AXIS_Z]);

        len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
            gyro_obj->offset[LSM6DS3_AXIS_X]*mul + gyro_obj->cali_sw[LSM6DS3_AXIS_X],
            gyro_obj->offset[LSM6DS3_AXIS_Y]*mul + gyro_obj->cali_sw[LSM6DS3_AXIS_Y],
            gyro_obj->offset[LSM6DS3_AXIS_Z]*mul + gyro_obj->cali_sw[LSM6DS3_AXIS_Z],
            tmp[LSM6DS3_AXIS_X], tmp[LSM6DS3_AXIS_Y], tmp[LSM6DS3_AXIS_Z]);
        
        return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{ 
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 
    int err, x, y, z;
    int dat[LSM6DS3_AXES_NUM];

    if(!strncmp(buf, "rst", 3))
    {
        if((err = lsm6ds3_gyro_reset_calibration(gyro_obj)))
        {
            ST_ERR("reset offset err = %d\n", err);
        }    
    }
    else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
    {
        dat[LSM6DS3_AXIS_X] = x;
        dat[LSM6DS3_AXIS_Y] = y;
        dat[LSM6DS3_AXIS_Z] = z;
        if((err = lsm6ds3_gyro_write_calibration(gyro_obj, dat)))
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
#endif
static ssize_t lsm6ds3_attr_gyro_show_power_status(struct device_driver *ddri, char *buf)
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
static ssize_t lsm6ds3_attr_gyro_show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_LSM6DS3_LOWPASS
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 
    if(atomic_read(&gyro_obj->firlen))
    {
        int idx, len = atomic_read(&gyro_obj->firlen);
        ST_LOG("len = %2d, idx = %2d\n", gyro_obj->fir.num, gyro_obj->fir.idx);

        for(idx = 0; idx < len; idx++)
        {
            ST_LOG("[%5d %5d %5d]\n", gyro_obj->fir.raw[idx][LSM6DS3_AXIS_X], gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Y], gyro_obj->fir.raw[idx][LSM6DS3_AXIS_Z]);
        }
        
        ST_LOG("sum = [%5d %5d %5d]\n", gyro_obj->fir.sum[LSM6DS3_AXIS_X], gyro_obj->fir.sum[LSM6DS3_AXIS_Y], gyro_obj->fir.sum[LSM6DS3_AXIS_Z]);
        ST_LOG("avg = [%5d %5d %5d]\n", gyro_obj->fir.sum[LSM6DS3_AXIS_X]/len, gyro_obj->fir.sum[LSM6DS3_AXIS_Y]/len, gyro_obj->fir.sum[LSM6DS3_AXIS_Z]/len);
    }
    return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&gyro_obj->firlen));
#else
    return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_LSM6DS3_LOWPASS
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 
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
        atomic_set(&gyro_obj->firlen, firlen);
        if(0 == firlen)
        {
            atomic_set(&gyro_obj->fir_en, 0);
        }
        else
        {
            memset(&gyro_obj->fir, 0x00, sizeof(gyro_obj->fir));
            atomic_set(&gyro_obj->fir_en, 1);
        }
    }
#endif    
    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&gyro_obj->trace));     
    return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 
    int trace;
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&gyro_obj->trace, trace);
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }
    
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_status_value(struct device_driver *ddri, char *buf)
{    
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data; 
    ssize_t len = 0;
    
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(gyro_obj->lsm6ds3_gyro_hw)
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: i2c_num=%d, direction=%d, sensitivity = %d,(power_id=%d, power_vol=%d)\n", 
                gyro_obj->lsm6ds3_gyro_hw->i2c_num, gyro_obj->lsm6ds3_gyro_hw->direction, gyro_obj->reso->sensitivity, gyro_obj->lsm6ds3_gyro_hw->power_id, gyro_obj->lsm6ds3_gyro_hw->power_vol);   
        dumpReg(obj);
    }
    else
    {
        len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
    }
    return len;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&gyro_obj->trace)); 
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lsm6ds3_gyro_init(gyro_obj, 0);
    dumpReg(obj);

    return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_show_layout_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;

    if(NULL == obj)
    {
        ST_LOG("lsm6ds3_gyro is null!!\n");
        return -1;
    }

    return sprintf(buf, "(%d)\n[%+2d %+2d %+2d]\n[%+2d %+2d %+2d]\n",
        gyro_obj->lsm6ds3_gyro_hw->direction, gyro_obj->cvt.sign[0], gyro_obj->cvt.sign[1],
        gyro_obj->cvt.sign[2], gyro_obj->cvt.map[0], gyro_obj->cvt.map[1], gyro_obj->cvt.map[2]);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_gyro_store_layout_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;

    int layout = 0;

    if(NULL == obj)
    {
        ST_ERR("lsm6ds3_gyro is null!!\n");
        return count;
    }

    if(1 == sscanf(buf, "%d", &layout))
    {
        if(!hwmsen_get_convert(layout, &gyro_obj->cvt))
        {
            ST_ERR("HWMSEN_GET_CONVERT function error!\r\n");
        }
        else if(!hwmsen_get_convert(gyro_obj->lsm6ds3_gyro_hw->direction, &gyro_obj->cvt))
        {
            ST_LOG("invalid layout: %d, restore to %d\n", layout, gyro_obj->lsm6ds3_gyro_hw->direction);
        }
        else
        {
            ST_ERR("invalid layout: (%d, %d)\n", layout, gyro_obj->lsm6ds3_gyro_hw->direction);
            hwmsen_get_convert(0, &gyro_obj->cvt);
        }
    }
    else
    {
        ST_LOG("invalid format = '%s'\n", buf);
    }

    return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, lsm6ds3_attr_gyro_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lsm6ds3_attr_gyro_show_chipid_value,        NULL);
static DRIVER_ATTR(rawdata,              S_IRUGO, lsm6ds3_attr_gyro_show_rawdata_value,       NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lsm6ds3_attr_gyro_show_sensordata_value,    NULL);
//static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, lsm6ds3_attr_gyro_show_cali_value,          lsm6ds3_attr_gyro_store_cali_value);
static DRIVER_ATTR(power,                S_IRUGO, lsm6ds3_attr_gyro_show_power_status,        NULL);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, lsm6ds3_attr_gyro_show_firlen_value,        lsm6ds3_attr_gyro_store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lsm6ds3_attr_gyro_show_trace_value,         lsm6ds3_attr_gyro_store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, lsm6ds3_attr_gyro_show_status_value,        NULL);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lsm6ds3_attr_gyro_show_chipinit_value,      lsm6ds3_attr_gyro_store_chipinit_value);
static DRIVER_ATTR(layout,     S_IRUGO | S_IWUSR, lsm6ds3_attr_gyro_show_layout_value,        lsm6ds3_attr_gyro_store_layout_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *lsm6ds3_attr_gyro_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_rawdata,      /*dump sensor raw data*/
    //&driver_attr_cali,         /*show calibration data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_status,
    &driver_attr_chipinit,
    &driver_attr_layout,
};
/*----------------------------------------------------------------------------*/
int lsm6ds3_gyro_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_gyro_list)/sizeof(lsm6ds3_attr_gyro_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, lsm6ds3_attr_gyro_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6ds3_attr_gyro_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
int lsm6ds3_gyro_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_gyro_list)/sizeof(lsm6ds3_attr_gyro_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }
    

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lsm6ds3_attr_gyro_list[idx]);
    }
    

    return err;
}

/*----------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int lsm6ds3_gyro_open_report_data_intf(int open)
{
    //should queuq work to report event if  is_report_input_direct=true
    return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int lsm6ds3_gyro_enable_nodata_intf(int en)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
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
	gyro_obj->enabled = en;
    res = lsm6ds3_gyro_set_power_mode(gyro_obj, power);
    if(res != LSM6DS3_SUCCESS)
    {
        ST_ERR("lsm6ds3_gyro_set_power_mode fail!\n");
        return res;
    }
    ST_LOG("lsm6ds3_gyro_enable_nodata_intf OK!\n");
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_gyro_set_delay_intf(u64 ns)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;

    int value =0;
    int sample_delay=0;
    int err;
	
    value = (int)ns/1000/1000;
    if(value <= 5)
    {
        sample_delay = LSM6DS3_REG_CTRL2_G_ODR_208HZ;
    }
    else if(value <= 10)
    {
        sample_delay = LSM6DS3_REG_CTRL2_G_ODR_104HZ;
    }
    else
    {
        sample_delay = LSM6DS3_REG_CTRL2_G_ODR_52HZ;
    }

	gyro_obj->odr = sample_delay;
	err = lsm6ds3_gyro_set_odr(gyro_obj, gyro_obj->odr);
    if(err != LSM6DS3_SUCCESS ) //0x2C->BW=100Hz
    {
        ST_ERR("Set delay parameter error!\n");
    }

    if(value >= 50)
    {
        atomic_set(&gyro_obj->filter, 0);
    }
    else
    {                    
        gyro_obj->fir.num = 0;
        gyro_obj->fir.idx = 0;
        gyro_obj->fir.sum[LSM6DS3_AXIS_X] = 0;
        gyro_obj->fir.sum[LSM6DS3_AXIS_Y] = 0;
        gyro_obj->fir.sum[LSM6DS3_AXIS_Z] = 0;
        atomic_set(&gyro_obj->filter, 1);
    }
    
    ST_LOG("lsm6ds3_gyro_set_delay_intf (%d)\n",value);
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_gyro_get_data_intf(int* x ,int* y,int* z, int* status)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;

    u8 buff[LSM6DS3_BUFSIZE];
    lsm6ds3_gyro_read_data(gyro_obj, buff, LSM6DS3_BUFSIZE);
    
    sscanf(buff, "%x %x %x", x, y, z);        
    *status = SENSOR_STATUS_ACCURACY_MEDIUM;

    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_gyro_open(struct inode *inode, struct file *file)
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
static int lsm6ds3_gyro_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}
/*----------------------------------------------------------------------------*/
static long lsm6ds3_gyro_unlocked_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)

{
    struct lsm6ds3_data *obj = (struct lsm6ds3_data*)file->private_data;
	struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
	char strbuf[LSM6DS3_BUFSIZE] = {0};
	void __user *data;
	long err = 0;
	int copy_cnt = 0;
	struct SENSOR_DATA sensor_data;
	int cali[3] = {0};
	int smtRes=0;
	//GYRO_FUN();

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
		case GYROSCOPE_IOCTL_INIT:
			lsm6ds3_gyro_init(gyro_obj, 1);
			break;

		case GYROSCOPE_IOCTL_SMT_DATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}

			ST_LOG("IOCTL smtRes: %d!\n", smtRes);
			copy_cnt = copy_to_user(data, &smtRes,  sizeof(smtRes));

			if(copy_cnt)
			{
				err = -EFAULT;
				ST_ERR("copy gyro data to user failed!\n");
			}
			ST_LOG("copy gyro data to user OK: %d!\n", copy_cnt);
			break;


		case GYROSCOPE_IOCTL_READ_SENSORDATA:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}

			lsm6ds3_gyro_read_data(gyro_obj, strbuf, LSM6DS3_BUFSIZE);
			if(copy_to_user(data, strbuf, sizeof(strbuf)))
			{
				err = -EFAULT;
				break;
			}
			break;

		case GYROSCOPE_IOCTL_SET_CALI:
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

			else
			{
				cali[LSM6DS3_AXIS_X] = (s64)(sensor_data.x);// * 180*1000*1000/(LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142);
				cali[LSM6DS3_AXIS_Y] = (s64)(sensor_data.y);// * 180*1000*1000/(LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142);
				cali[LSM6DS3_AXIS_Z] = (s64)(sensor_data.z);// * 180*1000*1000/(LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142);
				err = lsm6ds3_gyro_write_calibration(gyro_obj, cali);
			}
			break;

		case GYROSCOPE_IOCTL_CLR_CALI:
			err = lsm6ds3_gyro_reset_calibration(gyro_obj);
			break;

		case GYROSCOPE_IOCTL_GET_CALI:
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;
			}
			err = lsm6ds3_gyro_read_calibration(gyro_obj, cali);
			if(err)
			{
				break;
			}

			sensor_data.x = (s64)(cali[LSM6DS3_AXIS_X]);// * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);
			sensor_data.y = (s64)(cali[LSM6DS3_AXIS_Y]);// * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);
			sensor_data.z = (s64)(cali[LSM6DS3_AXIS_Z]);// * LSM6DS3_GYRO_SENSITIVITY_2000DPS*3142/(180*1000*1000);

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
static long lsm6ds3_gyro_compat_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    long res = 0;
    void __user *arg32 = compat_ptr(arg);

    if (!file->f_op || !file->f_op->unlocked_ioctl)
        return -ENOTTY;

    switch (cmd) {
        case COMPAT_GYROSCOPE_IOCTL_INIT:
			 if(arg32 == NULL)
			 {
				 ST_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 res = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_INIT,
							(unsigned long)arg32);
			 if (res){
			 	ST_ERR("GYROSCOPE_IOCTL_INIT unlocked_ioctl failed.");
				return res;
			 }

			 break;

		 case COMPAT_GYROSCOPE_IOCTL_SET_CALI:
			 if(arg32 == NULL)
			 {
				 ST_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 res = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_SET_CALI,
							(unsigned long)arg32);
			 if (res){
			 	ST_ERR("GYROSCOPE_IOCTL_SET_CALI unlocked_ioctl failed.");
				return res;
			 }

			 break;

		 case COMPAT_GYROSCOPE_IOCTL_CLR_CALI:
			 if(arg32 == NULL)
			 {
				 ST_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 res = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_CLR_CALI,
							(unsigned long)arg32);
			 if (res){
			 	ST_ERR("GYROSCOPE_IOCTL_CLR_CALI unlocked_ioctl failed.");
				return res;
			 }

			 break;

		 case COMPAT_GYROSCOPE_IOCTL_GET_CALI:
			 if(arg32 == NULL)
			 {
				 ST_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 res = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_GET_CALI,
							(unsigned long)arg32);
			 if (res){
			 	ST_ERR("GYROSCOPE_IOCTL_GET_CALI unlocked_ioctl failed.");
				return res;
			 }

			 break;

		 case COMPAT_GYROSCOPE_IOCTL_READ_SENSORDATA:
			 if(arg32 == NULL)
			 {
				 ST_ERR("invalid argument.");
				 return -EINVAL;
			 }

			 res = file->f_op->unlocked_ioctl(file, GYROSCOPE_IOCTL_READ_SENSORDATA,
							(unsigned long)arg32);
			 if (res){
			 	ST_ERR("GYROSCOPE_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
				return res;
			 }

			 break;

		 default:
			 printk(KERN_ERR "%s not supported = 0x%04x", __FUNCTION__, cmd);
			 return -ENOIOCTLCMD;
			 break;
    }

    return res;
}
#endif


static struct file_operations lsm6ds3_gyro_fops = {
    .owner = THIS_MODULE,
    .open = lsm6ds3_gyro_open,
    .release = lsm6ds3_gyro_release,
    .unlocked_ioctl = lsm6ds3_gyro_unlocked_ioctl,
#ifdef CONFIG_COMPAT
    .compat_ioctl = lsm6ds3_gyro_compat_ioctl,
#endif
};

static struct miscdevice lsm6ds3_gyro_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "gyroscope",
    .fops = &lsm6ds3_gyro_fops,
};


/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_local_init(struct platform_device *pdev)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
    int err = 0;
    int retry = 0;
    struct gyro_control_path ctl={0};
    struct gyro_data_path data={0};    
    const u8 *name = "mediatek,lsm6ds3_gyro";
    ST_FUN();

    gyro_obj->lsm6ds3_gyro_hw = get_gyro_dts_func(name, &lsm6ds3_gyro_cust_hw);

    if (!gyro_obj->lsm6ds3_gyro_hw) {
        ST_ERR("get lsm6ds3 dts info failed\n");
    }

    if((err = hwmsen_get_convert(gyro_obj->lsm6ds3_gyro_hw->direction, &gyro_obj->cvt)))
    {
        ST_ERR("invalid direction: %d\n", gyro_obj->lsm6ds3_gyro_hw->direction);
        goto exit;
    }

    atomic_set(&gyro_obj->trace, 0);
    atomic_set(&gyro_obj->suspend, 0);
    
#ifdef CONFIG_LSM6DS3_LOWPASS
    if(gyro_obj->lsm6ds3_gyro_hw->firlen > C_MAX_FIR_LENGTH)
    {
        atomic_set(&gyro_obj->firlen, C_MAX_FIR_LENGTH);
    }    
    else
    {
        atomic_set(&gyro_obj->firlen, gyro_obj->lsm6ds3_gyro_hw->firlen);
    }
    
    if(atomic_read(&gyro_obj->firlen) > 0)
    {
        atomic_set(&gyro_obj->fir_en, 1);
    }
#endif

    for(retry = 0; retry < 3; retry++){
        if((err = lsm6ds3_gyro_init(gyro_obj, 1)))
        {
            ST_ERR("lsm6ds3_gyro_device init cilent fail time: %d\n", retry);
            continue;
        }
    }
    if(err != 0)
        goto exit_init_failed;

	sprintf(gyro_obj->name, "%s_GYRO", obj->name);
	
    if((err = misc_register(&lsm6ds3_gyro_device)))
    {
        ST_ERR("lsm6ds3_gyro_device register failed\n");
        goto exit_misc_device_register_failed;
    }

    if((err = lsm6ds3_gyro_create_attr(&(lsm6ds3_gyro_init_info.platform_diver_addr->driver))))
    {
        ST_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }
    ctl.is_use_common_factory = false;
    ctl.open_report_data= lsm6ds3_gyro_open_report_data_intf;
    ctl.enable_nodata = lsm6ds3_gyro_enable_nodata_intf;
    ctl.set_delay  = lsm6ds3_gyro_set_delay_intf;
    ctl.is_report_input_direct = false;
    
    err = gyro_register_control_path(&ctl);
    if(err)
    {
        ST_ERR("register acc control path err\n");
        goto exit_kfree;
    }

    data.get_data = lsm6ds3_gyro_get_data_intf;
    //data.vender_div = 1000*1000;
    data.vender_div = DEGREE_TO_RAD;
    err = gyro_register_data_path(&data);
    if(err) {
        ST_ERR("register acc data path err\n");
        goto exit_kfree;
    }

    ST_LOG("%s: OK\n", __func__);
    lsm6ds3_gyro_init_flag = 0;    
    return 0;

exit_create_attr_failed:
    misc_deregister(&lsm6ds3_gyro_device);
exit_misc_device_register_failed:
exit_init_failed:
exit_kfree:
    kfree(obj);
exit:
    ST_ERR("%s: err = %d\n", __func__, err);
    lsm6ds3_gyro_init_flag = -1;        
    return lsm6ds3_gyro_init_flag;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_gyro_local_remove(void)
{
    ST_FUN(); 
    misc_deregister(&lsm6ds3_gyro_device);
    lsm6ds3_gyro_delete_attr(&(lsm6ds3_gyro_init_info.platform_diver_addr->driver));
    
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct gyro_init_info lsm6ds3_gyro_init_info = {
        .name = "lsm6ds3_gyro",
        .init = lsm6ds3_gyro_local_init,
        .uninit = lsm6ds3_gyro_local_remove,
};


