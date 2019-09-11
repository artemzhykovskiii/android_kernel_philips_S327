
#include "lsm6ds3.h"

extern int  step_notify(STEP_NOTIFY_TYPE type);
extern int step_c_register_control_path(struct step_c_control_path *ctl);
extern int step_c_register_data_path(struct step_c_data_path *data);

int lsm6ds3_pedo_init_flag =-1; // 0<==>OK -1 <==> fail

static int lsm6ds3_pedo_set_deb_step(struct lsm6ds3_pedo *pedo_obj, u8 deb_step)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;
    u8 databuf[10] = {0};
    u8 addr = LSM6DS3_REG_PEDO_DEB;
    int res = 0;

    res = lsm6ds3_open_ram_page(obj, LSM6DS3_REG_FUNC_CFG_ACCESS_ENABLE);
    if(res)
    {
        ST_ERR("lsm6ds3_open_ram_page error!\n");
        return LSM6DS3_ERR_I2C;
    }

    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_PEDO_DEB register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~LSM6DS3_REG_PEDO_DEB_MASK_DEB_STEP;
    databuf[0] |= deb_step;

    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
    if(res < 0)
    {
        ST_ERR("write LSM6DS3_REG_PEDO_DEB register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    res = lsm6ds3_open_ram_page(obj, LSM6DS3_REG_FUNC_CFG_ACCESS_DISABLE);
    if(res)
    {
        ST_ERR("lsm6ds3_open_ram_page error!\n");
        return LSM6DS3_ERR_I2C;
    }
    
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_pedo_set_deb_time(struct lsm6ds3_pedo *pedo_obj, u8 deb_time)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;
    u8 databuf[10] = {0};
    u8 addr = LSM6DS3_REG_PEDO_DEB;
    int res = 0;

    res = lsm6ds3_open_ram_page(obj, LSM6DS3_REG_FUNC_CFG_ACCESS_ENABLE);
    if(res)
    {
        ST_ERR("lsm6ds3_open_ram_page error!\n");
        return LSM6DS3_ERR_I2C;
    }

    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_PEDO_DEB register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~LSM6DS3_REG_PEDO_DEB_MASK_DEB_TIME;
    databuf[0] |= deb_time;

    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
    if(res < 0)
    {
        ST_ERR("write LSM6DS3_REG_PEDO_DEB register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    res = lsm6ds3_open_ram_page(obj, LSM6DS3_REG_FUNC_CFG_ACCESS_DISABLE);
    if(res)
    {
        ST_ERR("lsm6ds3_open_ram_page error!\n");
        return LSM6DS3_ERR_I2C;
    }
    
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_pedo_set_threshold(struct lsm6ds3_pedo *pedo_obj, u8 threshold)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;

    u8 databuf[10] = {0};
    u8 addr = LSM6DS3_REG_PEDO_THS;
    int res = 0;
    
    res = lsm6ds3_open_ram_page(obj, LSM6DS3_REG_FUNC_CFG_ACCESS_ENABLE);
    if(res)
    {
        ST_ERR("lsm6ds3_open_ram_page error!\n");
        return LSM6DS3_ERR_I2C;
    }

    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_PEDO_THS register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~LSM6DS3_REG_PEDO_THS_MASK_PEDO_THS_MIN;
    databuf[0] |= threshold;

    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
    if(res)
    {
        ST_ERR("write LSM6DS3_REG_PEDO_THS register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    res = lsm6ds3_open_ram_page(obj, LSM6DS3_REG_FUNC_CFG_ACCESS_DISABLE);
    if(res)
    {
        ST_ERR("lsm6ds3_open_ram_page error!\n");
        return LSM6DS3_ERR_I2C;
    }	
    return LSM6DS3_SUCCESS; 
}

static int lsm6ds3_pedo_reset_counter(struct lsm6ds3_pedo *pedo_obj)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;

    u8 databuf[10] = {0};
    u8 addr = LSM6DS3_REG_CTRL10_C;
    int res = 0;

    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_PEDO_THS register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~LSM6DS3_REG_CTRL10_C_MASK_PEDO_RST_STEP;
    databuf[0] |= LSM6DS3_REG_CTRL10_C_FUNC_PEDO_RST_STEP;
    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
    if(res)
    {
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~LSM6DS3_REG_CTRL10_C_MASK_PEDO_RST_STEP;
    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
    if(res)
    {
        return LSM6DS3_ERR_I2C;
    }

    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#if 0
static int lsm6ds3_pedo_set_power_mode(struct lsm6ds3_pedo *pedo_obj, bool state)
{
    u8 databuf[2] = {0};    
    int res = 0;

    if(state == pedo_obj->lsm6ds3_pedo_power)
    {
        ST_LOG("Sensor power status is newest!\n");
        return LSM6DS3_SUCCESS;
    }

    if(state == true)
    {
		//for some interrupt
        //
    }

    else if(state == false)
    {
		//for some interrupt
        //
    }
	
    if(res < 0)
    {
        ST_LOG("set power mode failed!\n");
        return LSM6DS3_ERR_I2C;
    }
    else if(atomic_read(&pedo_obj->trace) & ADX_TRC_INFO)
    {
        ST_LOG("set power mode ok %d!\n", databuf[1]);
    }
	
	pedo_obj->lsm6ds3_pedo_power = state;
    return LSM6DS3_SUCCESS;
}
#endif
/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_init(struct lsm6ds3_pedo *pedo_obj, int reset_cali)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	//struct i2c_client *client = obj->client;
    int res = 0; 
    pedo_obj->odr = 0;

    res = lsm6ds3_embed_func_enable(obj, LSM6DS3_REG_CTRL10_C_FUNC_ENABLE);
    if(res < 0)
    {
        ST_ERR("lsm6ds3_embed_func_enable error!\n");
        return res;
    }
    return LSM6DS3_SUCCESS;

    res = lsm6ds3_pedo_set_threshold(pedo_obj, LSM6DS3_REG_PEDO_THS_PEDO_4G_320MG);
    if(res < 0)
    {
        ST_ERR("lsm6ds3_pedo_set_threshold error!\n");
        return res;
    }

    res = lsm6ds3_pedo_set_deb_time(pedo_obj, LSM6DS3_REG_PEDO_DEB_TIME_960MS);
    if(res < 0) 
    {
        ST_ERR("lsm6ds3_pedo_set_deb_time error!\n");
        return res;
    }

    res = lsm6ds3_pedo_set_deb_step(pedo_obj, LSM6DS3_REG_PEDO_DEB_STEP_7STEP);
    if(res < 0) 
    {
        ST_ERR("lsm6ds3_pedo_set_deb_step error!\n");
        return res;
    }

    res = lsm6ds3_pedo_reset_counter(pedo_obj);
    if(res < 0) 
    {
        ST_ERR("lsm6ds3_pedo_reset_counter error!\n");
        return res;
    }

    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_enable(struct lsm6ds3_pedo *pedo_obj, u8 value)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;
    u8 addr = LSM6DS3_REG_TAP_CFG;
    int res = 0;
    u8 databuf[2] = {0, 0};
    u8 odr = 0;
 
    addr = LSM6DS3_REG_CTRL1_XL;
    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_CTRL1_XL_MASK_ODR_XL register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    odr = databuf[0]&LSM6DS3_REG_CTRL1_XL_MASK_ODR_XL;

    if ((value == LSM6DS3_REG_TAP_CFG_PEDO_ENABLE) && (odr == LSM6DS3_REG_CTRL1_XL_ODR_0HZ))
    {
        databuf[0] &= ~LSM6DS3_REG_CTRL1_XL_MASK_ODR_XL;
        databuf[0] |= LSM6DS3_REG_CTRL1_XL_ODR_26HZ;
        res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
        ST_LOG("enable pedo set acc odr 26HZ\n");
        if(res)
        {
            ST_ERR("write LSM6DS3_REG_CTRL1_XL failed");
            return LSM6DS3_ERR_I2C;
        }
    }
    
    if ((value == LSM6DS3_REG_TAP_CFG_PEDO_DISABLE) && (odr == LSM6DS3_REG_CTRL1_XL_ODR_26HZ))
    {
        databuf[0] &= ~LSM6DS3_REG_CTRL1_XL_MASK_ODR_XL;
        databuf[0] |= LSM6DS3_REG_CTRL1_XL_ODR_0HZ;
        res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
        ST_LOG("disable pedo set acc odr 0HZ\n");
        if(res)
        {
            ST_ERR("write LSM6DS3_REG_CTRL1_XL failed");
            return LSM6DS3_ERR_I2C;
        }
    }

    addr = LSM6DS3_REG_TAP_CFG;
    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_TAP_CFG register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~LSM6DS3_REG_TAP_CFG_MASK_PEDO_EN;
    databuf[0] |= value;

    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x1);
    if(res)
    {
        ST_ERR("write LSM6DS3_REG_TAP_CFG error");
        return LSM6DS3_ERR_I2C;
    }
    ST_LOG("enable pedo\n");
    return LSM6DS3_SUCCESS;
}

static int lsmd6s3_pedo_enable_pedo_interrupt(struct lsm6ds3_pedo *pedo_obj, u8 value)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;
    u8 addr = LSM6DS3_REG_INT1_CTRL;
    u8 databuf[2] = {0};
    
    int res = 0;
    ST_FUN();
    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_INT1_CTRL register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~(LSM6DS3_REG_INT1_CTRL_MASK_INT1_STEP_DETECTOR);
    databuf[0] |= value;

    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("write LSM6DS3_REG_INT1_CTRL register err!\n");
        return LSM6DS3_ERR_I2C;
    }
    return LSM6DS3_SUCCESS;
}

static int lsmd6s3_pedo_enable_sigmotion_interrupt(struct lsm6ds3_pedo *pedo_obj, u8 value)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;
    u8 addr = LSM6DS3_REG_INT1_CTRL;
    u8 databuf[2] = {0};
    int res;

    ST_FUN();
    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("read LSM6DS3_REG_INT1_CTRL register err!\n");
        return LSM6DS3_ERR_I2C;
    }

    databuf[0] &= ~(LSM6DS3_REG_INT1_CTRL_MASK_INT1_SIGN_MOT);
    databuf[0] |= value;

    res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x01);
    if(res)
    {
        ST_ERR("write LSM6DS3_REG_INT1_CTRL register err!\n");
        return LSM6DS3_ERR_I2C;
    }
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_pedo_read_chip_name(struct lsm6ds3_pedo *pedo_obj, u8 *buf, int bufsize)
{
    sprintf(buf, "%s", pedo_obj->name);
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_read_rawdata(struct lsm6ds3_pedo *pedo_obj, u16 *counter)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
	struct i2c_client *client = obj->client;
    int res = 0;
    u8 data[2];
    
    res = lsm6ds3_i2c_read_block(client, LSM6DS3_REG_STEP_COUNTER_L, data, 0x02);
    
    *counter = (u16)(data[1]<<8|data[0]);
    //ST_ERR("data = %d\n", data[0]);
    if(res)
    {
        ST_ERR("read pedometer sensor data register err!\n");
        *counter = 0;
        return LSM6DS3_ERR_I2C;
    }
    //*counter = data;
    return LSM6DS3_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_read_data(struct lsm6ds3_pedo *pedo_obj, u32 *value)
{
    struct lsm6ds3_data *obj = container_of(pedo_obj, struct lsm6ds3_data, lsm6ds3_pedo_data);
    u16 counter = 0;
    int res = 0;

    if(atomic_read(&pedo_obj->suspend))
    {
        ST_LOG("sensor in suspend read not data!\n");
        return 0;
    }

    res = lsm6ds3_pedo_read_rawdata(pedo_obj, &counter);
    if(res)
    {        
        ST_ERR("I2C error: ret value=%d", res);
        return res;
    }
    else
    {   
        if(counter>60000)
        {
            lsm6ds3_pedo_reset_counter(pedo_obj);
            pedo_obj->overflow++;
        }

        pedo_obj->data = counter + pedo_obj->overflow*60000; 
        *value = pedo_obj->data;

        if(atomic_read(&pedo_obj->trace) & ADX_TRC_IOCTL)//atomic_read(&obj->trace) & ADX_TRC_IOCTL
        {
            ST_LOG("step counter data: %d!\n", *value);
            dumpReg(obj);
        }
    }
    
    return 0;
}

int lsm6ds3_pedo_interrupt_step_d_handler(struct lsm6ds3_pedo *pedo_obj)
{
    return step_notify(TYPE_STEP_DETECTOR);
}

int lsm6ds3_pedo_interrupt_sigmotion_handler(struct lsm6ds3_pedo *pedo_obj)
{
    return step_notify(TYPE_SIGNIFICANT);
}

int lsm6ds3_pedo_interrupt_tilt_handler(struct lsm6ds3_pedo *pedo_obj)
{
    //return step_notify(TYPE_SIGNIFICANT);
    return 0;
}


/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_show_chipinfo_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    u8 strbuf[LSM6DS3_BUFSIZE];
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    
    lsm6ds3_pedo_read_chip_name(pedo_obj, strbuf, LSM6DS3_BUFSIZE);
    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_show_chipid_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    u8 strbuf[LSM6DS3_BUFSIZE] = "unkown chipid";
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }

    return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_show_sensordata_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    u32 value = 0;
    int res = 0;
    
    if(NULL == obj->client)
    {
        ST_ERR("i2c client is null!!\n");
        return 0;
    }
    res = lsm6ds3_pedo_read_data(pedo_obj, &value);
    return snprintf(buf, PAGE_SIZE, "%d\n", value);
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_show_power_status(struct device_driver *ddri, char *buf)
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
    return snprintf(buf, PAGE_SIZE, "0x%x\n", data);
}

/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_show_trace_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data; 

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&pedo_obj->trace));
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data; 
    int trace;
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%x", &trace))
    {
        atomic_set(&pedo_obj->trace, trace);
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }
    
    return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_show_chipinit_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&pedo_obj->trace)); 
    return res;
}
/*----------------------------------------------------------------------------*/
static ssize_t lsm6ds3_attr_pedo_store_chipinit_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return count;
    }

    lsm6ds3_pedo_init(pedo_obj, 0);
    dumpReg(obj);

    return count;
}
/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,             S_IRUGO, lsm6ds3_attr_pedo_show_chipinfo_value,      NULL);
static DRIVER_ATTR(chipid,               S_IRUGO, lsm6ds3_attr_pedo_show_chipid_value,        NULL);
static DRIVER_ATTR(sensordata,           S_IRUGO, lsm6ds3_attr_pedo_show_sensordata_value,    NULL);
static DRIVER_ATTR(power,                S_IRUGO, lsm6ds3_attr_pedo_show_power_status,        NULL);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, lsm6ds3_attr_pedo_show_trace_value,         lsm6ds3_attr_pedo_store_trace_value);
static DRIVER_ATTR(chipinit,   S_IWUSR | S_IRUGO, lsm6ds3_attr_pedo_show_chipinit_value,      lsm6ds3_attr_pedo_store_chipinit_value);
/*----------------------------------------------------------------------------*/
static struct driver_attribute *lsm6ds3_attr_pedo_list[] = {
    &driver_attr_chipinfo,     /*chip information*/
    &driver_attr_chipid,       /*chip id*/
    &driver_attr_sensordata,   /*dump sensor data*/
    &driver_attr_power,        /*show power reg*/
    &driver_attr_trace,        /*trace log*/
    &driver_attr_chipinit,
};
/*----------------------------------------------------------------------------*/
int lsm6ds3_pedo_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_pedo_list)/sizeof(lsm6ds3_attr_pedo_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, lsm6ds3_attr_pedo_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6ds3_attr_pedo_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}
/*----------------------------------------------------------------------------*/
int lsm6ds3_pedo_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_pedo_list)/sizeof(lsm6ds3_attr_pedo_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }
    

    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lsm6ds3_attr_pedo_list[idx]);
    }
    

    return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_open_report_data_intf(int open)
{
    return 0;
}

static int lsm6ds3_pedo_enable_nodata_intf(int en)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    int res =0;
    
    if((en == 1) && (pedo_obj->step_d_enabled == 0) && (pedo_obj->sigmotion_enabled == 0))
    {
        res = lsm6ds3_pedo_enable(pedo_obj, LSM6DS3_REG_TAP_CFG_PEDO_ENABLE);
    }
    else if ((en == 0) && (pedo_obj->step_d_enabled == 0) && (pedo_obj->sigmotion_enabled == 0))
    {
        res = lsm6ds3_pedo_enable(pedo_obj, LSM6DS3_REG_TAP_CFG_PEDO_DISABLE);
    }
    else
    {
        ST_LOG("pedometer has enabled\n");
    }

    if(res)
    {
        ST_ERR("lsm6ds3_pedo_enable fail!\n");
        return res;
    }

    pedo_obj->step_c_enabled = en;
    ST_LOG("lsm6ds3_pedo_enable_nodata_intf OK!\n");
    return 0;
}

static int lsm6ds3_pedo_set_delay_intf(u64 ns)
{
    return 0;
}

static int lsm6ds3_pedo_get_data_intf(uint32_t *value, int *status)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    int res = 0;

    res = lsm6ds3_pedo_read_data(pedo_obj, value);
    if (res<0)
    {
        ST_ERR("lsm6ds3_pedo_read_data error\n");
        return res;
    }
    *status = SENSOR_STATUS_ACCURACY_HIGH;

    return 0;
}

static int lsm6ds3_pedo_get_data_step_d(uint32_t *value, int *status)
{
    return 0;
}

static int lsm6ds3_pedo_get_data_significant(uint32_t *value, int *status)
{
    return 0;
}

static int lsm6ds3_pedo_enable_step_detect_intf(int en)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    int res = 0;
    ST_FUN();
    
    if (en)
    {
        res = lsmd6s3_pedo_enable_pedo_interrupt(pedo_obj, LSM6DS3_REG_INT1_CTRL_INT1_STEP_DETECTOR_ENABLE);
        if(res)
        {
            ST_ERR("lsmd6s3_pedo_enable_pedo_interrupt err!\n");
            return res;
        }

        if ((pedo_obj->step_c_enabled == 0) && (pedo_obj->sigmotion_enabled == 0))
        {
            res = lsm6ds3_pedo_enable(pedo_obj, LSM6DS3_REG_TAP_CFG_PEDO_ENABLE);
            if(res)
            {
                ST_ERR(" lsm6ds3_pedo_enable err!\n");
                return res;
            }
        }
    }
    else
    {
        res = lsmd6s3_pedo_enable_pedo_interrupt(pedo_obj, LSM6DS3_REG_INT1_CTRL_INT1_STEP_DETECTOR_DISABLE);
        if(res)
        {
            ST_ERR("lsmd6s3_pedo_enable_pedo_interrupt err!\n");
            return res;
        }

        if ((pedo_obj->step_c_enabled == 0) && (pedo_obj->sigmotion_enabled == 0))
        {
            res = lsm6ds3_pedo_enable(pedo_obj, LSM6DS3_REG_TAP_CFG_PEDO_DISABLE);
            if(res)
            {
                ST_ERR(" lsm6ds3_pedo_enable err!\n");
                return res;
            }
        }
    }
    pedo_obj->step_d_enabled = en;
    ST_LOG("lsm6ds3_pedo_enable_step_detect_intf (%d)\n",en);
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_pedo_enable_significant_intf(int en)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    int res = 0;
    ST_FUN();
    
    if (en)
    {
        res = lsmd6s3_pedo_enable_sigmotion_interrupt(pedo_obj, LSM6DS3_REG_INT1_CTRL_INT1_SIGN_MOT_ENABLE);
        if(res)
        {
            ST_ERR("lsmd6s3_pedo_enable_sigmotion_interrupt err!\n");
            return res;
        }
        if ((pedo_obj->step_c_enabled == 0) && (pedo_obj->step_d_enabled == 0))
        {
            res = lsm6ds3_pedo_enable(pedo_obj, LSM6DS3_REG_TAP_CFG_PEDO_ENABLE);
            if(res)
            {
                ST_ERR(" lsm6ds3_pedo_enable err!\n");
                return res;
            }
        }
    }
    else
    {
        res = lsmd6s3_pedo_enable_sigmotion_interrupt(pedo_obj, LSM6DS3_REG_INT1_CTRL_INT1_STEP_DETECTOR_DISABLE);
        if(res)
        {
            ST_ERR("lsmd6s3_pedo_enable_pedo_interrupt err!\n");
            return res;
        }
        if ((pedo_obj->step_c_enabled == 0) && (pedo_obj->step_d_enabled == 0))
        {
            res = lsm6ds3_pedo_enable(pedo_obj, LSM6DS3_REG_TAP_CFG_PEDO_DISABLE);
            if(res)
            {
                ST_ERR(" lsm6ds3_pedo_enable err!\n");
                return res;
            }
        }
    }
    pedo_obj->sigmotion_enabled = en;
    ST_LOG("lsm6ds3_pedo_enable_significant_intf (%d)\n",en);
    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_pedo_set_step_d_delay_intf(u64 ns)
{
    return 0;
}


/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_local_init(void)
{
    struct lsm6ds3_data *obj = obj_i2c_data;
    struct lsm6ds3_pedo *pedo_obj = &obj->lsm6ds3_pedo_data;
    int err = 0;
    int retry = 0;
    struct step_c_control_path ctl= {0};
    struct step_c_data_path data = {0};

    ST_FUN();

    atomic_set(&pedo_obj->trace, 0);
    atomic_set(&pedo_obj->suspend, 0);
    
    if((err = lsm6ds3_pedo_create_attr(&(lsm6ds3_pedo_init_info.platform_diver_addr->driver))))
    {
        ST_ERR("create attribute err = %d\n", err);
        goto exit_create_attr_failed;
    }

    for(retry = 0; retry < 3; retry++)
    {
        if((err = lsm6ds3_pedo_init(pedo_obj, 1)))
        {
            ST_ERR("lsm6ds3_pedo_device init cilent fail time: %d\n", retry);
            continue;
        }
    }

    if(err != 0)
        goto exit;

    sprintf(pedo_obj->name, "%s_PEDO", obj->name);

    ctl.open_report_data = lsm6ds3_pedo_open_report_data_intf;
	ctl.enable_nodata = lsm6ds3_pedo_enable_nodata_intf;
	ctl.enable_step_detect = lsm6ds3_pedo_enable_step_detect_intf;
	ctl.enable_significant = lsm6ds3_pedo_enable_significant_intf;
	ctl.step_c_set_delay = lsm6ds3_pedo_set_delay_intf;
	ctl.step_d_set_delay = lsm6ds3_pedo_set_step_d_delay_intf;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;

    err = step_c_register_control_path(&ctl);
    if(err)
    {
        ST_ERR("register step counter control path err\n");
        goto exit;
    }

    data.get_data = lsm6ds3_pedo_get_data_intf;
    data.vender_div = 1;
    data.get_data_step_d = lsm6ds3_pedo_get_data_step_d;
	data.get_data_significant = lsm6ds3_pedo_get_data_significant;
    err = step_c_register_data_path(&data);
    if(err) {
        ST_ERR("register acc data path err\n");
        goto exit;
    }

    ST_LOG("%s: OK\n", __func__);
    lsm6ds3_pedo_init_flag = 0;    
    return 0;

exit:
    ST_ERR("%s: err = %d\n", __func__, err);
    lsm6ds3_pedo_delete_attr(&(lsm6ds3_pedo_init_info.platform_diver_addr->driver));
exit_create_attr_failed:
    lsm6ds3_pedo_init_flag = -1;        
    return lsm6ds3_pedo_init_flag;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_pedo_local_remove(void)
{
    ST_FUN(); 

    lsm6ds3_pedo_delete_attr(&(lsm6ds3_pedo_init_info.platform_diver_addr->driver));
    
    return 0;
}
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
struct step_c_init_info lsm6ds3_pedo_init_info = {
        .name = "lsm6ds3",
        .init = lsm6ds3_pedo_local_init,
        .uninit = lsm6ds3_pedo_local_remove,
};


