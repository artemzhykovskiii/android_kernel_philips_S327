
#include "lsm6ds3.h"

static const struct i2c_device_id lsm6ds3_i2c_id[] = {{LSM6DS3_DEV_NAME,0},{}};

#ifdef CONFIG_OF
static const struct of_device_id lsm6ds3_of_match[] = {
    {.compatible = "mediatek,gsensor"},
    {}
};
#endif

static struct i2c_driver lsm6ds3_i2c_driver;

struct lsm6ds3_data *obj_i2c_data = NULL;

static DEFINE_MUTEX(lsm6ds3_i2c_mutex);
static DEFINE_MUTEX(lsm6ds3_op_mutex);

extern int step_c_driver_add(struct step_c_init_info *obj);

/*--------------------read function----------------------------------*/
int lsm6ds3_i2c_read_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{
    u8 beg = addr;
    int err;
    struct i2c_msg msgs[2]={{0},{0}};
    
    mutex_lock(&lsm6ds3_i2c_mutex);
    
    msgs[0].addr = client->addr;
    msgs[0].flags = 0;
    msgs[0].len =1;
    msgs[0].buf = &beg;

    msgs[1].addr = client->addr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = len;
    msgs[1].buf = data;
    
    if (!client)
    {
        mutex_unlock(&lsm6ds3_i2c_mutex);
        return -EINVAL;
    }
    else if (len > C_I2C_FIFO_SIZE) 
    {
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lsm6ds3_i2c_mutex);
        return -EINVAL;
    }
    err = i2c_transfer(client->adapter, msgs, sizeof(msgs)/sizeof(msgs[0]));
    
    if (err < 0) 
    {
        ST_ERR("i2c_transfer error: (%d %p %d) %d\n",addr, data, len, err);
        err = -EIO;
    } 
    else 
    {
        err = 0;
    }
    mutex_unlock(&lsm6ds3_i2c_mutex);
    return err; //if success will return 0
}

int lsm6ds3_i2c_write_block(struct i2c_client *client, u8 addr, u8 *data, u8 len)
{   /*because address also occupies one byte, the maximum length for write is 7 bytes*/
    int err, idx, num;
    u8 buf[C_I2C_FIFO_SIZE];
    err =0;
    mutex_lock(&lsm6ds3_i2c_mutex);
    if (!client)
    {
        mutex_unlock(&lsm6ds3_i2c_mutex);
        return -EINVAL;
    }
    else if (len >= C_I2C_FIFO_SIZE) 
    {        
        ST_ERR(" length %d exceeds %d\n", len, C_I2C_FIFO_SIZE);
        mutex_unlock(&lsm6ds3_i2c_mutex);
        return -EINVAL;
    }    

    num = 0;
    buf[num++] = addr;
    for (idx = 0; idx < len; idx++)
    {
        buf[num++] = data[idx];
    }

    err = i2c_master_send(client, buf, num);
    if (err < 0)
    {
        ST_ERR("send command error!!\n");
        mutex_unlock(&lsm6ds3_i2c_mutex);
        return -EFAULT;
    } 
    else
    {
        err = 0;
    }
    mutex_unlock(&lsm6ds3_i2c_mutex);
    return err; //if success will return transfer lenth
}
/*----------------------------------------------------------------------------*/

int lsm6ds3_open_ram_page(struct lsm6ds3_data *obj, u8 value)
{
    struct i2c_client *client = obj->client;
	u8 databuf[10] = {0}; 
	int res = 0;

    memset(databuf, 0, sizeof(u8)*10);

	if(lsm6ds3_i2c_read_block(client, LSM6DS3_REG_FUNC_CFG_ACCESS, databuf, 0x01))
	{
		ST_ERR("read LSM6DS3_REG_FUNC_CFG_ACCESS register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		ST_LOG("read LSM6DS3_REG_FUNC_CFG_ACCESS: 0x%x\n", databuf[0]);
	}
	
    databuf[0] &= ~LSM6DS3_REG_FUNC_CFG_ACCESS_MASK_FUNC_CFG_EN;
	databuf[0] |= value;
		
	res = lsm6ds3_i2c_write_block(client, LSM6DS3_REG_FUNC_CFG_ACCESS, databuf, 0x01);
	if(res)
	{
		ST_ERR("write LSM6DS3_REG_FUNC_CFG_ACCESS register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

int lsm6ds3_embed_func_enable(struct lsm6ds3_data *obj, u8 value)
{
    struct i2c_client *client = obj->client;
	u8 databuf[10] = {0}; 
    u8 addr = LSM6DS3_REG_CTRL10_C;
	int res = 0;
	//ST_FUN();    
	
    memset(databuf, 0, sizeof(u8)*10);

	if(lsm6ds3_i2c_read_block(client, addr, databuf, 0x01))
	{
		ST_ERR("read LSM6DS3_REG_CTRL10_C register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		ST_LOG("read LSM6DS3_REG_CTRL10_C: 0x%x\n", databuf[0]);
	}
	
    databuf[0] &= ~LSM6DS3_REG_CTRL10_C_MASK_FUNC_EN;
    databuf[0] |= value;
		
	res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x01);
	if(res)
	{
		ST_ERR("write LSM6DS3_REG_CTRL10_C register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	return LSM6DS3_SUCCESS;
}

void dumpReg(struct lsm6ds3_data *obj)
{
    struct i2c_client *client = obj->client;

    int i=0;
    u8 addr = 0x10;
    u8 regdata=0;
    for(i=0; i<10; i++)
    {
        lsm6ds3_i2c_read_block(client,addr,&regdata,1);
        ST_LOG("Reg addr=%x regdata=%x\n",addr,regdata);
        addr++;
    }
}

/*--------------------ADXL power control function----------------------------------*/
static void lsm6ds3_chip_power(struct lsm6ds3_data *obj, unsigned int on) 
{
    return;
}

static irqreturn_t lsm6ds3_interrupt_handler(int irq, void *data)
{
    struct lsm6ds3_data *obj = (struct lsm6ds3_data *)data;
    struct i2c_client *client = obj->client;
	
    u8 databuf[10] = {0}; 
    u8 addr = LSM6DS3_REG_FUNC_SRC;
	int res = 0;
    
    ST_FUN();
    
    memset(databuf, 0, sizeof(u8)*10);
    res = lsm6ds3_i2c_read_block(client, addr, databuf, 0x01); 
	if(res)
	{
		ST_ERR("read LSM6DS3_REG_FUNC_SRC register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		ST_LOG("read LSM6DS3_REG_FUNC_SRC: 0x%x\n", databuf[0]);
	}

    if (databuf[0]&LSM6DS3_REG_FUNC_SRC_MASK_STEP_DETECTED)
    {
        lsm6ds3_pedo_interrupt_step_d_handler(&obj->lsm6ds3_pedo_data);
    }

    if (databuf[0]&LSM6DS3_REG_FUNC_SRC_MASK_TILT_IA)
    {
        lsm6ds3_pedo_interrupt_tilt_handler(&obj->lsm6ds3_pedo_data);
    }

    if (databuf[0]&LSM6DS3_REG_FUNC_SRC_MASK_SIGN_MOTION_IA)
    {
        lsm6ds3_pedo_interrupt_sigmotion_handler(&obj->lsm6ds3_pedo_data);
    }

    return IRQ_HANDLED;
}

int lsm6ds3_set_interrupt(struct lsm6ds3_data *obj, u8 intenable)
{
    //struct lsm6ds3_data *obj = (lsm6ds3_data *)data;
    struct i2c_client *client = obj->client;
    struct device_node *node = NULL;
    int irq = 0;
    struct of_device_id gsensor_of_match[] = {
        { .compatible = "mediatek,gsensor", },
    };

    u8 databuf[2] = {0}; 
    u8 addr = LSM6DS3_REG_TAP_CFG;
	int res = 0;

	if(lsm6ds3_i2c_read_block(client, addr, databuf, 0x01))
	{
		ST_ERR("read LSM6DS3_REG_TAP_CFG register err!\n");
		return LSM6DS3_ERR_I2C;
	}
	else
	{
		ST_LOG("read LSM6DS3_REG_TAP_CFG: 0x%x\n", databuf[0]);
	}
	
    databuf[0] &= ~LSM6DS3_REG_TAP_CFG_MASK_LIR;
    databuf[0] |= LSM6DS3_REG_TAG_CFG_LIR;

	res = lsm6ds3_i2c_write_block(client, addr, databuf, 0x01);
	if(res)
	{
		ST_ERR("write LSM6DS3_REG_TAP_CFG register err!\n");
		return LSM6DS3_ERR_I2C;
	}

	node = of_find_matching_node(node, gsensor_of_match);
	if (node) {
		irq = irq_of_parse_and_map(node, 0);
		ST_LOG("irq number = %d!", irq);
	}

    res = request_threaded_irq(irq, NULL, (irq_handler_t)lsm6ds3_interrupt_handler, IRQF_TRIGGER_HIGH | IRQF_ONESHOT, "lsm6ds3_irq", obj);
    if (res<0)
    {
        ST_ERR("request irq error %d\n", res);
        return -1;
    }

    return LSM6DS3_SUCCESS;
}

static int lsm6ds3_read_chip_id(struct lsm6ds3_data *obj, u8 *data)
{
	struct i2c_client *client = obj->client;
	int res;
	
	res = lsm6ds3_i2c_read_block(client, LSM6DS3_REG_WHO_AM_I, data, 0x01);
	if (res < 0)
	{
		return LSM6DS3_ERR_I2C;
	}
	return LSM6DS3_SUCCESS;
}

static int lsm6ds3_check_device_id(struct lsm6ds3_data *obj)
{
	u8 buf;
	int res;

	res = lsm6ds3_read_chip_id(obj, &buf);
	if (res < 0)
	{
		ST_ERR("read chip id error\n");
		return LSM6DS3_ERR_I2C;
	}
	
	obj->chip_id = buf;
    ST_LOG("LSM6DS3 who am I = 0x%x\n", buf);
	if(buf ==LSM6DS3_FIXED_DEVID_LSM6DS3)
	{
		sprintf(obj->name, "LSM6DS3");
		return LSM6DS3_SUCCESS;
	}
	else if (buf ==LSM6DS3_FIXED_DEVID_LSM6DSL)
	{
		sprintf(obj->name, "LSM6DSL");
		return LSM6DS3_SUCCESS;
	}
	else if (buf ==LSM6DS3_FIXED_DEVID_LSM6DSM)
	{

		sprintf(obj->name, "LSM6DSM");
		return LSM6DS3_SUCCESS;
	}
	else
	{
		ST_ERR("not support chip id error\n");
		return LSM6DS3_ERR_IDENTIFICATION;
	}
}

static ssize_t lsm6ds3_attr_i2c_show_reg_value(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;

    u8 reg_value;
    ssize_t res;
    int err;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    err = lsm6ds3_i2c_read_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	if (err < 0)
	{
		res = snprintf(buf, PAGE_SIZE, "i2c read error!!\n");
        return res;
	}
	
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", reg_value); 
    return res;
}

static ssize_t lsm6ds3_attr_i2c_store_reg_value(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;

    u8 reg_value;
	int res;
	
    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%hhx", &reg_value))
    {
	    res = lsm6ds3_i2c_write_block(obj->client, obj->reg_addr, &reg_value, 0x01);
	    if(res < 0)
        {
            return LSM6DS3_ERR_I2C;
        }
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}

static ssize_t lsm6ds3_attr_i2c_show_reg_addr(struct device_driver *ddri, char *buf)
{
    struct lsm6ds3_data *obj = obj_i2c_data;

    ssize_t res;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    res = snprintf(buf, PAGE_SIZE, "0x%04X\n", obj->reg_addr); 
    return res;
}

static ssize_t lsm6ds3_attr_i2c_store_reg_addr(struct device_driver *ddri, const char *buf, size_t count)
{
    struct lsm6ds3_data *obj = obj_i2c_data;

	u8 reg_addr;

    if (obj == NULL)
    {
        ST_ERR("i2c_data obj is null!!\n");
        return 0;
    }
    
    if(1 == sscanf(buf, "0x%hhx", &reg_addr))
    {
        obj->reg_addr = reg_addr;
    }    
    else
    {
        ST_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
    }

    return count;
}
static DRIVER_ATTR(reg_value, S_IWUSR | S_IRUGO, lsm6ds3_attr_i2c_show_reg_value, lsm6ds3_attr_i2c_store_reg_value);
static DRIVER_ATTR(reg_addr,  S_IWUSR | S_IRUGO, lsm6ds3_attr_i2c_show_reg_addr,  lsm6ds3_attr_i2c_store_reg_addr);

static struct driver_attribute *lsm6ds3_attr_i2c_list[] = {
    &driver_attr_reg_value,
	&driver_attr_reg_addr,
};

int lsm6ds3_i2c_create_attr(struct device_driver *driver) 
{
    int idx, err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_i2c_list)/sizeof(lsm6ds3_attr_i2c_list[0]));
    if (driver == NULL)
    {
        return -EINVAL;
    }

    for(idx = 0; idx < num; idx++)
    {
        if((err = driver_create_file(driver, lsm6ds3_attr_i2c_list[idx])))
        {            
            ST_ERR("driver_create_file (%s) = %d\n", lsm6ds3_attr_i2c_list[idx]->attr.name, err);
            break;
        }
    }    
    return err;
}

int lsm6ds3_i2c_delete_attr(struct device_driver *driver)
{
    int idx ,err = 0;
    int num = (int)(sizeof(lsm6ds3_attr_i2c_list)/sizeof(lsm6ds3_attr_i2c_list[0]));

    if(driver == NULL)
    {
        return -EINVAL;
    }
    
    for(idx = 0; idx < num; idx++)
    {
        driver_remove_file(driver, lsm6ds3_attr_i2c_list[idx]);
    }
    
    return err;
}

/*----------------------------------------------------------------------------*/
static int lsm6ds3_suspend(struct i2c_client *client, pm_message_t msg) 
{
    struct lsm6ds3_data *obj = i2c_get_clientdata(client);
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
	struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
    int err = 0;
	
    ST_FUN();
    
    if((msg.event == PM_EVENT_SUSPEND) && (acc_obj->enabled == 1))
    {   
        mutex_lock(&lsm6ds3_op_mutex);
        if(obj == NULL)
        {    
            mutex_unlock(&lsm6ds3_op_mutex);
            ST_ERR("null pointer!!\n");
            return -EINVAL;
        }

        err = lsm6ds3_acc_set_power_mode(acc_obj, false);		
        if(err)
        {
            ST_ERR("write acc power control fail!!\n");
            mutex_unlock(&lsm6ds3_op_mutex);
            return err;   
        }

        err = lsm6ds3_gyro_set_power_mode(gyro_obj, false);
        if(err)
        {
            ST_ERR("write gyro power control fail!!\n");
            mutex_unlock(&lsm6ds3_op_mutex);
            return err;        
        }

        atomic_set(&acc_obj->suspend, 1);
		mutex_unlock(&lsm6ds3_op_mutex);
        lsm6ds3_chip_power(obj, 0);
    }
    
	ST_LOG("lsm6ds3 i2c suspended\n");
    return err;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_resume(struct i2c_client *client)
{
    struct lsm6ds3_data *obj = i2c_get_clientdata(client);
    struct lsm6ds3_acc *acc_obj = &obj->lsm6ds3_acc_data;
	struct lsm6ds3_gyro *gyro_obj = &obj->lsm6ds3_gyro_data;
    int err;
	
    ST_FUN();
    lsm6ds3_chip_power(obj, 1);
	if (acc_obj->enabled == 1) 
    {
	    mutex_lock(&lsm6ds3_op_mutex);
	    if(obj == NULL)
	    {
		    mutex_unlock(&lsm6ds3_op_mutex);
		    ST_ERR("null pointer!!\n");
		    return -EINVAL;
	    }
    
        err = lsm6ds3_acc_set_power_mode(acc_obj, true);
        if(err)
        {
            mutex_unlock(&lsm6ds3_op_mutex);
            ST_ERR("write acc power control fail!!\n");
            return err;        
        }

        err = lsm6ds3_gyro_set_power_mode(gyro_obj, true);
        if(err)
        {
            mutex_unlock(&lsm6ds3_op_mutex);
            ST_ERR("write gyro power control fail!!\n");
            return err;        
        }

        atomic_set(&acc_obj->suspend, 0);
        mutex_unlock(&lsm6ds3_op_mutex);
    }
	ST_LOG("lsm6ds3 i2c resumed\n");
    return 0;
}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{    
    strcpy(info->type, LSM6DS3_DEV_NAME);
    return 0;
}

/*----------------------------------------------------------------------------*/

static int lsm6ds3_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct lsm6ds3_data *obj;

    int err = 0;
	ST_FUN();
	
	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
    {
        err = -ENOMEM;
		return err;
    }
    memset(obj, 0, sizeof(struct lsm6ds3_data));
	obj_i2c_data = obj;
	client->addr = 0xD4 >> 1;
    obj->client = client;
    i2c_set_clientdata(client, obj);
	
	err = lsm6ds3_check_device_id(obj);
	if (err)
	{
        ST_ERR("check device error!\n");
		goto exit_check_device_failed;
	}

    err = lsm6ds3_i2c_create_attr(&lsm6ds3_i2c_driver.driver);
	if (err)
	{
        ST_ERR("create attr error!\n");
		goto exit_create_attr_failed;
	}
#ifdef CONFIG_LSM6DS3_ACC_DRY
	//TODO: request irq for data ready or pedometer, SMD, TILT, etc.
#endif	
    lsm6ds3_set_interrupt(obj, 1);

    acc_driver_add(&lsm6ds3_acc_init_info);
	gyro_driver_add(&lsm6ds3_gyro_init_info);

	step_c_driver_add(&lsm6ds3_pedo_init_info);

#if 0 // tile function
	tilt_driver_add(&lsm6ds3_tile_init_info);
#endif

	ST_LOG("lsm6ds3_i2c_probe exit\n");
    return 0;
exit_create_attr_failed:
exit_check_device_failed:
    kfree(obj);
	return err;

}
/*----------------------------------------------------------------------------*/
static int lsm6ds3_i2c_remove(struct i2c_client *client)
{
	lsm6ds3_i2c_delete_attr(&lsm6ds3_i2c_driver.driver);
    i2c_unregister_device(client);
    kfree(i2c_get_clientdata(client));
    return 0;
}
/*----------------------------------------------------------------------------*/
static struct i2c_driver lsm6ds3_i2c_driver = {
    .driver = {
        .name           = LSM6DS3_DEV_NAME,
#ifdef CONFIG_OF
        .of_match_table = lsm6ds3_of_match,
#endif
    },
    .probe              = lsm6ds3_i2c_probe,
    .remove             = lsm6ds3_i2c_remove,
    .detect             = lsm6ds3_i2c_detect,
    .suspend            = lsm6ds3_suspend,
    .resume             = lsm6ds3_resume,
    .id_table           = lsm6ds3_i2c_id,
};

/*----------------------------------------------------------------------------*/
static int __init lsm6ds3_module_init(void)
{
    ST_FUN();
    if(i2c_add_driver(&lsm6ds3_i2c_driver))
    {
        ST_ERR("add acc driver error\n");
        return -1;
    }

    return 0;
}
/*----------------------------------------------------------------------------*/
static void __exit lsm6ds3_module_exit(void)
{
    i2c_del_driver(&lsm6ds3_i2c_driver);

    ST_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(lsm6ds3_module_init);
module_exit(lsm6ds3_module_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("LSM6DS3 I2C driver");
