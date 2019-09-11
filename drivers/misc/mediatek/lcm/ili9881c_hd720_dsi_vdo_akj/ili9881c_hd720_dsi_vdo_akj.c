#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"
#include <linux/io.h>

#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
/** Local Constants **/
#define LCM_DSI_CMD_MODE								0
#define FRAME_WIDTH  									(720)
#define FRAME_HEIGHT 									(1280)
#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xF1   // END OF REGISTERS MARKER
#define I2C_ID_NAME "tps65132"

#define LCD_BIAS_EN 122  

/** Local Variables **/
static LCM_UTIL_FUNCS lcm_util = {0};
static struct i2c_client *tps65132_i2c_client = NULL;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_LCD_BIAS_EN(v)		(lcm_util.set_gpio_lcd_enp_bias(v))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define dsi_set_cmdq_V3(para_tbl,size,force_update)         lcm_util.dsi_set_cmdq_V3(para_tbl,size,force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg						lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)        


//wq add start

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);

static const struct of_device_id _lcm_i2c_of_match[] = {
	{
		.compatible = "mediatek,I2C_LCD_BIAS",
	},
};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	.driver		= {
		.owner   	= THIS_MODULE,
		.name	= "tps65132",
		.of_match_table = _lcm_i2c_of_match,
	},
};

static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk("[LCM]**%s**NT: info==>name=%s addr=0x%x\n",__func__,client->name,client->addr);
	tps65132_i2c_client  = client;		
	return 0;      
}


static int tps65132_remove(struct i2c_client *client)
{  	
	printk( "[LCM]**%s**\n",__func__);
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	
	if (client == NULL) {
		pr_warn("[LCM] i2c_client = NULL, skip tps65132_write_bytes\n");
		return 0;
	}
	write_data[0]= addr;
	write_data[1] = value;
	ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("[LCM] tps65132 write data fail !!\n");	
	return ret ;
}
/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

	printk( "[LCM]**%s**\n",__func__);
	i2c_add_driver(&tps65132_iic_driver);
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	printk( "[LCM]**%s**\n",__func__);
	i2c_del_driver(&tps65132_iic_driver);  
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL"); 
//wq add end 

//wq add it

//static LCM_setting_table_V3 lcm_deep_sleep_mode_in_setting[] = {
	// Display off sequence
//	{0x05,0x28, 1, {0x00}},
//	{REGFLAG_DELAY, 120, {}},
	// Sleep Mode On
//	{0x05,0x10, 1, {0x00}},
//	{REGFLAG_DELAY, 50, {}},
//	{REGFLAG_END_OF_TABLE, 0x00, {}},
//};


//add end

//GIP_1
static LCM_setting_table_V3 lcm_initialization_setting[] = {
	{0x39, 0xFF,3,{0x98,0x81,0x03}},
	{0x15,0x01,1,{0x00}},
	{0x15,0x02,1,{0x00}},
	{0x15,0x03,1,{0x73}},
	{0x15,0x04,1,{0x00}},
	{0x15,0x05,1,{0x00}},
	{0x15,0x06,1,{0x0A}},
	{0x15,0x07,1,{0x00}},
	{0x15,0x08,1,{0x00}},
	{0x15,0x09,1,{0x01}},
	{0x15,0x0a,1,{0x00}},
	{0x15,0x0b,1,{0x00}},
	{0x15,0x0c,1,{0x01}},
	{0x15,0x0d,1,{0x00}},
	{0x15,0x0e,1,{0x00}},
	{0x15,0x0f,1,{0x1D}},
	{0x15,0x10,1,{0x1D}},
	{0x15,0x11,1,{0x00}},
	{0x15,0x12,1,{0x00}},
	{0x15,0x13,1,{0x00}},
	{0x15,0x14,1,{0x00}},
	{0x15,0x15,1,{0x00}},
	{0x15,0x16,1,{0x00}},
	{0x15,0x17,1,{0x00}},
	{0x15,0x18,1,{0x00}},
	{0x15,0x19,1,{0x00}},
	{0x15,0x1a,1,{0x00}},
	{0x15,0x1b,1,{0x00}},
	{0x15,0x1c,1,{0x00}},
	{0x15,0x1d,1,{0x00}},
	{0x15,0x1e,1,{0x40}},
	{0x15,0x1f,1,{0x80}},
	{0x15,0x20,1,{0x06}},
	{0x15,0x21,1,{0x02}},
	{0x15,0x22,1,{0x00}},
	{0x15,0x23,1,{0x00}},
	{0x15,0x24,1,{0x00}},
	{0x15,0x25,1,{0x00}},
	{0x15,0x26,1,{0x00}},
	{0x15,0x27,1,{0x00}},
	{0x15,0x28,1,{0x33}},
	{0x15,0x29,1,{0x03}},
	{0x15,0x2a,1,{0x00}},
	{0x15,0x2b,1,{0x00}},
	{0x15,0x2c,1,{0x00}},
	{0x15,0x2d,1,{0x00}},
	{0x15,0x2e,1,{0x00}},
	{0x15,0x2f,1,{0x00}},
	{0x15,0x30,1,{0x00}},
	{0x15,0x31,1,{0x00}},
	{0x15,0x32,1,{0x00}},
	{0x15,0x33,1,{0x00}},
	{0x15,0x34,1,{0x04}},
	{0x15,0x35,1,{0x00}},
	{0x15,0x36,1,{0x00}},
	{0x15,0x37,1,{0x00}},
	{0x15,0x38,1,{0x3C}},
	{0x15,0x39,1,{0x00}},
	{0x15,0x3a,1,{0x40}},
	{0x15,0x3b,1,{0x40}},
	{0x15,0x3c,1,{0x00}},
	{0x15,0x3d,1,{0x00}},
	{0x15,0x3e,1,{0x00}},
	{0x15,0x3f,1,{0x00}},
	{0x15,0x40,1,{0x00}},
	{0x15,0x41,1,{0x00}},
	{0x15,0x42,1,{0x00}},
	{0x15,0x43,1,{0x00}},
	{0x15,0x44,1,{0x00}},
	{0x15,0x50,1,{0x01}},
	{0x15,0x51,1,{0x23}},
	{0x15,0x52,1,{0x45}},
	{0x15,0x53,1,{0x67}},
	{0x15,0x54,1,{0x89}},
	{0x15,0x55,1,{0xab}},
	{0x15,0x56,1,{0x01}},
	{0x15,0x57,1,{0x23}},
	{0x15,0x58,1,{0x45}},
	{0x15,0x59,1,{0x67}},
	{0x15,0x5a,1,{0x89}},
	{0x15,0x5b,1,{0xab}},
	{0x15,0x5c,1,{0xcd}},
	{0x15,0x5d,1,{0xef}},
	{0x15,0x5e,1,{0x11}},
	{0x15,0x5f,1,{0x01}},
	{0x15,0x60,1,{0x00}},
	{0x15,0x61,1,{0x15}},
	{0x15,0x62,1,{0x14}},
	{0x15,0x63,1,{0x0E}},
	{0x15,0x64,1,{0x0F}},
	{0x15,0x65,1,{0x0C}},
	{0x15,0x66,1,{0x0D}},
	{0x15,0x67,1,{0x06}},
	{0x15,0x68,1,{0x02}},
	{0x15,0x69,1,{0x07}},
	{0x15,0x6a,1,{0x02}},
	{0x15,0x6b,1,{0x02}},
	{0x15,0x6c,1,{0x02}},
	{0x15,0x6d,1,{0x02}},
	{0x15,0x6e,1,{0x02}},
	{0x15,0x6f,1,{0x02}},
	{0x15,0x70,1,{0x02}},
	{0x15,0x71,1,{0x02}},
	{0x15,0x72,1,{0x02}},
	{0x15,0x73,1,{0x02}},
	{0x15,0x74,1,{0x02}},
	{0x15,0x75,1,{0x01}},
	{0x15,0x76,1,{0x00}},
	{0x15,0x77,1,{0x14}},
	{0x15,0x78,1,{0x15}},
	{0x15,0x79,1,{0x0E}},
	{0x15,0x7a,1,{0x0F}},
	{0x15,0x7b,1,{0x0C}},
	{0x15,0x7c,1,{0x0D}},
	{0x15,0x7d,1,{0x06}},
	{0x15,0x7e,1,{0x02}},
	{0x15,0x7f,1,{0x07}},
	{0x15,0x80,1,{0x02}},
	{0x15,0x81,1,{0x02}},
	{0x15,0x82,1,{0x02}},
	{0x15,0x83,1,{0x02}},
	{0x15,0x84,1,{0x02}},
	{0x15,0x85,1,{0x02}},
	{0x15,0x86,1,{0x02}},
	{0x15,0x87,1,{0x02}},
	{0x15,0x88,1,{0x02}},
	{0x15,0x89,1,{0x02}},
	{0x15,0x8A,1,{0x02}},

	{0x39, 0xFF,3,{0x98,0x81,0x04}},
	//{0x15,0x00,1,{0x80}},//00-3LANE 80-4LANE
	{0x15,0x6C,1,{0x15}},
	{0x15,0x6E,1,{0x2B}},
	{0x15,0x6F,1,{0x33}},
	{0x15,0x8D,1,{0x18}},
	{0x15,0x87,1,{0xBA}},
	{0x15,0x26,1,{0x76}},
	{0x15,0xB2,1,{0xD1}},
	{0x15,0xB5,1,{0x06}},
	{0x15,0x3A,1,{0x24}},
	{0x15,0x35,1,{0x1F}},


	{0x39, 0xFF,3,{0x98,0x81,0x01}},
	{0x15,0x22,1,{0x0A}},
	{0x15,0x31,1,{0x00}},
	{0x15,0x40,1,{0x33}},
	{0x15,0x53,1,{0xA2}},
	{0x15,0x55,1,{0x92}},
	{0x15,0x50,1,{0x96}},
	{0x15,0x51,1,{0x96}},
	{0x15,0x60,1,{0x22}},
	{0x15,0x61,1,{0x00}},
	{0x15,0x62,1,{0x19}},
	{0x15,0x63,1,{0x00}},
	{0x15,0xA0,1,{0x08}},
	{0x15,0xA1,1,{0x11}},
	{0x15,0xA2,1,{0x19}},
	{0x15,0xA3,1,{0x0D}},
	{0x15,0xA4,1,{0x0D}},
	{0x15,0xA5,1,{0x1E}},
	{0x15,0xA6,1,{0x14}},
	{0x15,0xA7,1,{0x17}},
	{0x15,0xA8,1,{0x4F}},
	{0x15,0xA9,1,{0x1A}},
	{0x15,0xAA,1,{0x27}},
	{0x15,0xAB,1,{0x49}},
	{0x15,0xAC,1,{0x1A}},
	{0x15,0xAD,1,{0x18}},
	{0x15,0xAE,1,{0x4C}},
	{0x15,0xAF,1,{0x22}},
	{0x15,0xB0,1,{0x27}},
	{0x15,0xB1,1,{0x4B}},
	{0x15,0xB2,1,{0x60}},
	{0x15,0xB3,1,{0x39}},
	{0x15,0xC0,1,{0x08}},
	{0x15,0xC1,1,{0x11}},
	{0x15,0xC2,1,{0x19}},
	{0x15,0xC3,1,{0x0D}},
	{0x15,0xC4,1,{0x0D}},
	{0x15,0xC5,1,{0x1E}},
	{0x15,0xC6,1,{0x14}},
	{0x15,0xC7,1,{0x17}},
	{0x15,0xC8,1,{0x4F}},
	{0x15,0xC9,1,{0x1A}},
	{0x15,0xCA,1,{0x27}},
	{0x15,0xCB,1,{0x49}},
	{0x15,0xCC,1,{0x1A}},
	{0x15,0xCD,1,{0x18}},
	{0x15,0xCE,1,{0x4C}},
	{0x15,0xCF,1,{0x33}},
	{0x15,0xD0,1,{0x27}},
	{0x15,0xD1,1,{0x4B}},
	{0x15,0xD2,1,{0x60}},
	{0x15,0xD3,1,{0x39}},

	{0x39, 0xFF,3,{0x98,0x81,0x00}},
	{0x15,0x35,1,{0x00}},
	{0x15,0x36,1,{0x03}},

	{0x05,0x11,0,{0x00}},  // Sleep-Out 
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,120,{}},                                 
	{0x05,0x29,0,{0x00}}, 

};


//static LCM_setting_table_V3 lcm_ata_setting[] = {
//	{0x15, 0x51, 1,  {0x05}},
//	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,10,{}},
//};
//
#if 0
static LCM_setting_table_V3 test[] = {

	{0x37, 0x03,  0, {}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,20,{}},
};
#endif

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// --------------------------------------------------------------------------
//static void read_lcm_reg(unsigned char reg, int num_para)
//{
//	unsigned char buffer[10]={0xFF};
//	int array[4];
//	int i=0;
//	array[0] = (num_para<<16)|0x00003700;
//
//	dsi_set_cmdq(array, 1, 1);
//	read_reg_v2(reg, buffer, num_para);
//	printk("0x%x=\n",reg);
//	for(i=0;i<num_para;i++)
//		printk("[LCM]**0x%x**\n",buffer[i]);
//}
//
//
//static void lcm_read_test(void)
//{ 
//	read_lcm_reg(0x04,3);
//	read_lcm_reg(0xB2,10);
//}
//

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	printk("[LCM]**%s**\n",__func__);	
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type   = LCM_TYPE_DSI;
	params->width  = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

	// enable tearing-free
	params->dbi.te_mode 						= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity					= LCM_POLARITY_RISING;
	//wq	params->dsi.mode   						= SYNC_PULSE_VDO_MODE;
	params->dsi.mode   						= SYNC_EVENT_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM			 			= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 				= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    				= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding       				= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format         				= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	//	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	// 	params->dsi.word_count=FRAME_WIDTH*3;	

	params->dsi.vertical_sync_active				= 8;//VSPW
	params->dsi.vertical_backporch					= 24;//VBPD
	params->dsi.vertical_frontporch					= 16;//VFPD
	params->dsi.vertical_active_line				= FRAME_HEIGHT;
	params->dsi.horizontal_sync_active				= 64;//HSPW
	params->dsi.horizontal_backporch				= 120;//HBPD
	params->dsi.horizontal_frontporch				= 88;//HFPD
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;

#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 350; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 222;  //220; //this value must be in MTK suggested table: 920MHz MIPI_CLK /2 =460
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.cont_clock						= 0;
	params->dsi.clk_lp_per_line_enable 				= 0;
	/* esd check */
	params->dsi.esd_check_enable 					= 1;
	params->dsi.customization_esd_check_enable		        = 0;//TE sync 
	params->dsi.lcm_esd_check_table[0].cmd          		= 0xD9;
	params->dsi.lcm_esd_check_table[0].count        		= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] 		= 0x80;

}

//wq void init_lcm_registers(void)
//static struct LCM_setting_table lcm_initialization_setting[] = {
// {0x39, 0xB9, 3, {0xFF, 0x83, 0x92}},
//}

static void lcm_init(void)
{
	unsigned char cmd_0 = 0x00;
	unsigned char data_0 = 0x0E;
	unsigned char cmd_1 = 0x01;
	unsigned char data_1 = 0x0E;
	unsigned char cmd_3 = 0x03;
	unsigned char data_3 = 0x03;
	int gpio_ret = 0;
	int ret = 0;

	printk("[LCM]**%s**\n",__func__);
	/*Enable LCM Gate IC */
	gpio_ret = gpio_request(LCD_BIAS_EN,"lcd_bias_en");

	gpio_direction_output(LCD_BIAS_EN,0);
	gpio_set_value(LCD_BIAS_EN,1); 
	MDELAY(5); 
	gpio_set_value(LCD_BIAS_EN,1);
	gpio_free(LCD_BIAS_EN);


	//0x00 0x06
	ret = tps65132_write_bytes(cmd_0, data_0);
	if (ret < 0)
		printk("[LCM][KERNEL]r63419----tps6132---cmd_0=%0x-- i2c write error-----\n", cmd_0);
	else
		printk("[LCM][KERNEL]r63419----tps6132---cmd_0=%0x-- i2c write success-----\n", cmd_0);

	//0x01 0x20
	ret = tps65132_write_bytes(cmd_1, data_1);
	if (ret < 0)
		printk("[LCM][KERNEL]r63419----tps6132---cmd_1=%0x-- i2c write error-----\n", cmd_1);
	else
		printk("[LCM][KERNEL]r63419----tps6132---cmd_1=%0x-- i2c write success-----\n", cmd_1);

	//0x03 0x03
	ret = tps65132_write_bytes(cmd_3, data_3);
	if (ret < 0)
		printk("[LCM][KERNEL]r63419----tps6132---cmd_3=%0x-- i2c write error-----\n", cmd_3);
	else
		printk("[LCM][KERNEL]r63419----tps6132---cmd_3=%0x-- i2c write success-----\n", cmd_3);

	MDELAY(5);

	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);	
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(150);

	/*Send Initial commands*/
 	dsi_set_cmdq_V3(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(lcm_initialization_setting[0]), 1);

	// lcm_read_test();	
}

static void lcm_suspend(void)
{

	unsigned char cmd_3 = 0x03;
	unsigned char data_3 = 0x00;
	int ret = 0;
	int gpio_ret = 0;

	printk( "[LCM]**%s**\n",__func__);
//	dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);

	SET_RESET_PIN(0);
	//0x03 0x00
	ret = tps65132_write_bytes(cmd_3,data_3);
	if (ret < 0)
		printk("[LCM][KERNEL]r63419----tps6132---cmd_3=%0x-- i2c write error-----\n", cmd_3);
	else
		printk("[LCM][KERNEL]r63419----tps6132---cmd_3=%0x-- i2c write success-----\n", cmd_3);

	gpio_ret = gpio_request(LCD_BIAS_EN,"lcd_bias_en");
	gpio_direction_output(LCD_BIAS_EN,0);
	gpio_set_value(LCD_BIAS_EN,0); 
	MDELAY(5); 
	gpio_set_value(LCD_BIAS_EN,0);

	gpio_free(LCD_BIAS_EN);

	MDELAY(30);//for ESD test, can't eliminate
	SET_RESET_PIN(0);
	MDELAY(30);//for ESD test, can't eliminate

}

static void lcm_resume(void)
{
	lcm_init();
}

//static unsigned int lcm_ata_check(unsigned char *buffer)
//{
//	int ret=0;
//	int array[4];
//	unsigned char buf[4]={0xFF};
//
//	array[0] = 0x00023700;
//	dsi_set_cmdq(array, 1, 1);
//	read_reg_v2(0x52, buf, 1);
//
//	printk("0x52=0x%x\n",buf[0]);	
//
//	if(buf[0]==0x05)
//		ret=1;
//	else
//		ret=0;
//
//	dsi_set_cmdq_V3(lcm_ata_setting, sizeof(lcm_ata_setting) / sizeof(lcm_ata_setting[0]), 1);
//
//	return ret;
//
//}

LCM_DRIVER ili9881c_hd720_dsi_vdo_akj_lcm_drv= 
{
	.name	        = "ili9881c_hd720_dsi_vdo_akj",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init          	= lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//	.ata_check 	= lcm_ata_check,
};
