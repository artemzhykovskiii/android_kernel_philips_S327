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
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER
#define I2C_ID_NAME "tps65132"

#define LCD_BIAS_EN 122  
//#define USE_OLD_LCM

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

#ifdef USE_OLD_LCM
static LCM_setting_table_V3 lcm_initialization_setting_kd[] = {

	{0x39, 0xb9,  3, {0xff,0x83,0x94}},
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,20,{}},
	{0x39, 0xb6, 2,  {0xA8,0xA8}},	
	{0x39, 0xba, 2,  {0x72,0x82}},
	{0x39, 0xb1, 15, {0x6c,0x15,0x15,0x24,0xE4,0x11,0xf1,0x80,0xe8,0xd8,0x23,0x80,0xc0,0xd2,0x58}},
	{0x39, 0xb2, 11, {0x00,0x64,0x10,0x07,0x32,0x1C,0x08,0x08,0x1C,0x4D,0x00}},
	{0x15, 0xbc, 1,  {0x07}},
	{0x15, 0x53, 1,  {0x20}},//add for ATA test	
	{0x15, 0x51, 1,  {0x05}},//add for ATA test
	{0x39, 0xbf, 3,  {0x41, 0x0E,0x01}},		
	{0x39, 0xb4, 12, {0x00,0xFF,0x50,0x51,0x40,0x41,0x40,0x41,0x02,0x6A,0x02,0x6A}},	
	{0x39, 0xd3, 30, {0x00,0x06,0x00,0x40,0x07,0x00,0x00,0x32,0x10,0x08,0x00,0x08,0x52,0x15,0x0F,0x05,
				 0x0F,0x32,0x10,0x00,0x00,0x00,0x47,0x44,0x0C,0x0C,0x47,0x0C,0x0C,0x47}},	
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,5,{}},
	{0x39, 0xd5, 44, {0x20,0x21,0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x18,0x18,0x18,0x18,0x18,0x18,
				 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
				 0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x18,0x18,0x24,0x25}},
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,5,{}},
	{0x39, 0xd6, 44, {0x24,0x25,0x07,0x06,0x05,0x04,0x03,0x02,0x01,0x00,0x18,0x18,0x18,0x18,0x18,0x18,
				 0x18,0x18,0x58,0x58,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,
				 0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x19,0x19,0x20,0x21}},
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,5,{}},
	{0x39, 0xe0, 42, {0x02,0x07,0x09,0x36,0x3C,0x3F,0x18,0x42,0x07,0x0A,0x0c,0x17,0x0F,0x12,0x14,0x12,
				 0x13,0x07,0x14,0x19,0x1A,0x00,0x05,0x09,0x36,0x3C,0x3F,0x18,0x42,0x07,0x0A,0x0C,
				 0x17,0x0F,0x12,0x14,0x12,0x13,0x07,0x14,0x19,0x1A}},	
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,5,{}},
	{0x15, 0xcc, 1,  {0x01}},
	{0x15, 0x36, 1,  {0x02}},//0x02
	{0x39, 0xc0, 2,  {0x30,0x14}},	
	{0x39, 0xc7, 4,  {0x00,0xC0,0x40,0xC0}},	
	{0x05, 0x11,0,{}},
	{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,120,{}},	
	{0x05, 0x29,0,{}},
	//{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,20,{}},
};
#else
//wq static LCM_setting_table_V3 lcm_initialization_setting_kd2[] = {
#if 0
{0x39, 0xB9,  3, {0xFF,0x83,0x94}},
{0x39, 0xBA,  2, {0x72,0x83}},//////0x73,0x83
{0x39, 0xB1,  15, {0x6A,0x15,0x15,0x13,0x04,0x11,0xF1,0x80,0xEC,0x55,0x23,0x80,0xC0,0xD2,0x58}}, 
{0x39, 0xB2,  11, {0x00,0x64,0x10,0x07,0x12,0x1C,0x08,0x08,0x1C,0x4D,0x00}},
{0x39, 0xB4,  12, {0x00,0xFF,0x03,0x5A,0x03,0x5A,0x03,0x5A,0x01,0x6A,0x01,0x6A}}, 
{0x39, 0xBC,  1, {0x07}},
{0x39, 0xBF,  3, {0x41,0x0E,0x01}},
{0x39, 0xD2,  1, {0x55}},
{0x39, 0xD3,  33, {0x00,0x0F,0x00,0x40,0x1A,0x08,0x00,0x32,0x10,0x08,0x00,0x08
	,0x54,0x15,0x0F,0x05,0x04,0x02,0x12,0x10,0x05,0x07,0x33
		,0x34,0x0C,0x0C,0x37,0x10,0x07,0x07,0x17,0x11,0x08}},
{0x39, 0xD5,  44, {0x19,0x19,0x18,0x18,0x1B,0x1B,0x1A,0x1A,0x04,0x05,0x06,0x07
	,0x00,0x01,0x02,0x03,0x20,0x21,0x18,0x18,0x22,0x23,0x18
		,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18
		,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
{0x39, 0xD6,  44, {0x18,0x18,0x19,0x19,0x1B,0x1B,0x1A,0x1A,0x03,0x02,0x01,0x00
	,0x07,0x06,0x05,0x04,0x23,0x22,0x18,0x18,0x21,0x20,0x18
		,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18
		,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18,0x18}},
{0x39, 0xB6,  2, {0x82,0x82}},
{0x39, 0xE0,  42, {0x00,0x03,0x09,0x2D,0x33,0x3F,0x16,0x38,0x06,0x0A,0x0C,0x18
	,0x0D,0x11,0x13,0x11,0x11,0x06,0x10,0x12,0x16,0x00,0x03
		,0x09,0x2D,0x33,0x3F,0x16,0x38,0x06,0x0A,0x0C,0x18,0x0D
		,0x11,0x13,0x11,0x11,0x06,0x10,0x12,0x16}},
{0x39, 0xC0,  2, {0x30,0x14}},
{0x39, 0xC7,  4, {0x00,0xC0,0x00,0xC0}},
{0x15, 0xCC,  1, {0x09}},	//0x05
{0x39, 0xDF,  1, {0x88}},
{0x05, 0x11,0,{}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,120,{}},

	// {0x39, 0xCC,  1, {0x0F}},	//0x05

{0x05, 0x29,0,{}},
{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,10,{}},

#endif

	//wq };

#endif

//	static LCM_setting_table_V3 lcm_ata_setting[] = {
//		{0x15, 0x51, 1,  {0x05}},
//		{REGFLAG_ESCAPE_ID,REGFLAG_DELAY_MS_V3,10,{}},
//	};

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
//wq 	params->dsi.LANE_NUM			 			= LCM_THREE_LANE;
	params->dsi.LANE_NUM			 			= LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order 				= LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    				= LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding       				= LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format         				= LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	//wq	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;
	//wq 	params->dsi.word_count=FRAME_WIDTH*3;	

	params->dsi.vertical_sync_active				= 2;  
	params->dsi.vertical_backporch					= 16;  
	params->dsi.vertical_frontporch					= 9;   
	params->dsi.vertical_active_line				= FRAME_HEIGHT;
	params->dsi.horizontal_sync_active				= 150;  
	params->dsi.horizontal_backporch				= 150;  
	params->dsi.horizontal_frontporch				= 150;  
	params->dsi.horizontal_active_pixel				= FRAME_WIDTH;


#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 350; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 270;  //220; //this value must be in MTK suggested table: 920MHz MIPI_CLK /2 =460
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
	params->dsi.customization_esd_check_enable		        = 1;//TE sync 
	params->dsi.lcm_esd_check_table[0].cmd          		= 0xD9;
	params->dsi.lcm_esd_check_table[0].count        		= 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] 		= 0x80;

}

static void init_lcm_registers(void)
{
	unsigned int data_array[16];
	data_array[0] = 0x00043902;
	data_array[1] = 0x9483FFB9;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(20);
	data_array[0] = 0x00033902;
	data_array[1] = 0x008373BA;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00103902;
	data_array[1] = 0x11116AB1;
	data_array[2] = 0xF1110416;
	data_array[3] = 0x2354e880;
	data_array[4] = 0x58D2C080;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0] = 0x000C3902;
	data_array[1] = 0x106400B2;
	data_array[2] = 0x081C1207;
	data_array[3] = 0x004D1C08;
	dsi_set_cmdq(data_array, 4, 1);
	MDELAY(1);
	data_array[0] = 0x000D3902;
	data_array[1] = 0x03FF00B4;
	data_array[2] = 0x035A035A;
	data_array[3] = 0x016A015A;
	data_array[4] = 0x0000006A;
	dsi_set_cmdq(data_array, 5, 1);
	MDELAY(1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000007BC;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00043902;
	data_array[1] = 0x010E41BF;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000011D2;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00213902;
	data_array[1] = 0x000F00D3;
	data_array[2] = 0x00081A40;
	data_array[3] = 0x00081032;
	data_array[4] = 0x0F155408;
	data_array[5] = 0x12020405;
	data_array[6] = 0x33070510;
	data_array[7] = 0x370C0C34;
	data_array[8] = 0x11170707;
	data_array[9] = 0x00000008;
	dsi_set_cmdq(data_array, 10, 1);
	MDELAY(1);
	data_array[0] = 0x002D3902;
	data_array[1] = 0x181919D5;
	data_array[2] = 0x1A1B1B18;
	data_array[3] = 0x0605041A;
	data_array[4] = 0x02010007;
	data_array[5] = 0x18212003;
	data_array[6] = 0x18232218;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18181818;
	data_array[10] = 0x18181818;
	data_array[11] = 0x18181818;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
	MDELAY(1);
	data_array[0] = 0x002D3902;
	data_array[1] = 0x191818D6;
	data_array[2] = 0x1A1B1B19;
	data_array[3] = 0x0102031A;
	data_array[4] = 0x05060700;
	data_array[5] = 0x18222304;
	data_array[6] = 0x18202118;
	data_array[7] = 0x18181818;
	data_array[8] = 0x18181818;
	data_array[9] = 0x18181818;
	data_array[10] = 0x18181818;
	data_array[11] = 0x18181818;
	data_array[12] = 0x00000018;
	dsi_set_cmdq(data_array, 13, 1);
	MDELAY(1);
	//data_array[0] = 0x00033902;
	//data_array[1] = 0x009292B6;//vcom
	//dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x002B3902;//gamma
	data_array[1] = 0x020000E0;
	data_array[2] = 0x0C3F332A;//0x0D3F332D;
	data_array[3] = 0x0B090731;//0x0B090632;
	data_array[4] = 0x13110E17;//0x12100D16;
	data_array[5] = 0x11071311;//0x10061310;
	data_array[6] = 0x00001813;//0x00001612;
	data_array[7] = 0x3F322A03;//0x3F332D02;
	data_array[8] = 0x0905310C;//0x0906320D;
	data_array[9] = 0x100E170B;//0x100D160B;
	data_array[10] = 0x07121112;//0x06131012;
	data_array[11] = 0x00171412;//0x00161210;
	dsi_set_cmdq(data_array, 12, 1);
	MDELAY(1);
	data_array[0] = 0x00033902;
	data_array[1] = 0x001430C0;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00053902;
	data_array[1] = 0x00C000C7;
	data_array[2] = 0x000000C0;
	dsi_set_cmdq(data_array, 3, 1);
	MDELAY(1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000005CC;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00023902;
	data_array[1] = 0x000088DF;
	dsi_set_cmdq(data_array, 2, 1);
	MDELAY(1);
	data_array[0] = 0x00110500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(150);
	data_array[0] = 0x00290500;
	dsi_set_cmdq(data_array, 1, 1);
	MDELAY(20);

}

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

	//  printk("GPIO_LCD_BIAS_ENP_PIN(12):(DIR)=0x%x|(OUT)=0x%x|(MODE)=0x%x\n", __raw_readl(ioremap(0x10005000,4)), __raw_readl(ioremap(0x10005100,4)), __raw_readl(ioremap(0x10005310,4)));

	//SET_LCD_BIAS_EN(1);
	//MDELAY(5);
	//SET_LCD_BIAS_EN(1);
	//MDELAY(5);

	/*Set */
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);	
	MDELAY(50);
	SET_RESET_PIN(1);
	MDELAY(150);
	//printk("LCM_RST(158):(DIR)=0x%x|(OUT)=0x%x|(MODE)=0x%x\n", __raw_readl(ioremap(0x10005040,4)),__raw_readl(ioremap(0x10005140,4)), __raw_readl(ioremap(0x100053F0,4)));

	/*Send Initial commands*/

#ifdef USE_OLD_LCM
	dsi_set_cmdq_V3(lcm_initialization_setting_kd, sizeof(lcm_initialization_setting_kd) / sizeof(lcm_initialization_setting_kd[0]), 1);
#else
	//wq 		dsi_set_cmdq_V3(lcm_initialization_setting_kd2, sizeof(lcm_initialization_setting_kd2) / sizeof(lcm_initialization_setting_kd2[0]), 1);
	init_lcm_registers();
#endif
	//lcm_read_test();//because when we use this function will make the system dead 	


}

static void lcm_suspend(void)
{
	unsigned char cmd_3 = 0x03;
	unsigned char data_3 = 0x00;
	int ret = 0;
	int gpio_ret = 0;

	printk( "[LCM]**%s**\n",__func__);
	// dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
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

LCM_DRIVER hx8394d_hd720_dsi_vdo_tpv_new_lcm_drv= 
{
	.name	        = "hx8394d_hd720_dsi_vdo_tpv_new",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init          	= lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
//	.ata_check 	= lcm_ata_check,
};
