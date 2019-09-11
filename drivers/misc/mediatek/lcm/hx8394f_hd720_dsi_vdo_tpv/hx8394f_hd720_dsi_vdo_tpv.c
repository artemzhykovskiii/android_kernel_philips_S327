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

/** Local Constants **/
#define LCM_DSI_CMD_MODE								0
#define FRAME_WIDTH  									(720)
#define FRAME_HEIGHT 									(1280)

#define REGFLAG_DELAY             							0xFE
#define REGFLAG_END_OF_TABLE      							0xFF   // END OF REGISTERS MARKER
#define I2C_ID_NAME "tps65132"

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};
static struct i2c_client *tps65132_i2c_client = NULL;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define SET_LCD_BIAS_EN(v)		(lcm_util.set_gpio_lcd_enp_bias(v))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))


// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)						lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)			lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg						lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)        

struct LCM_setting_table {
	unsigned cmd;
	unsigned char count;
	unsigned char para_list[64];
};
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
#if  0 //wq add 
static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2]={0};	

	if (client == NULL) {
		pr_warn("i2c_client = NULL, skip tps65132_write_bytes\n");
		return 0;
	}
	write_data[0]= addr;
	write_data[1] = value;
	ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
		printk("tps65132 write data fail !!\n");	
	return ret ;
}
#endif  
/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

	printk( "[LCM]**%s**\n",__func__);
	i2c_add_driver(&tps65132_iic_driver);
	printk( "[LCM]**%s**success!!!\n",__func__);
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
static struct LCM_setting_table lcm_initialization_setting[] = {
	//Set extension command
	{ 0xB9, 3,  { 0xff, 0x83, 0x94 }},
	//Set power
	{ 0xB1, 10, { 0x50, 0x15, 0x75, 0x09, 0x32, 
			    0x44, 0x71, 0x31, 0x55, 0x2f  }},
	//Set MIPI control	                          
	{ 0xBA, 6,  { 0x63, 0x03, 0x68, 0x6b, 0xb2,0xc0 }},
	//SETOFFSET
	{ 0xD2, 1,  { 0x88 }},
	//Set display related register
	{ 0xB2, 5,  { 0x00, 0x80, 0x64, 0x10, 0x07 }},	               
	//Set display waveform cycles
	{ 0xB4, 21, { 0x01, 0x65, 0x01, 0x65, 0x01,
			    0x65, 0x01, 0x05, 0x7e, 0x25,              
			    0x00, 0x3f, 0x01, 0x65, 0x01,            	           
			    0x65, 0x01, 0x65, 0x01, 0x05,            	              
			    0x7e }},              
	//SETGIP0: Set GIP Option0
	{ 0xD3, 33, { 0x00, 0x00, 0x07, 0x07, 0x40,
			    0x1E, 0x08, 0x00, 0x32, 0x10,	              
			    0x08, 0x00, 0x08, 0x54, 0x15,            	              
			    0x10, 0x05, 0x04, 0x02, 0x12,         	              
			    0x10, 0x05, 0x07, 0x23, 0x23,	              
			    0x0C, 0x0C, 0x27, 0x10, 0x07,              
			    0x07, 0x10, 0x40}},
	//Set GIP Option1
	{ 0xD5, 44, { 0x19, 0x19, 0x18, 0x18, 0x1b,            
			    0x1b, 0x1a, 0x1a, 0x04, 0x05,            	              
			    0x06, 0x07, 0x00, 0x01, 0x02,
			    0x03, 0x20, 0x21, 0x18, 0x18,        	              
			    0x22, 0x23, 0x18, 0x18, 0x18,
			    0x18, 0x18, 0x18, 0x18, 0x18, 	              
			    0x18, 0x18, 0x18, 0x18, 0x18,             
			    0x18, 0x18, 0x18, 0x18, 0x18,             	              
			    0x18, 0x18, 0x18, 0x18 }},
	//Set GIP Option2			
	{ 0xD6, 44, { 0x18, 0x18, 0x19, 0x19, 0x1b,
			    0x1b, 0x1a, 0x1a, 0x03, 0x02,            	              
			    0x01, 0x00, 0x07, 0x06, 0x05,            
			    0x04, 0x23, 0x22, 0x18, 0x18,          	              
			    0x21, 0x20, 0x18, 0x18, 0x18,
			    0x18, 0x18, 0x18, 0x18, 0x18,            	              
			    0x18, 0x18, 0x18, 0x18, 0x18,
			    0x18, 0x18, 0x18, 0x18, 0x18,           	              
			    0x18, 0x18, 0x18, 0x18 }},    
	//SETGAMMA: Set gamma curve related setting          	            
	{ 0xE0, 58, { 0x00, 0x02, 0x09, 0x0f, 0x11,
			    0x14, 0x17, 0x14, 0x2b, 0x3c,            	              
			    0x4c, 0x4b, 0x57, 0x6a, 0x71,                                   
			    0x77, 0x86, 0x8c, 0x89, 0x97,                                   
			    0xa7, 0x54, 0x52, 0x55, 0x5a,                                   
			    0x5c, 0x63, 0x7d, 0x7F, 0x00,
			    0x02, 0x09, 0x0f, 0x10, 0x13,
			    0x16, 0x14, 0x2b, 0x3c, 0x4c,
			    0x4b, 0x56, 0x6a, 0x72, 0x78,
			    0x85, 0x8b, 0x87, 0x97, 0xa8,
			    0x52, 0x51, 0x56, 0x5a, 0x5c,
			    0x64, 0x7d, 0x7F}},
	//SETPANEL
	{ 0xCC, 1, { 0x0b }},
	//SETSTBA: Set Source Option
	{ 0xC0, 2, { 0x1f, 0x31 }},
	//SETVCOM: Set VCOM voltage
	{ 0xB6, 2, { 0xa5, 0xa5 }},
	/*Add for ESD Test*/
	{ 0xBD, 1, { 0x01}},
	//	{ 0xB1, 1, { 0x01}},
	{ 0xB1, 1, { 0x00}},
	{ 0xBD, 1, { 0x00}},
	//SETIOOPT
	{ 0xD4, 1, { 0x02 }}, 
	//	{REGFLAG_DELAY, 120, {}},
	// Sleep Out
	{ 0x11, 0, {0x00}},	
	{REGFLAG_DELAY, 120, {}},

	{ 0xB2, 12, { 0x00, 0x80, 0x64, 0x10, 0x07, 
			    0x2F, 0x00, 0x00, 0x00, 0x00,  
			    0xC0, 0x18}},

	// Display On
	{ 0x29, 0, {0x00}},
	//	{REGFLAG_DELAY, 100, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#if 0
static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {

	// Display OFF
	{0x28,0,{}},
	{REGFLAG_DELAY, 90,{}},

	// Sleep IN
	{0x10, 0, {}},
	{REGFLAG_DELAY, 10, {}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#endif


static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for(i = 0; i < count; i++) {

		unsigned cmd;
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY :
				MDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE :
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}

}


// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

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
	params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
	params->dbi.te_edge_polarity	= LCM_POLARITY_RISING;
	params->dsi.mode   				= BURST_VDO_MODE;

	// DSI
	/* Command mode setting */
	params->dsi.LANE_NUM			 = LCM_FOUR_LANE;
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq    = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding       = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format         = LCM_DSI_FORMAT_RGB888;

	// Highly depends on LCD driver capability.
	// Not support in MT6573
	params->dsi.packet_size=256;
	params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active			 = 0x02;
	params->dsi.vertical_backporch 				 = 0x10;
	params->dsi.vertical_frontporch				 = 0x0A;
	params->dsi.vertical_active_line			 = FRAME_HEIGHT;
	params->dsi.horizontal_sync_active 			 = 0x02;
	params->dsi.horizontal_backporch		     = 0x5C;
	params->dsi.horizontal_frontporch		     = 0x50;
	params->dsi.horizontal_active_pixel			 = FRAME_WIDTH;


#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 350; //this value must be in MTK suggested table
#else
	params->dsi.PLL_CLOCK = 230; //this value must be in MTK suggested table: 920MHz MIPI_CLK /2 =460
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif

	params->dsi.cont_clock=0;
	params->dsi.clk_lp_per_line_enable = 1;

	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd          = 0x09;
	params->dsi.lcm_esd_check_table[0].count        = 4;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
	params->dsi.lcm_esd_check_table[0].para_list[1] = 0x73;
	params->dsi.lcm_esd_check_table[0].para_list[2] = 0x04;
	params->dsi.lcm_esd_check_table[0].para_list[3] = 0x00;
}


static void lcm_init(void)
{
	printk("[LCM]**%s**start\n",__func__);
	/*Enable LCM Gate IC */
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#else
	SET_LCD_BIAS_EN(1);
#endif
	MDELAY(5);

	//  printk("GPIO_LCD_BIAS_ENP_PIN(12):(DIR)=0x%x|(OUT)=0x%x|(MODE)=0x%x\n", __raw_readl(ioremap(0x10005000,4)),__raw_readl(ioremap(0x10005100,4)), __raw_readl(ioremap(0x10005310,4)));


	/*Reset Pin*/		
	/*Set */
	SET_RESET_PIN(0);	
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(50);
	// printk("LCM_RST(158):(DIR)=0x%x|(OUT)=0x%x|(MODE)=0x%x\n", __raw_readl(ioremap(0x10005040,4)),__raw_readl(ioremap(0x10005140,4)), __raw_readl(ioremap(0x100053F0,4)));
	/*Send Initial commands*/
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);

	//	MDELAY(10); 

}


static void lcm_suspend(void)
{

	printk( "[LCM]**%s**\n",__func__);
	// dsi_set_cmdq_V3(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(lcm_deep_sleep_mode_in_setting[0]), 1);
	SET_RESET_PIN(0);
#ifdef CONFIG_MTK_LEGACY
	mt_set_gpio_mode(GPIO_LCD_BIAS_ENP_PIN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_LCD_BIAS_ENP_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_LCD_BIAS_ENP_PIN, GPIO_OUT_ONE);
#else
	SET_LCD_BIAS_EN(1);
#endif
	
	//SET_LCD_BIAS_EN(0);
	MDELAY(30);//for ESD test, can't eliminate
}


static void lcm_resume(void)
{
	lcm_init();
}


LCM_DRIVER hx8394f_hd720_dsi_vdo_tpv_lcm_drv= 
{
	.name	    = "hx8394f_hd720_dsi_vdo_tpv",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init          	 = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,

};
