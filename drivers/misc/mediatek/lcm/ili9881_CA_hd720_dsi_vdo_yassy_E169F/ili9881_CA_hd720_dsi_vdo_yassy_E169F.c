//guohongjin 20140813 end
/* BEGIN PN: , Added by guohongjin, 2014.08.13*/
#ifndef BUILD_LK
#include <linux/string.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/mt_gpio.h>
#elif defined(BUILD_UBOOT)
	#include <asm/arch/mt_gpio.h>
#else
	#include <mach/mt_gpio.h>
#endif

#ifdef BUILD_LK
#include <platform/gpio_const.h>
#include <platform/mt_gpio.h>
#include <platform/upmu_common.h>
#else
#include <mach/gpio_const.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <linux/string.h>
#endif


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH  										(720)
#define FRAME_HEIGHT 										(1280)
#define LCM_ID_ILI9881                                                              (0x0)

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;

#define __SAME_IC_COMPATIBLE__

#define SET_RESET_PIN(v)    								(lcm_util.set_reset_pin((v)))
#define MDELAY(n) 											(lcm_util.mdelay(n))

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)										lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)					lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

 struct LCM_setting_table {
    unsigned cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting1[] = {
	//CCMON
	/*
	Note :

	Data ID will depends on the following rule.
	
		count of parameters > 1	=> Data ID = 0x39
		count of parameters = 1	=> Data ID = 0x15
		count of parameters = 0	=> Data ID = 0x05

	Structure Format :

	{DCS command, count of parameters, {parameter list}}
	{REGFLAG_DELAY, milliseconds of time, {}},

	...
 
	Setting ending by predefined flag
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
	*/
      {0xFF ,3,{0x98, 0x81 ,0x03 }},
{0x01 ,1,{0x00}},
{0x02 ,1,{0x00}},
{0x03 ,1,{0x53}},
{0x04 ,1,{0x14}},
{0x05 ,1,{0x00}},
{0x06 ,1,{0x06}},
{0x07 ,1,{0x01}},
{0x08 ,1,{0x00}},
{0x09 ,1,{0x01}},
{0x0A ,1,{0x19}},
{0x0B ,1,{0x01}},
{0x0C ,1,{0x00}},
{0x0D ,1,{0x00}},
{0x0E ,1,{0x00}},
{0x0F ,1,{0x19}},
{0x10 ,1,{0x19}},
{0x11 ,1,{0x00}},
{0x12 ,1,{0x00}},
{0x13 ,1,{0x00}},
{0x14 ,1,{0x00}},
{0x15 ,1,{0x00}},
{0x16 ,1,{0x00}},
{0x17 ,1,{0x00}},
{0x18 ,1,{0x00}},
{0x19 ,1,{0x00}},
{0x1A ,1,{0x00}},
{0x1B ,1,{0x00}},
{0x1C ,1,{0x00}},
{0x1D ,1,{0x00}},
{0x1E ,1,{0x40}},
{0x1F ,1,{0x40}},
{0x20 ,1,{0x02}},
{0x21 ,1,{0x05}},
{0x22 ,1,{0x02}},
{0x23 ,1,{0x00}},
{0x24 ,1,{0x87}},
{0x25 ,1,{0x87}},
{0x26 ,1,{0x00}},
{0x27 ,1,{0x00}},
{0x28 ,1,{0x3b}},
{0x29 ,1,{0x03}},
{0x2a ,1,{0x00}},
{0x2b ,1,{0x00}},
{0x2c ,1,{0x00}},
{0x2d ,1,{0x00}},
{0x2e ,1,{0x00}},
{0x2f ,1,{0x00}},
{0x30 ,1,{0x00}},
{0x31 ,1,{0x00}},
{0x32 ,1,{0x00}},
{0x33 ,1,{0x00}},
{0x34 ,1,{0x04}},
{0x35 ,1,{0x00}},
{0x36 ,1,{0x00}},//01 00
{0x37 ,1,{0x00}},
{0x38 ,1,{0x01}},
{0x39 ,1,{0x01}},
{0x3A ,1,{0x40}},
{0x3B ,1,{0x40}},
{0x3C ,1,{0x00}},
{0x3D ,1,{0x00}},
{0x3e ,1,{0x00}}, 
{0x3f ,1,{0x00}},
{0x40 ,1,{0x00}},
{0x41 ,1,{0x88}},
{0x42 ,1,{0x00}},
{0x43 ,1,{0x00}},
{0x44 ,1,{0x00}},
{0x50 ,1,{0x01}}, 
{0x51 ,1,{0x23}}, 
{0x52 ,1,{0x45}}, 
{0x53 ,1,{0x67}}, 
{0x54 ,1,{0x89}}, 
{0x55 ,1,{0xab}}, 
{0x56 ,1,{0x01}}, 
{0x57 ,1,{0x23}}, 
{0x58 ,1,{0x45}}, 
{0x59 ,1,{0x67}}, 
{0x5A ,1,{0x89}}, 
{0x5B ,1,{0xab}}, 
{0x5C ,1,{0xcd}}, 
{0x5D ,1,{0xef}}, 
{0x5E ,1,{0x11}}, 
{0x5F ,1,{0x06}}, 
{0x60 ,1,{0x0c}}, 
{0x61 ,1,{0x0d}}, 
{0x62 ,1,{0x0e}}, 
{0x63 ,1,{0x0f}}, 
{0x64 ,1,{0x02}}, 
{0x65 ,1,{0x02}}, 
{0x66 ,1,{0x02}}, 
{0x67 ,1,{0x02}}, 
{0x68 ,1,{0x02}}, 
{0x69 ,1,{0x02}}, 
{0x6A ,1,{0x02}}, 
{0x6B ,1,{0x02}}, 
{0x6C ,1,{0x02}}, 
{0x6D ,1,{0x02}}, 
{0x6E ,1,{0x05}}, 
{0x6F ,1,{0x05}}, 
{0x70 ,1,{0x05}}, 
{0x71 ,1,{0x02}}, 
{0x72 ,1,{0x01}}, 
{0x73 ,1,{0x00}}, 
{0x74 ,1,{0x08}}, 
{0x75 ,1,{0x08}}, 
{0x76 ,1,{0x0c}}, 
{0x77 ,1,{0x0d}}, 
{0x78 ,1,{0x0e}}, 
{0x79 ,1,{0x0f}}, 
{0x7A ,1,{0x02}}, 
{0x7B ,1,{0x02}}, 
{0x7C ,1,{0x02}},
{0x7D ,1,{0x02}},
{0x7E ,1,{0x02}},
{0x7F ,1,{0x02}},
{0x80 ,1,{0x02}},
{0x81 ,1,{0x02}},
{0x82 ,1,{0x02}},
{0x83 ,1,{0x02}},
{0x84 ,1,{0x05}},
{0x85 ,1,{0x05}},
{0x86 ,1,{0x05}},
{0x87 ,1,{0x02}},
{0x88 ,1,{0x01}},
{0x89 ,1,{0x00}},
{0x8A ,1,{0x06}},
{0xFF ,3,{0x98, 0x81, 0x04}}, 
{0x6C ,1,{0x15}}, 
{0x6e ,1,{0x2b}}, 
{0x6f ,1,{0x33}}, 
{0x3a ,1,{0xa4}}, 
{0x8d ,1,{0x15}}, 
{0x87 ,1,{0xba}},
{0x26 ,1,{0x76}},
{0xb2 ,1,{0xd1}},

{0x21,1,{0xB8}},                       
{0x73,1,{0x14}},                       
{0xA6,1,{0x40}},                       
{0xA9,1,{0x80}},       
{0x87,1,{0x2A}}, 
{0xFF ,3,{0x98, 0x81, 0x01}}, 
{0x22 ,1,{0x0a}}, 
{0x31 ,1,{0x00}}, 
{0x53 ,1,{0x8a}}, 
{0x55 ,1,{0x76}}, 
{0x50,1,{0xae}},//C0
{0x51,1,{0xaf}},//C0
{0x60,1,{0x14}},//08 fae
{0xA0 ,1,{0x12}}, //1D tuwenzan modify at 20151020
{0xA1 ,1,{0x22}}, 
{0xA2 ,1,{0x2B}}, 
{0xA3 ,1,{0x0E}}, 
{0xA4 ,1,{0x10}}, 
{0xA5 ,1,{0x23}}, 
{0xA6 ,1,{0x17}}, 
{0xA7 ,1,{0x15}}, 
{0xA8 ,1,{0x97}}, 
{0xA9 ,1,{0x15}}, 
{0xAA ,1,{0x26}}, 
{0xAB ,1,{0x85}}, 
{0xAC ,1,{0x18}}, 
{0xAD ,1,{0x14}}, 
{0xAE ,1,{0x47}}, 
{0xAF ,1,{0x22}},
{0xB0 ,1,{0x25}}, 
{0xB1 ,1,{0x59}}, 
{0xB2 ,1,{0x6B}}, 
{0xB3 ,1,{0x39}}, 
{0xC0 ,1,{0x12}}, //1D tuwenzan modify at 20151020
{0xC1 ,1,{0x2D}}, 
{0xC2 ,1,{0x38}}, 
{0xC3 ,1,{0x09}}, 
{0xC4 ,1,{0x0B}}, 
{0xC5 ,1,{0x1E}}, 
{0xC6 ,1,{0x16}}, 
{0xC7 ,1,{0x1F}}, 
{0xC8 ,1,{0x8F}}, 
{0xC9 ,1,{0x20}}, 
{0xCA ,1,{0x2a}}, 
{0xCB ,1,{0x93}}, 
{0xCC ,1,{0x1F}}, 
{0xCD ,1,{0x20}}, 
{0xCE ,1,{0x55}}, 
{0xCF ,1,{0x22}}, 
{0xD0 ,1,{0x2C}}, 
{0xD1 ,1,{0x5A}}, 
{0xD2 ,1,{0x67}}, 
{0xD3 ,1,{0x39}}, 
{0xFF ,3,{0x98, 0x81, 0x00}},	
//{0x36,1,{0x02}},	
//{0x35,1,{0x00}},	
{0x11,1,{0x00}},  // Sleep-Out
{REGFLAG_DELAY, 120, {0}},	
{0x29,1,{0x00}},  // Display On
//{0xFF ,3,{0x98, 0x81, 0x05}},	 //tuwenzan add at 20151118
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_sleep_out_setting[] = {
    //Sleep Out
    {0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
    {0x29, 1, {0x00}},
    {REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_deep_sleep_mode_in_setting[] = {
    // Display off sequence
    {0x28, 1, {0x00}},
    {REGFLAG_DELAY, 40, {}},

    // Sleep Mode On
    {0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},
	//zhangaifeng begin
   // {0xFF,	3,		{0x98,0x81,0x01}},
   // {0x58, 1, {0x01}},
//{REGFLAG_DELAY, 20, {}},
		//zhangaifeng end
    {REGFLAG_END_OF_TABLE, 0x00, {}}
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
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

		memset(params, 0, sizeof(LCM_PARAMS));
		params->type   = LCM_TYPE_DSI;

		params->width  = FRAME_WIDTH;
		params->height = FRAME_HEIGHT;

		params->dbi.te_mode				= LCM_DBI_TE_MODE_VSYNC_ONLY;
		//LCM_DBI_TE_MODE_DISABLED;
		//LCM_DBI_TE_MODE_VSYNC_ONLY;  
		params->dbi.te_edge_polarity		= LCM_POLARITY_RISING; 
		/////////////////////   
		//if(params->dsi.lcm_int_te_monitor)  
		//params->dsi.vertical_frontporch *=2;  
		//params->dsi.lcm_ext_te_monitor= 0;//TRUE; 
	//	params->dsi.noncont_clock= TRUE;//FALSE;   
	//	params->dsi.noncont_clock_period=2;
		params->dsi.cont_clock=1;    //modify  yudengwu
		////////////////////          
		params->dsi.mode   = SYNC_PULSE_VDO_MODE;  
		// DSI    /* Command mode setting */  
		params->dsi.LANE_NUM				= LCM_FOUR_LANE;      
		//The following defined the fomat for data coming from LCD engine.  
		params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;   
		params->dsi.data_format.trans_seq	= LCM_DSI_TRANS_SEQ_MSB_FIRST; 
		params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB;    
		params->dsi.data_format.format	  = LCM_DSI_FORMAT_RGB888;       
		// Video mode setting		   
		params->dsi.intermediat_buffer_num = 2;  
		params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;  
		params->dsi.packet_size=256;    
		// params->dsi.word_count=480*3;	
		//DSI CMD mode need set these two bellow params, different to 6577   
		// params->dsi.vertical_active_line=800;   
		params->dsi.vertical_sync_active				= 8; //4   
		params->dsi.vertical_backporch				       = 20;  //14  
		params->dsi.vertical_frontporch				       = 10;  //16  
		params->dsi.vertical_active_line				       = FRAME_HEIGHT;     
		params->dsi.horizontal_sync_active				= 50;   //40 tuwenzan modify at 20151020
		params->dsi.horizontal_backporch				= 90;  //60  
		params->dsi.horizontal_frontporch				= 70;    //60
		params->dsi.horizontal_blanking_pixel				= 60;   
		params->dsi.horizontal_active_pixel				= FRAME_WIDTH;  
		
	//	params->dsi.pll_div1=1;		   
	//	params->dsi.pll_div2=1;		   
	//	params->dsi.fbk_div =28;//28	
//liqiang 20140820 beign
// To fix boot error
//tuwenzan modify at 20151012 begin
#ifdef CONFIG_WIND_PLL_CLOCK
		params->dsi.PLL_CLOCK = 215;	
#else
		params->dsi.PLL_CLOCK = 212;	   // 245;
#endif
//tuwenzan modify at 20151012 end

	//	params->dsi.ss

		params->dsi.HS_TRAIL = 10;
			
//liqiang 20140820 end
		params->dsi.esd_check_enable = 1;
                params->dsi.customization_esd_check_enable = 1;
//tuwenzan modify at 20151118 begin
    params->dsi.lcm_esd_check_table[0].cmd = 0x09;
                params->dsi.lcm_esd_check_table[0].count                = 3;
    params->dsi.lcm_esd_check_table[0].para_list[0] = 0x80;
    params->dsi.lcm_esd_check_table[0].para_list[1] = 0x03;
    params->dsi.lcm_esd_check_table[0].para_list[2] = 0x04;
//tuwenzan modify at 20151118 end
}
	//zhangaifeng begin
//#ifndef BUILD_LK

#if 0
#include <linux/kernel.h>
#include <linux/module.h>  
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
//#include <linux/jiffies.h>
#include <linux/uaccess.h>
//#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/***************************************************************************** 
 * Define
 *****************************************************************************/
//#ifndef FPGA_EARLY_PORTING
#define TPS_I2C_BUSNUM  0//for I2C channel 0
#define I2C_ID_NAME "ili9881"
#define TPS_ADDR 0x3E
/***************************************************************************** 
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info __initdata ili9881_board_info = {I2C_BOARD_INFO(I2C_ID_NAME, TPS_ADDR)};
static struct i2c_client *ili9881_i2c_client = NULL;


/***************************************************************************** 
 * Function Prototype
 *****************************************************************************/ 
static int ili9881_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int ili9881_remove(struct i2c_client *client);
/***************************************************************************** 
 * Data Structure
 *****************************************************************************/

 struct ili9881_dev	{	
	struct i2c_client	*client;
	
};

static const struct i2c_device_id ili9881_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

//#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif
static struct i2c_driver ili9881_iic_driver = {
	.id_table	= ili9881_id,
	.probe		= ili9881_probe,
	.remove		= ili9881_remove,
	//.detect		= mt6605_detect,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "ili9881",
	},
 
};
/***************************************************************************** 
 * Extern Area
 *****************************************************************************/ 
 
 

/***************************************************************************** 
 * Function
 *****************************************************************************/ 
static int ili9881_probe(struct i2c_client *client, const struct i2c_device_id *id)
{  
	printk( "ili9881_iic_probe\n");
	printk("TPS: info==>name=%s addr=0x%x\n",client->name,client->addr);
	ili9881_i2c_client  = client;		
	return 0;      
}


static int ili9881_remove(struct i2c_client *client)
{  	
  printk( "ili9881_remove\n");
  ili9881_i2c_client = NULL;
   i2c_unregister_device(client);
  return 0;
}


 static int ili9881_write_bytes(unsigned char addr, unsigned char value)
{	
	int ret = 0;
	struct i2c_client *client = ili9881_i2c_client;
	char write_data[2]={0};	
	write_data[0]= addr;
	write_data[1] = value;
    ret=i2c_master_send(client, write_data, 2);
	if(ret<0)
	printk("ili9881 write data fail !!\n");	
	return ret ;
}



/*
 * module load/unload record keeping
 */

static int __init ili9881_iic_init(void)
{

   printk( "ili9881_iic_init\n");
   i2c_register_board_info(TPS_I2C_BUSNUM, &ili9881_board_info, 1);
   printk( "ili9881_iic_init2\n");
   i2c_add_driver(&ili9881_iic_driver);
   printk( "ili9881_iic_init success\n");	
   return 0;
}

static void __exit ili9881_iic_exit(void)
{
  printk( "ili9881_iic_exit\n");
  i2c_del_driver(&ili9881_iic_driver);  
}


module_init(ili9881_iic_init);
module_exit(ili9881_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK ili9881 I2C Driver");
MODULE_LICENSE("GPL"); 
//#endif
#endif

#if 0

#define ili9881_I2C_ID	I2C0
static struct mt_i2c_t ili9881_i2c;
#define ili9881_SLAVE_ADDR_WRITE   0x7c
#define ili9881_SLAVE_ADDR_READ    0x7d
#include <platform/mt_i2c.h>
/*
kal_uint32 ili9881_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    ili9881_i2c.id = ili9881_I2C_ID;
    // Since i2c will left shift 1 bit, we need to set ili9881 I2C address to >>1 
    ili9881_i2c.addr = (ili9881_SLAVE_ADDR_WRITE >> 1);
    ili9881_i2c.mode = ST_MODE;
    ili9881_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&ili9881_i2c, write_data, len);

    if(I2C_OK != ret_code)
        dprintf(INFO, "%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}
*/
kal_uint32 ili9881_read_byte (kal_uint8 addr, kal_uint8 *dataBuffer) 
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint16 len;
    *dataBuffer = addr;

    ili9881_i2c.id = ili9881_I2C_ID;
    /* Since i2c will left shift 1 bit, we need to set ili9881 I2C address to >>1 */
    ili9881_i2c.addr = (ili9881_SLAVE_ADDR_READ >> 1);
    ili9881_i2c.mode = ST_MODE;
    ili9881_i2c.speed = 100;
    len = 1;

    ret_code = i2c_write_read(&ili9881_i2c, dataBuffer, len, len);

    if(I2C_OK != ret_code)
        dprintf(INFO, "%s: i2c_read: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}
#endif

#if 0
//#ifndef FPGA_EARLY_PORTING
#define ili9881_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t ili9881_i2c;

static int ili9881_write_byte(kal_uint8 addr, kal_uint8 value)
{
    kal_uint32 ret_code = I2C_OK;
    kal_uint8 write_data[2];
    kal_uint16 len;

    write_data[0]= addr;
    write_data[1] = value;

    ili9881_i2c.id = 0;//I2C2;
    /* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    ili9881_i2c.addr = (ili9881_SLAVE_ADDR_WRITE >> 1);
    ili9881_i2c.mode = ST_MODE;
    ili9881_i2c.speed = 100;
    len = 2;

    ret_code = i2c_write(&ili9881_i2c, write_data, len);
    //printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code);

    return ret_code;
}

#else
  
//	extern int mt8193_i2c_write(u16 addr, u32 data);
//	extern int mt8193_i2c_read(u16 addr, u32 *data);
	
//	#define ili9881_write_byte(add, data)  mt8193_i2c_write(add, data)
	//#define ili9881_read_byte(add)  mt8193_i2c_read(add)
  
//#endif
#endif

	//zhangaifeng end
static void lcm_init(void)
{
	/*
             MDELAY(50);
             SET_RESET_PIN(1);
		MDELAY(10);
		SET_RESET_PIN(0);
		MDELAY(20); 
		SET_RESET_PIN(1);
		MDELAY(120); 
*/
			//AVDD power on
			//zhangaifeng begin
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret=0;
	cmd=0x03;
	data=0x0;
			//zhangaifeng end
	//tuwenzan modify at 20150921 begin
//	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
//	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
	//		MDELAY(10);
//tuwenzan modify at 20150921 end

		//zhangaifeng begin
	#ifdef BUILD_LK
		  //  ili9881_write_byte(0x03,0x01);
#endif
/*
#ifdef BUILD_LK
	ret=ili9881_write_byte(cmd,data);
    if(ret)    	
    dprintf(0, "[LK]ili9881----tps6132----cmd=%0x--i2c write error----\n",cmd);    	
	else
	dprintf(0, "[LK]ili9881----tps6132----cmd=%0x--i2c write success----\n",cmd);    		
#else
	ret=ili9881_write_bytes(cmd,data);
	if(ret<0)
	printk("[KERNEL]ili9881----tps6132---cmd=%0x-- i2c write error-----\n",cmd);
	else
	printk("[KERNEL]ili9881----tps6132---cmd=%0x-- i2c write success-----\n",cmd);
#endif
	//zhangaifeng end
*/
	SET_RESET_PIN(1);
	MDELAY(1);
	SET_RESET_PIN(0);
	MDELAY(10);
	SET_RESET_PIN(1);
	MDELAY(120);
	
	      push_table(lcm_initialization_setting1, sizeof(lcm_initialization_setting1) / sizeof(struct LCM_setting_table), 1); 

}

static void lcm_suspend(void) 
{
     
	// when phone sleep , config output low, disable backlight drv chip  
	push_table(lcm_deep_sleep_mode_in_setting, sizeof(lcm_deep_sleep_mode_in_setting) / sizeof(struct LCM_setting_table), 1);
	
	//tuwenzan modify at 20150921 begin
//	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
//	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ZERO);
	//tuwenzan modify at 20150921 end
	MDELAY(10);

}


static void lcm_resume(void)
{
	lcm_init();
}

static unsigned int lcm_compare_id(void)
{
	unsigned int id=0,id1=0,id2=0;
	unsigned char buffer[3];
	unsigned int array[16];  
	//tuwenzan modify at 20150921 begin
//	mt_set_gpio_mode(GPIO_LCM_PWR_EN, GPIO_MODE_00);
//	mt_set_gpio_dir(GPIO_LCM_PWR_EN, GPIO_DIR_OUT);
//	mt_set_gpio_out(GPIO_LCM_PWR_EN, GPIO_OUT_ONE);
//	MDELAY(10);
//tuwenzan modify at 20150921 begin
	SET_RESET_PIN(1);
	SET_RESET_PIN(0);
	MDELAY(1);
	
	SET_RESET_PIN(1);
	MDELAY(120); 

array[0]=0x00043902;
    array[1]=0x078198FF;
   // array[2]=0x00000007;
    dsi_set_cmdq(array, 2, 1);
    MDELAY(10);  
    array[0] = 0x00013700;// read id return two byte,version and id
    dsi_set_cmdq(array, 1, 1);
    MDELAY(10); 
       
    read_reg_v2(0x03, buffer, 1);
    id1 = buffer[0]; 
    
    read_reg_v2(0x04, buffer, 1);
    id2 = buffer[1];  
    
    id2 = buffer[1]; //we test buffer 1
	id = (id1 << 8) | id2;

    #ifdef BUILD_LK
		printf("%s, LK ili9881 debug: ili9881 id = %x,%x,%x\n", __func__, id,id1,id2);
    #else
		printk("%s, kernel ili9881 horse debug: ili9881 id = 0x%08x\n", __func__, id);
    #endif
//return 1;
    if(id1 == LCM_ID_ILI9881)
    	return 1;
    else
        return 0;
}
static int err_count = 0;
static unsigned int lcm_esd_check(void)
{
  #ifndef BUILD_LK
    unsigned char buffer[8] = {0};

    unsigned int array[4];
    int i =0;

	if(lcm_esd_test)
	{
		lcm_esd_test = FALSE;
		return TRUE;
	}

    array[0] = 0x00013700;    

    dsi_set_cmdq(array, 1,1);

    read_reg_v2(0x0A, buffer,8);

	printk( "ili9881 lcm_esd_check: buffer[0] = %d,buffer[1] = %d,buffer[2] = %d,buffer[3] = %d,buffer[4] = %d,buffer[5] = %d,buffer[6] = %d,buffer[7] = %d\n",buffer[0],buffer[1],buffer[2],buffer[3],buffer[4],buffer[5],buffer[6],buffer[7]);

    if((buffer[0] != 0x9C))/*LCD work status error,need re-initalize*/

    {

        printk( "ili9881 lcm_esd_check buffer[0] = %d\n",buffer[0]);

        return TRUE;

    }

    else

    {

#if 0
        if(buffer[3] != 0x02) //error data type is 0x02

        {
		//  is not 02, 
             err_count = 0;

        }

        else

        {
		// is 02, so ,
             if((buffer[4] == 0x40) || (buffer[5] == 0x80))
             {
			 // buffer[4] is not 0, || (huo),buffer[5] is 0x80.
			   err_count = 0;
             }
             else

             {
			// is  0,0x80,  
			   err_count++;
             }             

             if(err_count >=2 )
             {
			
                 err_count = 0;

                 printk( "ili9881 lcm_esd_check buffer[4] = %d , buffer[5] = %d\n",buffer[4],buffer[5]);

                 return TRUE;

             }

        }
#endif
        return FALSE;

    }
#endif
	
}
static unsigned int lcm_esd_recover(void)
{
    #ifndef BUILD_LK
    printk( "ili9881 lcm_esd_recover\n");
    #endif
	lcm_init();
//	lcm_resume();

	return TRUE;
}
//tuwenzan modify at 20150921 begin
#ifdef CONFIG_WIND_LCD_POWER_SUPPLY_SUPPORT
extern void lcm_init_power(void);
extern void lcm_resume_power(void);
extern void lcm_suspend_power(void);
#endif
LCM_DRIVER ili9881_CA_hd720_dsi_vdo_yassy_lcm_drv =
{
    .name           	= "ili9881_CA_hd720_dsi_vdo_yassy",
    .set_util_funcs 	= lcm_set_util_funcs,
    .get_params     	= lcm_get_params,
    .init           	= lcm_init,
    .suspend        	= lcm_suspend,
    .resume         	= lcm_resume,
    .compare_id     	= lcm_compare_id,
	//.esd_check = lcm_esd_check,
	//.esd_recover = lcm_esd_recover,
#ifdef CONFIG_WIND_LCD_POWER_SUPPLY_SUPPORT
	.init_power		= lcm_init_power,
	.resume_power   = lcm_resume_power,
	.suspend_power  = lcm_suspend_power,
#endif    
};
//tuwenzan modify at 20150921 end
//late_initcall(lcm_init);
/* END PN: , Added by guohongjin, 2014.08.13*/
//guohongjin 20140813 end
