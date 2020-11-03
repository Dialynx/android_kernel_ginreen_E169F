//liqiang 20150228 begin
/* Copyright Statement:
 *
 * This software/firmware and related documentation ("MediaTek Software") are
 * protected under relevant copyright laws. The information contained herein
 * is confidential and proprietary to MediaTek Inc. and/or its licensors.
 * Without the prior written permission of MediaTek inc. and/or its licensors,
 * any reproduction, modification, use or disclosure of MediaTek Software,
 * and information contained herein, in whole or in part, shall be strictly prohibited.
 */
/* MediaTek Inc. (C) 2010. All rights reserved.
 *
 * BY OPENING THIS FILE, RECEIVER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
 * THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
 * RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO RECEIVER ON
 * AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
 * NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
 * SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
 * SUPPLIED WITH THE MEDIATEK SOFTWARE, AND RECEIVER AGREES TO LOOK ONLY TO SUCH
 * THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. RECEIVER EXPRESSLY ACKNOWLEDGES
 * THAT IT IS RECEIVER'S SOLE RESPONSIBILITY TO OBTAIN FROM ANY THIRD PARTY ALL PROPER LICENSES
 * CONTAINED IN MEDIATEK SOFTWARE. MEDIATEK SHALL ALSO NOT BE RESPONSIBLE FOR ANY MEDIATEK
 * SOFTWARE RELEASES MADE TO RECEIVER'S SPECIFICATION OR TO CONFORM TO A PARTICULAR
 * STANDARD OR OPEN FORUM. RECEIVER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND
 * CUMULATIVE LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
 * AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
 * OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY RECEIVER TO
 * MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
 *
 * The following software/firmware and/or related documentation ("MediaTek Software")
 * have been modified by MediaTek Inc. All revisions are subject to any receiver's
 * applicable license agreements with MediaTek Inc.
 */

/*****************************************************************************
*  Copyright Statement:
*  --------------------
*  This software is protected by Copyright and the information contained
*  herein is confidential. The software may not be copied and the information
*  contained herein may not be used or disclosed except with the written
*  permission of MediaTek Inc. (C) 2008
*
*  BY OPENING THIS FILE, BUYER HEREBY UNEQUIVOCALLY ACKNOWLEDGES AND AGREES
*  THAT THE SOFTWARE/FIRMWARE AND ITS DOCUMENTATIONS ("MEDIATEK SOFTWARE")
*  RECEIVED FROM MEDIATEK AND/OR ITS REPRESENTATIVES ARE PROVIDED TO BUYER ON
*  AN "AS-IS" BASIS ONLY. MEDIATEK EXPRESSLY DISCLAIMS ANY AND ALL WARRANTIES,
*  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF
*  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE OR NONINFRINGEMENT.
*  NEITHER DOES MEDIATEK PROVIDE ANY WARRANTY WHATSOEVER WITH RESPECT TO THE
*  SOFTWARE OF ANY THIRD PARTY WHICH MAY BE USED BY, INCORPORATED IN, OR
*  SUPPLIED WITH THE MEDIATEK SOFTWARE, AND BUYER AGREES TO LOOK ONLY TO SUCH
*  THIRD PARTY FOR ANY WARRANTY CLAIM RELATING THERETO. MEDIATEK SHALL ALSO
*  NOT BE RESPONSIBLE FOR ANY MEDIATEK SOFTWARE RELEASES MADE TO BUYER'S
*  SPECIFICATION OR TO CONFORM TO A PARTICULAR STANDARD OR OPEN FORUM.
*
*  BUYER'S SOLE AND EXCLUSIVE REMEDY AND MEDIATEK'S ENTIRE AND CUMULATIVE
*  LIABILITY WITH RESPECT TO THE MEDIATEK SOFTWARE RELEASED HEREUNDER WILL BE,
*  AT MEDIATEK'S OPTION, TO REVISE OR REPLACE THE MEDIATEK SOFTWARE AT ISSUE,
*  OR REFUND ANY SOFTWARE LICENSE FEES OR SERVICE CHARGE PAID BY BUYER TO
*  MEDIATEK FOR SUCH MEDIATEK SOFTWARE AT ISSUE.
*
*  THE TRANSACTION CONTEMPLATED HEREUNDER SHALL BE CONSTRUED IN ACCORDANCE
*  WITH THE LAWS OF THE STATE OF CALIFORNIA, USA, EXCLUDING ITS CONFLICT OF
*  LAWS PRINCIPLES.  ANY DISPUTES, CONTROVERSIES OR CLAIMS ARISING THEREOF AND
*  RELATED THERETO SHALL BE SETTLED BY ARBITRATION IN SAN FRANCISCO, CA, UNDER
*  THE RULES OF THE INTERNATIONAL CHAMBER OF COMMERCE (ICC).
*
*****************************************************************************/

#ifdef BUILD_LK
#else
    #include <linux/string.h>
    #if defined(BUILD_UBOOT)
        #include <asm/arch/mt_gpio.h>
    #else
        #include <mach/mt_gpio.h>
    #endif
#endif
#include "lcm_drv.h"


// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (480)
#define FRAME_HEIGHT (854)
#define PHYSICAL_HEIGHT (99)
#define PHYSICAL_WIDTH  (55)
//#define LCM_DSI_CMD_MODE

#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

//static unsigned int lcm_esd_test = FALSE;      ///only for ESD test

// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util = {0};

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             							0XFE
#define REGFLAG_END_OF_TABLE      							0xFD   // END OF REGISTERS MARKER

#define LCM_ID       (0x1080)



// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update)	lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)		lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)									lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)				lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg											lcm_util.dsi_read_reg()
#define read_reg_v2(cmd, buffer, buffer_size)   				lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)     

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};
//static int vcom = 0x50;
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

    for(i = 0; i < count; i++) {
		
        unsigned cmd;
        cmd = table[i].cmd;
		
        switch (cmd) {
			/*
				  case 0xb6:
			table[i].para_list[0] = vcom;
			table[i].para_list[1] = vcom;
			dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
			vcom += 1;
			break;*/
					
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

static struct LCM_setting_table lcm_sleep_out_setting[] = {
    // Sleep Out
	{0x11, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

    // Display ON
	{0x29, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_sleep_in_setting[] = {
	// Display off sequence
	{0x28, 1, {0x00}},
	{REGFLAG_DELAY, 10, {}},

    // Sleep Mode On
	{0x10, 1, {0x00}},
    {REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

static struct LCM_setting_table lcm_initialization_setting[] = {
	
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
//************* Start Initial Sequence **********// 
#if 1
	{0xB9, 3,{0xF1,0x08,0x01}},
          
{0xB1, 4,{0x22,0x1E,0x1E,0x87}},
          
{0xB2, 1,{0x22}},
          
{0xB3, 8,{0x01,0x00,0x06,0x06,0x18,0x13,0x39,0x35}},
          
{0xBA,17,{0x31,0x00,0x44,0x25,0x91,0x0A,0x00,0x00,0xE1,0x00,0x00,0x00,0x0F,0x02,0x4F,0xB9,0xEE}},
          
{0xE3, 5,{0x03,0x03,0x03,0x03,0x00}},
          
{0xB4, 1,{0x02}},
          
{0xB5, 2,{0xA5,0xA5}},
          
{0xB6, 2,{0x54,0x54}},
          
{0xB8, 2,{0x64,0x20}},
          
{0xCC, 1,{0x00}},
          
{0xBC, 1,{0x47}},
          
{0xE9,51,{0x00,0x00,0x06,0x00,0x00,0x0A,0x80,0x12,0x30,0x00,0x23,0x0A,0x0A,0x80,0x27,0x00,0x03,0x00,0x00,0x00,0x08,0x08,0x98,0xDD,0x20,0x64,0x02,0x88,0x88,0x88,0x88,0x98,0xDD,0x31,0x75,0x13,0x88,0x88,0x88,0x88,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},
          
{0xEA,22,{0x06,0x00,0x00,0x00,0x89,0xDD,0x35,0x71,0x31,0x88,0x88,0x88,0x88,0x89,0xDD,0x24,0x60,0x20,0x88,0x88,0x88,0x88}},
          
{0xE0,34,{0x00,0x00,0x00,0x11,0x11,0x3F,0x1E,0x33,0x06,0x0F,0x11,0x15,0x18,0x16,0x16,0x12,0x16,0x00,0x00,0x00,0x11,0x11,0x3F,0x1E,0x33,0x06,0x0F,0x11,0x15,0x18,0x16,0x16,0x12,0x16}},

//{0x51,  1 ,{0x7F}},

//{0x53,  1 ,{0x2C}},

//{0xC7,  5 ,{0xC5,0x00,0x00,0x00,0x05}}, 



{0x11, 1 , {0x00}},
{REGFLAG_DELAY, 150, {}},


{0x29, 1 , {0x00}},

{REGFLAG_DELAY, 20, {}},

#else
{0xFF,5,{0xFF,0x98,0x06,0x04,0x01}}, // Change to Page 1
{0x08,1,{0x10}},    
{0x21,1,{0x01}},   
{0x30,1,{0x01}},    // 480 X 854
{0x31,1,{0x00}},     // column inversion
{0x40,1,{0x11}},    
{0x41,1,{0x44}},  // DDVDH/DDVDL clamp
{0x42,1,{0x02}},   // VGH/VGL 
{0x43,1,{0x09}},   
{0x44,1,{0x06}},  
{0x50,1,{0x90}},   // VGMP
{0x51,1,{0x90}},    // VGMN
{0x52,1,{0x00}},
{0x53,1,{0x34}},     //Flicker//20 2E 35 40
{0x57,1,{0x50}},
{0x60,1,{0x07}},     
{0x61,1,{0x00}},    
{0x62,1,{0x09}},     
{0x63,1,{0x00}},    
{0xA0,1,{0x00}},
{0xA1,1,{0x02}},
{0xA2,1,{0x09}},
{0xA3,1,{0x0E}},
{0xA4,1,{0x0A}},
{0xA5,1,{0x1B}},
{0xA6,1,{0x0C}},
{0xA7,1,{0x0B}},
{0xA8,1,{0x00}},
{0xA9,1,{0x08}},
{0xAA,1,{0x04}},
{0xAB,1,{0x05}},
{0xAC,1,{0x0C}},
{0xAD,1,{0x32}},
{0xAE,1,{0x2D}},
{0xAF,1,{0x00}},
{0xC0,1,{0x00}},
{0xC1,1,{0x03}},
{0xC2,1,{0x09}},
{0xC3,1,{0x0E}},
{0xC4,1,{0x07}},
{0xC5,1,{0x1B}},
{0xC6,1,{0x0A}},
{0xC7,1,{0x09}},
{0xC8,1,{0x04}},
{0xC9,1,{0x09}},
{0xCA,1,{0x03}},
{0xCB,1,{0x02}},
{0xCC,1,{0x0B}},
{0xCD,1,{0x2F}},
{0xCE,1,{0x2C}},
{0xCF,1,{0x00}},
{0xFF,5,{0xFF,0x98,0x06,0x04,0x06}},     // Change to Page 6
{0x00,1,{0x21}},
{0x01,1,{0x06}},
{0x02,1,{0x00}},
{0x03,1,{0x02}},
{0x04,1,{0x00}},
{0x05,1,{0x16}},
{0x06,1,{0x80}},
{0x07,1,{0x02}},
{0x08,1,{0x07}},
{0x09,1,{0x00}},
{0x0A,1,{0x00}},
{0x0B,1,{0x00}},
{0x0C,1,{0x28}},
{0x0D,1,{0x28}},
{0x0E,1,{0x00}},
{0x0F,1,{0x00}},
{0x10,1,{0x77}},
{0x11,1,{0xF0}},
{0x12,1,{0x00}},
{0x13,1,{0x00}},
{0x14,1,{0x00}},
{0x15,1,{0xC0}},
{0x16,1,{0x08}},
{0x17,1,{0x00}},
{0x18,1,{0x00}},
{0x19,1,{0x00}},
{0x1A,1,{0x00}},
{0x1B,1,{0x00}},
{0x1C,1,{0x00}},
{0x1D,1,{0x00}},
{0x20,1,{0x01}},
{0x21,1,{0x23}},
{0x22,1,{0x45}},
{0x23,1,{0x67}},
{0x24,1,{0x01}},
{0x25,1,{0x23}},
{0x26,1,{0x45}},
{0x27,1,{0x67}},
{0x30,1,{0x11}},
{0x31,1,{0x22}},
{0x32,1,{0x11}},
{0x33,1,{0x00}},
{0x34,1,{0x66}},
{0x35,1,{0x88}},
{0x36,1,{0x22}},
{0x37,1,{0x22}},
{0x38,1,{0xAA}},
{0x39,1,{0xCC}},
{0x3A,1,{0xBB}},
{0x3B,1,{0xDD}},
{0x3C,1,{0x22}},
{0x3D,1,{0x22}},
{0x3E,1,{0x22}},
{0x3F,1,{0x22}},
{0x40,1,{0x22}},
{0x52,1,{0x10}},
{0x53,1,{0x10}},
{0xFF,5,{0xFF,0x98,0x06,0x04,0x07}},      // Change to Page 7
{0x17,1,{0x22}},     
{0x02,1,{0x77}},
{0xE1,1,{0x79}},     
{0xFF,5,{0xFF,0x98,0x06,0x04,0x00}},    // Change to Page 0
{0x35,1,{0x00}},      // TE on
{0x11,  1,      {0x00}},       //Sleep Out    
{REGFLAG_DELAY, 120, {}},                     
{0x29,  1,      {0x00}},         //Display On                     
{REGFLAG_DELAY, 20, {}},                                                                    
{REGFLAG_END_OF_TABLE, 0x00, {}}  
#endif
};
static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
    memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{

			memset(params, 0, sizeof(LCM_PARAMS));	 
			params->type	 = LCM_TYPE_DSI;			
			params->width  = FRAME_WIDTH; 	
			params->height = FRAME_HEIGHT;	
					
		    // physical size
		    params->physical_width = PHYSICAL_WIDTH;
		    params->physical_height = PHYSICAL_HEIGHT;
			// enable tearing-free		
			params->dbi.te_mode 			= LCM_DBI_TE_MODE_VSYNC_ONLY;
			//LCM_DBI_TE_MODE_DISABLED;
			//LCM_DBI_TE_MODE_VSYNC_ONLY;  
			params->dbi.te_edge_polarity		= LCM_POLARITY_RISING; 
			///////////////////// 	
			//if(params->dsi.lcm_int_te_monitor)	
			//params->dsi.vertical_frontporch *=2;	
			//params->dsi.lcm_ext_te_monitor= 0;//TRUE; 
			//guohongjin 20130725 begin
			params->dsi.noncont_clock= TRUE;//FALSE;	 
			params->dsi.noncont_clock_period=2;
			//guohongjin 20130725 end
			////////////////////					
			params->dsi.mode	 = SYNC_PULSE_VDO_MODE;  
			// DSI		/* Command mode setting */	
			params->dsi.LANE_NUM				= LCM_TWO_LANE; 		 
			//The following defined the fomat for data coming from LCD engine.	
			params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;	 
			params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST; 
			params->dsi.data_format.padding 	= LCM_DSI_PADDING_ON_LSB; 	 
			params->dsi.data_format.format		= LCM_DSI_FORMAT_RGB888;			 
			// Video mode setting 		 
			params->dsi.intermediat_buffer_num = 2;  
			params->dsi.PS=LCM_PACKED_PS_24BIT_RGB888;	
			params->dsi.packet_size=256;		
			// params->dsi.word_count=480*3;	
			//DSI CMD mode need set these two bellow params, different to 6577	 
			// params->dsi.vertical_active_line=800;	 
			params->dsi.vertical_sync_active				= 4;	 
			params->dsi.vertical_backporch				= 12; //8	 
			params->dsi.vertical_frontporch 			= 12; 	 
			params->dsi.vertical_active_line				= FRAME_HEIGHT; 		
			params->dsi.horizontal_sync_active				= 10;	 //6
			params->dsi.horizontal_backporch				= 50;  //37  
			params->dsi.horizontal_frontporch 			= 50; 	 //37
			params->dsi.horizontal_blanking_pixel 			= 60; 	
			params->dsi.horizontal_active_pixel 			= FRAME_WIDTH;		 
			// Bit rate calculation 	 
			// Bit rate calculation 	 
			//	params->dsi.pll_div1=26;//26; 	
			// fref=26MHz, fvco=fref*(div1+1) (div1=0~63, fvco=500MHZ~1GHz) 	
			//	params->dsi.pll_div2=1; 		
			// div2=0~15: fout=fvo/(2*div2) 			
			// params->dsi.pll_div1=26;//26;		
			// fref=26MHz, fvco=fref*(div1+1) (div1=0~63, fvco=500MHZ~1GHz) 	
			// params->dsi.pll_div2=1;		
			// div2=0~15: fout=fvo/(2*div2) 		 
			//params->dsi.pll_div1=1; 		 
			//params->dsi.pll_div2=1; 		 
			//params->dsi.fbk_div =28;//17	
			//liqiang 20140820 beign
			// To fix boot error
			params->dsi.PLL_CLOCK = 189;
			//params->dsi.customization_esd_check_enable = 1;
			params->dsi.esd_check_enable = 1;
			params->dsi.lcm_esd_check_table[0].cmd          = 0x0A;
			params->dsi.lcm_esd_check_table[0].count        = 1;
			params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9C;
			//liqiang 20140820 end
	
}

static void lcm_init(void)
{
    SET_RESET_PIN(1);
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(20);
    SET_RESET_PIN(1);
    MDELAY(120);
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);
}


static void lcm_suspend(void)
{
	//guohongjin 20131008 begin
			push_table(lcm_sleep_in_setting, sizeof(lcm_sleep_in_setting) / sizeof(struct LCM_setting_table), 1);
			SET_RESET_PIN(1);
			MDELAY(5);
			SET_RESET_PIN(0);
			MDELAY(20);
			SET_RESET_PIN(1);
			MDELAY(120);
	//guohongjin 20131008 end


}


static void lcm_resume(void)
{
	lcm_init();
	push_table(lcm_sleep_out_setting, sizeof(lcm_sleep_out_setting) / sizeof(struct LCM_setting_table), 1);

}


static void lcm_update(unsigned int x, unsigned int y,
                       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0>>8)&0xFF);
	unsigned char x0_LSB = (x0&0xFF);
	unsigned char x1_MSB = ((x1>>8)&0xFF);
	unsigned char x1_LSB = (x1&0xFF);
	unsigned char y0_MSB = ((y0>>8)&0xFF);
	unsigned char y0_LSB = (y0&0xFF);
	unsigned char y1_MSB = ((y1>>8)&0xFF);
	unsigned char y1_LSB = (y1&0xFF);

	unsigned int data_array[16];

	data_array[0]= 0x00053902;
	data_array[1]= (x1_MSB<<24)|(x0_LSB<<16)|(x0_MSB<<8)|0x2a;
	data_array[2]= (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);
	
	data_array[0]= 0x00053902;
	data_array[1]= (y1_MSB<<24)|(y0_LSB<<16)|(y0_MSB<<8)|0x2b;
	data_array[2]= (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

//	data_array[0]= 0x00290508; //HW bug, so need send one HS packet
//	dsi_set_cmdq(data_array, 1, 1);
	
	data_array[0]= 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}

static unsigned int lcm_compare_id(void)
{
	unsigned int id1 = 0, id2 = 0, id3 = 0, id4 = 0,id;
	unsigned char buffer[6];
	unsigned int LCD_ID_value = 0;
	unsigned int data_array[16];

    SET_RESET_PIN(1);	
    MDELAY(5);
    SET_RESET_PIN(0);
    MDELAY(10);
    SET_RESET_PIN(1);
    MDELAY(120);
	//FIX ME : liqiang 20150228 add this refer to spec  
	data_array[0] = 0x00033700;
	dsi_set_cmdq(data_array, 1, 3);
	read_reg_v2(0x04, buffer, 3);

	id1 = buffer[0];
	id2 = buffer[1];
	id = (id1 << 8) | id2; 
	//printf("wind fl10802:id=0x%x  0x%x 0x%x 0x%x\n",id,buffer[0],buffer[1],buffer[2]);
	return (LCM_ID == id)?1:0;
	
}

static unsigned int lcm_esd_check(void)
{
		#ifndef BUILD_LK     
			//  return 0; //FALSE  
		unsigned int id ,id1,id2,id3,id4,id5,id6,id7 = 0;  
		unsigned char buffer[6];   
		unsigned int data_array[16];   
		static int lcm_id;  
		UDELAY(600); //guohongjin 20131127 modify
		data_array[0] = 0x00013700;// read id return two byte,version and id  
		dsi_set_cmdq(data_array, 1, 1);  
		//MDELAY(10);
		read_reg_v2(0x0A, buffer, 1);    // A1  
		id = buffer[0]; //we only need ID     
		//printk("ghj ########## ili9806e lcd_id=%x,\r\n",id);
		if(id ==0x9c)     
			{    
			return 0;   
			}      
		else      
			{         
			return 1; //TRUE     

			}
		#endif
}

static unsigned int lcm_esd_recover(void)
{

	lcm_init();

    return TRUE;
}

static unsigned int lcm_check_status(void)
{
	unsigned char buffer[2];

	read_reg_v2(0x0A, buffer, 2);
#ifdef BUILD_LK
	printf("Check LCM Status: 0x%08x\n", buffer[0]);
#else
	printk("Check LCM Status: 0x%08x\n", buffer[0]);
#endif
	return 0;
}


// ---------------------------------------------------------------------------
//  Get LCM Driver Hooks
// ---------------------------------------------------------------------------
LCM_DRIVER fl10802_fwvga_dsi_vdo_hsd_lcm_drv = 
{
  .name			= "fl10802_fwvga_dsi_vdo_hsd",
	.set_util_funcs = lcm_set_util_funcs,
	.compare_id     = lcm_compare_id,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.suspend        = lcm_suspend,
	.resume         = lcm_resume,
#if defined(LCM_DSI_CMD_MODE)
    .update         = lcm_update,
#endif
    .esd_check      = lcm_esd_check,
    .esd_recover    = lcm_esd_recover,

    };
//liqiang 20150228 end
