/*
	liuying 20150908 add begin
*/
/*****************************************************************************
 *
 * Filename:
 * ---------
 *	 hi551mipi_Sensor.c
 *
 * Project:
 * --------
 *	 ALPS
 *
 * Description:
 * ------------
 *	 Source code of Sensor driver
 *
 *
 *------------------------------------------------------------------------------
 * Upper this line, this part is controlled by CC/CQ. DO NOT MODIFY!!
 *============================================================================
 ****************************************************************************/

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <asm/atomic.h>
//#include <asm/system.h>
#include <linux/xlog.h>

#include "kd_camera_hw.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_imgsensor_errcode.h"

#include "hi551mipi_Sensor.h"

/****************************Modify Following Strings for Debug****************************/
#define PFX "HI551_camera_sensor"
#define LOG_1 LOG_INF("HI551,MIPI 2LANE\n")
#define LOG_2 LOG_INF("preview 1280*960@30fps,864Mbps/lane; video 1280*960@30fps,864Mbps/lane; capture 5M@30fps,864Mbps/lane\n")
/****************************   Modify end    *******************************************/

#define LOG_INF(format, args...)    xlog_printk(ANDROID_LOG_INFO   , PFX, "[%s] " format, __FUNCTION__, ##args)

static DEFINE_SPINLOCK(imgsensor_drv_lock);
extern int iWriteReg(u16 a_u2Addr , u32 a_u4Data , u32 a_u4Bytes , u16 i2cId);
//#define HI551MIPI_I2C_ID  0x40
#define write_cmos_sensor(addr, para) iWriteReg((u16) addr , (u32) para , 2, 0x40)
extern void kdSetI2CSpeed(u32 i2cSpeed);//liuying 20150908 add 

static imgsensor_info_struct imgsensor_info = {
    	.sensor_id = HI551_SENSOR_ID,		//record sensor id defined in Kd_imgsensor.h
	
	.checksum_value = 0x9323df37,		//checksum value for Camera Auto Test
	
	.pre = {
		.pclk = 176000000,				//record different mode's pclk
		.linelength = 2940,				//record different mode's linelength
		.framelength = 1980,			//record different mode's framelength
		.startx = 0,					//record different mode's startx of grabwindow
		.starty = 0,					//record different mode's starty of grabwindow
		.grabwindow_width = 1296 ,		//record different mode's width of grabwindow
		.grabwindow_height = 972 ,		//record different mode's height of grabwindow
		/*	 following for MIPIDataLowPwr2HighSpeedSettleDelayCount by different scenario	*/
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		/*	 following for GetDefaultFramerateByScenario()	*/
		.max_framerate = 300,	
	},
	.cap = {
		.pclk = 176000000,
		.linelength = 2940,				//record different mode's linelength
		.framelength = 1980,			//record different mode's framelength
		.startx = 0,
		.starty = 0,    
  	       .grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 300,
	},
    .cap1 = {                            //capture for PIP 24fps relative information, capture1 mode must use same framelength, linelength with Capture mode for shutter calculate
        .pclk = 176000000,
		.linelength = 2940,				//record different mode's linelength
		.framelength = 1980,			//record different mode's framelength
		.startx = 0,
		.starty = 0,    
  	       .grabwindow_width = 2592,
		.grabwindow_height = 1944,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 150,  //less than 13M(include 13M),cap1 max framerate is 24fps,16M max framerate is 20fps, 20M max framerate is 15fps
    },
    .normal_video = {
		.pclk = 176000000,
        .linelength = 2940,
        .framelength = 1980,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1944,
        .grabwindow_height = 1458,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,
    },
    .hs_video = {
        .pclk = 176000000,
        .linelength = 2940,				//record different mode's linelength
		.framelength = 498,			//record different mode's framelength
		.startx = 0,
		.starty = 0,
		.grabwindow_width = 640 ,		//record different mode's width of grabwindow
		.grabwindow_height = 480 ,
		.mipi_data_lp2hs_settle_dc = 85,//unit , ns
		.max_framerate = 1200,
    },
    .slim_video = {
        .pclk = 176000000,
        .linelength = 2940,
        .framelength = 1980,
        .startx = 0,
        .starty = 0,
        .grabwindow_width = 1280,
        .grabwindow_height = 720,
        .mipi_data_lp2hs_settle_dc = 85,//unit , ns
        .max_framerate = 300,	   
    },
    .margin = 4,            //sensor framelength & shutter margin
    .min_shutter = 1,        //min shutter
    .max_frame_length = 0x7fff,//max framelength by sensor register's limitation
    .ae_shut_delay_frame = 0,    //shutter delay frame for AE cycle, 2 frame with ispGain_delay-shut_delay=2-0=2
    .ae_sensor_gain_delay_frame = 0,//sensor gain delay frame for AE cycle,2 frame with ispGain_delay-sensor_gain_delay=2-0=2
    .ae_ispGain_delay_frame = 2,//isp gain delay frame for AE cycle
    .ihdr_support = 0,      //1, support; 0,not support
    .ihdr_le_firstline = 0,  //1,le first ; 0, se first
    .sensor_mode_num = 5,      //support sensor mode num

    .cap_delay_frame = 1,        //enter capture delay frame num                     //liuying 20150825
    .pre_delay_frame = 1,         //enter preview delay frame num                    //liuying 20150825
    .video_delay_frame = 1,        //enter video delay frame num                     //liuying 20150825
    .hs_video_delay_frame = 1,    //enter high speed video  delay frame num    //liuying 20150825
    .slim_video_delay_frame = 1,//enter slim video delay frame num              //liuying 20150825
	
	.isp_driving_current = ISP_DRIVING_8MA,
	.sensor_interface_type = SENSOR_INTERFACE_TYPE_MIPI,
    .mipi_sensor_type = MIPI_OPHY_NCSI2, //0,MIPI_OPHY_NCSI2;  1,MIPI_OPHY_CSI2
    .mipi_settle_delay_mode = 1,//0,MIPI_SETTLEDELAY_AUTO; 1,MIPI_SETTLEDELAY_MANNUAL
    .sensor_output_dataformat = SENSOR_OUTPUT_FORMAT_RAW_Gb,//sensor output first pixel color
    .mclk = 24,//mclk value, suggest 24 or 26 for 24Mhz or 26Mhz
    .mipi_lane_num = SENSOR_MIPI_2_LANE,//mipi lane num
    .i2c_addr_table = {0x40, 0xff},//record sensor support all write id addr, only supprt 4must end with 0xff
};


static imgsensor_struct imgsensor = {
    .mirror = IMAGE_NORMAL,                //mirrorflip information
    .sensor_mode = IMGSENSOR_MODE_INIT, //IMGSENSOR_MODE enum value,record current sensor mode,such as: INIT, Preview, Capture, Video,High Speed Video, Slim Video
    .shutter = 0x3D0,                    //current shutter
    .gain = 0x100,                        //current gain
    .dummy_pixel = 0,                    //current dummypixel
    .dummy_line = 0,                    //current dummyline
    .current_fps = 300,  //full size current fps : 24fps for PIP, 30fps for Normal or ZSD
    .autoflicker_en = KAL_FALSE,  //auto flicker enable: KAL_FALSE for disable auto flicker, KAL_TRUE for enable auto flicker
    .test_pattern = KAL_FALSE,        //test pattern mode or not. KAL_FALSE for in test pattern mode, KAL_TRUE for normal output
    .current_scenario_id = MSDK_SCENARIO_ID_CAMERA_PREVIEW,//current scenario id
    .ihdr_en = 0, //sensor need support LE, SE with HDR feature
    .i2c_write_id = 0x40,//record current sensor's i2c write id
};


/* Sensor output window information */
static SENSOR_WINSIZE_INFO_STRUCT imgsensor_winsize_info[5] =
	{{ 2592, 1944,		0,	  0, 2592, 1944, 1296,	972, 0000, 0000, 1296,	972,	  0,	0, 1296,  972}, // Preview
	{ 2592, 1944,	   0,	 0, 2592, 1944, 2592, 1944, 0000, 0000, 2592, 1944, 	 0,    0, 2592, 1944}, // capture
	{ 2592, 1944,	   0,	 0, 2592, 1944, 1944, 1458, 0000, 0000,1944, 1458,		0,	  0, 1944, 1458}, // video
	{ 2592, 1944,	   15,	12,  2562,	1920,  640,  480, 0000, 0000,  640,  480,	   0,	 0,  640,  480}, //hight speed video
	{ 2592, 1944,	   16,	252, 2560, 1440,  1280,  720, 0000, 0000, 1280,  720,	   0,	 0, 1280,  720}};// slim video



static kal_uint16 read_cmos_sensor(kal_uint32 addr)
{
	kal_uint16 get_byte=0;

	char pu_send_cmd[2] = {(char)(addr >> 8), (char)(addr & 0xFF) };
	iReadRegI2C(pu_send_cmd, 2, (u8*)&get_byte, 1, imgsensor.i2c_write_id);

	return get_byte;
}

/*static void write_hi551_sensor(kal_uint32 addr, kal_uint32 para)
{
	char pu_send_cmd[3] = {(char)(addr >> 8), (char)(addr & 0xFF), (char)(para & 0xFF)};
	iWriteRegI2C(pu_send_cmd, 3, imgsensor.i2c_write_id);
}
*/
//liuying 20150302 add begin
#define HI545_USE_AWB_OTP
#ifdef HI545_USE_AWB_OTP
static void Hi545_bytewrite_cmos_sensor(kal_uint32 addr, kal_uint32 para)
{
	iWriteReg((u16)addr, (u32)para, 1, imgsensor.i2c_write_id);
}

#define RG_Ratio_Typical 0x16E   //liuying 20150504 
#define BG_Ratio_Typical 0x100   //liuying 20150504 

struct HI545_otp_struct 
{
	kal_uint16 R_Gain;
	kal_uint16 B_Gain;
	kal_uint8   awb_status;
};

static struct HI545_otp_struct current_otp = {0};

void Hi545_Otp_Initial_Setting(void)
{
	LOG_INF("Hi545OTPSetting begin:\n ");
	write_cmos_sensor(0x0118, 0x0000); 
	write_cmos_sensor(0x2000, 0x4031); 
	write_cmos_sensor(0x2002, 0x8400); 
	write_cmos_sensor(0x2004, 0x4392); 
	write_cmos_sensor(0x2006, 0x731C); 
	write_cmos_sensor(0x2008, 0x43C2); 
	write_cmos_sensor(0x200a, 0x0F82); 
	write_cmos_sensor(0x200c, 0x425F); 
	write_cmos_sensor(0x200e, 0x0118); 
	write_cmos_sensor(0x2010, 0xF37F); 
	write_cmos_sensor(0x2012, 0x930F); 
	write_cmos_sensor(0x2014, 0x2002); 
	write_cmos_sensor(0x2016, 0x0CC8); 
	write_cmos_sensor(0x2018, 0x3FF9); 
	write_cmos_sensor(0x201a, 0x43D2); 
	write_cmos_sensor(0x201c, 0x0A80); 
	write_cmos_sensor(0x201e, 0x43D2); 
	write_cmos_sensor(0x2020, 0x0180); 
	write_cmos_sensor(0x2022, 0x4392); 
	write_cmos_sensor(0x2024, 0x019A); 
	write_cmos_sensor(0x2026, 0x40B2); 
	write_cmos_sensor(0x2028, 0x0009); 
	write_cmos_sensor(0x202a, 0x019B); 
	write_cmos_sensor(0x202c, 0x12B0); 
	write_cmos_sensor(0x202e, 0xF14A); 
	write_cmos_sensor(0x2030, 0x93D2); 
	write_cmos_sensor(0x2032, 0x003E); 
	write_cmos_sensor(0x2034, 0x2082); 
	write_cmos_sensor(0x2036, 0x4392); 
	write_cmos_sensor(0x2038, 0x760E); 
	write_cmos_sensor(0x203a, 0x425F); 
	write_cmos_sensor(0x203c, 0x0118); 
	write_cmos_sensor(0x203e, 0xF37F); 
	write_cmos_sensor(0x2040, 0x930F); 
	write_cmos_sensor(0x2042, 0x2005); 
	write_cmos_sensor(0x2044, 0x43C2); 
	write_cmos_sensor(0x2046, 0x0A80); 
	write_cmos_sensor(0x2048, 0x0B00); 
	write_cmos_sensor(0x204a, 0x7302); 
	write_cmos_sensor(0x204c, 0xFFF0); 
	write_cmos_sensor(0x204e, 0x9382); 
	write_cmos_sensor(0x2050, 0x760C); 
	write_cmos_sensor(0x2052, 0x2002); 
	write_cmos_sensor(0x2054, 0x0C64); 
	write_cmos_sensor(0x2056, 0x3FF1); 
	write_cmos_sensor(0x2058, 0x12B0); 
	write_cmos_sensor(0x205a, 0xF14A); 
	write_cmos_sensor(0x205c, 0x421F); 
	write_cmos_sensor(0x205e, 0x760A); 
	write_cmos_sensor(0x2060, 0x903F); 
	write_cmos_sensor(0x2062, 0x0200); 
	write_cmos_sensor(0x2064, 0x245F); 
	write_cmos_sensor(0x2066, 0x930F); 
	write_cmos_sensor(0x2068, 0x245D); 
	write_cmos_sensor(0x206a, 0x903F); 
	write_cmos_sensor(0x206c, 0x0100); 
	write_cmos_sensor(0x206e, 0x23E3); 
	write_cmos_sensor(0x2070, 0x40B2); 
	write_cmos_sensor(0x2072, 0x0005); 
	write_cmos_sensor(0x2074, 0x7600); 
	write_cmos_sensor(0x2076, 0x4382); 
	write_cmos_sensor(0x2078, 0x7602); 
	write_cmos_sensor(0x207a, 0x0262); 
	write_cmos_sensor(0x207c, 0x0000); 
	write_cmos_sensor(0x207e, 0x0222); 
	write_cmos_sensor(0x2080, 0x0000); 
	write_cmos_sensor(0x2082, 0x0262); 
	write_cmos_sensor(0x2084, 0x0000); 
	write_cmos_sensor(0x2086, 0x0260); 
	write_cmos_sensor(0x2088, 0x0000); 
	write_cmos_sensor(0x208a, 0x425F); 
	write_cmos_sensor(0x208c, 0x0186); 
	write_cmos_sensor(0x208e, 0x4F4B); 
	write_cmos_sensor(0x2090, 0x421A); 
	write_cmos_sensor(0x2092, 0x018A); 
	write_cmos_sensor(0x2094, 0x93D2); 
	write_cmos_sensor(0x2096, 0x018F); 
	write_cmos_sensor(0x2098, 0x2443); 
	write_cmos_sensor(0x209a, 0x425F); 
	write_cmos_sensor(0x209c, 0x018F); 
	write_cmos_sensor(0x209e, 0x4F4D); 
	write_cmos_sensor(0x20a0, 0x430C); 
	write_cmos_sensor(0x20a2, 0x431F); 
	write_cmos_sensor(0x20a4, 0x4C0E); 
	write_cmos_sensor(0x20a6, 0x930E); 
	write_cmos_sensor(0x20a8, 0x2403); 
	write_cmos_sensor(0x20aa, 0x5F0F); 
	write_cmos_sensor(0x20ac, 0x831E); 
	write_cmos_sensor(0x20ae, 0x23FD); 
	write_cmos_sensor(0x20b0, 0xFB0F); 
	write_cmos_sensor(0x20b2, 0x242E); 
	write_cmos_sensor(0x20b4, 0x430F); 
	write_cmos_sensor(0x20b6, 0x9D0F); 
	write_cmos_sensor(0x20b8, 0x2C2B); 
	write_cmos_sensor(0x20ba, 0x4A82); 
	write_cmos_sensor(0x20bc, 0x7600); 
	write_cmos_sensor(0x20be, 0x4C82); 
	write_cmos_sensor(0x20c0, 0x7602); 
	write_cmos_sensor(0x20c2, 0x4B82); 
	write_cmos_sensor(0x20c4, 0x7604); 
	write_cmos_sensor(0x20c6, 0x0264); 
	write_cmos_sensor(0x20c8, 0x0000); 
	write_cmos_sensor(0x20ca, 0x0224); 
	write_cmos_sensor(0x20cc, 0x0000); 
	write_cmos_sensor(0x20ce, 0x0264); 
	write_cmos_sensor(0x20d0, 0x0000); 
	write_cmos_sensor(0x20d2, 0x0260); 
	write_cmos_sensor(0x20d4, 0x0000); 
	write_cmos_sensor(0x20d6, 0x0268); 
	write_cmos_sensor(0x20d8, 0x0000); 
	write_cmos_sensor(0x20da, 0x0C5A); 
	write_cmos_sensor(0x20dc, 0x02E8); 
	write_cmos_sensor(0x20de, 0x0000); 
	write_cmos_sensor(0x20e0, 0x0CB5); 
	write_cmos_sensor(0x20e2, 0x02A8); 
	write_cmos_sensor(0x20e4, 0x0000); 
	write_cmos_sensor(0x20e6, 0x0CB5); 
	write_cmos_sensor(0x20e8, 0x0CB5); 
	write_cmos_sensor(0x20ea, 0x0CB5); 
	write_cmos_sensor(0x20ec, 0x0CB5); 
	write_cmos_sensor(0x20ee, 0x0CB5); 
	write_cmos_sensor(0x20f0, 0x0CB5); 
	write_cmos_sensor(0x20f2, 0x0CB5); 
	write_cmos_sensor(0x20f4, 0x0CB5); 
	write_cmos_sensor(0x20f6, 0x0C00); 
	write_cmos_sensor(0x20f8, 0x02E8); 
	write_cmos_sensor(0x20fa, 0x0000); 
	write_cmos_sensor(0x20fc, 0x0CB5); 
	write_cmos_sensor(0x20fe, 0x0268); 
	write_cmos_sensor(0x2100, 0x0000); 
	write_cmos_sensor(0x2102, 0x0C5A); 
	write_cmos_sensor(0x2104, 0x0260); 
	write_cmos_sensor(0x2106, 0x0000); 
	write_cmos_sensor(0x2108, 0x0C5A); 
	write_cmos_sensor(0x210a, 0x531F); 
	write_cmos_sensor(0x210c, 0x9D0F); 
	write_cmos_sensor(0x210e, 0x2BD5); 
	write_cmos_sensor(0x2110, 0x531C); 
	write_cmos_sensor(0x2112, 0x923C); 
	write_cmos_sensor(0x2114, 0x2BC6); 
	write_cmos_sensor(0x2116, 0x0261); 
	write_cmos_sensor(0x2118, 0x0000); 
	write_cmos_sensor(0x211a, 0x12B0); 
	write_cmos_sensor(0x211c, 0xF14A); 
	write_cmos_sensor(0x211e, 0x3F8B); 
	write_cmos_sensor(0x2120, 0x432D); 
	write_cmos_sensor(0x2122, 0x3FBE); 
	write_cmos_sensor(0x2124, 0x421F); 
	write_cmos_sensor(0x2126, 0x018A); 
	write_cmos_sensor(0x2128, 0x12B0); 
	write_cmos_sensor(0x212a, 0xF15C); 
	write_cmos_sensor(0x212c, 0x421F); 
	write_cmos_sensor(0x212e, 0x7606); 
	write_cmos_sensor(0x2130, 0x4FC2); 
	write_cmos_sensor(0x2132, 0x0188); 
	write_cmos_sensor(0x2134, 0x0261); 
	write_cmos_sensor(0x2136, 0x0000); 
	write_cmos_sensor(0x2138, 0x3F7E); 
	write_cmos_sensor(0x213a, 0x0261); 
	write_cmos_sensor(0x213c, 0x0000); 
	write_cmos_sensor(0x213e, 0x43C2); 
	write_cmos_sensor(0x2140, 0x0180); 
	write_cmos_sensor(0x2142, 0x43D2); 
	write_cmos_sensor(0x2144, 0x003F); 
	write_cmos_sensor(0x2146, 0x4030); 
	write_cmos_sensor(0x2148, 0xF178); 
	write_cmos_sensor(0x214a, 0x0260); 
	write_cmos_sensor(0x214c, 0x0000); 
	write_cmos_sensor(0x214e, 0x0C5A); 
	write_cmos_sensor(0x2150, 0x0240); 
	write_cmos_sensor(0x2152, 0x0000); 
	write_cmos_sensor(0x2154, 0x0260); 
	write_cmos_sensor(0x2156, 0x0000); 
	write_cmos_sensor(0x2158, 0x0C14); 
	write_cmos_sensor(0x215a, 0x4130); 
	write_cmos_sensor(0x215c, 0x4382); 
	write_cmos_sensor(0x215e, 0x7602); 
	write_cmos_sensor(0x2160, 0x4F82); 
	write_cmos_sensor(0x2162, 0x7600); 
	write_cmos_sensor(0x2164, 0x0270); 
	write_cmos_sensor(0x2166, 0x0000); 
	write_cmos_sensor(0x2168, 0x0C1B); 
	write_cmos_sensor(0x216a, 0x0270); 
	write_cmos_sensor(0x216c, 0x0001); 
	write_cmos_sensor(0x216e, 0x421F); 
	write_cmos_sensor(0x2170, 0x7606); 
	write_cmos_sensor(0x2172, 0x4FC2); 
	write_cmos_sensor(0x2174, 0x0188); 
	write_cmos_sensor(0x2176, 0x4130); 
	write_cmos_sensor(0x2178, 0xDF02); 
	write_cmos_sensor(0x217a, 0x3FFE); 
	write_cmos_sensor(0x217c, 0x0000); 
	write_cmos_sensor(0x2ffe, 0xf000); 
	write_cmos_sensor(0x0f02, 0x0000);	// r_line_length_lines
	write_cmos_sensor(0x003E, 0x0100);	// r_line_length_lines
	write_cmos_sensor(0x0118, 0x0100);
	//Hi545OTPSetting();
}
void Hi545OTPSetting(void)
{	
	Hi545_bytewrite_cmos_sensor(0x0A02, 0x01);
	Hi545_bytewrite_cmos_sensor(0x0118, 0x00); //sleep On
	mdelay(100);//liuying 20150825
	Hi545_bytewrite_cmos_sensor(0x0F02, 0x00); //pll disable
	Hi545_bytewrite_cmos_sensor(0x011A, 0x01); //CP TRI_H
	Hi545_bytewrite_cmos_sensor(0x011B, 0x09); //IPGM TRIM_H
	Hi545_bytewrite_cmos_sensor(0x0D04, 0x01); //Fsync Output enable
	Hi545_bytewrite_cmos_sensor(0x0D00, 0x07); //Fsync Output Drivability
	Hi545_bytewrite_cmos_sensor(0x004C, 0x01); //TG MCU enable
	Hi545_bytewrite_cmos_sensor(0x003E, 0x01); //OTP R/W
	Hi545_bytewrite_cmos_sensor(0x0118, 0x01); //sleep off
	//mdelay(10);
	//Hi545_bytewrite_cmos_sensor(0x004C, 0x00); //TG MCU disable
	//Hi545_bytewrite_cmos_sensor(0x004C, 0x01); //TG MCU enable
	//mdelay(2);//liuying 20150825
	LOG_INF("Hi545OTPSetting exit :\n ");
}

kal_uint16 Hi545_Sensor_OTP_read(kal_uint16 otp_addr)
{
    kal_uint16 data;	
    Hi545_bytewrite_cmos_sensor(0x010a, (otp_addr & 0xff00)>> 8); //start address H        
    Hi545_bytewrite_cmos_sensor(0x010b, otp_addr& 0x00ff); //start address L
    Hi545_bytewrite_cmos_sensor(0x0102, 0x01); //single read
    //	mdelay(2);//liuying 20150825
    data = read_cmos_sensor(0x0108); //OTP data read  
	return data;
}
/*
static kal_uint16 Hi545_Sensor_Get_OTP_Mid(void)
{
	kal_uint16 ModuleHouseID = 0,info_flag = 0; 
	info_flag = Hi545_Sensor_OTP_read(0x1801);
	LOG_INF("info_flag = 0x%x\n",info_flag);
    switch (info_flag)
	{	
		case 0x40:
			ModuleHouseID = Hi545_Sensor_OTP_read(0x1802);
    			break;
			
		case 0xd0:
			ModuleHouseID = Hi545_Sensor_OTP_read(0x1812);
			break;
		
		case 0xf4:
			ModuleHouseID = Hi545_Sensor_OTP_read(0x1822);
			break;
		
		default:		
			LOG_INF("Hi545_Sensor: info_flag error value: %d \n ",info_flag);
			break;
	}
    LOG_INF("ModuleHouseID = %d,info_flag = 0x%x.(0x40-->1,0xd0-->2,0xf4-->3)\n", ModuleHouseID,info_flag);
	return ModuleHouseID;
}
*/

void Hi545_Sensor_OTP_info(void)
{
	uint16_t  ModuleHouseID = 0, Year = 0, Month = 0, Day = 0, LensID = 0, VCMID = 0, DriverID = 0, IRBGID=0;
	               
       uint16_t  info_flag = 0, infocheck = 0, checksum = 0;
	
	info_flag = Hi545_Sensor_OTP_read(0x1801);
	printk("%s,info_flag = 0x%x\n",__func__,info_flag);
    switch (info_flag)
	{	
		case 0x01:
		ModuleHouseID = Hi545_Sensor_OTP_read(0x1802);		
		Year = Hi545_Sensor_OTP_read(0x1804);
		Month = Hi545_Sensor_OTP_read(0x1805);
		Day = Hi545_Sensor_OTP_read(0x1806);
		LensID = Hi545_Sensor_OTP_read(0x1808);
		VCMID = Hi545_Sensor_OTP_read(0x1809);
		DriverID = Hi545_Sensor_OTP_read(0x180a);
		IRBGID = Hi545_Sensor_OTP_read(0x180B);		
		infocheck = Hi545_Sensor_OTP_read(0x1812);//liuying 20150504 
    	break;
		case 0x13://liuying 20150504 
		ModuleHouseID = Hi545_Sensor_OTP_read(0x1813);
		Year = Hi545_Sensor_OTP_read(0x1815);
		Month = Hi545_Sensor_OTP_read(0x1816);
		Day = Hi545_Sensor_OTP_read(0x1817);		
		LensID = Hi545_Sensor_OTP_read(0x1819);
		VCMID = Hi545_Sensor_OTP_read(0x181a);
		DriverID = Hi545_Sensor_OTP_read(0x181b);
		IRBGID = Hi545_Sensor_OTP_read(0x181c);
		infocheck = Hi545_Sensor_OTP_read(0x1823);//liuying 20150504 
		break;
		case 0x37://liuying 20150504 
		ModuleHouseID = Hi545_Sensor_OTP_read(0x1824);		
		Year = Hi545_Sensor_OTP_read(0x1826);
		Month = Hi545_Sensor_OTP_read(0x1827);
		Day = Hi545_Sensor_OTP_read(0x1828);		
		LensID = Hi545_Sensor_OTP_read(0x182a);
		VCMID = Hi545_Sensor_OTP_read(0x182b);
		DriverID = Hi545_Sensor_OTP_read(0x182c);
		IRBGID = Hi545_Sensor_OTP_read(0x182d);		
		infocheck = Hi545_Sensor_OTP_read(0x1834);//liuying 20150504 
		break;
		default:		
		LOG_INF("HI545_Sensor: info_flag error value: %d \n ",info_flag);
		printk("wangk_HI545_Sensor: info_flag error value: %d \n ",info_flag);
		break;
	}
	
	checksum = (ModuleHouseID +Year + Month + Day +  LensID + VCMID + DriverID+IRBGID) % 0xFF + 1;
	
	if (checksum == infocheck)
		{
		LOG_INF("HI545_Sensor: Module information checksum PASS\n ");
		printk("wangk_HI545_Sensor: Module information checksum PASS\n ");
		}
	else
		{
		LOG_INF("HI545_Sensor: Module information checksum Fail\n ");
		printk("wangk_HI545_Sensor: Module information checksum Fail\n ");
		}

    LOG_INF("ModuleHouseID = %x \n", ModuleHouseID);
    LOG_INF("Year = %d, Month = %d, Day = %d\n", Year, Month, Day);    
    LOG_INF("LensID = %x \n", LensID);
    LOG_INF("VCMID = %x \n", VCMID);
    LOG_INF("DriverID = %x \n", DriverID);
    LOG_INF("IRBGID = %d \n", IRBGID);

	
	printk("wangk_ModuleHouseID = %d \n", ModuleHouseID);
	printk("wangk_Year = %d, Month = %d, Day = %d\n", Year, Month, Day);
	printk("wangk_LensID = %d \n", LensID);
	printk("wangk_VCMID = %d \n", VCMID);
	printk("wangk_DriverID = %d \n", DriverID);
	printk("wangk_IRBGID = %d \n", IRBGID);
	

	//HI545_Sensor_calc_wbdata();
}

void Hi545_Sensor_update_wb_gain(struct HI545_otp_struct *otp)
{
    kal_uint16 rg,bg;

    Hi545_bytewrite_cmos_sensor(0x0118, 0x00); //sleep On
    Hi545_bytewrite_cmos_sensor(0x003e, 0x00); //OTP mode off
    Hi545_bytewrite_cmos_sensor(0x0118, 0x01); //sleep Off

	rg = otp->R_Gain;
	bg = otp->B_Gain;
   LOG_INF("rg = 0x%x, bg = 0x%x\n", rg, bg);

    Hi545_bytewrite_cmos_sensor(0x050c, (rg & 0xff00) >> 8); //r_gain
    Hi545_bytewrite_cmos_sensor(0x050d, rg & 0x00ff); //r_gain
    Hi545_bytewrite_cmos_sensor(0x050e, (bg & 0xff00) >> 8); //b_gain
    Hi545_bytewrite_cmos_sensor(0x050f, bg & 0x00ff); //b_gain

   LOG_INF("Hi545 Sensor update wb gain finished!!!\n");
}

void  Hi545_Sensor_calc_wbdata(struct HI545_otp_struct *otp)
{
    kal_uint16 wbcheck = 0, checksum = 0, wb_flag = 0;
    kal_uint16 r_gain = 0, b_gain = 0, g_gain = 0;
    kal_uint16 wb_unit_rg_h = 0, wb_unit_rg_l = 0, wb_unit_bg_h = 0, wb_unit_bg_l = 0, wb_unit_gg_h = 0,wb_unit_gg_l = 0;			 
    kal_uint16 rg_value = 0, bg_value = 0, rg_golden_value=0, bg_golden_value=0;
    kal_uint16 wb_golden_rg_h=0x183C,wb_golden_rg_l=0x183D,wb_golden_bg_h=0x183E,wb_golden_bg_l=0x183F;//liuying 20150504 

	wb_flag = Hi545_Sensor_OTP_read(0x1835);
	
	LOG_INF("wb_flag = 0x%x\n",wb_flag);

    switch (wb_flag)
	{
		case 0x01:	//Group1 valid
		wb_unit_rg_h = Hi545_Sensor_OTP_read(0x1836);
		wb_unit_rg_l = Hi545_Sensor_OTP_read(0x1837);
		wb_unit_bg_h = Hi545_Sensor_OTP_read(0x1838);
		wb_unit_bg_l = Hi545_Sensor_OTP_read(0x1839);
		wb_unit_gg_h = Hi545_Sensor_OTP_read(0x183a);
		wb_unit_gg_l = Hi545_Sensor_OTP_read(0x183b);
		wbcheck = Hi545_Sensor_OTP_read(0x1853);//liuying 20150504 
		//wb_golden_rg_h=0x183C;
		//wb_golden_rg_l=0x183D,
		//wb_golden_bg_h=0x183E,
		//wb_golden_bg_l=0x183F;
    		break;
		case 0x13:	//Group2 valid   //liuying 20150504 
		wb_unit_rg_h = Hi545_Sensor_OTP_read(0x1846);
		wb_unit_rg_l = Hi545_Sensor_OTP_read(0x1847);
		wb_unit_bg_h = Hi545_Sensor_OTP_read(0x1848);
		wb_unit_bg_l = Hi545_Sensor_OTP_read(0x1849);
		wb_unit_gg_h = Hi545_Sensor_OTP_read(0x184a);
		wb_unit_gg_l = Hi545_Sensor_OTP_read(0x184b);		
		wbcheck = Hi545_Sensor_OTP_read(0x1871); //liuying 20150504 
		break;
		case 0x37:	//Group3 valid  //liuying 20150504 
		wb_unit_rg_h = Hi545_Sensor_OTP_read(0x1856);
		wb_unit_rg_l = Hi545_Sensor_OTP_read(0x1857);
		wb_unit_bg_h = Hi545_Sensor_OTP_read(0x1858);
		wb_unit_bg_l = Hi545_Sensor_OTP_read(0x1859);
		wb_unit_gg_h = Hi545_Sensor_OTP_read(0x185a);
		wb_unit_gg_l = Hi545_Sensor_OTP_read(0x185b);
		wbcheck = Hi545_Sensor_OTP_read(0x188f); //liuying 20150504 
		break;
		default:
		LOG_INF("Hi545_Sensor: wb_flag error value: 0x%x\n ",wb_flag);
		break;	
	}
	
		checksum = (wb_unit_rg_h + wb_unit_rg_l + wb_unit_bg_h + wb_unit_bg_l  ) % 0xFF + 1;


	//if (checksum == wbcheck)
		//{
			LOG_INF("Hi545_Sensor: WB checksum PASS\n ");
			otp->awb_status = 1;
		//}
	//else
		//{
			//LOG_INF("Hi545_Sensor: WB checksum Fail\n ");
		//}

	//rg_golden_value = (((wb_golden_rg_h << 2) | ((wb_golden_rg_l>>6) & 0x0003)) & 0x03ff);	//BIT[0-9], BIT[2-9] get from wb_golden_rg_h bit[0-7];BIT[0-1] get from wb_golden_rg_l bit[6-7] 
	//bg_golden_value = (((wb_golden_bg_h << 2) | ((wb_golden_bg_l>>6) & 0x0003)) & 0x03ff);
	rg_value = ((wb_unit_rg_h<<8)&0xff00) |(wb_unit_rg_l &0xff);                          //(((wb_unit_rg_h << 2) | ((wb_unit_rg_l>>6) & 0x0003)) & 0x03ff);
	bg_value = ((wb_unit_bg_h<<8)&0xff00) |(wb_unit_bg_l &0xff); 			           //(((wb_unit_bg_h << 2) | ((wb_unit_bg_l>>6) & 0x0003)) & 0x03ff);

	LOG_INF("Read from module: wb_unit_rg_h = 0x%x,wb_unit_rg_l = 0x%x wb_unit_bg_h = 0x%x,wb_unit_bg_l = 0x%x wb_unit_gg_h = 0x%x,wb_unit_gg_l = 0x%x\n",
		wb_unit_rg_h ,wb_unit_rg_l,wb_unit_bg_h,wb_unit_bg_l,wb_unit_gg_h,wb_unit_gg_l);

   LOG_INF("Read from module: rg_value = 0x%x,bg_value = 0x%x\n",rg_value ,bg_value);
   //LOG_INF("Read from module: rg_golden_value = 0x%x,bg_golden_value = 0x%x wbcheck=0x%x\n",rg_golden_value ,bg_golden_value,wbcheck);


    g_gain = 0x100;	
    //r_gain = (0x100 * rg_golden_value) / rg_value;
    //b_gain = (0x100 * bg_golden_value) / bg_value;
    r_gain = (0x100 * RG_Ratio_Typical) / rg_value;
    b_gain = (0x100 * BG_Ratio_Typical) / bg_value;

    LOG_INF("After calcuate: g_gain = 0x%x, r_gain = 0x%x,b_gain = 0x%x\n",g_gain,r_gain,b_gain);
	otp->R_Gain = r_gain;
	otp->B_Gain = b_gain;
  //  Hi545_Sensor_update_wb_gain( r_gain, b_gain);

}
#endif

////////////////////////////////////////////////////////////////////////////////////
//liuying 20150302 add end
static void set_dummy()
{


	LOG_INF("dummyline = %d, dummypixels = %d \n", imgsensor.dummy_line, imgsensor.dummy_pixel);
	/* you can set dummy by imgsensor.dummy_line and imgsensor.dummy_pixel, or you can set dummy by imgsensor.frame_length and imgsensor.line_length */
	
	write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x0006, imgsensor.frame_length );
	//write_cmos_sensor(0x0007, imgsensor.frame_length & 0xFF);	  
	write_cmos_sensor(0x0008, imgsensor.line_length );
	//write_cmos_sensor(0x0009, imgsensor.line_length & 0xFF);
	 write_cmos_sensor(0x0046, 0x0000);
  
}	/*	set_dummy  */

static kal_uint32 return_sensor_id()
{
    return ((read_cmos_sensor(0x0f16) << 8) | read_cmos_sensor(0x0f17));
}
static void set_max_framerate(UINT16 framerate,kal_bool min_framelength_en)
{
	kal_int16 dummy_line;
	kal_uint32 frame_length = imgsensor.frame_length;
	//unsigned long flags;

	LOG_INF("framerate = %d, min framelength should enable? \n", framerate,min_framelength_en);
   
	frame_length = imgsensor.pclk / framerate * 10 / imgsensor.line_length;
	spin_lock(&imgsensor_drv_lock);
	imgsensor.frame_length = (frame_length > imgsensor.min_frame_length) ? frame_length : imgsensor.min_frame_length; 
	imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	//dummy_line = frame_length - imgsensor.min_frame_length;
	//if (dummy_line < 0)
		//imgsensor.dummy_line = 0;
	//else
		//imgsensor.dummy_line = dummy_line;
	//imgsensor.frame_length = frame_length + imgsensor.dummy_line;
	if (imgsensor.frame_length > imgsensor_info.max_frame_length)
	{
		imgsensor.frame_length = imgsensor_info.max_frame_length;
		imgsensor.dummy_line = imgsensor.frame_length - imgsensor.min_frame_length;
	}
	if (min_framelength_en)
		imgsensor.min_frame_length = imgsensor.frame_length;
	spin_unlock(&imgsensor_drv_lock);
	set_dummy();
}	/*	set_max_framerate  */

static void set_shutter(kal_uint16 shutter)
{
    unsigned long flags;
    kal_uint16 realtime_fps = 0;
    kal_uint32 frame_length = 0;
    spin_lock_irqsave(&imgsensor_drv_lock, flags);
    imgsensor.shutter = shutter;
    spin_unlock_irqrestore(&imgsensor_drv_lock, flags);

    //write_shutter(shutter);
    /* 0x3500, 0x3501, 0x3502 will increase VBLANK to get exposure larger than frame exposure */
    /* AE doesn't update sensor gain at capture mode, thus extra exposure lines must be updated here. */

    // OV Recommend Solution
    // if shutter bigger than frame_length, should extend frame length first
    spin_lock(&imgsensor_drv_lock);
    if (shutter > imgsensor.min_frame_length - imgsensor_info.margin)
        imgsensor.frame_length = shutter + imgsensor_info.margin;
    else
        imgsensor.frame_length = imgsensor.min_frame_length;
    if (imgsensor.frame_length > imgsensor_info.max_frame_length)
        imgsensor.frame_length = imgsensor_info.max_frame_length;
    spin_unlock(&imgsensor_drv_lock);
    shutter = (shutter < imgsensor_info.min_shutter) ? imgsensor_info.min_shutter : shutter;
    shutter = (shutter > (imgsensor_info.max_frame_length - imgsensor_info.margin)) ? (imgsensor_info.max_frame_length - imgsensor_info.margin) : shutter;

    if (imgsensor.autoflicker_en) {
        realtime_fps = imgsensor.pclk / imgsensor.line_length * 10 / imgsensor.frame_length;
        if(realtime_fps >= 297 && realtime_fps <= 305)
            set_max_framerate(296,0);
        else if(realtime_fps >= 147 && realtime_fps <= 150)
            set_max_framerate(146,0);
        else {
        // Extend frame length
        write_cmos_sensor(0x0046, 0x0100);
			write_cmos_sensor(0x0006, imgsensor.frame_length );
			//write_cmos_sensor(0x0007, imgsensor.frame_length & 0xFF);
			write_cmos_sensor(0x0004, (shutter) );
			write_cmos_sensor(0x0046, 0x0000);
        }
    } else {
        // Extend frame length
write_cmos_sensor(0x0046, 0x0100);	
	write_cmos_sensor(0x0006, imgsensor.frame_length);
	write_cmos_sensor(0x0004, (shutter) );
	write_cmos_sensor(0x0046, 0x0000);

    }

	// Update Shutter
	write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x0004, (shutter) );
	//write_cmos_sensor(0x0005, shutter & 0xFF);	  
	write_cmos_sensor(0x0046, 0x0000);
	
		
	LOG_INF("Exit! shutter =%d, framelength =%d\n", shutter,imgsensor.frame_length);

}    /*    set_shutter */



/*************************************************************************
* FUNCTION
*	set_shutter
*
* DESCRIPTION
*	This function set e-shutter of sensor to change exposure time.
*
* PARAMETERS
*	iShutter : exposured lines
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/


static kal_uint16 gain2reg(const kal_uint16 gain)
{
	kal_uint16 reg_gain = 0x0000;
	reg_gain = gain / 4 - 16;
	
	reg_gain = reg_gain & 0xFFFF;
	return (kal_uint16)reg_gain;
}
/*************************************************************************
* FUNCTION
*	set_gain
*
* DESCRIPTION
*	This function is to set global gain to sensor.
*
* PARAMETERS
*	iGain : sensor global gain(base: 0x40)
*
* RETURNS
*	the actually gain set to sensor.
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint16 set_gain(kal_uint16 gain)
{
    kal_uint16 reg_gain;

    /* 0x350A[0:1], 0x350B[0:7] AGC real gain */
    /* [0:3] = N meams N /16 X    */
    /* [4:9] = M meams M X         */
    /* Total gain = M + N /16 X   */

    //
    if(gain > 1024)  
	{
		gain = 1024;
	}
	if(gain < 64)  
	{
		gain = 64;
	}

    reg_gain = gain2reg(gain);
    spin_lock(&imgsensor_drv_lock);
    imgsensor.gain = reg_gain;
    spin_unlock(&imgsensor_drv_lock);
    LOG_INF("gain = %d , reg_gain = 0x%x\n ", gain, reg_gain);

     write_cmos_sensor(0x0046, 0x0100);
	write_cmos_sensor(0x003a,(reg_gain&0xff)<<8);   
	write_cmos_sensor(0x0046, 0x0000);

    return gain;
}    /*    set_gain  */
static void ihdr_write_shutter_gain(kal_uint16 le, kal_uint16 se, kal_uint16 gain)
{
    LOG_INF("le:0x%x, se:0x%x, gain:0x%x\n",le,se,gain);
    if (imgsensor.ihdr_en) {

        spin_lock(&imgsensor_drv_lock);
        if (le > imgsensor.min_frame_length - imgsensor_info.margin)
            imgsensor.frame_length = le + imgsensor_info.margin;
        else
            imgsensor.frame_length = imgsensor.min_frame_length;
        if (imgsensor.frame_length > imgsensor_info.max_frame_length)
            imgsensor.frame_length = imgsensor_info.max_frame_length;
        spin_unlock(&imgsensor_drv_lock);
        if (le < imgsensor_info.min_shutter) le = imgsensor_info.min_shutter;
        if (se < imgsensor_info.min_shutter) se = imgsensor_info.min_shutter;


        // Extend frame length first
        write_cmos_sensor(0x380e, imgsensor.frame_length >> 8);
        write_cmos_sensor(0x380f, imgsensor.frame_length & 0xFF);

        write_cmos_sensor(0x3502, (le << 4) & 0xFF);
        write_cmos_sensor(0x3501, (le >> 4) & 0xFF);
        write_cmos_sensor(0x3500, (le >> 12) & 0x0F);

        write_cmos_sensor(0x3508, (se << 4) & 0xFF);
        write_cmos_sensor(0x3507, (se >> 4) & 0xFF);
        write_cmos_sensor(0x3506, (se >> 12) & 0x0F);

        set_gain(gain);
    }

}




static void set_mirror_flip(kal_uint8 image_mirror)
{
    LOG_INF("image_mirror = %d\n", image_mirror);

	/********************************************************
	   *
	   *   0x3820[2] ISP Vertical flip
	   *   0x3820[1] Sensor Vertical flip
	   *
	   *   0x3821[2] ISP Horizontal mirror
	   *   0x3821[1] Sensor Horizontal mirror
	   *
	   *   ISP and Sensor flip or mirror register bit should be the same!!
	   *
	   ********************************************************/
	
    switch (image_mirror) {
        case IMAGE_NORMAL://IMAGE_NOMAL:
            write_cmos_sensor(0x0000, 0x0000);//Set normal
            break;    
        case IMAGE_V_MIRROR://IMAGE_V_MIRROR:
            write_cmos_sensor(0x0000, 0x0200);	//Set flip
            break;
        case IMAGE_H_MIRROR://IMAGE_H_MIRROR:
            write_cmos_sensor(0x0000, 0x0100);//Set mirror
            break;
        case IMAGE_HV_MIRROR://IMAGE_H_MIRROR:
            write_cmos_sensor(0x0000, 0x0300);	//Set mirror & flip
            break;            
	default:
			LOG_INF("Error image_mirror setting\n");
    }

}

/*************************************************************************
* FUNCTION
*	night_mode
*
* DESCRIPTION
*	This function night mode of sensor.
*
* PARAMETERS
*	bEnable: KAL_TRUE -> enable night mode, otherwise, disable night mode
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static void night_mode(kal_bool enable)
{
/*No Need to implement this function*/ 
}	/*	night_mode	*/

static void sensor_init(void)
{
	LOG_INF("Enter sensor_init\n");
	kdSetI2CSpeed(400);//liuying 20150908 add
	
	//====================================================================
	// Hi-551 Initialize Register
	// Ver 140918
	// Validation/Project
	//====================================================================
	write_cmos_sensor(0x0118, 0x0000);	//sleep On
	
	write_cmos_sensor(0x0134, 0x1C1F);		 // Extra Dynamic Signal control 

	// SRAM Delay Margin -------------------------------------------
	write_cmos_sensor(0x0E00, 0x0001);  //150519 adp_sram_sdly
	write_cmos_sensor(0x0E02, 0x0001);  //150519 byrscl_sram_sdly0
	write_cmos_sensor(0x0E06, 0x0001);  //150519 byrscl_sram_sdly1
	write_cmos_sensor(0x0E0A, 0x0001);  //150519 byrscl_sram_sdly2
	write_cmos_sensor(0x0E0E, 0x0001);  //150519 byrscl_sram_sdly3
	write_cmos_sensor(0x0E12, 0x0001);  //150519 byrscl_fifomem_sram_sdly
	write_cmos_sensor(0x0E14, 0x0001);  //150519 tg_pmem_sram_sdly
	write_cmos_sensor(0x0E16, 0x0001);  //150519 tg_dmem_sram_sdly

	// Firmware Date : 20150331--------------------------------------
	write_cmos_sensor(0x2000, 0x4031);
	write_cmos_sensor(0x2002, 0x8400);
	write_cmos_sensor(0x2004, 0x4382);
	write_cmos_sensor(0x2006, 0x8172);
	write_cmos_sensor(0x2008, 0x4382);
	write_cmos_sensor(0x200a, 0x8104);
	write_cmos_sensor(0x200c, 0x4392);
	write_cmos_sensor(0x200e, 0x731C);
	write_cmos_sensor(0x2010, 0x4392);
	write_cmos_sensor(0x2012, 0x8120);
	write_cmos_sensor(0x2014, 0x4382);
	write_cmos_sensor(0x2016, 0x8130);
	write_cmos_sensor(0x2018, 0x4382);
	write_cmos_sensor(0x201a, 0x812E);
	write_cmos_sensor(0x201c, 0x4382);
	write_cmos_sensor(0x201e, 0x8100);
	write_cmos_sensor(0x2020, 0x4382);
	write_cmos_sensor(0x2022, 0x8126);
	write_cmos_sensor(0x2024, 0x4382);
	write_cmos_sensor(0x2026, 0x8124);
	write_cmos_sensor(0x2028, 0x4382);
	write_cmos_sensor(0x202a, 0x8122);
	write_cmos_sensor(0x202c, 0x4382);
	write_cmos_sensor(0x202e, 0x8108);
	write_cmos_sensor(0x2030, 0x4382);
	write_cmos_sensor(0x2032, 0x812C);
	write_cmos_sensor(0x2034, 0x430B);
	write_cmos_sensor(0x2036, 0x93D2);
	write_cmos_sensor(0x2038, 0x003F);
	write_cmos_sensor(0x203a, 0x2002);
	write_cmos_sensor(0x203c, 0x4030);
	write_cmos_sensor(0x203e, 0xF7B6);
	write_cmos_sensor(0x2040, 0x43C2);
	write_cmos_sensor(0x2042, 0x0F82);
	write_cmos_sensor(0x2044, 0x425F);
	write_cmos_sensor(0x2046, 0x0198);
	write_cmos_sensor(0x2048, 0xF37F);
	write_cmos_sensor(0x204a, 0x930F);
	write_cmos_sensor(0x204c, 0x2002);
	write_cmos_sensor(0x204e, 0x0CC8);
	write_cmos_sensor(0x2050, 0x3FF9);
	write_cmos_sensor(0x2052, 0x4F82);
	write_cmos_sensor(0x2054, 0x812A);
	write_cmos_sensor(0x2056, 0x43D2);
	write_cmos_sensor(0x2058, 0x0A80);
	write_cmos_sensor(0x205a, 0x43D2);
	write_cmos_sensor(0x205c, 0x0180);
	write_cmos_sensor(0x205e, 0x4392);
	write_cmos_sensor(0x2060, 0x019A);
	write_cmos_sensor(0x2062, 0x40B2);
	write_cmos_sensor(0x2064, 0x0009);
	write_cmos_sensor(0x2066, 0x019B);
	write_cmos_sensor(0x2068, 0x12B0);
	write_cmos_sensor(0x206a, 0xFE80);
	write_cmos_sensor(0x206c, 0x93D2);
	write_cmos_sensor(0x206e, 0x003E);
	write_cmos_sensor(0x2070, 0x2002);
	write_cmos_sensor(0x2072, 0x4030);
	write_cmos_sensor(0x2074, 0xF64E);
	write_cmos_sensor(0x2076, 0x4382);
	write_cmos_sensor(0x2078, 0x8128);
	write_cmos_sensor(0x207a, 0x4382);
	write_cmos_sensor(0x207c, 0x8102);
	write_cmos_sensor(0x207e, 0x421F);
	write_cmos_sensor(0x2080, 0x8128);
	write_cmos_sensor(0x2082, 0x521F);
	write_cmos_sensor(0x2084, 0x8102);
	write_cmos_sensor(0x2086, 0x503F);
	write_cmos_sensor(0x2088, 0x0030);
	write_cmos_sensor(0x208a, 0x12B0);
	write_cmos_sensor(0x208c, 0xFE92);
	write_cmos_sensor(0x208e, 0x421E);
	write_cmos_sensor(0x2090, 0x8102);
	write_cmos_sensor(0x2092, 0x4E0F);
	write_cmos_sensor(0x2094, 0x5F0F);
	write_cmos_sensor(0x2096, 0x429F);
	write_cmos_sensor(0x2098, 0x7606);
	write_cmos_sensor(0x209a, 0x813A);
	write_cmos_sensor(0x209c, 0x531E);
	write_cmos_sensor(0x209e, 0x4E82);
	write_cmos_sensor(0x20a0, 0x8102);
	write_cmos_sensor(0x20a2, 0x903E);
	write_cmos_sensor(0x20a4, 0x0003);
	write_cmos_sensor(0x20a6, 0x3BEB);
	write_cmos_sensor(0x20a8, 0x421F);
	write_cmos_sensor(0x20aa, 0x813C);
	write_cmos_sensor(0x20ac, 0x4F0D);
	write_cmos_sensor(0x20ae, 0xF03D);
	write_cmos_sensor(0x20b0, 0x000F);
	write_cmos_sensor(0x20b2, 0x108D);
	write_cmos_sensor(0x20b4, 0x521D);
	write_cmos_sensor(0x20b6, 0x813A);
	write_cmos_sensor(0x20b8, 0x4D82);
	write_cmos_sensor(0x20ba, 0x8138);
	write_cmos_sensor(0x20bc, 0x421E);
	write_cmos_sensor(0x20be, 0x813E);
	write_cmos_sensor(0x20c0, 0x5E0E);
	write_cmos_sensor(0x20c2, 0x5E0E);
	write_cmos_sensor(0x20c4, 0x5E0E);
	write_cmos_sensor(0x20c6, 0x5E0E);
	write_cmos_sensor(0x20c8, 0xC312);
	write_cmos_sensor(0x20ca, 0x100F);
	write_cmos_sensor(0x20cc, 0x110F);
	write_cmos_sensor(0x20ce, 0x110F);
	write_cmos_sensor(0x20d0, 0x110F);
	write_cmos_sensor(0x20d2, 0x5F0E);
	write_cmos_sensor(0x20d4, 0x4E82);
	write_cmos_sensor(0x20d6, 0x8136);
	write_cmos_sensor(0x20d8, 0x4D8B);
	write_cmos_sensor(0x20da, 0x5000);
	write_cmos_sensor(0x20dc, 0x429B);
	write_cmos_sensor(0x20de, 0x8136);
	write_cmos_sensor(0x20e0, 0x6000);
	write_cmos_sensor(0x20e2, 0x532B);
	write_cmos_sensor(0x20e4, 0x421F);
	write_cmos_sensor(0x20e6, 0x8128);
	write_cmos_sensor(0x20e8, 0x503F);
	write_cmos_sensor(0x20ea, 0x0003);
	write_cmos_sensor(0x20ec, 0x4F82);
	write_cmos_sensor(0x20ee, 0x8128);
	write_cmos_sensor(0x20f0, 0x903F);
	write_cmos_sensor(0x20f2, 0x0300);
	write_cmos_sensor(0x20f4, 0x3BC2);
	write_cmos_sensor(0x20f6, 0x0261);
	write_cmos_sensor(0x20f8, 0x0000);
	write_cmos_sensor(0x20fa, 0x43C2);
	write_cmos_sensor(0x20fc, 0x0180);
	write_cmos_sensor(0x20fe, 0x43D2);
	write_cmos_sensor(0x2100, 0x003F);
	write_cmos_sensor(0x2102, 0x403F);
	write_cmos_sensor(0x2104, 0x0B80);
	write_cmos_sensor(0x2106, 0x4F2E);
	write_cmos_sensor(0x2108, 0xD03E);
	write_cmos_sensor(0x210a, 0x8000);
	write_cmos_sensor(0x210c, 0x4E82);
	write_cmos_sensor(0x210e, 0x0B20);
	write_cmos_sensor(0x2110, 0xD0BF);
	write_cmos_sensor(0x2112, 0x8000);
	write_cmos_sensor(0x2114, 0x0000);
	write_cmos_sensor(0x2116, 0x0C0A);
	write_cmos_sensor(0x2118, 0x40B2);
	write_cmos_sensor(0x211a, 0x8085);
	write_cmos_sensor(0x211c, 0x0B20);
	write_cmos_sensor(0x211e, 0x40B2);
	write_cmos_sensor(0x2120, 0x8085);
	write_cmos_sensor(0x2122, 0x0B88);
	write_cmos_sensor(0x2124, 0x0C0A);
	write_cmos_sensor(0x2126, 0x40B2);
	write_cmos_sensor(0x2128, 0xAC49);
	write_cmos_sensor(0x212a, 0x0B20);
	write_cmos_sensor(0x212c, 0x40B2);
	write_cmos_sensor(0x212e, 0xAC49);
	write_cmos_sensor(0x2130, 0x0B8A);
	write_cmos_sensor(0x2132, 0x0C0A);
	write_cmos_sensor(0x2134, 0x40B2);
	write_cmos_sensor(0x2136, 0xE1BC);
	write_cmos_sensor(0x2138, 0x0B20);
	write_cmos_sensor(0x213a, 0x40B2);
	write_cmos_sensor(0x213c, 0xE1BC);
	write_cmos_sensor(0x213e, 0x0B8C);
	write_cmos_sensor(0x2140, 0x0C0A);
	write_cmos_sensor(0x2142, 0x40B2);
	write_cmos_sensor(0x2144, 0x0500);
	write_cmos_sensor(0x2146, 0x0B20);
	write_cmos_sensor(0x2148, 0x40B2);
	write_cmos_sensor(0x214a, 0x0500);
	write_cmos_sensor(0x214c, 0x0B9E);
	write_cmos_sensor(0x214e, 0x0C0A);
	write_cmos_sensor(0x2150, 0x43D2);
	write_cmos_sensor(0x2152, 0x0F82);
	write_cmos_sensor(0x2154, 0x0C3C);
	write_cmos_sensor(0x2156, 0x0C3C);
	write_cmos_sensor(0x2158, 0x0C3C);
	write_cmos_sensor(0x215a, 0x0C3C);
	write_cmos_sensor(0x215c, 0x421F);
	write_cmos_sensor(0x215e, 0x00A6);
	write_cmos_sensor(0x2160, 0x503F);
	write_cmos_sensor(0x2162, 0x07D0);
	write_cmos_sensor(0x2164, 0x3811);
	write_cmos_sensor(0x2166, 0x4F82);
	write_cmos_sensor(0x2168, 0x7100);
	write_cmos_sensor(0x216a, 0x0004);
	write_cmos_sensor(0x216c, 0x0C0D);
	write_cmos_sensor(0x216e, 0x0005);
	write_cmos_sensor(0x2170, 0x0C04);
	write_cmos_sensor(0x2172, 0x000D);
	write_cmos_sensor(0x2174, 0x0C09);
	write_cmos_sensor(0x2176, 0x003D);
	write_cmos_sensor(0x2178, 0x0C1D);
	write_cmos_sensor(0x217a, 0x003C);
	write_cmos_sensor(0x217c, 0x0C13);
	write_cmos_sensor(0x217e, 0x0004);
	write_cmos_sensor(0x2180, 0x0C09);
	write_cmos_sensor(0x2182, 0x0004);
	write_cmos_sensor(0x2184, 0x533F);
	write_cmos_sensor(0x2186, 0x37EF);
	write_cmos_sensor(0x2188, 0x40B2);
	write_cmos_sensor(0x218a, 0x0028);
	write_cmos_sensor(0x218c, 0x7000);
	write_cmos_sensor(0x218e, 0x43A2);
	write_cmos_sensor(0x2190, 0x8134);
	write_cmos_sensor(0x2192, 0xB3E2);
	write_cmos_sensor(0x2194, 0x00B4);
	write_cmos_sensor(0x2196, 0x2402);
	write_cmos_sensor(0x2198, 0x4392);
	write_cmos_sensor(0x219a, 0x8134);
	write_cmos_sensor(0x219c, 0x4328);
	write_cmos_sensor(0x219e, 0xB3D2);
	write_cmos_sensor(0x21a0, 0x00B4);
	write_cmos_sensor(0x21a2, 0x2002);
	write_cmos_sensor(0x21a4, 0x4030);
	write_cmos_sensor(0x21a6, 0xF63E);
	write_cmos_sensor(0x21a8, 0x4308);
	write_cmos_sensor(0x21aa, 0x403F);
	write_cmos_sensor(0x21ac, 0x002E);
	write_cmos_sensor(0x21ae, 0x12B0);
	write_cmos_sensor(0x21b0, 0xFE92);
	write_cmos_sensor(0x21b2, 0x4F82);
	write_cmos_sensor(0x21b4, 0x8110);
	write_cmos_sensor(0x21b6, 0x403F);
	write_cmos_sensor(0x21b8, 0x002F);
	write_cmos_sensor(0x21ba, 0x12B0);
	write_cmos_sensor(0x21bc, 0xFE92);
	write_cmos_sensor(0x21be, 0xF37F);
	write_cmos_sensor(0x21c0, 0x108F);
	write_cmos_sensor(0x21c2, 0xDF82);
	write_cmos_sensor(0x21c4, 0x8110);
	write_cmos_sensor(0x21c6, 0x930F);
	write_cmos_sensor(0x21c8, 0x242F);
	write_cmos_sensor(0x21ca, 0x4382);
	write_cmos_sensor(0x21cc, 0x8128);
	write_cmos_sensor(0x21ce, 0x421B);
	write_cmos_sensor(0x21d0, 0x8128);
	write_cmos_sensor(0x21d2, 0x421F);
	write_cmos_sensor(0x21d4, 0x8110);
	write_cmos_sensor(0x21d6, 0x5392);
	write_cmos_sensor(0x21d8, 0x8110);
	write_cmos_sensor(0x21da, 0x12B0);
	write_cmos_sensor(0x21dc, 0xFE92);
	write_cmos_sensor(0x21de, 0x4FCB);
	write_cmos_sensor(0x21e0, 0x8112);
	write_cmos_sensor(0x21e2, 0x5392);
	write_cmos_sensor(0x21e4, 0x8128);
	write_cmos_sensor(0x21e6, 0x92A2);
	write_cmos_sensor(0x21e8, 0x8128);
	write_cmos_sensor(0x21ea, 0x3BF1);
	write_cmos_sensor(0x21ec, 0x425D);
	write_cmos_sensor(0x21ee, 0x8115);
	write_cmos_sensor(0x21f0, 0x108D);
	write_cmos_sensor(0x21f2, 0x425F);
	write_cmos_sensor(0x21f4, 0x8114);
	write_cmos_sensor(0x21f6, 0xDF0D);
	write_cmos_sensor(0x21f8, 0x4D82);
	write_cmos_sensor(0x21fa, 0x8162);
	write_cmos_sensor(0x21fc, 0x425B);
	write_cmos_sensor(0x21fe, 0x8113);
	write_cmos_sensor(0x2200, 0x4B4E);
	write_cmos_sensor(0x2202, 0x108E);
	write_cmos_sensor(0x2204, 0x425C);
	write_cmos_sensor(0x2206, 0x8112);
	write_cmos_sensor(0x2208, 0x4C4F);
	write_cmos_sensor(0x220a, 0xDF0E);
	write_cmos_sensor(0x220c, 0x4E82);
	write_cmos_sensor(0x220e, 0x816A);
	write_cmos_sensor(0x2210, 0x930D);
	write_cmos_sensor(0x2212, 0x2002);
	write_cmos_sensor(0x2214, 0x930E);
	write_cmos_sensor(0x2216, 0x2408);
	write_cmos_sensor(0x2218, 0x934B);
	write_cmos_sensor(0x221a, 0x2403);
	write_cmos_sensor(0x221c, 0x4E8D);
	write_cmos_sensor(0x221e, 0x0000);
	write_cmos_sensor(0x2220, 0x3FD4);
	write_cmos_sensor(0x2222, 0x4CCD);
	write_cmos_sensor(0x2224, 0x0000);
	write_cmos_sensor(0x2226, 0x3FD1);
	write_cmos_sensor(0x2228, 0x40B2);
	write_cmos_sensor(0x222a, 0x0005);
	write_cmos_sensor(0x222c, 0x7320);
	write_cmos_sensor(0x222e, 0x4392);
	write_cmos_sensor(0x2230, 0x7326);
	write_cmos_sensor(0x2232, 0x12B0);
	write_cmos_sensor(0x2234, 0xF9C6);
	write_cmos_sensor(0x2236, 0x4392);
	write_cmos_sensor(0x2238, 0x731C);
	write_cmos_sensor(0x223a, 0x9382);
	write_cmos_sensor(0x223c, 0x8120);
	write_cmos_sensor(0x223e, 0x200A);
	write_cmos_sensor(0x2240, 0x0B00);
	write_cmos_sensor(0x2242, 0x7302);
	write_cmos_sensor(0x2244, 0x02BC);
	write_cmos_sensor(0x2246, 0x4382);
	write_cmos_sensor(0x2248, 0x7004);
	write_cmos_sensor(0x224a, 0x430F);
	write_cmos_sensor(0x224c, 0x12B0);
	write_cmos_sensor(0x224e, 0xF870);
	write_cmos_sensor(0x2250, 0x12B0);
	write_cmos_sensor(0x2252, 0xF9C6);
	write_cmos_sensor(0x2254, 0x4392);
	write_cmos_sensor(0x2256, 0x8160);
	write_cmos_sensor(0x2258, 0x4382);
	write_cmos_sensor(0x225a, 0x740E);
	write_cmos_sensor(0x225c, 0xB3E2);
	write_cmos_sensor(0x225e, 0x0080);
	write_cmos_sensor(0x2260, 0x2402);
	write_cmos_sensor(0x2262, 0x4392);
	write_cmos_sensor(0x2264, 0x740E);
	write_cmos_sensor(0x2266, 0x431F);
	write_cmos_sensor(0x2268, 0x12B0);
	write_cmos_sensor(0x226a, 0xF870);
	write_cmos_sensor(0x226c, 0x4392);
	write_cmos_sensor(0x226e, 0x7004);
	write_cmos_sensor(0x2270, 0x4882);
	write_cmos_sensor(0x2272, 0x7110);
	write_cmos_sensor(0x2274, 0x403F);
	write_cmos_sensor(0x2276, 0x732A);
	write_cmos_sensor(0x2278, 0x4F2E);
	write_cmos_sensor(0x227a, 0xF31E);
	write_cmos_sensor(0x227c, 0x4E82);
	write_cmos_sensor(0x227e, 0x8100);
	write_cmos_sensor(0x2280, 0x4F2F);
	write_cmos_sensor(0x2282, 0xF32F);
	write_cmos_sensor(0x2284, 0x4F82);
	write_cmos_sensor(0x2286, 0x8124);
	write_cmos_sensor(0x2288, 0xD21F);
	write_cmos_sensor(0x228a, 0x8122);
	write_cmos_sensor(0x228c, 0x4F82);
	write_cmos_sensor(0x228e, 0x811C);
	write_cmos_sensor(0x2290, 0x9382);
	write_cmos_sensor(0x2292, 0x810C);
	write_cmos_sensor(0x2294, 0x2403);
	write_cmos_sensor(0x2296, 0x9392);
	write_cmos_sensor(0x2298, 0x7110);
	write_cmos_sensor(0x229a, 0x2128);
	write_cmos_sensor(0x229c, 0x9392);
	write_cmos_sensor(0x229e, 0x7110);
	write_cmos_sensor(0x22a0, 0x2098);
	write_cmos_sensor(0x22a2, 0x0B00);
	write_cmos_sensor(0x22a4, 0x7302);
	write_cmos_sensor(0x22a6, 0x0032);
	write_cmos_sensor(0x22a8, 0x4382);
	write_cmos_sensor(0x22aa, 0x7004);
	write_cmos_sensor(0x22ac, 0x422F);
	write_cmos_sensor(0x22ae, 0x12B0);
	write_cmos_sensor(0x22b0, 0xF870);
	write_cmos_sensor(0x22b2, 0x0800);
	write_cmos_sensor(0x22b4, 0x7114);
	write_cmos_sensor(0x22b6, 0x403F);
	write_cmos_sensor(0x22b8, 0x0003);
	write_cmos_sensor(0x22ba, 0x12B0);
	write_cmos_sensor(0x22bc, 0xF870);
	write_cmos_sensor(0x22be, 0x425F);
	write_cmos_sensor(0x22c0, 0x0CA7);
	write_cmos_sensor(0x22c2, 0xF37F);
	write_cmos_sensor(0x22c4, 0x421A);
	write_cmos_sensor(0x22c6, 0x811A);
	write_cmos_sensor(0x22c8, 0x4F0C);
	write_cmos_sensor(0x22ca, 0x12B0);
	write_cmos_sensor(0x22cc, 0xFEB6);
	write_cmos_sensor(0x22ce, 0x4E0A);
	write_cmos_sensor(0x22d0, 0xC312);
	write_cmos_sensor(0x22d2, 0x100A);
	write_cmos_sensor(0x22d4, 0x110A);
	write_cmos_sensor(0x22d6, 0x110A);
	write_cmos_sensor(0x22d8, 0x110A);
	write_cmos_sensor(0x22da, 0x903A);
	write_cmos_sensor(0x22dc, 0x0010);
	write_cmos_sensor(0x22de, 0x2C02);
	write_cmos_sensor(0x22e0, 0x403A);
	write_cmos_sensor(0x22e2, 0x0010);
	write_cmos_sensor(0x22e4, 0x425F);
	write_cmos_sensor(0x22e6, 0x0CA6);
	write_cmos_sensor(0x22e8, 0xF37F);
	write_cmos_sensor(0x22ea, 0x5F0F);
	write_cmos_sensor(0x22ec, 0x5F0F);
	write_cmos_sensor(0x22ee, 0x5F0F);
	write_cmos_sensor(0x22f0, 0x5F0F);
	write_cmos_sensor(0x22f2, 0x4F0C);
	write_cmos_sensor(0x22f4, 0x12B0);
	write_cmos_sensor(0x22f6, 0xFED0);
	write_cmos_sensor(0x22f8, 0x4C82);
	write_cmos_sensor(0x22fa, 0x816C);
	write_cmos_sensor(0x22fc, 0x425F);
	write_cmos_sensor(0x22fe, 0x0C9C);
	write_cmos_sensor(0x2300, 0x4F4E);
	write_cmos_sensor(0x2302, 0x430F);
	write_cmos_sensor(0x2304, 0x4E0F);
	write_cmos_sensor(0x2306, 0x430E);
	write_cmos_sensor(0x2308, 0x421A);
	write_cmos_sensor(0x230a, 0x0C9A);
	write_cmos_sensor(0x230c, 0x430B);
	write_cmos_sensor(0x230e, 0xDE0A);
	write_cmos_sensor(0x2310, 0xDF0B);
	write_cmos_sensor(0x2312, 0x425F);
	write_cmos_sensor(0x2314, 0x0CA0);
	write_cmos_sensor(0x2316, 0x4F4E);
	write_cmos_sensor(0x2318, 0x430F);
	write_cmos_sensor(0x231a, 0x4E0F);
	write_cmos_sensor(0x231c, 0x430E);
	write_cmos_sensor(0x231e, 0x421D);
	write_cmos_sensor(0x2320, 0x0C9E);
	write_cmos_sensor(0x2322, 0xDD0E);
	write_cmos_sensor(0x2324, 0x5E0A);
	write_cmos_sensor(0x2326, 0x6F0B);
	write_cmos_sensor(0x2328, 0x421E);
	write_cmos_sensor(0x232a, 0x0CA2);
	write_cmos_sensor(0x232c, 0x521E);
	write_cmos_sensor(0x232e, 0x0CA4);
	write_cmos_sensor(0x2330, 0x4382);
	write_cmos_sensor(0x2332, 0x8116);
	write_cmos_sensor(0x2334, 0xB3D2);
	write_cmos_sensor(0x2336, 0x0C8E);
	write_cmos_sensor(0x2338, 0x2430);
	write_cmos_sensor(0x233a, 0xB3D2);
	write_cmos_sensor(0x233c, 0x0C80);
	write_cmos_sensor(0x233e, 0x242D);
	write_cmos_sensor(0x2340, 0x430F);
	write_cmos_sensor(0x2342, 0x1209);
	write_cmos_sensor(0x2344, 0x4E0C);
	write_cmos_sensor(0x2346, 0x4F0D);
	write_cmos_sensor(0x2348, 0x4A0E);
	write_cmos_sensor(0x234a, 0x4B0F);
	write_cmos_sensor(0x234c, 0x12B0);
	write_cmos_sensor(0x234e, 0xF8DE);
	write_cmos_sensor(0x2350, 0x4F09);
	write_cmos_sensor(0x2352, 0x403F);
	write_cmos_sensor(0x2354, 0x0003);
	write_cmos_sensor(0x2356, 0x12B0);
	write_cmos_sensor(0x2358, 0xF870);
	write_cmos_sensor(0x235a, 0x4982);
	write_cmos_sensor(0x235c, 0x8116);
	write_cmos_sensor(0x235e, 0x5321);
	write_cmos_sensor(0x2360, 0x90B2);
	write_cmos_sensor(0x2362, 0x0019);
	write_cmos_sensor(0x2364, 0x8166);
	write_cmos_sensor(0x2366, 0x2831);
	write_cmos_sensor(0x2368, 0x4982);
	write_cmos_sensor(0x236a, 0x8106);
	write_cmos_sensor(0x236c, 0x421F);
	write_cmos_sensor(0x236e, 0x8116);
	write_cmos_sensor(0x2370, 0x523F);
	write_cmos_sensor(0x2372, 0x110F);
	write_cmos_sensor(0x2374, 0x110F);
	write_cmos_sensor(0x2376, 0x110F);
	write_cmos_sensor(0x2378, 0x110F);
	write_cmos_sensor(0x237a, 0x4F82);
	write_cmos_sensor(0x237c, 0x8116);
	write_cmos_sensor(0x237e, 0x903F);
	write_cmos_sensor(0x2380, 0x001E);
	write_cmos_sensor(0x2382, 0x3821);
	write_cmos_sensor(0x2384, 0x503F);
	write_cmos_sensor(0x2386, 0xFFE2);
	write_cmos_sensor(0x2388, 0x4F82);
	write_cmos_sensor(0x238a, 0x8116);
	write_cmos_sensor(0x238c, 0x4F82);
	write_cmos_sensor(0x238e, 0x0CAC);
	write_cmos_sensor(0x2390, 0x4F82);
	write_cmos_sensor(0x2392, 0x0CAE);
	write_cmos_sensor(0x2394, 0x42D2);
	write_cmos_sensor(0x2396, 0x0C93);
	write_cmos_sensor(0x2398, 0x0C96);
	write_cmos_sensor(0x239a, 0xC3E2);
	write_cmos_sensor(0x239c, 0x0C81);
	write_cmos_sensor(0x239e, 0x4292);
	write_cmos_sensor(0x23a0, 0x8120);
	write_cmos_sensor(0x23a2, 0x8130);
	write_cmos_sensor(0x23a4, 0x421F);
	write_cmos_sensor(0x23a6, 0x812E);
	write_cmos_sensor(0x23a8, 0xD21F);
	write_cmos_sensor(0x23aa, 0x8120);
	write_cmos_sensor(0x23ac, 0x5F0F);
	write_cmos_sensor(0x23ae, 0xF03F);
	write_cmos_sensor(0x23b0, 0x000E);
	write_cmos_sensor(0x23b2, 0x4F82);
	write_cmos_sensor(0x23b4, 0x812E);
	write_cmos_sensor(0x23b6, 0x4382);
	write_cmos_sensor(0x23b8, 0x8120);
	write_cmos_sensor(0x23ba, 0x432F);
	write_cmos_sensor(0x23bc, 0x12B0);
	write_cmos_sensor(0x23be, 0xF870);
	write_cmos_sensor(0x23c0, 0x12B0);
	write_cmos_sensor(0x23c2, 0xF810);
	write_cmos_sensor(0x23c4, 0x3F65);
	write_cmos_sensor(0x23c6, 0x430F);
	write_cmos_sensor(0x23c8, 0x3FDF);
	write_cmos_sensor(0x23ca, 0x4292);
	write_cmos_sensor(0x23cc, 0x8106);
	write_cmos_sensor(0x23ce, 0x8116);
	write_cmos_sensor(0x23d0, 0x3FCD);
	write_cmos_sensor(0x23d2, 0x0B00);
	write_cmos_sensor(0x23d4, 0x7302);
	write_cmos_sensor(0x23d6, 0x0002);
	write_cmos_sensor(0x23d8, 0x0404);
	write_cmos_sensor(0x23da, 0x0C01);
	write_cmos_sensor(0x23dc, 0x0001);
	write_cmos_sensor(0x23de, 0x0C01);
	write_cmos_sensor(0x23e0, 0x0003);
	write_cmos_sensor(0x23e2, 0x0C01);
	write_cmos_sensor(0x23e4, 0x004B);
	write_cmos_sensor(0x23e6, 0x0C0A);
	write_cmos_sensor(0x23e8, 0x0243);
	write_cmos_sensor(0x23ea, 0x791F);
	write_cmos_sensor(0x23ec, 0x0C01);
	write_cmos_sensor(0x23ee, 0x0405);
	write_cmos_sensor(0x23f0, 0x0C41);
	write_cmos_sensor(0x23f2, 0x0647);
	write_cmos_sensor(0x23f4, 0x0C03);
	write_cmos_sensor(0x23f6, 0x0672);
	write_cmos_sensor(0x23f8, 0x0C01);
	write_cmos_sensor(0x23fa, 0x0670);
	write_cmos_sensor(0x23fc, 0x0C01);
	write_cmos_sensor(0x23fe, 0x0778);
	write_cmos_sensor(0x2400, 0x0C0B);
	write_cmos_sensor(0x2402, 0x067C);
	write_cmos_sensor(0x2404, 0x0C07);
	write_cmos_sensor(0x2406, 0x0686);
	write_cmos_sensor(0x2408, 0x0C01);
	write_cmos_sensor(0x240a, 0x066C);
	write_cmos_sensor(0x240c, 0x0C7D);
	write_cmos_sensor(0x240e, 0x0684);
	write_cmos_sensor(0x2410, 0x0C8E);
	write_cmos_sensor(0x2412, 0x0203);
	write_cmos_sensor(0x2414, 0x4E1F);
	write_cmos_sensor(0x2416, 0x0393);
	write_cmos_sensor(0x2418, 0x7A1F);
	write_cmos_sensor(0x241a, 0x0C0B);
	write_cmos_sensor(0x241c, 0x0083);
	write_cmos_sensor(0x241e, 0x0C55);
	write_cmos_sensor(0x2420, 0x0686);
	write_cmos_sensor(0x2422, 0x0C01);
	write_cmos_sensor(0x2424, 0x0664);
	write_cmos_sensor(0x2426, 0x0CFF);
	write_cmos_sensor(0x2428, 0x0CC2);
	write_cmos_sensor(0x242a, 0x0684);
	write_cmos_sensor(0x242c, 0x4392);
	write_cmos_sensor(0x242e, 0x7004);
	write_cmos_sensor(0x2430, 0x430F);
	write_cmos_sensor(0x2432, 0x9382);
	write_cmos_sensor(0x2434, 0x8160);
	write_cmos_sensor(0x2436, 0x2001);
	write_cmos_sensor(0x2438, 0x431F);
	write_cmos_sensor(0x243a, 0x4F82);
	write_cmos_sensor(0x243c, 0x8160);
	write_cmos_sensor(0x243e, 0x12B0);
	write_cmos_sensor(0x2440, 0xF810);
	write_cmos_sensor(0x2442, 0x930F);
	write_cmos_sensor(0x2444, 0x2405);
	write_cmos_sensor(0x2446, 0x4292);
	write_cmos_sensor(0x2448, 0x8120);
	write_cmos_sensor(0x244a, 0x8130);
	write_cmos_sensor(0x244c, 0x4382);
	write_cmos_sensor(0x244e, 0x8120);
	write_cmos_sensor(0x2450, 0x9382);
	write_cmos_sensor(0x2452, 0x8160);
	write_cmos_sensor(0x2454, 0x242C);
	write_cmos_sensor(0x2456, 0x9382);
	write_cmos_sensor(0x2458, 0x8108);
	write_cmos_sensor(0x245a, 0x2003);
	write_cmos_sensor(0x245c, 0x9382);
	write_cmos_sensor(0x245e, 0x812C);
	write_cmos_sensor(0x2460, 0x2416);
	write_cmos_sensor(0x2462, 0x0B00);
	write_cmos_sensor(0x2464, 0x7302);
	write_cmos_sensor(0x2466, 0x04FE);
	write_cmos_sensor(0x2468, 0x0660);
	write_cmos_sensor(0x246a, 0x0C03);
	write_cmos_sensor(0x246c, 0x0509);
	write_cmos_sensor(0x246e, 0x0C01);
	write_cmos_sensor(0x2470, 0x00BB);
	write_cmos_sensor(0x2472, 0x0C01);
	write_cmos_sensor(0x2474, 0x040A);
	write_cmos_sensor(0x2476, 0x9382);
	write_cmos_sensor(0x2478, 0x8172);
	write_cmos_sensor(0x247a, 0x2003);
	write_cmos_sensor(0x247c, 0x930F);
	write_cmos_sensor(0x247e, 0x2708);
	write_cmos_sensor(0x2480, 0x3EDC);
	write_cmos_sensor(0x2482, 0x43C2);
	write_cmos_sensor(0x2484, 0x0A80);
	write_cmos_sensor(0x2486, 0x0B00);
	write_cmos_sensor(0x2488, 0x7302);
	write_cmos_sensor(0x248a, 0xFFF0);
	write_cmos_sensor(0x248c, 0x3F01);
	write_cmos_sensor(0x248e, 0x0B00);
	write_cmos_sensor(0x2490, 0x7302);
	write_cmos_sensor(0x2492, 0x0580);
	write_cmos_sensor(0x2494, 0x0407);
	write_cmos_sensor(0x2496, 0x0C02);
	write_cmos_sensor(0x2498, 0x033D);
	write_cmos_sensor(0x249a, 0xB810);
	write_cmos_sensor(0x249c, 0x0C01);
	write_cmos_sensor(0x249e, 0x003C);
	write_cmos_sensor(0x24a0, 0x0C01);
	write_cmos_sensor(0x24a2, 0x0004);
	write_cmos_sensor(0x24a4, 0x0C01);
	write_cmos_sensor(0x24a6, 0x06A5);
	write_cmos_sensor(0x24a8, 0x0C03);
	write_cmos_sensor(0x24aa, 0x06A1);
	write_cmos_sensor(0x24ac, 0x3FE4);
	write_cmos_sensor(0x24ae, 0x9382);
	write_cmos_sensor(0x24b0, 0x8108);
	write_cmos_sensor(0x24b2, 0x2003);
	write_cmos_sensor(0x24b4, 0x9382);
	write_cmos_sensor(0x24b6, 0x812C);
	write_cmos_sensor(0x24b8, 0x240B);
	write_cmos_sensor(0x24ba, 0x0B00);
	write_cmos_sensor(0x24bc, 0x7302);
	write_cmos_sensor(0x24be, 0x04FE);
	write_cmos_sensor(0x24c0, 0x0660);
	write_cmos_sensor(0x24c2, 0x0C03);
	write_cmos_sensor(0x24c4, 0x0508);
	write_cmos_sensor(0x24c6, 0x0C01);
	write_cmos_sensor(0x24c8, 0x0083);
	write_cmos_sensor(0x24ca, 0x0C01);
	write_cmos_sensor(0x24cc, 0x06A1);
	write_cmos_sensor(0x24ce, 0x3FD3);
	write_cmos_sensor(0x24d0, 0x0B00);
	write_cmos_sensor(0x24d2, 0x7302);
	write_cmos_sensor(0x24d4, 0x0580);
	write_cmos_sensor(0x24d6, 0x0406);
	write_cmos_sensor(0x24d8, 0x0C02);
	write_cmos_sensor(0x24da, 0x0305);
	write_cmos_sensor(0x24dc, 0xB810);
	write_cmos_sensor(0x24de, 0x0C01);
	write_cmos_sensor(0x24e0, 0x0004);
	write_cmos_sensor(0x24e2, 0x0C03);
	write_cmos_sensor(0x24e4, 0x06A5);
	write_cmos_sensor(0x24e6, 0x0C03);
	write_cmos_sensor(0x24e8, 0x06A1);
	write_cmos_sensor(0x24ea, 0x3FC5);
	write_cmos_sensor(0x24ec, 0x0B00);
	write_cmos_sensor(0x24ee, 0x7302);
	write_cmos_sensor(0x24f0, 0x0002);
	write_cmos_sensor(0x24f2, 0x040B);
	write_cmos_sensor(0x24f4, 0x0C1F);
	write_cmos_sensor(0x24f6, 0x06A1);
	write_cmos_sensor(0x24f8, 0x0C05);
	write_cmos_sensor(0x24fa, 0x0001);
	write_cmos_sensor(0x24fc, 0x0C01);
	write_cmos_sensor(0x24fe, 0x0003);
	write_cmos_sensor(0x2500, 0x0C01);
	write_cmos_sensor(0x2502, 0x004B);
	write_cmos_sensor(0x2504, 0x0C17);
	write_cmos_sensor(0x2506, 0x0043);
	write_cmos_sensor(0x2508, 0x0C01);
	write_cmos_sensor(0x250a, 0x0672);
	write_cmos_sensor(0x250c, 0x0243);
	write_cmos_sensor(0x250e, 0x79DF);
	write_cmos_sensor(0x2510, 0x0C3B);
	write_cmos_sensor(0x2512, 0x0647);
	write_cmos_sensor(0x2514, 0x0C05);
	write_cmos_sensor(0x2516, 0x0672);
	write_cmos_sensor(0x2518, 0x0C01);
	write_cmos_sensor(0x251a, 0x0670);
	write_cmos_sensor(0x251c, 0x0C0D);
	write_cmos_sensor(0x251e, 0x0778);
	write_cmos_sensor(0x2520, 0x0C0B);
	write_cmos_sensor(0x2522, 0x067C);
	write_cmos_sensor(0x2524, 0x0C13);
	write_cmos_sensor(0x2526, 0x0686);
	write_cmos_sensor(0x2528, 0x0C01);
	write_cmos_sensor(0x252a, 0x066C);
	write_cmos_sensor(0x252c, 0x0C67);
	write_cmos_sensor(0x252e, 0x067C);
	write_cmos_sensor(0x2530, 0x0C01);
	write_cmos_sensor(0x2532, 0x0003);
	write_cmos_sensor(0x2534, 0x0393);
	write_cmos_sensor(0x2536, 0x7A1F);
	write_cmos_sensor(0x2538, 0x0C17);
	write_cmos_sensor(0x253a, 0x0083);
	write_cmos_sensor(0x253c, 0x0C6F);
	write_cmos_sensor(0x253e, 0x0686);
	write_cmos_sensor(0x2540, 0x0C01);
	write_cmos_sensor(0x2542, 0x0664);
	write_cmos_sensor(0x2544, 0x4392);
	write_cmos_sensor(0x2546, 0x7004);
	write_cmos_sensor(0x2548, 0x430F);
	write_cmos_sensor(0x254a, 0x9382);
	write_cmos_sensor(0x254c, 0x8160);
	write_cmos_sensor(0x254e, 0x2001);
	write_cmos_sensor(0x2550, 0x431F);
	write_cmos_sensor(0x2552, 0x4F82);
	write_cmos_sensor(0x2554, 0x8160);
	write_cmos_sensor(0x2556, 0x930F);
	write_cmos_sensor(0x2558, 0x2468);
	write_cmos_sensor(0x255a, 0x0B00);
	write_cmos_sensor(0x255c, 0x7302);
	write_cmos_sensor(0x255e, 0x033A);
	write_cmos_sensor(0x2560, 0x0674);
	write_cmos_sensor(0x2562, 0x0C02);
	write_cmos_sensor(0x2564, 0x0339);
	write_cmos_sensor(0x2566, 0xB810);
	write_cmos_sensor(0x2568, 0x0C01);
	write_cmos_sensor(0x256a, 0x003C);
	write_cmos_sensor(0x256c, 0x0C01);
	write_cmos_sensor(0x256e, 0x0004);
	write_cmos_sensor(0x2570, 0x0B00);
	write_cmos_sensor(0x2572, 0x7302);
	write_cmos_sensor(0x2574, 0x0366);
	write_cmos_sensor(0x2576, 0x040C);
	write_cmos_sensor(0x2578, 0x0C25);
	write_cmos_sensor(0x257a, 0x0001);
	write_cmos_sensor(0x257c, 0x0C01);
	write_cmos_sensor(0x257e, 0x0003);
	write_cmos_sensor(0x2580, 0x0C01);
	write_cmos_sensor(0x2582, 0x004B);
	write_cmos_sensor(0x2584, 0x0C17);
	write_cmos_sensor(0x2586, 0x0043);
	write_cmos_sensor(0x2588, 0x0C01);
	write_cmos_sensor(0x258a, 0x0672);
	write_cmos_sensor(0x258c, 0x0243);
	write_cmos_sensor(0x258e, 0x79DF);
	write_cmos_sensor(0x2590, 0x0C3B);
	write_cmos_sensor(0x2592, 0x0647);
	write_cmos_sensor(0x2594, 0x0C05);
	write_cmos_sensor(0x2596, 0x0672);
	write_cmos_sensor(0x2598, 0x0C01);
	write_cmos_sensor(0x259a, 0x0670);
	write_cmos_sensor(0x259c, 0x0C0D);
	write_cmos_sensor(0x259e, 0x0778);
	write_cmos_sensor(0x25a0, 0x0C0B);
	write_cmos_sensor(0x25a2, 0x067C);
	write_cmos_sensor(0x25a4, 0x0C13);
	write_cmos_sensor(0x25a6, 0x0686);
	write_cmos_sensor(0x25a8, 0x0C01);
	write_cmos_sensor(0x25aa, 0x066C);
	write_cmos_sensor(0x25ac, 0x0C67);
	write_cmos_sensor(0x25ae, 0x067C);
	write_cmos_sensor(0x25b0, 0x0C01);
	write_cmos_sensor(0x25b2, 0x0003);
	write_cmos_sensor(0x25b4, 0x0393);
	write_cmos_sensor(0x25b6, 0x7A1F);
	write_cmos_sensor(0x25b8, 0x0C17);
	write_cmos_sensor(0x25ba, 0x0083);
	write_cmos_sensor(0x25bc, 0x0C6F);
	write_cmos_sensor(0x25be, 0x0686);
	write_cmos_sensor(0x25c0, 0x0C01);
	write_cmos_sensor(0x25c2, 0x0664);
	write_cmos_sensor(0x25c4, 0x12B0);
	write_cmos_sensor(0x25c6, 0xF810);
	write_cmos_sensor(0x25c8, 0x930F);
	write_cmos_sensor(0x25ca, 0x2405);
	write_cmos_sensor(0x25cc, 0x4292);
	write_cmos_sensor(0x25ce, 0x8120);
	write_cmos_sensor(0x25d0, 0x8130);
	write_cmos_sensor(0x25d2, 0x4382);
	write_cmos_sensor(0x25d4, 0x8120);
	write_cmos_sensor(0x25d6, 0x9382);
	write_cmos_sensor(0x25d8, 0x8160);
	write_cmos_sensor(0x25da, 0x2419);
	write_cmos_sensor(0x25dc, 0x0B00);
	write_cmos_sensor(0x25de, 0x7302);
	write_cmos_sensor(0x25e0, 0x069E);
	write_cmos_sensor(0x25e2, 0x0674);
	write_cmos_sensor(0x25e4, 0x0C02);
	write_cmos_sensor(0x25e6, 0x0339);
	write_cmos_sensor(0x25e8, 0xB810);
	write_cmos_sensor(0x25ea, 0x0C01);
	write_cmos_sensor(0x25ec, 0x003C);
	write_cmos_sensor(0x25ee, 0x0C01);
	write_cmos_sensor(0x25f0, 0x0004);
	write_cmos_sensor(0x25f2, 0x0C15);
	write_cmos_sensor(0x25f4, 0x06A5);
	write_cmos_sensor(0x25f6, 0x0C05);
	write_cmos_sensor(0x25f8, 0x06A1);
	write_cmos_sensor(0x25fa, 0x9382);
	write_cmos_sensor(0x25fc, 0x8172);
	write_cmos_sensor(0x25fe, 0x273E);
	write_cmos_sensor(0x2600, 0x43C2);
	write_cmos_sensor(0x2602, 0x0A80);
	write_cmos_sensor(0x2604, 0x0B00);
	write_cmos_sensor(0x2606, 0x7302);
	write_cmos_sensor(0x2608, 0xFFF0);
	write_cmos_sensor(0x260a, 0x4030);
	write_cmos_sensor(0x260c, 0xF290);
	write_cmos_sensor(0x260e, 0x0B00);
	write_cmos_sensor(0x2610, 0x7302);
	write_cmos_sensor(0x2612, 0x069E);
	write_cmos_sensor(0x2614, 0x0674);
	write_cmos_sensor(0x2616, 0x0C02);
	write_cmos_sensor(0x2618, 0x0301);
	write_cmos_sensor(0x261a, 0xB810);
	write_cmos_sensor(0x261c, 0x0C01);
	write_cmos_sensor(0x261e, 0x0004);
	write_cmos_sensor(0x2620, 0x0C17);
	write_cmos_sensor(0x2622, 0x06A5);
	write_cmos_sensor(0x2624, 0x0C05);
	write_cmos_sensor(0x2626, 0x06A1);
	write_cmos_sensor(0x2628, 0x3FE8);
	write_cmos_sensor(0x262a, 0x0B00);
	write_cmos_sensor(0x262c, 0x7302);
	write_cmos_sensor(0x262e, 0x033A);
	write_cmos_sensor(0x2630, 0x0674);
	write_cmos_sensor(0x2632, 0x0C02);
	write_cmos_sensor(0x2634, 0x0301);
	write_cmos_sensor(0x2636, 0xB810);
	write_cmos_sensor(0x2638, 0x0C01);
	write_cmos_sensor(0x263a, 0x0004);
	write_cmos_sensor(0x263c, 0x3F99);
	write_cmos_sensor(0x263e, 0xB3E2);
	write_cmos_sensor(0x2640, 0x00B4);
	write_cmos_sensor(0x2642, 0x2002);
	write_cmos_sensor(0x2644, 0x4030);
	write_cmos_sensor(0x2646, 0xF1AA);
	write_cmos_sensor(0x2648, 0x4318);
	write_cmos_sensor(0x264a, 0x4030);
	write_cmos_sensor(0x264c, 0xF1AA);
	write_cmos_sensor(0x264e, 0x4392);
	write_cmos_sensor(0x2650, 0x760E);
	write_cmos_sensor(0x2652, 0x425F);
	write_cmos_sensor(0x2654, 0x0198);
	write_cmos_sensor(0x2656, 0xF37F);
	write_cmos_sensor(0x2658, 0x930F);
	write_cmos_sensor(0x265a, 0x2005);
	write_cmos_sensor(0x265c, 0x43C2);
	write_cmos_sensor(0x265e, 0x0A80);
	write_cmos_sensor(0x2660, 0x0B00);
	write_cmos_sensor(0x2662, 0x7302);
	write_cmos_sensor(0x2664, 0xFFF0);
	write_cmos_sensor(0x2666, 0x9382);
	write_cmos_sensor(0x2668, 0x760C);
	write_cmos_sensor(0x266a, 0x2002);
	write_cmos_sensor(0x266c, 0x0C64);
	write_cmos_sensor(0x266e, 0x3FF1);
	write_cmos_sensor(0x2670, 0x4F82);
	write_cmos_sensor(0x2672, 0x812A);
	write_cmos_sensor(0x2674, 0x421F);
	write_cmos_sensor(0x2676, 0x760A);
	write_cmos_sensor(0x2678, 0x932F);
	write_cmos_sensor(0x267a, 0x2013);
	write_cmos_sensor(0x267c, 0x4292);
	write_cmos_sensor(0x267e, 0x018A);
	write_cmos_sensor(0x2680, 0x8140);
	write_cmos_sensor(0x2682, 0x12B0);
	write_cmos_sensor(0x2684, 0xFE80);
	write_cmos_sensor(0x2686, 0x40B2);
	write_cmos_sensor(0x2688, 0x0005);
	write_cmos_sensor(0x268a, 0x7600);
	write_cmos_sensor(0x268c, 0x4382);
	write_cmos_sensor(0x268e, 0x7602);
	write_cmos_sensor(0x2690, 0x0262);
	write_cmos_sensor(0x2692, 0x0000);
	write_cmos_sensor(0x2694, 0x0222);
	write_cmos_sensor(0x2696, 0x0000);
	write_cmos_sensor(0x2698, 0x0262);
	write_cmos_sensor(0x269a, 0x0000);
	write_cmos_sensor(0x269c, 0x0260);
	write_cmos_sensor(0x269e, 0x0000);
	write_cmos_sensor(0x26a0, 0x3FD6);
	write_cmos_sensor(0x26a2, 0x903F);
	write_cmos_sensor(0x26a4, 0x0003);
	write_cmos_sensor(0x26a6, 0x287F);
	write_cmos_sensor(0x26a8, 0x903F);
	write_cmos_sensor(0x26aa, 0x0102);
	write_cmos_sensor(0x26ac, 0x2072);
	write_cmos_sensor(0x26ae, 0x43C2);
	write_cmos_sensor(0x26b0, 0x018C);
	write_cmos_sensor(0x26b2, 0x425F);
	write_cmos_sensor(0x26b4, 0x0186);
	write_cmos_sensor(0x26b6, 0x4F48);
	write_cmos_sensor(0x26b8, 0x93D2);
	write_cmos_sensor(0x26ba, 0x018F);
	write_cmos_sensor(0x26bc, 0x2468);
	write_cmos_sensor(0x26be, 0x425F);
	write_cmos_sensor(0x26c0, 0x018F);
	write_cmos_sensor(0x26c2, 0x4F4A);
	write_cmos_sensor(0x26c4, 0x4382);
	write_cmos_sensor(0x26c6, 0x8128);
	write_cmos_sensor(0x26c8, 0x430B);
	write_cmos_sensor(0x26ca, 0x421C);
	write_cmos_sensor(0x26cc, 0x8102);
	write_cmos_sensor(0x26ce, 0x4219);
	write_cmos_sensor(0x26d0, 0x8140);
	write_cmos_sensor(0x26d2, 0x4B07);
	write_cmos_sensor(0x26d4, 0x431F);
	write_cmos_sensor(0x26d6, 0x4B0E);
	write_cmos_sensor(0x26d8, 0x930E);
	write_cmos_sensor(0x26da, 0x2403);
	write_cmos_sensor(0x26dc, 0x5F0F);
	write_cmos_sensor(0x26de, 0x831E);
	write_cmos_sensor(0x26e0, 0x23FD);
	write_cmos_sensor(0x26e2, 0xF80F);
	write_cmos_sensor(0x26e4, 0x2448);
	write_cmos_sensor(0x26e6, 0x430C);
	write_cmos_sensor(0x26e8, 0x931A);
	write_cmos_sensor(0x26ea, 0x282F);
	write_cmos_sensor(0x26ec, 0x490D);
	write_cmos_sensor(0x26ee, 0x4B0E);
	write_cmos_sensor(0x26f0, 0x430F);
	write_cmos_sensor(0x26f2, 0x4D82);
	write_cmos_sensor(0x26f4, 0x7600);
	write_cmos_sensor(0x26f6, 0x4E82);
	write_cmos_sensor(0x26f8, 0x7602);
	write_cmos_sensor(0x26fa, 0x4882);
	write_cmos_sensor(0x26fc, 0x7604);
	write_cmos_sensor(0x26fe, 0x0264);
	write_cmos_sensor(0x2700, 0x0000);
	write_cmos_sensor(0x2702, 0x0224);
	write_cmos_sensor(0x2704, 0x0000);
	write_cmos_sensor(0x2706, 0x0264);
	write_cmos_sensor(0x2708, 0x0000);
	write_cmos_sensor(0x270a, 0x0260);
	write_cmos_sensor(0x270c, 0x0000);
	write_cmos_sensor(0x270e, 0x0268);
	write_cmos_sensor(0x2710, 0x0000);
	write_cmos_sensor(0x2712, 0x0C5A);
	write_cmos_sensor(0x2714, 0x02E8);
	write_cmos_sensor(0x2716, 0x0000);
	write_cmos_sensor(0x2718, 0x0CB5);
	write_cmos_sensor(0x271a, 0x02A8);
	write_cmos_sensor(0x271c, 0x0000);
	write_cmos_sensor(0x271e, 0x0CB5);
	write_cmos_sensor(0x2720, 0x0CB5);
	write_cmos_sensor(0x2722, 0x0CB5);
	write_cmos_sensor(0x2724, 0x0CB5);
	write_cmos_sensor(0x2726, 0x0CB5);
	write_cmos_sensor(0x2728, 0x0CB5);
	write_cmos_sensor(0x272a, 0x0CB5);
	write_cmos_sensor(0x272c, 0x0CB5);
	write_cmos_sensor(0x272e, 0x0C00);
	write_cmos_sensor(0x2730, 0x02E8);
	write_cmos_sensor(0x2732, 0x0000);
	write_cmos_sensor(0x2734, 0x0CB5);
	write_cmos_sensor(0x2736, 0x0268);
	write_cmos_sensor(0x2738, 0x0000);
	write_cmos_sensor(0x273a, 0x0C5A);
	write_cmos_sensor(0x273c, 0x0260);
	write_cmos_sensor(0x273e, 0x0000);
	write_cmos_sensor(0x2740, 0x0C5A);
	write_cmos_sensor(0x2742, 0x531F);
	write_cmos_sensor(0x2744, 0x9A0F);
	write_cmos_sensor(0x2746, 0x2BD5);
	write_cmos_sensor(0x2748, 0x4F0C);
	write_cmos_sensor(0x274a, 0x4B82);
	write_cmos_sensor(0x274c, 0x7602);
	write_cmos_sensor(0x274e, 0x4982);
	write_cmos_sensor(0x2750, 0x7600);
	write_cmos_sensor(0x2752, 0x0270);
	write_cmos_sensor(0x2754, 0x0000);
	write_cmos_sensor(0x2756, 0x0C1B);
	write_cmos_sensor(0x2758, 0x0270);
	write_cmos_sensor(0x275a, 0x0001);
	write_cmos_sensor(0x275c, 0x421F);
	write_cmos_sensor(0x275e, 0x7606);
	write_cmos_sensor(0x2760, 0x4F4E);
	write_cmos_sensor(0x2762, 0x431F);
	write_cmos_sensor(0x2764, 0x470D);
	write_cmos_sensor(0x2766, 0x930D);
	write_cmos_sensor(0x2768, 0x2403);
	write_cmos_sensor(0x276a, 0x5F0F);
	write_cmos_sensor(0x276c, 0x831D);
	write_cmos_sensor(0x276e, 0x23FD);
	write_cmos_sensor(0x2770, 0xFF4E);
	write_cmos_sensor(0x2772, 0xDEC2);
	write_cmos_sensor(0x2774, 0x018C);
	write_cmos_sensor(0x2776, 0x531B);
	write_cmos_sensor(0x2778, 0x923B);
	write_cmos_sensor(0x277a, 0x3BAB);
	write_cmos_sensor(0x277c, 0x4C82);
	write_cmos_sensor(0x277e, 0x8102);
	write_cmos_sensor(0x2780, 0x4B82);
	write_cmos_sensor(0x2782, 0x8128);
	write_cmos_sensor(0x2784, 0x0260);
	write_cmos_sensor(0x2786, 0x0000);
	write_cmos_sensor(0x2788, 0x5392);
	write_cmos_sensor(0x278a, 0x8140);
	write_cmos_sensor(0x278c, 0x3F60);
	write_cmos_sensor(0x278e, 0x432A);
	write_cmos_sensor(0x2790, 0x3F99);
	write_cmos_sensor(0x2792, 0x903F);
	write_cmos_sensor(0x2794, 0x0201);
	write_cmos_sensor(0x2796, 0x235B);
	write_cmos_sensor(0x2798, 0x5392);
	write_cmos_sensor(0x279a, 0x8140);
	write_cmos_sensor(0x279c, 0x421F);
	write_cmos_sensor(0x279e, 0x8140);
	write_cmos_sensor(0x27a0, 0x12B0);
	write_cmos_sensor(0x27a2, 0xFE92);
	write_cmos_sensor(0x27a4, 0x3F54);
	write_cmos_sensor(0x27a6, 0x931F);
	write_cmos_sensor(0x27a8, 0x2352);
	write_cmos_sensor(0x27aa, 0x12B0);
	write_cmos_sensor(0x27ac, 0xFE80);
	write_cmos_sensor(0x27ae, 0x4292);
	write_cmos_sensor(0x27b0, 0x018A);
	write_cmos_sensor(0x27b2, 0x8140);
	write_cmos_sensor(0x27b4, 0x3FF3);
	write_cmos_sensor(0x27b6, 0x403F);
	write_cmos_sensor(0x27b8, 0x0B80);
	write_cmos_sensor(0x27ba, 0x4F2E);
	write_cmos_sensor(0x27bc, 0xF03E);
	write_cmos_sensor(0x27be, 0x7FFF);
	write_cmos_sensor(0x27c0, 0x4E82);
	write_cmos_sensor(0x27c2, 0x0B20);
	write_cmos_sensor(0x27c4, 0xF0BF);
	write_cmos_sensor(0x27c6, 0x7FFF);
	write_cmos_sensor(0x27c8, 0x0000);
	write_cmos_sensor(0x27ca, 0x0C0A);
	write_cmos_sensor(0x27cc, 0x40B2);
	write_cmos_sensor(0x27ce, 0x8084);
	write_cmos_sensor(0x27d0, 0x0B20);
	write_cmos_sensor(0x27d2, 0x40B2);
	write_cmos_sensor(0x27d4, 0x8084);
	write_cmos_sensor(0x27d6, 0x0B88);
	write_cmos_sensor(0x27d8, 0x0C0A);
	write_cmos_sensor(0x27da, 0x40B2);
	write_cmos_sensor(0x27dc, 0xAC48);
	write_cmos_sensor(0x27de, 0x0B20);
	write_cmos_sensor(0x27e0, 0x40B2);
	write_cmos_sensor(0x27e2, 0xAC48);
	write_cmos_sensor(0x27e4, 0x0B8A);
	write_cmos_sensor(0x27e6, 0x0C0A);
	write_cmos_sensor(0x27e8, 0x40B2);
	write_cmos_sensor(0x27ea, 0x01BC);
	write_cmos_sensor(0x27ec, 0x0B20);
	write_cmos_sensor(0x27ee, 0x40B2);
	write_cmos_sensor(0x27f0, 0x01BC);
	write_cmos_sensor(0x27f2, 0x0B8C);
	write_cmos_sensor(0x27f4, 0x0C0A);
	write_cmos_sensor(0x27f6, 0x40B2);
	write_cmos_sensor(0x27f8, 0x0500);
	write_cmos_sensor(0x27fa, 0x0B20);
	write_cmos_sensor(0x27fc, 0x40B2);
	write_cmos_sensor(0x27fe, 0x0500);
	write_cmos_sensor(0x2800, 0x0B9E);
	write_cmos_sensor(0x2802, 0x0C0A);
	write_cmos_sensor(0x2804, 0x43C2);
	write_cmos_sensor(0x2806, 0x003F);
	write_cmos_sensor(0x2808, 0x4030);
	write_cmos_sensor(0x280a, 0xF040);
	write_cmos_sensor(0x280c, 0x4030);
	write_cmos_sensor(0x280e, 0xFEB2);
	write_cmos_sensor(0x2810, 0xE3B2);
	write_cmos_sensor(0x2812, 0x740E);
	write_cmos_sensor(0x2814, 0x425F);
	write_cmos_sensor(0x2816, 0x0198);
	write_cmos_sensor(0x2818, 0xF37F);
	write_cmos_sensor(0x281a, 0x4F82);
	write_cmos_sensor(0x281c, 0x812A);
	write_cmos_sensor(0x281e, 0x930F);
	write_cmos_sensor(0x2820, 0x2005);
	write_cmos_sensor(0x2822, 0x93C2);
	write_cmos_sensor(0x2824, 0x0A82);
	write_cmos_sensor(0x2826, 0x2402);
	write_cmos_sensor(0x2828, 0x4392);
	write_cmos_sensor(0x282a, 0x8172);
	write_cmos_sensor(0x282c, 0x9382);
	write_cmos_sensor(0x282e, 0x812A);
	write_cmos_sensor(0x2830, 0x2002);
	write_cmos_sensor(0x2832, 0x4392);
	write_cmos_sensor(0x2834, 0x8104);
	write_cmos_sensor(0x2836, 0x421F);
	write_cmos_sensor(0x2838, 0x710E);
	write_cmos_sensor(0x283a, 0x93A2);
	write_cmos_sensor(0x283c, 0x7110);
	write_cmos_sensor(0x283e, 0x2411);
	write_cmos_sensor(0x2840, 0x9382);
	write_cmos_sensor(0x2842, 0x710E);
	write_cmos_sensor(0x2844, 0x240C);
	write_cmos_sensor(0x2846, 0x5292);
	write_cmos_sensor(0x2848, 0x8134);
	write_cmos_sensor(0x284a, 0x7110);
	write_cmos_sensor(0x284c, 0x4382);
	write_cmos_sensor(0x284e, 0x740E);
	write_cmos_sensor(0x2850, 0xB3E2);
	write_cmos_sensor(0x2852, 0x0080);
	write_cmos_sensor(0x2854, 0x2402);
	write_cmos_sensor(0x2856, 0x4392);
	write_cmos_sensor(0x2858, 0x740E);
	write_cmos_sensor(0x285a, 0x4392);
	write_cmos_sensor(0x285c, 0x8160);
	write_cmos_sensor(0x285e, 0x430F);
	write_cmos_sensor(0x2860, 0x4130);
	write_cmos_sensor(0x2862, 0xF31F);
	write_cmos_sensor(0x2864, 0x27ED);
	write_cmos_sensor(0x2866, 0x40B2);
	write_cmos_sensor(0x2868, 0x0003);
	write_cmos_sensor(0x286a, 0x7110);
	write_cmos_sensor(0x286c, 0x431F);
	write_cmos_sensor(0x286e, 0x4130);
	write_cmos_sensor(0x2870, 0x4F0E);
	write_cmos_sensor(0x2872, 0x421D);
	write_cmos_sensor(0x2874, 0x8104);
	write_cmos_sensor(0x2876, 0x425F);
	write_cmos_sensor(0x2878, 0x0198);
	write_cmos_sensor(0x287a, 0xF37F);
	write_cmos_sensor(0x287c, 0x903E);
	write_cmos_sensor(0x287e, 0x0003);
	write_cmos_sensor(0x2880, 0x2405);
	write_cmos_sensor(0x2882, 0x931E);
	write_cmos_sensor(0x2884, 0x2403);
	write_cmos_sensor(0x2886, 0x0B00);
	write_cmos_sensor(0x2888, 0x7302);
	write_cmos_sensor(0x288a, 0x0384);
	write_cmos_sensor(0x288c, 0x930F);
	write_cmos_sensor(0x288e, 0x2402);
	write_cmos_sensor(0x2890, 0x930D);
	write_cmos_sensor(0x2892, 0x2403);
	write_cmos_sensor(0x2894, 0x9392);
	write_cmos_sensor(0x2896, 0x7110);
	write_cmos_sensor(0x2898, 0x2018);
	write_cmos_sensor(0x289a, 0x9382);
	write_cmos_sensor(0x289c, 0x7308);
	write_cmos_sensor(0x289e, 0x2402);
	write_cmos_sensor(0x28a0, 0x930E);
	write_cmos_sensor(0x28a2, 0x2419);
	write_cmos_sensor(0x28a4, 0x9382);
	write_cmos_sensor(0x28a6, 0x7328);
	write_cmos_sensor(0x28a8, 0x2402);
	write_cmos_sensor(0x28aa, 0x931E);
	write_cmos_sensor(0x28ac, 0x2414);
	write_cmos_sensor(0x28ae, 0x9382);
	write_cmos_sensor(0x28b0, 0x710E);
	write_cmos_sensor(0x28b2, 0x2402);
	write_cmos_sensor(0x28b4, 0x932E);
	write_cmos_sensor(0x28b6, 0x240F);
	write_cmos_sensor(0x28b8, 0x9382);
	write_cmos_sensor(0x28ba, 0x7114);
	write_cmos_sensor(0x28bc, 0x2402);
	write_cmos_sensor(0x28be, 0x922E);
	write_cmos_sensor(0x28c0, 0x240A);
	write_cmos_sensor(0x28c2, 0x903E);
	write_cmos_sensor(0x28c4, 0x0003);
	write_cmos_sensor(0x28c6, 0x23D7);
	write_cmos_sensor(0x28c8, 0x3C06);
	write_cmos_sensor(0x28ca, 0x43C2);
	write_cmos_sensor(0x28cc, 0x0A80);
	write_cmos_sensor(0x28ce, 0x0B00);
	write_cmos_sensor(0x28d0, 0x7302);
	write_cmos_sensor(0x28d2, 0xFFF0);
	write_cmos_sensor(0x28d4, 0x3FD0);
	write_cmos_sensor(0x28d6, 0x4F82);
	write_cmos_sensor(0x28d8, 0x812A);
	write_cmos_sensor(0x28da, 0x431F);
	write_cmos_sensor(0x28dc, 0x4130);
	write_cmos_sensor(0x28de, 0x120B);
	write_cmos_sensor(0x28e0, 0x120A);
	write_cmos_sensor(0x28e2, 0x1209);
	write_cmos_sensor(0x28e4, 0x1208);
	write_cmos_sensor(0x28e6, 0x1207);
	write_cmos_sensor(0x28e8, 0x1206);
	write_cmos_sensor(0x28ea, 0x403B);
	write_cmos_sensor(0x28ec, 0x000E);
	write_cmos_sensor(0x28ee, 0x510B);
	write_cmos_sensor(0x28f0, 0x4C09);
	write_cmos_sensor(0x28f2, 0x4D0A);
	write_cmos_sensor(0x28f4, 0x4B26);
	write_cmos_sensor(0x28f6, 0x5E0E);
	write_cmos_sensor(0x28f8, 0x6F0F);
	write_cmos_sensor(0x28fa, 0x5E0E);
	write_cmos_sensor(0x28fc, 0x6F0F);
	write_cmos_sensor(0x28fe, 0x5E0E);
	write_cmos_sensor(0x2900, 0x6F0F);
	write_cmos_sensor(0x2902, 0x5E0E);
	write_cmos_sensor(0x2904, 0x6F0F);
	write_cmos_sensor(0x2906, 0x4E0C);
	write_cmos_sensor(0x2908, 0x4F0D);
	write_cmos_sensor(0x290a, 0x4A0B);
	write_cmos_sensor(0x290c, 0x490A);
	write_cmos_sensor(0x290e, 0x12B0);
	write_cmos_sensor(0x2910, 0xFEEC);
	write_cmos_sensor(0x2912, 0x4C0F);
	write_cmos_sensor(0x2914, 0x9382);
	write_cmos_sensor(0x2916, 0x8126);
	write_cmos_sensor(0x2918, 0x2031);
	write_cmos_sensor(0x291a, 0x9382);
	write_cmos_sensor(0x291c, 0x811C);
	write_cmos_sensor(0x291e, 0x202E);
	write_cmos_sensor(0x2920, 0x9382);
	write_cmos_sensor(0x2922, 0x8120);
	write_cmos_sensor(0x2924, 0x202B);
	write_cmos_sensor(0x2926, 0x9382);
	write_cmos_sensor(0x2928, 0x812E);
	write_cmos_sensor(0x292a, 0x2028);
	write_cmos_sensor(0x292c, 0x4217);
	write_cmos_sensor(0x292e, 0x816C);
	write_cmos_sensor(0x2930, 0x4C0A);
	write_cmos_sensor(0x2932, 0x470C);
	write_cmos_sensor(0x2934, 0x430B);
	write_cmos_sensor(0x2936, 0x430D);
	write_cmos_sensor(0x2938, 0x12B0);
	write_cmos_sensor(0x293a, 0xFECC);
	write_cmos_sensor(0x293c, 0x4E08);
	write_cmos_sensor(0x293e, 0x4F09);
	write_cmos_sensor(0x2940, 0x403F);
	write_cmos_sensor(0x2942, 0x0100);
	write_cmos_sensor(0x2944, 0x870F);
	write_cmos_sensor(0x2946, 0x460A);
	write_cmos_sensor(0x2948, 0x4F0C);
	write_cmos_sensor(0x294a, 0x430B);
	write_cmos_sensor(0x294c, 0x430D);
	write_cmos_sensor(0x294e, 0x12B0);
	write_cmos_sensor(0x2950, 0xFECC);
	write_cmos_sensor(0x2952, 0x5E08);
	write_cmos_sensor(0x2954, 0x6F09);
	write_cmos_sensor(0x2956, 0x1088);
	write_cmos_sensor(0x2958, 0x1089);
	write_cmos_sensor(0x295a, 0xE948);
	write_cmos_sensor(0x295c, 0xE908);
	write_cmos_sensor(0x295e, 0xF379);
	write_cmos_sensor(0x2960, 0x480F);
	write_cmos_sensor(0x2962, 0x421E);
	write_cmos_sensor(0x2964, 0x8106);
	write_cmos_sensor(0x2966, 0x980E);
	write_cmos_sensor(0x2968, 0x2C05);
	write_cmos_sensor(0x296a, 0x480D);
	write_cmos_sensor(0x296c, 0x8E0D);
	write_cmos_sensor(0x296e, 0x4D82);
	write_cmos_sensor(0x2970, 0x8166);
	write_cmos_sensor(0x2972, 0x3C09);
	write_cmos_sensor(0x2974, 0x880E);
	write_cmos_sensor(0x2976, 0x4E82);
	write_cmos_sensor(0x2978, 0x8166);
	write_cmos_sensor(0x297a, 0x3C05);
	write_cmos_sensor(0x297c, 0x4F82);
	write_cmos_sensor(0x297e, 0x8106);
	write_cmos_sensor(0x2980, 0x40B2);
	write_cmos_sensor(0x2982, 0x00FF);
	write_cmos_sensor(0x2984, 0x8166);
	write_cmos_sensor(0x2986, 0x4136);
	write_cmos_sensor(0x2988, 0x4137);
	write_cmos_sensor(0x298a, 0x4138);
	write_cmos_sensor(0x298c, 0x4139);
	write_cmos_sensor(0x298e, 0x413A);
	write_cmos_sensor(0x2990, 0x413B);
	write_cmos_sensor(0x2992, 0x4130);
	write_cmos_sensor(0x2994, 0x120A);
	write_cmos_sensor(0x2996, 0x4E0D);
	write_cmos_sensor(0x2998, 0x421A);
	write_cmos_sensor(0x299a, 0x811A);
	write_cmos_sensor(0x299c, 0x4F0C);
	write_cmos_sensor(0x299e, 0x12B0);
	write_cmos_sensor(0x29a0, 0xFEB6);
	write_cmos_sensor(0x29a2, 0x4E0F);
	write_cmos_sensor(0x29a4, 0xC312);
	write_cmos_sensor(0x29a6, 0x100F);
	write_cmos_sensor(0x29a8, 0x110F);
	write_cmos_sensor(0x29aa, 0x110F);
	write_cmos_sensor(0x29ac, 0x110F);
	write_cmos_sensor(0x29ae, 0x4F0A);
	write_cmos_sensor(0x29b0, 0x4D0C);
	write_cmos_sensor(0x29b2, 0x12B0);
	write_cmos_sensor(0x29b4, 0xFEB6);
	write_cmos_sensor(0x29b6, 0x4E0F);
	write_cmos_sensor(0x29b8, 0xC312);
	write_cmos_sensor(0x29ba, 0x100F);
	write_cmos_sensor(0x29bc, 0x110F);
	write_cmos_sensor(0x29be, 0x110F);
	write_cmos_sensor(0x29c0, 0x110F);
	write_cmos_sensor(0x29c2, 0x413A);
	write_cmos_sensor(0x29c4, 0x4130);
	write_cmos_sensor(0x29c6, 0x120B);
	write_cmos_sensor(0x29c8, 0x120A);
	write_cmos_sensor(0x29ca, 0x1209);
	write_cmos_sensor(0x29cc, 0x1208);
	write_cmos_sensor(0x29ce, 0x4292);
	write_cmos_sensor(0x29d0, 0x0190);
	write_cmos_sensor(0x29d2, 0x0A84);
	write_cmos_sensor(0x29d4, 0x4292);
	write_cmos_sensor(0x29d6, 0x0192);
	write_cmos_sensor(0x29d8, 0x0A92);
	write_cmos_sensor(0x29da, 0x4292);
	write_cmos_sensor(0x29dc, 0x0194);
	write_cmos_sensor(0x29de, 0x0A94);
	write_cmos_sensor(0x29e0, 0x425F);
	write_cmos_sensor(0x29e2, 0x008C);
	write_cmos_sensor(0x29e4, 0x4FC2);
	write_cmos_sensor(0x29e6, 0x810C);
	write_cmos_sensor(0x29e8, 0x43C2);
	write_cmos_sensor(0x29ea, 0x810D);
	write_cmos_sensor(0x29ec, 0x4382);
	write_cmos_sensor(0x29ee, 0x0384);
	write_cmos_sensor(0x29f0, 0x40B2);
	write_cmos_sensor(0x29f2, 0x0020);
	write_cmos_sensor(0x29f4, 0x0386);
	write_cmos_sensor(0x29f6, 0xB3D2);
	write_cmos_sensor(0x29f8, 0x0080);
	write_cmos_sensor(0x29fa, 0x2403);
	write_cmos_sensor(0x29fc, 0x40B2);
	write_cmos_sensor(0x29fe, 0x8006);
	write_cmos_sensor(0x2a00, 0x0384);
	write_cmos_sensor(0x2a02, 0x4392);
	write_cmos_sensor(0x2a04, 0x8160);
	write_cmos_sensor(0x2a06, 0x4382);
	write_cmos_sensor(0x2a08, 0x740E);
	write_cmos_sensor(0x2a0a, 0xB3E2);
	write_cmos_sensor(0x2a0c, 0x0080);
	write_cmos_sensor(0x2a0e, 0x2402);
	write_cmos_sensor(0x2a10, 0x4392);
	write_cmos_sensor(0x2a12, 0x740E);
	write_cmos_sensor(0x2a14, 0x40F2);
	write_cmos_sensor(0x2a16, 0x001E);
	write_cmos_sensor(0x2a18, 0x0C92);
	write_cmos_sensor(0x2a1a, 0xD0F2);
	write_cmos_sensor(0x2a1c, 0x0006);
	write_cmos_sensor(0x2a1e, 0x0C81);
	write_cmos_sensor(0x2a20, 0x403B);
	write_cmos_sensor(0x2a22, 0x00BA);
	write_cmos_sensor(0x2a24, 0x4B6F);
	write_cmos_sensor(0x2a26, 0xF37F);
	write_cmos_sensor(0x2a28, 0x503F);
	write_cmos_sensor(0x2a2a, 0x0010);
	write_cmos_sensor(0x2a2c, 0x4F82);
	write_cmos_sensor(0x2a2e, 0x811A);
	write_cmos_sensor(0x2a30, 0x425F);
	write_cmos_sensor(0x2a32, 0x0C87);
	write_cmos_sensor(0x2a34, 0x4F4E);
	write_cmos_sensor(0x2a36, 0x425F);
	write_cmos_sensor(0x2a38, 0x0C88);
	write_cmos_sensor(0x2a3a, 0xF37F);
	write_cmos_sensor(0x2a3c, 0x12B0);
	write_cmos_sensor(0x2a3e, 0xF994);
	write_cmos_sensor(0x2a40, 0x4F82);
	write_cmos_sensor(0x2a42, 0x0C8C);
	write_cmos_sensor(0x2a44, 0x425F);
	write_cmos_sensor(0x2a46, 0x0C85);
	write_cmos_sensor(0x2a48, 0x4F4E);
	write_cmos_sensor(0x2a4a, 0x425F);
	write_cmos_sensor(0x2a4c, 0x0C89);
	write_cmos_sensor(0x2a4e, 0xF37F);
	write_cmos_sensor(0x2a50, 0x12B0);
	write_cmos_sensor(0x2a52, 0xF994);
	write_cmos_sensor(0x2a54, 0x4F82);
	write_cmos_sensor(0x2a56, 0x0C8A);
	write_cmos_sensor(0x2a58, 0x425D);
	write_cmos_sensor(0x2a5a, 0x00B7);
	write_cmos_sensor(0x2a5c, 0x5D4D);
	write_cmos_sensor(0x2a5e, 0x4DC2);
	write_cmos_sensor(0x2a60, 0x0CB0);
	write_cmos_sensor(0x2a62, 0x425E);
	write_cmos_sensor(0x2a64, 0x00B8);
	write_cmos_sensor(0x2a66, 0x5E4E);
	write_cmos_sensor(0x2a68, 0x4EC2);
	write_cmos_sensor(0x2a6a, 0x0CB1);
	write_cmos_sensor(0x2a6c, 0x4B6F);
	write_cmos_sensor(0x2a6e, 0xF37F);
	write_cmos_sensor(0x2a70, 0x108F);
	write_cmos_sensor(0x2a72, 0x403D);
	write_cmos_sensor(0x2a74, 0x0B90);
	write_cmos_sensor(0x2a76, 0x4D2E);
	write_cmos_sensor(0x2a78, 0xF37E);
	write_cmos_sensor(0x2a7a, 0xDE0F);
	write_cmos_sensor(0x2a7c, 0x4F82);
	write_cmos_sensor(0x2a7e, 0x0B20);
	write_cmos_sensor(0x2a80, 0x4B6F);
	write_cmos_sensor(0x2a82, 0xF37F);
	write_cmos_sensor(0x2a84, 0x108F);
	write_cmos_sensor(0x2a86, 0x4D2E);
	write_cmos_sensor(0x2a88, 0xF37E);
	write_cmos_sensor(0x2a8a, 0xDE0F);
	write_cmos_sensor(0x2a8c, 0x4F8D);
	write_cmos_sensor(0x2a8e, 0x0000);
	write_cmos_sensor(0x2a90, 0x0C0A);
	write_cmos_sensor(0x2a92, 0x425F);
	write_cmos_sensor(0x2a94, 0x009E);
	write_cmos_sensor(0x2a96, 0x4F4A);
	write_cmos_sensor(0x2a98, 0x425F);
	write_cmos_sensor(0x2a9a, 0x009F);
	write_cmos_sensor(0x2a9c, 0xF37F);
	write_cmos_sensor(0x2a9e, 0x5F0A);
	write_cmos_sensor(0x2aa0, 0x110A);
	write_cmos_sensor(0x2aa2, 0x110A);
	write_cmos_sensor(0x2aa4, 0x425F);
	write_cmos_sensor(0x2aa6, 0x00B2);
	write_cmos_sensor(0x2aa8, 0x4F4B);
	write_cmos_sensor(0x2aaa, 0x425F);
	write_cmos_sensor(0x2aac, 0x00B3);
	write_cmos_sensor(0x2aae, 0xF37F);
	write_cmos_sensor(0x2ab0, 0x5F0B);
	write_cmos_sensor(0x2ab2, 0x110B);
	write_cmos_sensor(0x2ab4, 0x110B);
	write_cmos_sensor(0x2ab6, 0x4A0E);
	write_cmos_sensor(0x2ab8, 0x5E0E);
	write_cmos_sensor(0x2aba, 0x5E0E);
	write_cmos_sensor(0x2abc, 0x5E0E);
	write_cmos_sensor(0x2abe, 0x5E0E);
	write_cmos_sensor(0x2ac0, 0x4B0F);
	write_cmos_sensor(0x2ac2, 0x5F0F);
	write_cmos_sensor(0x2ac4, 0x5F0F);
	write_cmos_sensor(0x2ac6, 0x5F0F);
	write_cmos_sensor(0x2ac8, 0x5F0F);
	write_cmos_sensor(0x2aca, 0x5F0F);
	write_cmos_sensor(0x2acc, 0x5F0F);
	write_cmos_sensor(0x2ace, 0x5F0F);
	write_cmos_sensor(0x2ad0, 0xDF0E);
	write_cmos_sensor(0x2ad2, 0x4E82);
	write_cmos_sensor(0x2ad4, 0x0A8E);
	write_cmos_sensor(0x2ad6, 0x403E);
	write_cmos_sensor(0x2ad8, 0x00BD);
	write_cmos_sensor(0x2ada, 0x4E6F);
	write_cmos_sensor(0x2adc, 0xF35F);
	write_cmos_sensor(0x2ade, 0x4F0D);
	write_cmos_sensor(0x2ae0, 0xF31D);
	write_cmos_sensor(0x2ae2, 0x4D82);
	write_cmos_sensor(0x2ae4, 0x8108);
	write_cmos_sensor(0x2ae6, 0x4E6F);
	write_cmos_sensor(0x2ae8, 0xF36F);
	write_cmos_sensor(0x2aea, 0x4F0E);
	write_cmos_sensor(0x2aec, 0xF32E);
	write_cmos_sensor(0x2aee, 0x4E82);
	write_cmos_sensor(0x2af0, 0x812C);
	write_cmos_sensor(0x2af2, 0x43B2);
	write_cmos_sensor(0x2af4, 0x7500);
	write_cmos_sensor(0x2af6, 0x43B2);
	write_cmos_sensor(0x2af8, 0x7502);
	write_cmos_sensor(0x2afa, 0x43B2);
	write_cmos_sensor(0x2afc, 0x7504);
	write_cmos_sensor(0x2afe, 0x43B2);
	write_cmos_sensor(0x2b00, 0x7506);
	write_cmos_sensor(0x2b02, 0x43B2);
	write_cmos_sensor(0x2b04, 0x7508);
	write_cmos_sensor(0x2b06, 0x43B2);
	write_cmos_sensor(0x2b08, 0x750A);
	write_cmos_sensor(0x2b0a, 0x43B2);
	write_cmos_sensor(0x2b0c, 0x750C);
	write_cmos_sensor(0x2b0e, 0x43B2);
	write_cmos_sensor(0x2b10, 0x750E);
	write_cmos_sensor(0x2b12, 0x43B2);
	write_cmos_sensor(0x2b14, 0x7510);
	write_cmos_sensor(0x2b16, 0x43B2);
	write_cmos_sensor(0x2b18, 0x7512);
	write_cmos_sensor(0x2b1a, 0x43B2);
	write_cmos_sensor(0x2b1c, 0x7514);
	write_cmos_sensor(0x2b1e, 0x43B2);
	write_cmos_sensor(0x2b20, 0x7516);
	write_cmos_sensor(0x2b22, 0x43B2);
	write_cmos_sensor(0x2b24, 0x7518);
	write_cmos_sensor(0x2b26, 0x43B2);
	write_cmos_sensor(0x2b28, 0x751A);
	write_cmos_sensor(0x2b2a, 0x43B2);
	write_cmos_sensor(0x2b2c, 0x751C);
	write_cmos_sensor(0x2b2e, 0x43B2);
	write_cmos_sensor(0x2b30, 0x751E);
	write_cmos_sensor(0x2b32, 0xB3E2);
	write_cmos_sensor(0x2b34, 0x0080);
	write_cmos_sensor(0x2b36, 0x2403);
	write_cmos_sensor(0x2b38, 0x40F2);
	write_cmos_sensor(0x2b3a, 0x0003);
	write_cmos_sensor(0x2b3c, 0x00B5);
	write_cmos_sensor(0x2b3e, 0x43B2);
	write_cmos_sensor(0x2b40, 0x752C);
	write_cmos_sensor(0x2b42, 0x40B2);
	write_cmos_sensor(0x2b44, 0x1001);
	write_cmos_sensor(0x2b46, 0x7500);
	write_cmos_sensor(0x2b48, 0x40B2);
	write_cmos_sensor(0x2b4a, 0x080D);
	write_cmos_sensor(0x2b4c, 0x7502);
	write_cmos_sensor(0x2b4e, 0x40B2);
	write_cmos_sensor(0x2b50, 0x080F);
	write_cmos_sensor(0x2b52, 0x7504);
	write_cmos_sensor(0x2b54, 0x40B2);
	write_cmos_sensor(0x2b56, 0x6803);
	write_cmos_sensor(0x2b58, 0x7506);
	write_cmos_sensor(0x2b5a, 0x40B2);
	write_cmos_sensor(0x2b5c, 0x0802);
	write_cmos_sensor(0x2b5e, 0x7508);
	write_cmos_sensor(0x2b60, 0x40B2);
	write_cmos_sensor(0x2b62, 0x0800);
	write_cmos_sensor(0x2b64, 0x750A);
	write_cmos_sensor(0x2b66, 0x403F);
	write_cmos_sensor(0x2b68, 0x0003);
	write_cmos_sensor(0x2b6a, 0x12B0);
	write_cmos_sensor(0x2b6c, 0xF870);
	write_cmos_sensor(0x2b6e, 0x421F);
	write_cmos_sensor(0x2b70, 0x0098);
	write_cmos_sensor(0x2b72, 0x821F);
	write_cmos_sensor(0x2b74, 0x0092);
	write_cmos_sensor(0x2b76, 0x531F);
	write_cmos_sensor(0x2b78, 0xC312);
	write_cmos_sensor(0x2b7a, 0x100F);
	write_cmos_sensor(0x2b7c, 0x421E);
	write_cmos_sensor(0x2b7e, 0x00AC);
	write_cmos_sensor(0x2b80, 0x821E);
	write_cmos_sensor(0x2b82, 0x00A6);
	write_cmos_sensor(0x2b84, 0x531E);
	write_cmos_sensor(0x2b86, 0x4A0D);
	write_cmos_sensor(0x2b88, 0x930D);
	write_cmos_sensor(0x2b8a, 0x2404);
	write_cmos_sensor(0x2b8c, 0xC312);
	write_cmos_sensor(0x2b8e, 0x100F);
	write_cmos_sensor(0x2b90, 0x831D);
	write_cmos_sensor(0x2b92, 0x23FC);
	write_cmos_sensor(0x2b94, 0x531F);
	write_cmos_sensor(0x2b96, 0xC31F);
	write_cmos_sensor(0x2b98, 0x4F82);
	write_cmos_sensor(0x2b9a, 0x0A86);
	write_cmos_sensor(0x2b9c, 0x4B0F);
	write_cmos_sensor(0x2b9e, 0x930F);
	write_cmos_sensor(0x2ba0, 0x2404);
	write_cmos_sensor(0x2ba2, 0xC312);
	write_cmos_sensor(0x2ba4, 0x100E);
	write_cmos_sensor(0x2ba6, 0x831F);
	write_cmos_sensor(0x2ba8, 0x23FC);
	write_cmos_sensor(0x2baa, 0x531E);
	write_cmos_sensor(0x2bac, 0xC31E);
	write_cmos_sensor(0x2bae, 0x4E82);
	write_cmos_sensor(0x2bb0, 0x0A88);
	write_cmos_sensor(0x2bb2, 0xB0B2);
	write_cmos_sensor(0x2bb4, 0x0010);
	write_cmos_sensor(0x2bb6, 0x0A84);
	write_cmos_sensor(0x2bb8, 0x2553);
	write_cmos_sensor(0x2bba, 0x421F);
	write_cmos_sensor(0x2bbc, 0x068C);
	write_cmos_sensor(0x2bbe, 0xC312);
	write_cmos_sensor(0x2bc0, 0x100F);
	write_cmos_sensor(0x2bc2, 0x4F82);
	write_cmos_sensor(0x2bc4, 0x0784);
	write_cmos_sensor(0x2bc6, 0x4292);
	write_cmos_sensor(0x2bc8, 0x068E);
	write_cmos_sensor(0x2bca, 0x0786);
	write_cmos_sensor(0x2bcc, 0xB3D2);
	write_cmos_sensor(0x2bce, 0x0CB6);
	write_cmos_sensor(0x2bd0, 0x2417);
	write_cmos_sensor(0x2bd2, 0x421A);
	write_cmos_sensor(0x2bd4, 0x0CB8);
	write_cmos_sensor(0x2bd6, 0x430B);
	write_cmos_sensor(0x2bd8, 0x425F);
	write_cmos_sensor(0x2bda, 0x0CBA);
	write_cmos_sensor(0x2bdc, 0x4F4E);
	write_cmos_sensor(0x2bde, 0x430F);
	write_cmos_sensor(0x2be0, 0x4E0F);
	write_cmos_sensor(0x2be2, 0x430E);
	write_cmos_sensor(0x2be4, 0xDE0A);
	write_cmos_sensor(0x2be6, 0xDF0B);
	write_cmos_sensor(0x2be8, 0x421F);
	write_cmos_sensor(0x2bea, 0x0CBC);
	write_cmos_sensor(0x2bec, 0x4F0C);
	write_cmos_sensor(0x2bee, 0x430D);
	write_cmos_sensor(0x2bf0, 0x421F);
	write_cmos_sensor(0x2bf2, 0x0CBE);
	write_cmos_sensor(0x2bf4, 0x430E);
	write_cmos_sensor(0x2bf6, 0xDE0C);
	write_cmos_sensor(0x2bf8, 0xDF0D);
	write_cmos_sensor(0x2bfa, 0x12B0);
	write_cmos_sensor(0x2bfc, 0xFEEC);
	write_cmos_sensor(0x2bfe, 0x4C09);
	write_cmos_sensor(0x2c00, 0xB2A2);
	write_cmos_sensor(0x2c02, 0x0A84);
	write_cmos_sensor(0x2c04, 0x2418);
	write_cmos_sensor(0x2c06, 0x421B);
	write_cmos_sensor(0x2c08, 0x0A96);
	write_cmos_sensor(0x2c0a, 0xC312);
	write_cmos_sensor(0x2c0c, 0x100B);
	write_cmos_sensor(0x2c0e, 0x110B);
	write_cmos_sensor(0x2c10, 0x110B);
	write_cmos_sensor(0x2c12, 0x43C2);
	write_cmos_sensor(0x2c14, 0x0A98);
	write_cmos_sensor(0x2c16, 0x4392);
	write_cmos_sensor(0x2c18, 0x8118);
	write_cmos_sensor(0x2c1a, 0x421D);
	write_cmos_sensor(0x2c1c, 0x8118);
	write_cmos_sensor(0x2c1e, 0x4B0A);
	write_cmos_sensor(0x2c20, 0x4D0C);
	write_cmos_sensor(0x2c22, 0x12B0);
	write_cmos_sensor(0x2c24, 0xFEB6);
	write_cmos_sensor(0x2c26, 0x9E09);
	write_cmos_sensor(0x2c28, 0x2910);
	write_cmos_sensor(0x2c2a, 0x531D);
	write_cmos_sensor(0x2c2c, 0x4D82);
	write_cmos_sensor(0x2c2e, 0x8118);
	write_cmos_sensor(0x2c30, 0x903D);
	write_cmos_sensor(0x2c32, 0x0009);
	write_cmos_sensor(0x2c34, 0x3BF2);
	write_cmos_sensor(0x2c36, 0xB3D2);
	write_cmos_sensor(0x2c38, 0x0788);
	write_cmos_sensor(0x2c3a, 0x2427);
	write_cmos_sensor(0x2c3c, 0xB3D2);
	write_cmos_sensor(0x2c3e, 0x07BC);
	write_cmos_sensor(0x2c40, 0x2424);
	write_cmos_sensor(0x2c42, 0xB0B2);
	write_cmos_sensor(0x2c44, 0x0020);
	write_cmos_sensor(0x2c46, 0x0A84);
	write_cmos_sensor(0x2c48, 0x2420);
	write_cmos_sensor(0x2c4a, 0x43C2);
	write_cmos_sensor(0x2c4c, 0x07AD);
	write_cmos_sensor(0x2c4e, 0x43C2);
	write_cmos_sensor(0x2c50, 0x07AF);
	write_cmos_sensor(0x2c52, 0x43C2);
	write_cmos_sensor(0x2c54, 0x07B1);
	write_cmos_sensor(0x2c56, 0x421E);
	write_cmos_sensor(0x2c58, 0x0084);
	write_cmos_sensor(0x2c5a, 0x421F);
	write_cmos_sensor(0x2c5c, 0x07B8);
	write_cmos_sensor(0x2c5e, 0x9F0E);
	write_cmos_sensor(0x2c60, 0x28C6);
	write_cmos_sensor(0x2c62, 0x43F2);
	write_cmos_sensor(0x2c64, 0x07AC);
	write_cmos_sensor(0x2c66, 0x40F2);
	write_cmos_sensor(0x2c68, 0x0053);
	write_cmos_sensor(0x2c6a, 0x07AE);
	write_cmos_sensor(0x2c6c, 0x40F2);
	write_cmos_sensor(0x2c6e, 0x002C);
	write_cmos_sensor(0x2c70, 0x07B0);
	write_cmos_sensor(0x2c72, 0x40F2);
	write_cmos_sensor(0x2c74, 0x001F);
	write_cmos_sensor(0x2c76, 0x07B2);
	write_cmos_sensor(0x2c78, 0x425E);
	write_cmos_sensor(0x2c7a, 0x00BA);
	write_cmos_sensor(0x2c7c, 0x425F);
	write_cmos_sensor(0x2c7e, 0x07B4);
	write_cmos_sensor(0x2c80, 0x9F4E);
	write_cmos_sensor(0x2c82, 0x28A7);
	write_cmos_sensor(0x2c84, 0x40F2);
	write_cmos_sensor(0x2c86, 0x0040);
	write_cmos_sensor(0x2c88, 0x0789);
	write_cmos_sensor(0x2c8a, 0x403F);
	write_cmos_sensor(0x2c8c, 0x7524);
	write_cmos_sensor(0x2c8e, 0x4FA2);
	write_cmos_sensor(0x2c90, 0x7528);
	write_cmos_sensor(0x2c92, 0x429F);
	write_cmos_sensor(0x2c94, 0x0084);
	write_cmos_sensor(0x2c96, 0x0000);
	write_cmos_sensor(0x2c98, 0x4292);
	write_cmos_sensor(0x2c9a, 0x0088);
	write_cmos_sensor(0x2c9c, 0x7316);
	write_cmos_sensor(0x2c9e, 0x93C2);
	write_cmos_sensor(0x2ca0, 0x008C);
	write_cmos_sensor(0x2ca2, 0x2403);
	write_cmos_sensor(0x2ca4, 0x4292);
	write_cmos_sensor(0x2ca6, 0x008A);
	write_cmos_sensor(0x2ca8, 0x7316);
	write_cmos_sensor(0x2caa, 0x403D);
	write_cmos_sensor(0x2cac, 0x002E);
	write_cmos_sensor(0x2cae, 0x93C2);
	write_cmos_sensor(0x2cb0, 0x008C);
	write_cmos_sensor(0x2cb2, 0x2407);
	write_cmos_sensor(0x2cb4, 0x421D);
	write_cmos_sensor(0x2cb6, 0x7316);
	write_cmos_sensor(0x2cb8, 0xC312);
	write_cmos_sensor(0x2cba, 0x100D);
	write_cmos_sensor(0x2cbc, 0x110D);
	write_cmos_sensor(0x2cbe, 0x503D);
	write_cmos_sensor(0x2cc0, 0x002E);
	write_cmos_sensor(0x2cc2, 0x430E);
	write_cmos_sensor(0x2cc4, 0x421F);
	write_cmos_sensor(0x2cc6, 0x0086);
	write_cmos_sensor(0x2cc8, 0x503F);
	write_cmos_sensor(0x2cca, 0xFFFB);
	write_cmos_sensor(0x2ccc, 0x9F82);
	write_cmos_sensor(0x2cce, 0x0084);
	write_cmos_sensor(0x2cd0, 0x2801);
	write_cmos_sensor(0x2cd2, 0x431E);
	write_cmos_sensor(0x2cd4, 0x4292);
	write_cmos_sensor(0x2cd6, 0x0086);
	write_cmos_sensor(0x2cd8, 0x7314);
	write_cmos_sensor(0x2cda, 0x93C2);
	write_cmos_sensor(0x2cdc, 0x00BC);
	write_cmos_sensor(0x2cde, 0x2008);
	write_cmos_sensor(0x2ce0, 0xB31E);
	write_cmos_sensor(0x2ce2, 0x2406);
	write_cmos_sensor(0x2ce4, 0x421F);
	write_cmos_sensor(0x2ce6, 0x0084);
	write_cmos_sensor(0x2ce8, 0x503F);
	write_cmos_sensor(0x2cea, 0x0005);
	write_cmos_sensor(0x2cec, 0x4F82);
	write_cmos_sensor(0x2cee, 0x7314);
	write_cmos_sensor(0x2cf0, 0x425F);
	write_cmos_sensor(0x2cf2, 0x00BC);
	write_cmos_sensor(0x2cf4, 0xF37F);
	write_cmos_sensor(0x2cf6, 0xFE0F);
	write_cmos_sensor(0x2cf8, 0x2465);
	write_cmos_sensor(0x2cfa, 0x421E);
	write_cmos_sensor(0x2cfc, 0x0086);
	write_cmos_sensor(0x2cfe, 0x503E);
	write_cmos_sensor(0x2d00, 0xFFFB);
	write_cmos_sensor(0x2d02, 0x4E82);
	write_cmos_sensor(0x2d04, 0x7524);
	write_cmos_sensor(0x2d06, 0x430E);
	write_cmos_sensor(0x2d08, 0x421F);
	write_cmos_sensor(0x2d0a, 0x7524);
	write_cmos_sensor(0x2d0c, 0x9F82);
	write_cmos_sensor(0x2d0e, 0x7528);
	write_cmos_sensor(0x2d10, 0x2C01);
	write_cmos_sensor(0x2d12, 0x431E);
	write_cmos_sensor(0x2d14, 0x430F);
	write_cmos_sensor(0x2d16, 0x9382);
	write_cmos_sensor(0x2d18, 0x8120);
	write_cmos_sensor(0x2d1a, 0x2001);
	write_cmos_sensor(0x2d1c, 0x431F);
	write_cmos_sensor(0x2d1e, 0xFE0F);
	write_cmos_sensor(0x2d20, 0x4D82);
	write_cmos_sensor(0x2d22, 0x7522);
	write_cmos_sensor(0x2d24, 0x2404);
	write_cmos_sensor(0x2d26, 0x503D);
	write_cmos_sensor(0x2d28, 0x0036);
	write_cmos_sensor(0x2d2a, 0x4D82);
	write_cmos_sensor(0x2d2c, 0x7522);
	write_cmos_sensor(0x2d2e, 0x421F);
	write_cmos_sensor(0x2d30, 0x8126);
	write_cmos_sensor(0x2d32, 0xD21F);
	write_cmos_sensor(0x2d34, 0x8100);
	write_cmos_sensor(0x2d36, 0x5F0F);
	write_cmos_sensor(0x2d38, 0xF32F);
	write_cmos_sensor(0x2d3a, 0x4F82);
	write_cmos_sensor(0x2d3c, 0x8126);
	write_cmos_sensor(0x2d3e, 0x421F);
	write_cmos_sensor(0x2d40, 0x8122);
	write_cmos_sensor(0x2d42, 0xD21F);
	write_cmos_sensor(0x2d44, 0x8124);
	write_cmos_sensor(0x2d46, 0x5F0F);
	write_cmos_sensor(0x2d48, 0xF32F);
	write_cmos_sensor(0x2d4a, 0x4F82);
	write_cmos_sensor(0x2d4c, 0x8122);
	write_cmos_sensor(0x2d4e, 0xD392);
	write_cmos_sensor(0x2d50, 0x7102);
	write_cmos_sensor(0x2d52, 0xB2E2);
	write_cmos_sensor(0x2d54, 0x010E);
	write_cmos_sensor(0x2d56, 0x2432);
	write_cmos_sensor(0x2d58, 0x421F);
	write_cmos_sensor(0x2d5a, 0x0124);
	write_cmos_sensor(0x2d5c, 0xF35F);
	write_cmos_sensor(0x2d5e, 0x4FC2);
	write_cmos_sensor(0x2d60, 0x816E);
	write_cmos_sensor(0x2d62, 0xB3E2);
	write_cmos_sensor(0x2d64, 0x010E);
	write_cmos_sensor(0x2d66, 0x2407);
	write_cmos_sensor(0x2d68, 0x421F);
	write_cmos_sensor(0x2d6a, 0x0084);
	write_cmos_sensor(0x2d6c, 0x9F82);
	write_cmos_sensor(0x2d6e, 0x0116);
	write_cmos_sensor(0x2d70, 0x2C22);
	write_cmos_sensor(0x2d72, 0x43D2);
	write_cmos_sensor(0x2d74, 0x816E);
	write_cmos_sensor(0x2d76, 0xB3D2);
	write_cmos_sensor(0x2d78, 0x010E);
	write_cmos_sensor(0x2d7a, 0x2419);
	write_cmos_sensor(0x2d7c, 0x434F);
	write_cmos_sensor(0x2d7e, 0x93C2);
	write_cmos_sensor(0x2d80, 0x816E);
	write_cmos_sensor(0x2d82, 0x2001);
	write_cmos_sensor(0x2d84, 0x435F);
	write_cmos_sensor(0x2d86, 0x4FC2);
	write_cmos_sensor(0x2d88, 0x00CA);
	write_cmos_sensor(0x2d8a, 0x430E);
	write_cmos_sensor(0x2d8c, 0x923E);
	write_cmos_sensor(0x2d8e, 0x246F);
	write_cmos_sensor(0x2d90, 0x9382);
	write_cmos_sensor(0x2d92, 0x8120);
	write_cmos_sensor(0x2d94, 0x2406);
	write_cmos_sensor(0x2d96, 0x4E0F);
	write_cmos_sensor(0x2d98, 0x5F0F);
	write_cmos_sensor(0x2d9a, 0x4F9F);
	write_cmos_sensor(0x2d9c, 0x0B00);
	write_cmos_sensor(0x2d9e, 0x8142);
	write_cmos_sensor(0x2da0, 0x3C66);
	write_cmos_sensor(0x2da2, 0x4E0F);
	write_cmos_sensor(0x2da4, 0x5F0F);
	write_cmos_sensor(0x2da6, 0x4F9F);
	write_cmos_sensor(0x2da8, 0x8142);
	write_cmos_sensor(0x2daa, 0x0B00);
	write_cmos_sensor(0x2dac, 0x3C60);
	write_cmos_sensor(0x2dae, 0x42D2);
	write_cmos_sensor(0x2db0, 0x816E);
	write_cmos_sensor(0x2db2, 0x00CA);
	write_cmos_sensor(0x2db4, 0x3FEA);
	write_cmos_sensor(0x2db6, 0x43C2);
	write_cmos_sensor(0x2db8, 0x816E);
	write_cmos_sensor(0x2dba, 0x3FDD);
	write_cmos_sensor(0x2dbc, 0x42D2);
	write_cmos_sensor(0x2dbe, 0x00CA);
	write_cmos_sensor(0x2dc0, 0x816E);
	write_cmos_sensor(0x2dc2, 0x3FCF);
	write_cmos_sensor(0x2dc4, 0x90B2);
	write_cmos_sensor(0x2dc6, 0x0005);
	write_cmos_sensor(0x2dc8, 0x7524);
	write_cmos_sensor(0x2dca, 0x2F9D);
	write_cmos_sensor(0x2dcc, 0x42A2);
	write_cmos_sensor(0x2dce, 0x7524);
	write_cmos_sensor(0x2dd0, 0x3F9A);
	write_cmos_sensor(0x2dd2, 0x425E);
	write_cmos_sensor(0x2dd4, 0x00BA);
	write_cmos_sensor(0x2dd6, 0x425F);
	write_cmos_sensor(0x2dd8, 0x07B5);
	write_cmos_sensor(0x2dda, 0x9F4E);
	write_cmos_sensor(0x2ddc, 0x2804);
	write_cmos_sensor(0x2dde, 0x40F2);
	write_cmos_sensor(0x2de0, 0x0020);
	write_cmos_sensor(0x2de2, 0x0789);
	write_cmos_sensor(0x2de4, 0x3F52);
	write_cmos_sensor(0x2de6, 0x40F2);
	write_cmos_sensor(0x2de8, 0x0010);
	write_cmos_sensor(0x2dea, 0x0789);
	write_cmos_sensor(0x2dec, 0x3F4E);
	write_cmos_sensor(0x2dee, 0x421F);
	write_cmos_sensor(0x2df0, 0x0084);
	write_cmos_sensor(0x2df2, 0x9F82);
	write_cmos_sensor(0x2df4, 0x07BA);
	write_cmos_sensor(0x2df6, 0x2810);
	write_cmos_sensor(0x2df8, 0x425F);
	write_cmos_sensor(0x2dfa, 0x00BA);
	write_cmos_sensor(0x2dfc, 0x9FC2);
	write_cmos_sensor(0x2dfe, 0x07B7);
	write_cmos_sensor(0x2e00, 0x280B);
	write_cmos_sensor(0x2e02, 0x42E2);
	write_cmos_sensor(0x2e04, 0x0789);
	write_cmos_sensor(0x2e06, 0x43C2);
	write_cmos_sensor(0x2e08, 0x07AC);
	write_cmos_sensor(0x2e0a, 0x43C2);
	write_cmos_sensor(0x2e0c, 0x07AE);
	write_cmos_sensor(0x2e0e, 0x43C2);
	write_cmos_sensor(0x2e10, 0x07B0);
	write_cmos_sensor(0x2e12, 0x43C2);
	write_cmos_sensor(0x2e14, 0x07B2);
	write_cmos_sensor(0x2e16, 0x3F39);
	write_cmos_sensor(0x2e18, 0x421F);
	write_cmos_sensor(0x2e1a, 0x07BA);
	write_cmos_sensor(0x2e1c, 0x9F82);
	write_cmos_sensor(0x2e1e, 0x0084);
	write_cmos_sensor(0x2e20, 0x2B34);
	write_cmos_sensor(0x2e22, 0x421E);
	write_cmos_sensor(0x2e24, 0x0084);
	write_cmos_sensor(0x2e26, 0x421F);
	write_cmos_sensor(0x2e28, 0x07B8);
	write_cmos_sensor(0x2e2a, 0x9F0E);
	write_cmos_sensor(0x2e2c, 0x2F2E);
	write_cmos_sensor(0x2e2e, 0x42F2);
	write_cmos_sensor(0x2e30, 0x0789);
	write_cmos_sensor(0x2e32, 0x43F2);
	write_cmos_sensor(0x2e34, 0x07AC);
	write_cmos_sensor(0x2e36, 0x40F2);
	write_cmos_sensor(0x2e38, 0x0024);
	write_cmos_sensor(0x2e3a, 0x07AE);
	write_cmos_sensor(0x2e3c, 0x40F2);
	write_cmos_sensor(0x2e3e, 0x001A);
	write_cmos_sensor(0x2e40, 0x07B0);
	write_cmos_sensor(0x2e42, 0x40F2);
	write_cmos_sensor(0x2e44, 0x0005);
	write_cmos_sensor(0x2e46, 0x07B2);
	write_cmos_sensor(0x2e48, 0x3F20);
	write_cmos_sensor(0x2e4a, 0x431F);
	write_cmos_sensor(0x2e4c, 0x4D0C);
	write_cmos_sensor(0x2e4e, 0x533C);
	write_cmos_sensor(0x2e50, 0x930C);
	write_cmos_sensor(0x2e52, 0x2403);
	write_cmos_sensor(0x2e54, 0x5F0F);
	write_cmos_sensor(0x2e56, 0x831C);
	write_cmos_sensor(0x2e58, 0x23FD);
	write_cmos_sensor(0x2e5a, 0x4FC2);
	write_cmos_sensor(0x2e5c, 0x0A98);
	write_cmos_sensor(0x2e5e, 0x3EEB);
	write_cmos_sensor(0x2e60, 0x4292);
	write_cmos_sensor(0x2e62, 0x0A86);
	write_cmos_sensor(0x2e64, 0x0784);
	write_cmos_sensor(0x2e66, 0x4292);
	write_cmos_sensor(0x2e68, 0x0A88);
	write_cmos_sensor(0x2e6a, 0x0786);
	write_cmos_sensor(0x2e6c, 0x3EAF);
	write_cmos_sensor(0x2e6e, 0x531E);
	write_cmos_sensor(0x2e70, 0x903E);
	write_cmos_sensor(0x2e72, 0x000F);
	write_cmos_sensor(0x2e74, 0x2B8B);
	write_cmos_sensor(0x2e76, 0x4138);
	write_cmos_sensor(0x2e78, 0x4139);
	write_cmos_sensor(0x2e7a, 0x413A);
	write_cmos_sensor(0x2e7c, 0x413B);
	write_cmos_sensor(0x2e7e, 0x4130);
	write_cmos_sensor(0x2e80, 0x0260);
	write_cmos_sensor(0x2e82, 0x0000);
	write_cmos_sensor(0x2e84, 0x0C5A);
	write_cmos_sensor(0x2e86, 0x0240);
	write_cmos_sensor(0x2e88, 0x0000);
	write_cmos_sensor(0x2e8a, 0x0260);
	write_cmos_sensor(0x2e8c, 0x0000);
	write_cmos_sensor(0x2e8e, 0x0C14);
	write_cmos_sensor(0x2e90, 0x4130);
	write_cmos_sensor(0x2e92, 0x4382);
	write_cmos_sensor(0x2e94, 0x7602);
	write_cmos_sensor(0x2e96, 0x4F82);
	write_cmos_sensor(0x2e98, 0x7600);
	write_cmos_sensor(0x2e9a, 0x0270);
	write_cmos_sensor(0x2e9c, 0x0000);
	write_cmos_sensor(0x2e9e, 0x0C1B);
	write_cmos_sensor(0x2ea0, 0x0270);
	write_cmos_sensor(0x2ea2, 0x0001);
	write_cmos_sensor(0x2ea4, 0x403E);
	write_cmos_sensor(0x2ea6, 0x7606);
	write_cmos_sensor(0x2ea8, 0x4E2F);
	write_cmos_sensor(0x2eaa, 0x4FC2);
	write_cmos_sensor(0x2eac, 0x0188);
	write_cmos_sensor(0x2eae, 0x4E2F);
	write_cmos_sensor(0x2eb0, 0x4130);
	write_cmos_sensor(0x2eb2, 0xDF02);
	write_cmos_sensor(0x2eb4, 0x3FFE);
	write_cmos_sensor(0x2eb6, 0x430E);
	write_cmos_sensor(0x2eb8, 0x930A);
	write_cmos_sensor(0x2eba, 0x2407);
	write_cmos_sensor(0x2ebc, 0xC312);
	write_cmos_sensor(0x2ebe, 0x100C);
	write_cmos_sensor(0x2ec0, 0x2801);
	write_cmos_sensor(0x2ec2, 0x5A0E);
	write_cmos_sensor(0x2ec4, 0x5A0A);
	write_cmos_sensor(0x2ec6, 0x930C);
	write_cmos_sensor(0x2ec8, 0x23F7);
	write_cmos_sensor(0x2eca, 0x4130);
	write_cmos_sensor(0x2ecc, 0x4030);
	write_cmos_sensor(0x2ece, 0xFF16);
	write_cmos_sensor(0x2ed0, 0xEE0E);
	write_cmos_sensor(0x2ed2, 0x403B);
	write_cmos_sensor(0x2ed4, 0x0011);
	write_cmos_sensor(0x2ed6, 0x3C05);
	write_cmos_sensor(0x2ed8, 0x100D);
	write_cmos_sensor(0x2eda, 0x6E0E);
	write_cmos_sensor(0x2edc, 0x9A0E);
	write_cmos_sensor(0x2ede, 0x2801);
	write_cmos_sensor(0x2ee0, 0x8A0E);
	write_cmos_sensor(0x2ee2, 0x6C0C);
	write_cmos_sensor(0x2ee4, 0x6D0D);
	write_cmos_sensor(0x2ee6, 0x831B);
	write_cmos_sensor(0x2ee8, 0x23F7);
	write_cmos_sensor(0x2eea, 0x4130);
	write_cmos_sensor(0x2eec, 0xEF0F);
	write_cmos_sensor(0x2eee, 0xEE0E);
	write_cmos_sensor(0x2ef0, 0x4039);
	write_cmos_sensor(0x2ef2, 0x0021);
	write_cmos_sensor(0x2ef4, 0x3C0A);
	write_cmos_sensor(0x2ef6, 0x1008);
	write_cmos_sensor(0x2ef8, 0x6E0E);
	write_cmos_sensor(0x2efa, 0x6F0F);
	write_cmos_sensor(0x2efc, 0x9B0F);
	write_cmos_sensor(0x2efe, 0x2805);
	write_cmos_sensor(0x2f00, 0x2002);
	write_cmos_sensor(0x2f02, 0x9A0E);
	write_cmos_sensor(0x2f04, 0x2802);
	write_cmos_sensor(0x2f06, 0x8A0E);
	write_cmos_sensor(0x2f08, 0x7B0F);
	write_cmos_sensor(0x2f0a, 0x6C0C);
	write_cmos_sensor(0x2f0c, 0x6D0D);
	write_cmos_sensor(0x2f0e, 0x6808);
	write_cmos_sensor(0x2f10, 0x8319);
	write_cmos_sensor(0x2f12, 0x23F1);
	write_cmos_sensor(0x2f14, 0x4130);
	write_cmos_sensor(0x2f16, 0x430E);
	write_cmos_sensor(0x2f18, 0x430F);
	write_cmos_sensor(0x2f1a, 0x3C08);
	write_cmos_sensor(0x2f1c, 0xC312);
	write_cmos_sensor(0x2f1e, 0x100D);
	write_cmos_sensor(0x2f20, 0x100C);
	write_cmos_sensor(0x2f22, 0x2802);
	write_cmos_sensor(0x2f24, 0x5A0E);
	write_cmos_sensor(0x2f26, 0x6B0F);
	write_cmos_sensor(0x2f28, 0x5A0A);
	write_cmos_sensor(0x2f2a, 0x6B0B);
	write_cmos_sensor(0x2f2c, 0x930C);
	write_cmos_sensor(0x2f2e, 0x23F6);
	write_cmos_sensor(0x2f30, 0x930D);
	write_cmos_sensor(0x2f32, 0x23F4);
	write_cmos_sensor(0x2f34, 0x4130);
	write_cmos_sensor(0x2f36, 0x0000);
	write_cmos_sensor(0x2ffe, 0xf000);
	write_cmos_sensor(0x3000, 0x02B0);
	write_cmos_sensor(0x3002, 0x02B0);
	write_cmos_sensor(0x3004, 0x0678);
	write_cmos_sensor(0x3006, 0x0278);
	write_cmos_sensor(0x3008, 0x0678);
	write_cmos_sensor(0x300A, 0x0279);
	write_cmos_sensor(0x300C, 0x024A);
	write_cmos_sensor(0x300E, 0x024A);
	write_cmos_sensor(0x3010, 0x0A42);
	write_cmos_sensor(0x3012, 0x0A42);
	write_cmos_sensor(0x3014, 0x0242);
	write_cmos_sensor(0x3016, 0x0678);
	write_cmos_sensor(0x3018, 0x0278);
	write_cmos_sensor(0x301A, 0x0678);
	write_cmos_sensor(0x301C, 0x0278);
	write_cmos_sensor(0x4000, 0x0005);
	write_cmos_sensor(0x4002, 0x003D);
	write_cmos_sensor(0x4004, 0x0F00);
	write_cmos_sensor(0x4006, 0xCF41);
	write_cmos_sensor(0x4008, 0x1F04);
	write_cmos_sensor(0x400A, 0xDF43);
	write_cmos_sensor(0x400C, 0x1F87);
	write_cmos_sensor(0x400E, 0x1F87);
	write_cmos_sensor(0x4010, 0x1F86);
	write_cmos_sensor(0x4012, 0x1FBE);
	write_cmos_sensor(0x4014, 0x1F83);
	write_cmos_sensor(0x4016, 0x1F04);
	write_cmos_sensor(0x4018, 0x1F04);
	write_cmos_sensor(0x401A, 0x1F04);
	write_cmos_sensor(0x401C, 0x1F04);
	//-------- FW End ----------//


	// SREG --------------------------------------------------------------
	write_cmos_sensor(0x0B00, 0xE1DA);
	write_cmos_sensor(0x0B02, 0x0000);
	write_cmos_sensor(0x0B04, 0x6A2E); // 140905 pcp 3.8v : 
	write_cmos_sensor(0x0B06, 0x59AF); // 140606 NCP ON : -0.9V, ncp pumping cap. control
	write_cmos_sensor(0x0B08, 0x8085); // Reset Clamp Level 
	write_cmos_sensor(0x0B0A, 0xBD49); // 140725 CDS amp1,amp2 bias 
	write_cmos_sensor(0x0B0C, 0xE1BC); // NC control
	write_cmos_sensor(0x0B0E, 0xB361); // input rng : 600mv
	write_cmos_sensor(0x0B10, 0xF000); // vrst
	write_cmos_sensor(0x0B12, 0x5000); // 140421 ramp clk_div2, 4lsb control enable 
	write_cmos_sensor(0x0B14, 0x370B);	
	write_cmos_sensor(0x0B16, 0x440F); // Analog Clock Speed
	write_cmos_sensor(0x0B18, 0x0000);
	write_cmos_sensor(0x0B1A, 0x9000);
	write_cmos_sensor(0x0B1C, 0x1000);
	write_cmos_sensor(0x0B1E, 0x0500);
	//--------------------------------------------------------------------

	// mipi --------------------------------------------------------------
	write_cmos_sensor(0x090A, 0x03E8); // mipi_vblank_delay_h / l
	write_cmos_sensor(0x090C, 0x0020); // mipi_hblank_short_delay_h / l
	write_cmos_sensor(0x090E, 0x0E00); // mipi_hblank_long_delay_h / l
	write_cmos_sensor(0x0902, 0x4301); //4101); // mipi_tx_op_mode1 / mipi_tx_op_mode2
	write_cmos_sensor(0x0a02, 0x0100); // fast sleep
	write_cmos_sensor(0x0910, 0x5D07); // mipi_Exit_sequence / mipi_LPX
	write_cmos_sensor(0x0912, 0x061e); // mipi_CLK_prepare   / mipi_clk_zero
	write_cmos_sensor(0x0914, 0x0407); // mipi_clk_pre		/ mipi_data_prepare
	write_cmos_sensor(0x0916, 0x0b0d); // mipi_data_zero		/ mipi_data_trail	
	write_cmos_sensor(0x0918, 0x0f09); // mipi_clk_post		/ mipi_clk_trail

	// system ------------------------------------------------------------
	write_cmos_sensor(0x0F02, 0x0106); // pll_cfg1 / pll_cfg2
	write_cmos_sensor(0x0C36, 0x0100); // g_sum_ctl / null

	// tg ----------------------------------------------------------------
	write_cmos_sensor(0x0000, 0x0000); // image_orient / null
	write_cmos_sensor(0x003C, 0x0000); // fixed_frame / tg_ctl_0
	write_cmos_sensor(0x003E, 0x0000); // tg_ctl1 / tg_ctl2
	write_cmos_sensor(0x004C, 0x0100); // tg_enable / null
	write_cmos_sensor(0x0C10, 0x0532); // 131102 BLC offset for DDS operation 

	// blc  --------------------------------------------------------------
	write_cmos_sensor(0x0C00, 0x7B06); // 131102 blc_ctl1,2 - lblc,lblc_dpc,adp_dead_pxl_th,dither enable
	write_cmos_sensor(0x0C02, 0x1000); // 131102 blc_ofs_wgt
	write_cmos_sensor(0x0C0E, 0xE501); // 140606 blc_ctl3,4 - line blc on  
	write_cmos_sensor(0x0C04, 0x8004); // 131102 analog_ofs_man / lblc_dead_pixel_th 
	write_cmos_sensor(0x0C26, 0x4C10); // 131102 curr_frm_obp_avg_wgt (x0.6),frm_obp_avg_pga_wgt (x1)

	// pixel address -----------------------------------------------------
	write_cmos_sensor(0x000E, 0x0000); // x_addr_start_lobp_h / l
	write_cmos_sensor(0x0014, 0x003F); // x_addr_end_lobp_h / l
	write_cmos_sensor(0x0010, 0x0060); // x_addr_start_robp_h / l
	write_cmos_sensor(0x0016, 0x009F); // x_addr_end_robp_h / l
	write_cmos_sensor(0x0012, 0x00C0); // x_addr_start_hact_h / l
	write_cmos_sensor(0x0018, 0x0ADF); // x_addr_end_hact_h / l
	write_cmos_sensor(0x0022, 0x0004); // y_addr_start_fobp_h / l
	write_cmos_sensor(0x0028, 0x000B); // y_addr_end_fobp_h / l
	write_cmos_sensor(0x0024, 0xFFFA); // y_addr_start_dummy_h / l
	write_cmos_sensor(0x002A, 0xFFFF); // y_addr_end_dummy_h / l
	write_cmos_sensor(0x0026, 0x0030); // y_addr_start_vact_h / l
	write_cmos_sensor(0x002C, 0x07C7); // y_addr_end_vact_h / l
	write_cmos_sensor(0x0032, 0x0101); // y_odd_inc_vact / y_even_inc_vact

	// size info -----------------------------------------------------
	write_cmos_sensor(0x0006, 0x07ca); // frame_length_lines_h / l
	write_cmos_sensor(0x0008, 0x0B7C); // line_length_pck_h / l
	write_cmos_sensor(0x0020, 0x0700); // x_region_sel / x_region_orient
	write_cmos_sensor(0x0034, 0x0700); // y_region_sel / y_resion_orient
	write_cmos_sensor(0x0112, 0x0A20); // x_output_size_h / l
	write_cmos_sensor(0x0114, 0x0798); // y_output_size_h / l

	// isp control ---------------------------------------------------
	write_cmos_sensor(0x0110, 0x0123); // 140429 isp_en_h / l
	write_cmos_sensor(0x020A, 0x0002); // test_pattern_mode

	// Exp. time & AG ---------------------------------------------
	write_cmos_sensor(0x0002, 0x0200); // fine_integ_time_h / l
	write_cmos_sensor(0x0004, 0x07b7); // coarse_integ_time_h / l  30ms
	write_cmos_sensor(0x003A, 0x0400); // analog_gain_code_global / null


	// Analog set  --------------------------------------------------
	write_cmos_sensor(0x0036, 0x007F); //131102 ramp_init_pofs / ramp_rst_pofs
	write_cmos_sensor(0x0038, 0x7F00); //131102 ramp_sig_pofs / null
	write_cmos_sensor(0x0138, 0x0304); //140421 Rx,Sx pcp enable 
	write_cmos_sensor(0x013A, 0x0100); // d2a_row_idle_ctrl_en
	write_cmos_sensor(0x0120, 0x0064); // 140421 Extra Dynamic Signal Timing Setting 
	write_cmos_sensor(0x0122, 0x0582);
	write_cmos_sensor(0x0C06, 0x0008); // fblc_ofs_wgt / fblc_dead_pixel_th 
	write_cmos_sensor(0x0C08, 0x0302); // dpc pga weight   16 = 1x,   x/16
	write_cmos_sensor(0x013C, 0x0001); // pll clk inversion 
	write_cmos_sensor(0x013E, 0x1F23); // 140725 analog dithering ON, manual control
	write_cmos_sensor(0x0140, 0x0004); // 140725 analog dithering pulse rising
	write_cmos_sensor(0x0142, 0x0064); // 140725 analog dithering pulse falling
	write_cmos_sensor(0x073c, 0x0100); // 140616 mcu on/of
	write_cmos_sensor(0x0C12, 0x0040); // blc offset 0 
	write_cmos_sensor(0x0A10, 0x0000); // Pedestal value
	write_cmos_sensor(0x0500, 0x0000); // Pedestal enable

	// Soft standby & dpc  --------------------------------------
	write_cmos_sensor(0x0708, 0x0100); //140628 dpc disable, DPC select reg. off=0x0000, on=0x0100
	write_cmos_sensor(0x070a, 0x390b); //140616
	write_cmos_sensor(0x0712, 0x643c); //140616
	write_cmos_sensor(0x0720, 0xa050); //140616
	write_cmos_sensor(0x0726, 0x1428); //140616
	write_cmos_sensor(0x0728, 0x2867); //140616
	write_cmos_sensor(0x072a, 0x5437); //140616
	write_cmos_sensor(0x0734, 0xf070); //140616 //AG max 16x, AG middle 8x
	write_cmos_sensor(0x0736, 0x0000); //140616
	write_cmos_sensor(0x0738, 0x0EE1); //1d3a); //140616 //Exp time max 125ms
	write_cmos_sensor(0x073a, 0x01f2); //140616

	write_cmos_sensor(0x0118, 0x0100); //sleep Off

	LOG_INF("Exit sensor_init\n");

}	/*	sensor_init  */
static void preview_setting(void)
{
 LOG_INF("Enter pre setting\n");
 //=====================================================================
 // Preview mode 
 // Resolution : 1296x972
 // MIPI Speed : 440Mbps x 2lane (PCLK 176Mhz)
 // Lin length 2940 / Frame length 1980
 // Frame rate : max 30.23fps
 //=====================================================================
 write_cmos_sensor(0x0118, 0x0000); 		 // Sleep On
 write_cmos_sensor(0x0032, 0x0101); 	 // byrscl_crtl1 / tyrscl_fifo_read_delay (add 140714)
 write_cmos_sensor(0x090A, 0x0384); 	 // mipi_vblank_delay_h / l
 write_cmos_sensor(0x090C, 0x000A); 	 // mipi_hblank_short_delay_h / l
 write_cmos_sensor(0x090E, 0x03E8); 	 // mipi_hblank_long_delay_h / l
 write_cmos_sensor(0x0902, 0x4101); 	 // mipi_tx_op_mode1 / mipi_tx_op_mode2
 write_cmos_sensor(0x0910, 0x5D04); 		 //5d03 	 // mipi_value_exit_seq / mipi_value_lpx
 write_cmos_sensor(0x0912, 0x030E); 	 // mipi_value_clk_prepare / mipi_value_clk_zeo
 write_cmos_sensor(0x0914, 0x0404); 		 //0403 	 // mipi_value_clk_pre / mipi_value_data_prepare
 write_cmos_sensor(0x0916, 0x0709); 	 // mipi_value_data_zero / mipi_value_data_trail
 write_cmos_sensor(0x0918, 0x0c05); 		 //0a04 	 // mipi_value_clk_post / mipi_value_clk_trail
 write_cmos_sensor(0x091A, 0x0800); 	 // mipi_value_exit / null
 
 write_cmos_sensor(0x0B14, 0x370B);   
 write_cmos_sensor(0x0B16, 0x448F); 		 // pll_mipi_clk_div
 write_cmos_sensor(0x0700, 0xA0A3); 	 // byrscl_ctrl1 / byrscl_fifo_read_delay
 write_cmos_sensor(0x0012, 0x00BE); 	 // x_addr_start_hact_h / l 192
 write_cmos_sensor(0x0018, 0x0AE1); 	 // x_addr_end_hact_h / l 2783
 write_cmos_sensor(0x0026, 0x002E); 	 // y_addr_start_vact_h / l 48
 write_cmos_sensor(0x002C, 0x07C9); 	 // y_addr_end_vact_h / l 1991
 write_cmos_sensor(0x0112, 0x0510); 	 // x_output_size_h / l 1296
 write_cmos_sensor(0x0114, 0x03CC); 	 // y_output_size_h / l 972
 //write_cmos_sensor(0x0118, 0x0100); 		 // Sleep Off
write_cmos_sensor(0x0708, 0x0300);
  if (imgsensor.ihdr_en) {

    } else {

    }
// Sleep Off (Streaming On)
write_cmos_sensor(0x0118, 0x0100);	//sleep Off
LOG_INF("Exit pre setting\n");
  
  
}    /*    preview_setting  */

static void capture_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);
    if (currefps == 150) { //24fps for PIP
        //@@full_132PCLK_24.75
        
		//=====================================================================
		// Capture mode 
		// Resolution : 2592x1944
		// MIPI Speed : 448Mbps x 2lane (PCLK 176Mhz)
		// Lin length 2940 / Frame length 1980
		// Frame rate : max 15.28fps
		//=====================================================================
		write_cmos_sensor(0x0118, 0x0000);	 // Sleep On
		write_cmos_sensor(0x0032, 0x0101);		// byrscl_crtl1 / tyrscl_fifo_read_delay 
		
		write_cmos_sensor(0x090A, 0x29E8);		// mipi_vblank_delay_h / l
		write_cmos_sensor(0x090C, 0x0020);		// mipi_hblank_short_delay_h / l
		write_cmos_sensor(0x090E, 0x0E00);		// mipi_hblank_long_delay_h / l
		write_cmos_sensor(0x0902, 0x4101);		// mipi_tx_op_mode1 / mipi_tx_op_mode2
		write_cmos_sensor(0x0910, 0x5D05);			// mipi_Exit_sequence / mipi_LPX
		write_cmos_sensor(0x0912, 0x0410);			// mipi_CLK_prepare   / mipi_clk_zero
		write_cmos_sensor(0x0914, 0x0404);			// mipi_clk_pre 	/ mipi_data_prepare
		write_cmos_sensor(0x0916, 0x0b09);			// mipi_data_zero		/ mipi_data_trail	
		write_cmos_sensor(0x0918, 0x0f05);			// mipi_clk_post		/ mipi_clk_trail
		write_cmos_sensor(0x091A, 0x0A00);		// mipi_value_exit / null
		write_cmos_sensor(0x0B14, 0x1C0B);		//mdiv :28
		write_cmos_sensor(0x0B16, 0x4A0F);			// pll_mipi_clk_div
		write_cmos_sensor(0x0700, 0x00A3);		// byrscl_ctrl1 / byrscl_fifo_read_delay
		write_cmos_sensor(0x0012, 0x00BE);		// x_addr_start_hact_h / l 192
		write_cmos_sensor(0x0018, 0x0AE1);		// x_addr_end_hact_h / l 2783
		write_cmos_sensor(0x0026, 0x002E);		// y_addr_start_vact_h / l 48
		write_cmos_sensor(0x002C, 0x07C9);		// y_addr_end_vact_h / l 1991
		write_cmos_sensor(0x0112, 0x0A20);		// x_output_size_h / l 2592
		write_cmos_sensor(0x0114, 0x0798);		// y_output_size_h / l 1944 
		write_cmos_sensor(0x0708, 0x0100);
        
    } else {   //30fps            //30fps for Normal capture & ZSD
        LOG_INF("E! currefps:%d\n",currefps);
	//====================================================================

	//Image size 2592x1944
// Sleep On (Straming Off)
	//=====================================================================
	// Capture mode 
	// Resolution : 2592x1944
	// MIPI Speed : 880Mbps x 2lane (PCLK 176Mhz)
	// Lin length 2940 / Frame length 1980
	// Frame rate : max 30.23fps
	//=====================================================================
	write_cmos_sensor(0x0118, 0x0000);	 // Sleep On
	write_cmos_sensor(0x0032, 0x0101);		// byrscl_crtl1 / tyrscl_fifo_read_delay 
	write_cmos_sensor(0x090A, 0x03E8);		// mipi_vblank_delay_h / l
	write_cmos_sensor(0x090C, 0x0020);		// mipi_hblank_short_delay_h / l
	write_cmos_sensor(0x090E, 0x0E00);		// mipi_hblank_long_delay_h / l
	write_cmos_sensor(0x0902, 0x4101);		// mipi_tx_op_mode1 / mipi_tx_op_mode2
	write_cmos_sensor(0x0910, 0x5D07);		// mipi_Exit_sequence / mipi_LPX
	write_cmos_sensor(0x0912, 0x061e);		// mipi_CLK_prepare   / mipi_clk_zero
	write_cmos_sensor(0x0914, 0x0407);		// mipi_clk_pre 	/ mipi_data_prepare
	write_cmos_sensor(0x0916, 0x0b0d);			//0b0a // mipi_data_zero		/ mipi_data_trail	
	write_cmos_sensor(0x0918, 0x0f09);			//0e09 // mipi_clk_post 	/ mipi_clk_trail
	write_cmos_sensor(0x091A, 0x0A00);		// mipi_value_exit / null
	
	write_cmos_sensor(0x0B14, 0x370B);	 
	write_cmos_sensor(0x0B16, 0x440F);			// pll_mipi_clk_div
	write_cmos_sensor(0x0700, 0x00A3);		// byrscl_ctrl1 / byrscl_fifo_read_delay
	write_cmos_sensor(0x0012, 0x00BE);		// x_addr_start_hact_h / l 192
	write_cmos_sensor(0x0018, 0x0AE1);		// x_addr_end_hact_h / l 2783
	write_cmos_sensor(0x0026, 0x002E);		// y_addr_start_vact_h / l 48
	write_cmos_sensor(0x002C, 0x07C9);		// y_addr_end_vact_h / l 1991
	write_cmos_sensor(0x0112, 0x0A20);		// x_output_size_h / l 2592
	write_cmos_sensor(0x0114, 0x0798);		// y_output_size_h / l 1944 
	write_cmos_sensor(0x0708, 0x0100);

// Sleep Off (Streaming On)

        if (imgsensor.ihdr_en) {

    } else {

    }
        write_cmos_sensor(0x0118, 0x0100);	//sleep Off
    }

}

static void normal_video_setting(kal_uint16 currefps)
{
    LOG_INF("E! currefps:%d\n",currefps);

	// Sleep On (Straming Off)
 write_cmos_sensor(0x0118, 0x0000); 		 // Sleep On
 write_cmos_sensor(0x0032, 0x0101); 	 // byrscl_crtl1 / tyrscl_fifo_read_delay (add 140714)
 write_cmos_sensor(0x090A, 0x0384); 	 // mipi_vblank_delay_h / l
 write_cmos_sensor(0x090C, 0x000A); 	 // mipi_hblank_short_delay_h / l
 write_cmos_sensor(0x090E, 0x03E8); 	 // mipi_hblank_long_delay_h / l
 write_cmos_sensor(0x0902, 0x4101); 	 // mipi_tx_op_mode1 / mipi_tx_op_mode2
 write_cmos_sensor(0x0910, 0x5D04); 		 //5d03 	 // mipi_value_exit_seq / mipi_value_lpx
 write_cmos_sensor(0x0912, 0x030E); 	 // mipi_value_clk_prepare / mipi_value_clk_zeo
 write_cmos_sensor(0x0914, 0x0404); 		 //0403 	 // mipi_value_clk_pre / mipi_value_data_prepare
 write_cmos_sensor(0x0916, 0x0709); 	 // mipi_value_data_zero / mipi_value_data_trail
 write_cmos_sensor(0x0918, 0x0c05); 		 //0a04 	 // mipi_value_clk_post / mipi_value_clk_trail
 write_cmos_sensor(0x091A, 0x0800); 	 // mipi_value_exit / null
 
 write_cmos_sensor(0x0B14, 0x370B);   
 write_cmos_sensor(0x0B16, 0x448F); 		 // pll_mipi_clk_div
 write_cmos_sensor(0x0700, 0xA0A3); 	 // byrscl_ctrl1 / byrscl_fifo_read_delay
 write_cmos_sensor(0x0012, 0x00BE); 	 // x_addr_start_hact_h / l 192
 write_cmos_sensor(0x0018, 0x0AE1); 	 // x_addr_end_hact_h / l 2783
 write_cmos_sensor(0x0026, 0x002E); 	 // y_addr_start_vact_h / l 48
 write_cmos_sensor(0x002C, 0x07C9); 	 // y_addr_end_vact_h / l 1991
 write_cmos_sensor(0x0112, 0x0510); 	 // x_output_size_h / l 1296
 write_cmos_sensor(0x0114, 0x03CC); 	 // y_output_size_h / l 972
 //write_cmos_sensor(0x0118, 0x0100); 		 // Sleep Off
write_cmos_sensor(0x0708, 0x0300);
 if (imgsensor.ihdr_en) {
    } else {
    }
// Sleep Off (Streaming On)
write_cmos_sensor(0x0118, 0x0100);	//sleep Off
}
static void hs_video_setting()
{
	write_cmos_sensor(0x0118, 0x0000);			// Sleep On
	write_cmos_sensor(0x0032, 0x0701);		// byrscl_crtl1 / tyrscl_fifo_read_delay
	write_cmos_sensor(0x090A, 0x0200);		// mipi_vblank_delay_h / l
	write_cmos_sensor(0x090C, 0x0003);		// mipi_hblank_short_delay_h / l
	write_cmos_sensor(0x090E, 0x0004);		// mipi_hblank_long_delay_h / l
	write_cmos_sensor(0x0902, 0x4101);		// mipi_tx_op_mode1 / mipi_tx_op_mode2
	write_cmos_sensor(0x0910, 0x5D02);		// mipi_Exit_sequence / mipi_LPX
	write_cmos_sensor(0x0912, 0x020a);		// mipi_CLK_prepare   / mipi_clk_zero
	write_cmos_sensor(0x0914, 0x0202);			//0203);		// mipi_clk_pre / mipi_data_prepare
	write_cmos_sensor(0x0916, 0x0407);			//0406);		 // mipi_data_zero/ mipi_data_trail 
	write_cmos_sensor(0x0918, 0x0b03);			//		 // mipi_clk_post/ mipi_clk_trail
	write_cmos_sensor(0x091A, 0x0300);		// mipi_value_exit / null
	
	write_cmos_sensor(0x0B14, 0x370B);	 
	write_cmos_sensor(0x0B16, 0x450F);			// pll_mipi_clk_div
	write_cmos_sensor(0x0700, 0x30a3);		// byrscl_ctrl1 / byrscl_fifo_read_delay
	write_cmos_sensor(0x0012, 0x00BE);		// x_addr_start_hact_h / l 208
	write_cmos_sensor(0x0018, 0x0AE1);		// x_addr_end_hact_h / l 2769
	write_cmos_sensor(0x0026, 0x002E);		// y_addr_start_vact_h / l 60
	write_cmos_sensor(0x002C, 0x07C9);		// y_addr_end_vact_h / l 1979
	write_cmos_sensor(0x0112, 0x0280);		// x_output_size_h / l 640
	write_cmos_sensor(0x0114, 0x01e0);		// y_output_size_h / l 480
	write_cmos_sensor(0x0006, 0x01F2);		// frame_length_lines_h / l 498 
	write_cmos_sensor(0x0008, 0x0B7C);	 // line_length_pck_h / l 2940 
	
	
	write_cmos_sensor(0x0118, 0x0100);			// Sleep Off

}
static void slim_video_setting()
{
    LOG_INF("E\n");
 
 //=====================================================================
 // Preview mode 
 // Resolution : 1280x720
 // MIPI Speed : 440Mbps x 2lane (PCLK 176Mhz)
 // Lin length 2940 / Frame length 1980
 // Frame rate : max 30.23fps
 //=====================================================================
 write_cmos_sensor(0x0118, 0x0000);   // Sleep On
 write_cmos_sensor(0x0032, 0x0101); 	 // byrscl_crtl1 / tyrscl_fifo_read_delay (add 140714)
 write_cmos_sensor(0x090A, 0x0384); 	 // mipi_vblank_delay_h / l
 write_cmos_sensor(0x090C, 0x000A); 	 // mipi_hblank_short_delay_h / l
 write_cmos_sensor(0x090E, 0x03E8); 	 // mipi_hblank_long_delay_h / l
 write_cmos_sensor(0x0902, 0x4101); 	 // mipi_tx_op_mode1 / mipi_tx_op_mode2
 write_cmos_sensor(0x0910, 0x5D04); 		 //5d03 	 // mipi_value_exit_seq / mipi_value_lpx
 write_cmos_sensor(0x0912, 0x030E); 	 // mipi_value_clk_prepare / mipi_value_clk_zeo
 write_cmos_sensor(0x0914, 0x0404); 		 //0403 	 // mipi_value_clk_pre / mipi_value_data_prepare
 write_cmos_sensor(0x0916, 0x0709); 	 // mipi_value_data_zero / mipi_value_data_trail
 write_cmos_sensor(0x0918, 0x0c05); 		 //0a04 	 // mipi_value_clk_post / mipi_value_clk_trail
 write_cmos_sensor(0x091A, 0x0800); 	 // mipi_value_exit / null
 
 write_cmos_sensor(0x0B14, 0x370B);   
 write_cmos_sensor(0x0B16, 0x448F); 		 // pll_mipi_clk_div
 write_cmos_sensor(0x0700, 0xA0A3); 	 // byrscl_ctrl1 / byrscl_fifo_read_delay
 write_cmos_sensor(0x0012, 0x00BE); 	 // x_addr_start_hact_h / l 208 
 write_cmos_sensor(0x0018, 0x0AE1); 	 // x_addr_end_hact_h / l 2767
 write_cmos_sensor(0x0026, 0x002E); 	 // y_addr_start_vact_h / l 300 
 write_cmos_sensor(0x002C, 0x07C9); 	 // y_addr_end_vact_h / l 1739
 write_cmos_sensor(0x0112, 0x0500); 	 // x_output_size_h / l 1280
 write_cmos_sensor(0x0114, 0x02D0); 	 // y_output_size_h / l 720
 write_cmos_sensor(0x0118, 0x0100); 		 // Sleep Off

}

static kal_uint32 set_test_pattern_mode(kal_bool enable)
{
    LOG_INF("enable: %d\n", enable);

    if (enable) {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
       write_cmos_sensor(0x020a,0x0200);
    } else {
        // 0x5E00[8]: 1 enable,  0 disable
        // 0x5E00[1:0]; 00 Color bar, 01 Random Data, 10 Square, 11 BLACK
       write_cmos_sensor(0x020a,0x0000);
    }
    spin_lock(&imgsensor_drv_lock);
    imgsensor.test_pattern = enable;
    spin_unlock(&imgsensor_drv_lock);
    return ERROR_NONE;
}

/*************************************************************************
* FUNCTION
*	get_imgsensor_id
*
* DESCRIPTION
*	This function get the sensor ID 
*
* PARAMETERS
*	*sensorID : return the sensor ID 
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 get_imgsensor_id(UINT32 *sensor_id) 
{

LOG_INF("[get_imgsensor_id] ");
	kal_uint8 i = 0;
	kal_uint8 retry = 2;
	//sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
	while (imgsensor_info.i2c_addr_table[i] != 0xff) {
			spin_lock(&imgsensor_drv_lock);
			imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
			spin_unlock(&imgsensor_drv_lock);
			do {
            *sensor_id = return_sensor_id();
				if (*sensor_id == imgsensor_info.sensor_id) {				
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
					return ERROR_NONE;
				}	
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,*sensor_id);
				retry--;
			} while(retry > 0);
			i++;
			retry = 2;
}
	if (*sensor_id != imgsensor_info.sensor_id) {
		// if Sensor ID is not correct, Must set *sensor_id to 0xFFFFFFFF 
		*sensor_id = 0xFFFFFFFF;
		return ERROR_SENSOR_CONNECT_FAIL;
	}
	return ERROR_NONE;
}


/*************************************************************************
* FUNCTION
*	open
*
* DESCRIPTION
*	This function initialize the registers of CMOS sensor
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 open(void)
{
    kal_uint8 i = 0;
    kal_uint8 retry = 2;
    kal_uint32 sensor_id = 0;
    LOG_1;
    LOG_2;
    //sensor have two i2c address 0x6c 0x6d & 0x21 0x20, we should detect the module used i2c address
    LOG_INF("Enter open\n");
    while (imgsensor_info.i2c_addr_table[i] != 0xff) {
        spin_lock(&imgsensor_drv_lock);
        imgsensor.i2c_write_id = imgsensor_info.i2c_addr_table[i];
        spin_unlock(&imgsensor_drv_lock);
        do {
            sensor_id = return_sensor_id();
            if (sensor_id == imgsensor_info.sensor_id) {
                LOG_INF("i2c write id: 0x%x, sensor id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
                break;
            }
            LOG_INF("Read sensor id fail, write id: 0x%x, id: 0x%x\n", imgsensor.i2c_write_id,sensor_id);
            retry--;
        } while(retry > 0);
        i++;
        if (sensor_id == imgsensor_info.sensor_id)
            break;
        retry = 2;
    }
    if (imgsensor_info.sensor_id != sensor_id)
        return ERROR_SENSOR_CONNECT_FAIL;

    /* initail sequence write in  */
    sensor_init();
   //liuying 20150302 add begin
   LOG_INF("OTP begin\n");
   #ifdef HI545_USE_AWB_OTP
   	Hi545OTPSetting();
	Hi545_Sensor_OTP_info();
	Hi545_Sensor_calc_wbdata(&current_otp);
   if(current_otp.awb_status == 1)
   {
   	Hi545_Sensor_update_wb_gain(&current_otp);
   }
   #endif
   LOG_INF("OTP end\n");
   //liuying 20150302 add end

    spin_lock(&imgsensor_drv_lock);

    imgsensor.autoflicker_en= KAL_FALSE;
    imgsensor.sensor_mode = IMGSENSOR_MODE_INIT;
    imgsensor.pclk = imgsensor_info.pre.pclk;
    imgsensor.frame_length = imgsensor_info.pre.framelength;
    imgsensor.line_length = imgsensor_info.pre.linelength;
    imgsensor.min_frame_length = imgsensor_info.pre.framelength;
    imgsensor.dummy_pixel = 0;
    imgsensor.dummy_line = 0;
    imgsensor.ihdr_en = 0;
    imgsensor.test_pattern = KAL_FALSE;
    imgsensor.current_fps = imgsensor_info.pre.max_framerate;
    spin_unlock(&imgsensor_drv_lock);

    LOG_INF("Exit open\n");
	
    return ERROR_NONE;
}    /*    open  */



/*************************************************************************
* FUNCTION
*	close
*
* DESCRIPTION
*	
*
* PARAMETERS
*	None
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 close(void)
{
	LOG_INF("close E");

	/*No Need to implement this function*/ 
	
	return ERROR_NONE;
}	/*	close  */


/*************************************************************************
* FUNCTION
* preview
*
* DESCRIPTION
*	This function start the sensor preview.
*
* PARAMETERS
*	*image_window : address pointer of pixel numbers in one period of HSYNC
*  *sensor_config_data : address pointer of line numbers in one period of VSYNC
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 preview(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("Enter preview\n");

	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
    	set_mirror_flip(IMAGE_H_MIRROR);
	LOG_INF("Exit  preview\n");
	return ERROR_NONE;
}	/*	preview   */

/*************************************************************************
* FUNCTION
*	capture
*
* DESCRIPTION
*	This function setup the CMOS sensor in capture MY_OUTPUT mode
*
* PARAMETERS
*
* RETURNS
*	None
*
* GLOBALS AFFECTED
*
*************************************************************************/
static kal_uint32 capture(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
						  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_CAPTURE;
	imgsensor.current_fps = 300;
	
		imgsensor.pclk = imgsensor_info.cap.pclk;
		imgsensor.line_length = imgsensor_info.cap.linelength;
		imgsensor.frame_length = imgsensor_info.cap.framelength;  
		imgsensor.min_frame_length = imgsensor_info.cap.framelength;
		imgsensor.current_fps = imgsensor_info.cap.max_framerate;
		//imgsensor.current_fps = imgsensor_info.cap.max_framerate;
		imgsensor.autoflicker_en = KAL_FALSE;
	
	spin_unlock(&imgsensor_drv_lock);

	capture_setting(imgsensor.current_fps); 
	
    set_mirror_flip(IMAGE_H_MIRROR);
	return ERROR_NONE;
}	/* capture() */
static kal_uint32 normal_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("E");
	
	spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_VIDEO;
	imgsensor.pclk = imgsensor_info.normal_video.pclk;
	imgsensor.line_length = imgsensor_info.normal_video.linelength;
	imgsensor.frame_length = imgsensor_info.normal_video.framelength;  
	imgsensor.min_frame_length = imgsensor_info.normal_video.framelength;
	imgsensor.current_fps = 300;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	normal_video_setting(imgsensor.current_fps);
	
    set_mirror_flip(IMAGE_H_MIRROR);
	return ERROR_NONE;
}	/*	normal_video   */

static kal_uint32 hs_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");

    spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_HIGH_SPEED_VIDEO;
    imgsensor.pclk = imgsensor_info.hs_video.pclk;
    //imgsensor.video_mode = KAL_TRUE;
    imgsensor.line_length = imgsensor_info.hs_video.linelength;
    imgsensor.frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.hs_video.framelength;
    imgsensor.dummy_line = 0;
    imgsensor.dummy_pixel = 0;
    imgsensor.autoflicker_en = KAL_FALSE;
    spin_unlock(&imgsensor_drv_lock);
    hs_video_setting();	
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
    return ERROR_NONE;
}    /*    hs_video   */

static kal_uint32 slim_video(MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("E\n");
spin_lock(&imgsensor_drv_lock);
    imgsensor.sensor_mode = IMGSENSOR_MODE_SLIM_VIDEO;
    imgsensor.pclk = imgsensor_info.slim_video.pclk;
	
    imgsensor.line_length = imgsensor_info.slim_video.linelength;
    imgsensor.frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.min_frame_length = imgsensor_info.slim_video.framelength;
    imgsensor.current_fps = imgsensor_info.slim_video.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
   
    spin_unlock(&imgsensor_drv_lock);
    slim_video_setting();
	//set_mirror_flip(sensor_config_data->SensorImageMirror);
	set_mirror_flip(IMAGE_H_MIRROR);

    return ERROR_NONE;
    
    /*
    spin_lock(&imgsensor_drv_lock);
	imgsensor.sensor_mode = IMGSENSOR_MODE_PREVIEW;
	imgsensor.pclk = imgsensor_info.pre.pclk;
	//imgsensor.video_mode = KAL_FALSE;
	imgsensor.line_length = imgsensor_info.pre.linelength;
	imgsensor.frame_length = imgsensor_info.pre.framelength; 
	imgsensor.min_frame_length = imgsensor_info.pre.framelength;
	imgsensor.current_fps = imgsensor_info.pre.max_framerate;
	imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	preview_setting();
    set_mirror_flip(IMAGE_H_MIRROR);
	return ERROR_NONE;*/
}   






static kal_uint32 get_resolution(MSDK_SENSOR_RESOLUTION_INFO_STRUCT *sensor_resolution)
{
    LOG_INF("E\n");
    sensor_resolution->SensorFullWidth = imgsensor_info.cap.grabwindow_width;
    sensor_resolution->SensorFullHeight = imgsensor_info.cap.grabwindow_height;

    sensor_resolution->SensorPreviewWidth = imgsensor_info.pre.grabwindow_width;
    sensor_resolution->SensorPreviewHeight = imgsensor_info.pre.grabwindow_height;

    sensor_resolution->SensorVideoWidth = imgsensor_info.normal_video.grabwindow_width;
    sensor_resolution->SensorVideoHeight = imgsensor_info.normal_video.grabwindow_height;


    sensor_resolution->SensorHighSpeedVideoWidth     = imgsensor_info.hs_video.grabwindow_width;
    sensor_resolution->SensorHighSpeedVideoHeight     = imgsensor_info.hs_video.grabwindow_height;

    sensor_resolution->SensorSlimVideoWidth     = imgsensor_info.slim_video.grabwindow_width;
    sensor_resolution->SensorSlimVideoHeight     = imgsensor_info.slim_video.grabwindow_height;
    return ERROR_NONE;
}    /*    get_resolution    */


static kal_uint32 get_info(MSDK_SCENARIO_ID_ENUM scenario_id,
                      MSDK_SENSOR_INFO_STRUCT *sensor_info,
                      MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
    LOG_INF("scenario_id = %d\n", scenario_id);


    //sensor_info->SensorVideoFrameRate = imgsensor_info.normal_video.max_framerate/10; /* not use */
    //sensor_info->SensorStillCaptureFrameRate= imgsensor_info.cap.max_framerate/10; /* not use */
    //imgsensor_info->SensorWebCamCaptureFrameRate= imgsensor_info.v.max_framerate; /* not use */

    sensor_info->SensorClockPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorClockFallingPolarity = SENSOR_CLOCK_POLARITY_LOW; /* not use */
    sensor_info->SensorHsyncPolarity = SENSOR_CLOCK_POLARITY_LOW; // inverse with datasheet
    sensor_info->SensorVsyncPolarity = SENSOR_CLOCK_POLARITY_LOW;
    sensor_info->SensorInterruptDelayLines = 4; /* not use */
    sensor_info->SensorResetActiveHigh = FALSE; /* not use */
    sensor_info->SensorResetDelayCount = 5; /* not use */

    sensor_info->SensroInterfaceType = imgsensor_info.sensor_interface_type;
    sensor_info->MIPIsensorType = imgsensor_info.mipi_sensor_type;
    sensor_info->SettleDelayMode = imgsensor_info.mipi_settle_delay_mode;
    sensor_info->SensorOutputDataFormat = imgsensor_info.sensor_output_dataformat;

    sensor_info->CaptureDelayFrame = imgsensor_info.cap_delay_frame;
    sensor_info->PreviewDelayFrame = imgsensor_info.pre_delay_frame;
    sensor_info->VideoDelayFrame = imgsensor_info.video_delay_frame;
    sensor_info->HighSpeedVideoDelayFrame = imgsensor_info.hs_video_delay_frame;
    sensor_info->SlimVideoDelayFrame = imgsensor_info.slim_video_delay_frame;

    sensor_info->SensorMasterClockSwitch = 0; /* not use */
    sensor_info->SensorDrivingCurrent = imgsensor_info.isp_driving_current;

    sensor_info->AEShutDelayFrame = imgsensor_info.ae_shut_delay_frame;          /* The frame of setting shutter default 0 for TG int */
    sensor_info->AESensorGainDelayFrame = imgsensor_info.ae_sensor_gain_delay_frame;    /* The frame of setting sensor gain */
    sensor_info->AEISPGainDelayFrame = imgsensor_info.ae_ispGain_delay_frame;
    sensor_info->IHDR_Support = imgsensor_info.ihdr_support;
    sensor_info->IHDR_LE_FirstLine = imgsensor_info.ihdr_le_firstline;
    sensor_info->SensorModeNum = imgsensor_info.sensor_mode_num;

    sensor_info->SensorMIPILaneNumber = imgsensor_info.mipi_lane_num;
    sensor_info->SensorClockFreq = imgsensor_info.mclk;
    sensor_info->SensorClockDividCount = 3; /* not use */
    sensor_info->SensorClockRisingCount = 0;
    sensor_info->SensorClockFallingCount = 2; /* not use */
    sensor_info->SensorPixelClockCount = 3; /* not use */
    sensor_info->SensorDataLatchCount = 2; /* not use */

    sensor_info->MIPIDataLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->MIPICLKLowPwr2HighSpeedTermDelayCount = 0;
    sensor_info->SensorWidthSampling = 0;  // 0 is default 1x
    sensor_info->SensorHightSampling = 0;    // 0 is default 1x
    sensor_info->SensorPacketECCOrder = 1;

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            sensor_info->SensorGrabStartX = imgsensor_info.cap.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.cap.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.cap.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:

            sensor_info->SensorGrabStartX = imgsensor_info.normal_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.normal_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.normal_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.hs_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.hs_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.hs_video.mipi_data_lp2hs_settle_dc;

            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            sensor_info->SensorGrabStartX = imgsensor_info.slim_video.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.slim_video.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.slim_video.mipi_data_lp2hs_settle_dc;

            break;
        default:
            sensor_info->SensorGrabStartX = imgsensor_info.pre.startx;
            sensor_info->SensorGrabStartY = imgsensor_info.pre.starty;

            sensor_info->MIPIDataLowPwr2HighSpeedSettleDelayCount = imgsensor_info.pre.mipi_data_lp2hs_settle_dc;
            break;
    }

    return ERROR_NONE;
}    /*    get_info  */


static kal_uint32 control(MSDK_SCENARIO_ID_ENUM scenario_id, MSDK_SENSOR_EXPOSURE_WINDOW_STRUCT *image_window,
					  MSDK_SENSOR_CONFIG_STRUCT *sensor_config_data)
{
	LOG_INF("[contrlo]scenario_id = %d", scenario_id);
	spin_lock(&imgsensor_drv_lock);
	imgsensor.current_scenario_id = scenario_id;
	spin_unlock(&imgsensor_drv_lock);
	switch (scenario_id) {
		case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
		
			LOG_INF("preview\n");
			preview(image_window, sensor_config_data);
			break;
		case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
			case MSDK_SCENARIO_ID_CAMERA_ZSD:
			capture(image_window, sensor_config_data);
			break;	
		case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
			normal_video(image_window, sensor_config_data);
			break;	  
		case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            hs_video(image_window, sensor_config_data);
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            slim_video(image_window, sensor_config_data);
            break;	  
		default:
			LOG_INF("Error ScenarioId setting");
			preview(image_window, sensor_config_data);
			return ERROR_INVALID_SCENARIO_ID;
	}
	return ERROR_NONE;
}	/* control() */



static kal_uint32 set_video_mode(UINT16 framerate)
{
	LOG_INF("framerate = %d ", framerate);
	// SetVideoMode Function should fix framerate
	if (framerate == 0)
		// Dynamic frame rate
		return ERROR_NONE;
	spin_lock(&imgsensor_drv_lock);
	
    if ((framerate == 300) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 296;
    else if ((framerate == 150) && (imgsensor.autoflicker_en == KAL_TRUE))
		imgsensor.current_fps = 146;
	else
        imgsensor.current_fps = framerate;
	spin_unlock(&imgsensor_drv_lock);
	set_max_framerate(imgsensor.current_fps,1);

	return ERROR_NONE;
}

static kal_uint32 set_auto_flicker_mode(kal_bool enable, UINT16 framerate)
{

	
	LOG_INF("enable = %d, framerate = %d ", enable, framerate);
	spin_lock(&imgsensor_drv_lock);
	if (enable) 	  
		imgsensor.autoflicker_en = KAL_TRUE;
	else //Cancel Auto flick
		imgsensor.autoflicker_en = KAL_FALSE;
	spin_unlock(&imgsensor_drv_lock);
	return ERROR_NONE;
}


static kal_uint32 set_max_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 framerate)
{
    kal_uint32 frame_length;

    LOG_INF("scenario_id = %d, framerate = %d\n", scenario_id, framerate);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
         //   set_dummy();
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            if(framerate == 0)
                return ERROR_NONE;
            frame_length = imgsensor_info.normal_video.pclk / framerate * 10 / imgsensor_info.normal_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.normal_video.framelength) ? (frame_length - imgsensor_info.normal_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.normal_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
            //set_dummy();
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
        	  if (imgsensor.current_fps == imgsensor_info.cap1.max_framerate) {
                frame_length = imgsensor_info.cap1.pclk / framerate * 10 / imgsensor_info.cap1.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap1.framelength) ? (frame_length - imgsensor_info.cap1.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap1.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            } else {
        		    if (imgsensor.current_fps != imgsensor_info.cap.max_framerate)
                    LOG_INF("Warning: current_fps %d fps is not support, so use cap's setting: %d fps!\n",framerate,imgsensor_info.cap.max_framerate/10);
                frame_length = imgsensor_info.cap.pclk / framerate * 10 / imgsensor_info.cap.linelength;
                spin_lock(&imgsensor_drv_lock);
		            imgsensor.dummy_line = (frame_length > imgsensor_info.cap.framelength) ? (frame_length - imgsensor_info.cap.framelength) : 0;
		            imgsensor.frame_length = imgsensor_info.cap.framelength + imgsensor.dummy_line;
		            imgsensor.min_frame_length = imgsensor.frame_length;
		            spin_unlock(&imgsensor_drv_lock);
            }
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            frame_length = imgsensor_info.hs_video.pclk / framerate * 10 / imgsensor_info.hs_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.hs_video.framelength) ? (frame_length - imgsensor_info.hs_video.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.hs_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            frame_length = imgsensor_info.slim_video.pclk / framerate * 10 / imgsensor_info.slim_video.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.slim_video.framelength) ? (frame_length - imgsensor_info.slim_video.framelength): 0;
            imgsensor.frame_length = imgsensor_info.slim_video.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            break;
        default:  //coding with  preview scenario by default
            frame_length = imgsensor_info.pre.pclk / framerate * 10 / imgsensor_info.pre.linelength;
            spin_lock(&imgsensor_drv_lock);
            imgsensor.dummy_line = (frame_length > imgsensor_info.pre.framelength) ? (frame_length - imgsensor_info.pre.framelength) : 0;
            imgsensor.frame_length = imgsensor_info.pre.framelength + imgsensor.dummy_line;
            imgsensor.min_frame_length = imgsensor.frame_length;
            spin_unlock(&imgsensor_drv_lock);
          //  set_dummy();
            LOG_INF("error scenario_id = %d, we use preview scenario \n", scenario_id);
            break;
    }
    return ERROR_NONE;
}


static kal_uint32 get_default_framerate_by_scenario(MSDK_SCENARIO_ID_ENUM scenario_id, MUINT32 *framerate)
{
    LOG_INF("scenario_id = %d\n", scenario_id);

    switch (scenario_id) {
        case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
            *framerate = imgsensor_info.pre.max_framerate;
            break;
        case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
            *framerate = imgsensor_info.normal_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
            *framerate = imgsensor_info.cap.max_framerate;
            break;
        case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
            *framerate = imgsensor_info.hs_video.max_framerate;
            break;
        case MSDK_SCENARIO_ID_SLIM_VIDEO:
            *framerate = imgsensor_info.slim_video.max_framerate;
            break;
        default:
            break;
    }

    return ERROR_NONE;
}

static kal_uint32 feature_control(MSDK_SENSOR_FEATURE_ENUM feature_id,
                             UINT8 *feature_para,UINT32 *feature_para_len)
{
    UINT16 *feature_return_para_16=(UINT16 *) feature_para;
    UINT16 *feature_data_16=(UINT16 *) feature_para;
    UINT32 *feature_return_para_32=(UINT32 *) feature_para;
    UINT32 *feature_data_32=(UINT32 *) feature_para;
    unsigned long long *feature_data=(unsigned long long *) feature_para;
    unsigned long long *feature_return_para=(unsigned long long *) feature_para;

    SENSOR_WINSIZE_INFO_STRUCT *wininfo;
    MSDK_SENSOR_REG_INFO_STRUCT *sensor_reg_data=(MSDK_SENSOR_REG_INFO_STRUCT *) feature_para;

    printk("feature_id = %d\n", feature_id);
    switch (feature_id) {
        case SENSOR_FEATURE_GET_PERIOD:
            *feature_return_para_16++ = imgsensor.line_length;
            *feature_return_para_16 = imgsensor.frame_length;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_GET_PIXEL_CLOCK_FREQ:
            LOG_INF("feature_Control imgsensor.pclk = %d,imgsensor.current_fps = %d\n", imgsensor.pclk,imgsensor.current_fps);
            *feature_return_para_32 = imgsensor.pclk;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_ESHUTTER:
            set_shutter(*feature_data);
            break;
        case SENSOR_FEATURE_SET_NIGHTMODE:
            night_mode((BOOL) *feature_data);
            break;
        case SENSOR_FEATURE_SET_GAIN:
            set_gain((UINT16) *feature_data);
            break;
        case SENSOR_FEATURE_SET_FLASHLIGHT:
            break;
        case SENSOR_FEATURE_SET_ISP_MASTER_CLOCK_FREQ:
            break;
        case SENSOR_FEATURE_SET_REGISTER:
            write_cmos_sensor(sensor_reg_data->RegAddr, sensor_reg_data->RegData);
            break;
        case SENSOR_FEATURE_GET_REGISTER:
            sensor_reg_data->RegData = read_cmos_sensor(sensor_reg_data->RegAddr);
            break;
        case SENSOR_FEATURE_GET_LENS_DRIVER_ID:
            // get the lens driver ID from EEPROM or just return LENS_DRIVER_ID_DO_NOT_CARE
            // if EEPROM does not exist in camera module.
            *feature_return_para_32=LENS_DRIVER_ID_DO_NOT_CARE;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_VIDEO_MODE:
            set_video_mode(*feature_data);
            break;
        case SENSOR_FEATURE_CHECK_SENSOR_ID:
            get_imgsensor_id(feature_return_para_32);
            break;
        case SENSOR_FEATURE_SET_AUTO_FLICKER_MODE:
            set_auto_flicker_mode((BOOL)*feature_data_16,*(feature_data_16+1));
            break;
        case SENSOR_FEATURE_SET_MAX_FRAME_RATE_BY_SCENARIO:
            set_max_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*feature_data, *(feature_data+1));
            break;
        case SENSOR_FEATURE_GET_DEFAULT_FRAME_RATE_BY_SCENARIO:
            get_default_framerate_by_scenario((MSDK_SCENARIO_ID_ENUM)*(feature_data), (MUINT32 *)(uintptr_t)(*(feature_data+1)));           
            break;
        case SENSOR_FEATURE_SET_TEST_PATTERN:
            set_test_pattern_mode((BOOL)*feature_data);
            break;
        case SENSOR_FEATURE_GET_TEST_PATTERN_CHECKSUM_VALUE: //for factory mode auto testing
            *feature_return_para_32 = imgsensor_info.checksum_value;
            *feature_para_len=4;
            break;
        case SENSOR_FEATURE_SET_FRAMERATE:
            LOG_INF("current fps :%d\n", *feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.current_fps = *feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_SET_HDR:
            LOG_INF("ihdr enable :%d\n", (BOOL)*feature_data);
            spin_lock(&imgsensor_drv_lock);
            imgsensor.ihdr_en = (BOOL)*feature_data;
            spin_unlock(&imgsensor_drv_lock);
            break;
        case SENSOR_FEATURE_GET_CROP_INFO:
            LOG_INF("SENSOR_FEATURE_GET_CROP_INFO scenarioId:%d\n", *feature_data);

            wininfo = (SENSOR_WINSIZE_INFO_STRUCT *)(uintptr_t)(*(feature_data+1));

            switch (*feature_data_32) {
                case MSDK_SCENARIO_ID_CAMERA_CAPTURE_JPEG:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[1],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_VIDEO_PREVIEW:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[2],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_HIGH_SPEED_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[3],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_SLIM_VIDEO:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[4],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
                case MSDK_SCENARIO_ID_CAMERA_PREVIEW:
                default:
                    memcpy((void *)wininfo,(void *)&imgsensor_winsize_info[0],sizeof(SENSOR_WINSIZE_INFO_STRUCT));
                    break;
            }
        case SENSOR_FEATURE_SET_IHDR_SHUTTER_GAIN:
            LOG_INF("SENSOR_SET_SENSOR_IHDR LE=%d, SE=%d, Gain=%d\n",(UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            ihdr_write_shutter_gain((UINT16)*feature_data,(UINT16)*(feature_data+1),(UINT16)*(feature_data+2));
            break;
        default:
            break;
    }

    return ERROR_NONE;
}   /*  feature_control()  */

static SENSOR_FUNCTION_STRUCT sensor_func = {
    open,
    get_info,
    get_resolution,
    feature_control,
    control,
    close
};

UINT32 HI551_MIPI_RAW_SensorInit(PSENSOR_FUNCTION_STRUCT *pfFunc)
{
	/* To Do : Check Sensor status here */
	if (pfFunc!=NULL)
		*pfFunc=&sensor_func;
	return ERROR_NONE;
}	/*	HI841_MIPI_RAW_SensorInit	*/
/*
	liuying 20150908 add end
*/