//lichengmin begin
/* KXCNL motion sensor driver
 *
 *
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kobject.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <asm/atomic.h>
#include <linux/version.h>

#include <mach/mt_typedefs.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <cust_eint.h>

#include <accel.h>
#include <linux/batch.h>

#define POWER_NONE_MACRO MT65XX_POWER_NONE
#define KIONIX_KXCNL_HW_PEDOMETER	1	/* 1:enable KXCNL hardware pedometer, 0: disable */

#include <cust_acc.h>
#include <linux/hwmsensor.h>
#include <linux/hwmsen_dev.h>
#include <linux/sensors_io.h>
#include "kxcnl.h"
#include "kxcnl_regs.h"
#include <linux/hwmsen_helper.h>
#if  1//def KIONIX_KXCNL_HW_PEDOMETER
#include "../../step_counter/step_counter.h"
#endif

/*----------------------------------------------------------------------------*/
#define DEBUG 1
/*----------------------------------------------------------------------------*/
//#define CONFIG_KXCNL_LOWPASS   /*apply low pass filter on output*/       
#define SW_CALIBRATION
//#define USE_EARLY_SUSPEND
/*----------------------------------------------------------------------------*/
#define KXCNL_AXIS_X          (0)
#define KXCNL_AXIS_Y          (1)
#define KXCNL_AXIS_Z          (2)
#define KXCNL_DATA_LEN        (6)
#define KXCNL_DEV_NAME        "KXCNL"
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id kxcnl_i2c_id[] = {{KXCNL_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_kxcnl={ I2C_BOARD_INFO(KXCNL_DEV_NAME, (KXCNL_I2C_SLAVE_ADDR>>1))};
/*the adapter id will be available in customization*/
//static unsigned short kxcnl_force[] = {0x00, KXCNL_I2C_SLAVE_ADDR, I2C_CLIENT_END, I2C_CLIENT_END};
//static const unsigned short *const kxcnl_forces[] = { kxcnl_force, NULL };
//static struct i2c_client_address_data kxcnl_addr_data = { .forces = kxcnl_forces,};

/*----------------------------------------------------------------------------*/
static int kxcnl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id); 
static int kxcnl_i2c_remove(struct i2c_client *client);
//static int kxcnl_i2c_detect(struct i2c_client *client, int kind, struct i2c_board_info *info);
static int kxcnl_suspend(struct i2c_client *client, pm_message_t msg);
static int kxcnl_resume(struct i2c_client *client);

static int kxcnl_local_init(void);
static int kxcnl_remove(void);

extern struct acc_hw* kxcnl_get_cust_acc_hw(void);

/*----------------------------------------------------------------------------*/
typedef enum {
    ADX_TRC_FILTER  = 0x01,
    ADX_TRC_RAWDATA = 0x02,
    ADX_TRC_IOCTL   = 0x04,
    ADX_TRC_CALI	= 0X08,
    ADX_TRC_INFO	= 0X10,
} ADX_TRC;
/*----------------------------------------------------------------------------*/
struct scale_factor{
    u8  whole;
    u8  fraction;
};
/*----------------------------------------------------------------------------*/
struct data_resolution {
    struct scale_factor scalefactor;
    int                 sensitivity;
};
/*----------------------------------------------------------------------------*/
#define C_MAX_FIR_LENGTH (32)
/*----------------------------------------------------------------------------*/
struct data_filter {
    s16 raw[C_MAX_FIR_LENGTH][KXCNL_AXES_NUM];
    int sum[KXCNL_AXES_NUM];
    int num;
    int idx;
};
/*----------------------------------------------------------------------------*/
struct kxcnl_i2c_data {
    struct i2c_client *client;
    struct acc_hw *hw;
    struct hwmsen_convert   cvt;

#if  1//def KIONIX_KXCNL_HW_PEDOMETER
    struct work_struct	eint_work;
#endif
    
    /*misc*/
    struct data_resolution *reso;
    atomic_t                trace;
    atomic_t                suspend;
    atomic_t                selftest;
	atomic_t				filter;
    s16                     cali_sw[KXCNL_AXES_NUM+1];

    /*data*/
    s8                      offset[KXCNL_AXES_NUM+1];  /*+1: for 4-byte alignment*/
    s16                     data[KXCNL_AXES_NUM+1];

#if defined(CONFIG_KXCNL_LOWPASS)
    atomic_t                firlen;
    atomic_t                fir_en;
    struct data_filter      fir;
#endif 
    /*early suspend*/
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
    struct early_suspend    early_drv;
#endif     

#if  1//KIONIX_KXCNL_HW_PEDOMETER
	u8   pedo_enabled;
	u8	pedo_mode;
	u16 current_step;
	u16 irq1;
#endif
};
/*----------------------------------------------------------------------------*/
static struct i2c_driver kxcnl_i2c_driver = {
    .driver = {
//        .owner          = THIS_MODULE,
        .name           = KXCNL_DEV_NAME,
    },
	.probe      		= kxcnl_i2c_probe,
	.remove    			= kxcnl_i2c_remove,
//	.detect				= kxcnl_i2c_detect,
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
    .suspend            = kxcnl_suspend,
    .resume             = kxcnl_resume,
#endif
	.id_table = kxcnl_i2c_id,
//	.address_data = &kxcnl_addr_data,
};

/*----------------------------------------------------------------------------*/
static struct i2c_client *kxcnl_i2c_client = NULL;
static struct kxcnl_i2c_data *obj_i2c_data = NULL;
static bool sensor_power = false;
static GSENSOR_VECTOR3D gsensor_gain={0};
static char selftestRes[8]= {0}; 
static DEFINE_MUTEX(kxcnl_mutex);
static bool g_bGsensorOnoff = false;


#define KXCNL_INIT_NODO    1
#define KXCNL_INIT_OK   0
#define KXCNL_INIT_FAIL   -1
static int kxcnl_init_flag =KXCNL_INIT_NODO; // 0<==>OK -1 <==> fail  1<==>nodo

static struct acc_init_info kxcnl_init_info = {
		.name = "kxcnl",
		.init = kxcnl_local_init,
		.uninit = kxcnl_remove,
	
};

#if  1//def KIONIX_KXCNL_HW_PEDOMETER
static int kxcnl_step_c_local_init(void);
static int kxcnl_step_c_local_uninit(void);
static struct step_c_init_info  kxcnl_step_c_init_info = {
	.name   = "KXCNL_STEP_C",
	.init   = kxcnl_step_c_local_init,
	.uninit = kxcnl_step_c_local_uninit,
};
#endif

/*----------------------------------------------------------------------------*/
#define GSE_TAG                  "[Gsensor] "
#define GSE_FUN(f)               printk( GSE_TAG"%s\n", __FUNCTION__)
#define GSE_ERR(fmt, args...)    printk(KERN_ERR GSE_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define GSE_LOG(fmt, args...)    printk( GSE_TAG fmt, ##args)
/*----------------------------------------------------------------------------*/
static struct data_resolution kxcnl_data_resolution[1] = {
 /* combination by {FULL_RES,RANGE}*/
    {{ 0, 9}, 1024}, // dataformat +/-2g  in 12-bit resolution;  { 3, 9} = 3.9 = (2*2*1000)/(2^12);  256 = (2^12)/(2*2)          
};
/*----------------------------------------------------------------------------*/
static struct data_resolution kxcnl_offset_resolution = {{15, 6}, 64};
/*----------------------------------------------------------------------------*/
static int KXCNL_SetPowerMode(struct i2c_client *client, bool enable);
/*--------------------KXCNL power control function----------------------------------*/
static void KXCNL_power(struct acc_hw *hw, unsigned int on) 
{
	static unsigned int power_on = 0;
       GSE_LOG("KXCNL_power: %d\n", on);

	if(hw->power_id != POWER_NONE_MACRO)		// have externel LDO
	{        
		GSE_LOG("power %s\n", on ? "on" : "off");
		if(power_on == on)	// power status not change
		{
			GSE_LOG("ignore power control: %d\n", on);
		}
		else if(on)	// power on
		{
			if(!hwPowerOn(hw->power_id, hw->power_vol, "KXCNL"))
			{
				GSE_ERR("power on fails!!\n");
			}
		}
		else	// power off
		{
			if (!hwPowerDown(hw->power_id, "KXCNL"))
			{
				GSE_ERR("power off fail!!\n");
			}			  
		}
	}
	power_on = on;    
}
/*----------------------------------------------------------------------------*/
//	For KXCNL, always 12bit resolution, no setting needed
/*----------------------------------------------------------------------------*/
static int KXCNL_SetDataResolution(struct kxcnl_i2c_data *obj)
{
#if 0
	int err;
	u8  databuf[2];
	bool cur_sensor_power = sensor_power;

	KXCNL_SetPowerMode(obj->client, false);

	if(hwmsen_read_block(obj->client, KXCNL_REG_DATA_RESOLUTION, databuf, 0x01))
	{
		printk("kxcnl read Dataformat failt \n");
		return KXCNL_ERR_I2C;
	}

	databuf[0] &= ~KXCNL_RANGE_DATA_RESOLUTION_MASK;
	databuf[0] |= KXCNL_RANGE_DATA_RESOLUTION_MASK;//12bit
	databuf[1] = databuf[0];
	databuf[0] = KXCNL_REG_DATA_RESOLUTION;


	err = i2c_master_send(obj->client, databuf, 0x2);

	if(err <= 0)
	{
		return KXCNL_ERR_I2C;
	}

	KXCNL_SetPowerMode(obj->client, cur_sensor_power/*true*/);
#endif 

	//kxcnl_data_resolution[0] has been set when initialize: +/-2g  in 8-bit resolution:  15.6 mg/LSB   
	obj->reso = &kxcnl_data_resolution[0];
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_ReadData(struct i2c_client *client, s16 data[KXCNL_AXES_NUM])
{
	struct kxcnl_i2c_data *priv = i2c_get_clientdata(client);        
	int err = 0;
	u8 addr = KXCNL_REG_DATAX0;
	u8 buf[KXCNL_DATA_LEN] = {0};

	if(NULL == client)
	{
		return -EINVAL;
	}

	if((err = hwmsen_read_block(client, addr, buf, 0x06)) != 0)
	{
		GSE_ERR("error: %d\n", err);
		return err;
	}

	data[KXCNL_AXIS_X] = (s16)(buf[KXCNL_AXIS_X*2]|(buf[KXCNL_AXIS_X*2+1]<<8));
	data[KXCNL_AXIS_Y] = (s16)(buf[KXCNL_AXIS_Y*2] |(buf[KXCNL_AXIS_Y*2+1]<<8));
	data[KXCNL_AXIS_Z] = (s16)(buf[KXCNL_AXIS_Z*2]|(buf[KXCNL_AXIS_Z*2+1]<<8));

	if(atomic_read(&priv->trace) & ADX_TRC_RAWDATA)
	{
		GSE_LOG("[%08X %08X %08X] => [%5d %5d %5d]\n", data[KXCNL_AXIS_X], data[KXCNL_AXIS_Y], data[KXCNL_AXIS_Z],
	                               data[KXCNL_AXIS_X], data[KXCNL_AXIS_Y], data[KXCNL_AXIS_Z]);
	}
	
#ifdef CONFIG_KXCNL_LOWPASS
	if(atomic_read(&priv->filter))
	{
		if(atomic_read(&priv->fir_en) && !atomic_read(&priv->suspend))
		{
			int idx, firlen = atomic_read(&priv->firlen);   
			if(priv->fir.num < firlen)
			{                
				priv->fir.raw[priv->fir.num][KXCNL_AXIS_X] = data[KXCNL_AXIS_X];
				priv->fir.raw[priv->fir.num][KXCNL_AXIS_Y] = data[KXCNL_AXIS_Y];
				priv->fir.raw[priv->fir.num][KXCNL_AXIS_Z] = data[KXCNL_AXIS_Z];
				priv->fir.sum[KXCNL_AXIS_X] += data[KXCNL_AXIS_X];
				priv->fir.sum[KXCNL_AXIS_Y] += data[KXCNLIK_AXIS_Y];
				priv->fir.sum[KXCNL_AXIS_Z] += data[KXCNL_AXIS_Z];
				if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
				{
					GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d]\n", priv->fir.num,
						priv->fir.raw[priv->fir.num][KXCNL_AXIS_X], priv->fir.raw[priv->fir.num][KXCNL_AXIS_Y], priv->fir.raw[priv->fir.num][KXCNL_AXIS_Z],
						priv->fir.sum[KXCNL_AXIS_X], priv->fir.sum[KXCNL_AXIS_Y], priv->fir.sum[KXCNL_AXIS_Z]);
				}
				priv->fir.num++;
				priv->fir.idx++;
			}
			else
			{
				idx = priv->fir.idx % firlen;
				priv->fir.sum[KXCNL_AXIS_X] -= priv->fir.raw[idx][KXCNL_AXIS_X];
				priv->fir.sum[KXCNL_AXIS_Y] -= priv->fir.raw[idx][KXCNL_AXIS_Y];
				priv->fir.sum[KXCNL_AXIS_Z] -= priv->fir.raw[idx][KXCNL_AXIS_Z];
				priv->fir.raw[idx][KXCNL_AXIS_X] = data[KXCNL_AXIS_X];
				priv->fir.raw[idx][KXCNL_AXIS_Y] = data[KXCNL_AXIS_Y];
				priv->fir.raw[idx][KXCNL_AXIS_Z] = data[KXCNL_AXIS_Z];
				priv->fir.sum[KXCNL_AXIS_X] += data[KXCNL_AXIS_X];
				priv->fir.sum[KXCNL_AXIS_Y] += data[KXCNL_AXIS_Y];
				priv->fir.sum[KXCNL_AXIS_Z] += data[KXCNL_AXIS_Z];
				priv->fir.idx++;
				data[KXCNL_AXIS_X] = priv->fir.sum[KXCNL_AXIS_X]/firlen;
				data[KXCNL_AXIS_Y] = priv->fir.sum[KXCNL_AXIS_Y]/firlen;
				data[KXCNL_AXIS_Z] = priv->fir.sum[KXCNL_AXIS_Z]/firlen;
				if(atomic_read(&priv->trace) & ADX_TRC_FILTER)
				{
					GSE_LOG("add [%2d] [%5d %5d %5d] => [%5d %5d %5d] : [%5d %5d %5d]\n", idx,
					priv->fir.raw[idx][KXCNL_AXIS_X], priv->fir.raw[idx][KXCNL_AXIS_Y], priv->fir.raw[idx][KXCNL_AXIS_Z],
					priv->fir.sum[KXCNL_AXIS_X], priv->fir.sum[KXCNL_AXIS_Y], priv->fir.sum[KXCNL_AXIS_Z],
					data[KXCNL_AXIS_X], data[KXCNL_AXIS_Y], data[KXCNL_AXIS_Z]);
				}
			}
		}
	}	
#endif         
	return err;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_ReadOffset(struct i2c_client *client, s8 ofs[KXCNL_AXES_NUM])
{    
	int err = 0;

	ofs[1]=ofs[2]=ofs[0]=0x00;

	printk("offesx=%x, y=%x, z=%x",ofs[0],ofs[1],ofs[2]);
	
	return err;    
}
/*----------------------------------------------------------------------------*/
static int KXCNL_ResetCalibration(struct i2c_client *client)
{
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	int err = 0;
	GSE_LOG("%s\n", __func__);

	memset(obj->cali_sw, 0x00, sizeof(obj->cali_sw));
	memset(obj->offset, 0x00, sizeof(obj->offset));
	return err;    
}
/*----------------------------------------------------------------------------*/
static int KXCNL_ReadCalibration(struct i2c_client *client, int dat[KXCNL_AXES_NUM])
{
    struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
    int mul=0;

#ifndef SW_CALIBRATION
	if ((err = KXCNL_ReadOffset(client, obj->offset))) {
       	GSE_ERR("read offset fail, %d\n", err);
        	return err;
    	}    
    	mul = obj->reso->sensitivity/kxcnl_offset_resolution.sensitivity;
#endif

    dat[obj->cvt.map[KXCNL_AXIS_X]] = obj->cvt.sign[KXCNL_AXIS_X]*(obj->offset[KXCNL_AXIS_X]*mul + obj->cali_sw[KXCNL_AXIS_X]);
    dat[obj->cvt.map[KXCNL_AXIS_Y]] = obj->cvt.sign[KXCNL_AXIS_Y]*(obj->offset[KXCNL_AXIS_Y]*mul + obj->cali_sw[KXCNL_AXIS_Y]);
    dat[obj->cvt.map[KXCNL_AXIS_Z]] = obj->cvt.sign[KXCNL_AXIS_Z]*(obj->offset[KXCNL_AXIS_Z]*mul + obj->cali_sw[KXCNL_AXIS_Z]);                        
                                       
    return 0;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_ReadCalibrationEx(struct i2c_client *client, int act[KXCNL_AXES_NUM], int raw[KXCNL_AXES_NUM])
{  
	/*raw: the raw calibration data; act: the actual calibration data*/
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
    #ifdef SW_CALIBRATION
    #else
	int err;
    #endif
	int mul;

 
	GSE_FUN();
	#ifdef SW_CALIBRATION
		mul = 0;//only SW Calibration, disable HW Calibration
	#else
		if(err = KXCNL_ReadOffset(client, obj->offset))
		{
			GSE_ERR("read offset fail, %d\n", err);
			return err;
		}   
		mul = obj->reso->sensitivity/kxcnl_offset_resolution.sensitivity;
	#endif
	
	raw[KXCNL_AXIS_X] = obj->offset[KXCNL_AXIS_X]*mul + obj->cali_sw[KXCNL_AXIS_X];
	raw[KXCNL_AXIS_Y] = obj->offset[KXCNL_AXIS_Y]*mul + obj->cali_sw[KXCNL_AXIS_Y];
	raw[KXCNL_AXIS_Z] = obj->offset[KXCNL_AXIS_Z]*mul + obj->cali_sw[KXCNL_AXIS_Z];

	act[obj->cvt.map[KXCNL_AXIS_X]] = obj->cvt.sign[KXCNL_AXIS_X]*raw[KXCNL_AXIS_X];
	act[obj->cvt.map[KXCNL_AXIS_Y]] = obj->cvt.sign[KXCNL_AXIS_Y]*raw[KXCNL_AXIS_Y];
	act[obj->cvt.map[KXCNL_AXIS_Z]] = obj->cvt.sign[KXCNL_AXIS_Z]*raw[KXCNL_AXIS_Z];                        
	                       
	return 0;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_WriteCalibration(struct i2c_client *client, int dat[KXCNL_AXES_NUM])
{
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	int err=0;
	int cali[KXCNL_AXES_NUM], raw[KXCNL_AXES_NUM];

#ifndef SW_CALIBRATION
	int lsb = kxcnl_offset_resolution.sensitivity;
	int divisor = obj->reso->sensitivity/lsb;
#endif

	GSE_FUN();

	if(0 != (err = KXCNL_ReadCalibrationEx(client, cali, raw)))	/*offset will be updated in obj->offset*/
	{ 
		GSE_ERR("read offset fail, %d\n", err);
		return err;
	}

	GSE_LOG("OLDOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		raw[KXCNL_AXIS_X], raw[KXCNL_AXIS_Y], raw[KXCNL_AXIS_Z],
		obj->offset[KXCNL_AXIS_X], obj->offset[KXCNL_AXIS_Y], obj->offset[KXCNL_AXIS_Z],
		obj->cali_sw[KXCNL_AXIS_X], obj->cali_sw[KXCNL_AXIS_Y], obj->cali_sw[KXCNL_AXIS_Z]);

	/*calculate the real offset expected by caller*/
	cali[KXCNL_AXIS_X] += dat[KXCNL_AXIS_X];
	cali[KXCNL_AXIS_Y] += dat[KXCNL_AXIS_Y];
	cali[KXCNL_AXIS_Z] += dat[KXCNL_AXIS_Z];

	GSE_LOG("UPDATE: (%+3d %+3d %+3d)\n", 
		dat[KXCNL_AXIS_X], dat[KXCNL_AXIS_Y], dat[KXCNL_AXIS_Z]);

#ifdef SW_CALIBRATION
	obj->cali_sw[KXCNL_AXIS_X] = obj->cvt.sign[KXCNL_AXIS_X]*(cali[obj->cvt.map[KXCNL_AXIS_X]]);
	obj->cali_sw[KXCNL_AXIS_Y] = obj->cvt.sign[KXCNL_AXIS_Y]*(cali[obj->cvt.map[KXCNL_AXIS_Y]]);
	obj->cali_sw[KXCNL_AXIS_Z] = obj->cvt.sign[KXCNL_AXIS_Z]*(cali[obj->cvt.map[KXCNL_AXIS_Z]]);	
#else
	obj->offset[KXCNL_AXIS_X] = (s8)(obj->cvt.sign[KXCNL_AXIS_X]*(cali[obj->cvt.map[KXCNL_AXIS_X]])/(divisor));
	obj->offset[KXCNL_AXIS_Y] = (s8)(obj->cvt.sign[KXCNL_AXIS_Y]*(cali[obj->cvt.map[KXCNL_AXIS_Y]])/(divisor));
	obj->offset[KXCNL_AXIS_Z] = (s8)(obj->cvt.sign[KXCNL_AXIS_Z]*(cali[obj->cvt.map[KXCNL_AXIS_Z]])/(divisor));

	/*convert software calibration using standard calibration*/
	obj->cali_sw[KXCNL_AXIS_X] = obj->cvt.sign[KXCNL_AXIS_X]*(cali[obj->cvt.map[KXCNL_AXIS_X]])%(divisor);
	obj->cali_sw[KXCNL_AXIS_Y] = obj->cvt.sign[KXCNL_AXIS_Y]*(cali[obj->cvt.map[KXCNL_AXIS_Y]])%(divisor);
	obj->cali_sw[KXCNL_AXIS_Z] = obj->cvt.sign[KXCNL_AXIS_Z]*(cali[obj->cvt.map[KXCNL_AXIS_Z]])%(divisor);

	GSE_LOG("NEWOFF: (%+3d %+3d %+3d): (%+3d %+3d %+3d) / (%+3d %+3d %+3d)\n", 
		obj->offset[KXCNL_AXIS_X]*divisor + obj->cali_sw[KXCNL_AXIS_X], 
		obj->offset[KXCNL_AXIS_Y]*divisor + obj->cali_sw[KXCNL_AXIS_Y], 
		obj->offset[KXCNL_AXIS_Z]*divisor + obj->cali_sw[KXCNL_AXIS_Z], 
		obj->offset[KXCNL_AXIS_X], obj->offset[KXCNL_AXIS_Y], obj->offset[KXCNL_AXIS_Z],
		obj->cali_sw[KXCNL_AXIS_X], obj->cali_sw[KXCNL_AXIS_Y], obj->cali_sw[KXCNL_AXIS_Z]);

	if(err = hwmsen_write_block(obj->client, KXCNL_REG_OFSX, obj->offset, KXCNL_AXES_NUM))
	{
		GSE_ERR("write offset fail: %d\n", err);
		return err;
	}
#endif

	return err;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_CheckDeviceID(struct i2c_client *client)
{
	u8 databuf[2]={0xff};    
	int res = 0;
	u8 addr = KXCNL_REG_DEVID;

	mdelay(50);
	
	if(hwmsen_read_block(client, addr, databuf, 0x01))
	{
		GSE_LOG("KXCNL_CheckDeviceID read id fail\n");
		return KXCNL_ERR_I2C;
	}

	if (databuf[0] == KXCNL_DEVICE_ID)
	{
		GSE_LOG("KXCNL_CheckDeviceID:id=0x%x OK\n", databuf[0]);
	}
	else
	{
		GSE_LOG("KXCNL_CheckDeviceID:id=0x%x fail!\n", databuf[0]);		
		return KXCNL_ERR_IDENTIFICATION;
	}
	
	return KXCNL_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_SetPowerMode(struct i2c_client *client, bool enable)
{
	int res = 0;
    u8 databuf[2];
	u8 addr = KXCNL_REG_POWER_CTL;
	
	if(enable == sensor_power)
	{
		GSE_LOG("Sensor power status is newest!\n");
		return KXCNL_SUCCESS;
	}

	if(hwmsen_read_block(client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return KXCNL_ERR_I2C;
	}

	
	if(enable == TRUE)
	{
		databuf[0] |= KXCNL_MEASURE_MODE;
	}
	else
	{
		databuf[0] &= ~KXCNL_MEASURE_MODE;
	}
	databuf[1] = databuf[0];
	databuf[0] = KXCNL_REG_POWER_CTL;
	

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXCNL_ERR_I2C;
	}

	GSE_LOG("KXCNL_SetPowerMode %d OK\n ",enable);


	sensor_power = enable;

	mdelay(5);
	
	return KXCNL_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int KXCNL_SetDataFormat(struct i2c_client *client, u8 dataformat)
{
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10];    
	int res = 0;
	bool cur_sensor_power = sensor_power;

	memset(databuf, 0, sizeof(u8)*10);  

	KXCNL_SetPowerMode(client, false);

	if(hwmsen_read_block(client, KXCNL_REG_DATA_FORMAT, databuf, 0x01))
	{
		printk("kxcnl read Dataformat failt \n");
		return KXCNL_ERR_I2C;
	}

	databuf[0] &= ~KXCNL_RANGE_MASK;
	databuf[0] |= dataformat;
	databuf[1] = databuf[0];
	databuf[0] = KXCNL_REG_DATA_FORMAT;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXCNL_ERR_I2C;
	}

	KXCNL_SetPowerMode(client, cur_sensor_power/*true*/);
	
	printk("KXCNL_SetDataFormat OK! \n");
	

	return KXCNL_SetDataResolution(obj);    
}
/*----------------------------------------------------------------------------*/
static int KXCNL_SetBWRate(struct i2c_client *client, u8 bwrate)
{
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	u8 databuf[10]={0};    
	int res = 0;
	bool cur_sensor_power = sensor_power;

#if  1//KIONIX_KXCNL_HW_PEDOMETER
	if(obj->pedo_enabled)	
	{
		printk("Pedometer enabled, ODR default 100Hz!\n");
		return KXCNL_SUCCESS;
	}
#endif
	
	KXCNL_SetPowerMode(client, false);

	if(hwmsen_read_block(client, KXCNL_REG_BW_RATE, databuf, 0x01))
	{
		printk("kxcnl read rate failt \n");
		return KXCNL_ERR_I2C;
	}

	databuf[0] &= ~KXCNL_BW_MASK;
	databuf[0] |= bwrate;
	databuf[1] = databuf[0];
	databuf[0] = KXCNL_REG_BW_RATE;


	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXCNL_ERR_I2C;
	}

	
	KXCNL_SetPowerMode(client, cur_sensor_power/*true*/);
	printk("KXCNL_SetBWRate OK! \n");
	
	return KXCNL_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int KXCNL_SetIntEnable(struct i2c_client *client, u8 intenable)
{
	u8 databuf[10];    
	int res = 0;
	
	GSE_LOG("%s\n", __func__);
	memset(databuf, 0, sizeof(u8)*10);    
	databuf[0] = KXCNL_REG_INT_ENABLE;    
	databuf[1] = 0x00;

	res = i2c_master_send(client, databuf, 0x2);

	if(res <= 0)
	{
		return KXCNL_ERR_I2C;
	}
	
	return KXCNL_SUCCESS;    
}
/*----------------------------------------------------------------------------*/
static int kxcnl_init_client(struct i2c_client *client, int reset_cali)
{
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	int res = 0;

       GSE_LOG("%s\n", __func__);
	res = KXCNL_CheckDeviceID(client); 
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}	

	res = KXCNL_SetPowerMode(client, g_bGsensorOnoff/*false*/);
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}
	

	res = KXCNL_SetBWRate(client, KXCNL_BW_100HZ);
	if(res != KXCNL_SUCCESS ) //0x2C->BW=100Hz
	{
		return res;
	}

	res = KXCNL_SetDataFormat(client, KXCNL_RANGE_2G);
	if(res != KXCNL_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}

	gsensor_gain.x = gsensor_gain.y = gsensor_gain.z = obj->reso->sensitivity;

	res = KXCNL_SetIntEnable(client, 0x00);        
	if(res != KXCNL_SUCCESS)//0x2E->0x80
	{
		return res;
	}

	if(0 != reset_cali)
	{ 
		/*reset calibration only in power on*/
		res = KXCNL_ResetCalibration(client);
		if(res != KXCNL_SUCCESS)
		{
			return res;
		}
	}
	printk("kxcnl_init_client OK!\n");
#ifdef CONFIG_KXCNL_LOWPASS
	memset(&obj->fir, 0x00, sizeof(obj->fir));  
#endif

	return KXCNL_SUCCESS;
}
/*----------------------------------------------------------------------------*/
static int KXCNL_ReadChipInfo(struct i2c_client *client, char *buf, int bufsize)
{
	if((NULL == buf)||(bufsize<=30))
	{
		return -1;
	}
	
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "KXCNL Chip");
	return 0;
}

/*Kionix Auto-Cali Start*/
#define KIONIX_AUTO_CAL     //Setup AUTO-Cali parameter
#ifdef KIONIX_AUTO_CAL
//#define DEBUG_MSG_CAL
#define Sensitivity_def      1024	//	
#define Detection_range   200 	// Follow KXTJ2 SPEC Offset Range define
#define Stable_range        50     	// Stable iteration
#define BUF_RANGE_Limit 10 	
static int BUF_RANGE = BUF_RANGE_Limit;			
static int temp_zbuf[50]={0};
static int temp_zsum = 0; // 1024 * BUF_RANGE ;
static int Z_AVG[2] = {Sensitivity_def,Sensitivity_def} ;
static int Wave_Max,Wave_Min;
#endif
/*Kionix Auto-Cali End*/

/*----------------------------------------------------------------------------*/
static int KXCNL_ReadSensorData(struct i2c_client *client, char *buf, int bufsize)
{
	struct kxcnl_i2c_data *obj = (struct kxcnl_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];
	int acc[KXCNL_AXES_NUM];
	int res = 0;
/*Kionix Auto-Cali Start*/
#ifdef KIONIX_AUTO_CAL
    s16 raw[3];
    int k;
#endif    
/*Kionix Auto-Cali End*/

	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	if (atomic_read(&obj->suspend))
	{
		return 0;
	}
	/*if(sensor_power == FALSE)
	{
		res = KXCNL_SetPowerMode(client, true);
		if(res)
		{
			GSE_ERR("Power on kxcnl error %d!\n", res);
		}
	}*/

	if(0 != (res = KXCNL_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -3;
	}
	else
	{
/*Kionix Auto-Cali Start*/
#ifdef KIONIX_AUTO_CAL
        raw[0]=obj->data[KXCNL_AXIS_X];
        raw[1]=obj->data[KXCNL_AXIS_Y];
        raw[2]=obj->data[KXCNL_AXIS_Z];

        if(     (abs(raw[0]) < Detection_range)  
            &&  (abs(raw[1]) < Detection_range) 
            &&  (abs((abs(raw[2])- Sensitivity_def))  < ((Detection_range)+ 308)))
        {

            #ifdef DEBUG_MSG_CAL
            printk("+++KXTJ2 Calibration Raw Data,%d,%d,%d\n",raw[0],raw[1],raw[2]);
            #endif
            temp_zsum = 0;
            Wave_Max =-4095;
            Wave_Min = 4095;
            
            // BUF_RANGE = 1000 / acc_data.delay; **************************88 
            //BUF_RANGE = 1000 / acceld->poll_interval ; 
            
            if ( BUF_RANGE > BUF_RANGE_Limit ) BUF_RANGE = BUF_RANGE_Limit; 
                
            //k printk("KXTJ2 Buffer Range =%d\n",BUF_RANGE);
            
            for (k=0; k < BUF_RANGE-1; k++) {
                temp_zbuf[k] = temp_zbuf[k+1];
                if (temp_zbuf[k] == 0) temp_zbuf[k] = Sensitivity_def ;
                temp_zsum += temp_zbuf[k];
                if (temp_zbuf[k] > Wave_Max) Wave_Max = temp_zbuf[k];
                if (temp_zbuf[k] < Wave_Min) Wave_Min = temp_zbuf[k];
            }

            temp_zbuf[k] = raw[2]; // k=BUF_RANGE-1, update Z raw to bubber
            temp_zsum += temp_zbuf[k];
            if (temp_zbuf[k] > Wave_Max) Wave_Max = temp_zbuf[k];
            if (temp_zbuf[k] < Wave_Min) Wave_Min = temp_zbuf[k];      
            if (Wave_Max-Wave_Min < Stable_range )
            {
                
                if ( temp_zsum > 0)
                {
                    Z_AVG[0] = temp_zsum / BUF_RANGE;
                    //k
    		        #ifdef DEBUG_MSG_CAL
                    printk("+++ Z_AVG=%d\n ", Z_AVG[0]);
                    #endif
                }
                else 
                {
                    Z_AVG[1] = temp_zsum / BUF_RANGE;
                    //k 
		            #ifdef DEBUG_MSG_CAL
                    printk("--- Z_AVG=%d\n ", Z_AVG[1]);
                    #endif
                }
                // printk("KXTJ2 start Z compensation Z_AVG Max Min,%d,%d,%d\n",(temp_zsum / BUF_RANGE),Wave_Max,Wave_Min);
            }
        }
        else if(abs((abs(raw[2])- Sensitivity_def))  > ((Detection_range)+ 154))
        {
            #ifdef DEBUG_MSG_CAL
            printk("KXTJ2 out of SPEC Raw Data,%d,%d,%d\n",raw[0],raw[1],raw[2]);
            #endif
        }
        //else
        //{
        //    printk("KXTJ2 not in horizontal X=%d, Y=%d\n", raw[0], raw[1]);
        //}

        if ( raw[2] >=0) 
            raw[2] = raw[2] * 1024 / abs(Z_AVG[0]); // Gain Compensation
        else 
            raw[2] = raw[2] * 1024 / abs(Z_AVG[1]); // Gain Compensation
                
        //k
        #ifdef DEBUG_MSG_CAL
        //printk("---KXTJ2 Calibration Raw Data,%d,%d,%d==> Z+=%d  Z-=%d \n",raw[0],raw[1],raw[2],Z_AVG[0],Z_AVG[1]);
        printk("---After Cali,X=%d,Y=%d,Z=%d \n",raw[0],raw[1],raw[2]);
        #endif
        obj->data[KXCNL_AXIS_X]=raw[0];
        obj->data[KXCNL_AXIS_Y]=raw[1];
        obj->data[KXCNL_AXIS_Z]=raw[2];
#endif
/*Kionix Auto-Cali End*/

		//printk("raw data x=%d, y=%d, z=%d \n",obj->data[KXCNL_AXIS_X],obj->data[KXCNL_AXIS_Y],obj->data[KXCNL_AXIS_Z]);
		obj->data[KXCNL_AXIS_X] += obj->cali_sw[KXCNL_AXIS_X];
		obj->data[KXCNL_AXIS_Y] += obj->cali_sw[KXCNL_AXIS_Y];
		obj->data[KXCNL_AXIS_Z] += obj->cali_sw[KXCNL_AXIS_Z];
		
		//printk("cali_sw x=%d, y=%d, z=%d \n",obj->cali_sw[KXCNL_AXIS_X],obj->cali_sw[KXCNL_AXIS_Y],obj->cali_sw[KXCNL_AXIS_Z]);
		
		/*remap coordinate*/
		acc[obj->cvt.map[KXCNL_AXIS_X]] = obj->cvt.sign[KXCNL_AXIS_X]*obj->data[KXCNL_AXIS_X];
		acc[obj->cvt.map[KXCNL_AXIS_Y]] = obj->cvt.sign[KXCNL_AXIS_Y]*obj->data[KXCNL_AXIS_Y];
		acc[obj->cvt.map[KXCNL_AXIS_Z]] = obj->cvt.sign[KXCNL_AXIS_Z]*obj->data[KXCNL_AXIS_Z];
		//printk("cvt x=%d, y=%d, z=%d \n",obj->cvt.sign[KXCNL_AXIS_X],obj->cvt.sign[KXCNL_AXIS_Y],obj->cvt.sign[KXCNL_AXIS_Z]);


		//GSE_LOG("Mapped gsensor data: %d, %d, %d!\n", acc[KXCNL_AXIS_X], acc[KXCNL_AXIS_Y], acc[KXCNL_AXIS_Z]);

		//Out put the mg
		//printk("mg acc=%d, GRAVITY=%d, sensityvity=%d \n",acc[KXCNL_AXIS_X],GRAVITY_EARTH_1000,obj->reso->sensitivity);
		acc[KXCNL_AXIS_X] = acc[KXCNL_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[KXCNL_AXIS_Y] = acc[KXCNL_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
		acc[KXCNL_AXIS_Z] = acc[KXCNL_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;		

		sprintf(buf, "%04x %04x %04x", acc[KXCNL_AXIS_X], acc[KXCNL_AXIS_Y], acc[KXCNL_AXIS_Z]);
		if(atomic_read(&obj->trace) & ADX_TRC_IOCTL)
		{
			GSE_LOG("gsensor data: %s!\n", buf);
		}
	}
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int KXCNL_ReadRawData(struct i2c_client *client, char *buf)
{
	struct kxcnl_i2c_data *obj = (struct kxcnl_i2c_data*)i2c_get_clientdata(client);
	int res = 0;

	GSE_FUN();
	if (!buf || !client)
	{
		return -EINVAL;
	}

	if(0 != (res = KXCNL_ReadData(client, obj->data)))
	{        
		GSE_ERR("I2C error: ret value=%d", res);
		return -EIO;
	}
	else
	{
		sprintf(buf, "KXCNL_ReadRawData %04x %04x %04x", obj->data[KXCNL_AXIS_X], 
			obj->data[KXCNL_AXIS_Y], obj->data[KXCNL_AXIS_Z]);
	
	}
	
	return 0;
}

/*----------------------------------------------------------------------------*/
static int KXCNL_InitSelfTest(struct i2c_client *client)
{
/*	int res = 0;
	u8  data,result;

	
	res = hwmsen_read_byte(client, KXCNL_REG_CTL_REG3, &data);
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}
//enable selftest bit
	res = hwmsen_write_byte(client, KXCNL_REG_CTL_REG3,  KXCNL_SELF_TEST|data);
	if(res != KXCNL_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
//step 1
	res = hwmsen_read_byte(client, KXCNL_DCST_RESP, &result);
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}
	printk("step1: result = %x",result);
	if(result != 0xaa)
		return -EINVAL;

//step 2
	res = hwmsen_write_byte(client, KXCNL_REG_CTL_REG3,  KXCNL_SELF_TEST|data);
	if(res != KXCNL_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
//step 3
	res = hwmsen_read_byte(client, KXCNL_DCST_RESP, &result);
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}
	printk("step3: result = %x",result);
	if(result != 0xAA)
		return -EINVAL;
		
//step 4
	res = hwmsen_read_byte(client, KXCNL_DCST_RESP, &result);
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}
	printk("step4: result = %x",result);
	if(result != 0x55)
		return -EINVAL;
	else
*/
		return KXCNL_SUCCESS;
}
/*----------------------------------------------------------------------------*/
#if 0
static int KXCNL_JudgeTestResult(struct i2c_client *client, s32 prv[KXCNL_AXES_NUM], s32 nxt[KXCNL_AXES_NUM])
{

    int res=0;
	u8 test_result=0;
    if(0 != (res = hwmsen_read_byte(client, 0x0c, &test_result)))
        return res;

	printk("test_result = %x \n",test_result);
    if ( test_result != 0xaa ) 
	{
        GSE_ERR("KXCNL_JudgeTestResult failt\n");
        res = -EINVAL;
    }
    return res;
}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_chipinfo_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	char strbuf[KXCNL_BUFSIZE];
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	
	KXCNL_ReadChipInfo(client, strbuf, KXCNL_BUFSIZE);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);        
}
#if 0
static ssize_t gsensor_init(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = kxcnl_i2c_client;
		char strbuf[KXCNL_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		kxcnl_init_client(client, 1);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_sensordata_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	char strbuf[KXCNL_BUFSIZE];
	
	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}
	KXCNL_ReadSensorData(client, strbuf, KXCNL_BUFSIZE);
	//KXCNL_ReadRawData(client, strbuf);
	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);            
}
#if 0
static ssize_t show_sensorrawdata_value(struct device_driver *ddri, char *buf, size_t count)
	{
		struct i2c_client *client = kxcnl_i2c_client;
		char strbuf[KXCNL_BUFSIZE];
		
		if(NULL == client)
		{
			GSE_ERR("i2c client is null!!\n");
			return 0;
		}
		//KXCNL_ReadSensorData(client, strbuf, KXCNL_BUFSIZE);
		KXCNL_ReadRawData(client, strbuf);
		return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);			
	}
#endif
/*----------------------------------------------------------------------------*/
static ssize_t show_cali_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *obj;
	int err, len = 0, mul;
	int tmp[KXCNL_AXES_NUM];

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);



	if(0 != (err = KXCNL_ReadOffset(client, obj->offset)))
	{
		return -EINVAL;
	}
	else if(0 != (err = KXCNL_ReadCalibration(client, tmp)))
	{
		return -EINVAL;
	}
	else
	{    
		mul = obj->reso->sensitivity/kxcnl_offset_resolution.sensitivity;
		len += snprintf(buf+len, PAGE_SIZE-len, "[HW ][%d] (%+3d, %+3d, %+3d) : (0x%02X, 0x%02X, 0x%02X)\n", mul,                        
			obj->offset[KXCNL_AXIS_X], obj->offset[KXCNL_AXIS_Y], obj->offset[KXCNL_AXIS_Z],
			obj->offset[KXCNL_AXIS_X], obj->offset[KXCNL_AXIS_Y], obj->offset[KXCNL_AXIS_Z]);
		len += snprintf(buf+len, PAGE_SIZE-len, "[SW ][%d] (%+3d, %+3d, %+3d)\n", 1, 
			obj->cali_sw[KXCNL_AXIS_X], obj->cali_sw[KXCNL_AXIS_Y], obj->cali_sw[KXCNL_AXIS_Z]);

		len += snprintf(buf+len, PAGE_SIZE-len, "[ALL]    (%+3d, %+3d, %+3d) : (%+3d, %+3d, %+3d)\n", 
			obj->offset[KXCNL_AXIS_X]*mul + obj->cali_sw[KXCNL_AXIS_X],
			obj->offset[KXCNL_AXIS_Y]*mul + obj->cali_sw[KXCNL_AXIS_Y],
			obj->offset[KXCNL_AXIS_Z]*mul + obj->cali_sw[KXCNL_AXIS_Z],
			tmp[KXCNL_AXIS_X], tmp[KXCNL_AXIS_Y], tmp[KXCNL_AXIS_Z]);
		
		return len;
    }
}
/*----------------------------------------------------------------------------*/
static ssize_t store_cali_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct i2c_client *client = kxcnl_i2c_client;  
	int err, x, y, z;
	int dat[KXCNL_AXES_NUM];

	if(!strncmp(buf, "rst", 3))
	{
		if(0 != (err = KXCNL_ResetCalibration(client)))
		{
			GSE_ERR("reset offset err = %d\n", err);
		}	
	}
	else if(3 == sscanf(buf, "0x%02X 0x%02X 0x%02X", &x, &y, &z))
	{
		dat[KXCNL_AXIS_X] = x;
		dat[KXCNL_AXIS_Y] = y;
		dat[KXCNL_AXIS_Z] = z;
		if(0 != (err = KXCNL_WriteCalibration(client, dat)))
		{
			GSE_ERR("write calibration err = %d\n", err);
		}		
	}
	else
	{
		GSE_ERR("invalid format\n");
	}
	
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_self_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

    return snprintf(buf, 8, "%s\n", selftestRes);
}
/*----------------------------------------------------------------------------*/
static ssize_t store_self_value(struct device_driver *ddri, const char *buf, size_t count)
{   /*write anything to this register will trigger the process*/
	struct item{
	s16 raw[KXCNL_AXES_NUM];
	};
	
	struct i2c_client *client = kxcnl_i2c_client;  
	int res, num;
	struct item *prv = NULL, *nxt = NULL;
	u8 data;

	if(1 != sscanf(buf, "%d", &num))
	{
		GSE_ERR("parse number fail\n");
		return count;
	}
	else if(num == 0)
	{
		GSE_ERR("invalid data count\n");
		return count;
	}

	prv = kzalloc(sizeof(*prv) * num, GFP_KERNEL);
	nxt = kzalloc(sizeof(*nxt) * num, GFP_KERNEL);
	if (!prv || !nxt)
	{
		goto exit;
	}


	GSE_LOG("NORMAL:\n");
	KXCNL_SetPowerMode(client,true); 

	/*initial setting for self test*/
	if(!KXCNL_InitSelfTest(client))
	{
		GSE_LOG("SELFTEST : PASS\n");
		strcpy(selftestRes,"y");
	}	
	else
	{
		GSE_LOG("SELFTEST : FAIL\n");		
		strcpy(selftestRes,"n");
	}

	res = hwmsen_read_byte(client, KXCNL_REG_CTL_REG3, &data);
	if(res != KXCNL_SUCCESS)
	{
		return res;
	}

	res = hwmsen_write_byte(client, KXCNL_REG_CTL_REG3,  ~KXCNL_SELF_TEST&data);
	if(res != KXCNL_SUCCESS) //0x2C->BW=100Hz
	{
		return res;
	}
	
	exit:
	/*restore the setting*/    
	kxcnl_init_client(client, 0);
	kfree(prv);
	kfree(nxt);
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_selftest_value(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *obj;

	if(NULL == client)
	{
		GSE_ERR("i2c client is null!!\n");
		return 0;
	}

	obj = i2c_get_clientdata(client);
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->selftest));
}
/*----------------------------------------------------------------------------*/
static ssize_t store_selftest_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxcnl_i2c_data *obj = obj_i2c_data;
	int tmp;

	if(NULL == obj)
	{
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}
	
	
	if(1 == sscanf(buf, "%d", &tmp))
	{        
		if(atomic_read(&obj->selftest) && !tmp)
		{
			/*enable -> disable*/
			kxcnl_init_client(obj->client, 0);
		}
		else if(!atomic_read(&obj->selftest) && tmp)
		{
			/*disable -> enable*/
			KXCNL_InitSelfTest(obj->client);            
		}
		
		GSE_LOG("selftest: %d => %d\n", atomic_read(&obj->selftest), tmp);
		atomic_set(&obj->selftest, tmp); 
	}
	else
	{ 
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);   
	}
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_firlen_value(struct device_driver *ddri, char *buf)
{
#ifdef CONFIG_KXCNL_LOWPASS
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	if(atomic_read(&obj->firlen))
	{
		int idx, len = atomic_read(&obj->firlen);
		GSE_LOG("len = %2d, idx = %2d\n", obj->fir.num, obj->fir.idx);

		for(idx = 0; idx < len; idx++)
		{
			GSE_LOG("[%5d %5d %5d]\n", obj->fir.raw[idx][KXCNL_AXIS_X], obj->fir.raw[idx][KXCNL_AXIS_Y], obj->fir.raw[idx][KXCNL_AXIS_Z]);
		}
		
		GSE_LOG("sum = [%5d %5d %5d]\n", obj->fir.sum[KXCNL_AXIS_X], obj->fir.sum[KXCNL_AXIS_Y], obj->fir.sum[KXCNL_AXIS_Z]);
		GSE_LOG("avg = [%5d %5d %5d]\n", obj->fir.sum[KXCNL_AXIS_X]/len, obj->fir.sum[KXCNL_AXIS_Y]/len, obj->fir.sum[KXCNL_AXIS_Z]/len);
	}
	return snprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&obj->firlen));
#else
	return snprintf(buf, PAGE_SIZE, "not support\n");
#endif
}
/*----------------------------------------------------------------------------*/
static ssize_t store_firlen_value(struct device_driver *ddri, const char *buf, size_t count)
{
#ifdef CONFIG_KXCNL_LOWPASS
	struct i2c_client *client = kxcnl_i2c_client;  
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);
	int firlen;

	if(1 != sscanf(buf, "%d", &firlen))
	{
		GSE_ERR("invallid format\n");
	}
	else if(firlen > C_MAX_FIR_LENGTH)
	{
		GSE_ERR("exceeds maximum filter length\n");
	}
	else
	{ 
		atomic_set(&obj->firlen, firlen);
		if(NULL == firlen)
		{
			atomic_set(&obj->fir_en, 0);
		}
		else
		{
			memset(&obj->fir, 0x00, sizeof(obj->fir));
			atomic_set(&obj->fir_en, 1);
		}
	}
#endif    
	return count;
}
/*----------------------------------------------------------------------------*/
static ssize_t show_trace_value(struct device_driver *ddri, char *buf)
{
	ssize_t res;
	struct kxcnl_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	res = snprintf(buf, PAGE_SIZE, "0x%04X\n", atomic_read(&obj->trace));     
	return res;    
}
/*----------------------------------------------------------------------------*/
static ssize_t store_trace_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxcnl_i2c_data *obj = obj_i2c_data;
	int trace;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}
	
	if(1 == sscanf(buf, "0x%x", &trace))
	{
		atomic_set(&obj->trace, trace);
	}	
	else
	{
		GSE_ERR("invalid content: '%s', length = %d\n", buf, (int)count);
	}
	
	return count;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_status_value(struct device_driver *ddri, char *buf)
{
	ssize_t len = 0;    
	struct kxcnl_i2c_data *obj = obj_i2c_data;
	if (obj == NULL)
	{
		GSE_ERR("i2c_data obj is null!!\n");
		return 0;
	}	
	
	if(obj->hw)
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: %d %d (%d %d)\n", 
	            obj->hw->i2c_num, obj->hw->direction, obj->hw->power_id, obj->hw->power_vol);   
	}
	else
	{
		len += snprintf(buf+len, PAGE_SIZE-len, "CUST: NULL\n");
	}
	return len;    
}
/*----------------------------------------------------------------------------*/
static ssize_t show_power_status_value(struct device_driver *ddri, char *buf)
{
	u8 databuf[2];    
	u8 addr = KXCNL_REG_POWER_CTL;
	if(hwmsen_read_block(kxcnl_i2c_client, addr, databuf, 0x01))
	{
		GSE_ERR("read power ctl register err!\n");
		return KXCNL_ERR_I2C;
	}
    
	if(sensor_power)
		printk("G sensor is in work mode, sensor_power = %d\n", sensor_power);
	else
		printk("G sensor is in standby mode, sensor_power = %d\n", sensor_power);

	return snprintf(buf, PAGE_SIZE, "%x\n", databuf[0]);
}

#if  1//KIONIX_KXCNL_HW_PEDOMETER
static int kionix_i2c_read(struct i2c_client *client, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[2]={{0},{0}};

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len   = 1;
	msgs[0].buf   = &addr;
	
	msgs[1].addr = client->addr;
	msgs[1].flags = client->flags | I2C_M_RD;
	msgs[1].len   = len;
	msgs[1].buf   = data;

	return i2c_transfer(client->adapter, msgs, 2);
}

static int kionix_i2c_write(struct i2c_client *client, u8 addr, u8 *data, int len)
{
	u8 p[256],i;
	struct i2c_msg msgs[1]={{0}};
	
	*p = addr;
	for(i=0; i<len; i++)
	{
		*(p+1+i) = *data++;
	}

	msgs[0].addr = client->addr;
	msgs[0].flags = client->flags;
	msgs[0].len   = len+1;
	msgs[0].buf   = p;

	return i2c_transfer(client->adapter, msgs, 1);
}

static int kionix_strtok(const char *buf, size_t count, char **token, const int token_nr)
{
	char *buf2 = (char *)kzalloc((count + 1) * sizeof(char), GFP_KERNEL);
	char **token2 = token;
	unsigned int num_ptr = 0, num_nr = 0, num_neg = 0;
	int i = 0, start = 0, end = (int)count;

	strcpy(buf2, buf);

	/* We need to breakup the string into separate chunks in order for kstrtoint
	 * or strict_strtol to parse them without returning an error. Stop when the end of
	 * the string is reached or when enough value is read from the string */
	while((start < end) && (i < token_nr)) {
		/* We found a negative sign */
		if(*(buf2 + start) == '-') {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				/* If there is a pending negative sign, we adjust the variables to account for it */
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
				/* Reset */
				num_ptr = num_nr = 0;
			}
			/* This indicates that there is a pending negative sign in the string */
			num_neg = 1;
		}
		/* We found a numeric */
		else if((*(buf2 + start) >= '0') && (*(buf2 + start) <= '9')) {
			/* If the previous char(s) are not numeric, set num_ptr to current char */
			if(num_nr < 1)
				num_ptr = start;
			num_nr++;
		}
		/* We found an unwanted character */
		else {
			/* Previous char(s) are numeric, so we store their value first before proceed */
			if(num_nr > 0) {
				if(num_neg) {
					num_ptr--;
					num_nr++;
				}
				*token2 = (char *)kzalloc((num_nr + 2) * sizeof(char), GFP_KERNEL);
				strncpy(*token2, (const char *)(buf2 + num_ptr), (size_t) num_nr);
				*(*token2+num_nr) = '\n';
				i++;
				token2++;
			}
			/* Reset all the variables to start afresh */
			num_ptr = num_nr = num_neg = 0;
		}
		start++;
	}

	kfree(buf2);

	return (i == token_nr) ? token_nr : -1;
}

//KXCNL related function
static int KXCNL_updateStateMachine1(struct kxcnl_i2c_data *acceld, int newMode)
{
	int err = 0;
	u8 val;
	
	if(newMode == MODE_WALKING) {
		// update walking mode params to step counter state machine
		val = SM1_WALKING_TH1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_THRS1_1, val);
		if (err < 0)
				return err;	

		val = SM1_WALKING_TIM1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM1_1_L, val);
		if (err < 0)
				return err;	

		val = SM1_WALKING_TIM2;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM2_1_L, val);
		if (err < 0)
				return err;
	} else if(newMode == MODE_RUNNING) {
		val = SM1_RUNNING_TH1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_THRS1_1, val);
		if (err < 0)
				return err;	

		val = SM1_RUNNING_TIM1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM1_1_L, val);
		if (err < 0)
				return err;	

		val = SM1_RUNNING_TIM2;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM2_1_L, val);
		if (err < 0)
				return err;
	}
	
	return err;
}

static int KXCNL_updateStateMachine2(struct kxcnl_i2c_data *acceld, int newMode)
{
	int err = 0;
	u8 val;

	if(newMode == MODE_WALKING) {
		// Update running detection to state machine 2
		err = kionix_i2c_write(acceld->client, KXCNL_ST1_2, sm2_observe_running, 4);
		if (err < 0)
				return err;	
		err = kionix_i2c_write(acceld->client, KXCNL_ST1_2+4, &sm2_observe_running[4], sizeof(sm2_observe_running)-4);
		if (err < 0)
				return err;	
			
		// Update running detection threshold and timers 
		val = SM2_RUN_DETECT_TH1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_THRS1_2, val);
		if (err < 0)
				return err;	

		val = SM2_RUN_DETECT_TIM1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM1_2_L, val);
		if (err < 0)
				return err;	

		val = SM2_RUN_DETECT_TIM2;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM2_2_L, val);
		if (err < 0)
				return err;
	} else if(newMode == MODE_RUNNING) {
		// Update walking detection to state machine 2
		err = kionix_i2c_write(acceld->client, KXCNL_ST1_2, sm2_observe_walking, sizeof(sm2_observe_walking));
		if (err < 0)
				return err;	
			
		// Update walking detection threshold and timers 
		val = SM2_WALK_DETECT_TH1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_THRS1_2, val);
		if (err < 0)
				return err;	

		val = SM2_WALK_DETECT_TIM1;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM1_2_L, val);
		if (err < 0)
				return err;	

		val = SM2_WALK_DETECT_TIM2;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_TIM2_2_L, val);
		if (err < 0)
				return err;
	}
	
	return err;
}

static int	KXCNL_pedometer_start(struct kxcnl_i2c_data *acceld)
{
	u8 val, temp[4];
	int err = 0;

	GSE_FUN();
	// clear ctrl1
	val = 0;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL1, val);
	if (err < 0)
			return err;

	// Set filter
	temp[0] = KXCNL_VFC1_DEFAULT_VALUE;
	temp[1] = KXCNL_VFC2_DEFAULT_VALUE;
	temp[2] = KXCNL_VFC3_DEFAULT_VALUE;
	temp[3] = KXCNL_VFC4_DEFAULT_VALUE;
	err = kionix_i2c_write(acceld->client, KXCNL_VFC_1, temp, 4);
	if (err < 0)
			return err;

	// Set cntl4 - active high, enable int1 and vfilter
	val = KXCNL_CNTL4_IEA | KXCNL_CNTL4_INT1_EN | KXCNL_CNTL4_VFILT;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL4, val);
	if (err < 0)
			return err;

	// Set counter value
	temp[0] = KXCNL_LC_SENSOR_INIT_VALUE1;
	temp[1] = KXCNL_LC_SENSOR_INIT_VALUE2;
	err = kionix_i2c_write(acceld->client, KXCNL_LC_L, temp, 2);
	if (err < 0)
			return err;	

	// Step counter is in state machine 1
	//	initStateMachine1();
	// Write state machine code
	err = kionix_i2c_write(acceld->client, KXCNL_ST1_1, step_counter_code_values, 4);
	if (err < 0)
			return err;	
	err = kionix_i2c_write(acceld->client, KXCNL_ST1_1+4, &step_counter_code_values[4], 4);
	if (err < 0)
			return err;	
	err = kionix_i2c_write(acceld->client, KXCNL_ST1_1+8, &step_counter_code_values[8], 4);
	if (err < 0)
			return err;	
	err = kionix_i2c_write(acceld->client, KXCNL_ST1_1+12, &step_counter_code_values[12], sizeof(step_counter_code_values)-12);
	if (err < 0)
			return err;	
	
	// Set axis and program flow
	val = KXCNL_MA2_P_V;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_MA1, val);
	if (err < 0)
			return err;	

	val = KXCNL_SETT2_SITR;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_SETT1, val);
	if (err < 0)
			return err;	

	// Run/walk mode detection in state machine 2
	//initStateMachine2();
	val = KXCNL_MA2_P_V;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_MA2, val);
	if (err < 0)
			return err;	
		
	val = KXCNL_SETT2_SITR;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_SETT2, val);
	if (err < 0)
			return err;	

	// Walk mode is default
	acceld->pedo_mode = MODE_WALKING;
	KXCNL_updateStateMachine1(acceld, MODE_WALKING);
	KXCNL_updateStateMachine2(acceld, MODE_WALKING);

    // Enable state machine 1 and 2 
	val = KXCNL_CNTL2_SM1_EN;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL2, val);
	if (err < 0)
			return err;	

	val = KXCNL_CNTL3_SM2_EN; 
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL3, val);
	if (err < 0)
			return err;	

	// Set cntl1
	val = KXCNL_CNTL1_PC | KXCNL_CNTL1_SC_2g | KXCNL_CNTL1_ODR_100 | KXCNL_CNTL1_IEN;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL1, val);
	if (err < 0)
			return err;	
	GSE_LOG("KXCNL_pedometer_start OK\n");
	return err;	
}

static int	KXCNL_pedometer_stop(struct kxcnl_i2c_data *acceld)
{
	u8 val;
	int err = 0;
	struct kxcnl_i2c_data *obj = acceld;

	GSE_FUN();
	// Disable state machines
	val = 0;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL2, val);
	if (err < 0)
			return err;	
		
	val = 0;
	err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL3, val);
	if (err < 0)
			return err;	

	if(atomic_read(&obj->suspend))
	{
		val = 0;
		err = i2c_smbus_write_byte_data(acceld->client, KXCNL_CNTL1, val);
		if (err < 0)
				return err;		
	}
	
	return err;		
}

static int	KXCNL_get_current_step(struct kxcnl_i2c_data *acceld)
{
	u16 steps;
	int err = 0;	
	u8	temp[4]={0};

       GSE_FUN();
	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&kxcnl_mutex);

	err = hwmsen_read_block(acceld->client, KXCNL_LC_L, temp, 2);	
	if (err < 0)
			return err;	
		
	steps = temp[0] + temp[1]*256;
	steps = 0x7FFF - steps;

	mutex_unlock(&kxcnl_mutex);

	return steps;		
}

/* Returns the pedometer enable state of device */
static ssize_t show_pedometer_enable(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);

	GSE_LOG("%s: enable status=%d\n", __FUNCTION__, acceld->pedo_enabled);
	return sprintf(buf, "%d\n", (acceld->pedo_enabled) > 0 ? 1 : 0);
}

/* Allow users to enable/disable the hardware pedometer */
static ssize_t store_pedometer_enable(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
	int err = 0;
       char data[2]={0};

	GSE_FUN();
	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&kxcnl_mutex);

	if (count!=2) 
	{
		GSE_ERR("pedo_enable, paramLen error=0x%x\n", (int)count);
		mutex_unlock(&kxcnl_mutex);
		return -1;
	}

       strncpy(&data[0], buf, 2);

	if (data[0]==0x31)   // input 1
	{
		GSE_LOG("enable=1\n");
		acceld->pedo_enabled = 1;
		err = KXCNL_pedometer_start(acceld);
	}
	else if(data[0]==0x30)   // input 0
	{
		GSE_LOG("enable=0\n");
		acceld->pedo_enabled = 0;
		err = KXCNL_pedometer_stop(acceld);
	}
	else
	{
		GSE_ERR("pedo_enable, param err=0x%x 0x%x", data[0], data[1]);
		mutex_unlock(&kxcnl_mutex);
		return -1;
	}

	mutex_unlock(&kxcnl_mutex);

	return (err < 0) ? err : count;
}

/* Returns the pedometer enable state of device */
static ssize_t show_pedometer_mode(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
	
	GSE_FUN();	
	return sprintf(buf, "%d\n", acceld->pedo_mode);
}

/* Returns current step counters*/
static ssize_t show_pedometer_step(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
 	int count=0;

	GSE_FUN();
	
	//Get step from KXCNL and re-initiate state machine
	count = KXCNL_get_current_step(acceld);
	if (count<0) count=0;

	count += acceld->current_step;
	
	return sprintf(buf, "%d\n", count);
}

/* Allow users to change the calibration value of the device */
static ssize_t store_pedometer_step(struct device_driver *ddri, char *buf, size_t count)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
	char *buf2;
	const int step_temp_count = 1;
	unsigned long step_temp;
	int err = 0;

	/* Lock the device to prevent races with open/close (and itself) */
	mutex_lock(&kxcnl_mutex);

	if(kionix_strtok(buf, count, &buf2, step_temp_count) < 0) {
		GSE_ERR("%s: No enable data being read. " \
				"No enable data will be updated.\n", __func__);
	}

	else {
		/* Removes any leading negative sign */
		while(*buf2 == '-')
			buf2++;
#if (LINUX_VERSION_CODE > KERNEL_VERSION(2,6,35))
		err = kstrtouint((const char *)buf2, 10, (unsigned int *)&step_temp);
		if (err < 0) {
			GSE_ERR("%s: kstrtouint returned err = %d\n", __func__, err);
			goto exit;
		}
#else
		err = strict_strtoul((const char *)buf2, 10, &step_temp);
		if (err < 0) {
			GSE_ERR("%s: strict_strtoul returned err = %d\n", __func__, err);
			goto exit;
		}
#endif
		
		acceld->current_step = step_temp;
		//Reinitial KXCNL state machine here
		KXCNL_pedometer_start(acceld);
	}

exit:
	mutex_unlock(&kxcnl_mutex);

	return (err < 0) ? err : count;
}



/* Read and print current regs*/
static ssize_t show_full_regs(struct device_driver *ddri, char *buf)
{
	struct i2c_client *client = kxcnl_i2c_client;

	u8 temp[130]={0};
	u8 addr = 0;
	int err = 0;

	GSE_FUN();
       mutex_lock(&kxcnl_mutex);
	for(addr=0;addr<32;addr++)
	{
		err = hwmsen_read_block(client, addr*4, &temp[addr*4], 4);		//read out all 128 registers
		if (err < 0)	return err;	
	}
	
	mutex_unlock(&kxcnl_mutex);
	
	return sprintf(buf, "regs:\nWIA:0x%2x \
						 \nOUT:%2x,%2x,%2x,%2x,%2x,%2x	 LC:%2x,%2x    STAT:%2x \
						 \nCTRL:%2x,%2x,%2x,%2x	\
						 \n\nVFC:%2x,%2x,%2x,%2x	\
						 \nST1:%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x \nTIM2/1_1:%2x,%2x,%2x,%2x   THRES1_1: %2x	MA1:%2x  SETT1: %2x \
						 \nST2:%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x,%2x \nTIM2/1_2:%2x,%2x,%2x,%2x   THRES1_2: %2x	MA2:%2x  SETT2: %2x \n", 
						 temp[15],
						 temp[16],temp[17],temp[18],temp[19],temp[20],temp[21],
						 temp[22],temp[23],temp[24],
						 temp[27],temp[28],temp[29],temp[30],
						 temp[44],temp[45],temp[46],temp[47],
						 temp[64],temp[65],temp[66],temp[67],temp[68],temp[69],temp[70],temp[71],
						 temp[72],temp[73],temp[74],temp[75],temp[76],temp[77],temp[78],temp[79],
						 temp[82],temp[83],temp[84],temp[85],temp[87],temp[90],temp[91],
						 temp[96],temp[97],temp[98],temp[99],temp[100],temp[101],temp[102],temp[103],
						 temp[104],temp[105],temp[106],temp[107],temp[108],temp[109],temp[110],temp[111],
						 temp[114],temp[115],temp[116],temp[117],temp[119],temp[122],temp[123]);
}

static void kxcnl_eint_work(struct work_struct *work)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *sdata = i2c_get_clientdata(client);
	
	//s32 outs;
	u8 status;
	int err = 0;	
	u8	temp[2]={0};
	
	//status = i2c_smbus_read_byte_data(sdata->client, KXCNL_STAT);
      
	err = hwmsen_read_block(client, KXCNL_STAT, temp, 1);	
	if (err<0) return ;
	
	status = temp[0];	
	GSE_ERR("%s: statusReg=0x%x\n", __FUNCTION__, status);

	if (status & KXCNL_STAT_INT_SM1) {
		GSE_ERR("%s - KXCNL_STAT_INT_SM1\n", __FUNCTION__);
		//outs = i2c_smbus_read_byte_data(sdata->client,KXCNL_OUTS1);
	}

	if (status & KXCNL_STAT_INT_SM2) {
		GSE_ERR("%s - KXCNL_STAT_INT_SM2\n", __FUNCTION__);
		//outs = i2c_smbus_read_byte_data(sdata->client,KXCNL_OUTS2);
		
		/* TODO - sm1 - run step counter */
		/* TODO - sm2 - walk detect */
		if(sdata->pedo_mode == MODE_WALKING)
		{
			sdata->pedo_mode = MODE_RUNNING;
			KXCNL_updateStateMachine1(sdata, MODE_RUNNING);
			KXCNL_updateStateMachine2(sdata, MODE_RUNNING);
		}
		else
		{
			sdata->pedo_mode = MODE_WALKING;
			KXCNL_updateStateMachine1(sdata, MODE_WALKING);
			KXCNL_updateStateMachine2(sdata, MODE_WALKING);	
		}
	}

	if (status & KXCNL_STAT_LONG) {
	/* TODO - read out LC and reset */
	//Get step from KXCNL and re-initiate state machine
	sdata->current_step += KXCNL_get_current_step(sdata);
	KXCNL_pedometer_start(sdata);
	}

	return ;
}

void kxcnl_eint_handler(void)
{
	GSE_FUN();

	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
	
	schedule_work(&acceld->eint_work);
	//schedule_delayed_work(&obj->eint_work);

	return;
}


static int KXCNL_ReadTotalSteps(struct i2c_client *client, char *buf, int bufsize)
{
	struct kxcnl_i2c_data *obj = (struct kxcnl_i2c_data*)i2c_get_clientdata(client);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}
	//Get step from KXCNL and re-initiate state machine
	obj->current_step += KXCNL_get_current_step(obj);
	KXCNL_pedometer_start(obj);
	sprintf(buf, "%d", obj->current_step);
	
	return 0;
}

static int KXCNL_pedometer_get_mode(struct i2c_client *client, char *buf, int bufsize)
{
	struct kxcnl_i2c_data *obj = (struct kxcnl_i2c_data*)i2c_get_clientdata(client);
	u8 databuf[20];

	memset(databuf, 0, sizeof(u8)*10);

	if(NULL == buf)
	{
		return -1;
	}
	if(NULL == client)
	{
		*buf = 0;
		return -2;
	}

	sprintf(buf, "%d", obj->pedo_mode);
	
	return 0;
}
#endif	//KIONIX_KXCNL_HW_PEDOMETER, Added by Kionix

/*----------------------------------------------------------------------------*/
static DRIVER_ATTR(chipinfo,   S_IWUSR | S_IRUGO, show_chipinfo_value,      NULL);
static DRIVER_ATTR(sensordata, S_IWUSR | S_IRUGO, show_sensordata_value,    NULL);
static DRIVER_ATTR(cali,       S_IWUSR | S_IRUGO, show_cali_value,          store_cali_value);
static DRIVER_ATTR(selftest, S_IWUSR | S_IRUGO, show_self_value,  store_self_value);
static DRIVER_ATTR(self,   S_IWUSR | S_IRUGO, show_selftest_value,      store_selftest_value);
static DRIVER_ATTR(firlen,     S_IWUSR | S_IRUGO, show_firlen_value,        store_firlen_value);
static DRIVER_ATTR(trace,      S_IWUSR | S_IRUGO, show_trace_value,         store_trace_value);
static DRIVER_ATTR(status,               S_IRUGO, show_status_value,        NULL);
static DRIVER_ATTR(powerstatus,               S_IRUGO, show_power_status_value,        NULL);
#if 1// KIONIX_KXCNL_HW_PEDOMETER
static DRIVER_ATTR(pedo_enable, S_IRUGO|S_IWUSR, show_pedometer_enable, store_pedometer_enable);
static DRIVER_ATTR(pedo_mode, S_IRUGO, show_pedometer_mode, NULL);
static DRIVER_ATTR(step_counter, S_IRUGO|S_IWUSR, show_pedometer_step, store_pedometer_step);
static DRIVER_ATTR(dump_regs, S_IRUGO, show_full_regs, NULL);
#endif	//KIONIX_KXCNL_HW_PEDOMETER, Added by Kionix

/*----------------------------------------------------------------------------*/
static u8 i2c_dev_reg =0 ;

static ssize_t show_register(struct device_driver *pdri, char *buf)
{
	printk("i2c_dev_reg is 0x%2x \n", i2c_dev_reg);

	return 0;
}

static ssize_t store_register(struct device_driver *ddri, const char *buf, size_t count)
{
	i2c_dev_reg = simple_strtoul(buf, NULL, 16);
	printk("set i2c_dev_reg = 0x%2x \n", i2c_dev_reg);

	return 0;
}
static ssize_t store_register_value(struct device_driver *ddri, const char *buf, size_t count)
{
	struct kxcnl_i2c_data *obj = obj_i2c_data;
	u8 databuf[2];  
	unsigned long input_value;
	int res;
	
	memset(databuf, 0, sizeof(u8)*2);    

	input_value = simple_strtoul(buf, NULL, 16);
	printk("input_value = 0x%2x \n", (unsigned int)input_value);

	if(NULL == obj)
	{
		GSE_ERR("i2c data obj is null!!\n");
		return 0;
	}

	databuf[0] = i2c_dev_reg;
	databuf[1] = input_value;
	printk("databuf[0]=0x%2x  databuf[1]=0x%2x \n", databuf[0],databuf[1]);

	res = i2c_master_send(obj->client, databuf, 0x2);

	if(res <= 0)
	{
		return KXCNL_ERR_I2C;
	}
	return 0;
	
}

static ssize_t show_register_value(struct device_driver *ddri, char *buf)
{
		struct kxcnl_i2c_data *obj = obj_i2c_data;
		u8 databuf[1];	
		
		memset(databuf, 0, sizeof(u8)*1);	 
	
		if(NULL == obj)
		{
			GSE_ERR("i2c data obj is null!!\n");
			return 0;
		}
		
		if(hwmsen_read_block(obj->client, i2c_dev_reg, databuf, 0x01))
		{
			GSE_ERR("read power ctl register err!\n");
			return KXCNL_ERR_I2C;
		}

		printk("i2c_dev_reg=0x%2x  data=0x%2x \n", i2c_dev_reg,databuf[0]);
	
		return 0;
		
}


static DRIVER_ATTR(i2c,      S_IWUSR | S_IRUGO, show_register_value,         store_register_value);
static DRIVER_ATTR(register,      S_IWUSR | S_IRUGO, show_register,         store_register);


/*----------------------------------------------------------------------------*/
static struct driver_attribute *kxcnl_attr_list[] = {
	&driver_attr_chipinfo,     /*chip information*/
	&driver_attr_sensordata,   /*dump sensor data*/
	&driver_attr_cali,         /*show calibration data*/
	&driver_attr_self,         /*self test demo*/
	&driver_attr_selftest,     /*self control: 0: disable, 1: enable*/
	&driver_attr_firlen,       /*filter length: 0: disable, others: enable*/
	&driver_attr_trace,        /*trace log*/
	&driver_attr_status,
	&driver_attr_powerstatus,
	&driver_attr_register,
	&driver_attr_i2c,
#if	1//KIONIX_KXCNL_HW_PEDOMETER
	&driver_attr_pedo_enable.attr,
	&driver_attr_pedo_mode.attr,
	&driver_attr_step_counter.attr,	
	&driver_attr_dump_regs.attr,
#endif	//KIONIX_KXCNL_HW_PEDOMETER, Added by Kionix
};
/*----------------------------------------------------------------------------*/
static int kxcnl_create_attr(struct device_driver *driver) 
{
	int idx, err = 0;
	int num = (int)(sizeof(kxcnl_attr_list)/sizeof(kxcnl_attr_list[0]));
	if (driver == NULL)
	{
		return -EINVAL;
	}

	for(idx = 0; idx < num; idx++)
	{
		if(0 != (err = driver_create_file(driver, kxcnl_attr_list[idx])))
		{            
			GSE_ERR("driver_create_file (%s) = %d\n", kxcnl_attr_list[idx]->attr.name, err);
			break;
		}
	}    
	return err;
}
/*----------------------------------------------------------------------------*/
static int kxcnl_delete_attr(struct device_driver *driver)
{
	int idx ,err = 0;
	int num = (int)(sizeof(kxcnl_attr_list)/sizeof(kxcnl_attr_list[0]));

	if(driver == NULL)
	{
		return -EINVAL;
	}
	

	for(idx = 0; idx < num; idx++)
	{
		driver_remove_file(driver, kxcnl_attr_list[idx]);
	}
	

	return err;
}
/****************************************************************************** 
 * Function Configuration
******************************************************************************/
static int kxcnl_open(struct inode *inode, struct file *file)
{
	file->private_data = kxcnl_i2c_client;

	if(file->private_data == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	return nonseekable_open(inode, file);
}
/*----------------------------------------------------------------------------*/
static int kxcnl_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

#ifdef CONFIG_COMPAT
static long kxcnl_compat_ioctl(struct file *file, unsigned int cmd,
       unsigned long arg)
{
    long err = 0;

	void __user *arg32 = compat_ptr(arg);

	GSE_LOG("kxcnl_compat_ioctl(), cmd=0x%x\n", cmd);
	
	if (!file->f_op || !file->f_op->unlocked_ioctl)
		return -ENOTTY;
	
    switch (cmd)
    {
        case COMPAT_GSENSOR_IOCTL_READ_SENSORDATA:
            if (arg32 == NULL)
            {
                err = -EINVAL;
                break;    
            }
		
		err = file->f_op->unlocked_ioctl(file, GSENSOR_IOCTL_READ_SENSORDATA, arg32);
		if (err){
			GSE_ERR("GSENSOR_IOCTL_READ_SENSORDATA unlocked_ioctl failed.");
		       return err;
		}
        break;

        default:
            GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
            err = -ENOIOCTLCMD;
        break;

    }

    return err;
}
#endif

/*----------------------------------------------------------------------------*/
//static int kxcnl_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
//       unsigned long arg)
static long kxcnl_unlocked_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	struct i2c_client *client = (struct i2c_client*)file->private_data;
	struct kxcnl_i2c_data *obj = (struct kxcnl_i2c_data*)i2c_get_clientdata(client);	
	char strbuf[KXCNL_BUFSIZE];
	void __user *data;
	SENSOR_DATA sensor_data;
	long err = 0;
	int cali[3];

	GSE_LOG("kxcnl_unlocked_ioctl(), cmd=0x%x\n", cmd);
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
		GSE_ERR("access error: %08X, (%2d, %2d)\n", cmd, _IOC_DIR(cmd), _IOC_SIZE(cmd));
		return -EFAULT;
	}

	switch(cmd)
	{
		case GSENSOR_IOCTL_INIT:
			GSE_LOG("kxcnl_unlocked_ioctl(), GSENSOR_IOCTL_INIT\n");
			kxcnl_init_client(client, 0);			
			break;

		case GSENSOR_IOCTL_READ_CHIPINFO:
			GSE_LOG("kxcnl_unlocked_ioctl(), GSENSOR_IOCTL_READ_CHIPINFO\n");
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			
			KXCNL_ReadChipInfo(client, strbuf, KXCNL_BUFSIZE);
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
			KXCNL_SetPowerMode(obj->client, true);
			KXCNL_ReadSensorData(client, strbuf, KXCNL_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;

#if  1//KIONIX_KXCNL_HW_PEDOMETER		
		case GSENSOR_IOCTL_READ_TOTAL_STEPS:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			KXCNL_ReadTotalSteps(client, strbuf, KXCNL_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;		

		case GSENSOR_IOCTL_CLEAR_TOTAL_STEPS:
			obj->current_step = 0;
		//Reinitial KXCNL state machine here
			KXCNL_pedometer_start(obj);
			break;		

		case GSENSOR_IOCTL_GET_PEDO_MODE:
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}

			KXCNL_pedometer_get_mode(client, strbuf, KXCNL_BUFSIZE);
			if(copy_to_user(data, strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}				 
			break;			
#endif	//KIONIX_KXCNL_HW_PEDOMETER, Added by Kionix
			
		case GSENSOR_IOCTL_READ_GAIN:
			GSE_LOG("kxcnl_unlocked_ioctl(), GSENSOR_IOCTL_READ_GAIN\n");				
			data = (void __user *) arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}			
			
			if(copy_to_user(data, &gsensor_gain, sizeof(GSENSOR_VECTOR3D)))
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
			KXCNL_ReadRawData(client, strbuf);
			if(copy_to_user(data, &strbuf, strlen(strbuf)+1))
			{
				err = -EFAULT;
				break;	  
			}
			break;	  

		case GSENSOR_IOCTL_SET_CALI:
			GSE_LOG("kxcnl_unlocked_ioctl(), GSENSOR_IOCTL_SET_CALI\n");	
			return 0;  //  no gsensor calibration.
			
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
			if(atomic_read(&obj->suspend))
			{
				GSE_ERR("Perform calibration in suspend state!!\n");
				err = -EINVAL;
			}
			else
			{
				cali[KXCNL_AXIS_X] = sensor_data.x * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[KXCNL_AXIS_Y] = sensor_data.y * obj->reso->sensitivity / GRAVITY_EARTH_1000;
				cali[KXCNL_AXIS_Z] = sensor_data.z * obj->reso->sensitivity / GRAVITY_EARTH_1000;			  
				err = KXCNL_WriteCalibration(client, cali);			 
			}
			break;

		case GSENSOR_IOCTL_CLR_CALI:
			GSE_LOG("kxcnl_unlocked_ioctl(), GSENSOR_IOCTL_CLR_CALI\n");					
			err = KXCNL_ResetCalibration(client);
			break;

		case GSENSOR_IOCTL_GET_CALI:
			GSE_LOG("kxcnl_unlocked_ioctl(), GSENSOR_IOCTL_GET_CALI\n");					
			data = (void __user*)arg;
			if(data == NULL)
			{
				err = -EINVAL;
				break;	  
			}
			if(0 != (err = KXCNL_ReadCalibration(client, cali)))
			{
				break;
			}
			
			sensor_data.x = cali[KXCNL_AXIS_X] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.y = cali[KXCNL_AXIS_Y] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			sensor_data.z = cali[KXCNL_AXIS_Z] * GRAVITY_EARTH_1000 / obj->reso->sensitivity;
			if(copy_to_user(data, &sensor_data, sizeof(sensor_data)))
			{
				err = -EFAULT;
				break;
			}		
			break;
		

		default:
			GSE_ERR("unknown IOCTL: 0x%08x\n", cmd);
			err = -ENOIOCTLCMD;
			break;
			
	}

	return err;
}
 

/*----------------------------------------------------------------------------*/
static struct file_operations kxcnl_fops = {
	.owner = THIS_MODULE,
	.open = kxcnl_open,
	.release = kxcnl_release,
	.unlocked_ioctl = kxcnl_unlocked_ioctl,
	#ifdef CONFIG_COMPAT
	.compat_ioctl = kxcnl_compat_ioctl,
	#endif
	//.ioctl = kxcnl_ioctl,
};
/*----------------------------------------------------------------------------*/
static struct miscdevice kxcnl_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "gsensor",
	.fops = &kxcnl_fops,
};
/*----------------------------------------------------------------------------*/
#if !defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/
static int kxcnl_suspend(struct i2c_client *client, pm_message_t msg) 
{
  
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);    
	int err = 0;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}
	
	if(msg.event != PM_EVENT_SUSPEND)  
		return 0;

#if 1//KIONIX_KXCNL_HW_PEDOMETER
	if(obj->pedo_enabled)
	{
		GSE_LOG("stepcounter working, no suspend.\n");
		return 0;
	}
#endif

	mutex_lock(&kxcnl_mutex);
	atomic_set(&obj->suspend, 1);

	if(0 != (err = KXCNL_SetPowerMode(obj->client,false)))
	{
		GSE_ERR("write power control fail!!\n");
		mutex_unlock(&kxcnl_mutex);
		return -1;
	}
        mutex_unlock(&kxcnl_mutex);

	return err;
}
/*----------------------------------------------------------------------------*/
static int kxcnl_resume(struct i2c_client *client)
{
	struct kxcnl_i2c_data *obj = i2c_get_clientdata(client);        
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return -EINVAL;
	}

	if (atomic_read(&obj->suspend) == 0)
	{
		GSE_ERR("driver no suspend, return.\n");
		return 0;
	}

	mutex_lock(&kxcnl_mutex);
	if(0 != (err = kxcnl_init_client(client, 0)))
	{
		GSE_ERR("initialize client fail!!\n");
		mutex_unlock(&kxcnl_mutex);
		return err;        
	}
	atomic_set(&obj->suspend, 0);
    mutex_unlock(&kxcnl_mutex);

	return 0;
}
/*----------------------------------------------------------------------------*/
#else //!defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/
static void kxcnl_early_suspend(struct early_suspend *h) 
{
	struct kxcnl_i2c_data *obj = container_of(h, struct kxcnl_i2c_data, early_drv);   
	int err;
	GSE_FUN();    

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

#if  1//KIONIX_KXCNL_HW_PEDOMETER
	if(obj->pedo_enabled)
	{
		GSE_LOG("stepcounter working, no suspend.\n");
		return;
	}
#endif

	mutex_lock(&kxcnl_mutex);
	atomic_set(&obj->suspend, 1);

	if(err = KXCNL_SetPowerMode(obj->client, false))
	{
		GSE_ERR("write power control fail!!\n");
		mutex_unlock(&kxcnl_mutex);
		return;
	}
	mutex_unlock(&kxcnl_mutex);

	//sensor_power = false;
}
/*----------------------------------------------------------------------------*/
static void kxcnl_late_resume(struct early_suspend *h)
{
	struct kxcnl_i2c_data *obj = container_of(h, struct kxcnl_i2c_data, early_drv);         
	int err;
	GSE_FUN();

	if(obj == NULL)
	{
		GSE_ERR("null pointer!!\n");
		return;
	}

	if (atomic_read(&obj->suspend) == 0)
	{
		GSE_ERR("driver no suspend, return.\n");
		return 0;
	}

	mutex_lock(&kxcnl_mutex);
	if(err = kxcnl_init_client(obj->client, 0))
	{
		GSE_ERR("initialize client fail!!\n");
		mutex_unlock(&kxcnl_mutex);
		return;        
	}
	atomic_set(&obj->suspend, 0); 
	mutex_unlock(&kxcnl_mutex);
}
/*----------------------------------------------------------------------------*/
#endif //!defined(CONFIG_HAS_EARLYSUSPEND) || !defined(USE_EARLY_SUSPEND)
/*----------------------------------------------------------------------------*/

// if use  this typ of enable , Gsensor should report inputEvent(x, y, z ,stats, div) to HAL
static int kxcnl_open_report_data(int open)
{
	//should queuq work to report event if  is_report_input_direct=true
	//GSE_FUN();
	return 0;
}

// if use  this typ of enable , Gsensor only enabled but not report inputEvent to HAL

static int kxcnl_enable_nodata(int en)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
	int err = 0;
	
	GSE_LOG("%s:en=%d", __func__, en);
	mutex_lock(&kxcnl_mutex);

	if(en == 1)  g_bGsensorOnoff = true;
	if(en == 0)  g_bGsensorOnoff = false;
	
	// if step_counter is work, no power down.
       if((0 == en)&&(acceld->pedo_enabled==1))  //kaka
       {
       	mutex_unlock(&kxcnl_mutex);
		printk("kxcnl_enable_nodata: pedo working, no power down!\n");
		return 0;
       }
	   
	if (atomic_read(&obj_i2c_data->suspend) == 0)
	{
		err = KXCNL_SetPowerMode(obj_i2c_data->client, g_bGsensorOnoff);
		GSE_LOG("kxcnl not in suspend, power_status = %d\n",g_bGsensorOnoff);
	}
	else
	{
		GSE_LOG("kxcnl in suspend ! Do nothing.\n");
	}
	mutex_unlock(&kxcnl_mutex);

	if(err != KXCNL_SUCCESS)
	{
		printk("kxcnl_enable_nodata fail!\n");
		return -1;
	}

	printk("kxcnl_enable_nodata OK!\n");
	return 0;
}

#if  1//KIONIX_KXCNL_HW_PEDOMETER
static int kxcnl_pedometer_enable(int en)
{
	int err = 0;

	GSE_LOG("kxcnl_pedometer_enable, en=%d\n", en);
	mutex_lock(&kxcnl_mutex);
	if(en)
	{
		obj_i2c_data->pedo_enabled = 1;
		err = KXCNL_pedometer_start(obj_i2c_data);
	}
	else
	{
		obj_i2c_data->pedo_enabled = 0;
		err = KXCNL_pedometer_stop(obj_i2c_data);
	}
	mutex_unlock(&kxcnl_mutex);

	if(err != KXCNL_SUCCESS)
	{
		printk("kxcnl_enable_pedometer fail!\n");
		return -1;
	}

	GSE_LOG("kxcnl_enable_pedometer OK!\n");
	return 0;
}
#endif

static int kxcnl_set_delay(u64 ns)
{
    int err = 0;
    int value;
	int sample_delay;

    value = (int)ns/1000/1000;

	if(value <= 5)
	{
		sample_delay = KXCNL_BW_400HZ;
	}
	else if(value <= 10)
	{
		sample_delay = KXCNL_BW_100HZ;
	}
	else
	{
		sample_delay = KXCNL_BW_50HZ;
	}

	mutex_lock(&kxcnl_mutex);
	err = KXCNL_SetBWRate(obj_i2c_data->client, sample_delay);
	mutex_unlock(&kxcnl_mutex);
	if(err != KXCNL_SUCCESS ) //0x2C->BW=100Hz
	{
		GSE_ERR("Set delay parameter error!\n");
        return -1;
	}

	if(value >= 50)
	{
		atomic_set(&obj_i2c_data->filter, 0);
	}
	else
	{	
	#if defined(CONFIG_KXCNL_LOWPASS)
		priv->fir.num = 0;
		priv->fir.idx = 0;
		priv->fir.sum[KXCNL_AXIS_X] = 0;
		priv->fir.sum[KXCNL_AXIS_Y] = 0;
		priv->fir.sum[KXCNL_AXIS_Z] = 0;
		atomic_set(&priv->filter, 1);
	#endif
	}
   	GSE_LOG("kxcnl_set_delay (%d)\n",value);

	return 0;
}

static int kxcnl_set_batch(int flags, int64_t period_ns, int64_t timeout)
{
    int err = 0;
    return err;
}

static int kxcnl_get_data(int* x ,int* y,int* z, int* status)
{
	char buff[KXCNL_BUFSIZE];

	mutex_lock(&kxcnl_mutex);
	KXCNL_ReadSensorData(obj_i2c_data->client, buff, KXCNL_BUFSIZE);
	mutex_unlock(&kxcnl_mutex);

	sscanf(buff, "%x %x %x", x, y, z);				
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	return 0;
}

/*----------------------------------------------------------------------------*/
static int kxcnl_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_client *new_client;
	struct kxcnl_i2c_data *obj;
	struct acc_control_path ctl={0};
    struct acc_data_path data={0};
	int err = 0;
	GSE_FUN();

	if(!(obj = kzalloc(sizeof(*obj), GFP_KERNEL)))
	{
		err = -ENOMEM;
		goto exit;
	}
	
	memset(obj, 0, sizeof(struct kxcnl_i2c_data));

	obj->hw = kxcnl_get_cust_acc_hw();
	
	GSE_LOG("%s-1\n", __func__);
	
	if(0 != (err = hwmsen_get_convert(obj->hw->direction, &obj->cvt)))
	{
		GSE_ERR("invalid direction: %d\n", obj->hw->direction);
		goto exit;
	}

#if 1//def KIONIX_KXCNL_HW_PEDOMETER
	INIT_WORK(&obj->eint_work, kxcnl_eint_work);
#endif

	obj_i2c_data = obj;
	obj->client = client;
	new_client = obj->client;
	i2c_set_clientdata(new_client,obj);
	
	atomic_set(&obj->trace, 0);
	atomic_set(&obj->suspend, 0);
	
	GSE_LOG("%s-2\n", __func__);

#ifdef CONFIG_KXCNL_LOWPASS
	if(obj->hw->firlen > C_MAX_FIR_LENGTH)
	{
		atomic_set(&obj->firlen, C_MAX_FIR_LENGTH);
	}	
	else
	{
		atomic_set(&obj->firlen, obj->hw->firlen);
	}
	
	if(atomic_read(&obj->firlen) > 0)
	{
		atomic_set(&obj->fir_en, 1);
	}
	
#endif

	kxcnl_i2c_client = new_client;	

	GSE_LOG("%s-3\n", __func__);
	
	if(0 != (err = kxcnl_init_client(new_client, 1)))
	{
		GSE_LOG("%s-3.5\n", __func__);
		goto exit_init_failed;
	}
	GSE_LOG("%s-4\n", __func__);

	if(0 != (err = misc_register(&kxcnl_device)))
	{
		GSE_ERR("kxcnl_device register failed\n");
		goto exit_misc_device_register_failed;
	}

	if(0 != (err = kxcnl_create_attr(&kxcnl_init_info.platform_diver_addr->driver)))
	{
		GSE_ERR("create attribute err = %d\n", err);
		goto exit_create_attr_failed;
	}
	GSE_LOG("%s-5\n", __func__);
	
#if  1//KIONIX_KXCNL_HW_PEDOMETER
#if 0
		//Add interrupt configuration here
	err = gpio_request(MTK_IRQ1_GPIO,"kxcnl_interrupt_1");
	if (err) {
		printk("gpio_request(%d) kxcnl_interrupt_1 fail %d\n",MTK_IRQ1_GPIO, err);
	}
	err = gpio_direction_input(MTK_IRQ1_GPIO);
	if (err) {
		printk("gpio_direction_input(%d) fail %d\n",MTK_IRQ1_GPIO, err);
	}
	
	obj->irq1 = gpio_to_irq(MTK_IRQ1_GPIO);
	
	GSE_LOG("%s-6\n", __func__);
	
	err = request_threaded_irq(obj->irq1, NULL,
				kxcnl_irq_handler, IRQF_TRIGGER_RISING | IRQF_ONESHOT,
				"kxcnl_irq1", obj);
	if (err) {
		dev_err(&client->dev, "unable to request irq1\n");
		goto exit_init_failed;
	}

#else
	mt_set_gpio_dir(GPIO_GSE_1_EINT_PIN, GPIO_DIR_IN);
	mt_set_gpio_mode(GPIO_GSE_1_EINT_PIN, GPIO_GSE_1_EINT_PIN_M_EINT);
	mt_set_gpio_pull_enable(GPIO_GSE_1_EINT_PIN, TRUE);
	mt_set_gpio_pull_select(GPIO_GSE_1_EINT_PIN, GPIO_PULL_UP);

	mt_eint_set_hw_debounce(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_DEBOUNCE_CN);
	mt_eint_registration(CUST_EINT_GSE_1_NUM, CUST_EINT_GSE_1_TYPE, kxcnl_eint_handler, 0);
	mt_eint_unmask(CUST_EINT_GSE_1_NUM);  
	irq_set_irq_wake(GPIO_GSE_1_EINT_PIN, 1);
#endif	
		
#endif
	GSE_LOG("%s-7\n", __func__);
	
	ctl.open_report_data= kxcnl_open_report_data;
	ctl.enable_nodata = kxcnl_enable_nodata;
	ctl.set_delay  = kxcnl_set_delay;
	
#if  1//KIONIX_KXCNL_HW_PEDOMETER
	ctl.pedo_enable  = kxcnl_pedometer_enable;
#endif
    //ctl.batch = kxcnl_set_batch;
	ctl.is_report_input_direct = false;
	ctl.is_support_batch = false;

	err = acc_register_control_path(&ctl);
	if(err)
	{
	 	GSE_ERR("register acc control path err\n");
		goto exit_create_attr_failed;
	}

	GSE_LOG("%s-8\n", __func__);
	
	data.get_data = kxcnl_get_data;
	data.vender_div = 1000;
	err = acc_register_data_path(&data);
	if(err)
	{
	 	GSE_ERR("register acc data path err\n");
		goto exit_create_attr_failed;
	}

	GSE_LOG("%s-9\n", __func__);
		
	err = batch_register_support_info(ID_ACCELEROMETER,ctl.is_support_batch, 102, 0); //divisor is 1000/9.8
    if(err)
    {
        GSE_ERR("register gsensor batch support err = %d\n", err);
        goto exit_create_attr_failed;
    }

	GSE_LOG("%s-10\n", __func__);

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(USE_EARLY_SUSPEND)
	obj->early_drv.level    = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	obj->early_drv.suspend  = kxcnl_early_suspend,
	obj->early_drv.resume   = kxcnl_late_resume,    
	register_early_suspend(&obj->early_drv);
#endif 

    kxcnl_init_flag =KXCNL_INIT_OK;
	GSE_LOG("%s: OK\n", __func__);    
	return 0;

exit_create_attr_failed:
	misc_deregister(&kxcnl_device);
exit_misc_device_register_failed:
exit_init_failed:
	kfree(obj);
exit:
	GSE_ERR("%s: err = %d\n", __func__, err);
	kxcnl_init_flag =KXCNL_INIT_FAIL;
	return err;
}

/*----------------------------------------------------------------------------*/
static int kxcnl_i2c_remove(struct i2c_client *client)
{
	int err = 0;	
	
	if(0 != (err = kxcnl_delete_attr(&(kxcnl_init_info.platform_diver_addr->driver))))
	{
		GSE_ERR("kxcnl_delete_attr fail: %d\n", err);
	}
	
	if(0 != (err = misc_deregister(&kxcnl_device)))
	{
		GSE_ERR("misc_deregister fail: %d\n", err);
	}

	kxcnl_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(i2c_get_clientdata(client));
	return 0;
}
/*----------------------------------------------------------------------------*/
static int kxcnl_remove(void)
{
    struct acc_hw *hw = kxcnl_get_cust_acc_hw();

    GSE_FUN();    
    KXCNL_power(hw, 0);    
    i2c_del_driver(&kxcnl_i2c_driver);
    return 0;
}

static int  kxcnl_local_init(void)
{
    struct acc_hw *hw = kxcnl_get_cust_acc_hw();
	GSE_FUN();

	if(KXCNL_INIT_FAIL== kxcnl_init_flag)
	{
		GSE_ERR("kxcnl_local_init   kxcnl_init_flag=fail\n");
	   	return -1;	
	}	
	else if(KXCNL_INIT_OK== kxcnl_init_flag)
	{
		GSE_ERR("kxcnl_local_init   kxcnl_init_flag=ok\n");
	   	return 0;	
	}	

	//KXCNL_INIT_NODO== kxcnl_init_flag
	KXCNL_power(hw, 1);
	if(i2c_add_driver(&kxcnl_i2c_driver))
	{
		GSE_ERR("add driver error\n");
		KXCNL_power(hw, 0);
		return -1;
	}

	if(KXCNL_INIT_FAIL== kxcnl_init_flag)
	{
		i2c_del_driver(&kxcnl_i2c_driver);
		GSE_ERR("kxcnl_local_init   kxcnl_init_flag2=-1\n");
	   	return -1;	
	}	
	return 0;
}

#if  1//def KIONIX_KXCNL_HW_PEDOMETER
static int kxcnl_step_c_open_report_data(int open)
{
	GSE_FUN();	
	return 0;
}
static int kxcnl_step_c_enable_nodata(int en)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
	int err=-1;
	bool stepC_onoff = false;

	GSE_LOG("%s, en=%d\n", __FUNCTION__, en);

	mutex_lock(&kxcnl_mutex);	
	if (en == 1)  stepC_onoff = true;
	if (en == 0)  stepC_onoff = false;
	
	// if step_counter is work, no power down.
       if ((0 == en)&&(g_bGsensorOnoff==1))
       {
       	mutex_unlock(&kxcnl_mutex);
		printk("kxcnl_enable_nodata: pedo working, no power down!\n");
		return 0;
       }
	   
	if (atomic_read(&obj_i2c_data->suspend) == 0)
	{
		err = KXCNL_SetPowerMode(obj_i2c_data->client, stepC_onoff);
		GSE_LOG("kxcnl not in suspend, power_status = %d\n",stepC_onoff);
	}
	else
	{
		GSE_LOG("kxcnl in suspend ! Do nothing.\n");
	}

	if(en == 1)                // input 1
	{
		acceld->pedo_enabled = 1;
		err = KXCNL_pedometer_start(acceld);
	}
	else                           // input 0
	{
		acceld->pedo_enabled = 0;
		err = KXCNL_pedometer_stop(acceld);
	}
	mutex_unlock(&kxcnl_mutex);
       return err;
}

static int kxcnl_step_c_enable_step_detect(int en)
{
	GSE_FUN();
	return kxcnl_step_c_enable_nodata(en);
}

static int kxcnl_step_c_set_delay(u64 delay)
{
	GSE_FUN();	
	return 0;
}

static int kxcnl_step_c_sigmotion(int en)
{
	return 0;
}

static int kxcnl_step_c_get_data(u64 *value, int *status)
{
	struct i2c_client *client = kxcnl_i2c_client;
	struct kxcnl_i2c_data *acceld = i2c_get_clientdata(client);
       int data =0;

	data =  KXCNL_get_current_step(acceld);
	if (data<0) data=0;
		
	GSE_LOG("kxcnl_step_c_getdata, datainchip=%d, current_step=%d\n", data, acceld->current_step+data);
	data += acceld->current_step;

	*value = (u64)data;
	*status = SENSOR_STATUS_ACCURACY_MEDIUM;
	
	return 0;
}
static int kxcnl_step_c_get_data_step_d(u64 *value, int *status)
{
	GSE_FUN();
	return 0;
}
static int kxcnl_step_c_get_data_significant(u64 *value, int *status)
{
	GSE_FUN();
	return 0;
}

static int kxcnl_step_c_local_init(void)
{
	struct acc_hw *hw = kxcnl_get_cust_acc_hw();
	struct step_c_control_path step_ctl={0};
	struct step_c_data_path step_data={0};	
	int res = 0;

       GSE_FUN();

	if(KXCNL_INIT_FAIL== kxcnl_init_flag)
	{
		GSE_ERR("kxcnl_step_c_local_init   kxcnl_init_flag=-1\n");
	   	return -1;	
	}	

	if (KXCNL_INIT_NODO == kxcnl_init_flag)
	{
		KXCNL_power(hw, 1);
		if(i2c_add_driver(&kxcnl_i2c_driver))
		{
			GSE_ERR("add driver error\n");
			KXCNL_power(hw, 0);
			return -1;
		}
		
		if(KXCNL_INIT_FAIL == kxcnl_init_flag)
		{
			GSE_ERR("kxcnl_step_c_local_init   kxcnl_init_flag2=-1\n");
			i2c_del_driver(&kxcnl_i2c_driver);
	   		return -1;	
		}
	}

	// (kxcnl_init_flag== KXCNL_INIT_OK)
	mutex_lock(&kxcnl_mutex);

	KXCNL_power(hw, 1);

	step_ctl.open_report_data= kxcnl_step_c_open_report_data;
	step_ctl.enable_nodata = kxcnl_step_c_enable_nodata;
	step_ctl.enable_step_detect  = kxcnl_step_c_enable_step_detect;
	step_ctl.enable_significant = kxcnl_step_c_sigmotion;
	step_ctl.set_delay = kxcnl_step_c_set_delay;
	step_ctl.is_report_input_direct = false;
	step_ctl.is_support_batch = false;		

	res = step_c_register_control_path(&step_ctl);
	if(res)
	{
		 GSE_ERR("register step counter control path err\n");
		goto step_c_local_init_failed;
	}
		
	step_data.get_data = kxcnl_step_c_get_data;
	step_data.get_data_step_d =kxcnl_step_c_get_data_step_d;
	step_data.get_data_significant = kxcnl_step_c_get_data_significant;
	step_data.vender_div = 1;
	res = step_c_register_data_path(&step_data);
	if(res)
	{
		GSE_ERR("register step counter data path err= %d\n", res);
		goto step_c_local_init_failed;
	}

	mutex_unlock(&kxcnl_mutex);
	return 0;
	
step_c_local_init_failed:
	mutex_unlock(&kxcnl_mutex);
	GSE_ERR("%s init failed!\n", __FUNCTION__);
	return res;
}

static int kxcnl_step_c_local_uninit(void)
{
	GSE_FUN();
	return 0;
}
#endif


/*----------------------------------------------------------------------------*/
static int __init kxcnl_init(void)
{
	struct acc_hw *hw = kxcnl_get_cust_acc_hw();

	GSE_LOG("%s: i2c_number=%d\n", __func__,hw->i2c_num);
	i2c_register_board_info(hw->i2c_num, &i2c_kxcnl, 1);
	acc_driver_add(&kxcnl_init_info);
#if  1//KIONIX_KXCNL_HW_PEDOMETER
	step_c_driver_add(&kxcnl_step_c_init_info); //step counter
#endif		
	return 0;    
}
/*----------------------------------------------------------------------------*/
static void __exit kxcnl_exit(void)
{
	GSE_FUN();
}
/*----------------------------------------------------------------------------*/
module_init(kxcnl_init);
module_exit(kxcnl_exit);
/*----------------------------------------------------------------------------*/
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("KXCNL I2C driver");
MODULE_AUTHOR("Dexiang.Liu@mediatek.com");
//lichengmin end