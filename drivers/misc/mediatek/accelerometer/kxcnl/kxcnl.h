//lichengmin begin
/* linux/drivers/hwmon/adxl345.c
 *
 * (C) Copyright 2008 
 * MediaTek <www.mediatek.com>
 *
 * KXCNL driver for MT6575
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  KXTIK
 */
#ifndef KXCNL_H
#define KXCNL_H
	 
#include <linux/ioctl.h>
	 
#define KXCNL_I2C_SLAVE_ADDR		0x3D		//addr=0:0x3D, addr=1:0x3B
	 
 /* KXCNL Register Map  (Please refer to KXCNL Specifications) */
#define KXCNL_REG_DEVID						0x0F	//
#define	KXCNL_REG_BW_RATE					0x1B	//
#define KXCNL_REG_POWER_CTL					0x1B	//
#define KXCNL_REG_CTL_REG3					0x1D
#define KXCNL_REG_DATA_FORMAT				0x1B	//
#define KXCNL_REG_DATAX0					0x10	// 
#define KXCNL_BW_MASK						0x1C	//
#define KXCNL_BW_400HZ						0x18	//
#define KXCNL_BW_100HZ						0x14	//
#define KXCNL_BW_50HZ						0x10	// 
#define KXCNL_MEASURE_MODE					0x80	//	 
#define KXCNL_RANGE_MASK					0x60	//
#define KXCNL_RANGE_2G						0x00	//
#define KXCNL_RANGE_4G						0x20	//
#define KXCNL_RANGE_8G						0x60	//
#define KXCNL_REG_INT_ENABLE				0x1E

#define KXCNL_SELF_TEST           			0x1E	//
	 	 
#define KXCNL_DEVICE_ID						0x0B	//
		 
#define KXCNL_SUCCESS						0
#define KXCNL_ERR_I2C						-1
#define KXCNL_ERR_STATUS					-3
#define KXCNL_ERR_SETUP_FAILURE				-4
#define KXCNL_ERR_GETGSENSORDATA			-5
#define KXCNL_ERR_IDENTIFICATION			-6
	 
#define KXCNL_VFC1_DEFAULT_VALUE             0x7d
#define KXCNL_VFC2_DEFAULT_VALUE             0x72
#define KXCNL_VFC3_DEFAULT_VALUE             0x4c
#define KXCNL_VFC4_DEFAULT_VALUE             0x26
	 
#define KXCNL_BUFSIZE						256
	 
#define KXCNL_AXES_NUM        				3

#define MODE_WALKING  1    //zhangxin add temp
#define MODE_RUNNING  2
#define GSENSOR_IOCTL_READ_TOTAL_STEPS     _IOR(GSENSOR, 0x09, int)//20008509
#define GSENSOR_IOCTL_CLEAR_TOTAL_STEPS   _IO(GSENSOR, 0x0a)//0000850a
#define GSENSOR_IOCTL_GET_PEDO_MODE          _IOR(GSENSOR, 0x0b, int)//2000850b

/*----------------------------------------------------------------------------*/
typedef enum{
    KXCNL_CUST_ACTION_SET_CUST = 1,
    KXCNL_CUST_ACTION_SET_CALI,
    KXCNL_CUST_ACTION_RESET_CALI
}CUST_ACTION;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
}KXCNL_CUST;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    uint16_t    part;
    int32_t     data[0];
}KXCNL_SET_CUST;
/*----------------------------------------------------------------------------*/
typedef struct
{
    uint16_t    action;
    int32_t     data[KXCNL_AXES_NUM];
}KXCNL_SET_CALI;
/*----------------------------------------------------------------------------*/
typedef KXCNL_CUST KXCNL_RESET_CALI;
/*----------------------------------------------------------------------------*/
typedef union
{
    uint32_t                data[10];
    KXCNL_CUST         cust;
    KXCNL_SET_CUST     setCust;
    KXCNL_SET_CALI     setCali;
    KXCNL_RESET_CALI   resetCali;
}KXCNL_CUST_DATA;
/*----------------------------------------------------------------------------*/
extern void mt_eint_unmask(unsigned int eint_num);
extern void mt_eint_set_hw_debounce(unsigned int eint_num, unsigned int ms);
extern void mt_eint_registration(unsigned int eint_num, unsigned int flow, void (EINT_FUNC_PTR)(void), unsigned int is_auto_umask);
#endif

//lichengmin end