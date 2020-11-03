//lichengmin begin
/* include/linux/input/kxcnl_regs.h - Kionix accelerometer driver
 *
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Kuching Tan <kuchingtan@kionix.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef __KXCNL_REGS_H__
#define __KXCNL_REGS_H__

 /* registers */
#define KXCNL_INFO1 0x0D
#define KXCNL_INFO2 0x0E
#define KXCNL_WIA 0x0F
#define KXCNL_OUTX_L 0x10
#define KXCNL_OUTY_L 0x12
#define KXCNL_OUTZ_L 0x14
#define KXCNL_LC_L 0x16
#define KXCNL_LC_H 0x17
#define KXCNL_STAT 0x18
#define KXCNL_PEAK1 0x19
#define KXCNL_PEAK2 0x1A
#define KXCNL_CNTL1 0x1B
#define KXCNL_CNTL2 0x1C
#define KXCNL_CNTL3 0x1D
#define KXCNL_CNTL4 0x1E
#define KXCNL_THRS3 0x1F
#define KXCNL_OFF_X 0x20
#define KXCNL_OFF_Y 0x21
#define KXCNL_OFF_Z 0x22
#define KXCNL_CS_X 0x24
#define KXCNL_CS_Y 0x25
#define KXCNL_CS_Z 0x26
#define KXCNL_X_DEBUG 0x28
#define KXCNL_Y_DEBUG 0x29
#define KXCNL_Z_DEBUG 0x2A
#define KXCNL_VFC_1 0x2C
#define KXCNL_VFC_2 0x2D
#define KXCNL_VFC_3 0x2E
#define KXCNL_VFC_4 0x2F
#define KXCNL_ST1_1 0x40
#define KXCNL_TIM4_1 0x50
#define KXCNL_TIM3_1 0x51
#define KXCNL_TIM2_1_L 0x52
#define KXCNL_TIM1_1_L 0x54
#define KXCNL_THRS2_1 0x56
#define KXCNL_THRS1_1 0x57
#define KXCNL_SA1 0x59
#define KXCNL_MA1 0x5A
#define KXCNL_SETT1 0x5B
#define KXCNL_PPRP1 0x5C
#define KXCNL_TC1_L 0x5D
#define KXCNL_OUTS1 0x5F
#define KXCNL_ST1_2 0x60
#define KXCNL_TIM4_2 0x70
#define KXCNL_TIM3_2 0x71
#define KXCNL_TIM2_2_L 0x72
#define KXCNL_TIM1_2_L 0x74
#define KXCNL_THRS2_2 0x76
#define KXCNL_THRS1_2 0x77
#define KXCNL_DES2 0x78
#define KXCNL_SA2 0x79
#define KXCNL_MA2 0x7A
#define KXCNL_SETT2 0x7B
#define KXCNL_PPRP2 0x7C
#define KXCNL_TC2_L 0x7D
#define KXCNL_OUTS2 0x7F

 /* registers bits */ 
#define KXCNL_STAT_LONG (0x01 << 7)
#define KXCNL_STAT_SYNCW (0x01 << 6)
#define KXCNL_STAT_SYNC1 (0x01 << 5)
#define KXCNL_STAT_SYNC2 (0x01 << 4)
#define KXCNL_STAT_INT_SM1 (0x01 << 3)
#define KXCNL_STAT_INT_SM2 (0x01 << 2)
#define KXCNL_STAT_DOR (0x01 << 1)
#define KXCNL_STAT_DRDY (0x01 << 0)

//controls the operating mode of the KXCNL.
#define KXCNL_CNTL1_PC (0x01 << 7)
#define KXCNL_CNTL1_SC_2g (0x0 << 5)
#define KXCNL_CNTL1_SC_4g (0x1 << 5)
#define KXCNL_CNTL1_SC_6g (0x2 << 5)
#define KXCNL_CNTL1_SC_8g (0x3 << 5)
#define KXCNL_CNTL1_ODR_3p125 (0x0 << 2)
#define KXCNL_CNTL1_ODR_6p25 (0x1 << 2)
#define KXCNL_CNTL1_ODR_12p5 (0x2 << 2)
#define KXCNL_CNTL1_ODR_25 (0x3 << 2)
#define KXCNL_CNTL1_ODR_50 (0x4 << 2)
#define KXCNL_CNTL1_ODR_100 (0x5 << 2)
#define KXCNL_CNTL1_ODR_400 (0x6 << 2)
#define KXCNL_CNTL1_ODR_1600 (0x7 << 2)
#define KXCNL_CNTL1_DEBUG (0x01 << 1)
#define KXCNL_CNTL1_IEN (0x01 << 0)
#define KXCNL_CNTL2_SM1_PIN (0x01 << 3)
#define KXCNL_CNTL2_SM1_EN (0x01 << 0)
#define KXCNL_CNTL3_SM2_PIN (0x01 << 3)
#define KXCNL_CNTL3_SM2_EN (0x01 << 0)
#define KXCNL_CNTL4_DR_EN (0x01 << 7)
#define KXCNL_CNTL4_IEA (0x01 << 6)
#define KXCNL_CNTL4_IEL (0x01 << 5)
#define KXCNL_CNTL4_INT2_EN (0x01 << 4)
#define KXCNL_CNTL4_INT1_EN (0x01 << 3)
#define KXCNL_CNTL4_VFILT (0x01 << 2)
#define KXCNL_CNTL4_STP (0x01 << 1)
#define KXCNL_CNTL4_STRT (0x01 << 0)
#define KXCNL_SA1_P_X (0x01 << 7)
#define KXCNL_SA1_N_X (0x01 << 6)
#define KXCNL_SA1_P_Y (0x01 << 5)
#define KXCNL_SA1_N_Y (0x01 << 4)
#define KXCNL_SA1_P_Z (0x01 << 3)
#define KXCNL_SA1_N_Z (0x01 << 2)
#define KXCNL_SA1_P_V (0x01 << 1)
#define KXCNL_SA1_N_V (0x01 << 0)
#define KXCNL_MA1_P_X (0x01 << 7)
#define KXCNL_MA1_N_X (0x01 << 6)
#define KXCNL_MA1_P_Y (0x01 << 5)
#define KXCNL_MA1_N_Y (0x01 << 4)
#define KXCNL_MA1_P_Z (0x01 << 3)
#define KXCNL_MA1_N_Z (0x01 << 2)
#define KXCNL_MA1_P_V (0x01 << 1)
#define KXCNL_MA1_N_V (0x01 << 0)
#define KXCNL_SETT1_P_DET (0x01 << 7)
#define KXCNL_SETT1_THR3_SA (0x01 << 6)
#define KXCNL_SETT1_ABS (0x01 << 5)
#define KXCNL_SETT1_ABS_UNSIGNED (0x0 << 5)
#define KXCNL_SETT1_ABS_SIGNED (0x1 << 5)
#define KXCNL_SETT1_THR3_MA (0x01 << 2)
#define KXCNL_SETT1_R_TAM (0x01 << 1)
#define KXCNL_SETT1_SITR (0x01 << 0)
#define KXCNL_OUTS1_P_X (0x01 << 7)
#define KXCNL_OUTS1_N_X (0x01 << 6)
#define KXCNL_OUTS1_P_Y (0x01 << 5)
#define KXCNL_OUTS1_N_Y (0x01 << 4)
#define KXCNL_OUTS1_P_Z (0x01 << 3)
#define KXCNL_OUTS1_N_Z (0x01 << 2)
#define KXCNL_OUTS1_P_V (0x01 << 1)
#define KXCNL_OUTS1_N_V (0x01 << 0)
#define KXCNL_SA2_P_X (0x01 << 7)
#define KXCNL_SA2_N_X (0x01 << 6)
#define KXCNL_SA2_P_Y (0x01 << 5)
#define KXCNL_SA2_N_Y (0x01 << 4)
#define KXCNL_SA2_P_Z (0x01 << 3)
#define KXCNL_SA2_N_Z (0x01 << 2)
#define KXCNL_SA2_P_V (0x01 << 1)
#define KXCNL_SA2_N_V (0x01 << 0)
#define KXCNL_MA2_P_X (0x01 << 7)
#define KXCNL_MA2_N_X (0x01 << 6)
#define KXCNL_MA2_P_Y (0x01 << 5)
#define KXCNL_MA2_N_Y (0x01 << 4)
#define KXCNL_MA2_P_Z (0x01 << 3)
#define KXCNL_MA2_N_Z (0x01 << 2)
#define KXCNL_MA2_P_V (0x01 << 1)
#define KXCNL_MA2_N_V (0x01 << 0)
#define KXCNL_SETT2_P_DET (0x01 << 7)
#define KXCNL_SETT2_THR3_SA (0x01 << 6)
#define KXCNL_SETT2_ABS_UNSIGNED (0x0 << 5)
#define KXCNL_SETT2_ABS_SIGNED (0x1 << 5)
#define KXCNL_SETT2_RADI (0x01 << 4)
#define KXCNL_SETT2_D_CS (0x01 << 3)
#define KXCNL_SETT2_THR3_MA (0x01 << 2)
#define KXCNL_SETT2_R_TAM (0x01 << 1)
#define KXCNL_SETT2_SITR (0x01 << 0)
#define KXCNL_OUTS2_P_X (0x01 << 7)
#define KXCNL_OUTS2_N_X (0x01 << 6)
#define KXCNL_OUTS2_P_Y (0x01 << 5)
#define KXCNL_OUTS2_N_Y (0x01 << 4)
#define KXCNL_OUTS2_P_Z (0x01 << 3)
#define KXCNL_OUTS2_N_Z (0x01 << 2)
#define KXCNL_OUTS2_P_V (0x01 << 1)

 /*registers bit masks */
#define KXCNL_STAT_LONG_MASK 0x80
#define KXCNL_STAT_SYNCW_MASK 0x40
#define KXCNL_STAT_SYNC1_MASK 0x20
#define KXCNL_STAT_SYNC2_MASK 0x10
#define KXCNL_STAT_INT_SM1_MASK 0x8
#define KXCNL_STAT_INT_SM2_MASK 0x4
#define KXCNL_STAT_DOR_MASK 0x2
#define KXCNL_STAT_DRDY_MASK 0x1
#define KXCNL_CNTL1_PC_MASK 0x80
#define KXCNL_CNTL1_SC_MASK 0b01100000
#define KXCNL_CNTL1_ODR_MASK 0b00011100
#define KXCNL_CNTL1_DEBUG_MASK 0x2
#define KXCNL_CNTL1_IEN_MASK 0x1
#define KXCNL_CNTL2_HYST1_MASK 0b11100000
#define KXCNL_CNTL2_SM1_PIN_MASK 0x8
#define KXCNL_CNTL2_SM1_EN_MASK 0x1
#define KXCNL_CNTL3_HYST2_MASK 0b11100000
#define KXCNL_CNTL3_SM2_PIN_MASK 0x8
#define KXCNL_CNTL3_SM2_EN_MASK 0x1
#define KXCNL_CNTL4_DR_EN_MASK 0x80
#define KXCNL_CNTL4_IEA_MASK 0x40
#define KXCNL_CNTL4_IEL_MASK 0x20
#define KXCNL_CNTL4_INT2_EN_MASK 0x10
#define KXCNL_CNTL4_INT1_EN_MASK 0x8
#define KXCNL_CNTL4_VFILT_MASK 0x4
#define KXCNL_CNTL4_STP_MASK 0x2
#define KXCNL_CNTL4_STRT_MASK 0x1
#define KXCNL_SA1_P_X_MASK 0x80
#define KXCNL_SA1_N_X_MASK 0x40
#define KXCNL_SA1_P_Y_MASK 0x20
#define KXCNL_SA1_N_Y_MASK 0x10
#define KXCNL_SA1_P_Z_MASK 0x8
#define KXCNL_SA1_N_Z_MASK 0x4
#define KXCNL_SA1_P_V_MASK 0x2
#define KXCNL_SA1_N_V_MASK 0x1
#define KXCNL_MA1_P_X_MASK 0x80
#define KXCNL_MA1_N_X_MASK 0x40
#define KXCNL_MA1_P_Y_MASK 0x20
#define KXCNL_MA1_N_Y_MASK 0x10
#define KXCNL_MA1_P_Z_MASK 0x8
#define KXCNL_MA1_N_Z_MASK 0x4
#define KXCNL_MA1_P_V_MASK 0x2
#define KXCNL_MA1_N_V_MASK 0x1
#define KXCNL_SETT1_P_DET_MASK 0x80
#define KXCNL_SETT1_THR3_SA_MASK 0x40
#define KXCNL_SETT1_ABS_MASK 0x20
#define KXCNL_SETT1_THR3_MA_MASK 0x4
#define KXCNL_SETT1_R_TAM_MASK 0x2
#define KXCNL_SETT1_SITR_MASK 0x1
#define KXCNL_PPRP1_RESET_POINT_MASK 0b11110000
#define KXCNL_PPRP1_PROGRAM_COUNTER_MASK 0b11110000
#define KXCNL_OUTS1_P_X_MASK 0x80
#define KXCNL_OUTS1_N_X_MASK 0x40
#define KXCNL_OUTS1_P_Y_MASK 0x20
#define KXCNL_OUTS1_N_Y_MASK 0x10
#define KXCNL_OUTS1_P_Z_MASK 0x8
#define KXCNL_OUTS1_N_Z_MASK 0x4
#define KXCNL_OUTS1_P_V_MASK 0x2
#define KXCNL_OUTS1_N_V_MASK 0x1
#define KXCNL_SA2_P_X_MASK 0x80
#define KXCNL_SA2_N_X_MASK 0x40
#define KXCNL_SA2_P_Y_MASK 0x20
#define KXCNL_SA2_N_Y_MASK 0x10
#define KXCNL_SA2_P_Z_MASK 0x8
#define KXCNL_SA2_N_Z_MASK 0x4
#define KXCNL_SA2_P_V_MASK 0x2
#define KXCNL_SA2_N_V_MASK 0x1
#define KXCNL_MA2_P_X_MASK 0x80
#define KXCNL_MA2_N_X_MASK 0x40
#define KXCNL_MA2_P_Y_MASK 0x20
#define KXCNL_MA2_N_Y_MASK 0x10
#define KXCNL_MA2_P_Z_MASK 0x8
#define KXCNL_MA2_N_Z_MASK 0x4
#define KXCNL_MA2_P_V_MASK 0x2
#define KXCNL_MA2_N_V_MASK 0x1
#define KXCNL_SETT2_P_DET_MASK 0x80
#define KXCNL_SETT2_THR3_SA_MASK 0x40
#define KXCNL_SETT2_ABS_MASK 0x20
#define KXCNL_SETT2_RADI_MASK 0x10
#define KXCNL_SETT2_D_CS_MASK 0x8
#define KXCNL_SETT2_THR3_MA_MASK 0x4
#define KXCNL_SETT2_R_TAM_MASK 0x2
#define KXCNL_SETT2_SITR_MASK 0x1
#define KXCNL_OUTS2_P_X_MASK 0x80
#define KXCNL_OUTS2_N_X_MASK 0x40
#define KXCNL_OUTS2_P_Y_MASK 0x20
#define KXCNL_OUTS2_N_Y_MASK 0x10
#define KXCNL_OUTS2_P_Z_MASK 0x8
#define KXCNL_OUTS2_N_Z_MASK 0x4
#define KXCNL_OUTS2_P_V_MASK 0x2

/******* Sensor State Machine defines *******/
// Sensor Long counter values
#define KXCNL_LC_INIT_VALUE 0x7FFF
#define KXCNL_LC_SENSOR_INIT_VALUE1 0xFF
#define KXCNL_LC_SENSOR_INIT_VALUE2 0x7F

// Pedometer - State Machine 1
#define SM1_WALKING_TIM1 0x46
#define SM1_WALKING_TIM2 0x30
#define SM1_WALKING_TH1 0x04
#define SM1_RUNNING_TIM1 0x32
#define SM1_RUNNING_TIM2 0x17
#define SM1_RUNNING_TH1 0x08

/* Instructions set: Next/Reset conditions */
#define KXCNL_NOP			0x0
#define KXCNL_TI1			0x1
#define KXCNL_TI2			0x2
#define KXCNL_TI3			0x3
#define KXCNL_TI4			0x4
#define KXCNL_GNTH1			0x5
#define KXCNL_GNTH2			0x6
#define KXCNL_LNTH1			0x7
#define KXCNL_LNTH2			0x8
#define KXCNL_GTTH1			0x9
#define KXCNL_LLTH2			0xA
#define KXCNL_GRTH1			0xB
#define KXCNL_LRTH1			0xC
#define KXCNL_GRTH2			0xD
#define KXCNL_LRTH2			0xE
#define KXCNL_NZERO			0xF
/* Instruction set: Commands */
#define KXCNL_STOP			0x00
#define KXCNL_CONT			0x11
#define KXCNL_JMP			0x22
#define KXCNL_SRP			0x33
#define KXCNL_CRP			0x44
#define KXCNL_SETP			0x55
#define KXCNL_SETS1			0x66
#define KXCNL_STHR1			0x77
#define KXCNL_OUTC			0x88
#define KXCNL_OUTW			0x99
#define KXCNL_STHR2			0xAA
#define KXCNL_DEC			0xBB
#define KXCNL_SISW			0xCC
#define KXCNL_REL			0xDD
#define KXCNL_STHR3			0xEE
#define KXCNL_SSYNC			0xFF

u8 step_counter_code_values[] = {
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 0
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 1
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 2
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 3
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 4
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 5
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 6
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 7
        KXCNL_DEC,                      //# 8
        KXCNL_DEC,                      //# 9
        KXCNL_DEC,                      //# a
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# b
        KXCNL_NOP << 4 | KXCNL_TI2,     //# c
        KXCNL_JMP,                      //# d
        KXCNL_GNTH1 << 4 | KXCNL_LNTH1, //# e
        0xaa                            //# f
		};

// Run and walk Mode - State Machine 2
#define SM2_WALK_DETECT_TIM1 0x32
#define SM2_WALK_DETECT_TIM2 0x00
#define SM2_WALK_DETECT_TH1 0x10
#define SM2_RUN_DETECT_TIM1 0x17
#define SM2_RUN_DETECT_TIM2 0x16
#define SM2_RUN_DETECT_TH1 0x16
#define MTK_IRQ1_GPIO (66)				//Change according to hardware

u8 sm2_observe_running[] = {
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 0 
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 1
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 2
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 3
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 4
        KXCNL_NOP << 4 | KXCNL_TI2,     //# 5
        KXCNL_TI1 << 4 | KXCNL_GNTH1,   //# 6
        KXCNL_CONT                      //# 7
};

u8 sm2_observe_walking[] = {
        KXCNL_GNTH1 << 4 | KXCNL_TI1,
        KXCNL_NOP   << 4 | KXCNL_TI1,
        KXCNL_GNTH1 << 4 | KXCNL_TI1,
        KXCNL_CONT
};

#endif  /* __KXCNL_REGS_H__ */

//lichengmin end