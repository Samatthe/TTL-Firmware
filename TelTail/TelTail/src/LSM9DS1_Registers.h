/*
	Copyright 2019 Matthew Sauve	mattsauve@solidcircuits.net

	This file is part of the TTL firmware.

	The TTL firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The TTL firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef __LSM9DS1_Registers_H__
#define __LSM9DS1_Registers_H__


#ifdef HW_3v4
/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define ACT_THS				0x04
#define ACT_DUR				0x05
#define INT_GEN_CFG_XL		0x06
#define INT_GEN_THS_X_XL	0x07
#define INT_GEN_THS_Y_XL	0x08
#define INT_GEN_THS_Z_XL	0x09
#define INT_GEN_DUR_XL		0x0A
#define REFERENCE_G			0x0B
#define INT1_CTRL			0x0C
#define INT2_CTRL			0x0D
#define WHO_AM_I_XG			0x0F
#define CTRL_REG1_G			0x10
#define CTRL_REG2_G			0x11
#define CTRL_REG3_G			0x12
#define ORIENT_CFG_G		0x13
#define INT_GEN_SRC_G		0x14
#define OUT_TEMP_L			0x15
#define OUT_TEMP_H			0x16
#define STATUS_REG_0		0x17
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19
#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B
#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D
#define CTRL_REG4			0x1E
#define CTRL_REG5_XL		0x1F
#define CTRL_REG6_XL		0x20
#define CTRL_REG7_XL		0x21
#define CTRL_REG8			0x22
#define CTRL_REG9			0x23
#define CTRL_REG10			0x24
#define INT_GEN_SRC_XL		0x26
#define STATUS_REG_1		0x27
#define OUT_X_L_XL			0x28
#define OUT_X_H_XL			0x29
#define OUT_Y_L_XL			0x2A
#define OUT_Y_H_XL			0x2B
#define OUT_Z_L_XL			0x2C
#define OUT_Z_H_XL			0x2D
#define FIFO_CTRL			0x2E
#define FIFO_SRC			0x2F
#define INT_GEN_CFG_G		0x30
#define INT_GEN_THS_XH_G	0x31
#define INT_GEN_THS_XL_G	0x32
#define INT_GEN_THS_YH_G	0x33
#define INT_GEN_THS_YL_G	0x34
#define INT_GEN_THS_ZH_G	0x35
#define INT_GEN_THS_ZL_G	0x36
#define INT_GEN_DUR_G		0x37

///////////////////////////////
// LSM9DS1 Magneto Registers //
///////////////////////////////
#define OFFSET_X_REG_L_M	0x05
#define OFFSET_X_REG_H_M	0x06
#define OFFSET_Y_REG_L_M	0x07
#define OFFSET_Y_REG_H_M	0x08
#define OFFSET_Z_REG_L_M	0x09
#define OFFSET_Z_REG_H_M	0x0A
#define WHO_AM_I_M			0x0F
#define CTRL_REG1_M			0x20
#define CTRL_REG2_M			0x21
#define CTRL_REG3_M			0x22
#define CTRL_REG4_M			0x23
#define CTRL_REG5_M			0x24
#define STATUS_REG_M		0x27
#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29
#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B
#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D
#define INT_CFG_M			0x30
#define INT_SRC_M			0x30
#define INT_THS_L_M			0x32
#define INT_THS_H_M			0x33

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP		0x68
#define WHO_AM_I_M_RSP		0x3D

#endif


#if  defined(HW_4v0) || defined(HW_4v1)
/////////////////////////////////////////
// LSM9DS1 Accel/Gyro (XL/G) Registers //
/////////////////////////////////////////
#define FUNC_CFG_ACCESS		0x01
#define SENSOR_SYNC_TIME_FRAME	0x04
#define FIFO_CTRL1			0x06
#define FIFO_CTRL2			0x07
#define FIFO_CTRL3			0x08
#define FIFO_CTRL4			0x09
#define FIFO_CTRL5			0x0A
#define ORIENT_CFG_G		0x0B
#define INT1_CTRL			0x0D
#define INT2_CTRL			0x0E
#define WHO_AM_I_XG			0x0F
#define CTRL1_XL			0x10
#define CTRL2_G				0x11
#define CTRL3_C				0x12
#define CTRL4_C				0x13
#define CTRL5_C				0x14
#define CTRL6_C				0x15
#define CTRL7_G				0x16
#define CTRL8_XL			0x17
#define CTRL9_XL			0x18
#define CTRL10_C			0x19
#define MASTER_CONFIG		0x1A
#define WAKE_UP_SRC			0x1B
#define TAP_SRC				0x1C
#define D6D_SRC				0x1D
#define STATUS_REG			0x1E
#define OUT_TEMP_L			0x20
#define OUT_TEMP_H			0x21
#define OUTX_L_G			0x22
#define OUTX_H_G			0x23
#define OUTY_L_G			0x24
#define OUTY_H_G			0x25
#define OUTZ_L_G			0x26
#define OUTZ_H_G			0x27
#define OUTX_L_XL			0x28
#define OUTX_H_XL			0x29
#define OUTY_L_XL			0x2A
#define OUTY_H_XL			0x2B
#define OUTZ_L_XL			0x2C
#define OUTZ_H_XL			0x2D
#define SENSORHUB1_REG		0x2E
#define SENSORHUB2_REG		0x2F
#define SENSORHUB3_REG		0x30
#define SENSORHUB4_REG		0x31
#define SENSORHUB5_REG		0x32
#define SENSORHUB6_REG		0x33
#define SENSORHUB7_REG		0x34
#define SENSORHUB8_REG		0x35
#define SENSORHUB9_REG		0x36
#define SENSORHUB10_REG		0x37
#define SENSORHUB11_REG		0x38
#define SENSORHUB12_REG		0x39
#define FIFO_STATUS1		0x3A
#define FIFO_STATUS2		0x3B
#define FIFO_STATUS3		0x3C
#define FIFO_STATUS4		0x3D
#define FIFO_DATA_OUT_L		0x3E
#define FIFO_DATA_OUT_H		0x3F
#define TIMESTAMP0_REG		0x40
#define TIMESTAMP1_REG		0x41
#define TIMESTAMP2_REG		0x42
#define STEP_TIMESTAMP_L	0x49
#define STEP_TIMESTAMP_H	0x4A
#define STEP_COUNTER_L		0x4B
#define STEP_COUNTER_H		0x4C
#define SENSORHUB13_REG		0x4D
#define SENSORHUB14_REG		0x4E
#define SENSORHUB15_REG		0x4F
#define SENSORHUB16_REG		0x50
#define SENSORHUB17_REG		0x51
#define SENSORHUB18_REG		0x52
#define FUNC_SRC			0x53
#define TAP_CFG				0x58
#define TAP_THS_6D			0x59
#define INT_DUR2			0x5A
#define WAKE_UP_THS			0x5B
#define WAKE_UP_DUR			0x5C
#define FREE_FALL			0x5D
#define MD1_CFG				0x5E
#define MD2_CFG				0x5F
#define OUT_MAG_RAW_X_L		0x66
#define OUT_MAG_RAW_X_H		0x67
#define OUT_MAG_RAW_Y_L		0x68
#define OUT_MAG_RAW_Y_H		0x69
#define OUT_MAG_RAW_Z_L		0x6A
#define OUT_MAG_RAW_X_H		0x6B

////////////////////////////////
// LSM9DS1 WHO_AM_I Responses //
////////////////////////////////
#define WHO_AM_I_AG_RSP		0x69

#endif

#endif