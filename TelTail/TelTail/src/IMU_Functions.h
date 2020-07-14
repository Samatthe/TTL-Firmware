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

#ifndef IMU_FUNCTIONS_H
#define IMU_FUNCTIONS_H

#include <asf.h>
#include "LSM9DS1_Registers.h"
#include "LSM9DS1_Types.h"
#include "LED_Functions.h"

#ifdef HW_3v4
#define LSMXD_AG_ADDR 0x6B
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
#define LSMXD_AG_ADDR 0x6A
#endif
#define LSM9D_M_ADDR 0x1E

// i2c master globals
#define DATA_LENGTH 10
uint8_t write_buffer[DATA_LENGTH];
uint8_t read_buffer[DATA_LENGTH];
#define MASTER_TIMEOUT 250
#define MASTER_BAUD 500
struct i2c_master_module i2c_master_instance;

enum lsm9ds1_axis {
	X_AXIS,
	Y_AXIS,
	Z_AXIS,
	ALL_AXIS
};

enum orientation{
	ORIENT_UP = 1,
	ORIENT_DOWN,
	ORIENT_LEFT,
	ORIENT_RIGHT,
	ORIENT_REAR,
	ORIENT_FRONT
};

struct IMUSettings settings;
	
// We'll store the gyro, accel, and magnetometer readings in a series of
// public class variables. Each sensor gets three variables -- one for each
// axis. Call readGyro(), readAccel(), and readMag() first, before using
// these variables!
// These values are the RAW signed 16-bit readings from the sensors.
int16_t gx = 0, gy = 0, gz = 0; // x, y, and z axis readings of the gyroscope
int16_t ax = 0, ay = 0, az = 0; // x, y, and z axis readings of the accelerometer
int16_t mx = 0, my = 0, mz = 0; // x, y, and z axis readings of the magnetometer
int16_t temperature; // Chip temperature
float gBias[3], aBias[3], mBias[3];
int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

// Orientaion vars
uint8_t ORIENTATION[2] = {1,2}; // {connectors, power}
int16_t cgx = 0, cgy = 0, cgz = 0; // corrected x, y, and z axis readings of the gyroscope
int16_t cax = 0, cay = 0, caz = 0; // corrected x, y, and z axis readings of the accelerometer
int16_t cmx = 0, cmy = 0, cmz = 0; // corrected x, y, and z axis readings of the magnetometer
	
// begin() -- Initialize the gyro, accelerometer, and magnetometer.
// This will set up the scale and output rate of each sensor. The values set
// in the IMUSettings struct will take effect after calling this function.
uint16_t beginIMU(void);
	
void calibrate(bool autoCalc);
void calibrateMag(bool loadIn);
void magOffset(uint8_t axis, int16_t offset);
	
// accelAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t accelAvailable(void);
	
// gyroAvailable() -- Polls the gyroscope status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t gyroAvailable(void);
	
// gyroAvailable() -- Polls the temperature status register to check
// if new data is available.
// Output:	1 - New data available
//			0 - No new data available
uint8_t tempAvailable(void);
	
// magAvailable() -- Polls the accelerometer status register to check
// if new data is available.
// Input:
//	- axis can be either X_AXIS, Y_AXIS, Z_AXIS, to check for new data
//	  on one specific axis. Or ALL_AXIS (default) to check for new data
//	  on all axes.
// Output:	1 - New data available
//			0 - No new data available
uint8_t magAvailable(enum lsm9ds1_axis axis);
	
// readGyro() -- Read the gyroscope output registers.
// This function will read all six gyroscope output registers.
// The readings are stored in the class' gx, gy, and gz variables. Read
// those _after_ calling readGyro().
void readGyro(void);
	
// int16_t readGyro(axis) -- Read a specific axis of the gyroscope.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t readGyro_axis(enum lsm9ds1_axis axis);
	
// readAccel() -- Read the accelerometer output registers.
// This function will read all six accelerometer output registers.
// The readings are stored in the class' ax, ay, and az variables. Read
// those _after_ calling readAccel().
void readAccel(void);
	
// int16_t readAccel(axis) -- Read a specific axis of the accelerometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t readAccel_axis(enum lsm9ds1_axis axis);
	
// readMag() -- Read the magnetometer output registers.
// This function will read all six magnetometer output registers.
// The readings are stored in the class' mx, my, and mz variables. Read
// those _after_ calling readMag().
void readMag(void);
	
// int16_t readMag(axis) -- Read a specific axis of the magnetometer.
// [axis] can be any of X_AXIS, Y_AXIS, or Z_AXIS.
// Input:
//	- axis: can be either X_AXIS, Y_AXIS, or Z_AXIS.
// Output:
//	A 16-bit signed integer with sensor data on requested axis.
int16_t readMag_axis(enum lsm9ds1_axis axis);

// readTemp() -- Read the temperature output register.
// This function will read two temperature output registers.
// The combined readings are stored in the class' temperature variables. Read
// those _after_ calling readTemp().
void readTemp(void);
	
// calcGyro() -- Convert from RAW signed 16-bit value to degrees per second
// This function reads in a signed 16-bit value and returns the scaled
// DPS. This function relies on gScale and gRes being correct.
// Input:
//	- gyro = A signed 16-bit raw reading from the gyroscope.
float calcGyro(int16_t gyro);
	
// calcAccel() -- Convert from RAW signed 16-bit value to gravity (g's).
// This function reads in a signed 16-bit value and returns the scaled
// g's. This function relies on aScale and aRes being correct.
// Input:
//	- accel = A signed 16-bit raw reading from the accelerometer.
float calcAccel(int16_t accel);
	
// calcMag() -- Convert from RAW signed 16-bit value to Gauss (Gs)
// This function reads in a signed 16-bit value and returns the scaled
// Gs. This function relies on mScale and mRes being correct.
// Input:
//	- mag = A signed 16-bit raw reading from the magnetometer.
float calcMag(int16_t mag);
	
// setGyroScale() -- Set the full-scale range of the gyroscope.
// This function can be called to set the scale of the gyroscope to
// 245, 500, or 200 degrees per second.
// Input:
// 	- gScl = The desired gyroscope scale. Must be one of three possible
//		values from the gyro_scale.
void setGyroScale(uint16_t gScl);
	
// setAccelScale() -- Set the full-scale range of the accelerometer.
// This function can be called to set the scale of the accelerometer to
// 2, 4, 6, 8, or 16 g's.
// Input:
// 	- aScl = The desired accelerometer scale. Must be one of five possible
//		values from the accel_scale.
void setAccelScale(uint8_t aScl);
	
// setMagScale() -- Set the full-scale range of the magnetometer.
// This function can be called to set the scale of the magnetometer to
// 2, 4, 8, or 12 Gs.
// Input:
// 	- mScl = The desired magnetometer scale. Must be one of four possible
//		values from the mag_scale.
void setMagScale(uint8_t mScl);
	
// setGyroODR() -- Set the output data rate and bandwidth of the gyroscope
// Input:
//	- gRate = The desired output rate and cutoff frequency of the gyro.
void setGyroODR(uint8_t gRate);
	
// setAccelODR() -- Set the output data rate of the accelerometer
// Input:
//	- aRate = The desired output rate of the accel.
void setAccelODR(uint8_t aRate);
	
// setMagODR() -- Set the output data rate of the magnetometer
// Input:
//	- mRate = The desired output rate of the mag.
void setMagODR(uint8_t mRate);
	
// configInactivity() -- Configure inactivity interrupt parameters
// Input:
//	- duration = Inactivity duration - actual value depends on gyro ODR
//	- threshold = Activity Threshold
//	- sleepOn = Gyroscope operating mode during inactivity.
//	  true: gyroscope in sleep mode
//	  false: gyroscope in power-down
void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn);
	
// configAccelInt() -- Configure Accelerometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_XL, ZLIE_XL, YHIE_XL, YLIE_XL, XHIE_XL, XLIE_XL
//	- andInterrupts = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
void configAccelInt(uint8_t generator, bool andInterrupts);
	
// configAccelThs() -- Configure the threshold of an accelereomter axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-255.
//	  Multiply by 128 to get the actual raw accel value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void configAccelThs(uint8_t threshold, enum lsm9ds1_axis axis, uint8_t duration, bool wait);
	
// configGyroInt() -- Configure Gyroscope Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZHIE_G, ZLIE_G, YHIE_G, YLIE_G, XHIE_G, XLIE_G
//	- aoi = AND/OR combination of interrupt events
//	  true: AND combination
//	  false: OR combination
//	- latch: latch gyroscope interrupt request.
void configGyroInt(uint8_t generator, bool aoi, bool latch);
	
// configGyroThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw gyroscope value.
//	- axis = Axis to be configured. Either X_AXIS, Y_AXIS, or Z_AXIS
//	- duration = Duration value must be above or below threshold to trigger interrupt
//	- wait = Wait function on duration counter
//	  true: Wait for duration samples before exiting interrupt
//	  false: Wait function off
void configGyroThs(int16_t threshold, enum lsm9ds1_axis axis, uint8_t duration, bool wait);
	
// configInt() -- Configure INT1 or INT2 (Gyro and Accel Interrupts only)
// Input:
//	- interrupt = Select INT1 or INT2
//	  Possible values: XG_INT1 or XG_INT2
//	- generator = Or'd combination of interrupt generators.
//	  Possible values: INT_DRDY_XL, INT_DRDY_G, INT1_BOOT (INT1 only), INT2_DRDY_TEMP (INT2 only)
//	  INT_FTH, INT_OVR, INT_FSS5, INT_IG_XL (INT1 only), INT1_IG_G (INT1 only), INT2_INACT (INT2 only)
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- pushPull =  Push-pull or open drain interrupt configuration
//	  Can be either INT_PUSH_PULL or INT_OPEN_DRAIN
void configInt(enum interrupt_select interupt, uint8_t generator,
enum h_lactive activeLow, enum pp_od pushPull);
	
// configMagInt() -- Configure Magnetometer Interrupt Generator
// Input:
//	- generator = Interrupt axis/high-low events
//	  Any OR'd combination of ZIEN, YIEN, XIEN
//	- activeLow = Interrupt active configuration
//	  Can be either INT_ACTIVE_HIGH or INT_ACTIVE_LOW
//	- latch: latch gyroscope interrupt request.
void configMagInt(uint8_t generator, enum h_lactive activeLow, bool latch);
	
// configMagThs() -- Configure the threshold of a gyroscope axis
// Input:
//	- threshold = Interrupt threshold. Possible values: 0-0x7FF.
//	  Value is equivalent to raw magnetometer value.
void configMagThs(uint16_t threshold);
	
// getGyroIntSrc() -- Get contents of Gyroscope interrupt source register
uint8_t getGyroIntSrc(void);
	
// getGyroIntSrc() -- Get contents of accelerometer interrupt source register
uint8_t getAccelIntSrc(void);
	
// getGyroIntSrc() -- Get contents of magnetometer interrupt source register
uint8_t getMagIntSrc(void);
	
// getGyroIntSrc() -- Get status of inactivity interrupt
uint8_t getInactivity(void);
	
// sleepGyro() -- Sleep or wake the gyroscope
// Input:
//	- enable: True = sleep gyro. False = wake gyro.
void sleepGyro(bool enable);
	
// enableFIFO() - Enable or disable the FIFO
// Input:
//	- enable: true = enable, false = disable.
void enableFIFO(bool enable);
	
// setFIFO() - Configure FIFO mode and Threshold
// Input:
//	- fifoMode: Set FIFO mode to off, FIFO (stop when full), continuous, bypass
//	  Possible inputs: FIFO_OFF, FIFO_THS, FIFO_CONT_TRIGGER, FIFO_OFF_TRIGGER, FIFO_CONT
//	- fifoThs: FIFO threshold level setting
//	  Any value from 0-0x1F is acceptable.
void setFIFO(enum fifoMode_type fifoMode, uint16_t fifoThs);
	
// getFIFOSamples() - Get number of FIFO samples
uint8_t getFIFOSamples(void);
	
// x_mAddress and gAddress store the I2C address or SPI chip select pin
// for each sensor.
uint8_t _mAddress, _xgAddress;
	
// gRes, aRes, and mRes store the current resolution for each sensor.
// Units of these values would be DPS (or g's or Gs's) per ADC tick.
// This value is calculated as (sensor scale) / (2^15).
float gRes, aRes, mRes;
	
// _autoCalc keeps track of whether we're automatically subtracting off
// accelerometer and gyroscope bias calculated in calibrate().
bool _autoCalc;
	
// init() -- Sets up gyro, accel, and mag settings to default.
// - interface - Sets the interface mode (IMU_MODE_I2C or IMU_MODE_SPI)
// - xgAddr - Sets either the I2C address of the accel/gyro or SPI chip
//   select pin connected to the CS_XG pin.
// - mAddr - Sets either the I2C address of the magnetometer or SPI chip
//   select pin connected to the CS_M pin.
void initIMU(void);
	
// initGyro() -- Sets up the gyroscope to begin reading.
// This function steps through all five gyroscope control registers.
// Upon exit, the following parameters will be set:
//	- CTRL_REG1_G = 0x0F: Normal operation mode, all axes enabled.
//		95 Hz ODR, 12.5 Hz cutoff frequency.
//	- CTRL_REG2_G = 0x00: HPF set to normal mode, cutoff frequency
//		set to 7.2 Hz (depends on ODR).
//	- CTRL_REG3_G = 0x88: Interrupt enabled on INT_G (set to push-pull and
//		active high). Data-ready output enabled on DRDY_G.
//	- CTRL_REG4_G = 0x00: Continuous update mode. Data LSB stored in lower
//		address. Scale set to 245 DPS. SPI mode set to 4-wire.
//	- CTRL_REG5_G = 0x00: FIFO disabled. HPF disabled.
void initGyro(void);
	
// initAccel() -- Sets up the accelerometer to begin reading.
// This function steps through all accelerometer related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG0_XM = 0x00: FIFO disabled. HPF bypassed. Normal mode.
//	- CTRL_REG1_XM = 0x57: 100 Hz data rate. Continuous update.
//		all axes enabled.
//	- CTRL_REG2_XM = 0x00:  2g scale. 773 Hz anti-alias filter BW.
//	- CTRL_REG3_XM = 0x04: Accel data ready signal on INT1_XM pin.
void initAccel(void);
	
// initMag() -- Sets up the magnetometer to begin reading.
// This function steps through all magnetometer-related control registers.
// Upon exit these registers will be set as:
//	- CTRL_REG4_XM = 0x04: Mag data ready signal on INT2_XM pin.
//	- CTRL_REG5_XM = 0x14: 100 Hz update rate. Low resolution. Interrupt
//		requests don't latch. Temperature sensor disabled.
//	- CTRL_REG6_XM = 0x00:  2 Gs scale.
//	- CTRL_REG7_XM = 0x00: Continuous conversion mode. Normal HPF mode.
//	- INT_CTRL_REG_M = 0x09: Interrupt active-high. Enable interrupts.
void initMag(void);
	
// gReadByte() -- Reads a byte from a specified gyroscope register.
// Input:
// 	- subAddress = Register to be read from.
// Output:
// 	- An 8-bit value read from the requested address.
uint8_t mReadByte(uint8_t subAddress);
	
// gReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the gyroscope.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
// gWriteByte() -- Write a byte to a register in the gyroscope.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void mWriteByte(uint8_t subAddress, uint8_t data);
	
// xmReadByte() -- Read a byte from a register in the accel/mag sensor
// Input:
//	- subAddress = Register to be read from.
// Output:
//	- An 8-bit value read from the requested register.
uint8_t xgReadByte(uint8_t subAddress);
	
// xmReadBytes() -- Reads a number of bytes -- beginning at an address
// and incrementing from there -- from the accelerometer/magnetometer.
// Input:
// 	- subAddress = Register to be read from.
// 	- * dest = A pointer to an array of uint8_t's. Values read will be
//		stored in here on return.
//	- count = The number of bytes to be read.
// Output: No value is returned, but the `dest` array will store
// 	the data read upon exit.
void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count);
	
// xmWriteByte() -- Write a byte to a register in the accel/mag sensor.
// Input:
//	- subAddress = Register to be written to.
//	- data = data to be written to the register.
void xgWriteByte(uint8_t subAddress, uint8_t data);
	
// calcgRes() -- Calculate the resolution of the gyroscope.
// This function will set the value of the gRes variable. gScale must
// be set prior to calling this function.
void calcgRes(void);
	
// calcmRes() -- Calculate the resolution of the magnetometer.
// This function will set the value of the mRes variable. mScale must
// be set prior to calling this function.
void calcmRes(void);
	
// calcaRes() -- Calculate the resolution of the accelerometer.
// This function will set the value of the aRes variable. aScale must
// be set prior to calling this function.
void calcaRes(void);
	
//////////////////////
// Helper Functions //
//////////////////////
void constrainScales(void);
	
///////////////////
// I2C Functions //
///////////////////
// Configure SERCOM channel 3 as I2C master for IMU communication
void initI2C(void);
	
// I2CwriteByte() -- Write a byte out of I2C to a register in the device
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be written to.
//	- data = Byte to be written to the register.
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
	
// I2CreadByte() -- Read a single byte from a register over I2C.
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to be read from.
// Output:
//	- The byte read from the requested address.
uint8_t readByte(uint8_t address, uint8_t subAddress);
	
// I2CreadBytes() -- Read a series of bytes, starting at a register via SPI
// Input:
//	- address = The 7-bit I2C address of the slave device.
//	- subAddress = The register to begin reading.
// 	- * dest = Pointer to an array where we'll store the readings.
//	- count = Number of registers to be read.
// Output: No value is returned by the function, but the registers read are
// 		all stored in the *dest array given.
uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count);


// Correct the axis values depending on the configured module orientation
void CorrectIMUvalues(uint8_t connector_orient, uint8_t power_orient);
void calculate_heading(void);
void initKalman(float meas, float est, float _q);
float updateKalman(float meas, int kalmanIndex);
void update_kalman_limits(void);
int16_t averageAZ(void);
int16_t averageAY(void);
int16_t averageAX(void);
float averageGZ(void);
float averageGY(void);
float averageGX(void);


/////////////////////////////////////////////////////////////////////////////////////////////////



float magSensitivity[4] = {0.00014, 0.00029, 0.00043, 0.00058};

void initIMU()
{
	settings.device.agAddress = LSMXD_AG_ADDR;
	settings.device.mAddress = LSM9D_M_ADDR;

	settings.gyro.enabled = true;
	settings.gyro.enableX = true;
	settings.gyro.enableY = true;
	settings.gyro.enableZ = true;
	settings.gyro.lowPowerEnable = false;
	settings.gyro.HPFEnable = false;
	settings.gyro.flipX = false;
	settings.gyro.flipY = false; // LSM6D inverted from LSM9D, corrected in init func
	settings.gyro.flipZ = false;
	settings.gyro.latchInterrupt = true;
	
	settings.accel.enabled = true;
	settings.accel.enableX = true;
	settings.accel.enableY = true;
	settings.accel.enableZ = true;
	settings.accel.highResEnable = false;

	settings.temp.enabled = true;

#ifdef HW_3v4
	// gyro scale can be 245, 500, or 2000
	settings.gyro.scale = 245;//245
	// gyro sample rate: value between 1-6
	// 1 = 14.9    4 = 238
	// 2 = 59.5    5 = 476
	// 3 = 119     6 = 952
	settings.gyro.sampleRate = 6;
	// gyro cutoff frequency: value between 0-3
	// Actual value of cutoff frequency depends
	// on sample rate.
	settings.gyro.bandwidth = 0;
	// Gyro HPF cutoff frequency: value between 0-9
	// Actual value depends on sample rate. Only applies
	// if gyroHPFEnable is true.
	settings.gyro.HPFCutoff = 0;
	settings.gyro.orientation = 0;

	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 16;//8
	// accel sample rate can be 1-6
	// 1 = 10 Hz    4 = 238 Hz
	// 2 = 50 Hz    5 = 476 Hz
	// 3 = 119 Hz   6 = 952 Hz
	settings.accel.sampleRate = 6;
	// Accel cutoff freqeuncy can be any value between -1 - 3. 
	// -1 = bandwidth determined by sample rate
	// 0 = 408 Hz   2 = 105 Hz
	// 1 = 211 Hz   3 = 50 Hz
	settings.accel.bandwidth = -1;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/50    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 0;

	settings.mag.enabled = true;
	// mag scale can be 4, 8, 12, or 16
	settings.mag.scale = 8;
	// mag data rate can be 0-7
	// 0 = 0.625 Hz  4 = 10 Hz
	// 1 = 1.25 Hz   5 = 20 Hz
	// 2 = 2.5 Hz    6 = 40 Hz
	// 3 = 5 Hz      7 = 80 Hz
	settings.mag.sampleRate = 7;
	settings.mag.tempCompensationEnable = true;
	// magPerformance can be any value between 0-3
	// 0 = Low power mode      2 = high performance
	// 1 = medium performance  3 = ultra-high performance
	settings.mag.XYPerformance = 1;
	settings.mag.ZPerformance = 1;
	settings.mag.lowPowerEnable = false;
	// magOperatingMode can be 0-2
	// 0 = continuous conversion
	// 1 = single-conversion
	// 2 = power down
	settings.mag.operatingMode = 0;
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	// gyro scale can be 250, 500, 1000, or 2000
	settings.gyro.scale = 250;//245
	// gyro sample rate: value between 1-8
	// 1 = 12.5		4 = 104		7 = 833
	// 2 = 26		5 = 208		8 = 1660
	// 3 = 52		6 = 416
	settings.gyro.sampleRate = 8;
	// Gyro HPF cutoff frequency: value between 0-3
	// 0 = 0.0081	2 = 2.07
	// 1 = 0.0324	3 = 16.32
	// Only applies if gyroHPFEnable is true.
	settings.gyro.HPFCutoff = 0;
	settings.gyro.orientation = 0;

	// accel scale can be 2, 4, 8, or 16
	settings.accel.scale = 16;//8
	// accel sample rate can be 1-10
	// 1 = 12.5 Hz		4 = 104 Hz	7 = 833Hz	10 = 6.66kHz
	// 2 = 26 Hz		5 = 208 Hz	8 = 1.66kHz	
	// 3 = 152 Hz		6 = 412 Hz	9 = 3.33kHz	
	settings.accel.sampleRate = 8;
	// Accel cutoff freqeuncy can be any value between 0 - 3.
	// -1 = bandwidth determined by sample rate
	// 0 = 400 Hz   2 = 100 Hz
	// 1 = 200 Hz   3 = 50 Hz
	settings.accel.bandwidth = 0;
	// accelHighResBandwidth can be any value between 0-3
	// LP cutoff is set to a factor of sample rate
	// 0 = ODR/4    2 = ODR/9
	// 1 = ODR/100   3 = ODR/400
	settings.accel.highResBandwidth = 0;
#endif

	for (int i=0; i<3; i++)
	{
		gBias[i] = 0;
		aBias[i] = 0;
		mBias[i] = 0;
		gBiasRaw[i] = 0;
		aBiasRaw[i] = 0;
		mBiasRaw[i] = 0;
	}
	_autoCalc = false;
}


uint16_t beginIMU()
{
	//! Todo: don't use _xgAddress or _mAddress, duplicating memory
	_xgAddress = settings.device.agAddress;
	_mAddress = settings.device.mAddress;
	
	constrainScales();
	// Once we have the scale values, we can calculate the resolution
	// of each sensor. That's what these functions are for. One for each sensor
	calcgRes(); // Calculate DPS / ADC tick, stored in gRes variable
	calcmRes(); // Calculate Gs / ADC tick, stored in mRes variable
	calcaRes(); // Calculate g / ADC tick, stored in aRes variable
	
	initI2C();	// Initialize I2C
		
	// To verify communication, we can read from the WHO_AM_I register of
	// each device. Store those in a variable so we can return them.
	uint8_t xgTest = xgReadByte(WHO_AM_I_XG);	// Read the accel/mag WHO_AM_I
	
#ifdef HW_3v4
	uint8_t mTest = mReadByte(WHO_AM_I_M);		// Read the gyro WHO_AM_I
	uint16_t whoAmICombined = (xgTest << 8) | mTest;
	if (whoAmICombined != ((WHO_AM_I_AG_RSP << 8) | WHO_AM_I_M_RSP))
		return 0;
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	uint16_t whoAmICombined = xgTest;
	if (xgTest != 0x69 && xgTest != 0x6A && xgTest != 0x6B && xgTest != 0x6C)//WHO_AM_I_AG_RSP) // A work around to protect for the use of al LSM6DS parts
		return 0;
#endif
	// Gyro initialization stuff:
	initGyro();	// This will "turn on" the gyro. Setting up interrupts, etc.
	
	// Accelerometer initialization stuff:
	initAccel(); // "Turn on" all axes of the accel. Set up interrupts, etc.
	
	// Magnetometer initialization stuff:
#ifdef HW_3v4
	initMag(); // "Turn on" all axes of the mag. Set up interrupts, etc.
#endif

	// Once everything is initialized, return the WHO_AM_I registers we read:
	return whoAmICombined;
}

void initGyro()
{
#ifdef HW_3v4
	uint8_t tempRegValue = 0;

	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection

	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate & 0x07) << 5;
	}
	switch (settings.gyro.scale)
	{
		case 500:
		tempRegValue |= (0x1 << 3);
		break;
		case 2000:
		tempRegValue |= (0x3 << 3);
		break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	tempRegValue |= (settings.gyro.bandwidth & 0x3);
	xgWriteByte(CTRL_REG1_G, tempRegValue);

	// CTRL_REG2_G (Default value: 0x00)
	// [0][0][0][0][INT_SEL1][INT_SEL0][OUT_SEL1][OUT_SEL0]
	// INT_SEL[1:0] - INT selection configuration
	// OUT_SEL[1:0] - Out selection configuration
	xgWriteByte(CTRL_REG2_G, 0x00);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | (settings.gyro.HPFCutoff & 0x0F);
	}
	xgWriteByte(CTRL_REG3_G, tempRegValue);

	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	if (settings.gyro.latchInterrupt) tempRegValue |= (1<<1);
	xgWriteByte(CTRL_REG4, tempRegValue);

	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (settings.gyro.flipY) tempRegValue |= (1<<4);
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_G (Default value: 0x00)
	// [ODR_G2][ODR_G1][ODR_G0][FS_G1][FS_G0][0][BW_G1][BW_G0]
	// ODR_G[2:0] - Output data rate selection
	// FS_G[1:0] - Gyroscope full-scale selection
	// BW_G[1:0] - Gyroscope bandwidth selection
	
	// To disable gyro, set sample rate bits to 0. We'll only set sample
	// rate if the gyro is enabled.
	if (settings.gyro.enabled)
	{
		tempRegValue = (settings.gyro.sampleRate & 0x0F) << 4;
	}
	switch (settings.gyro.scale)
	{
		case 500:
			tempRegValue |= (0x1 << 2);
			break;
		case 1000:
			tempRegValue |= (0x2 << 2);
			break;
		case 2000:
			tempRegValue |= (0x3 << 2);
			break;
		// Otherwise we'll set it to 245 dps (0x0 << 4)
	}
	xgWriteByte(CTRL2_G, tempRegValue);

	// CTRL_REG3_G (Default value: 0x00)
	// [LP_mode][HP_EN][0][0][HPCF3_G][HPCF2_G][HPCF1_G][HPCF0_G]
	// LP_mode - Low-power mode enable (0: disabled, 1: enabled)
	// HP_EN - HPF enable (0:disabled, 1: enabled)
	// HPCF_G[3:0] - HPF cutoff frequency
	tempRegValue = settings.gyro.lowPowerEnable ? (1<<7) : 0;
	if (settings.gyro.HPFEnable)
	{
		tempRegValue |= (1<<6) | ((settings.gyro.HPFCutoff & 0x03)<<4);
	}
	xgWriteByte(CTRL7_G, tempRegValue);
	
	// CTRL_REG4 (Default value: 0x38)
	// [0][0][Zen_G][Yen_G][Xen_G][0][LIR_XL1][4D_XL1]
	// Zen_G - Z-axis output enable (0:disable, 1:enable)
	// Yen_G - Y-axis output enable (0:disable, 1:enable)
	// Xen_G - X-axis output enable (0:disable, 1:enable)
	// LIR_XL1 - Latched interrupt (0:not latched, 1:latched)
	// 4D_XL1 - 4D option on interrupt (0:6D used, 1:4D used)
	tempRegValue = 0;
	if (settings.gyro.enableZ) tempRegValue |= (1<<5);
	if (settings.gyro.enableY) tempRegValue |= (1<<4);
	if (settings.gyro.enableX) tempRegValue |= (1<<3);
	xgWriteByte(CTRL10_C, tempRegValue);
	
	// ORIENT_CFG_G (Default value: 0x00)
	// [0][0][SignX_G][SignY_G][SignZ_G][Orient_2][Orient_1][Orient_0]
	// SignX_G - Pitch axis (X) angular rate sign (0: positive, 1: negative)
	// Orient [2:0] - Directional user orientation selection
	tempRegValue = 0;
	if (settings.gyro.flipX) tempRegValue |= (1<<5);
	if (!settings.gyro.flipY) tempRegValue |= (1<<4); // Flip to align with LSM9D
	if (settings.gyro.flipZ) tempRegValue |= (1<<3);
	xgWriteByte(ORIENT_CFG_G, tempRegValue);
#endif
}

void initAccel()
{
#ifdef HW_3v4
	uint8_t tempRegValue = 0;
	
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);
	
	xgWriteByte(CTRL_REG5_XL, tempRegValue);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_SCAL_ODR][BW_XL1][BW_XL0]
	// ODR_XL[2:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_SCAL_ODR - Bandwidth selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate & 0x07) << 5;
	}
	switch (settings.accel.scale)
	{
		case 4:
		tempRegValue |= (0x2 << 3);
		break;
		case 8:
		tempRegValue |= (0x3 << 3);
		break;
		case 16:
		tempRegValue |= (0x1 << 3);
		break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	if (settings.accel.bandwidth >= 0)
	{
		tempRegValue |= (1<<2); // Set BW_SCAL_ODR
		tempRegValue |= (settings.accel.bandwidth & 0x03);
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL_REG7_XL, tempRegValue);
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	uint8_t tempRegValue = 0;
	
	//	CTRL_REG5_XL (0x1F) (Default value: 0x38)
	//	[DEC_1][DEC_0][Zen_XL][Yen_XL][Zen_XL][0][0][0]
	//	DEC[0:1] - Decimation of accel data on OUT REG and FIFO.
	//		00: None, 01: 2 samples, 10: 4 samples 11: 8 samples
	//	Zen_XL - Z-axis output enabled
	//	Yen_XL - Y-axis output enabled
	//	Xen_XL - X-axis output enabled
	if (settings.accel.enableZ) tempRegValue |= (1<<5);
	if (settings.accel.enableY) tempRegValue |= (1<<4);
	if (settings.accel.enableX) tempRegValue |= (1<<3);
	xgWriteByte(CTRL9_XL, tempRegValue);
	
	// CTRL_REG6_XL (0x20) (Default value: 0x00)
	// [ODR_XL3][ODR_XL2][ODR_XL1][ODR_XL0][FS1_XL][FS0_XL][BW_XL1][BW_XL0]
	// ODR_XL[3:0] - Output data rate & power mode selection
	// FS_XL[1:0] - Full-scale selection
	// BW_XL[1:0] - Anti-aliasing filter bandwidth selection
	tempRegValue = 0;
	// To disable the accel, set the sampleRate bits to 0.
	if (settings.accel.enabled)
	{
		tempRegValue |= (settings.accel.sampleRate & 0x0F) << 4;
	}
	switch (settings.accel.scale)
	{
		case 4:
			tempRegValue |= (0x2 << 2);
			break;
		case 8:
			tempRegValue |= (0x3 << 2);
			break;
		case 16:
			tempRegValue |= (0x1 << 2);
			break;
		// Otherwise it'll be set to 2g (0x0 << 3)
	}
	tempRegValue |= (settings.accel.bandwidth & 0x03);
	xgWriteByte(CTRL1_XL, tempRegValue);
	
	// CTRL_REG7_XL (0x21) (Default value: 0x00)
	// [HR][DCF1][DCF0][0][0][FDS][0][HPIS1]
	// HR - High resolution mode (0: disable, 1: enable)
	// DCF[1:0] - Digital filter cutoff frequency
	// FDS - Filtered data selection
	// HPIS1 - HPF enabled for interrupt function
	tempRegValue = 0;
	if (settings.accel.highResEnable)
	{
		tempRegValue |= (1<<7); // Set HR bit
		tempRegValue |= (settings.accel.highResBandwidth & 0x3) << 5;
	}
	xgWriteByte(CTRL8_XL, tempRegValue);
#endif
}

// This is a function that uses the FIFO to accumulate sample of accelerometer and gyro data, average
// them, scales them to  gs and deg/s, respectively, and then passes the biases to the main sketch
// for subtraction from all subsequent data. There are no gyro and accelerometer bias registers to store
// the data as there are in the ADXL345, a precursor to the LSM9DS0, or the MPU-9150, so we have to
// subtract the biases ourselves. This results in a more accurate measurement in general and can
// remove errors due to imprecise or varying initial placement. Calibration of sensor data in this manner
// is good practice.
void calibrate(bool autoCalc)
{
	_autoCalc = false; // Workaround so that calibrate doesnt include the current offset
	//uint8_t data[6] = {0, 0, 0, 0, 0, 0};
	uint16_t samples = 0;
	int ii;
	int32_t aBiasRawTemp[3] = {0, 0, 0};
	int32_t gBiasRawTemp[3] = {0, 0, 0};
	
	// Turn on FIFO and set threshold to 32 samples
#ifdef HW_3v4
	enableFIFO(true);
	setFIFO(FIFO_THS, 0x1F);
	while (samples < 0x1F)
	{
		samples = (xgReadByte(FIFO_SRC) & 0x3F); // Read number of stored samples
	}
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	setFIFO(FIFO_THS, 0x003D);
	while (samples < 0x003C)
	{
		samples = (((xgReadByte(FIFO_STATUS2) & 0x0F)<<8)|(xgReadByte(FIFO_STATUS1))); // Read number of stored samples
	}
#endif
	for(ii = 0; ii < samples ; ii++) 
	{	// Read the gyro data stored in the FIFO
		readGyro();
		gBiasRawTemp[0] += gx;
		gBiasRawTemp[1] += gy;
		gBiasRawTemp[2] += gz;
		readAccel();
		aBiasRawTemp[0] += ax;
		aBiasRawTemp[1] += ay;
		aBiasRawTemp[2] += az - (int16_t)(1./aRes); // Assumes sensor facing up!
	}  
	for (ii = 0; ii < 3; ii++)
	{
		gBiasRaw[ii] = gBiasRawTemp[ii] / samples;
		gBias[ii] = calcGyro(gBiasRaw[ii]);
		aBiasRaw[ii] = aBiasRawTemp[ii] / samples;
		aBias[ii] = calcAccel(aBiasRaw[ii]);
	}

#ifdef HW_3v4
	enableFIFO(false);
#endif
	setFIFO(FIFO_OFF, 0x00);
	
	if (autoCalc) _autoCalc = true;
}

void calibrateMag(bool loadIn)
{
	int i, j;
	int16_t magMin[3] = {0, 0, 0};
	int16_t magMax[3] = {0, 0, 0}; // The road warrior
	
	for (i=0; i<128; i++)
	{
		while (!magAvailable(ALL_AXIS))
			;
		readMag();
		int16_t magTemp[3] = {0, 0, 0};
		magTemp[0] = mx;		
		magTemp[1] = my;
		magTemp[2] = mz;
		for (j = 0; j < 3; j++)
		{
			if (magTemp[j] > magMax[j]) magMax[j] = magTemp[j];
			if (magTemp[j] < magMin[j]) magMin[j] = magTemp[j];
		}
	}
	for (j = 0; j < 3; j++)
	{
		mBiasRaw[j] = (magMax[j] + magMin[j]) / 2;
		mBias[j] = calcMag(mBiasRaw[j]);
		if (loadIn)
			magOffset(j, mBiasRaw[j]);
	}
	
}
void magOffset(uint8_t axis, int16_t offset)
{
#ifdef HW_3v4
	if (axis > 2)
		return;
	uint8_t msb, lsb;
	msb = (offset & 0xFF00) >> 8;
	lsb = offset & 0x00FF;
	mWriteByte(OFFSET_X_REG_L_M + (2 * axis), lsb);
	mWriteByte(OFFSET_X_REG_H_M + (2 * axis), msb);
	#endif
}

void initMag()
{
#ifdef HW_3v4
	uint8_t tempRegValue = 0;
	
	// CTRL_REG1_M (Default value: 0x10)
	// [TEMP_COMP][OM1][OM0][DO2][DO1][DO0][0][ST]
	// TEMP_COMP - Temperature compensation
	// OM[1:0] - X & Y axes op mode selection
	//	00:low-power, 01:medium performance
	//	10: high performance, 11:ultra-high performance
	// DO[2:0] - Output data rate selection
	// ST - Self-test enable
	if (settings.mag.tempCompensationEnable) tempRegValue |= (1<<7);
	tempRegValue |= (settings.mag.XYPerformance & 0x3) << 5;
	tempRegValue |= (settings.mag.sampleRate & 0x7) << 2;
	mWriteByte(CTRL_REG1_M, tempRegValue);
	
	// CTRL_REG2_M (Default value 0x00)
	// [0][FS1][FS0][0][REBOOT][SOFT_RST][0][0]
	// FS[1:0] - Full-scale configuration
	// REBOOT - Reboot memory content (0:normal, 1:reboot)
	// SOFT_RST - Reset config and user registers (0:default, 1:reset)
	tempRegValue = 0;
	switch (settings.mag.scale)
	{
	case 8:
		tempRegValue |= (0x1 << 5);
		break;
	case 12:
		tempRegValue |= (0x2 << 5);
		break;
	case 16:
		tempRegValue |= (0x3 << 5);
		break;
	// Otherwise we'll default to 4 gauss (00)
	}
	mWriteByte(CTRL_REG2_M, tempRegValue); // +/-4Gauss
	
	// CTRL_REG3_M (Default value: 0x03)
	// [I2C_DISABLE][0][LP][0][0][SIM][MD1][MD0]
	// I2C_DISABLE - Disable I2C interace (0:enable, 1:disable)
	// LP - Low-power mode cofiguration (1:enable)
	// SIM - SPI mode selection (0:write-only, 1:read/write enable)
	// MD[1:0] - Operating mode
	//	00:continuous conversion, 01:single-conversion,
	//  10,11: Power-down
	tempRegValue = 0;
	if (settings.mag.lowPowerEnable) tempRegValue |= (1<<5);
	tempRegValue |= (settings.mag.operatingMode & 0x3);
	mWriteByte(CTRL_REG3_M, tempRegValue); // Continuous conversion mode
	
	// CTRL_REG4_M (Default value: 0x00)
	// [0][0][0][0][OMZ1][OMZ0][BLE][0]
	// OMZ[1:0] - Z-axis operative mode selection
	//	00:low-power mode, 01:medium performance
	//	10:high performance, 10:ultra-high performance
	// BLE - Big/little endian data
	tempRegValue = 0;
	tempRegValue = (settings.mag.ZPerformance & 0x3) << 2;
	mWriteByte(CTRL_REG4_M, tempRegValue);
	
	// CTRL_REG5_M (Default value: 0x00)
	// [0][BDU][0][0][0][0][0][0]
	// BDU - Block data update for magnetic data
	//	0:continuous, 1:not updated until MSB/LSB are read
	tempRegValue = 0;
	mWriteByte(CTRL_REG5_M, tempRegValue);
#endif
}

uint8_t accelAvailable()
{
#ifdef HW_3v4
	uint8_t status = xgReadByte(STATUS_REG_1);

	
	return (status & (1<<0));
#endif
}

uint8_t gyroAvailable()
{
#ifdef HW_3v4
	uint8_t status = xgReadByte(STATUS_REG_1);

	
	return ((status & (1<<1)) >> 1);
#endif
}

uint8_t tempAvailable()
{
#ifdef HW_3v4
	uint8_t status = xgReadByte(STATUS_REG_1);
	
	return ((status & (1<<2)) >> 2);
#endif
}

uint8_t magAvailable(enum lsm9ds1_axis axis)
{
#ifdef HW_3v4
	uint8_t status = 0;

	status = mReadByte(STATUS_REG_M);
	
	return ((status & (1<<axis)) >> axis);
#endif
}

void readAccel()
{
	uint8_t temp[6]; // We'll read six bytes from the accelerometer into temp	
#ifdef HW_3v4
	xgReadBytes(OUT_X_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	xgReadBytes(OUTX_L_XL, temp, 6); // Read 6 bytes, beginning at OUT_X_L_XL
#endif
	ax = (temp[1] << 8) | temp[0]; // Store x-axis values into ax
	ay = (temp[3] << 8) | temp[2]; // Store y-axis values into ay
	az = (temp[5] << 8) | temp[4]; // Store z-axis values into az
	if (_autoCalc)
	{
		ax -= aBiasRaw[X_AXIS];
		ay -= aBiasRaw[Y_AXIS];
		az -= aBiasRaw[Z_AXIS];
	}
}

int16_t readAccel_axis(enum lsm9ds1_axis axis)
{
#ifdef HW_3v4
	uint8_t temp[2];
	int16_t value;
	xgReadBytes(OUT_X_L_XL + (2 * axis), temp, 2);
	value = (temp[1] << 8) | temp[0];
	
	if (_autoCalc)
		value -= aBiasRaw[axis];
	
	return value;
#endif
}

void readMag()
{
#ifdef HW_3v4
	uint8_t temp[6]; // We'll read six bytes from the mag into temp	
	mReadBytes(OUT_X_L_M, temp, 6); // Read 6 bytes, beginning at OUT_X_L_M
	mx = (temp[1] << 8) | temp[0]; // Store x-axis values into mx
	my = (temp[3] << 8) | temp[2]; // Store y-axis values into my
	mz = (temp[5] << 8) | temp[4]; // Store z-axis values into mz
#endif
}

int16_t readMag_axis(enum lsm9ds1_axis axis)
{
#ifdef HW_3v4
	uint8_t temp[2];
	mReadBytes(OUT_X_L_M + (2 * axis), temp, 2);
	return (temp[1] << 8) | temp[0];
#endif
}

void readTemp()
{
#ifdef HW_3v4
	uint8_t temp[2]; // We'll read two bytes from the temperature sensor into temp	
	xgReadBytes(OUT_TEMP_L, temp, 2); // Read 2 bytes, beginning at OUT_TEMP_L
	temperature = ((int16_t)temp[1] << 8) | temp[0];
#endif
}

void readGyro()
{
	uint8_t temp[6]; // We'll read six bytes from the gyro into temp#ifdef HW_3v4
#ifdef HW_3v4
	xgReadBytes(OUT_X_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
#endif
#if defined(HW_4v0) || defined(HW_4v1)
	xgReadBytes(OUTX_L_G, temp, 6); // Read 6 bytes, beginning at OUT_X_L_G
#endif
	gx = (temp[1] << 8) | temp[0]; // Store x-axis values into gx
	gy = (temp[3] << 8) | temp[2]; // Store y-axis values into gy
	gz = (temp[5] << 8) | temp[4]; // Store z-axis values into gz
	if (_autoCalc)
	{
		gx -= gBiasRaw[X_AXIS];
		gy -= gBiasRaw[Y_AXIS];
		gz -= gBiasRaw[Z_AXIS];
	}
}

int16_t readGyro_axis(enum lsm9ds1_axis axis)
{
#ifdef HW_3v4
	uint8_t temp[2];
	int16_t value;
	
	xgReadBytes(OUT_X_L_G + (2 * axis), temp, 2);
	
	value = (temp[1] << 8) | temp[0];
	
	if (_autoCalc)
		value -= gBiasRaw[axis];
	
	return value;
#endif
}

float calcGyro(int16_t gyro)
{
	// Return the gyro raw reading times our pre-calculated DPS / (ADC tick):
	return gRes * gyro; 
}

float calcAccel(int16_t accel)
{
	// Return the accel raw reading times our pre-calculated g's / (ADC tick):
	return aRes * accel;
}

float calcMag(int16_t mag)
{
	// Return the mag raw reading times our pre-calculated Gs / (ADC tick):
	return mRes * mag;
}

void setGyroScale(uint16_t gScl)
{
#ifdef HW_3v4
	// Read current value of CTRL_REG1_G:
	uint8_t ctrl1RegValue = xgReadByte(CTRL_REG1_G);
	// Mask out scale bits (3 & 4):
	ctrl1RegValue &= 0xE7;
	switch (gScl)
	{
		case 500:
			ctrl1RegValue |= (0x1 << 3);
			settings.gyro.scale = 500;
			break;
		case 2000:
			ctrl1RegValue |= (0x3 << 3);
			settings.gyro.scale = 2000;
			break;
		default: // Otherwise we'll set it to 245 dps (0x0 << 4)
			settings.gyro.scale = 245;
			break;
	}
	xgWriteByte(CTRL_REG1_G, ctrl1RegValue);
	
	calcgRes();	
#endif
}

void setAccelScale(uint8_t aScl)
{
#ifdef HW_3v4
	// We need to preserve the other bytes in CTRL_REG6_XL. So, first read it:
	uint8_t tempRegValue = xgReadByte(CTRL_REG6_XL);
	// Mask out accel scale bits:
	tempRegValue &= 0xE7;
	
	switch (aScl)
	{
		case 4:
			tempRegValue |= (0x2 << 3);
			settings.accel.scale = 4;
			break;
		case 8:
			tempRegValue |= (0x3 << 3);
			settings.accel.scale = 8;
			break;
		case 16:
			tempRegValue |= (0x1 << 3);
			settings.accel.scale = 16;
			break;
		default: // Otherwise it'll be set to 2g (0x0 << 3)
			settings.accel.scale = 2;
			break;
	}
	xgWriteByte(CTRL_REG6_XL, tempRegValue);
	
	// Then calculate a new aRes, which relies on aScale being set correctly:
	calcaRes();
#endif
}

void setMagScale(uint8_t mScl)
{
#ifdef HW_3v4
	// We need to preserve the other bytes in CTRL_REG6_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG2_M);
	// Then mask out the mag scale bits:
	temp &= 0xFF^(0x3 << 5);
	
	switch (mScl)
	{
	case 8:
		temp |= (0x1 << 5);
		settings.mag.scale = 8;
		break;
	case 12:
		temp |= (0x2 << 5);
		settings.mag.scale = 12;
		break;
	case 16:
		temp |= (0x3 << 5);
		settings.mag.scale = 16;
		break;
	default: // Otherwise we'll default to 4 gauss (00)
		settings.mag.scale = 4;
		break;
	}	
	
	// And write the new register value back into CTRL_REG6_XM:
	mWriteByte(CTRL_REG2_M, temp);
	
	// We've updated the sensor, but we also need to update our class variables
	// First update mScale:
	//mScale = mScl;
	// Then calculate a new mRes, which relies on mScale being set correctly:
	calcmRes();
#endif
}

void setGyroODR(uint8_t gRate)
{
#ifdef HW_3v4
	// Only do this if gRate is not 0 (which would disable the gyro)
	if ((gRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_G. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG1_G);
		// Then mask out the gyro ODR bits:
		temp &= 0xFF^(0x7 << 5);
		temp |= (gRate & 0x07) << 5;
		// Update our settings struct
		settings.gyro.sampleRate = gRate & 0x07;
		// And write the new register value back into CTRL_REG1_G:
		xgWriteByte(CTRL_REG1_G, temp);
	}
#endif
}

void setAccelODR(uint8_t aRate)
{
#ifdef HW_3v4
	// Only do this if aRate is not 0 (which would disable the accel)
	if ((aRate & 0x07) != 0)
	{
		// We need to preserve the other bytes in CTRL_REG1_XM. So, first read it:
		uint8_t temp = xgReadByte(CTRL_REG6_XL);
		// Then mask out the accel ODR bits:
		temp &= 0x1F;
		// Then shift in our new ODR bits:
		temp |= ((aRate & 0x07) << 5);
		settings.accel.sampleRate = aRate & 0x07;
		// And write the new register value back into CTRL_REG1_XM:
		xgWriteByte(CTRL_REG6_XL, temp);
	}
#endif
}

void setMagODR(uint8_t mRate)
{
#ifdef HW_3v4
	// We need to preserve the other bytes in CTRL_REG5_XM. So, first read it:
	uint8_t temp = mReadByte(CTRL_REG1_M);
	// Then mask out the mag ODR bits:
	temp &= 0xFF^(0x7 << 2);
	// Then shift in our new ODR bits:
	temp |= ((mRate & 0x07) << 2);
	settings.mag.sampleRate = mRate & 0x07;
	// And write the new register value back into CTRL_REG5_XM:
	mWriteByte(CTRL_REG1_M, temp);
#endif
}

void calcgRes()
{
	gRes = ((float) settings.gyro.scale) / 32768.0;
}

void calcaRes()
{
	aRes = ((float) settings.accel.scale) / 32768.0;
}

void calcmRes()
{
	//mRes = ((float) settings.mag.scale) / 32768.0;
	switch (settings.mag.scale)
	{
	case 4:
		mRes = magSensitivity[0];
		break;
	case 8:
		mRes = magSensitivity[1];
		break;
	case 12:
		mRes = magSensitivity[2];
		break;
	case 16:
		mRes = magSensitivity[3];
		break;
	}
	
}

void configInt(enum interrupt_select interrupt, uint8_t generator,
	                     enum h_lactive activeLow, enum pp_od pushPull)
{
#ifdef HW_3v4
	// Write to INT1_CTRL or INT2_CTRL. [interupt] should already be one of
	// those two values.
	// [generator] should be an OR'd list of values from the interrupt_generators enum
	xgWriteByte(interrupt, generator);
	
	// Configure CTRL_REG8
	uint8_t temp;
	temp = xgReadByte(CTRL_REG8);
	
	if (activeLow) temp |= (1<<5);
	else temp &= ~(1<<5);
	
	if (pushPull) temp &= ~(1<<4);
	else temp |= (1<<4);
	
	xgWriteByte(CTRL_REG8, temp);
#endif
}

void configInactivity(uint8_t duration, uint8_t threshold, bool sleepOn)
{
#ifdef HW_3v4
	uint8_t temp = 0;
	
	temp = threshold & 0x7F;
	if (sleepOn) temp |= (1<<7);
	xgWriteByte(ACT_THS, temp);
	
	xgWriteByte(ACT_DUR, duration);
#endif
}

uint8_t getInactivity()
{
#ifdef HW_3v4
	uint8_t temp = xgReadByte(STATUS_REG_0);
	temp &= (0x10);
	return temp;
#endif
}

void configAccelInt(uint8_t generator, bool andInterrupts)
{
#ifdef HW_3v4
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (andInterrupts) temp |= 0x80;
	xgWriteByte(INT_GEN_CFG_XL, temp);
#endif
}

void configAccelThs(uint8_t threshold, enum lsm9ds1_axis axis, uint8_t duration, bool wait)
{
#ifdef HW_3v4
	// Write threshold value to INT_GEN_THS_?_XL.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_X_XL + axis, threshold);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_XL, temp);
#endif
}

uint8_t getAccelIntSrc()
{
#ifdef HW_3v4
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_XL);
	
	// Check if the IA_XL (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
#endif
}

void configGyroInt(uint8_t generator, bool aoi, bool latch)
{
#ifdef HW_3v4
	// Use variables from accel_interrupt_generator, OR'd together to create
	// the [generator]value.
	uint8_t temp = generator;
	if (aoi) temp |= 0x80;
	if (latch) temp |= 0x40;
	xgWriteByte(INT_GEN_CFG_G, temp);
#endif
}

void configGyroThs(int16_t threshold, enum lsm9ds1_axis axis, uint8_t duration, bool wait)
{
#ifdef HW_3v4
	uint8_t buffer[2];
	buffer[0] = (threshold & 0x7F00) >> 8;
	buffer[1] = (threshold & 0x00FF);
	// Write threshold value to INT_GEN_THS_?H_G and  INT_GEN_THS_?L_G.
	// axis will be 0, 1, or 2 (x, y, z respectively)
	xgWriteByte(INT_GEN_THS_XH_G + (axis * 2), buffer[0]);
	xgWriteByte(INT_GEN_THS_XH_G + 1 + (axis * 2), buffer[1]);
	
	// Write duration and wait to INT_GEN_DUR_XL
	uint8_t temp;
	temp = (duration & 0x7F);
	if (wait) temp |= 0x80;
	xgWriteByte(INT_GEN_DUR_G, temp);
#endif
}

uint8_t getGyroIntSrc()
{
#ifdef HW_3v4
	uint8_t intSrc = xgReadByte(INT_GEN_SRC_G);
	
	// Check if the IA_G (interrupt active) bit is set
	if (intSrc & (1<<6))
	{
		return (intSrc & 0x3F);
	}
	
	return 0;
#endif
}

void configMagInt(uint8_t generator, enum h_lactive activeLow, bool latch)
{
#ifdef HW_3v4
	// Mask out non-generator bits (0-4)
	uint8_t config = (generator & 0xE0);	
	// IEA bit is 0 for active-low, 1 for active-high.
	if (activeLow == INT_ACTIVE_HIGH) config |= (1<<2);
	// IEL bit is 0 for latched, 1 for not-latched
	if (!latch) config |= (1<<1);
	// As long as we have at least 1 generator, enable the interrupt
	if (generator != 0) config |= (1<<0);
	
	mWriteByte(INT_CFG_M, config);
#endif
}

void configMagThs(uint16_t threshold)
{
#ifdef HW_3v4
	// Write high eight bits of [threshold] to INT_THS_H_M
	mWriteByte(INT_THS_H_M, (uint8_t)((threshold & 0x7F00) >> 8));
	// Write low eight bits of [threshold] to INT_THS_L_M
	mWriteByte(INT_THS_L_M, (uint8_t)threshold & 0x00FF);
#endif
}

uint8_t getMagIntSrc()
{
#ifdef HW_3v4
	uint8_t intSrc = mReadByte(INT_SRC_M);
	
	// Check if the INT (interrupt active) bit is set
	if (intSrc & (1<<0))
	{
		return (intSrc & 0xFE);
	}
	
	return 0;
#endif
}

void sleepGyro(bool enable)
{
#ifdef HW_3v4
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<6);
	else temp &= ~(1<<6);
	xgWriteByte(CTRL_REG9, temp);
#endif
}

void enableFIFO(bool enable)
{
#ifdef HW_3v4
	uint8_t temp = xgReadByte(CTRL_REG9);
	if (enable) temp |= (1<<1);
	else temp &= ~(1<<1);
	xgWriteByte(CTRL_REG9, temp);
#endif
}

void setFIFO(enum fifoMode_type fifoMode, uint16_t fifoThs)
{
	// Limit threshold - 0x1F (31) is the maximum. If more than that was asked
	// limit it to the maximum.
#ifdef HW_3v4
	uint16_t threshold = fifoThs <= 0x1F ? fifoThs : 0x1F;
	xgWriteByte(FIFO_CTRL, ((fifoMode & 0x7) << 5) | (threshold & 0x1F));
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	uint16_t threshold = fifoThs <= 0x0FFC ? fifoThs : 0x0FFC;
	//xgWriteByte(MASTER_CONFIG, 0x00);//Set the STOP_ON_FTH bit to limit the FIFO memory depth
	//xgWriteByte(CTRL3_C, 0x04);//Set the STOP_ON_FTH bit to limit the FIFO memory depth
	xgWriteByte(CTRL4_C, 0x01);//Set the STOP_ON_FTH bit to limit the FIFO memory depth
	//xgWriteByte(CTRL5_C, 0x00);//Set the STOP_ON_FTH bit to limit the FIFO memory depth
	//xgWriteByte(CTRL6_C, 0x00);//Set the STOP_ON_FTH bit to limit the FIFO memory depth
	xgWriteByte(FIFO_CTRL1, threshold & 0xFF);
	xgWriteByte(FIFO_CTRL2, (threshold & 0x0F00)>>8);
	xgWriteByte(FIFO_CTRL3, 0x09);//Set the FIFO to no accel or gyro decimation
	xgWriteByte(FIFO_CTRL4, 0x09);//Set the FIFO to no accel or gyro decimation
	xgWriteByte(FIFO_CTRL5, (fifoMode & 0x07)|0x40);//Set FIFO ODR to 6.66kHZ to enable FIFO
#endif
}

uint8_t getFIFOSamples()
{
#ifdef HW_3v4
	return (xgReadByte(FIFO_SRC) & 0x3F);
#endif
}

void constrainScales()
{
	if ((settings.gyro.scale != 245) && (settings.gyro.scale != 500) && 
		(settings.gyro.scale != 2000))
	{
		settings.gyro.scale = 245;
	}
		
	if ((settings.accel.scale != 2) && (settings.accel.scale != 4) &&
		(settings.accel.scale != 8) && (settings.accel.scale != 16))
	{
		settings.accel.scale = 2;
	}
		
	if ((settings.mag.scale != 4) && (settings.mag.scale != 8) &&
		(settings.mag.scale != 12) && (settings.mag.scale != 16))
	{
		settings.mag.scale = 4;
	}
}

void xgWriteByte(uint8_t subAddress, uint8_t data)
{
		writeByte(_xgAddress, subAddress, data);
}

void mWriteByte(uint8_t subAddress, uint8_t data)
{
		return writeByte(_mAddress, subAddress, data);
}

uint8_t xgReadByte(uint8_t subAddress)
{
		return readByte(_xgAddress, subAddress);
}

void xgReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
		readBytes(_xgAddress, subAddress, dest, count);
}

uint8_t mReadByte(uint8_t subAddress)
{
		return readByte(_mAddress, subAddress);
}

void mReadBytes(uint8_t subAddress, uint8_t * dest, uint8_t count)
{
		readBytes(_mAddress, subAddress, dest, count);
}

// Configure SERCOM channel 3 as I2C master for IMU communication
void initI2C()
{
	struct i2c_master_config config_i2c_master;
	i2c_master_get_config_defaults(&config_i2c_master);
	config_i2c_master.baud_rate = I2C_MASTER_BAUD_RATE_400KHZ;
	config_i2c_master.buffer_timeout = MASTER_TIMEOUT;
	config_i2c_master.pinmux_pad0 = PINMUX_PA22C_SERCOM3_PAD0;
	config_i2c_master.pinmux_pad1 = PINMUX_PA23C_SERCOM3_PAD1;
	i2c_master_init(&i2c_master_instance, SERCOM3, &config_i2c_master);
	i2c_master_enable(&i2c_master_instance);
}

// Wire.h read and write protocols
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
	/* Timeout counter. */
	uint16_t timeout = 0;
	/* Init i2c packet. */
	write_buffer[0] = subAddress;
	write_buffer[1] = data;
	struct i2c_master_packet packet = {
		.address     = address,
		.data_length = 2,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

    /* Write buffer to slave until success. */
    while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=
    STATUS_OK) {
	    /* Increment timeout counter and check if timed out. */
	    if (timeout++ == MASTER_TIMEOUT) {
		    break;
	    }
    }
}

uint8_t readByte(uint8_t address, uint8_t subAddress)
{
	int timeout = 0;
	uint8_t data; // `data` will store the register data	
	
	/* Init i2c packet. */
	write_buffer[0] = subAddress;
	struct i2c_master_packet packet = {
		.address     = address,
		.data_length = 1,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	/* Write buffer to slave until success. */
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == MASTER_TIMEOUT) {
			break;
		}
	}

	/* Read from slave until success. */
	timeout = 0;
	packet.data = read_buffer;
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == MASTER_TIMEOUT) {
			break;
		}
	}
	
	data = read_buffer[0];                      // Fill Rx buffer with result
	return data;                             // Return data read from slave register
}

uint8_t readBytes(uint8_t address, uint8_t subAddress, uint8_t * dest, uint8_t count)
{  
	int timeout = 0;
	
	/* Init i2c packet. */
#ifdef HW_3v4
	write_buffer[0] = subAddress | 0x80;
#endif
#if  defined(HW_4v0) || defined(HW_4v1)
	write_buffer[0] = subAddress;
#endif
	struct i2c_master_packet packet = {
		.address     = address,
		.data_length = 1,
		.data        = write_buffer,
		.ten_bit_address = false,
		.high_speed      = false,
		.hs_master_code  = 0x0,
	};

	/* Write buffer to slave until success. */
	while (i2c_master_write_packet_wait(&i2c_master_instance, &packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == MASTER_TIMEOUT) {
			break;
		}
	}

	/* Read from slave until success. */
	timeout = 0;
	packet.data_length = count;
	packet.data = read_buffer;
	while (i2c_master_read_packet_wait(&i2c_master_instance, &packet) !=
	STATUS_OK) {
		/* Increment timeout counter and check if timed out. */
		if (timeout++ == MASTER_TIMEOUT) {
			break;
		}
	}
	
	uint8_t i = 0;
	for (i=0; i<count; i++)
	{
		dest[i] = read_buffer[i];
	}
	return count;
}

void CorrectIMUvalues(uint8_t connector_orient, uint8_t power_orient){
	if((connector_orient == 0 || power_orient == 0)||(connector_orient > 6 || power_orient > 6)){
		caz = az;
		cgz = gz;
		cmz = mz;
		cax = ax;
		cay = ay;
		cgx = gx;
		cgy = gy;
		cmx = mx;
		cmy = my;
	} else if(connector_orient == ORIENT_UP){ // Connectors Up
		caz = az;
		cgz = gz;
		cmz = mz;
		if(power_orient == ORIENT_LEFT){ // Power Left
			cax = -ay;
			cay = ax;
			cgx = -gy;
			cgy = gx;
			cmx = my;
			cmy = -mx;
		}else if(power_orient == ORIENT_RIGHT){ // Power Right
			cax = ay;
			cay = -ax;
			cgx = gy;
			cgy = -gx;
			cmx = -my;
			cmy = mx;
		}else if(power_orient == ORIENT_REAR){ // Power Rear
			cax = -ax;
			cay = -ay;
			cgx = -gx;
			cgy = -gy;
			cmx = -mx;
			cmy = -my;
		}else if(power_orient == ORIENT_FRONT){ // Power Front
			cax = ax;
			cay = ay;
			cgx = gx;
			cgy = gy;
			cmx = mx;
			cmy = my;
		}
	} else if(connector_orient == ORIENT_DOWN){ // Connectors Down
		caz = -az;
		cgz = -gz;
		cmz = -mz;
		if(power_orient == ORIENT_LEFT){ // Power Left
			cax = ay;
			cay = ax;
			cgx = gy;
			cgy = gx;
			cmx = -my;
			cmy = -mx;
		}else if(power_orient == ORIENT_RIGHT){ // Power Right
			cax = -ay;
			cay = -ax;
			cgx = -gy;
			cgy = -gx;
			cmx = my;
			cmy = mx;
		}else if(power_orient == ORIENT_REAR){ // Power Rear
			cax = ax;
			cay = -ay;
			cgx = gx;
			cgy = -gy;
			cmx = mx;
			cmy = -my;
		}else if(power_orient == ORIENT_FRONT){ // Power Front
			cax = -ax;
			cay = ay;
			cgx = -gx;
			cgy = gy;
			cmx = -mx;
			cmy = my;
		}
	} else if(connector_orient == ORIENT_LEFT){ // Connectors Left
		caz = ax;
		cgz = gx;
		cmz = -mx;
		if(power_orient == ORIENT_UP){ // Power Up
			cax = ay;
			cay = az;
			cgx = gy;
			cgy = gz;
			cmx = -my;
			cmy = mz;
		}else if(power_orient == ORIENT_DOWN){ // Power Down
			cax = -ay;
			cay = -az;
			cgx = -gy;
			cgy = -gz;
			cmx = my;
			cmy = -mz;
		}else if(power_orient == ORIENT_REAR){ // Power Rear
			cax = az;
			cay = -ay;
			cgx = gz;
			cgy = -gy;
			cmx = -mz;
			cmy = -my;
		}else if(power_orient == ORIENT_FRONT){ // Power Front
			cax = -az;
			cay = ay;
			cgx = -gz;
			cgy = gy;
			cmx = mz;
			cmy = my;
		}
	} else if(connector_orient == ORIENT_RIGHT){ // Connectors Right
		caz = -ax;
		cgz = -gx;
		cmz = mx;
		if(power_orient == ORIENT_UP){ // Power Up
			cax = -ay;
			cay = az;
			cgx = -gy;
			cgy = gz;
			cmx = my;
			cmy = mz;
		}else if(power_orient == ORIENT_DOWN){ // Power Down
			cax = ay;
			cay = -az;
			cgx = gy;
			cgy = -gz;
			cmx = -my;
			cmy = -mz;
		}else if(power_orient == ORIENT_REAR){ // Power Rear
			cax = -az;
			cay = -ay;
			cgx = -gz;
			cgy = -gy;
			cmx = mz;
			cmy = -my;
		}else if(power_orient == ORIENT_FRONT){ // Power Front
			cax = az;
			cay = ay;
			cgx = gz;
			cgy = gy;
			cmx = -mz;
			cmy = my;
		}
	} else if(connector_orient == ORIENT_REAR){ // Connectors Rear
		caz = -ay;
		cgz = -gy;
		cmz = -my;
		if(power_orient == ORIENT_UP){ // Power Up
			cax = ax;
			cay = az;
			cgx = gx;
			cgy = gz;
			cmx = mx;
			cmy = mz;
		}else if(power_orient == ORIENT_DOWN){ // Power Down
			cax = -ax;
			cay = -az;
			cgx = -gx;
			cgy = -gz;
			cmx = -mx;
			cmy = -mz;
		}else if(power_orient == ORIENT_LEFT){ // Power Left
			cax = -az;
			cay = ax;
			cgx = -gz;
			cgy = gx;
			cmx = mz;
			cmy = -mx;
		}else if(power_orient == ORIENT_RIGHT){ // Power Right
			cax = az;
			cay = -ax;
			cgx = gz;
			cgy = -gx;
			cmx = -mz;
			cmy = mx;
		}
	} else if(connector_orient == ORIENT_FRONT){ // Connectors Front
		caz = ay;
		cgz = gy;
		cgz = my;
		if(power_orient == ORIENT_UP){ // Power Up
			cax = -ax;
			cay = az;
			cgx = -gx;
			cgy = gz;
			cmx = -mx;
			cmy = mz;
		}else if(power_orient == ORIENT_DOWN){ // Power Down
			cax = ax;
			cay = -az;
			cgx = gx;
			cgy = -gz;
			cmx = mx;
			cmy = -mz;
		}else if(power_orient == ORIENT_LEFT){ // Power Left
			cax = az;
			cay = ax;
			cgx = gz;
			cgy = gx;
			cmx = -mz;
			cmy = -mx;
		}else if(power_orient == ORIENT_RIGHT){ // Power Right
			cax = -az;
			cay = -ax;
			cgx = -gz;
			cgy = -gx;
			cmx = mz;
			cmy = mx;
		}
	}
}

void calculate_heading(){
	headingTime = millis();
	if(abs(gzKalman) >= 0.5){
		if(headingTime < lheadingTime){
			heading += (gzKalman) * (((float)(headingTime + (0xFFFFFFFF - lheadingTime)))/1000);
		}
		else
			heading += (gzKalman) * (((float)(headingTime - lheadingTime))/1000);
	}
	lheadingTime = headingTime;
	if(heading < 0)
		heading = 360 + heading;
	else if(heading > 360)
		heading = heading - 360;
}

void update_kalman_limits(){
	if(axKalman > kalmanAX_max)
		kalmanAX_max = axKalman;
	else if(axKalman < kalmanAX_min)
		kalmanAX_min = axKalman;

	if(ayKalman > kalmanAY_max)
		kalmanAY_max = ayKalman;
	else if(ayKalman < kalmanAY_min)
		kalmanAY_min = ayKalman;

	if(azKalman > kalmanAZ_max)
		kalmanAZ_max = azKalman;
	else if(azKalman < kalmanAZ_min)
		kalmanAZ_min = azKalman;
	
	if(gxKalman > kalmanGX_max)
		kalmanGX_max = gxKalman;
	else if(gxKalman < kalmanGX_min)
		kalmanGX_min = gxKalman;

	if(gyKalman > kalmanGY_max)
		kalmanGY_max = gyKalman;
	else if(gyKalman < kalmanGY_min)
		kalmanGY_min = gyKalman;

	if(gzKalman > kalmanAZ_max)
		kalmanGZ_max = gzKalman;
	else if(gzKalman < kalmanGZ_min)
		kalmanGZ_min = gzKalman;
}

int16_t averageAX(){
	static int sample_index = 0;
	AXtotal -= AXaverage[sample_index];
	AXtotal += cax;
	AXaverage[sample_index] = cax;
	
	sample_index++;
	if(sample_index == ACCELXYsamples)
	sample_index = 0;

	return (int16_t)(AXtotal/ACCELXYsamples);
}

int16_t averageAY(){
	static int sample_index = 0;
	AYtotal -= AYaverage[sample_index];
	AYtotal += cay;
	AYaverage[sample_index] = cay;
	
	sample_index++;
	if(sample_index == ACCELXYsamples)
	sample_index = 0;

	return (int16_t)(AYtotal/ACCELXYsamples);
}

int16_t averageAZ(){
	static int sample_index = 0;
	AZtotal -= AZaverage[sample_index];
	AZtotal += caz;
	AZaverage[sample_index] = caz;

	sample_index++;
	if(sample_index == ACCELZsamples)
	sample_index = 0;

	return (int16_t)(AZtotal/ACCELZsamples);
}

float averageGX(){
	static int sample_index = 0;
	GXtotal -= GXaverage[sample_index];
	GXtotal += calcGyro(cgx);
	GXaverage[sample_index] = calcGyro(cgx);

	sample_index++;
	if(sample_index == GYROsamples)
	sample_index = 0;

	return (GXtotal/GYROsamples);
}

float averageGY(){
	static int sample_index = 0;
	GYtotal -= GYaverage[sample_index];
	GYtotal += calcGyro(cgy);
	GYaverage[sample_index] = calcGyro(cgy);

	sample_index++;
	if(sample_index == GYROsamples)
	sample_index = 0;

	return (GYtotal/GYROsamples);
}

float averageGZ(){
	static int sample_index = 0;
	GZtotal -= GZaverage[sample_index];
	GZtotal += calcGyro(cgz);
	GZaverage[sample_index] = calcGyro(cgz);

	sample_index++;
	if(sample_index == GYROsamples)
	sample_index = 0;

	return (GZtotal/GYROsamples);
}

void initKalman(float meas, float est, float _q)
{
	for(int i = 0; i < KalmanArraySize; i++){
		err_measure[i] = meas;
		err_estimate[i] = est;
		q[i] = _q;
		current_estimate[i] = 0;
		last_estimate[i] = 0;
		kalman_gain[i] = 0;
	}

	err_measure[ax_kalman] = 15;
	err_estimate[ax_kalman] = 15;
	q[ax_kalman] = 0.3;

	err_measure[ay_kalman] = 15;
	err_estimate[ay_kalman] = 15;
	q[ay_kalman] = 0.3;

	// 	err_measure[ay_kalman] = 20;
	// 	err_estimate[ay_kalman] = 20;
	// 	q[ay_kalman] = 0.8;

	err_measure[az_kalman] = 30;
	err_estimate[az_kalman] = 30;
	q[az_kalman] = 0.3;

	// 	err_measure[gx_kalman] = 3;
	// 	err_estimate[gx_kalman] = 3;
	// 	q[gx_kalman] = 0.9;
	//
	// 	err_measure[gy_kalman] = 3;
	// 	err_estimate[gy_kalman] = 3;
	// 	q[gy_kalman] = 0.9;
	//
	// 	err_measure[gz_kalman] = 0.1;
	// 	err_estimate[gz_kalman] = 1;
	// 	q[gz_kalman] = 0.99;

	err_measure[light_kalman] = 200;
	err_estimate[light_kalman] = 200;
	q[light_kalman] = 0.008;
}

float updateKalman(float meas, int kalmanIndex)
{
	kalman_gain[kalmanIndex] = err_estimate[kalmanIndex]/(err_estimate[kalmanIndex] + err_measure[kalmanIndex]);
	kalman_gain[kalmanIndex] = max(kalman_gain[kalmanIndex],0.015);
	current_estimate[kalmanIndex] = last_estimate[kalmanIndex] + kalman_gain[kalmanIndex] * (meas - last_estimate[kalmanIndex]);
	err_estimate[kalmanIndex] =  (1.0 - kalman_gain[kalmanIndex])*err_estimate[kalmanIndex] + abs(last_estimate[kalmanIndex]-current_estimate[kalmanIndex])*q[kalmanIndex];
	last_estimate[kalmanIndex]=current_estimate[kalmanIndex];

	return current_estimate[kalmanIndex];
}

#endif