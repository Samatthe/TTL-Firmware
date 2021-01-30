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

#ifndef IMU_VARS_H
#define IMU_VARS_H

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
int16_t temperature_raw; // Chip temperature
float IMU_temp;
float gBias[3], aBias[3], mBias[3];
int16_t gBiasRaw[3], aBiasRaw[3], mBiasRaw[3];

// Orientaion vars
uint8_t ORIENTATION[2] = {1,2}; // {connectors, power}
int16_t cgx = 0, cgy = 0, cgz = 0; // corrected x, y, and z axis readings of the gyroscope
int16_t cax = 0, cay = 0, caz = 0; // corrected x, y, and z axis readings of the accelerometer
int16_t cmx = 0, cmy = 0, cmz = 0; // corrected x, y, and z axis readings of the magnetometer

// Sensor averaging vars
uint16_t light_sens = 0;
#define LGHTsamples 150
#define ACCELXYsamples 100
#define ACCELZsamples 150
#define GYROsamples 5
uint16_t LGHTaverage[LGHTsamples];
int16_t AXaverage[ACCELXYsamples];
int16_t AYaverage[ACCELXYsamples];
int16_t AZaverage[ACCELZsamples];
float GXaverage[GYROsamples];
float GYaverage[GYROsamples];
float GZaverage[GYROsamples];
unsigned long LGHTtotal = 0;
long AXtotal = 0;
long AYtotal = 0;
long AZtotal = 0;
float GXtotal = 0;
float GYtotal = 0;
float GZtotal = 0;

int16_t avgAX = 0;
int16_t avgAY = 0;
int16_t avgAZ = 0;
float avgGX = 0;
float avgGY = 0;
float avgGZ = 0;
float kalmanAX_min = -350;
float kalmanAX_max = 350;
float kalmanAY_min = -350;
float kalmanAY_max = 350;
float kalmanAZ_min = -350;
float kalmanAZ_max = 350;
float kalmanGX_min = -350;
float kalmanGX_max = 350;
float kalmanGY_min = -350;
float kalmanGY_max = 350;
float kalmanGZ_min = -350;
float kalmanGZ_max = 350;

// Kalman filter vars
#define KalmanArraySize 7
enum kalmans{
	ax_kalman = 0,
	ay_kalman = 1,
	az_kalman = 2,
	gx_kalman = 3,
	gy_kalman = 4,
	gz_kalman = 5,
	light_kalman = 6
};
float err_measure[KalmanArraySize];
float err_estimate[KalmanArraySize];
float q[KalmanArraySize];
float current_estimate[KalmanArraySize];
float last_estimate[KalmanArraySize];
float kalman_gain[KalmanArraySize];
float axKalman = 0;
float ayKalman = 0;
float azKalman = 0;
float gxKalman = 0;
float gyKalman = 0;
float gzKalman = 0;

float heading = 0;
uint32_t headingTime = 0;
uint32_t lheadingTime = 0;

#endif