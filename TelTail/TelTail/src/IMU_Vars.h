/*
 * IMU_Vars.h
 *
 * Created: 11/15/2019 2:19:04 PM
 *  Author: NEO
 */ 

#ifndef IMU_VARS_H
#define IMU_VARS_H

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