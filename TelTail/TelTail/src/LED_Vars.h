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

#ifndef LEDVARS_H
#define LEDVARS_H

///////////////////   LED Variables   //////////////////
////////////////////////////////////////////////////////
typedef enum {
	MODE_STATIC = 0,
	MODE_COLOR_CYCLE = 1,
	MODE_COMPASS_CYCLE = 2,
	MODE_THROTTLE = 3,
	MODE_RPM_CYCLE = 4,
	MODE_RPM_THROTTLE = 5,
	MODE_X_ACCEL = 6,
	MODE_Y_ACCEL = 7,
	MODE_CUSTOM = 8
} modes;

typedef enum {
	RATE_STATIC = 0,
	RATE_YAW_RATE = 1,
	RATE_ROLL_RATE = 2,
	RATE_RPM = 3,
	RATE_THROTTLE = 4,
	RATE_X_ACCEL = 5,
	RATE_Y_ACCEL = 6,
	RATE_Z_ACCEL = 7
} rate_bases;

typedef enum {
	BRIGHT_STATIC = 0,
	BRIGHT_YAW_RATE = 1,
	BRIGHT_ROLL_RATE = 2,
	BRIGHT_RPM = 3,
	BRIGHT_THROTTLE = 4,
	BRIGHT_X_ACCEL = 5,
	BRIGHT_Y_ACCEL = 6,
	BRIGHT_Z_ACCEL = 7
} bright_base;

typedef enum {
	COLOR_STATIC = 0,
	COLOR_COLOR_CYCLE = 1,
	COLOR_COMPASS = 2,
	COLOR_YAW_RATE = 3,
	COLOR_ROLL_RATE = 4,
	COLOR_PITCH_RATE = 5,
	COLOR_THROTTLE = 6,
	COLOR_RPM = 7,
	COLOR_X_ACCEL = 8,
	COLOR_Y_ACCEL = 9,
	COLOR_Z_ACCEL = 10
} color_bases;

struct RGB_Vals{
	uint16_t LR;
	uint16_t LG;
	uint16_t LB;
	uint16_t RR;
	uint16_t RG;
	uint16_t RB;
};

#define AUX_PIN PIN_PA08

// Mode Vars
const uint8_t light_modes = 9; // number of light modes, one indexed
uint8_t light_mode = 1; // current light mode, zero indexed

// Mode Specific Vars
struct RGB_Vals Static_RGB = {32767,16384,52428,0,52428,39321};
float RateSens[9] = {0,0.75,0,0.5,0.5,0,0.5,0.5,0.5};
float Brightness[9] = {0,0.25,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
uint8_t ColorBase[9] = {0,1,2,7,1,7,9,10,1};
uint8_t BrightBase[9] = {255,0,0,0,3,3,0,0,2};
uint8_t RateBase[9] = {255,0,255,0,0,255,0,0,3};
struct RGB_Vals Custom_RGB = {32767,16384,52428,0,52428,39321};

// Ouput Vars
uint16_t head, brake = 0; // head/tail brightness
uint16_t brake_offset = 0x0B00;
struct RGB_Vals RGB_Ouptut = {0,0,0,0,0,0};

// Bools
uint8_t HEADLIGHTS = 0; // head/tail light enable bool
uint8_t SIDELIGHTS = 1; // side lights enable bool
uint8_t LIGHTS_ON = 1; // all lights enable bool
uint8_t LIGHT_CONTROLLED = 0;
uint8_t IMU_CONTROLED = 0;
uint8_t SWITCHES = 0;

// Color Cycle Vars
uint8_t cycle = 0; // cycle state
uint32_t cycle_index = 0; // cycle color
uint16_t upColor = 0;
uint16_t downColor = 0;
float max_cycle_rate = 7500;

#endif