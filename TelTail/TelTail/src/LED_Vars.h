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

#define TURN_OFF_TIME 400
#define TURN_ON_TIME 250
#define SHUFFLE_TIME 30000

enum modes_analog{
	MODE_ANALOG_STATIC = 0,
	MODE_ANALOG_COLOR_CYCLE = 1,
	MODE_ANALOG_COMPASS_CYCLE = 2,
	MODE_ANALOG_THROTTLE = 3,
	MODE_ANALOG_RPM_CYCLE = 4,
	MODE_ANALOG_RPM_THROTTLE = 5,
	MODE_ANALOG_X_ACCEL = 6,
	MODE_ANALOG_Y_ACCEL = 7,
	MODE_ANALOG_CUSTOM = 8
} ;

enum modes_digital{
	MODE_DIGITAL_STATIC = 0,
	MODE_DIGITAL_SKITTLES = 1,
	MODE_DIGITAL_GRADIENT_CYCLE = 2,
	MODE_DIGITAL_COMPASS_CYCLE = 3,
	MODE_DIGITAL_THROTTLE = 4,
	MODE_DIGITAL_RPM_CYCLE = 5,
	MODE_DIGITAL_RPM_THROTTLE = 6,
	MODE_DIGITAL_COMPASS_WHEEL = 7,
	MODE_DIGITAL_COMPASS_SNAKE = 8
} ;

enum  rate_bases{
	RATE_STATIC = 0,
	RATE_YAW_RATE = 1,
	RATE_ROLL_RATE = 2,
	RATE_RPM = 3,
	RATE_THROTTLE = 4,
	RATE_X_ACCEL = 5,
	RATE_Y_ACCEL = 6,
	RATE_Z_ACCEL = 7
};

enum bright_base{
	BRIGHT_STATIC = 0,
	BRIGHT_YAW_RATE = 1,
	BRIGHT_ROLL_RATE = 2,
	BRIGHT_RPM = 3,
	BRIGHT_THROTTLE = 4,
	BRIGHT_X_ACCEL = 5,
	BRIGHT_Y_ACCEL = 6,
	BRIGHT_Z_ACCEL = 7,
	BRIGHT_STROBE = 8
};

enum color_bases{
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
};

enum RGB_types{
	RGB_ANALOG = 0,
	RGB_DIGITAL_APA102,
	//RGB_DIGITAL_WS2815,
	RGB_NONE
};

enum BRAKE_type{
	BRAKE_FADE = 0,
	BRAKE_BLINK = 1,
	BRAKE_FADE_BLINK = 2,
	BRAKE_BLINK_FADE = 3,
	BRAKE_FADING_BLINK = 4,
	BRAKE_PACED_BLINK = 5
};

enum ERROR_type{
	ERROR_RED = 0,
	ERROR_BLUE = 1,
	ERROR_GREEN = 2,
	ERROR_TEAL = 3,
	ERROR_YELLOW = 4,
	ERROR_PURPLE = 5,
};

enum ERROR_duration{
	SHORT_ERROR, // 2s
	LONG_ERROR, // 5s
	PERMINENT_ERROR
};

enum light_type{
	LIGHT_HEAD = 0,
	LIGHT_BRAKE,
	LIGHT_SIDE
};

enum thermal_throttle_stages{
	THERM_TROTTLE_STAGE1 = 1,
	THERM_TROTTLE_STAGE2,
	THERM_TROTTLE_STAGE3,
	THERM_TROTTLE_STAGE4,
	THERM_TROTTLE_STAGE5
};

struct RGB_Vals{
	uint16_t LR;
	uint16_t LG;
	uint16_t LB;
	uint16_t RR;
	uint16_t RG;
	uint16_t RB;
};

#define HORN_PIN PIN_PA08
#if defined(HW_4v1)
	#define STAT_LED PIN_PB03
#endif

// Mode Vars
#define ANALOG_MODE_NUM 9
#define DIGITAL_MODE_NUM 9
uint8_t light_modes = ANALOG_MODE_NUM; // number of light modes, one indexed
uint8_t light_mode = 1; // current light mode, zero indexed

// Mode Specific Vars
struct RGB_Vals Static_RGB = {32767,16384,52428,0,52428,39321};
float RateSens[9] = {0,0.75,0,0.5,0.5,0,0.5,0.5,0.5};
float Brightness[9] = {0,0.25,0.5,0.5,0.5,0.5,0.5,0.5,0.5};
uint8_t ColorBase[9] = {0,1,2,7,1,7,9,10,1};
uint8_t BrightBase[9] = {255,0,0,0,3,3,0,0,2};
uint8_t RateBase[9] = {255,0,255,0,0,255,0,0,3};
uint8_t Digital_Static_Zoom = 1;
uint8_t Digital_Static_Shift = 50;
uint8_t Digital_Static_Brightness = 50;
uint8_t Digital_Skittles_Brightness = 50;
uint8_t Digital_Cycle_Zoom = 1;
uint8_t Digital_Cycle_Rate = 50;
uint8_t Digital_Cycle_Brightness = 50;
uint8_t Digital_Compass_Brightness = 50;
uint8_t Digital_Throttle_Zoom = 1;
uint8_t Digital_Throttle_Shift = 50;
uint8_t Digital_Throttle_Sens = 50;
uint8_t Digital_Throttle_Brightness = 50;
uint8_t Digital_RPM_Zoom = 7;
uint8_t Digital_RPM_Rate = 50;
uint8_t Digital_RPM_Brightness = 50;
struct RGB_Vals Custom_RGB = {32767,16384,52428,0,52428,39321};

// Ouput Vars
uint16_t head, brake = 0; // head/tail brightness
uint16_t brake_offset = 0x0B00;
struct RGB_Vals RGB_Ouptut = {0,0,0,0,0,0};
bool SUPRESS_LEFT_RGB = false;
bool SUPRESS_RIGHT_RGB = false;
bool SYNC_RGB = true;
bool BRAKE_ALWAYS_ON = true;
uint8_t brake_light_mode = BRAKE_FADE;
bool DEFAULT_STATE = false;
bool BRIGHTS_ENABLED = false;
uint8_t lowbeam_level = 70; // ~70%
bool STANDBY_ENABLED = 0;
bool SHUFFLE_ENABLED = 0;
uint16_t shuffled_analog_modes = 0;
uint16_t shuffled_digital_modes = 0;
bool TurnSignalOn = 0;
bool RestoreTurnLights = 0;

// Bools
uint8_t HEADLIGHTS = 0; // head/tail light enable bool
uint8_t SIDELIGHTS = 1; // side lights enable bool
uint8_t LIGHTS_ON = 1; // all lights enable bool
uint8_t LIGHT_CONTROLLED = 0;
uint8_t IMU_CONTROLED = 0;
uint8_t SWITCHES = 0;
uint8_t BRIGHTS = 0;

// Color Cycle Vars
uint8_t cycle = 0; // cycle state
uint32_t cycle_index = 0; // cycle color
uint16_t upColor = 0;
uint16_t downColor = 0;
float max_cycle_rate = 7500;

// Strobe Vars
uint32_t strobe_time = 0;
uint16_t strobe_on_dur = 50;
uint16_t strobe_off_dur = 500;

// Analog LED vars

// Digital LED vars
uint8_t RGB_led_type;
uint8_t configured_RGB_led_type;

bool DIGITAL_OFF = false;

#define MAX_LEDCOUNT 72

uint8_t led_num = 30;
uint8_t current_led_num = 0;
uint8_t digital_refresh_rate = 50; //Hz
uint32_t digital_refresh_time = 0;

#define L_GND PIN_PA07
#define R_GND PIN_PA14

uint8_t L_SPI_send_buf[(MAX_LEDCOUNT*4)+8];
uint8_t R_SPI_send_buf[(MAX_LEDCOUNT*4)+8];

#define TEMP_LIMIT_HARD 85.0
#define TEMP_LIMIT_SOFT 75.0
#define COOLING_EVENT_BUFFER 7500
#define COOLING_SAMPLE_PERIOD 1000
#define COOLING_SAMPLE_NUM 7
uint32_t cooling_event_timer = 0;
uint8_t thermal_throttle_stage = 0;

struct spi_module L_LED_SPI_instance;
struct spi_module R_LED_SPI_instance;

// Brightness: 1-31
const uint8_t brightness = 31;

#endif