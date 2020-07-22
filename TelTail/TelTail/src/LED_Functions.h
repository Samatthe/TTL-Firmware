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

#ifndef LEDFUNCS_H
#define LEDFUNCS_H

#include "LED_Vars.h"
#include "Timing.h"
#include "APA102.h"
#include "SK9822.h"
#include "ESC_Vars.h"
#include "Remote_Vars.h"
#include "IMU_Vars.h"

// Define LED Functions
#define TURN_OFF_TIME 400
#define TURN_ON_TIME 250

struct tcc_module tcc0;
struct tcc_module tcc1;
struct tcc_module tcc2;
void configure_LED_PWM(void);
void setLeftRGB(uint16_t red, uint16_t green, uint16_t blue);
void setRightRGB(uint16_t red, uint16_t green, uint16_t blue);
void setWhite(uint16_t white);
void setRed(uint16_t red);
void setAux(bool aux);
struct RGB_Vals setCycleColor(uint16_t _upColor, uint16_t _downColor, int _cycle);
void setCycleColorSingle(uint16_t upColor, uint16_t downColor, int cycle, bool left);
void setConstBases(void);
void ERROR_LEDs(uint8_t error_type);
void TurnSignal(bool direction);
void BlinkTail(uint16_t brightness, float rate);
void AnalogSideLights(void);
void DigitalSideLights(void);

void getLightSens(uint16_t* light_val);
char sensorControl(void);
char lightControlHead(void);
char lightControlSide(void);
void setDigitalHue(uint16_t start, uint8_t zoom, uint16_t offset, uint8_t hue_brightness, bool reverse_direction);
void setDigitalLEDHue(uint16_t pos, uint8_t zoom, uint8_t hue_brightness, uint8_t led);



// Implement LED Functions



// Configure all of the LED ports as PWM outputs
void configure_LED_PWM(void)
{
	struct tcc_config config_tcc;
	tcc_get_config_defaults(&config_tcc, TCC0);
	config_tcc.counter.period = 0xFFFF;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

	if(RGB_led_type == RGB_ANALOG){
		config_tcc.compare.match[0] = 0;
		config_tcc.compare.match[1] = 0;
		config_tcc.compare.match[2] = 0;
		config_tcc.compare.match[3] = 0;
		config_tcc.pins.enable_wave_out_pin[0] = true;
		config_tcc.pins.enable_wave_out_pin[1] = true;
		config_tcc.pins.enable_wave_out_pin[2] = true;
		config_tcc.pins.enable_wave_out_pin[3] = true;
		config_tcc.pins.wave_out_pin[0]        = PIN_PA14F_TCC0_WO4;
		config_tcc.pins.wave_out_pin[1]        = PIN_PB11F_TCC0_WO5;
		config_tcc.pins.wave_out_pin[2]        = PIN_PA10F_TCC0_WO2;
		config_tcc.pins.wave_out_pin[3]        = PIN_PA11F_TCC0_WO3;
		config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA14F_TCC0_WO4;
		config_tcc.pins.wave_out_pin_mux[1]    = MUX_PB11F_TCC0_WO5;
		config_tcc.pins.wave_out_pin_mux[2]    = MUX_PA10F_TCC0_WO2;
		config_tcc.pins.wave_out_pin_mux[3]    = MUX_PA11F_TCC0_WO3;
		tcc_init(&tcc0, TCC0, &config_tcc);
		tcc_enable(&tcc0);
	}

	tcc_get_config_defaults(&config_tcc, TCC1);
	config_tcc.counter.period = 0xFFFF;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;
	config_tcc.compare.match[0] = 0;
	config_tcc.pins.enable_wave_out_pin[0] = true;
	config_tcc.pins.wave_out_pin[0]        = PIN_PA06E_TCC1_WO0;
	config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA06E_TCC1_WO0;
	if(RGB_led_type == RGB_ANALOG){
		config_tcc.compare.match[1] = 0;
		config_tcc.pins.enable_wave_out_pin[1] = true;
		config_tcc.pins.wave_out_pin[1]        = PIN_PA07E_TCC1_WO1;
		config_tcc.pins.wave_out_pin_mux[1]    = MUX_PA07E_TCC1_WO1;
	}
	tcc_init(&tcc1, TCC1, &config_tcc);
	tcc_enable(&tcc1);


	tcc_get_config_defaults(&config_tcc, TCC2);
	config_tcc.counter.period = 0xFFFF;
	config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

	config_tcc.compare.match[0] = 0;
	config_tcc.pins.enable_wave_out_pin[0] = true;
	config_tcc.pins.wave_out_pin[0]        = PIN_PA12E_TCC2_WO0;
	config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA12E_TCC2_WO0;
	if(RGB_led_type == RGB_ANALOG){
		config_tcc.pins.enable_wave_out_pin[1] = true;
		config_tcc.compare.match[1] = 0;
		config_tcc.pins.wave_out_pin[1]        = PIN_PA13E_TCC2_WO1;
		config_tcc.pins.wave_out_pin_mux[1]    = MUX_PA13E_TCC2_WO1;
	}
	tcc_init(&tcc2, TCC2, &config_tcc);
	tcc_enable(&tcc2);
}

void setLeftRGB(uint16_t red, uint16_t green, uint16_t blue) {
	RGB_Ouptut.LB = blue;
	RGB_Ouptut.LG = green;
	RGB_Ouptut.LR = red;
#if defined(HW_3v4) || defined(HW_4v0)
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (0), red);
	tcc_set_compare_value(&tcc2, (enum tcc_match_capture_channel) (1), green);
	tcc_set_compare_value(&tcc1, (enum tcc_match_capture_channel) (1), blue);
#endif
#if defined(HW_4v1)
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (1), red); 
	tcc_set_compare_value(&tcc2, (enum tcc_match_capture_channel) (1), green);
	tcc_set_compare_value(&tcc1, (enum tcc_match_capture_channel) (1), blue);
#endif
}

void setRightRGB(uint16_t red, uint16_t green, uint16_t blue) {
	RGB_Ouptut.RB = blue;
	RGB_Ouptut.RG = green;
	RGB_Ouptut.RR = red;
#if defined(HW_3v4) || defined(HW_4v0)
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (3), red);
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (1), green);
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (2), blue);
#endif
#if defined(HW_4v1)
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (2), red);
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (3), green);
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (0), blue);
#endif
}

void setWhite(uint16_t white) {
	head = white;
	tcc_set_compare_value(&tcc1, (enum tcc_match_capture_channel) (0), white);
}

void setRed(uint16_t red) {
	brake = red;
	tcc_set_compare_value(&tcc2, (enum tcc_match_capture_channel) (0), red);
}

void setAux(bool aux) {
	port_pin_set_output_level(HORN_PIN,!aux);
}

struct RGB_Vals setCycleColor(uint16_t _upColor, uint16_t _downColor, int _cycle){
	struct RGB_Vals color;
	if(_cycle == 0){
		color.LR=_upColor;
		color.LG = 0;
		color.LB = _downColor;
		color.RR=_upColor;
		color.RG = 0;
		color.RB = _downColor;
	}
	if(_cycle == 1){
		color.LR=_downColor;
		color.LG = _upColor;
		color.LB = 0;
		color.RR=_downColor;
		color.RG = _upColor;
		color.RB = 0;
	}
	if(_cycle == 2){
		color.LR=0;
		color.LG = _downColor;
		color.LB = _upColor;
		color.RR=0;
		color.RG = _downColor;
		color.RB = _upColor;
	}
	return color;
}

void setConstBases(){
	ColorBase[MODE_ANALOG_STATIC] = COLOR_STATIC;
	RateBase[MODE_ANALOG_STATIC] = RATE_STATIC;
	BrightBase[MODE_ANALOG_STATIC] = BRIGHT_STATIC;
	
	ColorBase[MODE_ANALOG_COLOR_CYCLE] = COLOR_COLOR_CYCLE;
	RateBase[MODE_ANALOG_COLOR_CYCLE] = RATE_STATIC;
	BrightBase[MODE_ANALOG_COLOR_CYCLE] = BRIGHT_STATIC;
	
	ColorBase[MODE_ANALOG_COMPASS_CYCLE] = COLOR_COMPASS;
	RateBase[MODE_ANALOG_COMPASS_CYCLE] = RATE_STATIC;
	BrightBase[MODE_ANALOG_COMPASS_CYCLE] = BRIGHT_STATIC;
	
	ColorBase[MODE_ANALOG_THROTTLE] = COLOR_THROTTLE;
	RateBase[MODE_ANALOG_THROTTLE] = RATE_STATIC;
	BrightBase[MODE_ANALOG_THROTTLE] = BRIGHT_STATIC;
	
	ColorBase[MODE_ANALOG_RPM_CYCLE] = COLOR_COLOR_CYCLE;
	RateBase[MODE_ANALOG_RPM_CYCLE] = RATE_RPM;
	BrightBase[MODE_ANALOG_RPM_CYCLE] = BRIGHT_RPM;
	
	ColorBase[MODE_ANALOG_RPM_THROTTLE] = COLOR_THROTTLE;
	RateBase[MODE_ANALOG_RPM_THROTTLE] = RATE_STATIC;
	BrightBase[MODE_ANALOG_RPM_THROTTLE] = BRIGHT_RPM;
	
	ColorBase[MODE_ANALOG_X_ACCEL] = COLOR_COLOR_CYCLE;
	RateBase[MODE_ANALOG_X_ACCEL] = RATE_STATIC;
	BrightBase[MODE_ANALOG_X_ACCEL] = BRIGHT_X_ACCEL;
	
	ColorBase[MODE_ANALOG_Y_ACCEL] = COLOR_Y_ACCEL;
	RateBase[MODE_ANALOG_Y_ACCEL] = RATE_STATIC;
	BrightBase[MODE_ANALOG_Y_ACCEL] = BRIGHT_STATIC;
}

// Flash the side LEDs red until restart
// 0: Red, 1: Blue, 2:Green, 3: Teal, 4: Yellow, 5:Purple
void ERROR_LEDs(uint8_t error_type){
	uint32_t timer = 0;

	uint16_t tempR = 0, tempG = 0, tempB = 0;
	if(error_type == 0 || error_type == 4  || error_type == 5)
		tempR = 0xFFFF;
	if(error_type >= 2 && error_type <= 4)
		tempG = 0xFFFF;
	if(error_type == 1 || error_type == 3 || error_type == 5)
		tempB = 0xFFFF;

	while(1){
		if(RGB_led_type == RGB_ANALOG){
			setLeftRGB(0,0,0);
			setRightRGB(0,0,0);
		} else {
			setDigitalHue(0,10,0,0,0);
			set_left_gnd();
			set_right_gnd();
			L_APA_write(led_num);
			R_APA_write(led_num);
	}
		
		setRed(0);
		setWhite(0);
#if  defined(HW_4v0) || defined(HW_4v1)
		port_pin_set_output_level(STAT_LED, false);
#endif

		while(millis() - timer < 1000) {
			///if(RGB_led_type != RGB_ANALOG) {
			//	L_APA_write(MAX_LEDCOUNT);
			//	R_APA_write(MAX_LEDCOUNT);
			//}
			check_time(&timer);
		}
		timer = millis();
		
		if(RGB_led_type == RGB_ANALOG){
			setLeftRGB(tempR,tempG,tempB);
			setRightRGB(tempR,tempG,tempB);
		} else {
			switch(error_type){
				case 2:
					setDigitalHue(0,15,0,31,0);// Green
					break;
				case 4:
					setDigitalHue(1910,15,0,31,0);// Yellow
					break;
				case 0:
					setDigitalHue(3820,15,0,31,0);// Red
					break;
				case 5:
					setDigitalHue(5730,15,0,31,0);// Purple
					break;
				case 1:
					setDigitalHue(7640,15,0,31,0);// Blue
					break;
				case 3:
					setDigitalHue(9550,15,0,31,0);// Teal
					break;
	}
		set_left_gnd();
		set_right_gnd();
		L_APA_write(led_num);
		R_APA_write(led_num);
		}
		
		setRed(0xFFFF);
		setWhite(0xFFFF);
#if  defined(HW_4v0) || defined(HW_4v1)
		port_pin_set_output_level(STAT_LED, true);
#endif

		while(millis() - timer < 250) {
			check_time(&timer);
		}
		timer = millis();
	}
}

// true = left    false = right
uint32_t turnTimer = 0;
uint16_t turnOutput = 0;
void TurnSignal(bool direction){

	check_time(&turnTimer);
	if(turnOutput == 0x0 && (millis() - turnTimer >= TURN_OFF_TIME)){
		turnOutput = 0xFFFF;
		turnTimer = millis();
	} else if(turnOutput == 0xFFFF && (millis() - turnTimer >= TURN_ON_TIME)){
		turnOutput = 0;
		turnTimer = millis();
	}

	if(direction == true){
		setLeftRGB(turnOutput,turnOutput,0);
		setRightRGB(0,0,0);
	} else {
		setLeftRGB(0,0,0);
		setRightRGB(turnOutput,turnOutput,0);
	}
}

//brightnes: 0 - 0xFFFF
//rate: 1-10
uint32_t blink_off_time = 0;
uint32_t blink_on_time = 0;
void BlinkTail(uint16_t brightness, float rate){
	blink_off_time = 300/(rate/3);
	blink_on_time = 100/(rate/3);
	static bool tail_on = false;
	static uint32_t timer = 0;
	check_time(&timer);
	if((tail_on && (millis()-timer) > blink_on_time) ||
		(!tail_on && (millis()-timer) > blink_off_time)){
		tail_on = !tail_on;
		timer = millis();
	}

	if(tail_on){
		setRed(brightness);
	}else{
		setRed(0);
	}
}

void AnalogSideLights(){
	// Variable for controlling the brightness of the side LEDS
	// brightness is a value from 0 to 1
	static float output_brightness = 0;

	// Variable for controlling the rate or sensitivity in applicable modes
	// brightness is a value from 0 to 1
	float output_rate_sens = 0;

	switch(RateBase[light_mode]){ // Set the value to be used for rate or sensitivity in the side LED algorithm
		case RATE_STATIC:
		{
			output_rate_sens = RateSens[light_mode];
			break;
		}
		case RATE_YAW_RATE:
		{
			if(gzKalman < 0)
			output_rate_sens = gzKalman/kalmanGZ_min;
			else
			output_rate_sens = gzKalman/kalmanGZ_max;
			break;
		}
		case RATE_ROLL_RATE:
		{
			if(gyKalman < 0)
			output_rate_sens = gyKalman/kalmanGY_min;
			else
			output_rate_sens = gyKalman/kalmanGY_max;
			break;
		}
		case RATE_RPM:
		{
			output_rate_sens = (((float)latest_vesc_vals.rpm)/mcconf_limits.max_erpm);
			break;
		}
		case RATE_THROTTLE:
		{
			output_rate_sens = remote_y/255.0;
		}
		break;
		case RATE_X_ACCEL:
		{
			if(axKalman < 0)
			output_rate_sens = axKalman/kalmanAX_min;
			else
			output_rate_sens = axKalman/kalmanAX_max;
			break;
		}
		case RATE_Y_ACCEL:
		{
			if(ayKalman < 0)
			output_rate_sens = ayKalman/kalmanAY_min;
			else
			output_rate_sens = ayKalman/kalmanAY_max;
			break;
		}
		case RATE_Z_ACCEL:
		{
			if(azKalman < 0)
			output_rate_sens = azKalman/kalmanAZ_min;
			else
			output_rate_sens = azKalman/kalmanAZ_max;
			break;
		}
	}
	
	if(output_rate_sens < 0)
	output_rate_sens = 0;
	else if(output_rate_sens > 1)
	output_rate_sens = 1;

	switch(BrightBase[light_mode]){ // Set the Brightness of the side LEDs
		case BRIGHT_STATIC:
		{
			output_brightness = Brightness[light_mode];
			break;
		}
		case BRIGHT_YAW_RATE:
		{
			if(gzKalman < 0)
			output_brightness = gzKalman/kalmanGZ_min;
			else
			output_brightness = gzKalman/kalmanGZ_max;
			break;
		}
		case BRIGHT_ROLL_RATE:
		{
			if(gyKalman < 0)
			output_brightness = gyKalman/kalmanGY_min;
			else
			output_brightness = gyKalman/kalmanGY_max;
			break;
		}
		case BRIGHT_RPM:
		{
			if(latest_vesc_vals.rpm != 0)
			output_brightness = ((float)abs(latest_vesc_vals.rpm))/(float)mcconf_limits.max_erpm;
			else
			output_brightness = 0;
			break;
		}
		case BRIGHT_THROTTLE:
		{
			output_brightness = remote_y/255.0;
			break;
		}
		case BRIGHT_X_ACCEL:
		{
			if(axKalman < 0){
				output_brightness = axKalman/kalmanAX_min;
				SUPRESS_RIGHT_RGB = true;
				} else{
				output_brightness = axKalman/kalmanAX_max;
				SUPRESS_LEFT_RGB = true;
			}
			break;
		}
		case BRIGHT_Y_ACCEL:
		{
			if(ayKalman < 0)
			output_brightness = ayKalman/kalmanAY_min;
			else
			output_brightness = ayKalman/kalmanAY_max;
			break;
		}
		case BRIGHT_Z_ACCEL:
		{
			if(azKalman < 0)
			output_brightness = azKalman/kalmanAZ_min;
			else
			output_brightness = azKalman/kalmanAZ_max;
			break;
		}
		case BRIGHT_STROBE:
		{
			check_time(&strobe_time);
			if(output_brightness == 0.0 && (millis()-strobe_time > strobe_off_dur)){
				output_brightness = 1.0;
				strobe_time = millis();
			}
			else if(output_brightness == 1.0 && (millis()-strobe_time > strobe_on_dur)){
				output_brightness = 0.0;
				strobe_time = millis();
			}
			break;
		}
	}
	
	if(output_brightness < 0)
	output_brightness = 0;
	else if(output_brightness > 1)
	output_brightness = 1;

	switch(ColorBase[light_mode]){ // Set the color of the side LEDs
		case COLOR_STATIC:
		{
			if(light_mode == MODE_ANALOG_STATIC)
			RGB_Ouptut = Static_RGB;
			else if(light_mode == MODE_ANALOG_CUSTOM)
			RGB_Ouptut = Custom_RGB;
			break;
		}
		case COLOR_COLOR_CYCLE:
		{
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;
			
			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);

			cycle_index += output_rate_sens*max_cycle_rate;
			if(cycle_index >= 0x0FFFF){
				cycle_index = 0;
				cycle += 1;
				if(cycle == 3)
				cycle = 0;
			}
			break;
		}
		case COLOR_COMPASS:
		{
			cycle_index = (int)(((((float)0x0FFFF) * 6) / 360) *heading) % 0x0FFFF;
			cycle = (int)(((((float)0x0FFFF) * 6) / 360) *heading) / 0x0FFFF;
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			if(cycle >= 3)
			cycle -= 3;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_YAW_RATE:
		{
			if(gzKalman < 0)
			cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanGZ_min) * gzKalman) % 0x0FFFF;
			cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanGZ_min) * gzKalman) / 0x0FFFF;
			if(gzKalman >= 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanGZ_max) * gzKalman) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanGZ_max) * gzKalman) / 0x0FFFF;
			}
			
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_ROLL_RATE:
		{
			if(gyKalman < 0)
			cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanGY_min) * gyKalman) % 0x0FFFF;
			cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanGY_min) * gyKalman) / 0x0FFFF;
			if(gyKalman >= 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanGY_max) * gyKalman) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanGY_max) * gyKalman) / 0x0FFFF;
			}
			
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_PITCH_RATE:
		{
			if(gxKalman < 0)
			cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanGX_min) * gxKalman) % 0x0FFFF;
			cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanGX_min) * gxKalman) / 0x0FFFF;
			if(gxKalman >= 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanGX_max) * gxKalman) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanGX_max) * gxKalman) / 0x0FFFF;
			}
			
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_THROTTLE:
		{
			cycle_index = (int)(((((float)0x0FFFF) * 2.0) / 256.0) * (255-remote_y)) % 0x0FFFF;
			cycle = (int)(((((float)0x0FFFF) * 2.0) / 256.0) * (255-remote_y)) / 0x0FFFF;
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			cycle = cycle+2;
			if(cycle > 2)
			cycle = cycle - 3;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);

			break;
		}
		case COLOR_RPM:
		{
			if(latest_vesc_vals.rpm != 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / (float)mcconf_limits.max_erpm) * (float)abs(latest_vesc_vals.rpm)) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / (float)mcconf_limits.max_erpm) * (float)abs(latest_vesc_vals.rpm)) / 0x0FFFF;
			}
			else{
				cycle_index = 0;
				cycle = 0;
			}
			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_X_ACCEL:
		{
			if(axKalman < 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (axKalman+1500)) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (axKalman+1500)) / 0x0FFFF;
				} else {
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (axKalman+1500)) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (axKalman+1500)) / 0x0FFFF;
			}

			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_Y_ACCEL:
		{
			if(ayKalman < 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (ayKalman+1500)) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (ayKalman+1500)) / 0x0FFFF;
				} else {
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (ayKalman+1500)) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / 3000) * (ayKalman+1500)) / 0x0FFFF;
			}

			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
		case COLOR_Z_ACCEL:
		{
			if(azKalman < 0){
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanAZ_min) * azKalman) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanAZ_min) * azKalman) / 0x0FFFF;
				} else {
				cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanAZ_max) * azKalman) % 0x0FFFF;
				cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanAZ_max) * azKalman) / 0x0FFFF;
			}

			upColor = cycle_index * output_brightness;
			downColor = (0xFFFF-cycle_index) * output_brightness;

			RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
			break;
		}
	}
	if(SUPRESS_LEFT_RGB){
		RGB_Ouptut.LR = 0;
		RGB_Ouptut.LG = 0;
		RGB_Ouptut.LB = 0;
		SUPRESS_LEFT_RGB = false;
	}
	if(SUPRESS_RIGHT_RGB){
		RGB_Ouptut.RR = 0;
		RGB_Ouptut.RG = 0;
		RGB_Ouptut.RB = 0;
		SUPRESS_RIGHT_RGB = false;
	}
	
	if(SYNC_RGB)
	setLeftRGB(RGB_Ouptut.LR,RGB_Ouptut.LG,RGB_Ouptut.LB);
	else
	setLeftRGB(RGB_Ouptut.LG, RGB_Ouptut.LB, RGB_Ouptut.LR);
	setRightRGB(RGB_Ouptut.RR,RGB_Ouptut.RG,RGB_Ouptut.RB);
}

void DigitalSideLights(){
	set_left_gnd();
	set_right_gnd();

	check_time(&digital_refresh_time);
	if((millis()-digital_refresh_time) > (1000/digital_refresh_rate)){
		if(led_num <= MAX_LEDCOUNT && led_num > 0){
			// Set the color frames
			switch(light_mode){
				case MODE_DIGITAL_STATIC:
					// Slider 1 controls gradient zoom
					// Slider 2 controls gradient position 
					// slider 3 controls brightness
					setDigitalHue(0, Digital_Static_Zoom, Digital_Static_Shift*(764/100)*Digital_Static_Zoom, (uint16_t)(Digital_Static_Brightness*(31.0/100.0)), false);
					break;
				case MODE_DIGITAL_SKITTLES:
				{
					static uint8_t skittles_refresh_counter = 0;
					skittles_refresh_counter++;
					if(digital_refresh_rate/skittles_refresh_counter <= 20){
						skittles_refresh_counter = 0;
						for(uint16_t i = 0; i < led_num; i++)
						{
							uint8_t weighted_index = (rand() / (RAND_MAX/6));
							L_SPI_send_buf[(i*4)+4] = R_SPI_send_buf[(i*4)+4] = (0b11100000 | (uint16_t)(Digital_Skittles_Brightness*(31.0/100.0)));
							switch(weighted_index){
								case 0:
									L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (rand() >> 31);
									L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (rand() >> 23);
									L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (rand() >> 23);
									break;
								case 1:
									L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (rand() >> 23);
									L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (rand() >> 31);
									L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (rand() >> 23);
									break;
								case 2:
									L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (rand() >> 23);
									L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (rand() >> 23);
									L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (rand() >> 31);
									break;
								case 3:
									L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (rand() >> 23);
									L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (rand() >> 27);
									L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (rand() >> 27);
									break;
								case 4:
									L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (rand() >> 27);
									L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (rand() >> 23);
									L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (rand() >> 27);
									break;
								case 5:
									L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (rand() >> 27);
									L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (rand() >> 27);
									L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (rand() >> 23);
									break;
							}
						}
					}
					break;
				}
				case MODE_DIGITAL_GRADIENT_CYCLE:
				{
					uint16_t x = (uint16_t)(millis()/((101.0-Digital_Cycle_Rate)/20.0))%(764*Digital_Cycle_Zoom);
					setDigitalHue(x, Digital_Cycle_Zoom, 0, (uint16_t)(Digital_Cycle_Brightness*(31.0/100.0)), false);
					break;
				}
				case MODE_DIGITAL_COMPASS_CYCLE:
				{
					uint8_t temp_zoom = 2;
					setDigitalHue((2*temp_zoom)*(heading/360.0)*(764*temp_zoom), temp_zoom, 0, (uint16_t)(Digital_Compass_Brightness*(31.0/100.0)), false);
					break;
				}
				case MODE_DIGITAL_THROTTLE:
				{
					setDigitalHue((remote_y)*(Digital_Throttle_Sens/10.0), Digital_Throttle_Zoom, Digital_Throttle_Shift*(764/100)*Digital_Throttle_Zoom, (uint16_t)(Digital_Throttle_Brightness*(31.0/100.0)), true);
					break;
				}
				case MODE_DIGITAL_RPM_CYCLE:
				{
					//Slider 1 rate
					//Slider 2 zoom
					static uint16_t pos = 0;
					pos = pos+(latest_vesc_vals.rpm*((1.0*Digital_RPM_Zoom)/(101.0-Digital_RPM_Rate)));
					if(pos >= (Digital_RPM_Zoom * 764))
						pos = 0;
					setDigitalHue(pos, Digital_RPM_Zoom, 0, (uint16_t)(Digital_RPM_Brightness*(31.0/100.0)), false);
					break;
				}
				case MODE_DIGITAL_RPM_THROTTLE:
				{
					// New color is chosen by throttle position
					// old colors get pushed back at a rate set by the RPM
					// brightness is set by slider
					static uint32_t shift_rate_timer = 0;
					if((millis()-shift_rate_timer) >= (100-(((float)latest_vesc_vals.rpm/mcconf_limits.max_erpm)*100))){
						shift_rate_timer = millis();
						memmove(L_SPI_send_buf+8,L_SPI_send_buf+4,(led_num-1)*4);
						memmove(R_SPI_send_buf+8,R_SPI_send_buf+4,(led_num-1)*4);
					}

					uint16_t x = (remote_y*3);

					L_SPI_send_buf[4] = R_SPI_send_buf[4] = (0b11100000 | brightness);
					if(x/255 == 0){
						L_SPI_send_buf[5] = R_SPI_send_buf[5] = 0;
						L_SPI_send_buf[6] = R_SPI_send_buf[6] = 255-(x%255);
						L_SPI_send_buf[7] = R_SPI_send_buf[7] = (x%255);
					} else if(x/255 == 1){
						L_SPI_send_buf[5] = R_SPI_send_buf[5] = (x%255);
						L_SPI_send_buf[6] = R_SPI_send_buf[6] = 0;
						L_SPI_send_buf[7] = R_SPI_send_buf[7] = 255-(x%255);
					} else if(x/255 == 2){
						L_SPI_send_buf[5] = R_SPI_send_buf[5] = 255-(x%255);
						L_SPI_send_buf[6] = R_SPI_send_buf[6] = (x%255);
						L_SPI_send_buf[7] = R_SPI_send_buf[7] = 0;
					}
					break;
				}
				case MODE_DIGITAL_COMPASS_WHEEL:
				{
					//use digital hue logic for each pixel
					//extend the gradient for further LEDs
					float zoom_delta = 0.2;
					float pos_delta = 50;
					if(led_num > 35){
						zoom_delta = 0.1;
						pos_delta = 25;
					}
						
					float temp_zoom = 1+(zoom_delta*(led_num/2));

					int x = (heading/360.0)*764*4;
					while(x<0)
						x += (764);
					while(x>(764))
						x -= (764);

					x = x-(pos_delta*(led_num/2));

					for(int i = 0; i < led_num; i++){
						setDigitalLEDHue(x, temp_zoom, brightness, i);
						if(i < (led_num/2)-1){
							x = x + pos_delta;
							temp_zoom = temp_zoom - zoom_delta;
						}
						if(i > (led_num/2)-1){
							x = x - pos_delta;
							temp_zoom = temp_zoom +zoom_delta;
						}
						while(x<0)
							x += (764);
						while(x>(764))
							x -= (764);
					}
					break;
				}
				case MODE_DIGITAL_COMPASS_SNAKE:
				{
					static uint32_t shift_rate_timer = 0;
					if((millis()-shift_rate_timer) >= 10){
						shift_rate_timer = millis();
						memmove(L_SPI_send_buf+8,L_SPI_send_buf+4,(led_num-1)*4);
						memmove(R_SPI_send_buf+8,R_SPI_send_buf+4,(led_num-1)*4);
					}

					int x = (heading/360.0)*764*4;
					while(x<0)
						x += (764);
					while(x>(764))
						x -= (764);

					L_SPI_send_buf[4] = R_SPI_send_buf[4] = (0b11100000 | brightness);
					if(x/255 == 0){
						L_SPI_send_buf[5] = R_SPI_send_buf[5] = 0;
						L_SPI_send_buf[6] = R_SPI_send_buf[6] = 255-(x%255);
						L_SPI_send_buf[7] = R_SPI_send_buf[7] = (x%255);
						} else if(x/255 == 1){
						L_SPI_send_buf[5] = R_SPI_send_buf[5] = (x%255);
						L_SPI_send_buf[6] = R_SPI_send_buf[6] = 0;
						L_SPI_send_buf[7] = R_SPI_send_buf[7] = 255-(x%255);
						} else if(x/255 == 2){
						L_SPI_send_buf[5] = R_SPI_send_buf[5] = 255-(x%255);
						L_SPI_send_buf[6] = R_SPI_send_buf[6] = (x%255);
						L_SPI_send_buf[7] = R_SPI_send_buf[7] = 0;
					}
					break;
				}
			}
		
			L_APA_write(led_num);
			R_APA_write(led_num);

			DIGITAL_OFF = false;
		}
		digital_refresh_time = millis();
	}
}

void BrakeLight(){
	if((HEADLIGHTS && lightControlHead() && LIGHTS_ON) | BRAKE_ALWAYS_ON){
		float temp_y = remote_y;
		float brake_temp;

		if(temp_y < 128-(128*((float)deadzone/100.0))){
			switch(brake_light_mode){
				case BRAKE_FADE:
				brake_temp = (((0xFFFF-brake_offset)/(128-(128*((float)deadzone/100.0)))*((128-(128*((float)deadzone/100.0)))-temp_y))+brake_offset);
				setRed(brake_temp);
				break;
				case BRAKE_BLINK:
				BlinkTail(0xFFFF, 7);
				break;
				case BRAKE_FADE_BLINK:
				brake_temp = (((0xFFFF-brake_offset)/(128-(128*((float)deadzone/100.0)))*((128-(128*((float)deadzone/100.0)))-temp_y))+brake_offset);
				if(temp_y > (128-(128*((float)deadzone/100.0)))*0.1)
				setRed(brake_temp);
				else
				BlinkTail(0xFFFF, 7);
				break;
				case BRAKE_BLINK_FADE:
				brake_temp = (((0xFFFF-brake_offset)/(128-(128*((float)deadzone/100.0)))*((128-(128*((float)deadzone/100.0)))-temp_y))+brake_offset);
				if(temp_y > (128-(128*((float)deadzone/100.0)))*0.1)
				BlinkTail(0xFFFF, 7);
				else
				setRed(brake_temp);
				break;
				case BRAKE_FADING_BLINK:
				brake_temp = (((0xFFFF-brake_offset)/(128-(128*((float)deadzone/100.0)))*((128-(128*((float)deadzone/100.0)))-temp_y))+brake_offset);
				BlinkTail(brake_temp, 7);
				break;
				case BRAKE_PACED_BLINK:
				brake_temp = (((0xFFFF-brake_offset)/(128-(128*((float)deadzone/100.0)))*((128-(128*((float)deadzone/100.0)))-temp_y))+brake_offset);
				BlinkTail(brake_temp, (3+(7/(128-(128*((float)deadzone/100.0))))*((128-(128*((float)deadzone/100.0)))-temp_y)));
				break;
			}
		}
		else {
			setRed(brake_offset);
		}
	}
	else{
		setRed(0);
	}
}

#define  BRIGHTS_TIMEOUT 600000 // 10min
void HeadLight(){
	static uint32_t brights_timer = 0;
	
	if(HEADLIGHTS && lightControlHead()){
		if(BRIGHTS_ENABLED){
			check_time(&brights_timer);
			if(BRIGHTS && (millis()-brights_timer > BRIGHTS_TIMEOUT)){
				BRIGHTS = false;
			}

			if(BRIGHTS) {
				setWhite(0xFFFF); // 100%
			} else {
				brights_timer = millis(); // reset the timer when low
				setWhite(0xFFFF*((float)lowbeam_level/100));
			}
		} else {
			setWhite(0xFFFF);
		}
	} else {
		setWhite(0);
	}
}


char lightControlSide() {
	// TO BE IMPLEMENTED

	return true;
}

char lightControlHead() {
	// TO BE IMPLEMENTED

	return true;
}

char sensorControl() {
	static uint8_t off_type = 0;
	static long count = 0;
	static bool result = 1;
	if(IMU_CONTROLED){
		if(result){
			if(ayKalman >= 1000 && result){
				count++;
				off_type = 1;
			}
			else if(ayKalman <= -1000 && result){
				count++;
				off_type = 2;
			}
			else if(axKalman >= 1250 && result){
				count++;
				off_type = 3;
			}
			else if(axKalman <= -1250 && result){
				count++;
				off_type = 4;
			}
			else
			count = 0;
		}
		else if(!result){
			if((ayKalman < 750 && off_type == 1) || (ayKalman > -750 && off_type == 2) || (axKalman < 1000 && off_type == 3) || (axKalman > -1000 && off_type == 4)){
				count++;
			}
			else
			count = 0;
		}
		
		if(count > 6)
		result = !result;

		if(result)
		off_type = 0;

		return result;
	}
	else
	return 1;
}


struct adc_module adc1;
void getLightSens(uint16_t* light_val) {
	static bool START_NEW_CONVERSION = true;
	if(START_NEW_CONVERSION){
		adc_start_conversion(&adc1);
		START_NEW_CONVERSION = false;
		} else if(adc_get_status(&adc1) != ADC_STATUS_RESULT_READY){
		adc_read(&adc1, light_val);
		adc_clear_status(&adc1, ADC_STATUS_RESULT_READY);
		START_NEW_CONVERSION = true;
	}
}

// Start: 0-764*zoom
// Zoom: 1-10
// Brightness: 1-31
void setDigitalHue(uint16_t start, uint8_t zoom, uint16_t offset, uint8_t hue_brightness, bool reverse_direction){
	int x = 0;
	for(uint16_t i = 0; i < led_num; i++)
	{
		//if(reverse_direction)
		//	start= -start+255*6;
		x = (start+offset) - (i*((764)/led_num));
		while(x<0)
			x += (764*zoom);
		while(x>(764*zoom))
			x -= (764*zoom);
			
		L_SPI_send_buf[(i*4)+4] = R_SPI_send_buf[(i*4)+4] = (0b11100000 | hue_brightness);
		if(x/(255*zoom) == 0){
			L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = 0;
			L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = 255-(x%(255*zoom))/zoom;
			L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = (x%(255*zoom))/zoom;
		} else if(x/(255*zoom) == 1){
			L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = (x%(255*zoom))/zoom;
			L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = 0;
			L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = 255-(x%(255*zoom))/zoom;
		} else if(x/(255*zoom) == 2){
			L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = 255-(x%(255*zoom))/zoom;
			L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = (x%(255*zoom))/zoom;
			L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = 0;
		}
	}
}

void setDigitalLEDHue(uint16_t pos, uint8_t zoom, uint8_t hue_brightness, uint8_t led){
	int x = 0;

	x = pos;
	
	L_SPI_send_buf[(led*4)+4] = R_SPI_send_buf[(led*4)+4] = (0b11100000 | hue_brightness);
	if(x/(255*zoom) == 0){
		L_SPI_send_buf[(led*4)+5] = R_SPI_send_buf[(led*4)+5] = 0;
		L_SPI_send_buf[(led*4)+6] = R_SPI_send_buf[(led*4)+6] = 255-(x%(255*zoom))/zoom;
		L_SPI_send_buf[(led*4)+7] = R_SPI_send_buf[(led*4)+7] = (x%(255*zoom))/zoom;
		} else if(x/(255*zoom) == 1){
		L_SPI_send_buf[(led*4)+5] = R_SPI_send_buf[(led*4)+5] = (x%(255*zoom))/zoom;
		L_SPI_send_buf[(led*4)+6] = R_SPI_send_buf[(led*4)+6] = 0;
		L_SPI_send_buf[(led*4)+7] = R_SPI_send_buf[(led*4)+7] = 255-(x%(255*zoom))/zoom;
		} else if(x/(255*zoom) == 2){
		L_SPI_send_buf[(led*4)+5] = R_SPI_send_buf[(led*4)+5] = 255-(x%(255*zoom))/zoom;
		L_SPI_send_buf[(led*4)+6] = R_SPI_send_buf[(led*4)+6] = (x%(255*zoom))/zoom;
		L_SPI_send_buf[(led*4)+7] = R_SPI_send_buf[(led*4)+7] = 0;
	}
}
#endif

