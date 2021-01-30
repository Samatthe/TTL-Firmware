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

#ifndef EEPROMFUNCS_H
#define EEPROMFUNCS_H
	
#include "Control_Vars.h"
#include "IMU_Functions.h"
#include "Remote_Vars.h"
#include "LED_Vars.h"

//EEPROM globals
uint8_t eeprom_data[EEPROM_PAGE_SIZE];

// Define functions
void save_led_data(void);
void restore_led_data(void);
void save_orientation_controls_remote_esc_lights(void);
void restore_orientation_controls_remote_esc_lights(void);
void save_cal_data(void);
void restore_cal_data(bool autoCal);


// EEPROM size needs to be set to 0x02


void save_led_data(){
	for(int i = 0; i < EEPROM_PAGE_SIZE; i++){
		eeprom_data[0] = 0;
	}

	eeprom_data[0] = SWITCHES;
	eeprom_data[1] = light_mode;
	eeprom_data[2] = (Static_RGB.LR & 0xFF);
	eeprom_data[3] = (Static_RGB.LR & 0xFF00) >> 8;
	eeprom_data[4] = (Static_RGB.LG & 0xFF);
	eeprom_data[5] = (Static_RGB.LG & 0xFF00) >> 8;
	eeprom_data[6] = (Static_RGB.LB & 0xFF);
	eeprom_data[7] = (Static_RGB.LB & 0xFF00) >> 8;
	eeprom_data[8] = (Static_RGB.RR & 0xFF);
	eeprom_data[9] = (Static_RGB.RR & 0xFF00) >> 8;
	eeprom_data[10] = (Static_RGB.RG & 0xFF);
	eeprom_data[11] = (Static_RGB.RG & 0xFF00) >> 8;
	eeprom_data[12] = (Static_RGB.RB & 0xFF);
	eeprom_data[13] = (Static_RGB.RB & 0xFF00) >> 8;
	int dataOffset = 14;
	for(int i = 0; i < 8; i++){
		eeprom_data[dataOffset + (i*5)] = ColorBase[i];
		eeprom_data[dataOffset + (i*5)+1] = BrightBase[i];
		eeprom_data[dataOffset + (i*5)+2] = RateBase[i];
		eeprom_data[dataOffset + (i*5)+3] = (uint8_t)(RateSens[i]*100);
		eeprom_data[dataOffset + (i*5)+4] = (uint8_t)(Brightness[i]*100);
	}

	eeprom_emulator_write_page(0, eeprom_data);
	eeprom_emulator_commit_page_buffer();

	
	for(int i = 0; i < EEPROM_PAGE_SIZE; i++){
		eeprom_data[0] = 0;
	}
	
	eeprom_data[0] = ColorBase[8];
	eeprom_data[1] = BrightBase[8];
	eeprom_data[2] = RateBase[8];
	eeprom_data[3] = (uint8_t)(RateSens[8]*100);
	eeprom_data[4] = (uint8_t)(Brightness[8]*100);
	eeprom_data[5] = (Custom_RGB.LR & 0xFF);
	eeprom_data[6] = (Custom_RGB.LR & 0xFF00) >> 8;
	eeprom_data[7] = (Custom_RGB.LG & 0xFF);
	eeprom_data[8] = (Custom_RGB.LG & 0xFF00) >> 8;
	eeprom_data[9] = (Custom_RGB.LB & 0xFF);
	eeprom_data[10] = (Custom_RGB.LB & 0xFF00) >> 8;
	eeprom_data[11] = (Custom_RGB.RR & 0xFF);
	eeprom_data[12] = (Custom_RGB.RR & 0xFF00) >> 8;
	eeprom_data[13] = (Custom_RGB.RG & 0xFF);
	eeprom_data[14] = (Custom_RGB.RG & 0xFF00) >> 8;
	eeprom_data[15] = (Custom_RGB.RB & 0xFF);
	eeprom_data[16] = (Custom_RGB.RB & 0xFF00) >> 8;

	eeprom_data[17] = Digital_Static_Zoom;
	eeprom_data[18] = Digital_Static_Shift;
	eeprom_data[19] = Digital_Static_Brightness;
	eeprom_data[20] = Digital_Skittles_Brightness;
	eeprom_data[21] = Digital_Cycle_Zoom;
	eeprom_data[22] = Digital_Cycle_Rate;
	eeprom_data[23] = Digital_Cycle_Brightness;
	eeprom_data[24] = Digital_Compass_Brightness;
	eeprom_data[25] = Digital_Throttle_Zoom;
	eeprom_data[26] = Digital_Throttle_Shift;
	eeprom_data[27] = Digital_Throttle_Sens;
	eeprom_data[28] = Digital_Throttle_Brightness;
	eeprom_data[29] = Digital_RPM_Zoom;
	eeprom_data[30] = Digital_RPM_Rate;
	eeprom_data[31] = Digital_RPM_Brightness;
	eeprom_data[32] = (shuffled_analog_modes & 0xFF);
	eeprom_data[33] = (shuffled_analog_modes & 0xFF00) >> 8;
	eeprom_data[34] = (shuffled_digital_modes & 0xFF);
	eeprom_data[35] = (shuffled_digital_modes & 0xFF00) >> 8;

	eeprom_emulator_write_page(1, eeprom_data);
	eeprom_emulator_commit_page_buffer();
}

void restore_led_data(){
	eeprom_emulator_read_page(0, eeprom_data);
	
	// If EEPROM has not been written, configure with default values then write them
	if(eeprom_data[0] == 0xFF && eeprom_data[1] == 0xFF) {
		SWITCHES = 0x30; // SIDE: on	HEAD: on	LIGHT: disable	IMU: disable
		SIDELIGHTS = (SWITCHES & 0x10) >> 4;
		HEADLIGHTS = (SWITCHES & 0x20) >> 5;
		LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
		IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
		light_mode = MODE_ANALOG_COLOR_CYCLE;

		Static_RGB.LR = 0;
		Static_RGB.LG = 0xFFFF;
		Static_RGB.LB = 0;
		Static_RGB.RR = 0;
		Static_RGB.RG = 0;
		Static_RGB.RB = 0xFFFF;

		
		uint8_t default_ColorBase[9] = {0,1,2,7,1,7,9,10,1};
		uint8_t default_BrightBase[9] = {255,0,0,0,3,3,0,0,2};
		uint8_t default_RateBase[9] = {255,0,255,0,0,255,0,0,3};
		float default_RateSens[9] = {0,0.75,0,0.5,0.5,0,0.5,0.5,0.5};
		float default_Brightness[9] = {0,0.25,0.5,0.5,0.5,0.5,0.5,0.5,0.5};

		for(int i = 0; i < 9; i++){
			ColorBase[i] = default_ColorBase[i];
			BrightBase[i] = default_BrightBase[i];
			RateBase[i] = default_RateBase[i];
			RateSens[i] = default_RateSens[i];
			Brightness[i] = default_Brightness[i];
		}

		Custom_RGB.LR = 0;
		Custom_RGB.LG = 0;
		Custom_RGB.LB = 0xFFFF;
		Custom_RGB.RR = 0;
		Custom_RGB.RG = 0xFFFF;
		Custom_RGB.RB = 0;

		Digital_Static_Zoom = 1;
		Digital_Static_Shift = 50;
		Digital_Static_Brightness = 50;
		Digital_Skittles_Brightness = 50;
		Digital_Cycle_Zoom = 1;
		Digital_Cycle_Rate = 50;
		Digital_Cycle_Brightness = 50;
		Digital_Compass_Brightness = 50;
		Digital_Throttle_Zoom = 1;
		Digital_Throttle_Shift = 50;
		Digital_Throttle_Sens = 50;
		Digital_Throttle_Brightness = 50;
		Digital_RPM_Zoom = 7;
		Digital_RPM_Rate = 50;
		Digital_RPM_Brightness = 50;
		shuffled_analog_modes = 0;
		shuffled_digital_modes = 0;

		save_led_data();
	}
	else{ // else restore the stored data
		SWITCHES = eeprom_data[0];
		SIDELIGHTS = (SWITCHES & 0x10) >> 4;
		HEADLIGHTS = (SWITCHES & 0x20) >> 5;
		LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
		IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
		light_mode = eeprom_data[1];
		Static_RGB.LR = eeprom_data[2];
		Static_RGB.LR = (Static_RGB.LR | (eeprom_data[3] << 8));
		Static_RGB.LG = eeprom_data[4];
		Static_RGB.LG = (Static_RGB.LG | (eeprom_data[5] << 8));
		Static_RGB.LB = eeprom_data[6];
		Static_RGB.LB = (Static_RGB.LB | (eeprom_data[7] << 8));
		Static_RGB.RR = eeprom_data[8];
		Static_RGB.RR = (Static_RGB.RR | (eeprom_data[9] << 8));
		Static_RGB.RG = eeprom_data[10];
		Static_RGB.RG = (Static_RGB.RG | (eeprom_data[11] << 8));
		Static_RGB.RB = eeprom_data[12];
		Static_RGB.RB = (Static_RGB.RB | (eeprom_data[13] << 8));
		int dataOffset = 14;
		for(int i = 0; i < 8; i++){
			ColorBase[i] = eeprom_data[dataOffset + (i*5)];
			BrightBase[i] = eeprom_data[dataOffset + (i*5) + 1];
			RateBase[i] = eeprom_data[dataOffset + (i*5) + 2];
			RateSens[i] = ((float)eeprom_data[dataOffset + (i*5) + 3])/100;
			Brightness[i] = ((float)eeprom_data[dataOffset + (i*5) + 4])/100;
		}

		eeprom_emulator_read_page(1, eeprom_data);

		ColorBase[8] = eeprom_data[0];
		BrightBase[8] = eeprom_data[1];
		RateBase[8] = eeprom_data[2];
		RateSens[8] = ((float)eeprom_data[3])/100;
		Brightness[8] = ((float)eeprom_data[4])/100;
		Custom_RGB.LR = eeprom_data[5];
		Custom_RGB.LR = (Custom_RGB.LR | (eeprom_data[6] << 8));
		Custom_RGB.LG = eeprom_data[7];
		Custom_RGB.LG = (Custom_RGB.LG | (eeprom_data[8] << 8));
		Custom_RGB.LB = eeprom_data[9];
		Custom_RGB.LB = (Custom_RGB.LB | (eeprom_data[10] << 8));
		Custom_RGB.RR = eeprom_data[11];
		Custom_RGB.RR = (Custom_RGB.RR | (eeprom_data[12] << 8));
		Custom_RGB.RG = eeprom_data[13];
		Custom_RGB.RG = (Custom_RGB.RG | (eeprom_data[14] << 8));
		Custom_RGB.RB = eeprom_data[15];
		Custom_RGB.RB = (Custom_RGB.RB | (eeprom_data[16] << 8));

		Digital_Static_Zoom = eeprom_data[17];
		Digital_Static_Shift = eeprom_data[18];
		Digital_Static_Brightness = eeprom_data[19];
		Digital_Skittles_Brightness = eeprom_data[20];
		Digital_Cycle_Zoom = eeprom_data[21];
		Digital_Cycle_Rate = eeprom_data[22];
		Digital_Cycle_Brightness = eeprom_data[23];
		Digital_Compass_Brightness = eeprom_data[24];
		Digital_Throttle_Zoom = eeprom_data[25];
		Digital_Throttle_Shift = eeprom_data[26];
		Digital_Throttle_Sens = eeprom_data[27];
		Digital_Throttle_Brightness = eeprom_data[28];
		Digital_RPM_Zoom = eeprom_data[29];
		Digital_RPM_Rate = eeprom_data[30];
		Digital_RPM_Brightness = eeprom_data[31];
		shuffled_analog_modes = eeprom_data[32];
		shuffled_analog_modes |= (eeprom_data[33] << 8);
		shuffled_digital_modes = eeprom_data[34];
		shuffled_digital_modes |= (eeprom_data[35] << 8);
	}
}

void save_cal_data()
{
	for(int i = 0; i < EEPROM_PAGE_SIZE; i++){
		eeprom_data[0] = 0;
	}

	eeprom_data[0] = ((gBiasRaw[0]) & 0xFF00) >> 8;
	eeprom_data[1] = ((gBiasRaw[0]) & 0xFF);
	eeprom_data[2] = ((gBiasRaw[1]) & 0xFF00) >> 8;
	eeprom_data[3] = ((gBiasRaw[1]) & 0xFF);
	eeprom_data[4] = ((gBiasRaw[2]) & 0xFF00) >> 8;
	eeprom_data[5] = ((gBiasRaw[2]) & 0xFF);
	eeprom_data[6] = ((aBiasRaw[0]) & 0xFF00) >> 8;
	eeprom_data[7] = ((aBiasRaw[0]) & 0xFF);
	eeprom_data[8] = ((aBiasRaw[1]) & 0xFF00) >> 8;
	eeprom_data[9] = ((aBiasRaw[1]) & 0xFF);
	eeprom_data[10] = ((aBiasRaw[2]) & 0xFF00) >> 8;
	eeprom_data[11] = ((aBiasRaw[2]) & 0xFF);
	eeprom_data[12] = ((mBiasRaw[0]) & 0xFF00) >> 8;
	eeprom_data[13] = ((mBiasRaw[0]) & 0xFF);
	eeprom_data[14] = ((mBiasRaw[1]) & 0xFF00) >> 8;
	eeprom_data[15] = ((mBiasRaw[1]) & 0xFF);
	eeprom_data[16] = ((mBiasRaw[2]) & 0xFF00) >> 8;
	eeprom_data[17] = ((mBiasRaw[2]) & 0xFF);

	//Write EEPROM data
	eeprom_emulator_write_page(2, eeprom_data);
	eeprom_emulator_commit_page_buffer();
}

void restore_cal_data(bool autoCal)
{
	eeprom_emulator_read_page(2, eeprom_data);
	int16_t temp = 0;

	// If EEPROM has not been written, configure with default values then write them
	if(eeprom_data[0] == 0xFF && eeprom_data[1] == 0xFF) {
		gBiasRaw[0] = 0;
		gBiasRaw[1] = 0;
		gBiasRaw[2] = 0;
		aBiasRaw[0] = 0;
		aBiasRaw[1] = 0;
		aBiasRaw[2] = 0;
		mBiasRaw[0] = 0;
		mBiasRaw[1] = 0;
		mBiasRaw[2] = 0;

		save_cal_data();
	}
	else{ // else restore the stored data
		gBiasRaw[0] = (temp | (eeprom_data[0] << 8) | eeprom_data[1]);
		gBiasRaw[1] = (temp | (eeprom_data[2] << 8) | eeprom_data[3]);
		gBiasRaw[2] = (temp | (eeprom_data[4] << 8) | eeprom_data[5]);
		aBiasRaw[0] = (temp | (eeprom_data[6] << 8) | eeprom_data[7]);
		aBiasRaw[1] = (temp | (eeprom_data[8] << 8) | eeprom_data[9]);
		aBiasRaw[2] = (temp | (eeprom_data[10] << 8) | eeprom_data[11]);
		mBiasRaw[0] = (temp | (eeprom_data[12] << 8) | eeprom_data[13]);
		mBiasRaw[1] = (temp | (eeprom_data[14] << 8) | eeprom_data[15]);
		mBiasRaw[2] = (temp | (eeprom_data[16] << 8) | eeprom_data[17]);
	}
	
	_autoCalc = autoCal;
}

void save_orientation_controls_remote_esc_lights()
{
	for(int i = 0; i < EEPROM_PAGE_SIZE; i++){
		eeprom_data[0] = 0;
	}

	eeprom_data[0] = ORIENTATION[0];
	eeprom_data[1] = ORIENTATION[1];

	eeprom_data[2] = ((BRIGHTS_ENABLED << 2) | (AUX_ENABLED << 1) | TURN_ENABLED);
	eeprom_data[3] = auxControlType;
	eeprom_data[4] = auxTimedDuration;
	eeprom_data[5] = single_aux_control;
	eeprom_data[6] = single_all_control;
	eeprom_data[7] = single_head_control;
	eeprom_data[8] = single_side_control;
	eeprom_data[9] = single_up_control;
	eeprom_data[10] = single_down_control;
	eeprom_data[11] = single_brights_control;
	eeprom_data[12] = lowbeam_level;
	/*eeprom_data[11] = dual_aux_control;
	eeprom_data[12] = dual_all_control;
	eeprom_data[13] = dual_head_control;
	eeprom_data[14] = dual_side_control;
	eeprom_data[15] = dual_up_control;
	eeprom_data[16] = dual_down_control;*/

	eeprom_data[17] = ((remote_type << 4) | (button_type & 0x0F));
	eeprom_data[18] = deadzone;
	
	eeprom_data[19] = esc_fw;
	eeprom_data[20] = ((esc_comms << 4) | (UART_baud & 0x0F));//*/
	
	eeprom_data[21] = (RGB_led_type << 4) | brake_light_mode;
	eeprom_data[22] = deadzone;
	eeprom_data[23] = led_num;
	eeprom_data[24] = (SYNC_RGB << 7) | (BRAKE_ALWAYS_ON << 6) | (DEFAULT_STATE << 5) | (STANDBY_ENABLED << 4) | (SHUFFLE_ENABLED << 3) | (AUTO_DETECT_ESC << 2);

	//Write EEPROM data
	eeprom_emulator_write_page(3, eeprom_data);
	eeprom_emulator_commit_page_buffer();
}

void restore_orientation_controls_remote_esc_lights()
{
	eeprom_emulator_read_page(3, eeprom_data);

	// If EEPROM has not been written, configure with default values then write them
	if(eeprom_data[0] == 0xFF && eeprom_data[1] == 0xFF) {
		ORIENTATION[0] = 1; // Connectors up
		ORIENTATION[1] = 6; // Power front

		BRIGHTS_ENABLED = false;
		AUX_ENABLED = false; // Aux disabled
		TURN_ENABLED = false; // Turn disabled
		auxControlType = AUX_MOMENTARY;
		auxTimedDuration = 10; // 1 second
		single_aux_control = PRESS_NONE;
		single_all_control = SINGLE_TAP;
		single_head_control = MEDIUM_PRESS;
		single_side_control = LONG_PRESS;
		single_up_control = DOUBLE_TAP;
		single_down_control = TRIPLE_TAP;
		single_brights_control = PRESS_NONE;
		lowbeam_level = 70;
		/*dual_aux_control = PRESS_NONE;
		dual_all_control = SINGLE_TAP;
		dual_head_control = MEDIUM_PRESS;
		dual_side_control = LONG_PRESS;
		dual_up_control =  RIGHT_TAP;
		dual_down_control = LEFT_TAP;*/

		remote_type = 0;
		deadzone = 10;
		button_type = 1;

		esc_fw = FW_3v7; // Set v3.7-v5.1 as the default FW to prevent bricking any ESCs
		esc_comms = COMMS_UART;
		UART_baud = BAUD_9600;

		RGB_led_type = RGB_ANALOG;
		brake_light_mode = BRAKE_FADE;
		deadzone = 20;
		led_num = 30;
		SYNC_RGB = true;
		BRAKE_ALWAYS_ON = false;
		DEFAULT_STATE = false;
		STANDBY_ENABLED = false;
		SHUFFLE_ENABLED = false;
		AUTO_DETECT_ESC = true;

		save_orientation_controls_remote_esc_lights();
	}
	else { // else restore the stored data
		ORIENTATION[0] = eeprom_data[0];
		ORIENTATION[1] = eeprom_data[1];
		
		BRIGHTS_ENABLED = (eeprom_data[2] & 0x04) >> 2;
		AUX_ENABLED = (eeprom_data[2] & 0x02) >> 1;
		TURN_ENABLED = (eeprom_data[2] & 0x01);
		auxControlType = eeprom_data[3];
		auxTimedDuration = eeprom_data[4];
		single_aux_control = eeprom_data[5];
		single_all_control = eeprom_data[6];
		single_head_control = eeprom_data[7];
		single_side_control = eeprom_data[8];
		single_up_control = eeprom_data[9];
		single_down_control = eeprom_data[10];
		single_brights_control = eeprom_data[11];
		lowbeam_level = eeprom_data[12];
		/*dual_aux_control = eeprom_data[11];
		dual_all_control = eeprom_data[12];
		dual_head_control = eeprom_data[13];
		dual_side_control = eeprom_data[14];
		dual_up_control = eeprom_data[15];
		dual_down_control = eeprom_data[16];*/

		remote_type = ((eeprom_data[17]&0xF0)>>4);
		button_type = (eeprom_data[17]&0x0F);
		deadzone = eeprom_data[18];

		esc_fw = eeprom_data[19];
		esc_comms = ((eeprom_data[20]&0xF0)>>4);
		UART_baud = (eeprom_data[20]&0x0F);//*/
		
		RGB_led_type = ((eeprom_data[21]&0xF0)>>4);
		brake_light_mode = (eeprom_data[21]&0x0F);
		deadzone = eeprom_data[22];
		led_num = eeprom_data[23];
		SYNC_RGB = ((eeprom_data[24]&0x80)>>7);
		BRAKE_ALWAYS_ON = ((eeprom_data[24]&0x40)>>6);
		DEFAULT_STATE = ((eeprom_data[24]&0x20)>>5);
		STANDBY_ENABLED = ((eeprom_data[24]&0x10)>>4);
		SHUFFLE_ENABLED = ((eeprom_data[24]&0x08)>>3);
		AUTO_DETECT_ESC = ((eeprom_data[24]&0x04)>>2);
	}
}

#endif