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

#ifndef CONTROLS_H
#define CONTROLS_H

#include "VESC_UART.h"
#include "Remote_Vars.h"
#include "Timing.h"

////////////////   Control Variables   /////////////////
////////////////////////////////////////////////////////
uint8_t VescRemoteX = 0, VescRemoteY = 0;

#define PPM_IN PIN_PB02

bool AUX_ENABLED = false;
bool TURN_ENABLED = false;
bool AUX_OUTPUT = false;

uint8_t auxControlType = 0;
uint8_t auxTimedDuration = 0;
uint8_t single_aux_control = 0;
uint8_t single_all_control = 0;
uint8_t single_head_control = 0;
uint8_t single_side_control = 0;
uint8_t single_up_control = 0;
uint8_t single_down_control = 0;
uint8_t dual_aux_control = 0;
uint8_t dual_all_control = 0;
uint8_t dual_head_control = 0;
uint8_t dual_side_control = 0;
uint8_t dual_up_control = 0;
uint8_t dual_down_control = 0;

uint8_t AppAuxButton = 0;
uint8_t lAppAuxButton = 0;
uint8_t VescRemoteButton = 0;
uint8_t ButtonPressType = 0;
uint8_t REMOTE_TYPE = 1;
bool tapSequence = 0;
uint8_t tapIndex = 0;

uint32_t ButtonHeldTime = 0;
uint32_t ButtonDownTime = 0;
uint32_t ButtonUpTime = 0;
uint32_t lButtonTime = 0;

#define UP_THRESH 10
#define DWN_THRESH 10

bool TurnSignalOn = 0;
bool RestoreTurnLights = 0;

uint32_t AuxOnTime = 0;

enum BUTTON_TYPE{
	BTN_NONE = 0,
	BTN_MOMENTARY,
	BTN_LATCHED,
	BTN_LATCHED_PPM,
	BTN_UART_C,
	BTN_UART_Z,
	BTN_THROTTLE_DWN,
	BTN_THROTTLE_UP,
};

enum LED_CONTROLS{
	PRESS_NONE = 0,
	SINGLE_TAP,
	DOUBLE_TAP,
	TRIPLE_TAP,
	LEFT_TAP,
	RIGHT_TAP,
	MEDIUM_PRESS,
	LONG_PRESS,
	HOLD_0s5
};

enum AUX_CONTROLS{
	AUX_MOMENTARY = 0,
	AUX_TOGGLED,
	AUX_TIMED,
	AUX_PATTERN
};

// This function is used to handle all control inputs
// Input types include:	App commands
//						Button Presses from coms based remotes (Nunchuck, Firefly-TBI)
//						Button Presses from PPM remote
void HandleUserInput(void);
int get_pulse_width(void);



//uint16_t light_sens = 0; // for PWM debugging
int get_pulse_width() {
	return TCC1->CC[0].bit.CC;
}

int pulse_width_last = 0;
int pulse_width = 0;
void HandleUserInput()
{
	///////////////   Use the appropriate throttle input   ///////////////
	//////////////////////////////////////////////////////////////////////
	switch(remote_type){
		case REMOTE_PPM:
		case REMOTE_UART_PPM:{
			READ_VESC_PWM = true;
			float temp = (((float)latest_vesc_vals.pwm_val + 595000.0)*(255.0/1495000.0));
			//temp = -temp + 255;
			if(temp < 0)
				temp = 0;
			else if(temp > 255)
				temp = 255;
			remote_y = (uint8_t)(temp);
			break;}
		case REMOTE_UART_SINGLE:
		case REMOTE_UART_DUAL:
			READ_VESC_CHUCK = true;
			remote_y = rec_chuck_struct.js_y;
			if(remote_type == REMOTE_UART_DUAL)
				remote_x;
			else
				remote_x = 255/2;
			break;
		case REMOTE_APP:
			//if(app_remote_check && REMOTE_TYPE < 2)
				//temp_y = AppRemoteY;
			//else
				//temp_y = VescRemoteY;
			break;
	}


	////////////////   Use the appropriate button input   ////////////////
	//////////////////////////////////////////////////////////////////////
	switch(button_type){
		case BTN_MOMENTARY: 
		case BTN_LATCHED:
			remote_btn_state = port_pin_get_input_level(PPM_IN);
			break;
		case BTN_LATCHED_PPM:{
			pulse_width = get_pulse_width();
			if(pulse_width > 9000)
				remote_btn_state = false;
			else
				remote_btn_state = true;
			pulse_width_last = pulse_width;
			//light_sens = pulse_width; // for debugging pulse width reading
			}
			break;
		case BTN_UART_C:
			remote_btn_state = rec_chuck_struct.bt_c;
			break;	
		case BTN_UART_Z:
			remote_btn_state = rec_chuck_struct.bt_z;
			break;
		case BTN_THROTTLE_DWN:
			remote_btn_state = (remote_y < (127 - DWN_THRESH));
			break;
		case BTN_THROTTLE_UP:
			remote_btn_state = (remote_y > (127 + UP_THRESH));
			break;
		default:
			remote_btn_state = 0;
			break;
	}

	////   Determine the time the button was held down and released   ////
	//////////////////////////////////////////////////////////////////////
	if(button_type != BTN_LATCHED && button_type != BTN_LATCHED_PPM){
		if(remote_btn_state == 1 && lremote_btn_state == 0){
			lButtonTime = millis(); // Mark the time of button state transition
			ButtonUpTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 1){
			check_time(&lButtonTime);
			ButtonDownTime = millis() - lButtonTime;  // Track time button was pressed

			lButtonTime = millis();  // Mark the time of button state transition
			ButtonHeldTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 0){
			check_time(&lButtonTime);
			ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed

			ButtonDownTime = 0;
			TurnSignalOn = false;
		} else if(remote_btn_state == 1 && lremote_btn_state == 1){
			check_time(&lButtonTime);
			ButtonHeldTime = millis() - lButtonTime; // Track time button is not pressed
		}
	} else {
		ButtonHeldTime = 0;
		if(remote_btn_state == 1 && lremote_btn_state == 0){
			ButtonDownTime = 250;  // Track time button was pressed
			lButtonTime = millis(); // Mark the time of button state transition
			ButtonUpTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 1){
			ButtonDownTime = 250;  // Track time button was pressed
			lButtonTime = millis();  // Mark the time of button state transition
			ButtonUpTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 0){
			check_time(&lButtonTime);
			ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed

			ButtonDownTime = 0;
		} else if(remote_btn_state == 1 && lremote_btn_state == 1){
			check_time(&lButtonTime);
			ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed
			
			ButtonDownTime = 0;
		}
	}

	////////   Determine the type of button press that occurred   ////////
	//////////////////////////////////////////////////////////////////////
	ButtonPressType = PRESS_NONE;
	if(ButtonDownTime > 0 && ButtonDownTime < 500){ // Button Tap
		tapIndex++;
	} else if(ButtonDownTime > 500 && ButtonDownTime < 1000 && !TurnSignalOn){ // Medium Press
		ButtonPressType = MEDIUM_PRESS;
	} else if(ButtonDownTime >= 1000 && !TurnSignalOn){ // Long Press
		ButtonPressType = LONG_PRESS;
	}
	if(tapIndex > 0 && ButtonUpTime > 200){
		tapSequence = 1;
	}
	if(tapSequence){
		if(remote_type == REMOTE_UART_DUAL && VescRemoteX <= 110 && tapIndex == 1)
			ButtonPressType = LEFT_TAP;
		else if(remote_type == REMOTE_UART_DUAL && VescRemoteX >= 150 && tapIndex == 1)
		ButtonPressType = RIGHT_TAP;
		else if(tapIndex == 1)
			ButtonPressType = SINGLE_TAP;
		else if(tapIndex == 2)
			ButtonPressType = DOUBLE_TAP;
		else if(tapIndex == 3)
			ButtonPressType = TRIPLE_TAP;
		
		tapSequence = 0;
		tapIndex = 0;
	}
	
	//////////////////////   Handle the aux output   /////////////////////
	//////////////////////////////////////////////////////////////////////
	if(AUX_ENABLED){
		switch(auxControlType){
			case AUX_MOMENTARY:
				if(ButtonHeldTime > 500){
					AUX_OUTPUT = true;
				} else {
					AUX_OUTPUT = false;
				}
				break;
			case AUX_TOGGLED:
				if((remote_type != REMOTE_UART_DUAL && single_aux_control == ButtonPressType)
					|| (remote_type == REMOTE_UART_DUAL && dual_aux_control == ButtonPressType)) {
					AUX_OUTPUT = !AUX_OUTPUT;
				}
				break;
			case AUX_TIMED:
				if((remote_type != REMOTE_UART_DUAL && single_aux_control == ButtonPressType)
					|| (remote_type == REMOTE_UART_DUAL && dual_aux_control == ButtonPressType)) {
					AUX_OUTPUT = true;
					AuxOnTime = millis();
				}

				check_time(&AuxOnTime);
				if(AUX_OUTPUT == true && ((millis() - AuxOnTime) >= (auxTimedDuration * 100)))
					AUX_OUTPUT = false;
				break;
			case AUX_PATTERN:
				break;
		}

		if(AppAuxButton == 1 && lAppAuxButton == 0) {
			AUX_OUTPUT = true;
		} else if(AppAuxButton == 0 && lAppAuxButton == 1){
			AUX_OUTPUT = false;
		}
		lAppAuxButton = AppAuxButton;

		port_pin_set_output_level(AUX_PIN, !AUX_OUTPUT);
	}
	else{
		port_pin_set_output_level(AUX_PIN, true);
	}

	/////////////   Handle the side, head, and tail lights   /////////////
	//////////////////////////////////////////////////////////////////////
	if(ButtonPressType != PRESS_NONE){
		if(remote_type != REMOTE_UART_DUAL){ // If single axis remote is connected
			if(single_all_control == ButtonPressType){
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(single_head_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = true;
					SIDELIGHTS = false;
					LIGHTS_ON = true;
				}

				else if(SIDELIGHTS)
				HEADLIGHTS = !HEADLIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(single_side_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = false;
					SIDELIGHTS = true;
					LIGHTS_ON = true;
				}
				else if(HEADLIGHTS)
				SIDELIGHTS = !SIDELIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(single_down_control == ButtonPressType){
				if(light_mode == 0)
					light_mode = light_modes - 1;
				else
					light_mode--;
			}
			else if(single_up_control == ButtonPressType){
				light_mode++;
				if(light_mode >= light_modes)
					light_mode = 0;
			}
		}
		else if(remote_type == REMOTE_UART_DUAL){ // If dual axis remote is connected
			if(dual_all_control == ButtonPressType){
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(dual_head_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = true;
					SIDELIGHTS = false;
					LIGHTS_ON = true;
				}

				else if(SIDELIGHTS)
				HEADLIGHTS = !HEADLIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(dual_side_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = false;
					SIDELIGHTS = true;
					LIGHTS_ON = true;
				}
				else if(HEADLIGHTS)
				SIDELIGHTS = !SIDELIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(dual_down_control == ButtonPressType){
				if(light_mode == 0)
					light_mode = light_modes - 1;
				else
					light_mode--;
			}
			else if(dual_up_control == ButtonPressType){
				light_mode++;
				if(light_mode >= light_modes)
					light_mode = 0;
			}
			else if(TURN_ENABLED == 1 && ButtonHeldTime > 500){
				if(VescRemoteX < 110){
					if(SIDELIGHTS)
						RestoreTurnLights = true;
					SIDELIGHTS = false;
				
					TurnSignalOn = true;
					TurnSignal(true);
				}
				else if(VescRemoteX > 150){
					if(SIDELIGHTS)
						RestoreTurnLights = true;
					SIDELIGHTS = false;

					TurnSignalOn = true;
					TurnSignal(false);
				}
			}
			else if(RestoreTurnLights){
				SIDELIGHTS = true;
				RestoreTurnLights = false;
			}
		}
	}
	
	lremote_btn_state = remote_btn_state;
}

#endif