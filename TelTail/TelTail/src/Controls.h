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
uint8_t RemoteButton = 0;
uint8_t lRemoteButton = 0;
uint8_t REMOTE_TYPE = 1;
bool tapSequence = 0;
uint8_t tapIndex = 0;

uint32_t ButtonHeldTime = 0;
uint32_t ButtonDownTime = 0;
uint32_t ButtonUpTime = 0;
uint32_t lButtonTime = 0;

bool TurnSignalOn = 0;
bool RestoreTurnLights = 0;

uint32_t AuxOnTime = 0;

typedef enum {
	NONE = 0,
	SINGLE_TAP = 1,
	DOUBLE_TAP = 2,
	TRIPLE_TAP = 3,
	LEFT_TAP = 4,
	RIGHT_TAP = 5,
	MEDIUM_PRESS = 6,
	LONG_PRESS = 7
} controls;

typedef enum {
	MOMENTARY = 0,
	TOGGLED = 1,
	TIMED = 2,
	PATTERN = 3
} aux_control_types;

// This function is used to handle all control inputs
// Input types include:	App commands
//						Button Presses from coms based remotes (Nunchuck, Firefly-TBI)
//						Button Presses from PPM remote
void HandleUserInput(void);




void HandleUserInput()
{
	////////////////   Use the appropriate button input   ////////////////
	//////////////////////////////////////////////////////////////////////
	if(REMOTE_TYPE != 0){
		if(REMOTE_TYPE == 1){
			RemoteButton = port_pin_get_input_level(PPM_IN);
		} else {
			RemoteButton = VescRemoteButton;
		}
	} else {
		RemoteButton = 0;
	}

	////   Determine the time the button was held down and released   ////
	//////////////////////////////////////////////////////////////////////
	if(RemoteButton == 1 && lRemoteButton == 0){
		lButtonTime = millis(); // Mark the time of button state transition
		ButtonUpTime = 0;
	} else if(RemoteButton == 0 && lRemoteButton == 1){
		if(lButtonTime > millis())
			lButtonTime = 0;
		ButtonDownTime = millis() - lButtonTime;  // Track time button was pressed

		lButtonTime = millis();  // Mark the time of button state transition
		ButtonHeldTime = 0;
	} else if(RemoteButton == 0 && lRemoteButton == 0){
		if(lButtonTime > millis())
			lButtonTime = 0;
		ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed

		ButtonDownTime = 0;
		TurnSignalOn = false;
	} else if(RemoteButton == 1 && lRemoteButton == 1){
		if(lButtonTime > millis())
			lButtonTime = 0;
		ButtonHeldTime = millis() - lButtonTime; // Track time button is not pressed
	}

	////////   Determine the type of button press that occurred   ////////
	//////////////////////////////////////////////////////////////////////
	ButtonPressType = NONE;
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
		if(REMOTE_TYPE == 2 && VescRemoteX <= 110 && tapIndex == 1)
			ButtonPressType = LEFT_TAP;
		else if(REMOTE_TYPE == 2 && VescRemoteX >= 150 && tapIndex == 1)
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
			case MOMENTARY:
				if(RemoteButton == 1){
					AUX_OUTPUT = true;
				} else {
					AUX_OUTPUT = false;
				}
				break;
			case TOGGLED:
				if((REMOTE_TYPE == 1 && single_aux_control == ButtonPressType)
					|| (REMOTE_TYPE == 2 && dual_aux_control == ButtonPressType)) {
					AUX_OUTPUT = !AUX_OUTPUT;
				}
				break;
			case TIMED:
				if((REMOTE_TYPE == 1 && single_aux_control == ButtonPressType)
					|| (REMOTE_TYPE == 2 && dual_aux_control == ButtonPressType)) {
					AUX_OUTPUT = true;
					AuxOnTime = millis();
				}

				if(AuxOnTime > millis())
					AuxOnTime = 0;
				if(AUX_OUTPUT == true && ((millis() - AuxOnTime) >= (auxTimedDuration * 100)))
					AUX_OUTPUT = false;
				break;
			case PATTERN:
				break;
		}

		if(AppAuxButton == 1 && lAppAuxButton == 0) {
			AUX_OUTPUT = true;
		} else if(AppAuxButton == 0 && lAppAuxButton == 1){
			AUX_OUTPUT = false;
		}
		lAppAuxButton = AppAuxButton;

		port_pin_set_output_level(AUX_PIN, AUX_OUTPUT);
	}
	else{
		port_pin_set_output_level(AUX_PIN, false);
	}

	/////////////   Handle the side, head, and tail lights   /////////////
	//////////////////////////////////////////////////////////////////////
	if(ButtonPressType != NONE){
		if(REMOTE_TYPE == 1){ // If single axis remote is connected
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
		else if(REMOTE_TYPE == 2){ // If dual axis remote is connected
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
	
	lRemoteButton = RemoteButton;
}

#endif