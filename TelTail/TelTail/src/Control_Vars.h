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

#ifndef CNTRLVARS_H
#define CNTRLVARS_H

uint8_t VescRemoteX = 0, VescRemoteY = 0;

#define PPM_IN PIN_PB02

#define BUTTON_TAP_TIME 750
#define BUTTON_LONG_HOLD_TIME 1500

bool AUX_ENABLED = false;
bool TURN_ENABLED = false;
bool AUX_OUTPUT = false;
bool NEW_REMOTE_DATA = false;

uint8_t auxControlType = 0;
uint8_t auxTimedDuration = 0;
uint8_t single_aux_control = 0;
uint8_t single_all_control = 0;
uint8_t single_head_control = 0;
uint8_t single_side_control = 0;
uint8_t single_up_control = 0;
uint8_t single_down_control = 0;
uint8_t single_brights_control = 0;
/*uint8_t dual_aux_control = 0;
uint8_t dual_all_control = 0;
uint8_t dual_head_control = 0;
uint8_t dual_side_control = 0;
uint8_t dual_up_control = 0;
uint8_t dual_down_control = 0;*/

uint8_t AppAuxButton = 0;
uint8_t lAppAuxButton = 0;
uint8_t VescRemoteButton = 0;
uint8_t ButtonPressType = 0;
uint8_t REMOTE_TYPE = 1;
bool tapSequence = 0;
uint8_t tapIndex = 0;
bool AppControlled = false;
uint8_t AppRemoteY = 128;

uint32_t ButtonHeldTime = 0;
uint32_t ButtonDownTime = 0;
uint32_t ButtonUpTime = 0;
uint32_t lButtonTime = 0;

#define UP_THRESH 10
#define DWN_THRESH 10

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

#endif