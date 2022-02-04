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

#ifndef REMOTEVARS_H
#define REMOTEVARS_H

enum REMOTE_TYPE{
	REMOTE_PPM = 0,
	REMOTE_UART,
	REMOTE_ADC,
	REMOTE_APP
};

uint8_t remote_type = 0;
uint8_t button_type = 0;
uint8_t deadzone = 10;

uint8_t remote_y = 0;
uint8_t remote_x = 0;
uint8_t remote_btn_state = 0;
uint8_t lremote_btn_state = 0;
uint8_t remote_btn_default_state = 0;

uint8_t DEFAULT_SET = false;

#endif
