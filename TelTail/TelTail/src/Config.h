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

#ifndef CONFIG_H#define CONFIG_H/* Uncomment based on target HW version */#define HW_3v4//#define HW_4v0//#define HW_4v1/* Uncomment if target will be used in a summer board *///#define SBX/* Uncomment when first flashing a module to test the LED functions */// Comment when releasing//#define LED_Test
#ifdef HW_3v4
uint16_t HW_VER = 304;
#elif defined(HW_4v0)
uint16_t HW_VER = 400;
#elif defined(HW_4v1)
uint16_t HW_VER = 401;
#endif
uint16_t TTL_FW = 20; // Format: v12.34 = 1234 | v0.5 = 0005#define BOOT_BTN PIN_PA15#endif