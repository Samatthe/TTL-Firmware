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
 
#ifndef ESCVARS_H
#define ESCVARS_H

enum ESC_FW{
	FW_3v6 = 0,
	FW_3v7_3v49,
	FW_3v50_5v2,
	FW_UNITY,
	FW_ACKMANIAC,
};

enum ESC_COMMS{
	COMMS_NONE = 0,
	COMMS_I2C,
	COMMS_UART,
};

enum UART_BAUD{
	BAUD_9600 = 0,
	BAUD_38400,
	BAUD_57600,
	BAUD_115200,
};

struct vesc_vals {
	int16_t temp_fet_filtered;
	//uint16_t temp_motor_filtered;
	int32_t avg_motor_current;
	int32_t avg_input_current;
	//uint32_t avg_id;
	//uint32_t avg_iq;
	int16_t duty_cycle;
	int32_t rpm;
	int16_t INPUT_VOLTAGE;
	int32_t amp_hours;
	int32_t amp_hours_charged;
	int32_t watt_hours;
	int32_t watt_hours_charged;
	int32_t tachometer_value;
	//uint32_t tachometer_abs_value;
	int8_t fault;
	//uint8_t pid_pos;
	//uint8_t controller_id;
	//uint8_t NTC_temp;
	int32_t pwm_val;
	uint8_t FW_VERSION_MAJOR;
	uint8_t FW_VERSION_MINOR;
};

uint8_t GET_LIMITS = 1;
uint8_t SEND_LIMITS = 0;
struct vesc_limits {
	int32_t motor_current_max;
	int32_t motor_current_min;
	int32_t input_current_max;
	int32_t input_current_min;
	int32_t abs_current_max;
	int32_t min_erpm;
	int32_t max_erpm;
	int32_t max_erpm_fbrake;
	int32_t max_erpm_fbrake_cc;
	int32_t min_vin;
	int32_t max_vin;
	float battery_cut_start;
	float battery_cut_end;
	int32_t temp_fet_start;
	int32_t temp_fet_end;
	int32_t temp_motor_start;
	int32_t temp_motor_end;
	int32_t min_duty;
	int32_t max_duty;
};
 
bool ESC_FW_READ = false;
uint8_t esc_fw = 1;
uint8_t esc_comms = 0;
uint8_t UART_baud = 0;

struct vesc_vals latest_vesc_vals;
struct vesc_limits mcconf_limits;
 
 #endif