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

#ifndef VESC_UART_H_
#define VESC_UART_H_

#include "crc.h"
#include "ESC_Vars.h"
#include <math.h>

#define VESC_UART_PERIOD 20

struct chuck_data{
	int8_t js_x;
	int8_t js_y;
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	bool bt_c;
	bool bt_z;
};

// Communication Command IDs
uint8_t COMM_FW_VERSION = 0;
uint8_t COMM_GET_VALUES = 0;
uint8_t COMM_GET_MCCONF = 0;
uint8_t COMM_ALIVE = 0;
uint8_t COMM_GET_DECODED_PPM = 0;
uint8_t COMM_GET_DECODED_CHUK = 0;
uint8_t COMM_SET_CHUCK_DATA = 0;

// Location in payload of COMM_GET_VALUES
uint8_t GET_VALUES_FET_TEMP = 0; 
uint8_t GET_VALUES_MTR_CURR = 0; 
uint8_t GET_VALUES_IN_CURR = 0; 
uint8_t GET_VALUES_DUTY = 0; 
uint8_t GET_VALUES_RPM = 0; 
uint8_t GET_VALUES_IN_VOLT = 0; 
uint8_t GET_VALUES_AH_USED = 0; 
uint8_t GET_VALUES_AH_CHRG = 0; 
uint8_t GET_VALUES_WH_USED = 0; 
uint8_t GET_VALUES_WH_CHRG = 0; 
uint8_t GET_VALUES_TACH = 0;
uint8_t GET_VALUES_FAULT = 0;

// Location in payload of COMM_GET_MCCONF
uint8_t GET_MCCONF_MTR_CURR_MAX = 0; 
uint8_t GET_MCCONF_MTR_CURR_MIN = 0; 
uint8_t GET_MCCONF_IN_CURR_MAX = 0; 
uint8_t GET_MCCONF_IN_CURR_MIN = 0; 
uint8_t GET_MCCONF_ABS_CURR_MAX = 0; 
uint8_t GET_MCCONF_ERPM_MIN = 0; 
uint8_t GET_MCCONF_ERPM_MAX = 0; 
uint8_t GET_MCCONF_ERPM_FBRAKE_MAX = 0; 
uint8_t GET_MCCONF_ERPM_FBRAKE_CC_MAX = 0; 
uint8_t GET_MCCONF_VIN_MIN = 0; 
uint8_t GET_MCCONF_VIN_MAX = 0; 
uint8_t GET_MCCONF_BAT_CUT_STRT = 0; 
uint8_t GET_MCCONF_BAT_CUT_END = 0; 
uint8_t GET_MCCONF_TMP_FET_STRT = 0; 
uint8_t GET_MCCONF_TMP_FET_END = 0; 
uint8_t GET_MCCONF_TMP_MTR_STRT = 0; 
uint8_t GET_MCCONF_TMP_MTR_END = 0; 
uint8_t GET_MCCONF_DUTY_MIN = 0; 
uint8_t GET_MCCONF_DUTY_MAX = 0; 

#define MAX_PAYLOAD_LEN 512
struct uart_packet{
	uint8_t start; //default is a short packet
	uint8_t len[2]; //second byte needs to be skipped if sending/recieving a short packet
	uint8_t payload[MAX_PAYLOAD_LEN];
	uint8_t crc[2];
	uint8_t stop;
};


#define VESC_USART_READ_DATA_LENGTH 512
uint8_t vesc_USART_read_buffer[MAX_PAYLOAD_LEN+6];
struct uart_packet vesc_revieve_packet;
struct usart_module vesc_usart;
uint32_t vesc_usart_time = 0;
uint32_t vesc_usart_delay = 2;
bool HOLD_FOR_REPLY = false;
uint32_t vesc_usart_timeout = 100;
struct chuck_data send_chuck_struct;
struct chuck_data rec_chuck_struct;
bool READ_VESC_PWM = false;
bool READ_VESC_CHUCK = false;
bool SEND_VESC_CHUCK = false;
bool READ_VESC_VALS = false;
bool READ_VESC_FW = false;
bool VESC_PACKET_RECIEVED = false;
bool ESC_UART_CONFIGED = false;
bool ESC_UART_DETECTED = false;
bool ESC_UART_PIN_CONFIG = false;
bool AUTO_DETECT_ESC = true;

///////////   ESC Communication Variables   ///////////
////////////////////////////////////////////////////////
uint8_t configured_comms = COMMS_NONE;
uint32_t ESC_noise_timer = 0;

// Set the ESC RX/TX pins according to HW version
#define ESC_UART_RXTX_1 PIN_PA16
#ifdef HW_3v4
#define ESC_UART_RXTX_2 PIN_PA17
#endif
#if  defined(HW_4v0)|| defined(HW_4v1)
#define ESC_UART_RXTX_2 PIN_PA18
#endif

void configure_vesc_usart(void);
void vesc_usart_read_callback(struct usart_module *const usart_module);
void configure_vesc_usart_callbacks(void);
void vesc_get_fw_version(void);
void vesc_get_vals(void);
void vesc_get_mcconf(void);
void vesc_send_alive(void);
void vesc_get_pwm(void);
void vesc_get_chuck(void);
void vesc_set_chuck(void);
void vesc_get_imu(void);
void send_packet(struct uart_packet send_pak);
void process_recieved_packet(void);
void vesc_read_all(void);
void detect_vesc_firmware(void);
struct uart_packet recieve_packet(void);
bool CHECK_BUFFER(uint8_t *buf);
void read_vesc_packet(void);
void detect_esc_baud_pins(void);
void detect_esc_uart_connected(void);
bool CHECK_FOR_NOISE_ESC(struct usart_module *const module, uint8_t buf[MAX_PAYLOAD_LEN+6], uint16_t max_size, uint32_t *noise_timer);

float buffer_get_float32_auto(uint8_t *buffer, int8_t index);


// Configure SERCOM5 as USART for VESC
void configure_vesc_usart()
{
	uint32_t baud = 0;
	if(UART_baud == BAUD_9600)
		baud = 9600;
	else if(UART_baud == BAUD_38400)
		baud = 38400;
	else if(UART_baud == BAUD_57600)
		baud = 57600;
	else if(UART_baud == BAUD_115200)
		baud = 115200;

	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = baud;
#ifdef HW_3v4
	config_usart.mux_setting = USART_RX_1_TX_0_XCK_1;
	config_usart.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	config_usart.pinmux_pad1 = PINMUX_PA17C_SERCOM1_PAD1;
	config_usart.pinmux_pad2 = PINMUX_UNUSED;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
#endif
#if  defined(HW_4v0)|| defined(HW_4v1)
	if(ESC_UART_PIN_CONFIG)
		config_usart.mux_setting = USART_RX_2_TX_0_XCK_1;
	else
		config_usart.mux_setting = USART_RX_0_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PA18C_SERCOM1_PAD2;
	config_usart.pinmux_pad3 = PINMUX_UNUSED;
#endif
	while (usart_init(&vesc_usart,SERCOM1, &config_usart) != STATUS_OK)
	{}
	usart_enable(&vesc_usart);

	latest_vesc_vals.FW_VERSION_MAJOR = 0;
	latest_vesc_vals.FW_VERSION_MINOR = 0;
}

int vesc_usart_count = 0;
uint8_t len_bytes = 0;
uint16_t packet_len = 0;
bool corrupted = false;
uint8_t vesc_uart_expected_bytes = 0;
enum VESC_UART_BYTES{
	VESC_UART_BYTES_START = 0,
	VESC_UART_BYTES_LEN,
	VESC_UART_BYTES_PAYLOAD,
	VESC_UART_BYTES_CRC,
	VESC_UART_BYTES_STOP,
};

uint8_t vesc_tx_buff[MAX_PAYLOAD_LEN+6];
void send_packet(struct uart_packet send_pak){
	if(check_timer_expired(&vesc_usart_time, vesc_usart_timeout))
		HOLD_FOR_REPLY = false;

	send_pak.stop = 0x03;

	if(!HOLD_FOR_REPLY){
		uint16_t payload_len = 0;
		if(send_pak.start == 0x03)
			payload_len = ((send_pak.len[0]<<8)|send_pak.len[1]);
		else
			payload_len = send_pak.len[0];
		
		uint16_t message_len = (send_pak.start+payload_len+3);
		uint8_t *send_ptr = &(send_pak.start);

		for(size_t i=0;i<message_len;i++){
			if(send_pak.start==0x02 && i==2)
			send_ptr+=1;
			else if(i==payload_len+send_pak.start)
			send_ptr+=(MAX_PAYLOAD_LEN-payload_len);

			vesc_tx_buff[i] = *send_ptr;

			send_ptr++;
		}
		
		HOLD_FOR_REPLY = true;
		usart_write_buffer_job(&vesc_usart, vesc_tx_buff, message_len);
		vesc_usart_time = millis();
	}
}

void process_recieved_packet(){
	uint8_t packet_id = vesc_revieve_packet.payload[0];
	if(packet_id == COMM_FW_VERSION){ // Bytes are the same for all FW's
		latest_vesc_vals.FW_VERSION_MAJOR = (uint16_t)vesc_revieve_packet.payload[1];
		latest_vesc_vals.FW_VERSION_MINOR = (uint16_t)vesc_revieve_packet.payload[2];
	} else if(packet_id == COMM_GET_VALUES){
		latest_vesc_vals.temp_fet_filtered = (vesc_revieve_packet.payload[GET_VALUES_FET_TEMP] << 8) | vesc_revieve_packet.payload[GET_VALUES_FET_TEMP+1];
		latest_vesc_vals.avg_motor_current = (vesc_revieve_packet.payload[GET_VALUES_MTR_CURR] << 24) | (vesc_revieve_packet.payload[GET_VALUES_MTR_CURR+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_MTR_CURR+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_MTR_CURR+3];
		latest_vesc_vals.avg_input_current = (vesc_revieve_packet.payload[GET_VALUES_IN_CURR] << 24) | (vesc_revieve_packet.payload[GET_VALUES_IN_CURR+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_IN_CURR+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_IN_CURR+3];
		latest_vesc_vals.duty_cycle = (vesc_revieve_packet.payload[GET_VALUES_DUTY] << 8) | vesc_revieve_packet.payload[GET_VALUES_DUTY+1];
		latest_vesc_vals.rpm = (vesc_revieve_packet.payload[GET_VALUES_RPM] << 24) | (vesc_revieve_packet.payload[GET_VALUES_RPM+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_RPM+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_RPM+3];
		latest_vesc_vals.INPUT_VOLTAGE = (vesc_revieve_packet.payload[GET_VALUES_IN_VOLT] << 8) | vesc_revieve_packet.payload[GET_VALUES_IN_VOLT+1];
		latest_vesc_vals.amp_hours = ((vesc_revieve_packet.payload[GET_VALUES_AH_USED] << 24) | (vesc_revieve_packet.payload[GET_VALUES_AH_USED+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_AH_USED+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_AH_USED+3])/100;
		latest_vesc_vals.amp_hours_charged = ((vesc_revieve_packet.payload[GET_VALUES_AH_CHRG] << 24) | (vesc_revieve_packet.payload[GET_VALUES_AH_CHRG+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_AH_CHRG+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_AH_CHRG+3])/100;
		latest_vesc_vals.watt_hours = ((vesc_revieve_packet.payload[GET_VALUES_WH_USED] << 24) | (vesc_revieve_packet.payload[GET_VALUES_WH_USED+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_WH_USED+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_WH_USED+3])/100;
		latest_vesc_vals.watt_hours_charged = ((vesc_revieve_packet.payload[GET_VALUES_WH_CHRG] << 24) | (vesc_revieve_packet.payload[GET_VALUES_WH_CHRG+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_WH_CHRG+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_WH_CHRG+3])/100;
		latest_vesc_vals.tachometer_value = (vesc_revieve_packet.payload[GET_VALUES_TACH] << 24) | (vesc_revieve_packet.payload[GET_VALUES_TACH+1] << 16) | (vesc_revieve_packet.payload[GET_VALUES_TACH+2] << 8) | vesc_revieve_packet.payload[GET_VALUES_TACH+2];
		latest_vesc_vals.fault = vesc_revieve_packet.payload[GET_VALUES_FAULT];
	} else if(packet_id == COMM_GET_MCCONF){
	//port_pin_set_output_level(STAT_LED,true);
		if(esc_fw == FW_3v6){
			mcconf_limits.motor_current_max = ((vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MAX+3])/1000;
			mcconf_limits.motor_current_min = ((vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MIN] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MIN+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MIN+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_MTR_CURR_MIN+3])/1000;
			mcconf_limits.input_current_max = ((vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MAX+3])/1000;
			mcconf_limits.input_current_min = ((vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MIN] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MIN+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MIN+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_IN_CURR_MIN+3])/1000;
			mcconf_limits.abs_current_max = ((vesc_revieve_packet.payload[GET_MCCONF_ABS_CURR_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_ABS_CURR_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_ABS_CURR_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_ABS_CURR_MAX+3])/1000;
			mcconf_limits.min_erpm = ((vesc_revieve_packet.payload[GET_MCCONF_ERPM_MIN] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_MIN+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_MIN+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_ERPM_MIN+3])/1000;
			mcconf_limits.max_erpm = ((vesc_revieve_packet.payload[GET_MCCONF_ERPM_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_ERPM_MAX+3])/1000;
			mcconf_limits.max_erpm_fbrake = ((vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_MAX+3])/1000;
			mcconf_limits.max_erpm_fbrake_cc = ((vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_CC_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_CC_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_CC_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_ERPM_FBRAKE_CC_MAX+3])/1000;
			mcconf_limits.min_vin = ((vesc_revieve_packet.payload[GET_MCCONF_VIN_MIN] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_VIN_MIN+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_VIN_MIN+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_VIN_MIN+3])/1000;
			mcconf_limits.max_vin = ((vesc_revieve_packet.payload[GET_MCCONF_VIN_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_VIN_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_VIN_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_VIN_MAX+3])/1000;
			mcconf_limits.battery_cut_start = ((vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_STRT] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_STRT+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_STRT+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_STRT+3])/1000;
			mcconf_limits.battery_cut_end = ((vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_END] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_END+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_END+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_BAT_CUT_END+3])/1000;
			mcconf_limits.temp_fet_start = ((vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_STRT] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_STRT+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_STRT+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_STRT+3])/1000;
			mcconf_limits.temp_fet_end = ((vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_END] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_END+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_END+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_TMP_FET_END+3])/1000;
			mcconf_limits.temp_motor_start = ((vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_STRT] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_STRT+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_STRT+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_STRT+2])/1000;
			mcconf_limits.temp_motor_end = ((vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_END] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_END+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_END+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_TMP_MTR_END+3])/1000;
			mcconf_limits.min_duty = ((vesc_revieve_packet.payload[GET_MCCONF_DUTY_MIN] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_DUTY_MIN+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_DUTY_MIN+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_DUTY_MIN+3])/1000;
			mcconf_limits.max_duty = ((vesc_revieve_packet.payload[GET_MCCONF_DUTY_MAX] << 24) | (vesc_revieve_packet.payload[GET_MCCONF_DUTY_MAX+1] << 16) | (vesc_revieve_packet.payload[GET_MCCONF_DUTY_MAX+2] << 8) | vesc_revieve_packet.payload[GET_MCCONF_DUTY_MAX+3])/1000;
		} else if (esc_fw == FW_3v7_3v49 || esc_fw == FW_3v50_5v2 || esc_fw == FW_UNITY || esc_fw == FW_ACKMANIAC){
			mcconf_limits.motor_current_max = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_MTR_CURR_MAX);
			mcconf_limits.motor_current_min = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_MTR_CURR_MIN);
			mcconf_limits.input_current_max = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_IN_CURR_MAX);
			mcconf_limits.input_current_min = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_IN_CURR_MIN);
			mcconf_limits.abs_current_max = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_ABS_CURR_MAX);
			mcconf_limits.min_erpm = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_ERPM_MIN);
			mcconf_limits.max_erpm = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_ERPM_MAX);
			mcconf_limits.max_erpm_fbrake = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_ERPM_FBRAKE_MAX);
			mcconf_limits.max_erpm_fbrake_cc = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_ERPM_FBRAKE_CC_MAX);
			mcconf_limits.min_vin = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_VIN_MIN);
			mcconf_limits.max_vin = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_VIN_MAX);
			mcconf_limits.battery_cut_start = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_BAT_CUT_STRT);
			mcconf_limits.battery_cut_end = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_BAT_CUT_END);
			mcconf_limits.temp_fet_start = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_TMP_FET_STRT);
			mcconf_limits.temp_fet_end = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_TMP_FET_END);
			mcconf_limits.temp_motor_start = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_TMP_MTR_STRT);
			mcconf_limits.temp_motor_end = buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_TMP_MTR_END);
			mcconf_limits.min_duty = (buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_DUTY_MIN)*100);
			mcconf_limits.max_duty = (buffer_get_float32_auto(vesc_revieve_packet.payload, GET_MCCONF_DUTY_MAX)*100);
		}
		//if(GET_LIMITS)
		SEND_LIMITS = 1;
		GET_LIMITS = 0;
	} else if(packet_id == COMM_GET_DECODED_PPM){
		latest_vesc_vals.pwm_val = (int32_t)(((vesc_revieve_packet.payload[1]&0x00FF)<<24)|((vesc_revieve_packet.payload[2]&0x00FF)<<16)|((vesc_revieve_packet.payload[3]&0x00FF)<<8)|(vesc_revieve_packet.payload[4]&0x00FF));
	} else if(packet_id == COMM_GET_DECODED_CHUK){
		//if(button_type == BTN_UART_C){
			rec_chuck_struct.bt_c = ((((vesc_revieve_packet.payload[1] & 0x000000FF) << 24)|((vesc_revieve_packet.payload[2] & 0x000000FF) << 16)|((vesc_revieve_packet.payload[3] & 0x000000FF) << 8)|(vesc_revieve_packet.payload[4] & 0x000000FF)) == 0xFFF0DC45); //-992187
			rec_chuck_struct.bt_z = rec_chuck_struct.bt_c;
		//}
		//rec_chuck_struct.js_x = vesc_revieve_packet.payload[1];
		//remote_y = vesc_revieve_packet.payload[2];
		//rec_chuck_struct.bt_c = vesc_revieve_packet.payload[3];
		//rec_chuck_struct.bt_z = vesc_revieve_packet.payload[4];
		//rec_chuck_struct.acc_x = (int16_t)(((vesc_revieve_packet.payload[5] & 0x00FF) << 8)|(vesc_revieve_packet.payload[6] & 0x00FF));
		//rec_chuck_struct.acc_y = (int16_t)(((vesc_revieve_packet.payload[7] & 0x00FF) << 8)|(vesc_revieve_packet.payload[8] & 0x00FF));
		//rec_chuck_struct.acc_z = (int16_t)(((vesc_revieve_packet.payload[9] & 0x00FF) << 8)|(vesc_revieve_packet.payload[10] & 0x00FF));
	}/* else if(packet_id == COMM_GET_VALUES_SELECTIVE){ // Only available in latest Official FW
		latest_vesc_vals.temp_fet_filtered = (vesc_revieve_packet.payload[5] << 8) | vesc_revieve_packet.payload[6];
		latest_vesc_vals.avg_motor_current = (vesc_revieve_packet.payload[7] << 24) | (vesc_revieve_packet.payload[8] << 16) | (vesc_revieve_packet.payload[9] << 8) | vesc_revieve_packet.payload[10];
		latest_vesc_vals.avg_input_current = (vesc_revieve_packet.payload[11] << 24) | (vesc_revieve_packet.payload[12] << 16) | (vesc_revieve_packet.payload[13] << 8) | vesc_revieve_packet.payload[14];
		latest_vesc_vals.duty_cycle = (vesc_revieve_packet.payload[15] << 8) | vesc_revieve_packet.payload[16];
		latest_vesc_vals.rpm = (vesc_revieve_packet.payload[17] << 24) | (vesc_revieve_packet.payload[18] << 16) | (vesc_revieve_packet.payload[19] << 8) | vesc_revieve_packet.payload[20];
		latest_vesc_vals.INPUT_VOLTAGE = (vesc_revieve_packet.payload[21] << 8) | vesc_revieve_packet.payload[22];
		latest_vesc_vals.amp_hours = ((vesc_revieve_packet.payload[23] << 24) | (vesc_revieve_packet.payload[24] << 16) | (vesc_revieve_packet.payload[25] << 8) | vesc_revieve_packet.payload[26])*10;
		latest_vesc_vals.amp_hours_charged = ((vesc_revieve_packet.payload[27] << 24) | (vesc_revieve_packet.payload[28] << 16) | (vesc_revieve_packet.payload[29] << 8) | vesc_revieve_packet.payload[30])*10;
		latest_vesc_vals.watt_hours = ((vesc_revieve_packet.payload[31] << 24) | (vesc_revieve_packet.payload[32] << 16) | (vesc_revieve_packet.payload[33] << 8) | vesc_revieve_packet.payload[34])/100;
		latest_vesc_vals.watt_hours_charged = ((vesc_revieve_packet.payload[35] << 24) | (vesc_revieve_packet.payload[36] << 16) | (vesc_revieve_packet.payload[37] << 8) | vesc_revieve_packet.payload[38])/100;
		latest_vesc_vals.tachometer_value = (vesc_revieve_packet.payload[39] << 24) | (vesc_revieve_packet.payload[40] << 16) | (vesc_revieve_packet.payload[41] << 8) | vesc_revieve_packet.payload[42];
		latest_vesc_vals.fault = vesc_revieve_packet.payload[43];
	} else if(packet_id == COMM_GET_IMU_DATA){ 
		// TODO
	}*/
}

float buffer_get_float32_auto(uint8_t *buffer, int8_t index) {
	uint32_t res = ((uint32_t) buffer[index]) << 24 | ((uint32_t) buffer[index+1]) << 16 | ((uint32_t) buffer[index+2]) << 8 | ((uint32_t) buffer[index+3]);

	int e = (res >> 23) & 0xFF;
	uint32_t sig_i = res & 0x7FFFFF;
	bool neg = res & (1U << 31);

	float sig = 0.0;
	if (e != 0 || sig_i != 0) {
		sig = (float)sig_i / (8388608.0 * 2.0) + 0.5;
		e -= 126;
	}

	if (neg) {
		sig = -sig;
	}
	
	return ldexpf(sig, e);
}


void vesc_get_fw_version(){
	struct uart_packet send_pack;

	send_pack.start = 0x02;
	send_pack.len[0] = 0x01;
	send_pack.payload[0] = COMM_FW_VERSION;
	uint16_t crc = crc16(send_pack.payload, 1);
	send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
	send_pack.crc[1] = (uint8_t)(crc&0x00FF);

	send_packet(send_pack);
}

void vesc_get_vals(){
	struct uart_packet send_pack;
	
	send_pack.start = 0x02;
	/*if(latest_vesc_vals.FW_VERSION_MINOR >= 48 && latest_vesc_vals.FW_VERSION_MINOR < 100){
		send_pack.len[0] = 0x05;
		send_pack.payload[0] = COMM_GET_VALUES_SELECTIVE;
		int32_t mask = (uint32_t)0b0001011111111001101;
		send_pack.payload[1] = (mask>>24)&0xFF;
		send_pack.payload[2] = (mask>>16)&0xFF;
		send_pack.payload[3] = (mask>>8)&0xFF;
		send_pack.payload[4] = (mask&0xFF);
		uint16_t crc = crc16(send_pack.payload, 5);
		send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
		send_pack.crc[1] = (uint8_t)(crc&0x00FF);
	} else {*/
		send_pack.len[0] = 0x01;
		send_pack.payload[0] = COMM_GET_VALUES;
		uint16_t crc = crc16(send_pack.payload, 1);
		send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
		send_pack.crc[1] = (uint8_t)(crc&0x00FF);
	//}

	send_packet(send_pack);
}

void vesc_get_mcconf(){
	struct uart_packet send_pack;

	send_pack.start = 0x02;
	send_pack.len[0] = 0x01;
	send_pack.payload[0] = COMM_GET_MCCONF;
	uint16_t crc = crc16(send_pack.payload, 1);
	send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
	send_pack.crc[1] = (uint8_t)(crc&0x00FF);

	send_packet(send_pack);
}

void vesc_send_alive(){
	struct uart_packet send_pack;

	send_pack.start = 0x02;
	send_pack.len[0] = 0x01;
	send_pack.payload[0] = COMM_ALIVE;
	uint16_t crc = crc16(send_pack.payload, 1);
	send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
	send_pack.crc[1] = (uint8_t)(crc&0x00FF);

	send_packet(send_pack);
}

void vesc_get_pwm(){
	if(!SEND_VESC_CHUCK)
	{
		struct uart_packet send_pack;

		send_pack.start = 0x02;
		send_pack.len[0] = 0x01;
		send_pack.payload[0] = COMM_GET_DECODED_PPM;
		uint16_t crc = crc16(send_pack.payload, 1);
		send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
		send_pack.crc[1] = (uint8_t)(crc&0x00FF);

		send_packet(send_pack);
	}
}

void vesc_get_chuck(){
	struct uart_packet send_pack;

	send_pack.start = 0x02;
	send_pack.len[0] = 0x01;
	send_pack.payload[0] = COMM_GET_DECODED_CHUK;
	uint16_t crc = crc16(send_pack.payload, 1);
	send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
	send_pack.crc[1] = (uint8_t)(crc&0x00FF);

	send_packet(send_pack);
}

void vesc_set_chuck(){
	struct uart_packet send_pack;

	send_pack.start = 0x02;
	send_pack.len[0] = 0x0B;
	send_pack.payload[0] = COMM_SET_CHUCK_DATA;
	send_pack.payload[1] = send_chuck_struct.js_x;
	send_pack.payload[2] = send_chuck_struct.js_y;
	send_pack.payload[3] = send_chuck_struct.bt_c;
	send_pack.payload[4] = send_chuck_struct.bt_z;
	send_pack.payload[5] = (send_chuck_struct.acc_x & 0xF0) >> 8;
	send_pack.payload[6] = (send_chuck_struct.acc_x & 0x0F);
	send_pack.payload[7] = (send_chuck_struct.acc_y & 0xF0) >> 8;
	send_pack.payload[8] = (send_chuck_struct.acc_y & 0x0F);
	send_pack.payload[9] = (send_chuck_struct.acc_z & 0xF0) >> 8;
	send_pack.payload[10] = (send_chuck_struct.acc_z & 0x0F);
	uint16_t crc = crc16(send_pack.payload, 11);
	send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
	send_pack.crc[1] = (uint8_t)(crc&0x00FF);

	send_packet(send_pack);
	HOLD_FOR_REPLY = false;
}

void vesc_get_imu(){
	struct uart_packet send_pack;

	send_pack.start = 0x02;
	send_pack.len[0] = 0x01;
	//send_pack.payload[0] = COMM_GET_IMU_DATA;
	uint16_t crc = crc16(send_pack.payload, 1);
	send_pack.crc[0] = (uint8_t)((crc&0xFF00)>>8);
	send_pack.crc[1] = (uint8_t)(crc&0x00FF);

	send_packet(send_pack);
}

void vesc_read_all(){
	static uint8_t read_index = 0;
	if(HOLD_FOR_REPLY && check_timer_expired(&vesc_usart_time, vesc_usart_timeout))
		HOLD_FOR_REPLY = false;
	
	if(!HOLD_FOR_REPLY){
		if(check_timer_expired(&vesc_usart_time, VESC_UART_PERIOD)){
			switch(read_index){
				case 0:
				if(!READ_VESC_PWM){
					read_index++;
				} else {
					read_index++;
					vesc_get_pwm();
				}
				break;
				case 1:
				if(!READ_VESC_FW){
					read_index++;
				} else {
					read_index++;
					vesc_get_fw_version();
				}
				break;
				case 2:
				if(!READ_VESC_VALS){
					read_index++;
				} else {
					read_index++;
					vesc_get_vals();
				}
				break;
				case 3:
				if(!READ_VESC_CHUCK){
					read_index++;
				} else {
					read_index++;
					vesc_get_chuck();
				}
				case 4:
				if(!SEND_VESC_CHUCK){
					read_index=0;
				} else {
					read_index=0;
					vesc_set_chuck();
				}
				break;
			}
		}
	}
	
	READ_VESC_PWM = false;
	READ_VESC_FW = false;
	READ_VESC_VALS = false;
	READ_VESC_CHUCK = false;
}


void detect_vesc_firmware(){
	vesc_get_fw_version();

	if(latest_vesc_vals.FW_VERSION_MAJOR != 0 || latest_vesc_vals.FW_VERSION_MINOR != 0)
	{
		ESC_FW_READ = true;

		// Define the location in which particular values can be found in COMM messages used by each FW
		if((latest_vesc_vals.FW_VERSION_MAJOR == 2 && latest_vesc_vals.FW_VERSION_MINOR <= 18)||
			(latest_vesc_vals.FW_VERSION_MAJOR == 3 && latest_vesc_vals.FW_VERSION_MINOR <= 6)){ // <= v2.18 || < 3.7
			esc_fw = FW_3v6;
		} else if((latest_vesc_vals.FW_VERSION_MAJOR == 3 && latest_vesc_vals.FW_VERSION_MINOR >= 7 && latest_vesc_vals.FW_VERSION_MINOR <= 49)){ // >= 3.7 && <= 3.49
			esc_fw = FW_3v7_3v49;
		} else if((latest_vesc_vals.FW_VERSION_MAJOR == 3 && latest_vesc_vals.FW_VERSION_MINOR >= 7 && latest_vesc_vals.FW_VERSION_MINOR <= 67)||
		(latest_vesc_vals.FW_VERSION_MAJOR == 4 && latest_vesc_vals.FW_VERSION_MINOR >= 0 && latest_vesc_vals.FW_VERSION_MINOR <= 2)||
		(latest_vesc_vals.FW_VERSION_MAJOR == 5 && latest_vesc_vals.FW_VERSION_MINOR >= 0 && latest_vesc_vals.FW_VERSION_MINOR <= 2)){ // >= 3.50 && <= 5.2
			esc_fw = FW_3v50_5v2;
		} else if(latest_vesc_vals.FW_VERSION_MAJOR == 23){ // Unity
			esc_fw = FW_UNITY;
		} else if(latest_vesc_vals.FW_VERSION_MAJOR == 3 && latest_vesc_vals.FW_VERSION_MINOR >= 100){ // Ackmaniac
			esc_fw = FW_ACKMANIAC;
		}

		switch(esc_fw){ // Set messgae IDs based on FW
			case FW_3v6: // <= v3.6
				COMM_FW_VERSION = 0;
				COMM_GET_VALUES = 4;
				COMM_GET_MCCONF = 13;
				COMM_ALIVE = 29;
				COMM_GET_DECODED_PPM = 30;
				COMM_GET_DECODED_CHUK = 32;
				COMM_SET_CHUCK_DATA = 34;
				break;
			case FW_3v7_3v49: // >= 3.7 && <= 3.49
			case FW_3v50_5v2: // >= 3.50 && <= 5.2
			case FW_UNITY:
			case FW_ACKMANIAC:
				COMM_FW_VERSION = 0;
				COMM_GET_VALUES = 4;
				COMM_GET_MCCONF = 14;
				COMM_ALIVE = 30;
				COMM_GET_DECODED_PPM = 31;
				COMM_GET_DECODED_CHUK = 33;
				COMM_SET_CHUCK_DATA = 35;
				break;
		}

		switch(esc_fw){ // Set GET_VALUES parsing based on FW
			case FW_3v6: // <= v3.6
				GET_VALUES_FET_TEMP = 1;
				GET_VALUES_MTR_CURR = 15;
				GET_VALUES_IN_CURR = 19;
				GET_VALUES_DUTY = 23;
				GET_VALUES_RPM = 25;
				GET_VALUES_IN_VOLT = 29;
				GET_VALUES_AH_USED = 31;
				GET_VALUES_AH_CHRG = 35;
				GET_VALUES_WH_USED = 39;
				GET_VALUES_WH_CHRG = 43;
				GET_VALUES_TACH = 47;
				GET_VALUES_FAULT = 55;
				break;
			case FW_3v7_3v49: // >= 3.7 && <= 3.49
			case FW_3v50_5v2: // >= 3.50 && <= 5.2
			case FW_ACKMANIAC:
				GET_VALUES_FET_TEMP = 1;
				GET_VALUES_MTR_CURR = 5;
				GET_VALUES_IN_CURR = 9;
				GET_VALUES_DUTY = 21;
				GET_VALUES_RPM = 23;
				GET_VALUES_IN_VOLT = 27;
				GET_VALUES_AH_USED = 29;
				GET_VALUES_AH_CHRG = 33;
				GET_VALUES_WH_USED = 37;
				GET_VALUES_WH_CHRG = 41;
				GET_VALUES_TACH = 45;
				GET_VALUES_FAULT = 53;
				break;
			case FW_UNITY:
				GET_VALUES_FET_TEMP = 1;
				GET_VALUES_MTR_CURR = 9;
				GET_VALUES_IN_CURR = 17;
				GET_VALUES_DUTY = 37;
				GET_VALUES_RPM = 41;
				GET_VALUES_IN_VOLT = 49;
				GET_VALUES_AH_USED = 51;
				GET_VALUES_AH_CHRG = 55;
				GET_VALUES_WH_USED = 59;
				GET_VALUES_WH_CHRG = 63;
				GET_VALUES_TACH = 67;
				GET_VALUES_FAULT = 83;
				break;
		}

		switch(esc_fw){ // Set GET_MCCONF parsing based on FW
			case FW_3v6: // <= v3.6
				GET_MCCONF_MTR_CURR_MAX = 5;
				GET_MCCONF_MTR_CURR_MIN = 9;
				GET_MCCONF_IN_CURR_MAX = 13;
				GET_MCCONF_IN_CURR_MIN = 17;
				GET_MCCONF_ABS_CURR_MAX = 21;
				GET_MCCONF_ERPM_MIN = 25;
				GET_MCCONF_ERPM_MAX = 29;
				GET_MCCONF_ERPM_FBRAKE_MAX = 33;
				GET_MCCONF_ERPM_FBRAKE_CC_MAX = 37;
				GET_MCCONF_VIN_MIN = 41;
				GET_MCCONF_VIN_MAX = 45;
				GET_MCCONF_BAT_CUT_STRT = 49;
				GET_MCCONF_BAT_CUT_END = 53;
				GET_MCCONF_TMP_FET_STRT = 59;
				GET_MCCONF_TMP_FET_END = 63;
				GET_MCCONF_TMP_MTR_STRT = 67;
				GET_MCCONF_TMP_MTR_END = 71;
				GET_MCCONF_DUTY_MIN = 75;
				GET_MCCONF_DUTY_MAX = 79;
				break;
			case FW_3v7_3v49: // >= 3.7 && <= 3.49
			case FW_UNITY:
			case FW_ACKMANIAC:
				GET_MCCONF_MTR_CURR_MAX = 5;
				GET_MCCONF_MTR_CURR_MIN = 9;
				GET_MCCONF_IN_CURR_MAX = 13;
				GET_MCCONF_IN_CURR_MIN = 17;
				GET_MCCONF_ABS_CURR_MAX = 21;
				GET_MCCONF_ERPM_MIN = 25;
				GET_MCCONF_ERPM_MAX = 29;
				GET_MCCONF_ERPM_FBRAKE_MAX = 37;
				GET_MCCONF_ERPM_FBRAKE_CC_MAX = 41;
				GET_MCCONF_VIN_MIN = 45;
				GET_MCCONF_VIN_MAX = 49;
				GET_MCCONF_BAT_CUT_STRT = 53;
				GET_MCCONF_BAT_CUT_END = 57;
				GET_MCCONF_TMP_FET_STRT = 62;
				GET_MCCONF_TMP_FET_END = 66;
				GET_MCCONF_TMP_MTR_STRT = 70;
				GET_MCCONF_TMP_MTR_END = 74;
				GET_MCCONF_DUTY_MIN = 82;
				GET_MCCONF_DUTY_MAX = 86;
				break;
			case FW_3v50_5v2: // >= 3.50 && <= 5.2
				GET_MCCONF_MTR_CURR_MAX = 9;
				GET_MCCONF_MTR_CURR_MIN = 13;
				GET_MCCONF_IN_CURR_MAX = 17;
				GET_MCCONF_IN_CURR_MIN = 21;
				GET_MCCONF_ABS_CURR_MAX = 25;
				GET_MCCONF_ERPM_MIN = 29;
				GET_MCCONF_ERPM_MAX = 33;
				GET_MCCONF_ERPM_FBRAKE_MAX = 41;
				GET_MCCONF_ERPM_FBRAKE_CC_MAX = 45;
				GET_MCCONF_VIN_MIN = 49;
				GET_MCCONF_VIN_MAX = 53;
				GET_MCCONF_BAT_CUT_STRT = 57;
				GET_MCCONF_BAT_CUT_END = 61;
				GET_MCCONF_TMP_FET_STRT = 66;
				GET_MCCONF_TMP_FET_END = 70;
				GET_MCCONF_TMP_MTR_STRT = 74;
				GET_MCCONF_TMP_MTR_END = 78;
				GET_MCCONF_DUTY_MIN = 86;
				GET_MCCONF_DUTY_MAX = 90;
				break;
		}
	}
}

inline bool CHECK_BUFFER(uint8_t *buf){
	return (((buf[0] == 0x2) && (buf[buf[1]+4] == 0x3)) || ((buf[0] == 0x3) && (buf[((buf[1]<<8)|buf[2])+5] == 0x3)));
}

void read_vesc_packet(void){
	if(CHECK_BUFFER(vesc_USART_read_buffer)){
		VESC_PACKET_RECIEVED = true;

		if(vesc_USART_read_buffer[0] == 0x2){
			packet_len = vesc_USART_read_buffer[1];
			memcpy(vesc_revieve_packet.payload,vesc_USART_read_buffer+2,packet_len);
			vesc_revieve_packet.crc[0] = vesc_USART_read_buffer[packet_len+2];
			vesc_revieve_packet.crc[1] = vesc_USART_read_buffer[packet_len+3];
		} else{
			packet_len = ((vesc_USART_read_buffer[1]<<8)|vesc_USART_read_buffer[2]);
			memcpy(vesc_revieve_packet.payload,vesc_USART_read_buffer+3,packet_len);
			vesc_revieve_packet.crc[0] = vesc_USART_read_buffer[packet_len+3];
			vesc_revieve_packet.crc[1] = vesc_USART_read_buffer[packet_len+4];
		}
		
		// Check if the message was corrupted
		uint16_t crc_check = crc16(vesc_revieve_packet.payload, packet_len);
		if(crc_check != (uint16_t)((vesc_revieve_packet.crc[0]<<8)|vesc_revieve_packet.crc[1])){
			//ERROR_LEDs(ERROR_TEAL);
			VESC_PACKET_RECIEVED = false; // dont handle the packet if it was
		}
		
		vesc_usart_time = millis();
		HOLD_FOR_REPLY = false;
		
		memset(vesc_USART_read_buffer,0,MAX_PAYLOAD_LEN+6);
		//Stop listening to the ESC UART
		usart_abort_job(&vesc_usart, USART_TRANSCEIVER_RX);
		// Start listening to the ESC UART
		usart_read_buffer_job(&vesc_usart, vesc_USART_read_buffer, MAX_PAYLOAD_LEN+6);
	} else if(CHECK_FOR_NOISE_ESC(&vesc_usart, vesc_USART_read_buffer, MAX_PAYLOAD_LEN+6, &ESC_noise_timer)){
		//Stop listening to the BLE UART
		usart_abort_job(&vesc_usart, USART_TRANSCEIVER_RX);
		memset(vesc_USART_read_buffer, 0, MAX_PAYLOAD_LEN+6);
		uint32_t temp_timer = millis();
		while(!check_timer_expired(&temp_timer, 10)){}
		// Start listening to the BLE UART
		usart_read_buffer_job(&vesc_usart, vesc_USART_read_buffer, MAX_PAYLOAD_LEN+6);
	}

	if(VESC_PACKET_RECIEVED){
		process_recieved_packet();
	}
}

void detect_esc_baud_pins(void){
	static uint32_t wait_time = 0;
	static uint32_t send_delay = 15;
	static uint32_t recieve_delay = 50;
	static bool CONFIG_PACKET_SENT = false;
	static bool DISABLE_UART = true;

	if(!ESC_UART_CONFIGED){
		if(DISABLE_UART){
			usart_disable(&vesc_usart);
			wait_time = millis();
			DISABLE_UART = false;
		}

		if(!CONFIG_PACKET_SENT && check_timer_expired(&wait_time, send_delay)){ // Pause 5ms

			// Configure the VESC UART with a new buad rate
			UART_baud++;
			if(UART_baud > BAUD_115200){
				UART_baud = BAUD_9600;
				ESC_UART_PIN_CONFIG = !ESC_UART_PIN_CONFIG;
			}
			configure_vesc_usart();

			//port_pin_set_output_level(STATUS_LED, true); // TODO: Add status LED

			// Start listening for responses
			usart_read_buffer_job(&vesc_usart, vesc_USART_read_buffer, MAX_PAYLOAD_LEN+6);

			// Dont wait for the normal send timeout to expire
			HOLD_FOR_REPLY = false;
			// Request FW version
			vesc_get_fw_version();

			CONFIG_PACKET_SENT = true;

			wait_time = millis();
		}
		else if(CONFIG_PACKET_SENT && check_timer_expired(&wait_time, recieve_delay)){

			//port_pin_set_output_level(STATUS_LED, false);

			// Check if a response was received
			if(CHECK_BUFFER(vesc_USART_read_buffer)){
				ESC_UART_CONFIGED = true;
				memset(vesc_USART_read_buffer,0,MAX_PAYLOAD_LEN+6);
				//Stop listening to the ESC UART
				usart_abort_job(&vesc_usart, USART_TRANSCEIVER_RX);
				// Start listening to the ESC UART
				usart_read_buffer_job(&vesc_usart, vesc_USART_read_buffer, MAX_PAYLOAD_LEN+6);
			}
			else
				memset(vesc_USART_read_buffer,0,MAX_PAYLOAD_LEN+6); // Clear the VESC receive buffer

			CONFIG_PACKET_SENT = false;
			DISABLE_UART = true;
		}
	}
}

bool CHECK_FOR_NOISE_ESC(struct usart_module *const module, uint8_t buf[MAX_PAYLOAD_LEN+6], uint16_t max_size, uint32_t *noise_timer){
	if(buf[0] != 0x02 && buf[0] != 0x03 && module->remaining_rx_buffer_length != max_size){
		return true;
	} else if(buf[0] == 0x02 || buf[0] == 0x03){
		if(check_timer_expired(noise_timer,500)){
			return true;
		}else{
			return false;
		}
	} else {
		*noise_timer = millis();
		return false;
	}
}


void detect_esc_uart_connected(void){
	if(port_pin_get_input_level(ESC_UART_RXTX_1) || port_pin_get_input_level(ESC_UART_RXTX_2)){
		ESC_UART_DETECTED = true;
	}
}
#endif /* CRC_H_ */