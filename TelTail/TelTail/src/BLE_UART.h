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




// usart globals
#define MAX_BLE_PAYLOAD_SIZE 15
#define MAX_BLE_MESSAGE_SIZE MAX_BLE_PAYLOAD_SIZE+4
#define BLE_START_BYTE 0xA5
#define BLE_STOP_BYTE 0x5A
uint8_t ble_USART_read_buffer[MAX_BLE_MESSAGE_SIZE];
uint8_t ble_write_buffer[44];
char SEND_LED_CHARS = 0;

struct usart_module ble_usart;

////////////   BLE Communication Variables   ///////////
////////////////////////////////////////////////////////
#define BLE_BAUD 115200
bool BLE_CONFIGURED = false; // Set monitor var to check if BLE is configured yet or not
uint8_t RECIEVE_REMOTE = 0;
uint8_t NEW_REMOTE_DATA = 0;
uint8_t SEND_CONTINUOUS = 1;
uint8_t FIRST_MESSAGE = 1;
uint8_t SEND_SENSORS = 0;
uint8_t CAL_IMU = 0;
uint8_t AppRemoteY = 128;
bool SEND_ORIENTAION_CONFIG = 0;
bool SEND_CONTROLS_CONFIG = 0;
bool SEND_REMOTE_CONFIG = 0;
bool SEND_ESC_CONFIG = 0;
bool OK_EXPECTED = 0;

int BLE_delay = 1000;

int ble_usart_count = 0;
uint8_t BLE_MSG[MAX_BLE_MESSAGE_SIZE];


// Direct Commands
// Settings Values
// LED Values
#define Read_Sensor_Vars		0xAC
#define Calibrate_All			0xAD
#define Remote_Data				0xBD
#define Read_LED_Vars			0xCD
#define Read_Motor_Limits		0xDD
#define LED_Mode_Up				0xE1
#define LED_Mode_Down			0xE2
#define LED_Toggle				0xE3
#define Read_Orientaion			0xFE
#define Calibrate_Gyro			0x00
#define Calibrate_Accel			0x00
#define Calibrate_Light			0x00
#define Read_Controls			0xFC
#define Aux_Pressed				0xAA
#define Aux_Released			0xAB
#define Read_Remote_Config		0xFB
#define Read_ESC_Config			0xFA
#define Custom_Values			0xB1
#define Y_Accel_Values			0xE6
#define X_Accel_Values			0xE7
#define RPM_Throttle			0xE8
#define RPM_Values				0xE9
#define Throttle_Values			0xEA
#define Compass_Cycle_Values	0xEB
#define Color_Cycle_Values		0xEC
#define Static_Values			0xED
#define Apply_Orientation		0xFD
#define Apply_Control_Settings	0xC2
#define Apply_Remote_Config		0xC3
#define Apply_ESC_Config		0xC4
#define End_of_Message			0xAE


bool check_ble_AT_recieved(void);
bool check_ble_packet_recieved(void);
void process_ble_packet(void);
void read_ble_packet(void);

struct ble_packet{
	uint8_t ID;
	uint8_t size;
	uint8_t payload[MAX_BLE_PAYLOAD_SIZE];
};

struct ble_packet ble_recieve_packet;

bool check_ble_AT_recieved(){
	return (ble_USART_read_buffer[0] == 'O' && ble_USART_read_buffer[1] == 'K');
}

bool check_ble_packet_recieved(){
	return (ble_USART_read_buffer[0] == BLE_START_BYTE && ble_USART_read_buffer[ble_USART_read_buffer[1]+3] == BLE_STOP_BYTE);
}

void process_ble_packet(){
	switch(ble_recieve_packet.ID){
		case Read_Motor_Limits:
			GET_LIMITS = 1;
			SEND_CONTINUOUS = 0;
			break;
		case (int)Read_LED_Vars:
			SEND_LED_CHARS = 1;
			SEND_CONTINUOUS = 0;
			break;
		case Calibrate_All:
			_autoCalc = false; // Workaround so that calibrate doesnt include the current offset
			calibrate(true);
			save_cal_data();
			break;
		case Read_Sensor_Vars:
			SEND_SENSORS = 1;
			SEND_CONTINUOUS = 0;
			break;
		case LED_Toggle:
			LIGHTS_ON = !LIGHTS_ON;
			save_led_data();
			break;
		case LED_Mode_Down:
			if(light_mode > 0)
			light_mode--;
			save_led_data();
			break;
		case LED_Mode_Up:
			if(light_mode < light_modes)
			light_mode++;
			save_led_data();
			break;
		case Read_Orientaion:
			SEND_ORIENTAION_CONFIG = 1;
			SEND_CONTINUOUS = 0;
			break;
		case Read_Controls:
			SEND_CONTROLS_CONFIG = 1;
			SEND_CONTINUOUS = 0;
			break;
		case Read_Remote_Config:
			SEND_REMOTE_CONFIG = 1;
			SEND_CONTINUOUS = 0;
			break;
		case Read_ESC_Config:
			SEND_ESC_CONFIG = 1;
			SEND_CONTINUOUS = 0;
			break;
		case Aux_Pressed:
			LIGHTS_ON = true;
			AppAuxButton = 1;
			break;
		case Aux_Released:
			LIGHTS_ON = false;
			AppAuxButton = 0;
			break;
		case Remote_Data:
			AppRemoteY = (ble_recieve_packet.payload[0] & 0x0FF);
			NEW_REMOTE_DATA = 1;
			break;
		case RPM_Throttle:
			LIGHTS_ON = 1;
			light_mode = MODE_RPM_THROTTLE;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			save_led_data();
			break;
		case Compass_Cycle_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_COMPASS_CYCLE;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			Brightness[MODE_COMPASS_CYCLE] = ((float)(ble_recieve_packet.payload[1]))/100;
			save_led_data();
			break;
		case RPM_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_RPM_CYCLE;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			RateSens[MODE_RPM_CYCLE] = ((float)(ble_recieve_packet.payload[1]))/100;
			save_led_data();
			break;
		case X_Accel_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_X_ACCEL;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			RateSens[MODE_X_ACCEL] = ((float)(ble_recieve_packet.payload[1]))/100;
			save_led_data();
			break;
		case Y_Accel_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_Y_ACCEL;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			Brightness[MODE_Y_ACCEL] = ((float)(ble_recieve_packet.payload[1]))/100;
			save_led_data();
			break;
		case Apply_Orientation:
			ORIENTATION[0] = ble_recieve_packet.payload[0];
			ORIENTATION[1] = ble_recieve_packet.payload[1];
			save_orientation_controls_remote_esc();
			break;
		case Apply_Remote_Config:
			remote_type = (ble_recieve_packet.payload[0]&0x0F0)>>4;
			button_type = (ble_recieve_packet.payload[0]&0x0F);
			deadzone = ble_recieve_packet.payload[1];
			save_orientation_controls_remote_esc();
			break;
		case Apply_ESC_Config:
			esc_fw = ble_recieve_packet.payload[0];
			esc_comms = (ble_recieve_packet.payload[1]&0x0F0)>>4;
			UART_baud = (ble_recieve_packet.payload[1]&0x0F);
			save_orientation_controls_remote_esc();
			configured_comms = esc_comms;
			break;
		case Color_Cycle_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_COLOR_CYCLE;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			RateSens[MODE_COLOR_CYCLE] = ((float)(ble_recieve_packet.payload[1]))/100;
			Brightness[MODE_COLOR_CYCLE] = ((float)(ble_recieve_packet.payload[2]))/100;
			save_led_data();
			break;
		case Throttle_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_THROTTLE;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			RateSens[MODE_THROTTLE] = ((float)(ble_recieve_packet.payload[1]))/100;
			Brightness[MODE_THROTTLE] = ((float)(ble_recieve_packet.payload[2]))/100;
			save_led_data();
			break;
		case Static_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_STATIC;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			Static_RGB.LR = (uint16_t)((float)ble_recieve_packet.payload[1] * 257);
			Static_RGB.LG = (uint16_t)((float)ble_recieve_packet.payload[2] * 257);
			Static_RGB.LB = (uint16_t)((float)ble_recieve_packet.payload[3] * 257);
			Static_RGB.RR = (uint16_t)((float)ble_recieve_packet.payload[4] * 257);
			Static_RGB.RG = (uint16_t)((float)ble_recieve_packet.payload[5] * 257);
			Static_RGB.RB = (uint16_t)((float)ble_recieve_packet.payload[6] * 257);
			save_led_data();
			break;
		case Apply_Control_Settings:
			AUX_ENABLED = (ble_recieve_packet.payload[0]&0x80)>>7;
			TURN_ENABLED = (ble_recieve_packet.payload[0]&0x40)>>6;
			auxControlType = (ble_recieve_packet.payload[0]&0x0F);
			auxTimedDuration = (ble_recieve_packet.payload[1]&0xFF);
			single_aux_control = (ble_recieve_packet.payload[2]&0xF0)>>4;
			single_all_control = (ble_recieve_packet.payload[2]&0x0F);
			single_head_control = (ble_recieve_packet.payload[3]&0xF0)>>4;
			single_side_control = (ble_recieve_packet.payload[3]&0x0F);
			single_down_control = (ble_recieve_packet.payload[4]&0xF0)>>4;
			single_up_control = (ble_recieve_packet.payload[4]&0x0F);
			dual_aux_control = (ble_recieve_packet.payload[5]&0xF0)>>4;
			dual_all_control = (ble_recieve_packet.payload[5]&0x0F);
			dual_head_control = (ble_recieve_packet.payload[6]&0xF0)>>4;
			dual_side_control = (ble_recieve_packet.payload[6]&0x0F);
			dual_down_control = (ble_recieve_packet.payload[7]&0xF0)>>4;
			dual_up_control = (ble_recieve_packet.payload[7]&0x0F);
			save_orientation_controls_remote_esc();
			break;
		case Custom_Values:
			LIGHTS_ON = 1;
			light_mode = MODE_CUSTOM;
			SWITCHES = ble_recieve_packet.payload[0];
			SIDELIGHTS = (SWITCHES & 0x10) >> 4;
			HEADLIGHTS = (SWITCHES & 0x20) >> 5;
			LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
			IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
			ColorBase[MODE_CUSTOM] = (SWITCHES & 0x0F);
			RateBase[MODE_CUSTOM] = (ble_recieve_packet.payload[1] & 0xF0) >> 4;
			BrightBase[MODE_CUSTOM] = (ble_recieve_packet.payload[1] & 0x0F);
			Custom_RGB.LR = (uint16_t)((float)ble_recieve_packet.payload[2] * 257);
			Custom_RGB.LG = (uint16_t)((float)ble_recieve_packet.payload[3] * 257);
			Custom_RGB.LB = (uint16_t)((float)ble_recieve_packet.payload[4] * 257);
			Custom_RGB.RR = (uint16_t)((float)ble_recieve_packet.payload[5] * 257);
			Custom_RGB.RG = (uint16_t)((float)ble_recieve_packet.payload[6] * 257);
			Custom_RGB.RB = (uint16_t)((float)ble_recieve_packet.payload[7] * 257);
			RateSens[MODE_CUSTOM] = ((float)(ble_recieve_packet.payload[8]))/100;
			Brightness[MODE_CUSTOM] = ((float)(ble_recieve_packet.payload[9]))/100;
			save_led_data();
			break;
	}
}

void read_ble_packet(){
	if(check_ble_packet_recieved()){
		ble_recieve_packet.size = ble_USART_read_buffer[1];
		ble_recieve_packet.ID = ble_USART_read_buffer[2];
		memcpy(ble_recieve_packet.payload, ble_USART_read_buffer+3, ble_recieve_packet.size);
		process_ble_packet();
			
		memset(ble_USART_read_buffer, 0, MAX_BLE_MESSAGE_SIZE);
		//Stop listening to the BLE UART
		usart_abort_job(&ble_usart, USART_TRANSCEIVER_RX);
		// Start listening to the BLE UART
		usart_read_buffer_job(&ble_usart, ble_USART_read_buffer, MAX_BLE_MESSAGE_SIZE);
	} else if(check_ble_AT_recieved()){
		if(!BLE_CONFIGURED && OK_EXPECTED){
			BLE_CONFIGURED = true;
			OK_EXPECTED = false;
		}
		
		memset(ble_USART_read_buffer, 0, MAX_BLE_MESSAGE_SIZE);
		//Stop listening to the BLE UART
		usart_abort_job(&ble_usart, USART_TRANSCEIVER_RX);
		// Start listening to the BLE UART
		usart_read_buffer_job(&ble_usart, ble_USART_read_buffer, MAX_BLE_MESSAGE_SIZE);
	}
}