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

#include <asf.h>
#include <string.h>
#include "IMU.h"
#include "Timing.h"
#include "LED_Vars.h"
#include "LED_Functions.h"
#include "Controls.h"
#include "EEPROM_Functions.h"
#include <math.h>


////////////   HW Configuration Variables   ////////////
////////////////////////////////////////////////////////
#define CONF_PWM_MODULE      TCC1
#define CONF_PWM_CHANNEL     0
#define CONF_PWM_OUTPUT      0
#define CONF_PWM_OUT_PIN     PIN_PA06E_TCC1_WO0
#define CONF_PWM_OUT_MUX     MUX_PA06E_TCC1_WO0

// i2c slave globals
#define SLAVE_READ_DATA_LENGTH 30
#define SLAVE_WRITE_DATA_LENGTH 3
uint8_t I2C_slave_read_buffer[SLAVE_READ_DATA_LENGTH];
uint8_t I2C_slave_write_buffer[SLAVE_WRITE_DATA_LENGTH];
#define SLAVE_ADDRESS 0x12
#define SLAVE_TIMEOUT 1
static struct i2c_slave_packet packet;

// usart globals
#define USART_READ_DATA_LENGTH 15
uint8_t USART_read_buffer[USART_READ_DATA_LENGTH];
uint8_t send_buffer[44];
char SEND_LED_CHARS = 0;


///////////   VESC Communication Variables   ///////////
////////////////////////////////////////////////////////
int16_t battery_current = 0;
int16_t battery_voltage = 0;
int16_t motor_current = 0;
int16_t vesc_temp = 0;
int8_t duty_cycle = 0;
uint8_t error_codes = 0;
int32_t motor_rpm = 0;
int32_t mAh_used = 0;
int32_t mAh_charged = 0;
int32_t Wh_used = 0;
int32_t Wh_charged = 0;
uint8_t battery_current_max = 30;
uint8_t battery_current_min = -15;
uint8_t absolute_max_current = 100;
uint8_t battery_voltage_max = 48;
uint8_t battery_voltage_min = 33;
uint8_t voltage_cutoff_start = 36;
uint8_t voltage_cutoff_end = 33;
uint8_t motor_current_max = 60;
uint8_t motor_current_min = -40;
int motor_rpm_max = 100000;
int motor_rpm_min = -100000;
int max_rpm_full = 70000;
int max_rpm_full_cc = 70000;
uint8_t mosfet_temp_start = 88;
uint8_t mosfet_temp_end = 85;
uint8_t motor_temp_start = 90;
uint8_t motor_temp_end = 89;
uint8_t duty_cycle_max = 95;
uint8_t duty_cycle_min = 5;

int BLE_delay = 1000;


/////////////////   Sensor Variables   /////////////////
////////////////////////////////////////////////////////
// Sensor averaging vars
uint16_t light_sens = 0;
#define LGHTsamples 150
#define ACCELsamples 15
#define GYROsamples 15
uint16_t LGHTaverage[LGHTsamples];
int16_t AXaverage[ACCELsamples];
int16_t AYaverage[ACCELsamples];
int16_t AZaverage[ACCELsamples];
//int16_t GXaverage[GYROsamples];
//int16_t GYaverage[GYROsamples];
//int16_t GZaverage[GYROsamples];
unsigned long LGHTtotal = 0;
long AXtotal = 0;
long AYtotal = 0;
long AZtotal = 0;
//long GXtotal = 0;
//long GYtotal = 0;
//long GZtotal = 0;

int16_t avgAX = 0;
int16_t avgAY = 0;
int16_t avgAZ = 0;
float kalmanAX_min = -350;
float kalmanAX_max = 350;
float kalmanAY_min = -350;
float kalmanAY_max = 350;
float kalmanAZ_min = -350;
float kalmanAZ_max = 350;
float kalmanGX_min = -350;
float kalmanGX_max = 350;
float kalmanGY_min = -350;
float kalmanGY_max = 350;
float kalmanGZ_min = -350;
float kalmanGZ_max = 350;

// Kalman filter vars
#define KalmanArraySize 7
typedef enum {
	ax_kalman = 0,
	ay_kalman = 1,
	az_kalman = 2,
	gx_kalman = 3,
	gy_kalman = 4,
	gz_kalman = 5,
	light_kalman = 6
} kalmans;
float err_measure[KalmanArraySize];
float err_estimate[KalmanArraySize];
float q[KalmanArraySize];
float current_estimate[KalmanArraySize];
float last_estimate[KalmanArraySize];
float kalman_gain[KalmanArraySize];
float axKalman = 0;
float ayKalman = 0;
float azKalman = 0;
float gxKalman = 0;
float gyKalman = 0;
float gzKalman = 0;


////////////   BLE Communication Variables   ///////////
////////////////////////////////////////////////////////
#define BLE_BAUD 38400
bool BLE_CONFIGURED = false; // Set monitor var to check if BLE is configured yet or not
uint8_t RECIEVE_REMOTE = 0;
uint8_t NEW_REMOTE_DATA = 0;
uint8_t SEND_CONTINUOUS = 1;
uint8_t SEND_LIMITS = 0;
uint8_t FIRST_MESSAGE = 1;
uint8_t GET_LIMITS = 0;
uint8_t SEND_SENSORS = 0;
uint8_t CAL_IMU = 0;
uint8_t AppRemoteY = 128;
bool SEND_ORIENTAION = 0;
bool SEND_CONTROLS = 0;


/////   Function Definitions and System Structs   //////
////////////////////////////////////////////////////////
struct i2c_slave_module i2c_slave_instance;
struct adc_module adc1;
struct usart_module usart_instance;

// Initialization functions
void configure_ADC(void);
void configure_i2c_master(void);
void configure_port_pins(void);
void configure_BLE_usart(void);
void configure_usart(int baud);
void configure_i2c_slave(void);
void number_to_string(uint32_t, char *);
void configure_i2c_slave_callbacks(void);
void i2c_write_request_callback(struct i2c_slave_module *const module);
void initKalman(float meas, float est, float _q);
void usart_read_callback(struct usart_module *const usart_module);
void configure_BLE_usart_callbacks(void);
void i2c_read_request_callback(struct i2c_slave_module *const module);
void configure_eeprom(void);

// Long-itude peripherals 
void getLightSens(uint16_t* light_val);
char sensorControl(void);
char lightControlHead(void);
char lightControlSide(void);
int16_t averageAZ(void);
int16_t averageAY(void);
int16_t averageAX(void);

// Utility Functions
float getRoll(void);
float getPitch(void);
float updateKalman(float meas, int kalmanIndex);


int usart_count = 0;
uint8_t BLE_MSG[USART_READ_DATA_LENGTH];
// The callback routine for when a BLE message is recieved
void usart_read_callback(struct usart_module *const usart_module)
{
	usart_count++;

	if(usart_count < USART_READ_DATA_LENGTH)
		BLE_MSG[usart_count-1] = USART_read_buffer[0];
	else
		ERROR_LEDs(1);

	usart_read_buffer_job(&usart_instance, (uint8_t *)USART_read_buffer, (uint16_t)1);
	if(USART_read_buffer[0] == 0xAE){
		switch(usart_count){
			case 2:
				if(BLE_MSG[0] == 0xDD){
					GET_LIMITS = 1;
				} else if(BLE_MSG[0] == 0xCD){
					SEND_LED_CHARS = 1;
					SEND_CONTINUOUS = 0;
				} else if(BLE_MSG[0] == 0xAD){
					_autoCalc = false; // Workaround so that calibrate doesnt include the current offset
					calibrate(true);
					save_cal_data();
				} else if(BLE_MSG[0] == 0xAC){
					SEND_SENSORS = 1;
					SEND_CONTINUOUS = 0;
				} else if(BLE_MSG[0] == 0xE3){
					LIGHTS_ON = !LIGHTS_ON;
					save_led_data();
				} else if(BLE_MSG[0] == 0xE2){
					if(light_mode > 0)
					light_mode--;
					save_led_data();
				} else if(BLE_MSG[0] == 0xE1){
					if(light_mode < light_modes)
					light_mode++;
					save_led_data();
				} else if(BLE_MSG[0] == 0xFE){
					SEND_ORIENTAION = 1;
					SEND_CONTINUOUS = 0;
				} else if(BLE_MSG[0] == 0xFC){
					SEND_CONTROLS = 1;
					SEND_CONTINUOUS = 0;
				} else if(BLE_MSG[0] == 0xAA){
					LIGHTS_ON = true;
					AppAuxButton = 1;
				} else if(BLE_MSG[0] == 0xAB){
					LIGHTS_ON = false;
					AppAuxButton = 0;
				}
			break;
			case 3:
				if(BLE_MSG[0] == 0xBD){
					AppRemoteY = (BLE_MSG[1] & 0x0FF);
					NEW_REMOTE_DATA = 1;
				}
				if(BLE_MSG[0] == 0xE8){
					LIGHTS_ON = 1;
					light_mode = MODE_RPM_THROTTLE;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					save_led_data();
				}
			break;
			case 4:
				if(BLE_MSG[0] == 0xEB){
					LIGHTS_ON = 1;
					light_mode = MODE_COMPASS_CYCLE;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					Brightness[MODE_COMPASS_CYCLE] = ((float)(BLE_MSG[2]))/100;
					save_led_data();
				} else if(BLE_MSG[0] == 0xE9){
					LIGHTS_ON = 1;
					light_mode = MODE_RPM_CYCLE;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					RateSens[MODE_RPM_CYCLE] = ((float)(BLE_MSG[2]))/100;
					save_led_data();
				} else if(BLE_MSG[0] == 0xE7){
					LIGHTS_ON = 1;
					light_mode = MODE_X_ACCEL;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					RateSens[MODE_X_ACCEL] = ((float)(BLE_MSG[2]))/100;
					save_led_data();
				} else if(BLE_MSG[0] == 0xE6){
					LIGHTS_ON = 1;
					light_mode = MODE_Y_ACCEL;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					RateSens[MODE_Y_ACCEL] = ((float)(BLE_MSG[2]))/100;
					save_led_data();
				} else if(BLE_MSG[0] == 0xFD){
					ORIENTATION[0] = BLE_MSG[1];
					ORIENTATION[1] = BLE_MSG[2];
					save_orientation_controls();
				}
			break;
			case 5:
				if(BLE_MSG[0] == 0xEC){
					LIGHTS_ON = 1;
					light_mode = MODE_COLOR_CYCLE;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					RateSens[MODE_COLOR_CYCLE] = ((float)(BLE_MSG[2]))/100;
					Brightness[MODE_COLOR_CYCLE] = ((float)(BLE_MSG[3]))/100;
					save_led_data();
				} else if(BLE_MSG[0] == 0xEA){
					LIGHTS_ON = 1;
					light_mode = MODE_THROTTLE;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					RateSens[MODE_THROTTLE] = ((float)(BLE_MSG[2]))/100;
					Brightness[MODE_THROTTLE] = ((float)(BLE_MSG[3]))/100;
					save_led_data();
				}
			break;
			case 9:
				if(BLE_MSG[0] == 0xED){
					LIGHTS_ON = 1;
					light_mode = MODE_STATIC;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					Static_RGB.LR = (uint16_t)((float)BLE_MSG[2] * 257);
					Static_RGB.LG = (uint16_t)((float)BLE_MSG[3] * 257);
					Static_RGB.LB = (uint16_t)((float)BLE_MSG[4] * 257);
					Static_RGB.RR = (uint16_t)((float)BLE_MSG[5] * 257);
					Static_RGB.RG = (uint16_t)((float)BLE_MSG[6] * 257);
					Static_RGB.RB = (uint16_t)((float)BLE_MSG[7] * 257);
					save_led_data();
				}
			break;
			case 10:
				if(BLE_MSG[0] == 0xC2){
					AUX_ENABLED = (BLE_MSG[1]&0x80)>>7;
					TURN_ENABLED = (BLE_MSG[1]&0x40)>>6;
					auxControlType = (BLE_MSG[1]&0x0F);
					auxTimedDuration = (BLE_MSG[2]&0xFF);
					single_aux_control = (BLE_MSG[3]&0xF0)>>4;
					single_all_control = (BLE_MSG[3]&0x0F);
					single_head_control = (BLE_MSG[4]&0xF0)>>4;
					single_side_control = (BLE_MSG[4]&0x0F);
					single_down_control = (BLE_MSG[5]&0xF0)>>4;
					single_up_control = (BLE_MSG[5]&0x0F);
					dual_aux_control = (BLE_MSG[6]&0xF0)>>4;
					dual_all_control = (BLE_MSG[6]&0x0F);
					dual_head_control = (BLE_MSG[7]&0xF0)>>4;
					dual_side_control = (BLE_MSG[7]&0x0F);
					dual_down_control = (BLE_MSG[8]&0xF0)>>4;
					dual_up_control = (BLE_MSG[8]&0x0F);
					save_orientation_controls();
				}
			break;
			case 12:
				if(BLE_MSG[0] == 0xB1){
					LIGHTS_ON = 1;
					light_mode = MODE_CUSTOM;
					SWITCHES = BLE_MSG[1];
					SIDELIGHTS = (SWITCHES & 0x10) >> 4;
					HEADLIGHTS = (SWITCHES & 0x20) >> 5;
					LIGHT_CONTROLLED = (SWITCHES & 0x40) >> 6;
					IMU_CONTROLED = (SWITCHES & 0x80) >> 7;
					ColorBase[MODE_CUSTOM] = (SWITCHES & 0x0F);
					RateBase[MODE_CUSTOM] = (BLE_MSG[2] & 0xF0) >> 4;
					BrightBase[MODE_CUSTOM] = (BLE_MSG[2] & 0x0F);
					Custom_RGB.LR = (uint16_t)((float)BLE_MSG[3] * 257);
					Custom_RGB.LG = (uint16_t)((float)BLE_MSG[4] * 257);
					Custom_RGB.LB = (uint16_t)((float)BLE_MSG[5] * 257);
					Custom_RGB.RR = (uint16_t)((float)BLE_MSG[6] * 257);
					Custom_RGB.RG = (uint16_t)((float)BLE_MSG[7] * 257);
					Custom_RGB.RB = (uint16_t)((float)BLE_MSG[8] * 257);
					RateSens[MODE_CUSTOM] = ((float)(BLE_MSG[9]))/100;
					Brightness[MODE_CUSTOM] = ((float)(BLE_MSG[10]))/100;
					save_led_data();
				}
			break;
		}
		usart_count = 0;
	}
	else if(BLE_MSG[usart_count-3] == 'O' && BLE_MSG[usart_count-2] == 'K' &&USART_read_buffer[0] == '+')
	{
		BLE_CONFIGURED = true;
		usart_count = 0;
	}
}

// Configure SERCOM5 as USART for BLE module
void configure_usart(int baud)
{
	struct usart_config config_usart;
	usart_get_config_defaults(&config_usart);
	config_usart.baudrate    = baud;
	config_usart.mux_setting = USART_RX_3_TX_2_XCK_3;
	config_usart.pinmux_pad0 = PINMUX_UNUSED;
	config_usart.pinmux_pad1 = PINMUX_UNUSED;
	config_usart.pinmux_pad2 = PINMUX_PA20C_SERCOM5_PAD2;
	config_usart.pinmux_pad3 = PINMUX_PA21C_SERCOM5_PAD3;
	while (usart_init(&usart_instance,SERCOM5, &config_usart) != STATUS_OK)
	{}
	usart_enable(&usart_instance);
}

void configure_BLE_usart()
{
	int baud = 0;
	int bauds[5] = {9600, 19200, 38400, 57600, 115200};
	while(1){
		configure_usart(bauds[baud]);
		configure_BLE_usart_callbacks();
		usart_read_buffer_job(&usart_instance, (uint8_t *)USART_read_buffer, (uint16_t)1);

		baud += 1;
		if(baud > 4)
			baud = 0;
			
		for(int i = 0; i < 10000; ++i);
		uint8_t string1[8];
		if(BLE_BAUD == 9600)
			strcpy(string1,"AT+BAUD0");
		else if(BLE_BAUD == 19200)
			strcpy(string1,"AT+BAUD1");
		else if(BLE_BAUD == 38400)
			strcpy(string1,"AT+BAUD2");
		else if(BLE_BAUD == 57600)
			strcpy(string1,"AT+BAUD3");
		else if(BLE_BAUD == 115200)
			strcpy(string1,"AT+BAUD4");
		while(usart_write_buffer_wait(&usart_instance, string1, sizeof(string1))!=STATUS_OK){}
		for(int i = 0; i < 25000; ++i);
		
		uint8_t string2[14] = "AT+NAMETelTail";
		while(usart_write_buffer_wait(&usart_instance, string2, sizeof(string2))!=STATUS_OK){}
		for(int i = 0; i < 25000; ++i);
		
		uint8_t string3[8] = "AT+POWE3"; // Default = 2
		while(usart_write_buffer_wait(&usart_instance, string3, sizeof(string3))!=STATUS_OK){}
		for(int i = 0; i < 25000; ++i);
		
		if(!BLE_CONFIGURED){
			usart_disable(&usart_instance);
			for(int i = 0; i < 10000; ++i);
		}
		else{
			uint8_t string4[8] = "AT+RESET";
			while(usart_write_buffer_wait(&usart_instance, string4, sizeof(string4))!=STATUS_OK){}
			for(int i = 0; i < 25000; ++i);
			usart_disable(&usart_instance);
			for(int i = 0; i < 500000; ++i);
			configure_usart(BLE_BAUD);
			for(int i = 0; i < 5000; ++i);
			uint8_t string5[2] = "AT";
			while(usart_write_buffer_wait(&usart_instance, string5, sizeof(string5))!=STATUS_OK){}
			for(int i = 0; i < 10000; ++i);
			configure_BLE_usart_callbacks();
			usart_read_buffer_job(&usart_instance, (uint8_t *)USART_read_buffer, (uint16_t)1);
			break;
		}
	}
}

// Configure SERCOM callback for recieving a buffer frame
void configure_BLE_usart_callbacks(void)
{
	usart_register_callback(&usart_instance, usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
}

// Configure the light sensor port as an input
void configure_ADC(void)
{
	for(int i = 0; i < LGHTsamples; ++i){
		LGHTaverage[i] = 0;
	}

	struct adc_config config_adc;
	adc_get_config_defaults(&config_adc);
	config_adc.reference = ADC_REFERENCE_INTVCC1;
	config_adc.resolution = ADC_RESOLUTION_16BIT;
	config_adc.differential_mode = DISABLE;
	config_adc.negative_input = ADC_NEGATIVE_INPUT_GND;
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN16;
	config_adc.freerunning = DISABLE;
	config_adc.run_in_standby = ENABLE;
	config_adc.left_adjust = false;
	adc_init(&adc1, ADC, &config_adc);
	adc_enable(&adc1);
}

// Configure the LED selection port as output
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	
	config_port_pin.powersave = false;
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	config_port_pin.input_pull = PORT_PIN_PULL_NONE;
	port_pin_set_config(PPM_IN, &config_port_pin);
	
	config_port_pin.powersave = false;
	config_port_pin.input_pull = PORT_PIN_PULL_NONE;
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(AUX_PIN, &config_port_pin);
}

// Turn number to a string for BLE transmission
void number_to_string(uint32_t number, char *str)
{
	memset(str, 0, strlen(str) * (sizeof str[0]) );
	if(number != 0){
		while(number != 0){
			uint8_t temp = number%10;
			number = number / 10;

			char tmp[2];
			tmp[0] = (char)(temp+48);
			tmp[1] = 0; // strings need a zero terminator
		
			strcat(tmp, str);
			strcpy(str,tmp);
		}
	}
	else{
		strcpy(str, "0");
	}
}

// Configure SERCOM1 as I2C slave for VESC communication
void configure_i2c_slave(void)
{	
	/* Create and initialize config_i2c_slave structure */
	struct i2c_slave_config config_i2c_slave;
	i2c_slave_get_config_defaults(&config_i2c_slave);
	/* Change address and address_mode */
	config_i2c_slave.address        = SLAVE_ADDRESS;
	config_i2c_slave.address_mode   = I2C_SLAVE_ADDRESS_MODE_MASK;
	config_i2c_slave.pinmux_pad0 = PINMUX_PA16C_SERCOM1_PAD0;
	config_i2c_slave.pinmux_pad1 = PINMUX_PA17C_SERCOM1_PAD1;
	config_i2c_slave.generator_source = GCLK_GENERATOR_0;
	/* Initialize and enable device with config_i2c_slave */
	i2c_slave_init(&i2c_slave_instance, SERCOM1, &config_i2c_slave);
	i2c_slave_enable(&i2c_slave_instance);
}

void i2c_write_request_callback(struct i2c_slave_module *const module)
{
	/* Init i2c packet. */
	packet.data_length = SLAVE_READ_DATA_LENGTH;
	packet.data        = I2C_slave_read_buffer;
	
	if(FIRST_MESSAGE == 1){
		FIRST_MESSAGE = 0;
		GET_LIMITS = 1; // Read the limits on first message to set lighting variables
	}
	i2c_slave_read_packet_job(module, &packet);
		if(I2C_slave_read_buffer[0] == 0x8D && I2C_slave_read_buffer[28] == 0xAD) {
			motor_current_max = I2C_slave_read_buffer[1];
			motor_current_min = I2C_slave_read_buffer[2];
			battery_current_max = I2C_slave_read_buffer[3];
			battery_current_min = I2C_slave_read_buffer[4];
			absolute_max_current = I2C_slave_read_buffer[5];
			battery_voltage_max = I2C_slave_read_buffer[6];
			battery_voltage_min = I2C_slave_read_buffer[7];
			voltage_cutoff_start = I2C_slave_read_buffer[8];
			voltage_cutoff_end = I2C_slave_read_buffer[9];
			motor_rpm_max = (I2C_slave_read_buffer[10] | (I2C_slave_read_buffer[11] << 8) | (I2C_slave_read_buffer[12] << 16));
			motor_rpm_min = (I2C_slave_read_buffer[13] | (I2C_slave_read_buffer[14] << 8) | (I2C_slave_read_buffer[15] << 16));
			max_rpm_full = (I2C_slave_read_buffer[16] | (I2C_slave_read_buffer[17] << 8) | (I2C_slave_read_buffer[18] << 16));
			max_rpm_full_cc = (I2C_slave_read_buffer[19] | (I2C_slave_read_buffer[20] << 8) | (I2C_slave_read_buffer[21] << 16));
			mosfet_temp_start = I2C_slave_read_buffer[22];
			mosfet_temp_end = I2C_slave_read_buffer[23];
			motor_temp_start = I2C_slave_read_buffer[24];
			motor_temp_end = I2C_slave_read_buffer[25];
			duty_cycle_max = I2C_slave_read_buffer[26];
			duty_cycle_min = I2C_slave_read_buffer[27];
			SEND_LIMITS = 1;
			SEND_CONTINUOUS = 0;
		} else if(I2C_slave_read_buffer[0] == 0xDD && I2C_slave_read_buffer[29] == 0xAD) {
			battery_current = I2C_slave_read_buffer[1];
			battery_current += (I2C_slave_read_buffer[2] << 8);
			battery_voltage = I2C_slave_read_buffer[3];
			battery_voltage += (I2C_slave_read_buffer[4] << 8);
			motor_current = I2C_slave_read_buffer[5];
			motor_current += (I2C_slave_read_buffer[6] << 8);
			vesc_temp = I2C_slave_read_buffer[7];
			vesc_temp += (I2C_slave_read_buffer[8] << 8);
			duty_cycle = I2C_slave_read_buffer[9];
			motor_rpm = (I2C_slave_read_buffer[10] | (I2C_slave_read_buffer[11] << 8) | (I2C_slave_read_buffer[12] << 16));
			mAh_used = (I2C_slave_read_buffer[13] | (I2C_slave_read_buffer[14] << 8) | (I2C_slave_read_buffer[15] << 16));
			mAh_charged = (I2C_slave_read_buffer[16] | (I2C_slave_read_buffer[17] << 8) | (I2C_slave_read_buffer[18] << 16));
			Wh_used = (I2C_slave_read_buffer[19] | (I2C_slave_read_buffer[20] << 8) | (I2C_slave_read_buffer[21] << 16));
			Wh_charged = (I2C_slave_read_buffer[22] | (I2C_slave_read_buffer[23] << 8) | (I2C_slave_read_buffer[24] << 16));
			VescRemoteX = I2C_slave_read_buffer[25];
			VescRemoteY = I2C_slave_read_buffer[26];
			REMOTE_TYPE = (I2C_slave_read_buffer[27] & 0x6) >> 1;
			VescRemoteButton = I2C_slave_read_buffer[27] & 0x1;
			error_codes = I2C_slave_read_buffer[28];
		}
}

uint8_t app_remote_check = 0;
void i2c_read_request_callback(struct i2c_slave_module *const module)
{
	static long request_count = 100;
	static uint32_t requestTimer = 0;

	I2C_slave_write_buffer[0] = AppRemoteY;
	I2C_slave_write_buffer[1] = GET_LIMITS;
	I2C_slave_write_buffer[2] = app_remote_check;

	/* Init i2c packet. */
	packet.data_length = SLAVE_WRITE_DATA_LENGTH;
	packet.data        = I2C_slave_write_buffer;
	/* Write buffer to master */
	i2c_slave_write_packet_job(module, &packet);
	NEW_REMOTE_DATA = 0;

	GET_LIMITS = 0;
}


void configure_i2c_slave_callbacks(void)
{
	/* Register and enable callback functions */
	i2c_slave_register_callback(&i2c_slave_instance, i2c_read_request_callback, I2C_SLAVE_CALLBACK_READ_REQUEST);
	i2c_slave_enable_callback(&i2c_slave_instance, I2C_SLAVE_CALLBACK_READ_REQUEST);

	i2c_slave_register_callback(&i2c_slave_instance, i2c_write_request_callback, I2C_SLAVE_CALLBACK_WRITE_REQUEST);
	i2c_slave_enable_callback(&i2c_slave_instance, I2C_SLAVE_CALLBACK_WRITE_REQUEST);
}

void configure_eeprom(void)
{
	/* Setup EEPROM emulator service */
	enum status_code error_code = eeprom_emulator_init();
	if (error_code == STATUS_ERR_NO_MEMORY) {
		while (true) {
			/* No EEPROM section has been set in the device's fuses */
		}
	}
	else if (error_code != STATUS_OK) {
		/* Erase the emulated EEPROM memory (assume it is unformatted or
		 * irrecoverably corrupt) */
		eeprom_emulator_erase_memory();
		eeprom_emulator_init();
	}
}




int main (void)
{
	system_init();
	configure_tc(); // Configure millis timer
	setConstBases();

	
	for(int i = 0; i < ACCELsamples; ++i){
		AXaverage[i] = 0;
	}
	
	for(int i = 0; i < ACCELsamples; ++i){
		AYaverage[i] = 0;
	}
	
	for(int i = 0; i < ACCELsamples; ++i){
		AZaverage[i] = 0;
	}

	for(int i = 0; i < SLAVE_READ_DATA_LENGTH; ++i){
		I2C_slave_read_buffer[i] = 0;
	}

	for(int i = 0; i < SLAVE_WRITE_DATA_LENGTH; ++i){
		I2C_slave_write_buffer[i] = 0;
	}

	// Initialize local variables used in main
	for(int i = 0; i < 44; ++i){
		send_buffer[i] = 0;
	}
	VescRemoteX = VescRemoteY = 128;

	float heading = 0;
	uint32_t headingTime = 0;
	uint32_t lheadingTime = 0;

	bool lPPM_btn = 0;

	int BLE_TX_INDEX = 0;
	uint16_t BLE_TX_DELAY = 15;
	uint32_t BLE_TX_TIME = 0;
	uint32_t BLE_DUMMY_TIME = 0;


	////////////////////////////////////////////

	// Configure Devices
	configure_ADC();
	configure_LED_PWM();
	configure_port_pins();
	configure_BLE_usart();
	configure_i2c_slave();
	configure_i2c_slave_callbacks();
	initIMU();
	if(!beginIMU()) ERROR_LEDs(0);
	initKalman(0.1, 0.1, 0.5);

	configure_eeprom();
	restore_led_data();
	restore_orientation_controls();
	restore_cal_data(true);
	
	while(1)
	{
		readAccel();
		readGyro();
		readMag();

		// All IMU measurements are corrected to orient power to front and connectors up
		CorrectIMUvalues(ORIENTATION[0], ORIENTATION[1]);


		if(abs(axKalman - ax) < 10000)
		{
			//avgAX = averageAX();
			axKalman = updateKalman(avgAX, ax_kalman);
		}
		//avgAY = averageAY();
		//avgAZ = averageAZ();
		
		getLightSens(&light_sens);
		//uint16_t raw_light = getLightSens();
		//light_sens = updateKalman(raw_light, light_kalman);
		
		ayKalman = updateKalman(ay, ay_kalman);
		azKalman = updateKalman(ax, az_kalman);
		//avgAZ = averageAZ();
		gxKalman = calcGyro(gx);//(uint16_t)(updateKalman(calcGyro(cgx), gx_kalman)*10);
		gyKalman = calcGyro(gy);//(uint16_t)(updateKalman(calcGyro(cgy), gy_kalman)*10);
		gzKalman = calcGyro(gz);//(updateKalman(calcGyro(cgz), gz_kalman));

		if(axKalman > kalmanAX_max)
			kalmanAX_max = axKalman;
		else if(axKalman < kalmanAX_min)
			kalmanAX_min = axKalman;

		if(ayKalman > kalmanAY_max)
			kalmanAY_max = ayKalman;
		else if(ayKalman < kalmanAY_min)
			kalmanAY_min = ayKalman;

		if(azKalman > kalmanAZ_max)
			kalmanAZ_max = azKalman;
		else if(azKalman < kalmanAZ_min)
			kalmanAZ_min = azKalman;
			
		if(gxKalman > kalmanGX_max)
			kalmanGX_max = gxKalman;
		else if(gxKalman < kalmanGX_min)
			kalmanGX_min = gxKalman;

		if(gyKalman > kalmanGY_max)
			kalmanGY_max = gyKalman;
		else if(gyKalman < kalmanGY_min)
			kalmanGY_min = gyKalman;

		if(gzKalman > kalmanAZ_max)
			kalmanGZ_max = gzKalman;
		else if(gzKalman < kalmanGZ_min)
			kalmanGZ_min = gzKalman;

		headingTime = millis();
		if(abs(gzKalman) >= 0.5){
			if(headingTime < lheadingTime){
				heading += (gzKalman) * (((float)(headingTime + (0xFFFFFFFF - lheadingTime)))/1000);
			}
			else
				heading += (gzKalman) * (((float)(headingTime - lheadingTime))/1000);
		}
		lheadingTime = headingTime;
		if(heading < 0)
			heading = 360 + heading;
		else if(heading > 360)
			heading = heading - 360;

		
		if(BLE_TX_TIME>millis())
			BLE_TX_TIME = 0;

		if(SEND_CONTINUOUS && app_remote_check == 0 &&((millis()-BLE_TX_TIME) >= BLE_TX_DELAY))
		{
			switch(BLE_TX_INDEX){
				case 0:
					send_buffer[0] = 0x11;
					send_buffer[1] = battery_current & 0xFF;
					send_buffer[2] = (battery_current & 0xFF00) >> 8;
					send_buffer[3] = 0x12;
					send_buffer[4] = battery_voltage;
					send_buffer[5] = (battery_voltage & 0xFF00) >> 8;
					send_buffer[6] = 0x13;
					send_buffer[7] = motor_current;
					send_buffer[8] = (motor_current & 0xFF00) >> 8;
					send_buffer[9] = 0x14;
					send_buffer[10] = vesc_temp;
					send_buffer[11] = (vesc_temp & 0xFF00) >> 8;
					send_buffer[12] = 0x15;
					send_buffer[13] = duty_cycle;
					send_buffer[14] = 0x16;
					send_buffer[15] = (motor_rpm & 0xFF);
					send_buffer[16] = (motor_rpm & 0xFF00) >> 8;
					send_buffer[17] = (motor_rpm & 0xFF0000) >> 16;
					usart_write_buffer_wait(&usart_instance, send_buffer, 18);
					break;
				case 1:
					send_buffer[0] = 0x17;
					send_buffer[1] = (mAh_used & 0xFF);
					send_buffer[2] = (mAh_used & 0xFF00) >> 8;
					send_buffer[3] = (mAh_used & 0xFF0000) >> 16;
					send_buffer[4] = 0x18;
					send_buffer[5] = (mAh_charged & 0xFF);
					send_buffer[6] = (mAh_charged & 0xFF00) >> 8;
					send_buffer[7] = (mAh_charged & 0xFF0000) >> 16;
					send_buffer[8] = 0x19;
					send_buffer[9] = (Wh_used & 0xFF);
					send_buffer[10] = (Wh_used & 0xFF00) >> 8;
					send_buffer[11] = (Wh_used & 0xFF0000) >> 16;
					send_buffer[12] = 0x1A;
					send_buffer[13] = (Wh_charged & 0xFF);
					send_buffer[14] = (Wh_charged & 0xFF00) >> 8;
					send_buffer[15] = (Wh_charged & 0xFF0000) >> 16;
					send_buffer[16] = 0x1B;
					send_buffer[17] = error_codes;
					send_buffer[18] = 0x21;
					send_buffer[19] = VescRemoteX;
					usart_write_buffer_wait(&usart_instance, send_buffer, 20);
					break;
				case 2:
					send_buffer[0] = 0x2E;
					send_buffer[1] = ((uint16_t)(heading*10) & 0xFF); // Heading
					send_buffer[2] = ((uint16_t)(heading*10) & 0xFF00) >> 8; // Heading
					send_buffer[3] = 0x22;
					send_buffer[4] = VescRemoteY;
					send_buffer[5] = 0x23;
					send_buffer[6] = (VescRemoteButton | (REMOTE_TYPE << 1));
					send_buffer[7] = 0x24;
					send_buffer[8] = ((uint16_t)ax & 0xFF); // Accel X
					send_buffer[9] = ((uint16_t)ax & 0xFF00) >> 8; // Accel X
					send_buffer[10] = 0x25;
					send_buffer[11] = ((uint16_t)ayKalman & 0xFF); // Accel Y
					send_buffer[12] = ((uint16_t)ayKalman & 0xFF00) >> 8; // Accel Y
					send_buffer[13] = 0x26;
					send_buffer[14] = ((uint16_t)azKalman & 0xFF); // Accel Z
					send_buffer[15] = ((uint16_t)azKalman & 0xFF00) >> 8; // Accel Z
					send_buffer[16] = 0x27;
					send_buffer[17] = ((uint16_t)(gxKalman*10) & 0xFF); // Gyro X
					send_buffer[18] = ((uint16_t)(gxKalman*10) & 0xFF00) >> 8; // Gyro X
					usart_write_buffer_wait(&usart_instance, send_buffer, 19);
					break;
				case 3:
					send_buffer[0] = 0x28;
					send_buffer[1] = ((uint16_t)(gyKalman*10) & 0xFF); // Gyro Y
					send_buffer[2] = ((uint16_t)(gyKalman*10) & 0xFF00) >> 8; // Gyro Y
					send_buffer[3] = 0x29;
					send_buffer[4] = ((uint16_t)(gzKalman*10) & 0xFF); // Gyro Z
					send_buffer[5] = ((uint16_t)(gzKalman*10) & 0xFF00) >> 8; // Gyro Z
					send_buffer[6] = 0x2A;
					send_buffer[7] = ((mx) & 0xFF); // Compass X
					send_buffer[8] = (mx & 0xFF00) >> 8; // Compass X
					send_buffer[9] = 0x2B;
					send_buffer[10] = (my & 0xFF); // Compass Y
					send_buffer[11] = (my & 0xFF00) >> 8; // Compass Y
					send_buffer[12] = 0x2C;
					send_buffer[13] = (mz & 0xFF); // Compass Z
					send_buffer[14] = (mz & 0xFF00) >> 8; // Compass Z
					send_buffer[15] = 0x2D;
					send_buffer[16] = ((int)(light_sens) & 0xFF); // Light Sensor
					send_buffer[17] = ((int)(light_sens) & 0xFF00) >> 8; // Light Sensor
					send_buffer[18] = 0xDE;
					usart_write_buffer_wait(&usart_instance, send_buffer, 19);
					break;
			}
			BLE_TX_INDEX++;
			if(BLE_TX_INDEX > 3)
				BLE_TX_INDEX = 0;

			BLE_TX_TIME = millis(); // Placed at end of transmit to provide accurate message timing
		}
		else
		{
			// Use a dummy delay that mimics the delay of the BLE send
			// commands to keep the timing of the light sections the same
			while((millis()-BLE_DUMMY_TIME) < BLE_TX_DELAY + (1.0/BLE_BAUD)*20.0){}
			BLE_DUMMY_TIME = millis();
		}

		
		////////////////////////////   Handle Limits Request   ////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_LIMITS)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			send_buffer[0] = 0x41;
			send_buffer[1] = motor_current_max; // Motor max current
			send_buffer[2] = 0x42;
			send_buffer[3] = motor_current_min; // Motor min current
			send_buffer[4] = 0x43;
			send_buffer[5] = battery_current_max; // Battery max current
			send_buffer[6] = 0x44;
			send_buffer[7] = battery_current_min; // Battery min current
			send_buffer[8] = 0x45;
			send_buffer[9] = absolute_max_current; // Battery ABS max current
			send_buffer[10] = 0x46;
			send_buffer[11] = battery_voltage_max; // Battery max voltage
			usart_write_buffer_wait(&usart_instance, send_buffer, 12);
			
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			send_buffer[0] = 0x48;
			send_buffer[1] = voltage_cutoff_start; // Battery cuttoff voltage start
			send_buffer[2] = 0x49;
			send_buffer[3] = voltage_cutoff_end; // Battery cuttoff voltage end
			send_buffer[4] = 0x4A;
			send_buffer[5] = (motor_rpm_max & 0xFF);
			send_buffer[6] = (motor_rpm_max & 0xFF00) >> 8;
			send_buffer[7] = (motor_rpm_max & 0xFF0000) >> 16;
			send_buffer[8] = 0x4B;
			send_buffer[9] = (motor_rpm_min & 0xFF);
			send_buffer[10] = (motor_rpm_min & 0xFF00) >> 8;
			send_buffer[11] = (motor_rpm_min & 0xFF0000) >> 16;
			usart_write_buffer_wait(&usart_instance, send_buffer, 12);

			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			send_buffer[0] = 0x4C;
			send_buffer[1] = (max_rpm_full & 0xFF);
			send_buffer[2] = (max_rpm_full & 0xFF00) >> 8;
			send_buffer[3] = (max_rpm_full & 0xFF0000) >> 16;
			send_buffer[4] = 0x4D;
			send_buffer[5] = (max_rpm_full_cc & 0xFF);
			send_buffer[6] = (max_rpm_full_cc & 0xFF00) >> 8;
			send_buffer[7] = (max_rpm_full_cc & 0xFF0000) >> 16;
			send_buffer[8] = 0x4E;
			send_buffer[9] = mosfet_temp_start; // MOSFET Temp Start
			send_buffer[10] = 0x4F;
			send_buffer[11] = mosfet_temp_end; // MOSFET Temp End
			usart_write_buffer_wait(&usart_instance, send_buffer, 12);

			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			send_buffer[0] = 0x50;
			send_buffer[1] = motor_temp_start; // Motor Temp Start
			send_buffer[2] = 0x51;
			send_buffer[3] = motor_temp_end; // Motor Temp End
			send_buffer[4] = 0x52;
			send_buffer[5] = duty_cycle_max; // Max Duty
			send_buffer[6] = 0x53;
			send_buffer[7] = duty_cycle_min; // Min Duty
			send_buffer[8] = 0x47;
			send_buffer[9] = battery_voltage_min; // REPLACE with Battery min voltage
			usart_write_buffer_wait(&usart_instance, send_buffer, 10);

			SEND_LIMITS = 0;
			SEND_CONTINUOUS = 1;
		}

		
		////////////////////////   Handle Sensor Params Request   /////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_SENSORS)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			send_buffer[0] = 0x61;
			send_buffer[1] = ((uint16_t)(err_estimate[ax_kalman]) & 0xFF); // Accel X estimated error
			send_buffer[2] = 0x62;
			send_buffer[3] = ((uint16_t)(err_estimate[ay_kalman]) & 0xFF); // AccelY estimated error
			send_buffer[4] = 0x63;
			send_buffer[5] = ((uint16_t)(err_estimate[az_kalman]) & 0xFF); // Accel Z estimated error
			send_buffer[6] = 0x64;
			send_buffer[7] = ((uint16_t)(err_estimate[gx_kalman]) & 0xFF); // Gyro X estimated error
			send_buffer[8] = 0x65;
			send_buffer[9] = ((uint16_t)(err_estimate[gy_kalman]) & 0xFF); // Gyro Y estimated error
			send_buffer[10] = 0x66;
			send_buffer[11] = ((uint16_t)(err_estimate[gz_kalman]) & 0xFF); // Gyro Z estimated error
			send_buffer[12] = 0x67;
			send_buffer[13] = ((uint16_t)(err_estimate[light_kalman]) & 0xFF); // Light Sensor estimated error
			usart_write_buffer_wait(&usart_instance, send_buffer, 14);
		
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			send_buffer[0] = 0x68;
			send_buffer[1] = ((uint16_t)(q[ax_kalman]*100.0) & 0xFF); // Accel X Sensitivity
			send_buffer[2] = 0x69;
			send_buffer[3] = ((uint16_t)(q[ay_kalman]*100.0) & 0xFF); // Accel Y Sensitivity
			send_buffer[4] = 0x6A;
			send_buffer[5] = ((uint16_t)(q[az_kalman]*100.0) & 0xFF); // Accel Z Sensitivity
			send_buffer[6] = 0x6B;
			send_buffer[7] = ((uint16_t)(q[gx_kalman]*100.0) & 0xFF); // Gyro X Sensitivity
			send_buffer[8] = 0x6C;
			send_buffer[9] = ((uint16_t)(q[gy_kalman]*100.0) & 0xFF); // Gyro Y Sensitivity
			send_buffer[10] = 0x6D;
			send_buffer[11] = ((uint16_t)(q[gz_kalman]*100.0) & 0xFF); // Gyro Z Sensitivity
			send_buffer[12] = 0x6E;
			send_buffer[13] = ((uint16_t)(q[light_kalman]*100.0) & 0xFF); // Light Sensitivity
			usart_write_buffer_wait(&usart_instance, send_buffer, 14);

			SEND_SENSORS = 0;
			SEND_CONTINUOUS = 1;
		}


		//////////////////////////   Handle LED Params Request   //////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_LED_CHARS)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			uint8_t led_mode_switches = ((light_mode << 4) | (HEADLIGHTS << 3) | (SIDELIGHTS << 2) | (LIGHT_CONTROLLED << 1) | IMU_CONTROLED);

			// Global LED Settings
			send_buffer[0] = 0x31;
			send_buffer[1] = led_mode_switches; // Current switch states
			// Static
			send_buffer[2] = 0x32;
			send_buffer[3] = (uint8_t)((float)Static_RGB.LR / 655.35);
			send_buffer[4] = (uint8_t)((float)Static_RGB.LG / 655.35);
			send_buffer[5] = (uint8_t)((float)Static_RGB.LB / 655.35);
			send_buffer[6] = (uint8_t)((float)Static_RGB.RR / 655.35);
			send_buffer[7] = (uint8_t)((float)Static_RGB.RG / 655.35);
			send_buffer[8] = (uint8_t)((float)Static_RGB.RB / 655.35);
			// Color Cycle
			send_buffer[9] = 0x33;
			send_buffer[10] = (uint8_t)(RateSens[MODE_COLOR_CYCLE] * 100);
			send_buffer[11] = (uint8_t)(Brightness[MODE_COLOR_CYCLE] * 100);
			// Compass Cycle
			send_buffer[12] = 0x34;
			send_buffer[13] = (uint8_t)(Brightness[MODE_COMPASS_CYCLE] * 100);
			// Throttle Based
			send_buffer[14] = 0x35;
			send_buffer[15] = (uint8_t)(RateSens[MODE_THROTTLE] * 100);
			send_buffer[16] = (uint8_t)(Brightness[MODE_THROTTLE] * 100);
			// RPM Based
			send_buffer[17] = 0x36;
			send_buffer[18] = (uint8_t)(RateSens[MODE_RPM_CYCLE] * 100);
			usart_write_buffer_wait(&usart_instance, send_buffer, 19);
			
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// X Accel Based
			send_buffer[0] = 0x37;
			send_buffer[1] = (uint8_t)(RateSens[MODE_X_ACCEL] * 100);
			// Y Accel Based
			send_buffer[2] = 0x38;
			send_buffer[3] = (uint8_t)(RateSens[MODE_Y_ACCEL] * 100);
			// Custom
			uint8_t color_bright_base = (ColorBase[MODE_CUSTOM] << 4) | BrightBase[MODE_CUSTOM];
			send_buffer[4] = 0x39;
			send_buffer[5] = color_bright_base;
			send_buffer[6] = RateBase[MODE_CUSTOM];
			send_buffer[7] = (uint8_t)((float)Custom_RGB.LR / 655.35);
			send_buffer[8] = (uint8_t)((float)Custom_RGB.LG / 655.35);
			send_buffer[9] = (uint8_t)((float)Custom_RGB.LB / 655.35);
			send_buffer[10] = (uint8_t)((float)Custom_RGB.RR / 655.35);
			send_buffer[11] = (uint8_t)((float)Custom_RGB.RG / 655.35);
			send_buffer[12] = (uint8_t)((float)Custom_RGB.RB / 655.35);
			send_buffer[13] = (uint8_t)(RateSens[MODE_CUSTOM] * 100);
			send_buffer[14] = (uint8_t)(Brightness[MODE_CUSTOM] * 100);
			usart_write_buffer_wait(&usart_instance, send_buffer, 15);

			SEND_LED_CHARS = 0;
			SEND_CONTINUOUS = 1;
		}
		

		//////////////////////////   Handle Orientation Request   /////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_ORIENTAION)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			send_buffer[0] = 0x71;
			send_buffer[1] = ORIENTATION[0]; // Connectors Orientation
			send_buffer[2] = ORIENTATION[1]; // Power Orientation
			usart_write_buffer_wait(&usart_instance, send_buffer, 3);


			SEND_ORIENTAION = 0;
			SEND_CONTINUOUS = 1;
		}


		///////////////////////////   Handle Controls Request   ///////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_CONTROLS)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			send_buffer[0] = 0x81;
			send_buffer[1] = (uint8_t)((AUX_ENABLED << 7) | (TURN_ENABLED << 6) | auxControlType);
			send_buffer[2] = (uint8_t)auxTimedDuration;
			send_buffer[3] = (uint8_t)((single_aux_control << 4) | single_all_control);
			send_buffer[4] = (uint8_t)((single_head_control << 4) | single_side_control);
			send_buffer[5] = (uint8_t)((single_down_control << 4) | single_up_control);
			send_buffer[6] = (uint8_t)((dual_aux_control << 4) | dual_all_control);
			send_buffer[7] = (uint8_t)((dual_head_control << 4) | dual_side_control);
			send_buffer[8] = (uint8_t)((dual_down_control << 4) | dual_up_control);
			usart_write_buffer_wait(&usart_instance, send_buffer, 9);

			SEND_CONTROLS = 0;
			SEND_CONTINUOUS = 1;
		}
		

		////////////////////////////////   LED Controls   /////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		HandleUserInput();


		//////////////////////////////////   LED MODES   //////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(sensorControl() && LIGHTS_ON){
			if(SIDELIGHTS && lightControlSide()){

			// Variable for controlling the brightness of the side LEDS
			// brightness is a value from 0 to 1
			float output_brightness = 0;

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
					output_rate_sens = (((float)motor_rpm)/motor_rpm_max);
					break;
				}
				case RATE_THROTTLE:
				{
					float temp_y;
					if(app_remote_check  && REMOTE_TYPE < 2)
						temp_y = AppRemoteY-43;
					else
						temp_y = VescRemoteY-43;
					if(temp_y < 0 )
						temp_y = 255+temp_y;
					output_rate_sens = temp_y/255.0;
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
					output_brightness = (((float)motor_rpm)/motor_rpm_max);
					break;
				}
				case BRIGHT_THROTTLE:
				{
					float temp_y;
					if(app_remote_check && REMOTE_TYPE < 2)
						temp_y = AppRemoteY-43;
					else
						temp_y = VescRemoteY-43;
					if(temp_y < 0 )
						temp_y = 255+temp_y;
					output_brightness = temp_y/255.0;
					break;
				}
				case BRIGHT_X_ACCEL:
				{
					if(axKalman < 0)
						output_brightness = axKalman/kalmanAX_min;
					else
						output_brightness = axKalman/kalmanAX_max;
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
			}
		
			if(output_brightness < 0)
				output_brightness = 0;
			else if(output_brightness > 1)
				output_brightness = 1;

			switch(ColorBase[light_mode]){ // Set the color of the side LEDs
				case COLOR_STATIC:
				{
					if(light_mode == MODE_STATIC)
						RGB_Ouptut = Static_RGB;
					else if(light_mode == MODE_CUSTOM)
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
					float temp_y;
					if(app_remote_check && REMOTE_TYPE < 2)
						temp_y = AppRemoteY-43;
					else
						temp_y = VescRemoteY-43;

					if(temp_y < 0 )
						temp_y = 255+temp_y;
					cycle_index = (int)(((((float)0x0FFFF) * 3.0) / 255.0) * temp_y) % 0x0FFFF;
					cycle = (int)(((((float)0x0FFFF) * 3.0) / 255.0) * temp_y) / 0x0FFFF;
					upColor = cycle_index * output_brightness;
					downColor = (0xFFFF-cycle_index) * output_brightness;

					RGB_Ouptut = setCycleColor(upColor, downColor, cycle);

					break;
				}
				case COLOR_RPM:	
				{				
					cycle_index = (int)(((((float)0x0FFFF) * 3.0) / (float)motor_rpm_max) * (float)motor_rpm) % 0x0FFFF;
					cycle = (int)(((((float)0x0FFFF) * 3.0) / (float)motor_rpm_max) * (float)motor_rpm) / 0x0FFFF;
					upColor = cycle_index * output_brightness;
					downColor = (0xFFFF-cycle_index) * output_brightness;

					RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
					break;
				}
				case COLOR_X_ACCEL:
				{
					if(axKalman < 0){
						cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanAX_min) * axKalman) % 0x0FFFF;
						cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanAX_min) * axKalman) / 0x0FFFF;
					} else {
						cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanAX_max) * axKalman) % 0x0FFFF;
						cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanAX_max) * axKalman) / 0x0FFFF;
					}

					upColor = cycle_index * output_brightness;
					downColor = (0xFFFF-cycle_index) * output_brightness;

					RGB_Ouptut = setCycleColor(upColor, downColor, cycle);
					if (axKalman < 0){
						RGB_Ouptut.RR = 0;
						RGB_Ouptut.RG = 0;
						RGB_Ouptut.RB = 0;
					} else {
						RGB_Ouptut.RR = 0;
						RGB_Ouptut.RG = 0;
						RGB_Ouptut.RB = 0;
					}

					break;
				}
				case COLOR_Y_ACCEL:
				{
					if(ayKalman < 0){
						cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanAY_min) * ayKalman) % 0x0FFFF;
						cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanAY_min) * ayKalman) / 0x0FFFF;
						} else {
						cycle_index = (int)(((((float)0x0FFFF) * 3.0) / kalmanAY_max) * ayKalman) % 0x0FFFF;
						cycle = (int)(((((float)0x0FFFF) * 3.0) / kalmanAY_max) * ayKalman) / 0x0FFFF;
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
			setLeftRGB(RGB_Ouptut.LR,RGB_Ouptut.LG,RGB_Ouptut.LB);
			setRightRGB(RGB_Ouptut.RR,RGB_Ouptut.RG,RGB_Ouptut.RB);
			}
			else {
				if(!TurnSignalOn) {
					setLeftRGB(0, 0, 0);
					setRightRGB(0, 0, 0);
				}
			}


			/////////////// Control the head and tail lights //////////////////
			if(HEADLIGHTS && lightControlHead()){
				setWhite(0xFFFF);

				float temp_y;
				if(app_remote_check && REMOTE_TYPE < 2)
					temp_y = AppRemoteY;
				else
					temp_y = VescRemoteY;

				if(temp_y < 120){
					float brake_temp = (((0xFFFF-brake_offset)/120)*(120-temp_y))+brake_offset;
					setRed(brake_temp);
				}
				else
					setRed(brake_offset);
			}
			else{
				setWhite(0);
				setRed(0);
			}
		} else {
			setWhite(0);
			setRed(0);
			setLeftRGB(0,0,0);
			setRightRGB(0,0,0);
		}//*/
	}
}





/* ///// LED REFERNCE //////
LB = tcc1[0]
LG = tcc0[3]
Head = tcc1[1]
LR = tcc2[1]
RG = tcc0[2]
RR = tcc2[0]
Tail = tcc0[1]
RB = tcc0[0]
*/


void getLightSens(uint16_t* light_val) {
	adc_start_conversion(&adc1);
	while(adc_get_status(&adc1) != ADC_STATUS_RESULT_READY);
	adc_read(&adc1, &light_val);
	adc_clear_status(&adc1, ADC_STATUS_RESULT_READY);
}

int16_t averageAX(){
	AXtotal -= AXaverage[ACCELsamples-1];
	for(int i = ACCELsamples-1; i > 0; --i){
		AXaverage[i] = AXaverage[i-1];
	}
	AXtotal += ax;
	AXaverage[0] = ax;

	return (int16_t)(AXtotal/ACCELsamples);
}

int16_t averageAY(){
	AYtotal -= AYaverage[ACCELsamples-1];
	for(int i = ACCELsamples-1; i > 0; --i){
		AYaverage[i] = AYaverage[i-1];
	}
	AYtotal += ay;
	AYaverage[0] = ay;

	return (int16_t)(AYtotal/ACCELsamples);
}

int16_t averageAZ(){
		AZtotal -= AZaverage[ACCELsamples-1];
		for(int i = ACCELsamples-1; i > 0; --i){
			AZaverage[i] = AZaverage[i-1];
		}
		AZtotal += azKalman;
		AZaverage[0] = azKalman;

		return (int16_t)(AZtotal/ACCELsamples);
}

char sensorControl() {
static long count = 0;
static bool result = 1;
	if(IMU_CONTROLED){
		if((ayKalman >= 750 && result) || (ayKalman < 750 && !result))
			count++;
		else
			count = 0;

		if(count > 6)
			result = !result;

		return result;
	}
	else
		return 1;
}



char lightControlSide() {
	// TO BE IMPLEMENTED

	return true;
}

char lightControlHead() {
	// TO BE IMPLEMENTED

	return true;
}

void initKalman(float meas, float est, float _q)
{
	for(int i = 0; i < KalmanArraySize; i++){
		err_measure[i] = meas;
		err_estimate[i] = est;
		q[i] = _q;
		current_estimate[i] = 0;
		last_estimate[i] = 0;
		kalman_gain[i] = 0;
	}

	err_measure[ax_kalman] = 15;
	err_estimate[ax_kalman] = 15;
	q[ax_kalman] = 0.3;

	err_measure[ay_kalman] = 15;
	err_estimate[ay_kalman] = 15;
	q[ay_kalman] = 0.3;

// 	err_measure[ay_kalman] = 20;
// 	err_estimate[ay_kalman] = 20;
// 	q[ay_kalman] = 0.8;

	err_measure[az_kalman] = 30;
	err_estimate[az_kalman] = 30;
	q[az_kalman] = 0.3;

// 	err_measure[gx_kalman] = 3;
// 	err_estimate[gx_kalman] = 3;
// 	q[gx_kalman] = 0.9;
// 
// 	err_measure[gy_kalman] = 3;
// 	err_estimate[gy_kalman] = 3;
// 	q[gy_kalman] = 0.9;
// 	
// 	err_measure[gz_kalman] = 0.1;
// 	err_estimate[gz_kalman] = 1;
// 	q[gz_kalman] = 0.99;

	err_measure[light_kalman] = 200;
	err_estimate[light_kalman] = 200;
	q[light_kalman] = 0.008;
}

float updateKalman(float meas, int kalmanIndex)
{
	  kalman_gain[kalmanIndex] = err_estimate[kalmanIndex]/(err_estimate[kalmanIndex] + err_measure[kalmanIndex]);
	  kalman_gain[kalmanIndex] = max(kalman_gain[kalmanIndex],0.015);
	  current_estimate[kalmanIndex] = last_estimate[kalmanIndex] + kalman_gain[kalmanIndex] * (meas - last_estimate[kalmanIndex]);
	  err_estimate[kalmanIndex] =  (1.0 - kalman_gain[kalmanIndex])*err_estimate[kalmanIndex] + abs(last_estimate[kalmanIndex]-current_estimate[kalmanIndex])*q[kalmanIndex];
	  last_estimate[kalmanIndex]=current_estimate[kalmanIndex];

	  return current_estimate[kalmanIndex];
}