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
#include "Config.h"
#include "IMU_Vars.h"
#include "IMU_Functions.h"
#include "Timing.h"
#include "Remote_Vars.h"
#include "ESC_Vars.h"
#include "LED_Vars.h"
#include "LED_Functions.h"
#include "Controls.h"
#include "EEPROM_Functions.h"#include "VESC_UART.h"#include "BLE_UART.h"#include "APA102.h"#include "WS2812.h"#include "SK9822.h"
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


/////   Function Definitions and System Structs   //////
////////////////////////////////////////////////////////
struct i2c_slave_module i2c_slave_instance;

// Initialization functions
void configure_ADC(void);
void configure_port_pins(void);
void configure_i2c_slave(void);
void number_to_string(uint32_t, char *);
void configure_i2c_slave_callbacks(void);
void i2c_write_request_callback(struct i2c_slave_module *const module);
void i2c_read_request_callback(struct i2c_slave_module *const module);
void configure_eeprom(void);


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
	config_adc.positive_input = ADC_POSITIVE_INPUT_PIN17;
	config_adc.freerunning = DISABLE;
	config_adc.run_in_standby = true;
	config_adc.left_adjust = false;
	adc_init(&adc1, ADC, &config_adc);
	adc_enable(&adc1);
}

// Configure the LED selection port as output
void configure_port_pins(void)
{
	struct port_config config_port_pin;
	port_get_config_defaults(&config_port_pin);
	
	if(RGB_led_type == RGB_DIGITAL_APA102 || RGB_led_type == RGB_DIGITAL_SK9822){
		config_port_pin.powersave = false;
		config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
		port_pin_set_config(L_GND, &config_port_pin);
		port_pin_set_output_level(L_GND,false);

		config_port_pin.powersave = false;
		config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
		port_pin_set_config(R_GND, &config_port_pin);
		port_pin_set_output_level(L_GND,false);
	}
	
	config_port_pin.powersave = false;
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(PIN_PA06E_TCC1_WO0, &config_port_pin);
	port_pin_set_output_level(PIN_PA06E_TCC1_WO0,false);

	config_port_pin.powersave = false;
	config_port_pin.input_pull = PORT_PIN_PULL_UP;
	config_port_pin.direction = PORT_PIN_DIR_INPUT;
	port_pin_set_config(PPM_IN, &config_port_pin);
	
	config_port_pin.powersave = false;
	config_port_pin.input_pull = PORT_PIN_PULL_NONE;
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(HORN_PIN, &config_port_pin);
	port_pin_set_output_level(HORN_PIN,true);

#if  defined(HW_4v0) || defined(HW_4v1)
	config_port_pin.powersave = false;
	config_port_pin.input_pull = PORT_PIN_PULL_NONE;
	config_port_pin.direction = PORT_PIN_DIR_OUTPUT;
	port_pin_set_config(STAT_LED, &config_port_pin);
	port_pin_set_output_level(STAT_LED,false);
#endif
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
			mcconf_limits.motor_current_max = I2C_slave_read_buffer[1];
			mcconf_limits.motor_current_min = I2C_slave_read_buffer[2];
			mcconf_limits.input_current_max = I2C_slave_read_buffer[3];
			mcconf_limits.input_current_min = I2C_slave_read_buffer[4];
			mcconf_limits.abs_current_max = I2C_slave_read_buffer[5];
			mcconf_limits.max_vin = I2C_slave_read_buffer[6];
			mcconf_limits.min_vin = I2C_slave_read_buffer[7];
			mcconf_limits.battery_cut_start = I2C_slave_read_buffer[8];
			mcconf_limits.battery_cut_end = I2C_slave_read_buffer[9];
			mcconf_limits.max_erpm = (I2C_slave_read_buffer[10] | (I2C_slave_read_buffer[11] << 8) | (I2C_slave_read_buffer[12] << 16));
			mcconf_limits.min_erpm = (I2C_slave_read_buffer[13] | (I2C_slave_read_buffer[14] << 8) | (I2C_slave_read_buffer[15] << 16));
			mcconf_limits.max_erpm_fbrake = (I2C_slave_read_buffer[16] | (I2C_slave_read_buffer[17] << 8) | (I2C_slave_read_buffer[18] << 16));
			mcconf_limits.max_erpm_fbrake_cc = (I2C_slave_read_buffer[19] | (I2C_slave_read_buffer[20] << 8) | (I2C_slave_read_buffer[21] << 16));
			mcconf_limits.temp_fet_start = I2C_slave_read_buffer[22];
			mcconf_limits.temp_fet_end = I2C_slave_read_buffer[23];
			mcconf_limits.temp_motor_start = I2C_slave_read_buffer[24];
			mcconf_limits.temp_motor_end = I2C_slave_read_buffer[25];
			mcconf_limits.max_duty = I2C_slave_read_buffer[26];
			mcconf_limits.min_duty = I2C_slave_read_buffer[27];
			SEND_LIMITS = 1;
			SEND_CONTINUOUS = 0;
		} else if(I2C_slave_read_buffer[0] == 0xDD && I2C_slave_read_buffer[29] == 0xAD) {
			latest_vesc_vals.avg_input_current = I2C_slave_read_buffer[1];
			latest_vesc_vals.avg_input_current += (I2C_slave_read_buffer[2] << 8);
			latest_vesc_vals.INPUT_VOLTAGE = I2C_slave_read_buffer[3];
			latest_vesc_vals.INPUT_VOLTAGE += (I2C_slave_read_buffer[4] << 8);
			latest_vesc_vals.avg_motor_current = I2C_slave_read_buffer[5];
			latest_vesc_vals.avg_motor_current += (I2C_slave_read_buffer[6] << 8);
			latest_vesc_vals.temp_fet_filtered = I2C_slave_read_buffer[7];
			latest_vesc_vals.temp_fet_filtered += (I2C_slave_read_buffer[8] << 8);
			latest_vesc_vals.duty_cycle = I2C_slave_read_buffer[9];
			latest_vesc_vals.rpm = (I2C_slave_read_buffer[10] | (I2C_slave_read_buffer[11] << 8) | (I2C_slave_read_buffer[12] << 16));
			latest_vesc_vals.amp_hours = (I2C_slave_read_buffer[13] | (I2C_slave_read_buffer[14] << 8) | (I2C_slave_read_buffer[15] << 16));
			latest_vesc_vals.amp_hours_charged = (I2C_slave_read_buffer[16] | (I2C_slave_read_buffer[17] << 8) | (I2C_slave_read_buffer[18] << 16));
			latest_vesc_vals.watt_hours = (I2C_slave_read_buffer[19] | (I2C_slave_read_buffer[20] << 8) | (I2C_slave_read_buffer[21] << 16));
			latest_vesc_vals.watt_hours_charged = (I2C_slave_read_buffer[22] | (I2C_slave_read_buffer[23] << 8) | (I2C_slave_read_buffer[24] << 16));
			remote_x = I2C_slave_read_buffer[25];
			remote_y = I2C_slave_read_buffer[26];
			remote_type = (I2C_slave_read_buffer[27] & 0x6) >> 1; // needs to change to conform with new use of remote_type
			remote_btn_state = I2C_slave_read_buffer[27] & 0x1;
			latest_vesc_vals.fault = I2C_slave_read_buffer[28];
		}
}

uint8_t app_remote_check = 0;
void i2c_read_request_callback(struct i2c_slave_module *const module)
{
	I2C_slave_write_buffer[0] = AppRemoteY;
	I2C_slave_write_buffer[1] = GET_LIMITS;
	I2C_slave_write_buffer[2] = app_remote_check;

	/* Init i2c packet. */
	packet.data_length = SLAVE_WRITE_DATA_LENGTH;
	packet.data        = I2C_slave_write_buffer;
	/* Write buffer to master */
	i2c_slave_write_packet_job(module, &packet);
	NEW_REMOTE_DATA = false;

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
void clockInit(void);

void clockInit(void)
{
	SYSCTRL->OSC8M.bit.PRESC = 0;                          // no prescaler (is 8 on reset)
	SYSCTRL->OSC8M.reg |= 1 << SYSCTRL_OSC8M_ENABLE_Pos;   // enable source

	GCLK->GENDIV.bit.ID = 0x01;                            // select GCLK_GEN[1]
	GCLK->GENDIV.bit.DIV = 0;                              // no prescaler

	GCLK->GENCTRL.bit.ID = 0x01;                           // select GCLK_GEN[1]
	GCLK->GENCTRL.reg |= GCLK_GENCTRL_SRC_OSC8M;           // OSC8M source
	GCLK->GENCTRL.bit.GENEN = 1;                           // enable generator

	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM4_CORE;      // SERCOM0 peripheral channel
	GCLK->CLKCTRL.reg |= GCLK_CLKCTRL_GEN_GCLK1;           // select source GCLK_GEN[1]
	GCLK->CLKCTRL.bit.CLKEN = 1;                           // enable generic clock

	PM->APBCSEL.bit.APBCDIV = 0;                           // no prescaler
	PM->APBCMASK.bit.SERCOM0_ = 1;                         // enable SERCOM0 interface
}


int main (void)
{
	system_init();
	//clockInit();
	configure_tc(); // Configure millis timer
	
	// Configure Devices
	configure_ADC();

	configure_eeprom();
	restore_led_data();
	restore_orientation_controls_remote_esc_lights();
	
	// Configure The button input pin and interrupt handlers for pulse width measurement
	configure_port_pins();
	config_eic();    // Configure the external interruption
	config_evsys();  // Configure the event system
	config_gpio();   // Configure the dedicated pin

	//ERROR_LEDs(0);
#if  defined(HW_4v0) || defined(HW_4v1)
	port_pin_set_output_level(STAT_LED, true);
#endif
	configure_BLE_module(); // Blocks when no BLE module is installed
	initIMU();
	restore_cal_data(true);
	if(!beginIMU()) ERROR_LEDs(0);
#if  defined(HW_4v0) || defined(HW_4v1)
	port_pin_set_output_level(STAT_LED, false);
#endif

	initKalman(0.1, 0.1, 0.5);
	setConstBases();

	if(esc_comms == COMMS_I2C){
		configure_i2c_slave();
		configure_i2c_slave_callbacks();
	} else if(esc_comms == COMMS_UART){
		// Nothing to do
	}

	configure_LED_PWM();
	if(RGB_led_type == RGB_DIGITAL_APA102 || RGB_led_type == RGB_DIGITAL_SK9822){
		configure_APA_SPI();
	}
	
	//ERROR_LEDs(1); // Uncomment for testing SAM-BA and LED output functionality
	
	////////////////////////////////////////////

	configured_comms = esc_comms;
	configured_RGB_led_type = RGB_led_type;
	current_led_num = led_num;

	memset(AXaverage, 0, ACCELXYsamples);
	memset(AYaverage, 0, ACCELXYsamples);
	memset(AZaverage, 0, ACCELZsamples);

	memset(I2C_slave_read_buffer, 0, SLAVE_READ_DATA_LENGTH);
	memset(I2C_slave_write_buffer, 0, SLAVE_WRITE_DATA_LENGTH);
	
	memset(ble_write_buffer, 0, BLE_WRITE_BUF_SIZE);

	// Initialize local variables used in main
	VescRemoteX = VescRemoteY = 128;

	int BLE_TX_INDEX = 0;
	uint16_t BLE_TX_DELAY = 15;
	uint32_t BLE_TX_TIME = 0;
	//uint32_t BLE_DUMMY_TIME = 0;

	mcconf_limits.max_erpm = 1000000;
	mcconf_limits.min_erpm = -1000000;

	////////////////////////////////////////////
	
	while(1)
	{
		// Reset the module if PA15 is pulled low
		if(port_pin_get_input_level(BOOT_BTN)==false)
			NVIC_SystemReset();

		// Handle BLE Communication
		read_ble_packet();

		if(configured_comms != esc_comms)
		{
			// TODO: Deconfigure old comms and configure new comms
			ERROR_LEDs(5);
		}
		
		if(configured_RGB_led_type != RGB_led_type)
		{
			if(configured_RGB_led_type == RGB_ANALOG){
				
			}
			// TODO: Reconfigure for new LED type
			ERROR_LEDs(5);
		}

		if((configured_RGB_led_type == RGB_DIGITAL_APA102 || configured_RGB_led_type == RGB_DIGITAL_SK9822) && current_led_num != led_num)
		{
			for(uint16_t i = 0; i < MAX_LEDCOUNT; i++)
			{
				L_SPI_send_buf[(i*4)+4] = R_SPI_send_buf[(i*4)+4] = (0b11100000 | 0);
				L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = 0;
				L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = 0;
				L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = 0;
			}

			L_APA_write(MAX_LEDCOUNT);
			R_APA_write(MAX_LEDCOUNT);
			current_led_num = led_num;
		}

		
		////////////////////////////   Communicate with ESC   /////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////

		if(esc_comms == COMMS_UART){
			if(ESC_UART_CONFIGED){
				read_vesc_packet();
				if(ESC_FW_READ){
					if(GET_LIMITS) {
						vesc_get_mcconf();
					} else if(SEND_CONTINUOUS){
						READ_VESC_VALS = true;
						vesc_read_all();
					}
				} else{
					detect_vesc_firmware();
				}
			} else{
				detect_esc_baud_pins();
			}
		}
		
		///////////////////////////////   Process Sensor data   //////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////

		readAccel();
		readGyro();
#ifdef HW_3v4
		readMag();
#endif

		// All IMU measurements are corrected to orient power to front and connectors up
		CorrectIMUvalues(ORIENTATION[0], ORIENTATION[1]);

		if(abs(axKalman - cax) < 10000)
		{
			avgAX = averageAX();
			axKalman = updateKalman(avgAX, ax_kalman);
		}
		avgAY = averageAY();
		avgAZ = averageAZ();
		ayKalman = updateKalman(avgAY, ay_kalman);
		azKalman = updateKalman(avgAZ, az_kalman);
		
		avgGX = averageGX();
		avgGY = averageGY();
		avgGZ = averageGZ();
		// TODO: Re-implement gyro kalman
		gxKalman = avgGX;
		gyKalman = avgGY;
		gzKalman = avgGZ;
		
		update_kalman_limits();
		calculate_heading();
		
		getLightSens(&light_sens);
		light_sens = updateKalman(light_sens, light_kalman);
		
		//////////////////////////////   Send Realtime Data   /////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		check_time(&BLE_TX_TIME);
		if(SEND_CONTINUOUS && app_remote_check == 0 && ((millis()-BLE_TX_TIME) >= BLE_TX_DELAY) && usart_get_job_status(&ble_usart, USART_TRANSCEIVER_TX) != STATUS_BUSY)
		{
#if  defined(HW_4v0) || defined(HW_4v1)
			port_pin_set_output_level(STAT_LED,true);
#endif
			switch(BLE_TX_INDEX){
				case 0:
					ble_write_buffer[0] = 0x11;
					ble_write_buffer[1] = latest_vesc_vals.avg_input_current & 0xFF;
					ble_write_buffer[2] = (latest_vesc_vals.avg_input_current & 0xFF00) >> 8;
					ble_write_buffer[3] = 0x12;
					ble_write_buffer[4] = latest_vesc_vals.INPUT_VOLTAGE;
					ble_write_buffer[5] = (latest_vesc_vals.INPUT_VOLTAGE & 0xFF00) >> 8;
					ble_write_buffer[6] = 0x13;
					ble_write_buffer[7] = latest_vesc_vals.avg_motor_current;
					ble_write_buffer[8] = (latest_vesc_vals.avg_motor_current & 0xFF00) >> 8;
					ble_write_buffer[9] = 0x14;
					ble_write_buffer[10] = latest_vesc_vals.temp_fet_filtered;
					ble_write_buffer[11] = (latest_vesc_vals.temp_fet_filtered & 0xFF00) >> 8;
					ble_write_buffer[12] = 0x15;
					ble_write_buffer[13] = latest_vesc_vals.duty_cycle;
					ble_write_buffer[14] = (latest_vesc_vals.duty_cycle & 0xFF00) >> 8;
					ble_write_buffer[15] = 0x16;
					ble_write_buffer[16] = (latest_vesc_vals.rpm & 0xFF);
					ble_write_buffer[17] = (latest_vesc_vals.rpm & 0xFF00) >> 8;
					ble_write_buffer[18] = (latest_vesc_vals.rpm & 0xFF0000) >> 16;
					usart_write_buffer_job(&ble_usart, ble_write_buffer, 19);
					break;
				case 1:
					ble_write_buffer[0] = 0x17;
					ble_write_buffer[1] = (latest_vesc_vals.amp_hours & 0xFF);
					ble_write_buffer[2] = (latest_vesc_vals.amp_hours & 0xFF00) >> 8;
					ble_write_buffer[3] = (latest_vesc_vals.amp_hours & 0xFF0000) >> 16;
					ble_write_buffer[4] = 0x18;
					ble_write_buffer[5] = (latest_vesc_vals.amp_hours_charged & 0xFF);
					ble_write_buffer[6] = (latest_vesc_vals.amp_hours_charged & 0xFF00) >> 8;
					ble_write_buffer[7] = (latest_vesc_vals.amp_hours_charged & 0xFF0000) >> 16;
					ble_write_buffer[8] = 0x19;
					ble_write_buffer[9] = (latest_vesc_vals.watt_hours & 0xFF);
					ble_write_buffer[10] = (latest_vesc_vals.watt_hours & 0xFF00) >> 8;
					ble_write_buffer[11] = (latest_vesc_vals.watt_hours & 0xFF0000) >> 16;
					ble_write_buffer[12] = 0x1A;
					ble_write_buffer[13] = (latest_vesc_vals.watt_hours_charged & 0xFF);
					ble_write_buffer[14] = (latest_vesc_vals.watt_hours_charged & 0xFF00) >> 8;
					ble_write_buffer[15] = (latest_vesc_vals.watt_hours_charged & 0xFF0000) >> 16;
					ble_write_buffer[16] = 0x1B;
					ble_write_buffer[17] = latest_vesc_vals.fault;
					ble_write_buffer[18] = 0x21;
					ble_write_buffer[19] = remote_x;
					usart_write_buffer_job(&ble_usart, ble_write_buffer, 20);
					break;
				case 2:
					ble_write_buffer[0] = 0x2E;
					ble_write_buffer[1] = ((uint16_t)(heading*10) & 0xFF); // Heading
					ble_write_buffer[2] = ((uint16_t)(heading*10) & 0xFF00) >> 8; // Heading
					ble_write_buffer[3] = 0x22;
					ble_write_buffer[4] = remote_y;
					ble_write_buffer[5] = 0x23;
					ble_write_buffer[6] = (remote_btn_state | (REMOTE_TYPE << 1));
					ble_write_buffer[7] = 0x24;
					ble_write_buffer[8] = ((uint16_t)axKalman & 0xFF); // Accel X
					ble_write_buffer[9] = ((uint16_t)axKalman & 0xFF00) >> 8; // Accel X
					ble_write_buffer[10] = 0x25;
					ble_write_buffer[11] = ((uint16_t)ayKalman & 0xFF); // Accel Y
					ble_write_buffer[12] = ((uint16_t)ayKalman & 0xFF00) >> 8; // Accel Y
					ble_write_buffer[13] = 0x26;
					ble_write_buffer[14] = ((uint16_t)azKalman & 0xFF); // Accel Z
					ble_write_buffer[15] = ((uint16_t)azKalman & 0xFF00) >> 8; // Accel Z
					ble_write_buffer[16] = 0x27;
					ble_write_buffer[17] = ((uint16_t)(gxKalman*10) & 0xFF); // Gyro X
					ble_write_buffer[18] = ((uint16_t)(gxKalman*10) & 0xFF00) >> 8; // Gyro X
					usart_write_buffer_job(&ble_usart, ble_write_buffer, 19);
					break;
				case 3:
					ble_write_buffer[0] = 0x28;
					ble_write_buffer[1] = ((uint16_t)(gyKalman*10) & 0xFF); // Gyro Y
					ble_write_buffer[2] = ((uint16_t)(gyKalman*10) & 0xFF00) >> 8; // Gyro Y
					ble_write_buffer[3] = 0x29;
					ble_write_buffer[4] = ((uint16_t)(gzKalman*10) & 0xFF); // Gyro Z
					ble_write_buffer[5] = ((uint16_t)(gzKalman*10) & 0xFF00) >> 8; // Gyro Z
					ble_write_buffer[6] = 0x2A;
					ble_write_buffer[7] = ((mx) & 0xFF); // Compass X
					ble_write_buffer[8] = (mx & 0xFF00) >> 8; // Compass X
					ble_write_buffer[9] = 0x2B;
					ble_write_buffer[10] = (my & 0xFF); // Compass Y
					ble_write_buffer[11] = (my & 0xFF00) >> 8; // Compass Y
					ble_write_buffer[12] = 0x2C;
					ble_write_buffer[13] = (mz & 0xFF); // Compass Z
					ble_write_buffer[14] = (mz & 0xFF00) >> 8; // Compass Z
					ble_write_buffer[15] = 0x2D;
					ble_write_buffer[16] = ((int)(light_sens) & 0xFF); // Light Sensor
					ble_write_buffer[17] = ((int)(light_sens) & 0xFF00) >> 8; // Light Sensor
					ble_write_buffer[18] = 0xDE;
					usart_write_buffer_job(&ble_usart, ble_write_buffer, 19);
					break;
		}
#if  defined(HW_4v0) || defined(HW_4v1)
			port_pin_set_output_level(STAT_LED,false);
#endif
			BLE_TX_INDEX++;
			if(BLE_TX_INDEX > 3)
				BLE_TX_INDEX = 0;

			BLE_TX_TIME = millis(); // Placed at end of transmit to provide accurate message timing
		}
		else
		{
			// Use a dummy delay that mimics the delay of the BLE send
			// commands to keep the timing of the light sections the same
			//while((millis()-BLE_DUMMY_TIME) < BLE_TX_DELAY + (1.0/BLE_BAUD)*20.0){}
			//BLE_DUMMY_TIME = millis();
		}

		
		////////////////////////////   Handle Limits Request   ////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_LIMITS)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x41;
			ble_write_buffer[1] = mcconf_limits.motor_current_max;
			ble_write_buffer[2] = 0x42;
			ble_write_buffer[3] = mcconf_limits.motor_current_min;
			ble_write_buffer[4] = 0x43;
			ble_write_buffer[5] = mcconf_limits.input_current_max;
			ble_write_buffer[6] = 0x44;
			ble_write_buffer[7] = mcconf_limits.input_current_min;
			ble_write_buffer[8] = 0x45;
			ble_write_buffer[9] = mcconf_limits.abs_current_max;
			ble_write_buffer[10] = 0x46;
			ble_write_buffer[11] = mcconf_limits.max_vin;
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 12);
			
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x48;
			ble_write_buffer[1] = mcconf_limits.battery_cut_start;
			ble_write_buffer[2] = 0x49;
			ble_write_buffer[3] = mcconf_limits.battery_cut_end;
			ble_write_buffer[4] = 0x4A;
			ble_write_buffer[5] = (mcconf_limits.max_erpm & 0xFF);
			ble_write_buffer[6] = (mcconf_limits.max_erpm & 0xFF00) >> 8;
			ble_write_buffer[7] = (mcconf_limits.max_erpm & 0xFF0000) >> 16;
			ble_write_buffer[8] = 0x4B;
			ble_write_buffer[9] = (mcconf_limits.min_erpm & 0xFF);
			ble_write_buffer[10] = (mcconf_limits.min_erpm & 0xFF00) >> 8;
			ble_write_buffer[11] = (mcconf_limits.min_erpm & 0xFF0000) >> 16;
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 12);

			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x4C;
			ble_write_buffer[1] = ((mcconf_limits.max_erpm_fbrake) & 0xFF);
			ble_write_buffer[2] = ((mcconf_limits.max_erpm_fbrake) & 0xFF00) >> 8;
			ble_write_buffer[3] = ((mcconf_limits.max_erpm_fbrake) & 0xFF0000) >> 16;
			ble_write_buffer[4] = 0x4D;
			ble_write_buffer[5] = ((mcconf_limits.max_erpm_fbrake_cc) & 0xFF);
			ble_write_buffer[6] = ((mcconf_limits.max_erpm_fbrake_cc) & 0xFF00) >> 8;
			ble_write_buffer[7] = ((mcconf_limits.max_erpm_fbrake_cc) & 0xFF0000) >> 16;
			ble_write_buffer[8] = 0x4E;
			ble_write_buffer[9] = mcconf_limits.temp_fet_start;
			ble_write_buffer[10] = 0x4F;
			ble_write_buffer[11] = mcconf_limits.temp_fet_end;
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 12);

			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x50;
			ble_write_buffer[1] = mcconf_limits.temp_motor_start;
			ble_write_buffer[2] = 0x51;
			ble_write_buffer[3] = mcconf_limits.temp_motor_end;
			ble_write_buffer[4] = 0x52;
			ble_write_buffer[5] = mcconf_limits.max_duty;
			ble_write_buffer[6] = 0x53;
			ble_write_buffer[7] = mcconf_limits.min_duty;
			ble_write_buffer[8] = 0x47;
			ble_write_buffer[9] = mcconf_limits.min_vin;
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 10);

			SEND_LIMITS = 0;
			SEND_CONTINUOUS = 1;
		}

		
		////////////////////////   Handle Sensor Params Request   /////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_SENSORS)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x61;
			ble_write_buffer[1] = ((uint16_t)(err_estimate[ax_kalman]) & 0xFF); // Accel X estimated error
			ble_write_buffer[2] = 0x62;
			ble_write_buffer[3] = ((uint16_t)(err_estimate[ay_kalman]) & 0xFF); // AccelY estimated error
			ble_write_buffer[4] = 0x63;
			ble_write_buffer[5] = ((uint16_t)(err_estimate[az_kalman]) & 0xFF); // Accel Z estimated error
			ble_write_buffer[6] = 0x64;
			ble_write_buffer[7] = ((uint16_t)(err_estimate[gx_kalman]) & 0xFF); // Gyro X estimated error
			ble_write_buffer[8] = 0x65;
			ble_write_buffer[9] = ((uint16_t)(err_estimate[gy_kalman]) & 0xFF); // Gyro Y estimated error
			ble_write_buffer[10] = 0x66;
			ble_write_buffer[11] = ((uint16_t)(err_estimate[gz_kalman]) & 0xFF); // Gyro Z estimated error
			ble_write_buffer[12] = 0x67;
			ble_write_buffer[13] = ((uint16_t)(err_estimate[light_kalman]) & 0xFF); // Light Sensor estimated error
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 14);
		
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x68;
			ble_write_buffer[1] = ((uint16_t)(q[ax_kalman]*100.0) & 0xFF); // Accel X Sensitivity
			ble_write_buffer[2] = 0x69;
			ble_write_buffer[3] = ((uint16_t)(q[ay_kalman]*100.0) & 0xFF); // Accel Y Sensitivity
			ble_write_buffer[4] = 0x6A;
			ble_write_buffer[5] = ((uint16_t)(q[az_kalman]*100.0) & 0xFF); // Accel Z Sensitivity
			ble_write_buffer[6] = 0x6B;
			ble_write_buffer[7] = ((uint16_t)(q[gx_kalman]*100.0) & 0xFF); // Gyro X Sensitivity
			ble_write_buffer[8] = 0x6C;
			ble_write_buffer[9] = ((uint16_t)(q[gy_kalman]*100.0) & 0xFF); // Gyro Y Sensitivity
			ble_write_buffer[10] = 0x6D;
			ble_write_buffer[11] = ((uint16_t)(q[gz_kalman]*100.0) & 0xFF); // Gyro Z Sensitivity
			ble_write_buffer[12] = 0x6E;
			ble_write_buffer[13] = ((uint16_t)(q[light_kalman]*100.0) & 0xFF); // Light Sensitivity
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 14);

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
			ble_write_buffer[0] = 0x31;
			ble_write_buffer[1] = led_mode_switches; // Current switch states
			ble_write_buffer[2] = RGB_led_type;
			// Static
			ble_write_buffer[3] = 0x32;
			ble_write_buffer[4] = (uint8_t)((float)Static_RGB.LR / 655.35);
			ble_write_buffer[5] = (uint8_t)((float)Static_RGB.LG / 655.35);
			ble_write_buffer[6] = (uint8_t)((float)Static_RGB.LB / 655.35);
			ble_write_buffer[7] = (uint8_t)((float)Static_RGB.RR / 655.35);
			ble_write_buffer[8] = (uint8_t)((float)Static_RGB.RG / 655.35);
			ble_write_buffer[9] = (uint8_t)((float)Static_RGB.RB / 655.35);
			// Color Cycle
			ble_write_buffer[10] = 0x33;
			ble_write_buffer[11] = (uint8_t)(RateSens[MODE_ANALOG_COLOR_CYCLE] * 100);
			ble_write_buffer[12] = (uint8_t)(Brightness[MODE_ANALOG_COLOR_CYCLE] * 100);
			// Compass Cycle
			ble_write_buffer[13] = 0x34;
			ble_write_buffer[14] = (uint8_t)(Brightness[MODE_ANALOG_COMPASS_CYCLE] * 100);
			// Throttle Based
			ble_write_buffer[15] = 0x35;
			ble_write_buffer[16] = (uint8_t)(RateSens[MODE_ANALOG_THROTTLE] * 100);
			ble_write_buffer[17] = (uint8_t)(Brightness[MODE_ANALOG_THROTTLE] * 100);
			// RPM Based
			ble_write_buffer[18] = 0x36;
			ble_write_buffer[19] = (uint8_t)(RateSens[MODE_ANALOG_RPM_CYCLE] * 100);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 20);
			
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// X Accel Based
			ble_write_buffer[0] = 0x37;
			ble_write_buffer[1] = (uint8_t)(RateSens[MODE_ANALOG_X_ACCEL] * 100);
			// Y Accel Based
			ble_write_buffer[2] = 0x38;
			ble_write_buffer[3] = (uint8_t)(Brightness[MODE_ANALOG_Y_ACCEL] * 100);
			// Custom
			uint8_t color_bright_base = (ColorBase[MODE_ANALOG_CUSTOM] << 4) | BrightBase[MODE_ANALOG_CUSTOM];
			ble_write_buffer[4] = 0x39;
			ble_write_buffer[5] = color_bright_base;
			ble_write_buffer[6] = RateBase[MODE_ANALOG_CUSTOM];
			ble_write_buffer[7] = (uint8_t)((float)Custom_RGB.LR / 655.35);
			ble_write_buffer[8] = (uint8_t)((float)Custom_RGB.LG / 655.35);
			ble_write_buffer[9] = (uint8_t)((float)Custom_RGB.LB / 655.35);
			ble_write_buffer[10] = (uint8_t)((float)Custom_RGB.RR / 655.35);
			ble_write_buffer[11] = (uint8_t)((float)Custom_RGB.RG / 655.35);
			ble_write_buffer[12] = (uint8_t)((float)Custom_RGB.RB / 655.35);
			ble_write_buffer[13] = (uint8_t)(RateSens[MODE_ANALOG_CUSTOM] * 100);
			ble_write_buffer[14] = (uint8_t)(Brightness[MODE_ANALOG_CUSTOM] * 100);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 15);
			
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Y Accel Based
			ble_write_buffer[0] = 0x3A;
			ble_write_buffer[1] = (uint8_t)(Digital_Static_Zoom);
			ble_write_buffer[2] = (uint8_t)(Digital_Static_Shift);
			ble_write_buffer[3] = (uint8_t)(Digital_Static_Brightness);
			ble_write_buffer[4] = 0x3B;
			ble_write_buffer[5] = (uint8_t)(Digital_Skittles_Brightness);
			ble_write_buffer[6] = 0x3C;
			ble_write_buffer[7] = (uint8_t)(Digital_Cycle_Zoom);
			ble_write_buffer[8] = (uint8_t)(Digital_Cycle_Rate);
			ble_write_buffer[9] = (uint8_t)(Digital_Cycle_Brightness);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 10);
			
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			ble_write_buffer[0] = 0x3D;
			ble_write_buffer[1] = (uint8_t)(Digital_Compass_Brightness);
			ble_write_buffer[2] = 0x3E;
			ble_write_buffer[3] = (uint8_t)(Digital_Throttle_Zoom);
			ble_write_buffer[4] = (uint8_t)(Digital_Throttle_Shift);
			ble_write_buffer[5] = (uint8_t)(Digital_Throttle_Sens);
			ble_write_buffer[6] = (uint8_t)(Digital_Throttle_Brightness);
			ble_write_buffer[7] = 0x3F;
			ble_write_buffer[8] = (uint8_t)(Digital_RPM_Zoom);
			ble_write_buffer[9] = (uint8_t)(Digital_RPM_Rate);
			ble_write_buffer[10] = (uint8_t)(Digital_RPM_Brightness);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 11);

			SEND_LED_CHARS = 0;
			SEND_CONTINUOUS = 1;
	}
		

		//////////////////////////   Handle Orientation Request   /////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_ORIENTAION_CONFIG)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			ble_write_buffer[0] = BLE_ORIENTATION_CONFIG;//0x71;
			ble_write_buffer[1] = ORIENTATION[0]; // Connectors Orientation
			ble_write_buffer[2] = ORIENTATION[1]; // Power Orientation
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 3);


			SEND_ORIENTAION_CONFIG = 0;
			SEND_CONTINUOUS = 1;
		}


		///////////////////////////   Handle Controls Request   ///////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_CONTROLS_CONFIG)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			ble_write_buffer[0] = BLE_CONTROLS_CONFIG;//0x81;
			ble_write_buffer[1] = (uint8_t)((AUX_ENABLED << 7) | (TURN_ENABLED << 6) | auxControlType);
			ble_write_buffer[2] = (uint8_t)auxTimedDuration;
			ble_write_buffer[3] = (uint8_t)((single_aux_control << 4) | single_all_control);
			ble_write_buffer[4] = (uint8_t)((single_head_control << 4) | single_side_control);
			ble_write_buffer[5] = (uint8_t)((single_down_control << 4) | single_up_control);
			ble_write_buffer[6] = (uint8_t)((dual_aux_control << 4) | dual_all_control);
			ble_write_buffer[7] = (uint8_t)((dual_head_control << 4) | dual_side_control);
			ble_write_buffer[8] = (uint8_t)((dual_down_control << 4) | dual_up_control);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 9);

			SEND_CONTROLS_CONFIG = 0;
			SEND_CONTINUOUS = 1;
		}


		/////////////////////////   Handle Remote Config Request   ////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_REMOTE_CONFIG)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			ble_write_buffer[0] = BLE_REMOTE_CONFIG;//0x72;
			ble_write_buffer[1] = (uint8_t)((remote_type << 4) | button_type);
			ble_write_buffer[2] = (uint8_t)(deadzone);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 3);

			SEND_REMOTE_CONFIG = 0;
			SEND_CONTINUOUS = 1;
		}


		//////////////////////////   Handle ESC Config Request   //////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_ESC_CONFIG)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			ble_write_buffer[0] = BLE_ESC_CONFIG;//0x73;
			ble_write_buffer[1] = (uint8_t)(esc_fw);
			ble_write_buffer[2] = (uint8_t)((esc_comms << 4) | UART_baud);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 3);

			SEND_ESC_CONFIG = 0;
			SEND_CONTINUOUS = 1;
		}
		

		//////////////////////////   Handle Lights Config Request   //////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_Lights_CONFIG)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			ble_write_buffer[0] = BLE_LIGHTS_CONFIG;//0x75;
			ble_write_buffer[1] = (uint8_t)(RGB_led_type << 4) | brake_light_mode;
			ble_write_buffer[2] = (uint8_t)(deadzone);
			ble_write_buffer[3] = (uint8_t)(led_num);
			ble_write_buffer[4] = (uint8_t)(SYNC_RGB << 7 | BRAKE_ALWAYS_ON << 6);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 5);

			SEND_Lights_CONFIG = 0;
			SEND_CONTINUOUS = 1;
		}


		//////////////////////////   Handle FW Read Request   //////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(SEND_TTL_FW)
		{
			while((millis()-BLE_TX_TIME) < BLE_TX_DELAY*2){}
			BLE_TX_TIME = millis();

			// Global LED Settings
			ble_write_buffer[0] = BLE_TTL_FW;//0x74;
			ble_write_buffer[1] = (uint8_t)(TTL_FW%100 & 0x00FF);
			ble_write_buffer[2] = (uint8_t)(TTL_FW/100 & 0x00FF);
			usart_write_buffer_wait(&ble_usart, ble_write_buffer, 3);

			SEND_TTL_FW = 0;
			SEND_CONTINUOUS = 1;
		}
		

		////////////////////////////////   LED Controls   /////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		HandleUserInput();


		/////////////////////////////////   App Remote   //////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		HandleAppRemote();


		//////////////////////////////////   LED MODES   //////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////
		if(sensorControl() && LIGHTS_ON){
			if(SIDELIGHTS && lightControlSide()){
				if(RGB_led_type == RGB_ANALOG){
					AnalogSideLights();
				}else if(RGB_led_type == RGB_DIGITAL_APA102 || RGB_led_type == RGB_DIGITAL_APA102) { // Digital LED Functions
					DigitalSideLights();
				} else{
					//No RGB LEDs
				}
			}
			else {
				if(!TurnSignalOn) {
					if(RGB_led_type == RGB_ANALOG){
						setLeftRGB(0, 0, 0);
						setRightRGB(0, 0, 0);
					}
					else{
						if(!DIGITAL_OFF){
							for(uint16_t i = 0; i < led_num; i++)
							{
								L_SPI_send_buf[(i*4)+4] = R_SPI_send_buf[(i*4)+4] = (0b11100000 | 0);
								L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = 0;
								L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = 0;
								L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = 0;
							}
							L_APA_write(led_num);
							R_APA_write(led_num);
							DIGITAL_OFF = true;
						}
					}
				}
			}


			/////////////// Control the head and tail lights //////////////////
			HeadLight();
			
		} else {
			setWhite(0);
			if(RGB_led_type == RGB_ANALOG){
				setLeftRGB(0,0,0);
				setRightRGB(0,0,0);
			}
			else{
				if(!DIGITAL_OFF){
					for(uint16_t i = 0; i < led_num; i++)
					{
						L_SPI_send_buf[(i*4)+4] = R_SPI_send_buf[(i*4)+4] = (0b11100000 | 0);
						L_SPI_send_buf[(i*4)+5] = R_SPI_send_buf[(i*4)+5] = 0;
						L_SPI_send_buf[(i*4)+6] = R_SPI_send_buf[(i*4)+6] = 0;
						L_SPI_send_buf[(i*4)+7] = R_SPI_send_buf[(i*4)+7] = 0;
					}
					L_APA_write(led_num);
					R_APA_write(led_num);
					DIGITAL_OFF = true;
				}
			}
		}//*/
		BrakeLight();
	}
}