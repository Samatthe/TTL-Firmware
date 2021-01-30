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

#ifndef APA102_H 
#define APA102_H

#include "LED_Vars.h"

#define APA_BAUD 160000

void configure_APA_SPI(void);
enum status_code L_APA_BUSY(void);
enum status_code R_APA_BUSY(void);

void configure_APA_SPI(void)
{
    struct spi_config config_spi_master;
	
	/* Configure, initialize and enable Left APA SPI module (SERCOM4)*/
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mode_specific.master.baudrate = APA_BAUD;
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	config_spi_master.pinmux_pad0 = PINMUX_PA13D_SERCOM4_PAD1; //CLK
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	config_spi_master.pinmux_pad2 = PINMUX_PB11D_SERCOM4_PAD3; //DATA
	config_spi_master.pinmux_pad3 = PINMUX_UNUSED;
	spi_init(&L_LED_SPI_instance, SERCOM4, &config_spi_master);
	spi_enable(&L_LED_SPI_instance);

    /* Configure, initialize and enable Right APA SPI module (SERCOM0)*/
    spi_get_config_defaults(&config_spi_master);
    config_spi_master.mode_specific.master.baudrate = APA_BAUD;
    config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_F;
    config_spi_master.pinmux_pad0 = PINMUX_UNUSED;
    config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
    config_spi_master.pinmux_pad2 = PINMUX_PA10D_SERCOM2_PAD2; //DATA
    config_spi_master.pinmux_pad3 = PINMUX_PA11D_SERCOM2_PAD3; //CLK
    spi_init(&R_LED_SPI_instance, SERCOM2, &config_spi_master);
    spi_enable(&R_LED_SPI_instance);

	// TODO: Replace with memcpy
	for(int i = 0; i < MAX_LEDCOUNT; i++){
		L_SPI_send_buf[(i*4)+4] = (0b11100000 | 0x0A);
		L_SPI_send_buf[(i*4)+5] = 0x00;
		L_SPI_send_buf[(i*4)+6] = 0x00;
		L_SPI_send_buf[(i*4)+7] = 0x00;
	}

	
	// TODO: Replace with memcpy
	for(int i = 0; i < MAX_LEDCOUNT; i++){
		R_SPI_send_buf[(i*4)+4] = (0b11100000 | 0x0A);
		R_SPI_send_buf[(i*4)+5] = 0x00;
		R_SPI_send_buf[(i*4)+6] = 0x00;
		R_SPI_send_buf[(i*4)+7] = 0x00;
	}
	
	// TODO: Replace with memcpy
	L_SPI_send_buf[0] = 0x00;
	L_SPI_send_buf[1] = 0x00;
	L_SPI_send_buf[2] = 0x00;
	L_SPI_send_buf[3] = 0x00;
	R_SPI_send_buf[0] = 0x00;
	R_SPI_send_buf[1] = 0x00;
	R_SPI_send_buf[2] = 0x00;
	R_SPI_send_buf[3] = 0x00;
}


  /*! A struct that can be used to represent colors.  Each field is a number
   * between 0 and 255 that represents the brightness of a component of the
   * color. */
  struct rgb_color
  {
    uint8_t red;
	uint8_t green;
	uint8_t blue;
  };

// Create a buffer for holding the colors (3 bytes per color).
struct rgb_color L_colors[MAX_LEDCOUNT];
struct rgb_color R_colors[MAX_LEDCOUNT];

void set_left_gnd(void);
void set_right_gnd(void);
void L_APA_write(uint16_t count);
void R_APA_write(uint16_t count);

void set_left_gnd(void){
	port_pin_set_output_level(L_GND,1);
}

void set_right_gnd(void){
	port_pin_set_output_level(R_GND,1);
}

void L_APA_write(uint16_t count)
{
	// Set the stop frame
	L_SPI_send_buf[count*4+4] = 0xFF;
	L_SPI_send_buf[count*4+5] = 0xFF;
	L_SPI_send_buf[count*4+6] = 0xFF;
	L_SPI_send_buf[count*4+7] = 0xFF;
	spi_write_buffer_job(&L_LED_SPI_instance, L_SPI_send_buf, (count*4)+8);
}

enum status_code L_APA_BUSY(){
	return spi_get_job_status(&L_LED_SPI_instance);
}

void R_APA_write(uint16_t count)
{
	// Set the stop frame
	R_SPI_send_buf[count*4+4] = 0xFF;
	R_SPI_send_buf[count*4+5] = 0xFF;
	R_SPI_send_buf[count*4+6] = 0xFF;
	R_SPI_send_buf[count*4+7] = 0xFF;
	spi_write_buffer_job(&R_LED_SPI_instance, R_SPI_send_buf, (count*4)+8);
}

enum status_code R_APA_BUSY(){
	return spi_get_job_status(&R_LED_SPI_instance);
}

#endif