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
 
#ifndef WS2815_H
#define WS2815_H

#include "LED_Vars.h"

#define WS_BAUD 4000000 // ~0.25us

void configure_WS_SPI(void);
void ws_set_left_gnd(void);
void ws_set_right_gnd(void);
void L_WS_write(uint16_t count);
void R_WS_write(uint16_t count);
void add_byte(uint8_t *buffer, uint16_t offset, int8_t byte);

 void configure_WS_SPI(void){
	 struct spi_config config_spi_master;
	 
	 /* Configure, initialize and enable Left APA SPI module (SERCOM4)*/
	 spi_get_config_defaults(&config_spi_master);
	 config_spi_master.mode_specific.master.baudrate = WS_BAUD;
	 config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	 config_spi_master.pinmux_pad0 = PINMUX_PA13D_SERCOM4_PAD1; //CLK
	 config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	 config_spi_master.pinmux_pad2 = PINMUX_PB11D_SERCOM4_PAD3; //DATA
	 config_spi_master.pinmux_pad3 = PINMUX_UNUSED;
	 spi_init(&L_LED_SPI_instance, SERCOM4, &config_spi_master);
	 spi_enable(&L_LED_SPI_instance);

	 /* Configure, initialize and enable Right APA SPI module (SERCOM0)*/
	 spi_get_config_defaults(&config_spi_master);
	 config_spi_master.mode_specific.master.baudrate = WS_BAUD;
	 config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_F;
	 config_spi_master.pinmux_pad0 = PINMUX_UNUSED;
	 config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	 config_spi_master.pinmux_pad2 = PINMUX_PA10D_SERCOM2_PAD2; //DATA
	 config_spi_master.pinmux_pad3 = PINMUX_PA11D_SERCOM2_PAD3; //CLK
	 spi_init(&R_LED_SPI_instance, SERCOM2, &config_spi_master);
	 spi_enable(&R_LED_SPI_instance);
 }

 void ws_set_left_gnd(void){
	 port_pin_set_output_level(L_GND,1);
 }

 void ws_set_right_gnd(void){
	 port_pin_set_output_level(R_GND,1);
 }

void add_byte(uint8_t *buffer, uint16_t offset, int8_t byte){
	for (int i = 0; i < 8; i++){
		if(byte & (1 << (7-i)))
			buffer[offset+i] = 0x00;
		else
			buffer[offset+i] = 0x80;
	}
}

void L_WS_write(uint16_t count)
{
	static uint8_t L_temp_buf[(MAX_LEDCOUNT*24)];
	memset(L_temp_buf, 0, (MAX_LEDCOUNT*24));
	for(int i = 0; i < led_num; i++){
		add_byte(L_temp_buf,(i*24),0xFF);//L_SPI_send_buf[(i*4)+1]); // Red
		add_byte(L_temp_buf,(i*24)+8,0xFF);//L_SPI_send_buf[(i*4)+2]); // Green
		add_byte(L_temp_buf,(i*24)+16,0xFF);//L_SPI_send_buf[(i*4)+3]); // Blue
	}
	spi_write_buffer_job(&L_LED_SPI_instance, L_temp_buf, 24);//(led_num*24));//*/
}

void R_WS_write(uint16_t count)
{
	static uint8_t R_temp_buf[(MAX_LEDCOUNT*24)];
	memset(R_temp_buf, 0, (MAX_LEDCOUNT*24));
	for(int i = 0; i < led_num; i++){
		add_byte(R_temp_buf,(i*24),0xFF);//R_SPI_send_buf[(i*4)+1]); // Red
		add_byte(R_temp_buf,(i*24)+8,0xFF);//R_SPI_send_buf[(i*4)+2]); // Green
		add_byte(R_temp_buf,(i*24)+16,0xFF);//R_SPI_send_buf[(i*4)+3]); // Blue
	}
	//spi_write_buffer_job(&R_LED_SPI_instance, R_SPI_send_buf, (count*4)+8);
	spi_write_buffer_job(&R_LED_SPI_instance, R_temp_buf, 24);//(led_num*24));//*/
}

 #endif