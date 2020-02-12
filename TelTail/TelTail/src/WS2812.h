/*
 * WS2812.c
 *
 * Created: 12/14/2019 1:40:08 AM
 *  Author: NEO
 */
 
 #include "LED_Vars.h"

void configure_WS2812_SPI(void);

 void configure_WS2812_SPI(void){
	 struct spi_config config_spi_master;
	 
	 /* Configure, initialize and enable Left APA SPI module (SERCOM4)*/
	 spi_get_config_defaults(&config_spi_master);
	 config_spi_master.mode_specific.master.baudrate = 4000000;
	 //config_spi_master.generator_source = 1;
	 config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	 config_spi_master.pinmux_pad0 = PINMUX_PA13D_SERCOM4_PAD1; //CLK
	 config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	 config_spi_master.pinmux_pad2 = PINMUX_PB11D_SERCOM4_PAD3; //DATA
	 config_spi_master.pinmux_pad3 = PINMUX_UNUSED;
	 spi_init(&L_LED_SPI_instance, SERCOM4, &config_spi_master);
	 spi_enable(&L_LED_SPI_instance);

	 /* Configure, initialize and enable Right APA SPI module (SERCOM0)*/
	 spi_get_config_defaults(&config_spi_master);
	 config_spi_master.mode_specific.master.baudrate = 4000000;
	 //config_spi_master.generator_source = 1;
	 config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_F;
	 config_spi_master.pinmux_pad0 = PINMUX_UNUSED;
	 config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	 config_spi_master.pinmux_pad2 = PINMUX_PA10D_SERCOM2_PAD2; //DATA
	 config_spi_master.pinmux_pad3 = PINMUX_PA11D_SERCOM2_PAD3; //CLK
	 spi_init(&R_LED_SPI_instance, SERCOM2, &config_spi_master);
	 spi_enable(&R_LED_SPI_instance);
 }
 
 
void WS_set_left_gnd(void);
void WS_set_right_gnd(void);
void L_WS_write(struct rgb_color *colors, uint16_t count, uint8_t brightness);
void R_WS_write(struct rgb_color *colors, uint16_t count, uint8_t brightness);
void add_byte(uint8_t *buffer, uint16_t offset, int8_t byte);

 void ws_set_left_gnd(void){
	 port_pin_set_output_level(L_GND,1);
 }

 void ws_set_right_gnd(void){
	 port_pin_set_output_level(R_GND,1);
 }

void add_byte(uint8_t *buffer, uint16_t offset, int8_t byte){
	for (int i = 0; i < 8; i++){
		if(byte & (1 << 7-i))
			buffer[offset+i] = 0xAA;
		else
			buffer[offset+i] = 0xAA;
	}
}

 void L_WS_write(struct rgb_color * colors, uint16_t count, uint8_t brightness)
 {
	 //if(spi_get_job_status(&L_LED_SPI_instance) != STATUS_BUSY){
		 //for(int i = 0; i < led_num; i++){
			 //add_byte(L_SPI_send_buf, (i*24), colors[i].green);
			 //add_byte(L_SPI_send_buf, (i*24)+8, colors[i].red);
			 //add_byte(L_SPI_send_buf, (i*24)+16, colors[i].blue);
		 //}
		 //spi_write_buffer_job(&L_LED_SPI_instance, L_SPI_send_buf, (led_num*24));
		 ////L_SPI_send_buf[0] = 0xAA;
		 ////L_SPI_send_buf[1] = 0xAA;
		 ////L_SPI_send_buf[2] = 0xAA;
		 ////L_SPI_send_buf[3] = 0xAA;
		 ////spi_write_buffer_job(&L_LED_SPI_instance, L_SPI_send_buf, (4));
	 //}
 }

 void R_WS_write(struct rgb_color * colors, uint16_t count, uint8_t brightness)
 {
	 //if(spi_get_job_status(&R_LED_SPI_instance) != STATUS_BUSY){
		 //for(int i = 0; i < led_num; i++){
			 //add_byte(R_SPI_send_buf, (i*24), colors[i].green);
			 //add_byte(R_SPI_send_buf, (i*24)+8, colors[i].red);
			 //add_byte(R_SPI_send_buf, (i*24)+16, colors[i].blue);
		 //}
		 //spi_write_buffer_job(&R_LED_SPI_instance, R_SPI_send_buf, (led_num*24));
		 ////R_SPI_send_buf[0] = 0xAA;
		 ////R_SPI_send_buf[1] = 0xAA;
		 ////R_SPI_send_buf[2] = 0xAA;
		 ////R_SPI_send_buf[3] = 0xAA;
		 ////spi_write_buffer_job(&R_LED_SPI_instance, R_SPI_send_buf, (4));
	 //}
 }