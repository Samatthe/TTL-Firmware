#ifndef SK9822_H 
#define SK9822_H

#include "LED_Vars.h"

void configure_SK98_SPI(void);

void configure_SK98_SPI(void)
{
    struct spi_config config_spi_master;
	
	/* Configure, initialize and enable Left SK98 SPI module (SERCOM4)*/
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mode_specific.master.baudrate = 350000;
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	config_spi_master.pinmux_pad0 = PINMUX_PA13D_SERCOM4_PAD1; //CLK
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	config_spi_master.pinmux_pad2 = PINMUX_PB11D_SERCOM4_PAD3; //DATA
	config_spi_master.pinmux_pad3 = PINMUX_UNUSED;
	spi_init(&L_LED_SPI_instance, SERCOM4, &config_spi_master);
	spi_enable(&L_LED_SPI_instance);

    /* Configure, initialize and enable Right SK98 SPI module (SERCOM0)*/
    spi_get_config_defaults(&config_spi_master);
    config_spi_master.mode_specific.master.baudrate = 350000;
    config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_F;
    config_spi_master.pinmux_pad0 = PINMUX_UNUSED;
    config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
    config_spi_master.pinmux_pad2 = PINMUX_PA10D_SERCOM2_PAD2; //DATA
    config_spi_master.pinmux_pad3 = PINMUX_PA11D_SERCOM2_PAD3; //CLK
    spi_init(&R_LED_SPI_instance, SERCOM2, &config_spi_master);
    spi_enable(&R_LED_SPI_instance);
}

// Create a buffer for holding the colors (3 bytes per color).

void L_SK98_write(struct rgb_color *colors, uint16_t count, uint8_t brightness);
void R_SK98_write(struct rgb_color *colors, uint16_t count, uint8_t brightness);

void L_SK98_write(struct rgb_color * colors, uint16_t count, uint8_t brightness)
{
	//if(spi_get_job_status(&L_LED_SPI_instance) != STATUS_BUSY){
		//L_SPI_send_buf[0] = 0x00;
		//L_SPI_send_buf[1] = 0x00;
		//L_SPI_send_buf[2] = 0x00;
		//L_SPI_send_buf[3] = 0x00;
		//for(int i = 0; i < count; i++){
			//L_SPI_send_buf[(i*4)+4] = (0b11100000 | brightness);
			//L_SPI_send_buf[(i*4)+5] = colors[i].blue;
			//L_SPI_send_buf[(i*4)+6] = colors[i].green;
			//L_SPI_send_buf[(i*4)+7] = colors[i].red;
		//}
		//L_SPI_send_buf[count*4+4] = 0xFF;
		//L_SPI_send_buf[count*4+5] = 0xFF;
		//L_SPI_send_buf[count*4+6] = 0xFF;
		//L_SPI_send_buf[count*4+7] = 0xFF;
		//
		//spi_write_buffer_job(&L_LED_SPI_instance, L_SPI_send_buf, (count*4)+8);
	//}
}

enum status_code L_SK98_BUSY(){
	return spi_get_job_status(&L_LED_SPI_instance);
}

void R_SK98_write(struct rgb_color * colors, uint16_t count, uint8_t brightness)
{
	//if(spi_get_job_status(&R_LED_SPI_instance) != STATUS_BUSY){
		//R_SPI_send_buf[0] = 0x00;
		//R_SPI_send_buf[1] = 0x00;
		//R_SPI_send_buf[2] = 0x00;
		//R_SPI_send_buf[3] = 0x00;
		//for(int i = 0; i < count; i++){
			//R_SPI_send_buf[(i*4)+4] = (0b11100000 | brightness);
			//R_SPI_send_buf[(i*4)+5] = colors[i].blue;
			//R_SPI_send_buf[(i*4)+6] = colors[i].green;
			//R_SPI_send_buf[(i*4)+7] = colors[i].red;
		//}
		//R_SPI_send_buf[count*4+4] = 0xFF;
		//R_SPI_send_buf[count*4+5] = 0xFF;
		//R_SPI_send_buf[count*4+6] = 0xFF;
		//R_SPI_send_buf[count*4+7] = 0xFF;
		//
		//spi_write_buffer_job(&R_LED_SPI_instance, R_SPI_send_buf, (count*4)+8);
	//}
}

enum status_code R_SK98_BUSY(){
	return spi_get_job_status(&R_LED_SPI_instance);
}

#endif