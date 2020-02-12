#ifndef APA102_H 
#define APA102_H

#include "LED_Vars.h"

void configure_APA_SPI(void);

void configure_APA_SPI(void)
{
    struct spi_config config_spi_master;
	
	/* Configure, initialize and enable Left APA SPI module (SERCOM4)*/
	spi_get_config_defaults(&config_spi_master);
	config_spi_master.mode_specific.master.baudrate = 80000;
	config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_I;
	config_spi_master.pinmux_pad0 = PINMUX_PA13D_SERCOM4_PAD1; //CLK
	config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
	config_spi_master.pinmux_pad2 = PINMUX_PB11D_SERCOM4_PAD3; //DATA
	config_spi_master.pinmux_pad3 = PINMUX_UNUSED;
	spi_init(&L_LED_SPI_instance, SERCOM4, &config_spi_master);
	spi_enable(&L_LED_SPI_instance);

    /* Configure, initialize and enable Right APA SPI module (SERCOM0)*/
    spi_get_config_defaults(&config_spi_master);
    config_spi_master.mode_specific.master.baudrate = 80000;
    config_spi_master.mux_setting = SPI_SIGNAL_MUX_SETTING_F;
    config_spi_master.pinmux_pad0 = PINMUX_UNUSED;
    config_spi_master.pinmux_pad1 = PINMUX_UNUSED;
    config_spi_master.pinmux_pad2 = PINMUX_PA10D_SERCOM2_PAD2; //DATA
    config_spi_master.pinmux_pad3 = PINMUX_PA11D_SERCOM2_PAD3; //CLK
    spi_init(&R_LED_SPI_instance, SERCOM2, &config_spi_master);
    spi_enable(&R_LED_SPI_instance);

	
	for(int i = 0; i < MAX_LEDCOUNT; i++){
		L_SPI_send_buf[(i*4)+4] = (0b11100000 | 0x0A);
		L_SPI_send_buf[(i*4)+5] = 0x00;
		L_SPI_send_buf[(i*4)+6] = 0x00;
		L_SPI_send_buf[(i*4)+7] = 0x00;
	}

	
	for(int i = 0; i < MAX_LEDCOUNT; i++){
		R_SPI_send_buf[(i*4)+4] = (0b11100000 | 0x0A);
		R_SPI_send_buf[(i*4)+5] = 0x00;
		R_SPI_send_buf[(i*4)+6] = 0x00;
		R_SPI_send_buf[(i*4)+7] = 0x00;
	}
	
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