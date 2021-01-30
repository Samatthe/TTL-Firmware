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

#ifndef CONTROLS_H
#define CONTROLS_H

#include "Control_Vars.h"
#include "ESC_UART.h"
#include "Remote_Vars.h"
#include "Timing.h"
#include "LED_Vars.h"
#include "EEPROM_Functions.h"
//#include "BLE_UART.h"

// This function is used to handle all control inputs
// Input types include:	App commands
//						Button Presses from coms based remotes (Nunchuck, Feather Remote)
//						Button Presses from PPM remote
void HandleUserInput(void);
int get_pulse_width(void);
void config_eic_channel(int ch, int sense, bool filt);
void config_eic(void);
void config_evsys(void);
void gpio_in(int port, int pin);
void gpio_pmuxen(int port, int pin, int mux);
void config_gpio(void);
void HandleAppRemote(void);
void configure_pw_tc(void);

struct tc_module pw_timer;
void configure_pw_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	config_tc.enable_capture_on_channel[TC_COMPARE_CAPTURE_CHANNEL_0] = true;
	//config_tc.counter_16_bit = true;
	tc_init(&pw_timer, TC3, &config_tc);
	TC3->COUNT16.CTRLC.reg  |= TC_CTRLC_CPTEN0;
	TC3->COUNT16.EVCTRL.reg |= TC_EVCTRL_TCEI | TC_EVCTRL_EVACT_PWP;
	tc_enable(&pw_timer);
}//*/

/* Sense: 
 * None, Rise, Fall, Both, High, Low
 * 0x0	 0x1   0x2	 0x3   0x4   0x5
 */
void config_eic_channel(int ch, int sense, bool filt) {
	// Config channel
	EIC->CONFIG[ch/8].reg &= ~(0xF << 4*(ch%8));
	EIC->CONFIG[ch/8].reg |= (0xF & ((filt? 0x8 : 0) | (0x7 & sense))) << 4*(ch%8);
	// No wake-up
	EIC->WAKEUP.reg &= ~(1 << ch);	
	// No interrupt
	EIC->INTENCLR.reg |= 1<<ch;
	// Generate Event 
	EIC->EVCTRL.reg |= 1<<ch;
}

void config_eic() {
PM->APBAMASK.reg |= PM_APBAMASK_EIC;
GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EIC_GCLK_ID) |
GCLK_CLKCTRL_CLKEN |
GCLK_CLKCTRL_GEN(0);
EIC->CTRL.reg = EIC_CTRL_SWRST;
while(EIC->CTRL.bit.SWRST && EIC->STATUS.bit.SYNCBUSY);
config_eic_channel(2, 4, false);

EIC->CTRL.bit.ENABLE = 1;
while(EIC->STATUS.bit.SYNCBUSY);
}

void config_evsys() {
	PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(EVSYS_GCLK_ID_0) |
	GCLK_CLKCTRL_CLKEN |
	GCLK_CLKCTRL_GEN(0);
	while(GCLK->STATUS.bit.SYNCBUSY);

	EVSYS->CTRL.bit.SWRST = 1;
	while(EVSYS->CTRL.bit.SWRST);

	// Event receiver
	EVSYS->USER.reg = EVSYS_USER_CHANNEL(1) | // Set channel n-1
	//EVSYS_USER_USER(EVSYS_ID_USER_TCC1_EV_1); // Match/Capture 1 on TCC1
	EVSYS_USER_USER(EVSYS_ID_USER_TC3_EVU); // Match/Capture on TC3
	// Event channel
	EVSYS->CHANNEL.reg = EVSYS_CHANNEL_CHANNEL(0) | // Set channel n
	EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
	EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_2) |
	EVSYS_CHANNEL_EDGSEL_BOTH_EDGES; // Detect both edges
	// Wait channel to be ready
	while(!EVSYS->CHSTATUS.bit.USRRDY0);
	// EVSYS is always enabled
}

void gpio_in(int port, int pin)	{
	PORT->Group[port].DIRCLR.reg = (1 << pin);
	PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_INEN;
}

void gpio_pmuxen(int port, int pin, int mux) {
	PORT->Group[port].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
	if (pin & 1)
	PORT->Group[port].PMUX[pin>>1].bit.PMUXO = mux;
	else
	PORT->Group[port].PMUX[pin>>1].bit.PMUXE = mux;
}

void config_gpio() {
	gpio_in(1, 2);
	gpio_pmuxen(1, 2, PINMUX_PB02A_EIC_EXTINT2);
}

//uint16_t light_sens = 0; // for PWM debugging
inline int get_pulse_width() {
	return TC3->COUNT16.CC[0].bit.CC;
}

void HandleUserInput()
{
	///////////////   Use the appropriate throttle input   ///////////////
	//////////////////////////////////////////////////////////////////////
	switch(remote_type){
		case REMOTE_PPM:
		case REMOTE_UART_PPM:{
			READ_VESC_PWM = true;
			float temp = (((float)latest_vesc_vals.pwm_val + 595000.0)*(255.0/1495000.0));
			//temp = -temp + 255;
			if(temp < 0)
				temp = 0;
			else if(temp > 255)
				temp = 255;
			remote_y = (uint8_t)(temp);
			break;}
		case REMOTE_UART_SINGLE:
		//case REMOTE_UART_DUAL:
			READ_VESC_CHUCK = true;
			remote_y = rec_chuck_struct.js_y;
			//if(remote_type == REMOTE_UART_DUAL)
			//	remote_x = rec_chuck_struct.js_x;
			//else
				remote_x = 255/2;
			break;
		case REMOTE_APP:
			//if(app_remote_check && REMOTE_TYPE < 2)
				//temp_y = AppRemoteY;
			//else
				//temp_y = VescRemoteY;
			break;
	}


	////////////////   Use the appropriate button input   ////////////////
	//////////////////////////////////////////////////////////////////////
	switch(button_type){
		case BTN_MOMENTARY: 
		case BTN_LATCHED:
			remote_btn_state = port_pin_get_input_level(PPM_IN);
			break;
		case BTN_LATCHED_PPM:{
				static bool FIRST_READ = true;
				//static int pulse_width = 0;		// For debugging pulse width
				//pulse_width = get_pulse_width();	// For debugging pulse width
				if(get_pulse_width() > 9000)
					remote_btn_state = false;
				else
					remote_btn_state = true;
				if(FIRST_READ){
					lremote_btn_state = remote_btn_state;
					FIRST_READ = false;
				}
				//light_sens = pulse_width; // for debugging pulse width reading
			}
			break;
		case BTN_UART_C:
			READ_VESC_CHUCK = true;
			remote_btn_state = rec_chuck_struct.bt_c;
			break;	
		case BTN_UART_Z:
			READ_VESC_CHUCK = true;
			remote_btn_state = rec_chuck_struct.bt_z;
			break;
		case BTN_THROTTLE_DWN:
			remote_btn_state = (remote_y < (127 - DWN_THRESH));
			break;
		case BTN_THROTTLE_UP:
			remote_btn_state = (remote_y > (127 + UP_THRESH));
			break;
		default:
			remote_btn_state = 0;
			break;
	}

	////   Determine the time the button was held down and released   ////
	//////////////////////////////////////////////////////////////////////
	if(button_type != BTN_LATCHED && button_type != BTN_LATCHED_PPM){
		if(remote_btn_state == 1 && lremote_btn_state == 0){
			lButtonTime = millis(); // Mark the time of button state transition
			ButtonUpTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 1){
			check_time(&lButtonTime);
			ButtonDownTime = millis() - lButtonTime;  // Track time button was pressed

			lButtonTime = millis();  // Mark the time of button state transition
			ButtonHeldTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 0){
			check_time(&lButtonTime);
			ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed

			ButtonDownTime = 0;
			TurnSignalOn = false;
		} else if(remote_btn_state == 1 && lremote_btn_state == 1){
			check_time(&lButtonTime);
			ButtonHeldTime = millis() - lButtonTime; // Track time button is not pressed
		}
	} else {
		ButtonHeldTime = 0;
		if(remote_btn_state == 1 && lremote_btn_state == 0){
			ButtonDownTime = 250;  // Track time button was pressed
			lButtonTime = millis(); // Mark the time of button state transition
			ButtonUpTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 1){
			ButtonDownTime = 250;  // Track time button was pressed
			lButtonTime = millis();  // Mark the time of button state transition
			ButtonUpTime = 0;
		} else if(remote_btn_state == 0 && lremote_btn_state == 0){
			check_time(&lButtonTime);
			ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed

			ButtonDownTime = 0;
		} else if(remote_btn_state == 1 && lremote_btn_state == 1){
			check_time(&lButtonTime);
			ButtonUpTime = millis() - lButtonTime; // Track time button is not pressed
			
			ButtonDownTime = 0;
		}
	}
	lremote_btn_state = remote_btn_state;

	////////   Determine the type of button press that occurred   ////////
	//////////////////////////////////////////////////////////////////////
	ButtonPressType = PRESS_NONE;
	if(ButtonDownTime > 0 && ButtonDownTime < BUTTON_TAP_TIME){ // Button Tap
		tapIndex++;
	} else if(ButtonDownTime > BUTTON_TAP_TIME && ButtonDownTime < BUTTON_LONG_HOLD_TIME && !TurnSignalOn){ // Medium Press
		ButtonPressType = MEDIUM_PRESS;
	} else if(ButtonDownTime >= BUTTON_LONG_HOLD_TIME && !TurnSignalOn){ // Long Press
		ButtonPressType = LONG_PRESS;
	}
	if(tapIndex > 0 && ButtonUpTime > 200){
		tapSequence = 1;
	}
	if(tapSequence){
		/*if(remote_type == REMOTE_UART_DUAL && VescRemoteX <= 110 && tapIndex == 1)
			ButtonPressType = LEFT_TAP;
		else if(remote_type == REMOTE_UART_DUAL && VescRemoteX >= 150 && tapIndex == 1)
		ButtonPressType = RIGHT_TAP;
		else*/ if(tapIndex == 1){
			ButtonPressType = SINGLE_TAP;
		}
		else if(tapIndex == 2)
			ButtonPressType = DOUBLE_TAP;
		else if(tapIndex == 3)
			ButtonPressType = TRIPLE_TAP;
		
		tapSequence = 0;
		tapIndex = 0;
	}
	
	//////////////////////   Handle the aux output   /////////////////////
	//////////////////////////////////////////////////////////////////////
	if(AUX_ENABLED){
		if(AppAuxButton && !lAppAuxButton) {
			 AUX_OUTPUT = true;
		} else if(!AppAuxButton && lAppAuxButton){
			 AUX_OUTPUT = false;
		} else {
			switch(auxControlType){
				case AUX_MOMENTARY:
				if(ButtonHeldTime > 500 && single_aux_control != PRESS_NONE){
					AUX_OUTPUT = true;
				} else {
					AUX_OUTPUT = false;
				}
				break;
				case AUX_TOGGLED:
				if(single_aux_control == ButtonPressType && single_aux_control != PRESS_NONE){
					AUX_OUTPUT = !AUX_OUTPUT;
				}
				break;
				case AUX_TIMED:
				if((single_aux_control == ButtonPressType && single_aux_control != PRESS_NONE)){
					AUX_OUTPUT = true;
					AuxOnTime = millis();
				}

				if(AUX_OUTPUT == true && check_timer_expired(&AuxOnTime, (auxTimedDuration * 100)) && !lAppAuxButton)
					AUX_OUTPUT = false;
				break;
				case AUX_PATTERN:
				break;
			}
		}
		lAppAuxButton = AppAuxButton;

		setAux(AUX_OUTPUT);
	}
	else{
		setAux(false);
	}

	/////////////   Handle the side, head, and tail lights   /////////////
	//////////////////////////////////////////////////////////////////////
	if(ButtonPressType != PRESS_NONE){
		//if(remote_type != REMOTE_UART_DUAL){ // If single axis remote is connected
			if(single_brights_control == ButtonPressType){
				BRIGHTS = !BRIGHTS;
			}
			else if(single_all_control == ButtonPressType){
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(single_head_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = true;
					SIDELIGHTS = false;
					LIGHTS_ON = true;
				}

				else if(SIDELIGHTS)
				HEADLIGHTS = !HEADLIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(single_side_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = false;
					SIDELIGHTS = true;
					LIGHTS_ON = true;
				}
				else if(HEADLIGHTS)
				SIDELIGHTS = !SIDELIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(single_down_control == ButtonPressType){
				if(light_mode == 0)
					light_mode = light_modes - 1;
				else
					light_mode--;
				save_led_data();
			}
			else if(single_up_control == ButtonPressType){
				light_mode++;
				if(light_mode >= light_modes)
					light_mode = 0;
				save_led_data();
			}
		//}
		/*else if(remote_type == REMOTE_UART_DUAL){ // If dual axis remote is connected
			if(dual_all_control == ButtonPressType){
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(dual_head_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = true;
					SIDELIGHTS = false;
					LIGHTS_ON = true;
				}

				else if(SIDELIGHTS)
				HEADLIGHTS = !HEADLIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(dual_side_control == ButtonPressType){
				if(!LIGHTS_ON){
					HEADLIGHTS = false;
					SIDELIGHTS = true;
					LIGHTS_ON = true;
				}
				else if(HEADLIGHTS)
				SIDELIGHTS = !SIDELIGHTS;
				else
				LIGHTS_ON = !LIGHTS_ON;
			}
			else if(dual_down_control == ButtonPressType){
				if(light_mode == 0)
					light_mode = light_modes - 1;
				else
					light_mode--;
			}
			else if(dual_up_control == ButtonPressType){
				light_mode++;
				if(light_mode >= light_modes)
					light_mode = 0;
			}
			else if(TURN_ENABLED == 1 && ButtonHeldTime > 500){
				if(VescRemoteX < 110){
					if(SIDELIGHTS)
						RestoreTurnLights = true;
					SIDELIGHTS = false;
				
					TurnSignalOn = true;
					TurnSignal(true);
				}
				else if(VescRemoteX > 150){
					if(SIDELIGHTS)
						RestoreTurnLights = true;
					SIDELIGHTS = false;

					TurnSignalOn = true;
					TurnSignal(false);
				}
			}
			else if(RestoreTurnLights){
				SIDELIGHTS = true;
				RestoreTurnLights = false;
			}
		}*/
	}
}

void HandleAppRemote(){
	send_chuck_struct.js_x = 0xFF/2;
	send_chuck_struct.bt_z = false;
	send_chuck_struct.bt_c = false;
	send_chuck_struct.acc_x = 0;
	send_chuck_struct.acc_y = 0;
	send_chuck_struct.acc_z = 0;
	
	uint32_t app_remote_soft_timeout = 100;
	uint32_t app_remote_hard_timeout = 500;
	static uint32_t app_remote_time = 0;

	if(NEW_REMOTE_DATA){
		send_chuck_struct.js_y = remote_y = AppRemoteY;
		app_remote_time = millis();
		NEW_REMOTE_DATA = false;
		SEND_VESC_CHUCK = true;
		//AppControlled = true;
	} else if(SEND_VESC_CHUCK && !check_timer_expired(&app_remote_time, app_remote_soft_timeout)){
		send_chuck_struct.js_y = remote_y = AppRemoteY;
		SEND_VESC_CHUCK = true;
	} else if(SEND_VESC_CHUCK && !check_timer_expired(&app_remote_time, app_remote_hard_timeout)){
		send_chuck_struct.js_y = remote_y = 0xFF/2;
		SEND_VESC_CHUCK = true;
	}
	else{
		SEND_VESC_CHUCK = false;
	}
}

#endif