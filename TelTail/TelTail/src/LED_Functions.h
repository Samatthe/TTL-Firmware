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

#ifndef LEDFUNCS_H
#define LEDFUNCS_H

// Define LED Functions
#define TURN_OFF_TIME 400
#define TURN_ON_TIME 250

struct tcc_module tcc0;
struct tcc_module tcc1;
struct tcc_module tcc2;
void configure_LED_PWM(void);
void setLeftRGB(uint16_t red, uint16_t green, uint16_t blue);
void setRightRGB(uint16_t red, uint16_t green, uint16_t blue);
void setWhite(uint16_t white);
void setRed(uint16_t red);
void setAux(bool aux);
struct RGB_Vals setCycleColor(uint16_t _upColor, uint16_t _downColor, int _cycle);
void setCycleColorSingle(uint16_t upColor, uint16_t downColor, int cycle, bool left);
void setConstBases(void);
void ERROR_LEDs(uint8_t error_type);
void TurnSignal(bool direction);




// Implement LED Functions



// Configure all of the LED ports as PWM outputs
void configure_LED_PWM(void)
{
struct tcc_config config_tcc;
tcc_get_config_defaults(&config_tcc, TCC0);
config_tcc.counter.period = 0xFFFF;
config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

config_tcc.compare.match[0] = 0;
config_tcc.compare.match[1] = 0;
config_tcc.compare.match[2] = 0;
config_tcc.compare.match[3] = 0;
config_tcc.pins.enable_wave_out_pin[0] = true;
config_tcc.pins.enable_wave_out_pin[1] = true;
config_tcc.pins.enable_wave_out_pin[2] = true;
config_tcc.pins.enable_wave_out_pin[3] = true;
config_tcc.pins.wave_out_pin[0]        = PIN_PA14F_TCC0_WO4; 
config_tcc.pins.wave_out_pin[1]        = PIN_PB11F_TCC0_WO5; 
config_tcc.pins.wave_out_pin[2]        = PIN_PA10F_TCC0_WO2; 
config_tcc.pins.wave_out_pin[3]        = PIN_PA11F_TCC0_WO3;  
config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA14F_TCC0_WO4;
config_tcc.pins.wave_out_pin_mux[1]    = MUX_PB11F_TCC0_WO5;
config_tcc.pins.wave_out_pin_mux[2]    = MUX_PA10F_TCC0_WO2;
config_tcc.pins.wave_out_pin_mux[3]    = MUX_PA11F_TCC0_WO3;


tcc_init(&tcc0, TCC0, &config_tcc);
tcc_enable(&tcc0);


tcc_get_config_defaults(&config_tcc, TCC1);
config_tcc.counter.period = 0xFFFF;
config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

//config_tcc.compare.match[0] = 0;
config_tcc.compare.match[1] = 0;
//config_tcc.pins.enable_wave_out_pin[0] = true;
config_tcc.pins.enable_wave_out_pin[1] = true;
//config_tcc.pins.wave_out_pin[0]        = PIN_PA06E_TCC1_WO0;
config_tcc.pins.wave_out_pin[1]        = PIN_PA07E_TCC1_WO1;
//config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA06E_TCC1_WO0;
config_tcc.pins.wave_out_pin_mux[1]    = MUX_PA07E_TCC1_WO1;

tcc_init(&tcc1, TCC1, &config_tcc);
// Configure the capture channel to read pulse width of PPM_IN pin
TCC1->CTRLA.reg  |= TCC_CTRLA_CPTEN0;
TCC1->EVCTRL.reg |= TCC_EVCTRL_TCEI1 | TCC_EVCTRL_EVACT1_PWP;
tcc_enable(&tcc1);


tcc_get_config_defaults(&config_tcc, TCC2);
config_tcc.counter.period = 0xFFFF;
config_tcc.compare.wave_generation = TCC_WAVE_GENERATION_SINGLE_SLOPE_PWM;

config_tcc.compare.match[0] = 0;
config_tcc.compare.match[1] = 0;
config_tcc.pins.enable_wave_out_pin[0] = true;
config_tcc.pins.enable_wave_out_pin[1] = true;
config_tcc.pins.wave_out_pin[0]        = PIN_PA12E_TCC2_WO0;
config_tcc.pins.wave_out_pin[1]        = PIN_PA13E_TCC2_WO1;
config_tcc.pins.wave_out_pin_mux[0]    = MUX_PA12E_TCC2_WO0;
config_tcc.pins.wave_out_pin_mux[1]    = MUX_PA13E_TCC2_WO1;

tcc_init(&tcc2, TCC2, &config_tcc);
tcc_enable(&tcc2);
}

void setLeftRGB(uint16_t red, uint16_t green, uint16_t blue) {
	RGB_Ouptut.LB = blue;
	RGB_Ouptut.LG = green;
	RGB_Ouptut.LR = red;
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (0), red);
	tcc_set_compare_value(&tcc2, (enum tcc_match_capture_channel) (1), green);
	tcc_set_compare_value(&tcc1, (enum tcc_match_capture_channel) (1), blue);
}

void setRightRGB(uint16_t red, uint16_t green, uint16_t blue) {
	RGB_Ouptut.RB = blue;
	RGB_Ouptut.RG = green;
	RGB_Ouptut.RR = red;
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (3), red);		
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (1), green);
	tcc_set_compare_value(&tcc0, (enum tcc_match_capture_channel) (2), blue);
}

void setWhite(uint16_t white) {
	head = white;
	//tcc_set_compare_value(&tcc1, (enum tcc_match_capture_channel) (0), white);
	port_pin_set_output_level(PIN_PA06E_TCC1_WO0, white); // Changed to non-pwm due to pulse width reading on PPM_IN pin
}

void setRed(uint16_t red) {
	brake = red;
	tcc_set_compare_value(&tcc2, (enum tcc_match_capture_channel) (0), red);
}

void setAux(bool aux) {
	port_pin_set_output_level(AUX_PIN,aux);
}

struct RGB_Vals setCycleColor(uint16_t _upColor, uint16_t _downColor, int _cycle){
	struct RGB_Vals color;
	if(_cycle == 0){
		color.LR=_upColor;
		color.LG = 0;
		color.LB = _downColor;
		color.RR=_upColor;
		color.RG = 0;
		color.RB = _downColor;
	}
	if(_cycle == 1){
		color.LR=_downColor;
		color.LG = _upColor;
		color.LB = 0;
		color.RR=_downColor;
		color.RG = _upColor;
		color.RB = 0;
	}
	if(_cycle == 2){
		color.LR=0;
		color.LG = _downColor;
		color.LB = _upColor;
		color.RR=0;
		color.RG = _downColor;
		color.RB = _upColor;
	}
	return color;
}

void setConstBases(){
	ColorBase[MODE_STATIC] = COLOR_STATIC;
	RateBase[MODE_STATIC] = RATE_STATIC;
	BrightBase[MODE_STATIC] = BRIGHT_STATIC;
	
	ColorBase[MODE_COLOR_CYCLE] = COLOR_COLOR_CYCLE;
	RateBase[MODE_COLOR_CYCLE] = RATE_STATIC;
	BrightBase[MODE_COLOR_CYCLE] = BRIGHT_STATIC;
	
	ColorBase[MODE_COMPASS_CYCLE] = COLOR_COMPASS;
	RateBase[MODE_COMPASS_CYCLE] = RATE_STATIC;
	BrightBase[MODE_COMPASS_CYCLE] = BRIGHT_STATIC;
	
	ColorBase[MODE_THROTTLE] = COLOR_THROTTLE;
	RateBase[MODE_THROTTLE] = RATE_STATIC;
	BrightBase[MODE_THROTTLE] = BRIGHT_STATIC;
	
	ColorBase[MODE_RPM_CYCLE] = COLOR_COLOR_CYCLE;
	RateBase[MODE_RPM_CYCLE] = RATE_RPM;
	BrightBase[MODE_RPM_CYCLE] = BRIGHT_RPM;
	
	ColorBase[MODE_RPM_THROTTLE] = COLOR_THROTTLE;
	RateBase[MODE_RPM_THROTTLE] = RATE_STATIC;
	BrightBase[MODE_RPM_THROTTLE] = BRIGHT_RPM;
	
	ColorBase[MODE_X_ACCEL] = COLOR_COLOR_CYCLE;
	RateBase[MODE_X_ACCEL] = RATE_STATIC;
	BrightBase[MODE_X_ACCEL] = BRIGHT_X_ACCEL;
	
	ColorBase[MODE_Y_ACCEL] = COLOR_Y_ACCEL;
	RateBase[MODE_Y_ACCEL] = RATE_STATIC;
	BrightBase[MODE_Y_ACCEL] = BRIGHT_STATIC;
}

// Flash the side LEDs red until restart
// 0: Red, 1: Blue, 2:Green, 3: Teal, 4: Yellow, 5:Purple
void ERROR_LEDs(uint8_t error_type){
	uint32_t timer = 0;

	uint16_t tempR = 0, tempG = 0, tempB = 0;
	if(error_type == 0 || error_type == 4  || error_type == 5)
		tempR = 0xFFFF;
	if(error_type >= 2 && error_type <= 4)
		tempG = 0xFFFF;
	if(error_type == 1 || error_type == 3 || error_type == 5)
		tempB = 0xFFFF;

	while(1){
		setLeftRGB(0,0,0);
		setRightRGB(0,0,0);
		
		setRed(0);
		setWhite(0);
		setAux(0);

		while(millis() - timer < 1000) {
			check_time(&timer);
		}
		timer = millis();

		setLeftRGB(tempR,tempG,tempB);
		setRightRGB(tempR,tempG,tempB);
		
		setRed(0xFFFF);
		setWhite(0xFFFF);
		setAux(1);

		while(millis() - timer < 250) {
			check_time(&timer);
		}
		timer = millis();
	}
}

// true = left    false = right
uint32_t turnTimer = 0;
uint16_t turnOutput = 0;
void TurnSignal(bool direction){

	check_time(&turnTimer);
	if(turnOutput == 0x0 && (millis() - turnTimer >= TURN_OFF_TIME)){
		turnOutput = 0xFFFF;
		turnTimer = millis();
	} else if(turnOutput == 0xFFFF && (millis() - turnTimer >= TURN_ON_TIME)){
		turnOutput = 0;
		turnTimer = millis();
	}

	if(direction == true){
		setLeftRGB(turnOutput,turnOutput,0);
		setRightRGB(0,0,0);
	} else {
		setLeftRGB(0,0,0);
		setRightRGB(turnOutput,turnOutput,0);
	}
}

#endif

