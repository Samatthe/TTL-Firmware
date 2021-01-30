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

#ifndef TIMING_H
#define TIMING_H

struct tc_module tc0;

uint32_t millis(void);
uint32_t micros(void);
void configure_tc(void);
void check_time(uint32_t* time_var);
void check_time_micros(uint32_t* time_var);
bool check_timer_expired(uint32_t* time_var, uint32_t dur);

void configure_tc(void)
{
	struct tc_config config_tc;
	tc_get_config_defaults(&config_tc);
	
	config_tc.counter_size = TC_COUNTER_SIZE_32BIT;
	config_tc.count_direction = TC_COUNT_DIRECTION_UP;
	config_tc.clock_source =  GCLK_CLKCTRL_GEN_GCLK0;
	config_tc.clock_prescaler = TC_CLOCK_PRESCALER_DIV1;

	tc_init(&tc0, TC4, &config_tc);
	tc_enable(&tc0);
}



uint32_t millis()
{
	return (tc_get_count_value(&tc0)/7500); //Overflows every 9.5m
}

uint32_t micros()
{
	return (tc_get_count_value(&tc0)/7.5);
}

void check_time(uint32_t* time_var){
	if(*time_var > millis())
		*time_var = 0;
}


void check_time_micros(uint32_t* time_var){
	if(*time_var > micros())
	*time_var = 0;
}

bool check_timer_expired(uint32_t* time_var, uint32_t dur){
	check_time(time_var);
	return (millis() - *time_var) > dur;
}

#endif