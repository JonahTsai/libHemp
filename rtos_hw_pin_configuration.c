/*
 * Copyright (C) 2013 Jonah Tsai
 *
 * This file is part of Hempstick.
 *
 * Hempstick is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * Hemmpstick is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Hempstick.  If not, see <http://www.gnu.org/licenses/>.
 *
 * hw_pin_configuration.c
 *
 * Created: 4/26/2013 8:34:14 AM
 *  Author: Jonah Tsai
 */ 


#include <rtos_hw_pin_configuration.h>
#include <gpio.h>


#ifdef __cplusplus
extern "C" {
#endif
 
 void configure_pins(void)
 {
	if(g_hw_pin_conf_table.mutex == NULL) {
		g_hw_pin_conf_table.mutex = xSemaphoreCreateMutex();
	}
	xSemaphoreTake(g_hw_pin_conf_table.mutex, portMAX_DELAY);
	for(uint16_t i = 0; i < CONF_NUM_PINS; i++) {
		if(g_hw_pin_conf_table.pin[i].conf & HW_PIN_ENABLE_MASK) {
			gpio_configure_pin((uint32_t)g_hw_pin_conf_table.pin[i].pin, (const uint32_t) g_hw_pin_conf_table.pin[i].mode);
		}
	}	 
	
	xSemaphoreGive(g_hw_pin_conf_table.mutex);
 }
 
#ifdef __cplusplus
}
#endif