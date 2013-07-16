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
 * hw_pin_configuratioin.h
 *
 * Created: 4/26/2013 8:33:44 AM
 *  Author: Jonah Tsai
 */ 

#include <FreeRTOS.h>
#include <semphr.h>

#include "conf_hempstead.h"

#ifndef RTOS_HW_PIN_CONFIGURATIOIN_H_
#define RTOS_HW_PIN_CONFIGURATIOIN_H_



#define HW_PIN_ENABLE_MASK 0x01


#ifdef __cplusplus
extern "C" {
#endif


typedef struct {
	uint32_t pin;
	uint32_t mode;
	uint8_t conf;
} hw_pin_configuration;

typedef struct {
	hw_pin_configuration pin[CONF_NUM_PINS];
	xSemaphoreHandle mutex; // mutex for protecting this data structure
} hw_pin_configuration_table;

extern hw_pin_configuration_table g_hw_pin_conf_table;

void configure_pins(void);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_HW_PIN_CONFIGURATIOIN_H_ */