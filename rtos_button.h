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
 * rtos_button.h
 *
 * Created: 6/4/2013 11:17:19 AM
 *  Author: Jonah Tsai
 */ 


#ifndef RTOS_BUTTON_H_
#define RTOS_BUTTON_H_

#include <FreeRTOS.h>
#include <semphr.h>

#include <stdint.h>
#include <pio.h>

#ifdef __cplusplus
extern "C" {
#endif

#define RTOS_BUTTON_PIN_ENABLED_MASK 0x0001
#define RTOS_BUTTON_PORT_UPDATED_MASK 0x01

// The start button index of the HAT switch(s), currently reserve 4 assuming there is only one hat switch.s
#define RTOS_BUTTON_HAT_POSITION_IDX	(UINT32_MAX - 3)
#define RTOS_BUTTON_OUT_OF_RANGE_IDX	RTOS_BUTTON_HAT_POSITION_IDX
#define RTOS_HAT_NO_STATE_VALUE			(UINT8_MAX)

typedef struct {
	uint32_t data_position;
	uint16_t flags;
} rtos_button_pio_button_t;

typedef struct {
	uint32_t last_update_mask;
	uint32_t last_update_data;
	uint8_t flags;
	rtos_button_pio_button_t button_conf[32];
} rtos_button_pio_port_t;

typedef struct {
	uint8_t* data;
	size_t num_button;
	uint8_t* hat_data;
	size_t num_hat;
	
	// PIO port stuff
#ifdef ID_PIOF
	rtos_button_pio_port_t ports[6];
#define MAX_PIO_PORT_IDX 5
#elif defined(ID_PIOE)
	rtos_button_pio_port_t ports[5];
#define MAX_PIO_PORT_IDX 4
#elif defined(ID_PIOD)
	rtos_button_pio_port_t ports[4];
#define MAX_PIO_PORT_IDX 3
#elif defined(ID_PIOC)
	rtos_button_pio_port_t ports[3];
#define MAX_PIO_PORT_IDX 2
#elif defined(ID_PIOB)
	rtos_button_pio_port_t ports[2];
#define MAX_PIO_PORT_IDX 1
#elif defined(ID_PIOA)
	rtos_button_pio_port_t ports[1];
#define MAX_PIO_PORT_IDX 0
#endif
	
	// RTOS Semaphores
	xSemaphoreHandle mutex; // mutex for protecting this data structure
	xSemaphoreHandle rtos_internal_task_semaphore; // mutex for FreeRTOS internal button processing task synchronization.
	xSemaphoreHandle rtos_task_semaphore; // mutex for FreeRTOS button external task synchronization.
} rtos_button_data_t;

extern rtos_button_data_t g_rtos_button_data;

void rtos_button_init(uint16_t num_buttons, bool is_enable_tm_stick);


#ifdef __cplusplus
}
#endif

#endif /* RTOS_BUTTON_H_ */