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
 * rtos_button.c
 *
 * Created: 6/4/2013 11:29:09 AM
 *  Author: Jonah Tsai
 */ 
#include "rtos_button.h"

#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

#include <pio.h>
#include <pio_handler.h>

#include <malloc.h>

#include "conf_hempstead.h"

#if CONF_ENABLE_TM_STICK_IN_BUTTON
#include "tm_stick.h"
static void tm_stick_task(void* parameters);
#endif

#ifdef __cplusplus
extern "C" {
#endif

static void button_handler(uint32_t portId, uint32_t mask);
static void button_task(void *parameters);




static void button_handler(uint32_t portId, uint32_t mask) {
	portBASE_TYPE xHigherTaskWoken = pdFALSE;
	xSemaphoreTakeFromISR(g_rtos_button_data.mutex, &xHigherTaskWoken);
	rtos_button_pio_port_t* my_port = &g_rtos_button_data.ports[portId - ID_PIOA];
	Pio* calculated_pio = (Pio*)(((uint32_t)PIOA) + PIO_DELTA * (portId - ID_PIOA));
	my_port->last_update_mask = mask;
	my_port->last_update_data = calculated_pio->PIO_PDSR;
	my_port->flags |= RTOS_BUTTON_PORT_UPDATED_MASK;

	xSemaphoreGiveFromISR(g_rtos_button_data.mutex, &xHigherTaskWoken);
	xHigherTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(g_rtos_button_data.rtos_internal_task_semaphore, &xHigherTaskWoken);

	portEND_SWITCHING_ISR(xHigherTaskWoken);
}

static void process_port_data(rtos_button_pio_port_t* my_port) {
	uint32_t current_data_byte_index;
	uint8_t current_data_byte_value;
	uint32_t current_data_byte_position_index;
	
	rtos_button_pio_button_t* current_button = NULL;
	uint32_t current_mask = 0;
	uint8_t current_data_byte_mask = 0;
	for(size_t i = 0; i < 32; i++) {
		current_button = &my_port->button_conf[i];
		if(!(current_button->flags & RTOS_BUTTON_PIN_ENABLED_MASK)) {
			continue;
		}
		current_mask = 1 << i;
		current_data_byte_index = current_button->data_position / 8;
		current_data_byte_position_index = current_button->data_position % 8;
		current_data_byte_value = g_rtos_button_data.data[current_data_byte_index];
		current_data_byte_mask = 0x01 << current_data_byte_position_index;
		if((my_port->last_update_data & current_mask) == 0) { // low gets put in as 1 , high gets put in as 0 in the final data
			current_data_byte_value |= current_data_byte_mask;
		} else {
			current_data_byte_mask = ~current_data_byte_mask;
			current_data_byte_value &= current_data_byte_mask;
		}
		g_rtos_button_data.data[current_data_byte_index] = current_data_byte_value;
	}
	
	my_port->flags &= ~RTOS_BUTTON_PORT_UPDATED_MASK;
}

static void button_task(void *parameters) {
	while(1) {
		xSemaphoreTake(g_rtos_button_data.rtos_internal_task_semaphore, portMAX_DELAY);
		xSemaphoreTake(g_rtos_button_data.mutex, portMAX_DELAY);
	
		rtos_button_pio_port_t* current_port = NULL;
		for(size_t i = 0; i <= MAX_PIO_PORT_IDX; i++) {
			current_port = &g_rtos_button_data.ports[i];
			if((current_port->flags & RTOS_BUTTON_PORT_UPDATED_MASK) == 0) {
				continue;
			}
		
			// process each port data
			process_port_data(current_port);
		}
	
		xSemaphoreGive(g_rtos_button_data.mutex);
		xSemaphoreGive(g_rtos_button_data.rtos_task_semaphore); // Notify the external RTOS task to wake up.
	}
}

static void tm_stick_task(void* parameters) {
	while(1) {
		xSemaphoreTake(g_tm_stick_data.rtos_task_semaphore, portMAX_DELAY); // wait for task notification
		xSemaphoreTake(g_rtos_button_data.mutex, portMAX_DELAY); // lock down for our own data structure
		xSemaphoreTake(g_tm_stick_data.mutex, portMAX_DELAY); // lock down tm stick's data structure
		int current_position = -1;
		uint32_t* stick_value_buf = (uint32_t*)g_tm_stick_data.data;
		uint32_t stick_value = *stick_value_buf;
		uint32_t current_tm_stick_mask = 0x0001;
		uint8_t current_position_mask = 0;
		uint8_t current_position_idx = 0;
		for(size_t i = 0; i < 24; i++) {
			current_position = g_tm_stick_data.button_position[i];
			if(current_position >= 0) {
				current_position_idx = current_position / 8;
				current_position_mask = 0x01 << (current_position % 8);
				if((stick_value & current_tm_stick_mask) == 0) {
					current_position_mask = ~current_position_mask;
					g_rtos_button_data.data[current_position_idx] &= current_position_mask;
					} else {
					g_rtos_button_data.data[current_position_idx] |= current_position_mask; // turn the bit on
				}
			}
			current_tm_stick_mask = current_tm_stick_mask << 1;
		}
		
		xSemaphoreGive(g_tm_stick_data.mutex);
		xSemaphoreGive(g_rtos_button_data.mutex);
		
		
		xSemaphoreGive(g_rtos_button_data.rtos_task_semaphore); // Notify
	}
}

void rtos_button_init(uint16_t num_buttons, bool is_enable_tm_stick) {
	if(g_rtos_button_data.mutex == NULL) {
		g_rtos_button_data.mutex = xSemaphoreCreateMutex();
	}
	
	if(g_rtos_button_data.rtos_internal_task_semaphore == NULL) {
		vSemaphoreCreateBinary(g_rtos_button_data.rtos_internal_task_semaphore);
		xSemaphoreTake(g_rtos_button_data.rtos_internal_task_semaphore, 0); // take it immediately so the waiting task doesn't fire on the first run.
	}
	
	
	if(g_rtos_button_data.rtos_task_semaphore == NULL) {
		vSemaphoreCreateBinary(g_rtos_button_data.rtos_task_semaphore);
		xSemaphoreTake(g_rtos_button_data.rtos_task_semaphore, 0); // take it immediately so the waiting task doesn't fire on the first run.
	}
	
	
	if(g_rtos_button_data.data == NULL) {
		g_rtos_button_data.num_button = num_buttons;
		size_t size = 0;
		if((num_buttons % 8) > 0) {
			size = 1;
		}
		size += num_buttons / 8;
		g_rtos_button_data.data = malloc(size);
		if(g_rtos_button_data.data != NULL) {
			for(size_t i = 0; i < size; i++) {
				g_rtos_button_data.data[i] = 0;
			}
		}
	}
	
	
	for(size_t i = 0; i <=  MAX_PIO_PORT_IDX; i++) {
		uint32_t mask = 0;
		NVIC_DisableIRQ(PIOA_IRQn + i);
		g_rtos_button_data.ports[i].last_update_mask = 0; // init, clear out all.
		g_rtos_button_data.ports[i].last_update_data = 0;
		g_rtos_button_data.ports[i].flags = 0;
		
		for(size_t j = 0; j < 32; j++) {
			if((g_rtos_button_data.ports[i].button_conf[j].flags & RTOS_BUTTON_PIN_ENABLED_MASK) > 0) {
				// enabled.
				mask |= (1 << j);
			}
		}
		if(mask > 0) {
			uint32_t calculated_pio = (uint32_t)PIOA + PIO_DELTA * i;
			uint32_t calculated_pio_id = ID_PIOA + i;
			pio_handler_set((Pio*)calculated_pio, calculated_pio_id,  mask,  PIO_IT_EDGE, button_handler);
			pio_handler_set_priority((Pio*)calculated_pio, PIOA_IRQn + i, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY); // Make sure the interrupt priority is not higher than FreeRTOS mandates. Set proiroty uses 0 to 2* configPRIO_BITS, unshifted. It internally shift it to the left. So, don't use the shfited priority value.
			NVIC_EnableIRQ(PIOA_IRQn + i);
			pio_enable_interrupt((Pio*)calculated_pio, mask);
		}
	}
	
	// create the internal FreeRTOS button processing task waiting on the rtos_internal_task_semaphore
	xTaskHandle button_task_handle;
	xTaskCreate(button_task, (const signed char*)"Button Processing Task", configMINIMAL_STACK_SIZE * 2, NULL, ( ( unsigned portBASE_TYPE ) configTIMER_TASK_PRIORITY ) | portPRIVILEGE_BIT, &button_task_handle);
	
	if(is_enable_tm_stick) {
		xTaskHandle tm_stick_handle;
		xTaskCreate(tm_stick_task, (const signed char*)"Button Processing Task", configMINIMAL_STACK_SIZE * 2, NULL, ( ( unsigned portBASE_TYPE ) configTIMER_TASK_PRIORITY ) | portPRIVILEGE_BIT, &tm_stick_handle);
	}
		
}



#ifdef __cplusplus
}
#endif
