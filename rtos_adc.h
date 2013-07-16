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
 * rtos_adc.h
 *
 * ASF modules required:
 *	ADC
 *
 * A FreeRTOS implementation of ADC module using PDC and TimerCounter hardware ADC triggering.
 *
 * This ADC module currently only uses the one and only one ADC. If your MCU has more than one ADC module, sorry, not supported.
 * 
 * It utilizes PDC transfer and automatic TIOA0 hardware trigger.
 * You must call init_adc_data() by providing an ADC overall configuraiton flag, and an array of channel configuration flags.
 * The init_adc_data() function will automatically initialize the g_adc_data data structure, including the mutexes, and enable the enabled ADC channels. (You don't need to enable individual ADC channels yourself).
 * It will then setup PDC transfer automatically when you call adc_start().
 * With adc_start(), you have to provide the ADC conversion frequency in Hz. This is used to setup the TIOA0 HW trigger to start ADC conversions.
 * An ADC interrupt handler is implemented internally, not exposed, which automatically reloads the PDC transfer buffer whenever a round of ADC conversion is completed.
 * It then uses ADC tags to copy the data to appropriate data[] array slots.
 * The reason for the two buffers is that PDC works in the background on hardware, so we cannot protect it with an RTOS mutex.
 * The only place we can guarantee that there is no data contention is in the ADC interrupt handler.
 * So, we have a pdc_sample_data[] array for PDC to free run it under HW trigger (except when we are in the ADC interrupt handler).
 * Then, in the ADC handler we copy the pdc_sample_data[] out to data[].
 * Remember, data[] is protected by the mutex, but pdc_sample_data[] is not.
 * 
 * The .rtos_task_mutex is there to synchronize the task for processing the .data[].
 * The deal is that now that a FreeRTOS task would block on the .rtos_task_mutex, and then every time the ADC interrupt handler has new data, it then give the mutex and unblocks the FreeRTOS task.
 * For instance, say you have a USB processing. You can just write a FreeRTOS task doing an infinite loop, blocking on the .rtos_task_mutex. Each time there is new ADC data, your task gets unblocked and it locks on the .mutex, and then send a USB packet.
 * 
 * You must provide the MAX_ADC_CHANNEL macro with an integer to specify the size of the "potential" number of channels possible.
 * It does not need to be the total number of the hardware ADC channels. It is the max. number of channels you may use, as long as it is smaller than the hardware number.
 * Also, the code does not check whether the MAX_ADC_CHANNEL is set to be greater than the hardware channel. It is up to you, the user, to ensure that condition is met.
 *
 * Depends on:
 *		ASF Modules: ADC, TC, FreeRTOS (no extra FreeRTOS PDC modules requires, we do our own PDC setup).
 *
 * Pins Use:
 *		Any ADC input pins you configure.
 *		TIOA0 (whatever that is on the MCU).
 *			If you also want this to be used to trigger something else, you must enabled the pin Peripheral function to let it output and then wire it up to the input in of the modules to be triggered.
 *			ADC TIOA0 triggering is internally wired in the MCU, you don't have to do external wiring.
 *			For instance, to re-use this TC pulse to also trigger SSC receiver, you will wire up the TIOA0 pin to the RF pin of the SSC.
 *			e.x. I put a 2KHz TIAO0 to auto-trigger ADC conversion. But I also need to trigger receiver of SSC to read ThrustMaster's Cougar/Warthog Stick, so I wired up PB28 to PB17 on an SAM3XE (Arduino Due/X). This way, whenever an ADC conversion is triggered,
 *			the stick readout using SSC is also triggered automatically.
 *	
 * Created: 4/7/2013 9:46:29 PM
 *  Author: Jonah Tsai
 */ 


#ifndef RTOS_ADC_H_
#define RTOS_ADC_H_

#include <FreeRTOS.h>
#include <semphr.h>

#include <adc.h>

#include "conf_hempstead.h"

#define ADC_CHANNEL_ENABLE_MASK 0x01u

#define ADC_PDC_ENABLE_MASK 0x01u

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t nextIdx;
	uint8_t num;
	uint32_t acc;
	uint16_t history[(1 << CONF_ADC_DECIMATION_BITS)];
} rtos_adc_decimation_buffer;

/************************************************************************
 * ADC structure.
 * Will only use one ADC.
 * .data[MAX_ADC_CHANNEL] : the array to store adc data, by MCU's ADC channel #. 
 * .channel_flags[MAX_ADC_CHANNEL] : ADC configuration flags for each channel.
 * .num_channel_enabled : The total number of ADC channels enabled. Updated each time init_adc_data() is called.
 * .adc_config : The overall ADC configuration flag.
 * .mutex : The FreeRTOS mutex protecting this structure.
 * .rtos_task_mutex : The FreeRTOS mutex for synchronizing data access with the FreeRTOS ADC processing task.
 * .pdc_sample_data[MAX_ADC_CHANNEL] : The PDC data buffer.
 ************************************************************************/
typedef struct {
	uint16_t data[MAX_ADC_CHANNEL];
	// uint32_t data_acc[MAX_ADC_CHANNEL];
	rtos_adc_decimation_buffer decimation_buf[MAX_ADC_CHANNEL];
	uint8_t channel_flags[MAX_ADC_CHANNEL];
	uint8_t num_channel_enabled;
	uint16_t adc_config;
	xSemaphoreHandle mutex; // mutex for protecting this data structure
	xSemaphoreHandle rtos_task_semaphore; // mutex for FreeRTOS adc processing task synchronization.
	uint16_t pdc_sample_data[MAX_ADC_CHANNEL]; // This buffer is for pdc, thus not protected by the mutex. It relies solely on ADC ISR reloading the buffer. It's a buffer used by DMA.
} rtos_adc_data_type;

extern rtos_adc_data_type g_adc_data;


void init_adc_data(uint16_t adc_flags);
void start_adc(uint32_t adc_trigger_hz);

#ifdef __cplusplus
}
#endif

#endif /* RTOS_ADC_H_ */