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
 * rtos_adc.c
 *
 * Created: 4/7/2013 9:46:43 PM
 *  Author: Jonah Tsai
 */ 

#include "rtos_adc.h"
#include <string.h>
#include <semphr.h>

#include <asf.h>

#ifdef __cplusplus
extern "C" {
#endif


void init_adc_data(uint16_t adc_flags)
{
	// If the mutex is not already initialized, create it.
	if(g_adc_data.mutex == NULL) {
		g_adc_data.mutex = xSemaphoreCreateMutex();
	}
	
	if(g_adc_data.rtos_task_semaphore == NULL) {
		vSemaphoreCreateBinary(g_adc_data.rtos_task_semaphore);
	}
	
	// Take the mutex.
	xSemaphoreTake(g_adc_data.mutex, portMAX_DELAY);

	g_adc_data.adc_config = adc_flags;
	uint8_t num_channel_enabled = 0;
	for(int i = 0; i < MAX_ADC_CHANNEL; i++) {
		if(g_adc_data.channel_flags[i] & ADC_CHANNEL_ENABLE_MASK) {
			num_channel_enabled++;
		}
		g_adc_data.decimation_buf[i].acc = 0;
		g_adc_data.decimation_buf[i].nextIdx = 0;
		g_adc_data.decimation_buf[i].num = 0;
		for(int j = 0; j < (1 << CONF_ADC_DECIMATION_BITS); j++)  {
			g_adc_data.decimation_buf[i].history[j] = 0;
		}
	}
	g_adc_data.num_channel_enabled = num_channel_enabled;
	
	memset(g_adc_data.data, 0, MAX_ADC_CHANNEL);
	memset(g_adc_data.pdc_sample_data, 0, MAX_ADC_CHANNEL);
	
	xSemaphoreGive(g_adc_data.mutex);
}


static void configure_time_trigger_for_adc(uint32_t adc_trigger_hz)
{
	uint32_t ul_div = 0;
	uint32_t ul_tc_clks = 0;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();
	uint32_t decimation_trigger_hz = adc_trigger_hz * (1 << (CONF_ADC_OVERSAMPLE_RESOLUTION_INCREASE_BITS + 1)); // Oversample frequency for decimation, * (4^n ==  2^(n+1))

	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_TC0);

	/* Configure TC for a 1Hz frequency and trigger on RC compare. */
	tc_find_mck_divisor(adc_trigger_hz, ul_sysclk, &ul_div, &ul_tc_clks, ul_sysclk);
	tc_init(TC0, 0, ul_tc_clks | TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
	tc_write_ra(TC0, 0, (ul_sysclk / ul_div) / 2 / decimation_trigger_hz);
	tc_write_rc(TC0, 0, (ul_sysclk / ul_div) / 1 / decimation_trigger_hz);

	/* Start the Timer. */
	tc_start(TC0, 0);
	/* Set TIOA0 trigger. */
	adc_configure_trigger(ADC, ADC_TRIG_TIO_CH_0, 0);
}


// This function will be called inside ISR.
static uint32_t reload_adc_read_buffer(Adc * p_adc, uint16_t * buffer, uint32_t size)
{
	/* Check if the first PDC bank is free. */
	if ((p_adc->ADC_RCR == 0) && (p_adc->ADC_RNCR == 0)) {
		p_adc->ADC_RPR = (uint32_t) buffer;
		p_adc->ADC_RCR = size;
		p_adc->ADC_PTCR = ADC_PTCR_RXTEN;
		return 1;
	} else {	/* Check if the second PDC bank is free. */
		if (p_adc->ADC_RNCR == 0) {
			p_adc->ADC_RNPR = (uint32_t) buffer;
			p_adc->ADC_RNCR = size;
			return 1;
		} else {
			return 0;
		}
	}
}




void ADC_Handler(void)
{
	// This implementation uses more data space, but less sampling frequency, leaving enough frequency to do oversampling to increase resolution.
	// It uses a running circular buffer to keep data history and accumulator for average.
	// This is more prone to noises, as one noise spike will be in the running history for the number of ADC_DECIMATION_BUF_SIZE times. So, it needs needs longer history data than the other implementation without history data and higher sampling frequency.
	// This method also require slightly higher CPU power than the other.
	// Both work fine though (as long as you have enough memory to accommodate the data space requirement.
	// TODO: could dynamically allocate the decimation buffer to save some memory though. Not a very high priority though.
	uint32_t ul_temp;
	uint8_t uc_ch_num;	
	// static uint32_t acc_count = 0;
	portBASE_TYPE xHigherTaskWoken = pdFALSE;

	xSemaphoreTakeFromISR(g_adc_data.mutex, &xHigherTaskWoken);
	if ((g_adc_data.adc_config & ADC_PDC_ENABLE_MASK)) {
		/* With PDC transfer */
		if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF) {
			// copy the data out before reloading the PDC buffer so that there is no chance PDC and this copy has data contention.
			uint32_t num_channels_2_process = g_adc_data.num_channel_enabled;
			for(uint32_t i = 0; i < num_channels_2_process; i++) {
				uint16_t channel_num = (g_adc_data.pdc_sample_data[i] & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos;
				rtos_adc_decimation_buffer* current_buffer = &g_adc_data.decimation_buf[channel_num];
				current_buffer->acc -= current_buffer->history[current_buffer->nextIdx]; // take out the old last data
				current_buffer->history[current_buffer->nextIdx] = (g_adc_data.pdc_sample_data[i] & ADC_LCDR_LDATA_Msk); // new data
				current_buffer->acc += current_buffer->history[current_buffer->nextIdx]; // accumulate
				current_buffer->nextIdx++; // advance the nextIdx
				current_buffer->nextIdx %= (1 << CONF_ADC_DECIMATION_BITS); // round over the nextIdx
				if(current_buffer->num < (1 << CONF_ADC_DECIMATION_BITS)) {
					current_buffer->num++;
					g_adc_data.data[channel_num] = current_buffer->acc / current_buffer->num;
				} else {
					g_adc_data.data[channel_num] = current_buffer->acc >> (CONF_ADC_DECIMATION_BITS - CONF_ADC_OVERSAMPLE_RESOLUTION_INCREASE_BITS);
				}
			}
			reload_adc_read_buffer(ADC, g_adc_data.pdc_sample_data, g_adc_data.num_channel_enabled);
		}
	} else {
		/* Without PDC transfer */
		/* Untested */
		if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY) { 
			ul_temp = adc_get_latest_value(ADC);
			for (uint32_t i = 0; i < g_adc_data.num_channel_enabled; i++) {
				uc_ch_num = (ul_temp & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos; // take the ADC channel tag.
				// g_adc_data.data[uc_ch_num] = ul_temp & ADC_LCDR_LDATA_Msk; // mask off the ADC channel tag.
				// g_adc_data.data_acc[uc_ch_num] += (ul_temp & ADC_LCDR_LDATA_Msk); // mask off the ADC channel tag and then put in accumulator.
				
				rtos_adc_decimation_buffer* current_buffer = &g_adc_data.decimation_buf[uc_ch_num];
				current_buffer->acc -= current_buffer->history[current_buffer->nextIdx]; // take out the old last data
				current_buffer->history[current_buffer->nextIdx] = (ul_temp & ADC_LCDR_LDATA_Msk); // new data
				current_buffer->acc += current_buffer->history[current_buffer->nextIdx]; // accumulate
				current_buffer->nextIdx++; // advance the nextIdx
				current_buffer->nextIdx %= (1 << CONF_ADC_DECIMATION_BITS); // round over the nextIdx
				if(current_buffer->num < (1 << CONF_ADC_DECIMATION_BITS)) {
					current_buffer->num++;
					g_adc_data.data[uc_ch_num] = current_buffer->acc / current_buffer->num;
				} else {
					g_adc_data.data[uc_ch_num] = current_buffer->acc >> CONF_ADC_DECIMATION_BITS;
				}

				
			}
		}
	}

#if 0
	// This implementation uses much higher sampling frequency, but uses less data space.
	// Essentially, it uses 2^CONF_ADC_DECIMATION_BITS samples to form one real data point.
	if ((g_adc_data.adc_config & ADC_PDC_ENABLE_MASK)) {
		/* With PDC transfer */
		if ((adc_get_status(ADC) & ADC_ISR_RXBUFF) == ADC_ISR_RXBUFF) {
			// g_adc_sample_data.us_done = ADC_DONE_MASK;
			// copy the data out before reloading the PDC buffer so that there is no chance PDC and this copy has data contention.
			for(uint32_t i = 0; i < g_adc_data.num_channel_enabled; i++) {
				uint16_t channel_num = (g_adc_data.pdc_sample_data[i] & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos;
				// g_adc_data.data[channel_num] = g_adc_data.pdc_sample_data[i] & ADC_LCDR_LDATA_Msk; // mask off the ADC channel tag.
				g_adc_data.data_acc[channel_num] += (g_adc_data.pdc_sample_data[i] & ADC_LCDR_LDATA_Msk); // mask off the ADC channel tag and then put in accumulator.
				acc_count++;
				if(acc_count >= (1 << CONF_ADC_DECIMATION_BITS)) {
					g_adc_data.data[channel_num] = g_adc_data.data_acc[channel_num] >> CONF_ADC_DECIMATION_BITS;
					acc_count = 0;
					g_adc_data.data_acc[channel_num] = 0;
				}
			}
			reload_adc_read_buffer(ADC, g_adc_data.pdc_sample_data, g_adc_data.num_channel_enabled);
		}
	} else {
		/* Without PDC transfer */
		/* Untested */
		if ((adc_get_status(ADC) & ADC_ISR_DRDY) == ADC_ISR_DRDY) { 
			ul_temp = adc_get_latest_value(ADC);
			for (uint32_t i = 0; i < g_adc_data.num_channel_enabled; i++) {
				uc_ch_num = (ul_temp & ADC_LCDR_CHNB_Msk) >> ADC_LCDR_CHNB_Pos; // take the ADC channel tag.
				// g_adc_data.data[uc_ch_num] = ul_temp & ADC_LCDR_LDATA_Msk; // mask off the ADC channel tag.
				g_adc_data.data_acc[uc_ch_num] += (ul_temp & ADC_LCDR_LDATA_Msk); // mask off the ADC channel tag and then put in accumulator.
				acc_count++;
				if(acc_count >= (1 << CONF_ADC_DECIMATION_BITS)) {
					g_adc_data.data[uc_ch_num] = g_adc_data.data_acc[uc_ch_num] >> CONF_ADC_DECIMATION_BITS;
					acc_count = 0;
					g_adc_data.data_acc[uc_ch_num] = 0;
				}
			}
		}
	}
#endif
	xSemaphoreGiveFromISR(g_adc_data.mutex, &xHigherTaskWoken);
	
	xHigherTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(g_adc_data.rtos_task_semaphore, &xHigherTaskWoken); // if there is an RTOS waiting for the ADC task mutex, wake it up.
	portEND_SWITCHING_ISR(xHigherTaskWoken);
}

void start_adc(uint32_t adc_trigger_hz)
{
	pmc_enable_periph_clk(ID_ADC);
	
	adc_init(ADC, sysclk_get_cpu_hz(), 64000000, 8);
	/*
	 * Formula: ADCClock = MCK / ( (PRESCAL+1) * 2 )
	 * For example, MCK = 64MHZ, PRESCAL = 4, then:
	 *     ADCClock = 64 / ((4+1) * 2) = 6.4MHz;
	 */
	/* Set ADC clock. */
	/* Formula:
	 *     Startup  Time = startup value / ADCClock
	 *     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
	 *     Tracking Time = (TRACKTIM + 1) / ADCClock
	 *     Settling Time = settling value / ADCClock
	 * For example, ADC clock = 6MHz (166.7 ns)
	 *     Startup time = 512 / 6MHz = 85.3 us
	 *     Transfer Time = (1 * 2 + 3) / 6MHz = 833.3 ns
	 *     Tracking Time = (0 + 1) / 6MHz = 166.7 ns
	 *     Settling Time = 3 / 6MHz = 500 ns
	 */
	/* Set ADC timing. */
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
	
	/* Enable channel number tag. */
	adc_enable_tag(ADC);

	xSemaphoreTake(g_adc_data.mutex, portMAX_DELAY);
	// Set ADC resolution
	// TODO: swtich to different resolution for different ADC
	adc_set_resolution(ADC, ADC_12_BITS);
	
	// Enable the configured ADC channels
	adc_disable_all_channel(ADC);
	for(int i = 0; i < MAX_ADC_CHANNEL; i++) {
		if(g_adc_data.channel_flags[i] & ADC_CHANNEL_ENABLE_MASK) {
			adc_enable_channel(ADC, i);
		}
	}		

	/* Transfer with/without PDC. */
	if (g_adc_data.adc_config & ADC_PDC_ENABLE_MASK) {
		reload_adc_read_buffer(ADC, g_adc_data.data, g_adc_data.num_channel_enabled);
		/* Enable PDC channel interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_RXBUFF);
	} else {
		/* Enable Data ready interrupt. */
		adc_enable_interrupt(ADC, ADC_IER_DRDY);
	}
	xSemaphoreGive(g_adc_data.mutex);
	
	// It is VERY IMPORTANT to set the priority of the interrupt to conform to what FreeRTOS needs because we are calling FreeRTOS interrupt-safe APIs (those *FromISR) from within interrupt handlers.
	// If we don't do this, if would work at the beginning, and then eventually the whole thing will come crashing down.
	// And it's not gonna crash even after millions of interrupts are handled. It will come crashing down in some very weird place after you start using another interrupt handler, like the pio handlers, and only after some handlling...
	// And, it will appear random. You press the button a couple of times, it dies. Then, at other times, you press and hold the button, etc. etc. Each time will be different.
	// You would be suspecting your stack overflowed, your code does a wild pointer, etc. etc. It's very difficult to debug. Trust me, you don't wanna go there.
	NVIC_ClearPendingIRQ(ADC_IRQn);
	NVIC_SetPriority(ADC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);
	
	// Start the hardware timing Trigger.
	configure_time_trigger_for_adc(adc_trigger_hz);

}


#ifdef __cplusplus
}
#endif