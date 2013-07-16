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
 * tm_stick.c
 *
 * Created: 5/31/2013 8:39:11 PM
 *  Author: Jonah Tsai
 */ 

#include "tm_stick.h"

#include <sysclk.h>
#include <ssc.h>
#include <tc.h>


#include <gpio.h>

#include "conf_hempstead.h"


#ifdef __cplusplus
extern "C" {
#endif

tm_stick_data_t g_tm_stick_data = {
	.mutex = NULL,
	.rtos_task_semaphore = NULL,
	.button_position[0] = 0,
	.button_position[1] = 1, // this pin is not connected.
	.button_position[2] = 2, // this pin is not connected.
	.button_position[3] = 3,
	.button_position[4] = 4,
	.button_position[5] = 5,
	.button_position[6] = 6,
	.button_position[7] = 7,
	.button_position[8] = 8,
	.button_position[9] = 9,
	.button_position[10] = 10,
	.button_position[11] = 11,
	.button_position[12] = 12,
	.button_position[13] = 13,
	.button_position[14] = 14,
	.button_position[15] = 15,
	.button_position[16] = 16,
	.button_position[17] = 17,
	.button_position[18] = 18,
	.button_position[19] = 19,
	.button_position[20] = 20,
	.button_position[21] = 21,
	.button_position[22] = 22,
	.button_position[23] = 23
};

static void configure_time_trigger_for_ssc(uint32_t ssc_trigger_hz)
{
	
#if 0
	gpio_configure_pin(PIO_PA12_IDX, PIO_PERIPH_B);
	pmc_enable_periph_clk(ID_PWM);

	/* Disable PWM channels */
	pwm_channel_disable(PWM, PWM_CHANNEL_1);

	/* Set PWM clock A as PWM_FREQUENCY*PERIOD_VALUE (clock B is not used) */
	pwm_clock_t clock_setting = {
		.ul_clka = 1000 * 100,
		.ul_clkb = 0,
		.ul_mck = sysclk_get_cpu_hz()
	};
	pwm_init(PWM, &clock_setting);
	
	pwm_channel_t g_pwm_channel;

	/* Period is left-aligned */
	g_pwm_channel.alignment = PWM_ALIGN_LEFT;
	/* Output waveform starts at a low level */
	g_pwm_channel.polarity = PWM_LOW;
	/* Use PWM clock A as source clock */
	g_pwm_channel.ul_prescaler = PWM_CMR_CPRE_CLKA;
	/* Period value of output waveform */
	g_pwm_channel.ul_period = 100;
	/* Duty cycle value of output waveform */
	g_pwm_channel.ul_duty = 50;
	g_pwm_channel.channel = PWM_CHANNEL_1;
	pwm_channel_init(PWM, &g_pwm_channel);
	
	/* Disable channel counter event interrupt */
	pwm_channel_disable_interrupt(PWM, PWM_CHANNEL_1, 0);
	
	pwm_channel_enable(PWM, PWM_CHANNEL_1);
#endif


	
	uint32_t ul_div = 0;
	uint32_t ul_tc_clks = 0;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	pmc_set_writeprotect(false);

	// Enable peripheral clock.
	pmc_enable_periph_clk(CONF_SSC_CLOCK_SOURCE_ID);

	// TIOA configuration 
	// gpio_configure_pin(PIN_TC0_TIOA0, PIN_TC0_TIOA0_FLAGS);
	// tc_set_writeprotect(TC2, true);
	tc_set_writeprotect(CONF_SSC_CLOCK_TC, false);

	// Configure TC for a 1Hz frequency and trigger on RC compare.
	tc_find_mck_divisor(ssc_trigger_hz, ul_sysclk, &ul_div, &ul_tc_clks, ul_sysclk);
	tc_init(CONF_SSC_CLOCK_TC, CONF_SSC_CLOCK_CHANNEL, ul_tc_clks | TC_CMR_WAVSEL_UP_RC | TC_CMR_WAVE | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET);
	uint32_t tmp_val = (ul_sysclk / ul_div) / ssc_trigger_hz;
	
	tc_write_ra(CONF_SSC_CLOCK_TC, CONF_SSC_CLOCK_CHANNEL, tmp_val / 2);
	tc_write_rc(CONF_SSC_CLOCK_TC, CONF_SSC_CLOCK_CHANNEL, tmp_val);

	// Start the Timer.
	tc_start(CONF_SSC_CLOCK_TC, CONF_SSC_CLOCK_CHANNEL);

}

void tm_stick_init(uint32_t bus_speed_hz) {
	if(g_tm_stick_data.mutex == NULL) {
		g_tm_stick_data.mutex =  xSemaphoreCreateMutex();
	}
	
	if(g_tm_stick_data.rtos_task_semaphore == NULL) {
		vSemaphoreCreateBinary(g_tm_stick_data.rtos_task_semaphore);
	}
	
	configure_time_trigger_for_ssc(1000);
	
	sysclk_enable_peripheral_clock(ID_SSC);
	ssc_reset(SSC);
	// Do not go over 1MHz, the pulse will be too long and extend over to the next clock's rising edge (The spec. sheet of 4021BCM does not really list the operation times of 3.3V operation.
	// I tested a couple of speeds.... at 1.25MHz, I found that it sometimes misses some pulses.
	if(bus_speed_hz > TM_STICK_MAX_BUS_SPEED){
		bus_speed_hz = TM_STICK_MAX_BUS_SPEED;
	}
	ssc_set_clock_divider(SSC, bus_speed_hz, sysclk_get_cpu_hz()); 
	clock_opt_t rx_clk_opt = {
		.ul_cks = SSC_RCMR_CKS_MCK,
		.ul_cko = SSC_RCMR_CKO_TRANSFER,
// TODO: Fix the SSC clock for some reason shift 1/2 clock pulse position.
// This makes the button results shift one position to the right. So we have to change rising/falling edge to "correct" it (oh, well, "hack" around it).
// Don't know why.
// One possible patch up is to use a pin to signal which one is desired to determine which of the following settings to use..
#if defined(CONF_BOARD_ARDUINO_DUE) 
		.ul_cki = 0, //SSC_RCMR_CKI,
#else
		.ul_cki = SSC_RCMR_CKI,
#endif
		.ul_ckg = SSC_RCMR_CKG_CONTINUOUS,
		.ul_start_sel = SSC_RCMR_START_RF_FALLING,
		.ul_period = 0,
		.ul_sttdly = 0
	};
	
	data_frame_opt_t rx_data_frame_opt = {
		.ul_datlen = 23,
		.ul_msbf = 0,  //SSC_RFMR_MSBF,
		.ul_fsos = SSC_RFMR_FSOS_NONE,
		.ul_datnb = 0,
		.ul_fsedge = SSC_RFMR_FSEDGE_NEGATIVE,
		.ul_fslen = 0,
		.ul_fslen_ext = 0
	};
	
	ssc_set_receiver(SSC, &rx_clk_opt, &rx_data_frame_opt);
	ssc_enable_interrupt(SSC, SSC_IER_RXRDY);
	
	// It is VERY IMPORTANT to set the priority of the interrupt to conform to what FreeRTOS needs because we are calling FreeRTOS interrupt-safe APIs (those *FromISR) from within interrupt handlers.
	// If we don't do this, if would work at the beginning, and then eventually the whole thing will come crashing down.
	// And it's not gonna crash even after millions of interrupts are handled. It will come crashing down in some very weird place after you start using another interrupt handler, like the pio handlers, and only after some handlling...
	// And, it will appear random. You press the button a couple of times, it dies. Then, at other times, you press and hold the button, etc. etc. Each time will be different.
	// You would be suspecting your stack overflowed, your code does a wild pointer, etc. etc. It's very difficult to debug. Trust me, you don't wanna go there.
	NVIC_ClearPendingIRQ(SSC_IRQn);
	NVIC_SetPriority(SSC_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
	
	NVIC_EnableIRQ(SSC_IRQn); 
	
	ssc_enable_rx(SSC);
}

void SSC_Handler( void ) {	
	portBASE_TYPE xHigherTaskWoken = pdFALSE;
	uint32_t* in_data_buf = (uint32_t*)g_tm_stick_data.data;
	uint32_t in_data = 0;
	static uint32_t previous_in_data = 0;
	static bool is_first_time = true;
	// calculate the mask needed to wipe off extra high bit junk.
	static uint32_t mask = 0xFFFFFFFF;
	if(is_first_time) {
		for(int i = 4; i > TM_STICK_NUM_DATA_BYTES; i--) {
			mask = mask >> 8;
		}
	}

	if(ssc_is_rx_ready(SSC) == SSC_RC_YES) {
		xSemaphoreTakeFromISR(g_tm_stick_data.mutex, &xHigherTaskWoken);
		in_data = SSC->SSC_RHR;
		// in_data >>= 1; // No idea why it always reads 25bits (one too many bit at the end LSB), instead of 24 I told it to. Therefore, we shift it off.
		in_data = ~in_data; // reverse it. So, now 1 is on, 0 is off.
		in_data &= mask;
		
		// Glitch filtering below.
		if(is_first_time
			||  (previous_in_data ^ in_data) == 0) {
			is_first_time = false;
			*in_data_buf = in_data;
		}
		previous_in_data = in_data;
		xSemaphoreGiveFromISR(g_tm_stick_data.mutex, &xHigherTaskWoken);
	
		xHigherTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(g_tm_stick_data.rtos_task_semaphore, &xHigherTaskWoken); // if there is an RTOS waiting for the ADC task mutex, wake it up.
		portEND_SWITCHING_ISR(xHigherTaskWoken);
	}
}

#ifdef __cplusplus
}
#endif