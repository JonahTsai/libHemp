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
 * tm_stick.h
 *
 * Read buttons from ThrustMaster's Cougar and Warthog stick.
 *
 * Cougar/Warthog sticks each has 3x CD4021BCM 8 stage parallel-in serial-out shift register chips in it.
 * We have to read the stick accordingly.
 * We are using the Synchronous Serial Communication component to read it.
 * It basically uses the 3 SSC receiver pins, RF, RD, and RK, programmed to send 24x pulses with the receiver start programmed to be triggered by some external signal on RF pin.
 * However, normally, SSC's RF trigger come from the sender. Now, TM stick does not do that, it's just three passive 8bit registers sitting there, so the MCU has to drive it.
 * So, we cheat. We hardwire TIOA0 pin to the RF pin.
 * In the Hempstead controller, the TIOA0 clock is also used to H/W trigger ADC conversions. If you are also using ADC, then you don't have to configure the TIOA0 clock at all, you just hardwire TIOA0 pin to the RF pin and you are good.
 * But, if for some weird reason, you are not using the Hempstead ADC library or does not start the ADC, then you will have to configure the TIOA0 to generate about 2KHZ clock.
 *
 * The TIOA0 H/W clock will then trigger the start of the SSC transfer frame on the RF pin (on falliing edge). This is wired up to the Parallel/Serial Control pin on the TM stick shift registers.
 * So, once the RF goes negative, it locks down the shift registers and turn them into serial out mode.
 * At the same time, the falling edge on RF also triggers SSC clock to output 24 clock pulses to shift out the locked shift register values.
 * 
 * We also use SSC to trigger interrupt on Receiver Data Ready. The SSC interrupt handler will lock down the data mutext, and copy the 24 bit data into g_tm_stick_data struct.
 * Then once the copying is done, it unlock the data mutex, and the notifies the rtos_task_mutex to get the waiting RTOS task going, if there is a task waiting on the task mutex.
 *
 * The order of the shift out of data on the wire is MSB first.
 * The order of the buttons shifted out is as follows (for the button names, please refer to Cougar Joystick diagram).
 * #0 == Trigger Stage 1
 * #1 == N/C
 * #2 == N/C
 * #3 == WPN/REL
 * #4 == Pinky Peddle Switch
 * #5 == Pinky Nose Wheel Steering Button
 * #6 == NWS / MSL STEP
 * #7 == Trigger Stage 2
 * #8 == Trim: Nose Down
 * #9 == Trim: RWD
 * #10 == Trim: Nose Up
 * #11 == Trim: LWD
 * #12 == TMS: Up
 * #13 == TMS: Right
 * #14 == TMS: Down
 * #15 == TMS: Left
 * #16 == DMS (castle hat): Up
 * #17 == DMS: Right
 * #18 == DMS: Down
 * #19 == DMS: Left
 * #20 == CMS: Up
 * #21 == CMS: Right
 * #22 == CMS: Down
 * #23 == CMS: Left
 *
 *
 * We then store the bits in the LSB order so that #0 is stored in bytes[0] bit 0, exactly in the index as it is described above.
 * Then we reverse the bits so now 0 becoems 1, 1 becomes 0, in preparation for sending it up stream via USB, also to conform to the programming convention of 1 is on, 0 is off, instead of the EE "convenience" of negative == true crap.
 *
 * ///////// NOTE //////////
 * On wire, 0 is button pressed. But in the data, we REVERSE them, so 1 is pressed, 0 is NOT pressed!!!
 * ///////// NOTE //////////
 *
 * Also, since this is read from a jam type shift register, we cannot use the nice glitch or debounce mechanism buit-in by the nice Atmel folx.
 * So, we have to a bit of debouncing ourselves. Since we are running off a 2KHz TIOA0 clock, we can simply read it twice. Only if all bits are read exactly the same "twice" do we store them in the value storage.
 *
 * Requires: ASF SSC, Timer Counter, and FreeRTOS modules.
 *           TIOA0 needs to generate about 2KHz clock pulses, and hardwire TIOA0 pin to RF pin.
 *
 * Created: 5/31/2013 8:37:33 PM
 *  Author: Jonah Tsai
 *
 */ 


#ifndef TM_STICK_H_
#define TM_STICK_H_

#include <stdint.h>
#include <FreeRTOS.h>
#include <semphr.h>

#define TM_STICK_MAX_BUS_SPEED 1000000
#define TM_STICK_NUM_DATA_BYTES 3

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint8_t data[4]; // alway 4 bytes to match the SSC RHR 32bit register size
	uint32_t button_position[24];
	xSemaphoreHandle mutex; // mutex for protecting this data structure
	xSemaphoreHandle rtos_task_semaphore; // mutex for FreeRTOS processing task synchronization.
} tm_stick_data_t;

extern tm_stick_data_t g_tm_stick_data;

void tm_stick_init(uint32_t bus_speed_hz);


#ifdef __cplusplus
}
#endif

#endif /* TM_STICK_H_ */