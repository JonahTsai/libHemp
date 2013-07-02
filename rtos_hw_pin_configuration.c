/*
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