/**
 * @file	platform.c
 * @brief  	Used to make the link between the blackmagic files and the ChibiOS project.
 * 
 * @written by  		Eliot Ferragni
 * @creation date		18.06.2018
 * @last modification	27.02.2019
 */

#include <stdio.h>
#include <string.h>

#include "platform.h"
#include "gdb_packet.h"

//////////////////////////////////////////PLATFORM FUNCTIONS/////////////////////////////////////////

void platform_srst_set_val(bool assert)
{
	if (assert) {
		gpio_clear(SRST_PORT, SRST_PIN);
		chThdSleepMilliseconds(1);
	} else {
		gpio_set(SRST_PORT, SRST_PIN);
	}
}

bool platform_srst_get_val(void)
{
	return gpio_get(SRST_PORT, SRST_PIN) == 0;
}

const char *platform_target_voltage(void)
{
	return "ABSENT!";
}

#ifdef USE_SECOND_GDB_INTERFACE

bool platform_is_second_gdb_interface_active(void){

	//to be replaced by your own implementation or to be let like this if not used
	return false;
}

bool platform_is_second_gdb_interface_connected(void){

	//to be replaced by your own implementation or to be let like this if not used
	return false;
}

#endif /* USE_SECOND_GDB_INTERFACE */

#ifdef POWER_ON_WHEN_SWDP_SCAN

void platform_turn_on_target_on_swdp_scan(void){
	//Add your own implementation or let it like this if not used
}

#endif /* POWER_ON_WHEN_SWDP_SCAN */
