/**
 * @file	platform.h
 * @brief  	Used to make the link between the blackmagic files and the ChibiOS project.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	15.02.2019
 */

#ifndef __PLATFORM_H
#define __PLATFORM_H

#include <ch.h>
#include <hal.h>

#include "gdb.h"

/**
 * Blackmagic wrappers
 */


#include "gpio.h"
#include "timing.h"
#include "timing_stm32.h"
#include "debug.h"

#define USB_NAME "Blackmagic probe"

#define NOT_USED	0	//Only for pin. The ports should always be defined, even if it's not used
#define JTAG_PORT 	GPIOA
#define TDI_PORT	JTAG_PORT
#define TMS_PORT	JTAG_PORT
#define TCK_PORT	JTAG_PORT
#define TDO_PORT	JTAG_PORT
#define TDI_PIN		JTAG_PORT
#define TMS_PIN		NOT_USED
#define TCK_PIN		NOT_USED
#define TDO_PIN		NOT_USED

#define SWDIO_PORT 	GPIOE
#define SWCLK_PORT 	GPIOE
#define SWDIO_PIN	GPIOE_SWCLK_OUT
#define SWCLK_PIN	GPIOE_SWDIO_OUT

#define TRST_PORT	JTAG_PORT
#define TRST_PIN	NOT_USED
#define SRST_PORT	GPIOB
#define SRST_PIN	NOT_USED

#define LED_PORT_ERROR		GPIOA //RED_LED
#define LED_PORT_UART		GPIOA //BLUE_LED
#define LED_PORT			GPIOA //GREEN_LED


#define LED_ERROR		NOT_USED	
#define LED_UART		NOT_USED	
#define LED_IDLE_RUN	NOT_USED	
#define LED_BOOTLOADER	NOT_USED

#define TMS_SET_MODE() {palSetPadMode(TMS_PORT, SWDIO_PIN, PAL_MODE_OUTPUT_PUSHPULL);}

#define SWDIO_MODE_FLOAT() {palSetPadMode(SWDIO_PORT, SWDIO_PIN, PAL_MODE_INPUT);}

#define SWDIO_MODE_DRIVE() {palSetPadMode(SWDIO_PORT, SWDIO_PIN, PAL_MODE_OUTPUT_PUSHPULL);}

#define SET_RUN_STATE(state)	{gdbSetFlag(state ? RUNNING_FLAG : IDLE_FLAG);};
#define SET_PROGRAMMING_STATE()	{gdbSetFlag(PROGRAMMING_FLAG);};
#define SET_IDLE_STATE(state)	{};
#define SET_ERROR_STATE(state)	{gdbSetFlag(ERROR_FLAG);};

static inline int platform_hwversion(void)
{
	return 0;
}

/* Use newlib provided integer only stdio functions */
#define sscanf siscanf
#define sprintf siprintf
#define vasprintf vasiprintf
#define snprintf sniprintf

#endif /* __PLATFORM_H */