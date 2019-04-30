/**
 * @file	custom_io.h
 * @brief  	Contains customs IO definitions for debugging purposes
 *
 * @written by  	Mohammed-Ismail Ben Salah
 * @creation date	02.04.2019
 */


#ifndef CUSTOM_IO_H_
#define CUSTOM_IO_H_


#include <ch.h>
#include <hal.h>

/*===========================================================================*/
/* IO Defines			                                                     */
/*===========================================================================*/

#define LD1_LINE					PAL_LINE(GPIOB,0U)
#define LD2_LINE					PAL_LINE(GPIOB,7U)
#define LD3_LINE					PAL_LINE(GPIOB,14U)
#define DEBUG_INT_LINE				PAL_LINE(GPIOB,2U)
#define DEBUG_INT_LINE2             PAL_LINE(GPIOB,6U)
#define DEBUG_INT_LINE3             PAL_LINE(GPIOG,14U)
#define DEBUG_INT_LINE4             PAL_LINE(GPIOF,11U)

#endif /* CUSTOM_IO_H_ */
