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


#endif /* CUSTOM_IO_H_ */
