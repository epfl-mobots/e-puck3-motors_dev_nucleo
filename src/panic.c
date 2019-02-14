/**
 * @file	panic.c
 * @brief  	Contains the panic handler function
 * 
 * @written by  	Eliot Ferragni
 * @creation date	19.06.2018
 */

#include <ch.h>
#include <hal.h>

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

/**
 * @brief Panic handler called when a kernel panic is invoked
 * @param reason Information given by the OS
 */
void panic_handler(const char *reason)
{
    (void)reason;
  
    while (1) {

    }
}