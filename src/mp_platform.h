/**
 * @file	mp_platform.h
 * @brief  	Used to make the link between the micropython files and the ChibiOS project.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	20.03.2020
 */
 
#ifndef MP_PLATFORM_H
#define MP_PLATFORM_H

#include "main.h"

#include "py/compile.h"
#include "py/runtime.h"
#include "py/repl.h"
#include "py/gc.h"
#include "py/mperrno.h"
#include "lib/utils/pyexec.h"
#include "mpconfigport.h"
#include "mpport.h"
#include "py_flash.h" 

#define MICROPYTHON_HEAP_SIZE	120000
#define MICROPYTHON_PORT		USB_SERIAL


/**
 * @brief Starts the micropython thread
 */
void micropythonStart(void);

#endif /* MP_PLATFORM_H */