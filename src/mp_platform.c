/**
 * @file	mp_platform.c
 * @brief  	Used to make the link between the micropython files and the ChibiOS project.
 * 
 * @written by  	Eliot Ferragni
 * @creation date	20.03.2020
 */

#include "mp_platform.h"

/////////////////////////////////////////PRIVATE FUNCTIONS/////////////////////////////////////////

static char *stack_top;
#if MICROPY_ENABLE_GC
static char heap[MICROPYTHON_HEAP_SIZE];
#endif

static THD_WORKING_AREA(waMicropythonThd,1024);
static THD_FUNCTION(MicropythonThd,arg) {
  	(void)arg;
  	chRegSetThreadName("Micropython Thd");


  	int stack_dummy;
	stack_top = (char*)&stack_dummy;

#if MICROPY_ENABLE_GC
	gc_init(heap, heap + sizeof(heap));
#endif
	mp_init();
#if MICROPY_ENABLE_COMPILER
	//compiles and eecutes the python script stored in flash
	micropython_parse_compile_execute_from_str(py_flash_code);

	// Main script is finished, so now go into REPL mode.
	// The REPL mode can change, or it can request a soft reset.

soft_reset:
	//Waits to be connected to the terminal
	while(!isUSBConfigured()){
		chThdSleepMilliseconds(500);
	}
	for (;;) {

	    if (pyexec_mode_kind == PYEXEC_MODE_RAW_REPL) {
	        if (pyexec_raw_repl() != 0) {
	            break;
	        }
	    } else {
	        if (pyexec_friendly_repl() != 0) {
	            break;
	        }
	    }
	}
	chprintf((BaseSequentialStream *)&MICROPYTHON_PORT,"MPY: soft reboot\r\n");
	goto soft_reset;
#else
	pyexec_frozen_module("frozentest.py");
#endif
	mp_deinit();
}

/////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void gc_collect(void) {
    // WARNING: This gc_collect implementation doesn't try to get root
    // pointers from CPU registers, and thus may function incorrectly.
    void *dummy;
    gc_collect_start();
    gc_collect_root(&dummy, ((mp_uint_t)stack_top - (mp_uint_t)&dummy) / sizeof(mp_uint_t));
    gc_collect_end();
    gc_dump_info();
}

// Receive single character
int mp_hal_stdin_rx_chr(void) {
    static uint8_t c[1] = {0};

	chnRead((BaseChannel*)&MICROPYTHON_PORT, c, 1);

	return (unsigned char)c[0];

}

// Send string of given length
void mp_hal_stdout_tx_strn(const char *str, mp_uint_t len) {

	if(len > 0){
		chnWrite((BaseChannel*)&MICROPYTHON_PORT, (uint8_t*)str, len);
	}
}

void micropythonStart(void){
	chThdCreateStatic(waMicropythonThd, sizeof(waMicropythonThd), NORMALPRIO, MicropythonThd, NULL);
}
