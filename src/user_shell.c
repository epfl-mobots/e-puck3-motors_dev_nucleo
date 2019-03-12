/**
 * @file	user_shell.c
 * @brief  	File containing functions and definitions to handle the shell provided by ChibiOS
 * 
 * @written by  	Eliot Ferragni
 * @creation date	12.03.2019
 */

#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "usbcfg.h"
#include "main.h"
#include "user_shell.h"

static THD_WORKING_AREA(waShell,2048);

static char sc_histbuf[SHELL_MAX_HIST_BUFF];
static char* completion_buffer[SHELL_MAX_COMPLETIONS];

static void cmd_set_phase(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void)argv;
	if (argc != 3) {
		shellUsage(chp, "set_phase motNumber phaseNumber 1|0");
		return;
	}else{
		uint8_t motNumber = (char)*argv[0]-'0';
		uint8_t phaseNumber = (char)*argv[1]-'0';
		uint8_t onOff = (char)*argv[2]-'0';
		if(motNumber > NB_MOTORS){
			chprintf(chp, "motNumber should be between 1-4" SHELL_NEWLINE_STR);
			return;
		}
		if(phaseNumber > NB_PHASES){
			chprintf(chp, "phaseNumber should be between 1-3" SHELL_NEWLINE_STR);
			return;
		}
		if(onOff > 1){
			onOff = 1;
		}
		chprintf(chp, "%d %d %d" SHELL_NEWLINE_STR, motNumber, phaseNumber, onOff);
		palWriteLine(motor_pins[motNumber-1][phaseNumber-1][0], onOff ? 1 : 0);
		palWriteLine(motor_pins[motNumber-1][phaseNumber-1][1], onOff ? 0 : 1);
	}
}

static const ShellCommand commands[] = {
	{"set_phase", cmd_set_phase},
	{NULL, NULL}
};

static const ShellConfig shell_cfg = {
	(BaseSequentialStream *)&SDU1,
	commands,
	sc_histbuf,
	sizeof(sc_histbuf),
	completion_buffer,
};

void spawn_shell(void){
	static thread_t *shellTh = NULL;
	if(shellTh == NULL){
		shellTh = chThdCreateStatic(waShell, sizeof(waShell), NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
	}	
}