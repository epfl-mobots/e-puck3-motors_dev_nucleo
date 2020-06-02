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
#include <stdlib.h>
#include <string.h>

#include "chprintf.h"
#include "usbcfg.h"
#include "main.h"
#include "user_shell.h"
#include "gate_drivers.h"
#include "usb_pd_controller.h"
#include "motors.h"
#include "threads_utilities.h"


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
		// chprintf(chp, "%d %d %d" SHELL_NEWLINE_STR, motNumber, phaseNumber, onOff);
		// palWriteLine(motor_pins[motNumber-1][phaseNumber-1][0], onOff ? 1 : 0);
		// palWriteLine(motor_pins[motNumber-1][phaseNumber-1][1], onOff ? 0 : 1);
		// chprintf(chp, "motor %d phase %d P : %s" SHELL_NEWLINE_STR, motNumber, phaseNumber, palReadLine(motor_pins[motNumber-1][phaseNumber-1][0]) ? "ON" : "OFF");
		// chprintf(chp, "motor %d phase %d N : %s" SHELL_NEWLINE_STR, motNumber, phaseNumber, palReadLine(motor_pins[motNumber-1][phaseNumber-1][1]) ? "ON" : "OFF");
	}
}

static void cmd_power_drivers(BaseSequentialStream *chp, int argc, char *argv[]) {

	(void)argv;	
	if(argc == 2){
		uint8_t motNumber = (char)*argv[0]-'0';
		uint8_t onOff = (char)*argv[1]-'0';
		//ioline_t pin = 0;
		gateDriver_id device_id;
		if(motNumber == 1){
			//pin = LINE_EN_DRIVER_1;
			device_id = 0;
		}else if(motNumber == 2){
			//pin = LINE_EN_DRIVER_2;
			device_id = 1;
		}else if(motNumber == 3){
			//pin = LINE_EN_DRIVER_3;
			device_id = 2;
		}else if(motNumber == 4){
			//pin = LINE_EN_DRIVER_4;
			device_id = 3;
		}else{
			return;
		}

		// palWriteLine(pin,onOff);
		if(onOff){
			gateDriversEnable(device_id);
		}else{
			gateDriversDisable(device_id);
		}
	}
	if(argc == 0 || argc == 2){
		//gives the actual state
		chprintf(chp, "power state driver 1 : %s" SHELL_NEWLINE_STR, palReadLine(LINE_EN_DRIVER_1) ? "ON" : "OFF");
		chprintf(chp, "power state driver 2 : %s" SHELL_NEWLINE_STR, palReadLine(LINE_EN_DRIVER_2) ? "ON" : "OFF");
		chprintf(chp, "power state driver 3 : %s" SHELL_NEWLINE_STR, palReadLine(LINE_EN_DRIVER_3) ? "ON" : "OFF");
		chprintf(chp, "power state driver 4 : %s" SHELL_NEWLINE_STR, palReadLine(LINE_EN_DRIVER_4) ? "ON" : "OFF");
	}else{
		shellUsage(chp, "power_drivers motNumber 1|0");
	}
}


static void cmd_step_time(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  if(argc == 1)
  {
    char *endptr;
    uint32_t lMaxStepCount = strtol(argv[0], &endptr, 0);
    (void)lMaxStepCount;
    //motor1.kMaxStepCount = lMaxStepCount;
  }
  if(argc == 2)
  {
      //gives the actual step number timer and the step status
      //chprintf(chp, "Iteration per step : %d" SHELL_NEWLINE_STR, motor1.kMaxStepCount);
  }
  else
  { 
    chprintf(chp, "Not Implemented" SHELL_NEWLINE_STR);
    //shellUsage(chp, "set_step_time 0-65535");
  }
}

static void cmd_motor_timing(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  if(argc == 1)
  {
    char *endptr;
    float timing = strtol(argv[0], &endptr, 0);
    timing -= 30;
    timing *= -1;
    timing /= 60;
    //motor1.ZCTiming = timing;


  }
  else
  {
      chprintf(chp, "Not Implemented" SHELL_NEWLINE_STR);
      //shellUsage(chp, "set_motor_timing -30 to 30 degrees");
  }
}

static void cmd_motor_set_speed(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  if(argc == 2)
  {
    char *endptr;
    uint8_t motNumber = strtol(argv[0], &endptr, 0);
    float speed = strtol(argv[1], &endptr, 0);

    brushless_motors_names_t motor_id;
    if(motNumber == 1){
      //pin = LINE_EN_DRIVER_1;
      motor_id = 0;
    }else if(motNumber == 2){
      //pin = LINE_EN_DRIVER_2;
      motor_id = 1;
    }else if(motNumber == 3){
      //pin = LINE_EN_DRIVER_3;
      motor_id = 2;
    }else if(motNumber == 4){
      //pin = LINE_EN_DRIVER_4;
      motor_id = 3;
    }else{
      return;
    }
    motorSetDutyCycle(motor_id, speed);

  }
  else
  {
      chprintf(chp, "motor_set_speed motNumber speed (0-100)" SHELL_NEWLINE_STR);
  }
}

static void cmd_sample_time(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  if(argc == 1)
  {
  	char *endptr;
    uint32_t width = strtol(argv[0], &endptr, 0);
    if (width > 100){
      width = 100;
    }
    //PWMD1.tim->CCXR[1] = (width * PERIOD_PWM_52_KHZ / 100) - 1;
  }
  else
  { 
    chprintf(chp, "Not Implemented" SHELL_NEWLINE_STR);
    //chprintf(chp, "Sampling time: %d%" SHELL_NEWLINE_STR, (PWMD1.tim->CCXR[1]+1)*100/PERIOD_PWM_52_KHZ);
  }
}

static void cmd_dir(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argv;
  if(argc == 1)
  {
    char *endptr;
    uint32_t lDirection = strtol(argv[0], &endptr, 0);

    (void)lDirection;
    // if(kCW == lDirection)
    // {
    //     motor1.RotationDir = kCW;
    //     chprintf(chp, "Motor direction : Clockwise" SHELL_NEWLINE_STR);
    // }
    // else
    // {
    //    motor1.RotationDir = kCCW;
    //    chprintf(chp, "Motor direction : CounterClockwise" SHELL_NEWLINE_STR);
    // }
  }
  else
  {
      chprintf(chp, "Not Implemented" SHELL_NEWLINE_STR);
      //shellUsage(chp, "set_direction 0=Clockwise|1=CounterCW");
  }
}


static void cmd_get_source_cap(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    if (argc > 0) {
    	shellUsage(chp, "get_source_cap");
        return;
    }

    usbPDControllerPrintSrcPDO(chp);
}

static void cmd_get_cfg(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    if (argc > 0) {
        shellUsage(chp, "get_cfg");
        return;
    }

    usbPDControllerPrintConfig(chp);
}

static void cmd_set_v(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    bool err = false;
    if (argc != 1) {
        shellUsage(chp, "set_v voltage_in_mV");
        return;
    }

    char *endptr;
    uint16_t voltage = strtol(argv[0], &endptr, 0);

    if(endptr <= argv[0]){
    	err = true;
    }else{
    	err = !usbPDControllerSetFixedVoltage(voltage);
    }

    if(err){
    	chprintf(chp, "Invalid voltage\r\n");
    } 
}

static void cmd_set_vrange(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    bool err = false;
    if (argc != 2) {
        shellUsage(chp, "set_v min_voltage_in_mV max_voltage_in_mV");
        return;
    }

    char *endptr_min;
    char *endptr_max;
    uint16_t vmin = strtol(argv[0], &endptr_min, 0);
    uint16_t vmax = strtol(argv[1], &endptr_max, 0);

    if(endptr_min <= argv[0] || endptr_max <= argv[1]){
    	err = true;
    }else{
    	err = !usbPDControllerSetRangeVoltage(vmin, vmax);
    }

    if(err){
    	chprintf(chp, "Invalid voltages\r\n");
    }
}

static void cmd_set_i(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    bool err = false;
    if (argc != 1) {
        shellUsage(chp, "set_v current_in_mA");
        return;
    }

    char *endptr;
    uint16_t current = strtol(argv[0], &endptr, 0);

    if(endptr <= argv[0]){
    	err = true;
    }else{
    	err = !usbPDControllerSetFixedCurrent(current);
    }

    if(err){
    	chprintf(chp, "Invalid current\r\n");
    }
    
}

static void cmd_hv_prefered(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    if (argc == 0) {
    	chprintf(chp, "Actual setting for HV prefered : %s\r\n",usbPDControllerGetHVPrefered() ? "enabled" : "disabled");
    	return;
    }else if(argc == 1){
    	uint8_t enable = (char)*argv[0]-'0';
    	usbPDControllerSetHVPrefered(enable);
    }else{
    	shellUsage(chp, "hv_prefered 1|0 or without arg to show the actual setting");
        return;
    }
}

static void cmd_get_contract(BaseSequentialStream *chp, int argc, char *argv[])
{
    (void) argv;
    if (argc > 0) {
    	shellUsage(chp, "get_contract");
        return;
    }

    chprintf(chp, "Do we have a contract ? : %s \r\n", usbPDControllerIsContract() ? "yes" : "no");
    uint16_t voltage = usbPDControllerGetNegociatedVoltage();
    chprintf(chp, "Actual voltage : %d.%03d V\r\n", voltage/1000, voltage%1000);
}

static const ShellCommand commands[] = {
  THREADS_UTILITIES_SHELL_CMD
	{"set_phase", cmd_set_phase},
	{"power_drivers", cmd_power_drivers},
	{"set_step_time", cmd_step_time},
	{"set_direction",cmd_dir},
  {"motor_set_speed",cmd_motor_set_speed},
	{"set_sample_time",cmd_sample_time},
	{"set_motor_timing",cmd_motor_timing},
	{"get_source_cap", cmd_get_source_cap},
	{"get_cfg", cmd_get_cfg},
	{"set_v", cmd_set_v},
	{"set_vrange", cmd_set_vrange},
	{"set_i", cmd_set_i},
	{"hv_prefered", cmd_hv_prefered},
	{"get_contract", cmd_get_contract},
	{NULL, NULL}
};

static const ShellConfig shell_cfg = {
	(BaseSequentialStream *)&USB_GDB,
	commands,
	sc_histbuf,
	sizeof(sc_histbuf),
	completion_buffer,
};

void spawn_shell(void){
	static thread_t *shellTh = NULL;
	if(shellTh == NULL){
		shellTh = chThdCreateStatic(waShell, sizeof(waShell), NORMALPRIO + 1, shellThread, (void *)&shell_cfg);
		shellTh->name = "Shell";
	}	
}