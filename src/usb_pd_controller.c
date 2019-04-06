/**
 * @file	usb_pd_controller.c
 * @brief  	File containing the high levels functions to use the PD Buddy library
 * 
 * @written by  	Eliot Ferragni
 * @creation date	08.04.2019
 */
#include <ch.h>
#include <hal.h>

#include <pdb.h>
#include <pd.h>
#include <device_policy_manager.h>

#include <usb_pd_controller.h>

/*
 * I2C configuration object.
 * I2C2_TIMINGR: 1000 kHz with I2CCLK = 48 MHz, rise time = 100 ns,
 *               fall time = 10 ns (0x00700818)
 */
static const I2CConfig i2c2config = {
    0x0080091C,
    0,
    0
};

/*
 * PD profile config
 */
static struct pdbs_config pd_config = {
    .flags    = 0,
    .v        = 9000,
    .i        = 200,
    .vmin     = 12000,
    .vmax     = 19000,
    .status   = PDBS_CONFIG_STATUS_VALID,
};

/*
 * PD Buddy Sink DPM data
 */
static struct pdbs_dpm_data dpm_data = {
    NULL,
    fusb_tcc_none,
    true,
    true,
    true,
    ._present_voltage = 5000
};

/*
 * PD Buddy firmware library configuration object
 */
static struct pdb_config pdb_config = {
    .fusb = {
        &I2CD2,
        FUSB302B_ADDR,
        LINE_INT_PD_CTRL_n
    },
    .dpm = {
    	pdbs_dpm_init,
        pdbs_dpm_evaluate_capability,
        pdbs_dpm_get_sink_capability,
        pdbs_dpm_giveback_enabled,
        pdbs_dpm_evaluate_typec_current,
        pdbs_dpm_check_vbus,
        pdbs_dpm_wait_vbus,
        pdbs_dpm_pd_start,
        pdbs_dpm_transition_default,
        pdbs_dpm_transition_min,
        pdbs_dpm_transition_standby,
        pdbs_dpm_transition_requested,
        pdbs_dpm_transition_typec,
        NULL /* not_supported_received */
    },
    .dpm_data = &dpm_data,
    .pd_config = &pd_config,
    .vbus_line = LINE_PWR_PP_STATE,
};

void usbPDControllerInit(void){
	i2cStart(pdb_config.fusb.i2cp, &i2c2config);
	pdb_init(&pdb_config);
}