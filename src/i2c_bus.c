#include <hal.h>
#include <ch.h>
#include "i2c_bus.h"

static systime_t timeout = TIME_MS2I(4); // 4 ms

void i2c_start(I2CDriver* I2C_USED) {

	if(I2C_USED == NULL){
		return;
	}

	if(I2C_USED->state != I2C_STOP) {
		return;
	}

    /*
     * I2C configuration structure
     */
    static const I2CConfig i2c_cfg1 = {
        .timingr = 0x6000030D
    };

    //simulate 16 clock pulses to unblock potential I2C periph blocked
    //take control of the pin
    palSetLineMode(LINE_I2C2_SCL , PAL_MODE_OUTPUT_OPENDRAIN );
    //16 clock pulses
    for(uint8_t i = 0 ; i < 32 ; i++){
    	palToggleLine(LINE_I2C2_SCL);
    	chThdSleepMilliseconds(1);
    }
    //make sure the output is high
    palSetLine(LINE_I2C2_SCL);
    //give the control of the pin to the I2C machine
    palSetLineMode(LINE_I2C2_SCL , PAL_MODE_ALTERNATE(4) | PAL_STM32_OTYPE_OPENDRAIN);
    i2cStart(I2C_USED, &i2c_cfg1);
}

void i2c_stop(I2CDriver* I2C_USED) {
	if(I2C_USED == NULL){
		return;
	}
	i2cStop(I2C_USED);
}

int8_t read_reg(I2CDriver* I2C_USED, uint8_t addr, uint8_t reg, uint8_t *value) {
	
	uint8_t txbuf[1] = {reg};
	uint8_t rxbuf[1] = {0};

	i2cAcquireBus(I2C_USED);
	if(I2C_USED->state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(I2C_USED, addr, txbuf, 1, rxbuf, 1, timeout);
		if (status != MSG_OK){
			if(I2C_USED->state == I2C_LOCKED){
				i2c_stop(I2C_USED);
				i2c_start(I2C_USED);
			}
			i2cReleaseBus(I2C_USED);
			return status;
		}
	}
	i2cReleaseBus(I2C_USED);

	*value = rxbuf[0];

    return MSG_OK;
}


int8_t write_reg(I2CDriver* I2C_USED, uint8_t addr, uint8_t reg, uint8_t value) {

	uint8_t txbuf[2] = {reg, value};
	uint8_t rxbuf[1] = {0};

	i2cAcquireBus(I2C_USED);
	if(I2C_USED->state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(I2C_USED, addr, txbuf, 2, rxbuf, 0, timeout);
		if (status != MSG_OK){
			if(I2C_USED->state == I2C_LOCKED){
				i2c_stop(I2C_USED);
				i2c_start(I2C_USED);
			}
			i2cReleaseBus(I2C_USED);
			return status;
		}
	}
	i2cReleaseBus(I2C_USED);

    return MSG_OK;
}

int8_t read_reg_multi(I2CDriver* I2C_USED, uint8_t addr, uint8_t reg, uint8_t *buf, int8_t len) {

	i2cAcquireBus(I2C_USED);
	if(I2C_USED->state != I2C_STOP) {
		msg_t status = i2cMasterTransmitTimeout(I2C_USED, addr, &reg, 1, buf, len, timeout);
		if (status != MSG_OK){
			if(I2C_USED->state == I2C_LOCKED){
				i2c_stop(I2C_USED);
				i2c_start(I2C_USED);
			}
			i2cReleaseBus(I2C_USED);
			return status;
		}
	}
	i2cReleaseBus(I2C_USED);

	return MSG_OK;
}
