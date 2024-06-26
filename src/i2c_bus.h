#ifndef I2C_BUS_H
#define I2C_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal.h>

/**
 * @brief Starts the I2C interface
 */
void i2c_start(I2CDriver* I2C);

/**
 * @brief Stops the I2C module	
 */
void i2c_stop(I2CDriver* I2C_USED);

/**
 * @brief 		Reads a register over I2C
 * 
 * @param addr	8bits address of the peripherical to read from
 * @param reg	8bits address of the register to read
 * @param value	Pointer used to store the value read
 * 
 * @return		The error code. msg_t format
 */
int8_t read_reg(I2CDriver* I2C_USED, uint8_t addr, uint8_t reg, uint8_t *value);

/**
 * @brief 		Writes a register over I2C
 * 
 * @param addr	8bits address of the peripherical to write to
 * @param reg	8bits address of the register to write
 * @param value	Value to write
 * 
 * @return		The error code. msg_t format
 */
int8_t write_reg(I2CDriver* I2C_USED, uint8_t addr, uint8_t reg, uint8_t value);

/**
 * @brief 		Reads a register bigger than 8bits over I2C
 * 
 * @param addr	8bits address of the peripherical to read from
 * @param reg	8bits address of the register to read
 * @param buf	Pointer to a buffer used to store the values read
 * @param len	Length of the requested read. the buf must be this size or greater
 * 
 * @return		The error code. msg_t format
 */
int8_t read_reg_multi(I2CDriver* I2C_USED, uint8_t addr, uint8_t reg, uint8_t *buf, int8_t len);

#ifdef __cplusplus
}
#endif

#endif
