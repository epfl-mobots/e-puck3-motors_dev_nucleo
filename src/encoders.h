/**
 * @file	encoders.h
 * @brief  	File containing the driver to communicate by SPI with a AS5055 encoder
 * 
 * @written by  	Eliot Ferragni
 * @creation date	07.06.2019
 */

#ifndef ENCODERS_H
#define ENCODERS_H

/**
 * @brief   Id of the two encoders present on the motor_dev board
 */
typedef enum {
	ENCODER_1 = 0,
	ENCODER_2,
	NB_OF_ENCODERS,
} encoders_id;

typedef struct {
	/**
	 * Alarm low bit. Tells if the magnetic field is too weak
	 */
	bool alarm_lo;
	/**
	 * 	Alarm High bit. Tells if the magnetic field is to strong
	 */
	bool alarm_hi;
	/**
	 * Angle in degree
	 */
	float angle;
}encoders_data_t;

/**
 * @brief 	Starts the thread reading the encoders.
 */
void encodersStartReading(void);

/**
 * @brief 	Gets the last data read form the encoders.
 * @return 	encoders_data_t data structure pointer.
 */
encoders_data_t* encodersGetData(void);


#endif /* ENCODERS_H */