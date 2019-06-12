/**
 * @file	encoders.h
 * @brief  	File containing the driver to communicate by SPI with a AS5055 encoder
 * 
 * @written by  	Eliot Ferragni
 * @creation date	07.06.2019
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#include "as5055.h"

void encodersStartReading(void);

AS5055_data_t* encodersGetData(void);


#endif /* ENCODERS_H */