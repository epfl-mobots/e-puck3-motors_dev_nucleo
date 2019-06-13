/**
 * @file	drv8323.h
 * @brief  	File containing the driver to communicate by SPI with a DRV8323 gate driver
 * 
 * @written by  	Eliot Ferragni
 * @creation date	14.03.2019
 */
 

#ifndef AS5055_H
#define AS5055_H

/**
 * @brief   Structure representing a set of AS5055 encoders in a daisy chain config.
 */
typedef struct {
	/**
	 * SPI driver used
	 */
	SPIDriver*				spip;
	/**
	 * SPI config used. 
	 * Note 1 :	The spicfg should not be declared as static const.
	 * 			Use static instead.
	 * Note 2 :	The SPI com should be configured to use 16bits words and CPHA=1
	 */
	SPIConfig* 				spicfg;
	/**
	 * Chip select line of the AS5055
	 */
	ioline_t				ssline;
	/**
	 * Interrupt line of the AS5055
	 */
	ioline_t				interruptline;
	/**
	 * Pointer to the data structure of the encoders. Must be the same
	 * size as the nb_of_as5055 parameter
	 */
	encoders_data_t* 			data;
	/**
	 * Number of AS5055 in daisy chain
	 */
	uint8_t					nb_of_as5055;

} AS5550Config;


/**
 * @brief 		Performs a master reset of the encoders.
 * 
 * @param as 	AS5550Config of the encoders
 */
void as5055DoAMasterReset(AS5550Config *as);

/**
 * @brief  		Reads the angle from the encoders. Needs to be called only when
 * 				the interruot line of the encoders is low.
 * 
 * @param as 	AS5550Config of the encoders
 */
void as5055ReadAngle(AS5550Config *as);

/********************          AS5055 REGISTER EXPLANATION         ********************/

/**
 * 	
 * 
 * 	SPI Command Frame : 
 *	bit  [15]		-> 	Write or read command
 *  bits [14..1]	->	Register address
 *	bits [0]		->	Parity bit
 *	
 *	SPI Read Data Frame :
 *	bit  [15..2]	-> 	Data to read
 *  bits [1]		->	EF bit (an error occured)
 *	bits [0]		->	Parity bit
 *	
 *	SPI Write Data Frame :
 *	bit  [15..2]	-> 	Data to write
 *  bits [1]		->	Don't care
 *	bits [0]		->	Parity bit
 *	
 *	To write to a register, we need to send a Command Frame and on the next frame should be
 *	a Write Data Frame. During the sending of the Write Data Frame, we receive the actual content 
 *	of the register we are writing to as a Read Data Frame.
 *	
 *	To read a register, we need to send a Command Frame and we will receive a Read Data Frame on 
 *	the next frame sent.
 *
 */

/********************               AS5055 WRITE/READ              ********************/

#define AS5055_WR_Pos		(15U)
#define AS5055_WRITE		(0x0U << AS5055_WR_Pos)
#define AS5055_READ			(0x1U << AS5055_WR_Pos)

/********************               AS5055 REGISTERS               ********************/

#define AS5055_REG_Pos					(1U)
#define AS5055_REG_Msk					(0x3FFFU << AS5055_REG_Pos)
#define AS5055_POR_OFF_REG				(0x3F22U << AS5055_REG_Pos)
#define AS5055_SOFTWARE_RESET_REG		(0x3C00U << AS5055_REG_Pos)
#define AS5055_MASTER_RESET_REG			(0x33A5U << AS5055_REG_Pos)
#define AS5055_CLEAR_EF_REG				(0x3380U << AS5055_REG_Pos)
#define AS5055_NOP_REG					(0x0000U << AS5055_REG_Pos)
#define AS5055_AGC_REG					(0x3FF8U << AS5055_REG_Pos)
#define AS5055_ANGULAR_DATA_REG			(0x3FFFU << AS5055_REG_Pos)
#define AS5055_ERROR_STATUS_REG			(0x335AU << AS5055_REG_Pos)
#define AS5055_SYSTEM_CONFIG_REG		(0x3F20U << AS5055_REG_Pos)

/********************      Bit definition for POR OFF register     ********************/

#define DIS_POR_Pos      	(2U)                                            
#define DIS_POR_Msk      	(0x5AU << DIS_POR_Pos)           /*!< 0x00000168 */
#define DIS_POR          	DIS_POR_Msk                 	 /*!< Deactivates the POR cell and reduces the current consumption in low power mode (IOFF). 	*/

/********************  Bit definition for Software Reset register  ********************/

#define RES_SPI_Pos      	(2U)                                            
#define RES_SPI_Msk      	(0x1U << RES_SPI_Pos) 			 /*!< 0x00000004 */
#define RES_SPI          	RES_SPI_Msk                      /*!< With the RES SPI bit of the Data Package set to 1 it is possible to reset the SPI registers 	*/

/********************        Bit definition for AGC register       ********************/

#define AGC_VAL_Pos      	(2U)                                            
#define AGC_VAL_Msk      	(0x3FU << AGC_VAL_Pos) 			 /*!< 0x000000FC */
#define AGC_VAL_MAX 		AGC_VAL_Msk                      /*!< Writing a value different than zero to this register, stops the AGC loop and keeps a constant AGC value. 	*/

/********************   Bit definition for Angular Data register   ********************/

#define ALARM_LO_Pos      	(15U)                                            
#define ALARM_LO_Msk      	(0x1U << ALARM_LO_Pos) 			 /*!< 0x00008000 */
#define ALARM_LO 	      	ALARM_LO_Msk                     /*!< Alarm flag, which indicates a too weak magnetic field.  	*/
#define ALARM_HI_Pos      	(14U)                                            
#define ALARM_HI_Msk      	(0x1U << ALARM_HI_Pos) 			 /*!< 0x00004000 */
#define ALARM_HI 	      	ALARM_HI_Msk                     /*!< Alarm flag, which indicates a too strong magnetic field.  	*/
#define ANGLE_VAL_Pos     	(2U)                                            
#define ANGLE_VAL_Msk     	(0xFFFU << ANGLE_VAL_Pos) 		 /*!< 0x00003FFC */
#define ANGLE_VAL 			ANGLE_VAL_Msk                    /*!< Angular value in 12 bit binary code.  	*/

/********************   Bit definition for Error Status register   ********************/

#define FIELD_ALARM_LO_Pos  (14U)                                            
#define FIELD_ALARM_LO_Msk  (0x1U << FIELD_ALARM_LO_Pos) 	 /*!< 0x00004000 */
#define FIELD_ALARM_LO 	    FIELD_ALARM_LO_Msk               /*!< AGC level is equal or even higher than the maximum level. Magnetic field is too weak.  	*/
#define FIELD_ALARM_HI_Pos  (13U)                                            
#define FIELD_ALARM_HI_Msk  (0x1U << FIELD_ALARM_HI_Pos) 	 /*!< 0x00002000 */
#define FIELD_ALARM_HI 	    FIELD_ALARM_HI_Msk               /*!< AGC level is equal or even lower than the minimum level. Magnetic field is too strong.  	*/
#define RANGE_Pos  			(12U)                                            
#define RANGE_Msk  			(0x1U << RANGE_Pos) 	 		 /*!< 0x00001000 */
#define RANGE 	    		RANGE_Msk               		 /*!< The RANGE flag signals that the Hall bias circuit has reached the head room limit.  	*/
#define CORDICOV_Pos  		(11U)                                            
#define CORDICOV_Msk  		(0x1U << CORDICOV_Pos) 	 		 /*!< 0x00000800 */
#define CORDICOV 	    	CORDICOV_Msk               		 /*!< The CORDIC calculates the angle. An error occurs when the input signals of the CORDIC are too large. The internal algorithm fails.  	*/
#define ADCOV_Pos  			(10U)                                            
#define ADCOV_Msk  			(0x1U << ADCOV_Pos) 	 		 /*!< 0x00000400 */
#define ADCOV 	    		ADCOV_Msk               		 /*!< The ADCOV bit occurs if the magnetic input field strength is too large for at least one Hall element.  	*/
#define WOW_Pos  			(6U)                                            
#define WOW_Msk  			(0x1U << WOW_Pos) 	 			 /*!< 0x00000040 */
#define WOW 	    		WOW_Msk               		 	 /*!< When a READ ANGLE command is in progress, the WOW flag is set to 1. At the end of the measurement the WOW flag is cleared to 0.  	*/
#define ADDMON_Pos  		(4U)                                            
#define ADDMON_Msk  		(0x1U << ADDMON_Pos) 	 		 /*!< 0x00000010 */
#define ADDMON 	    		ADDMON_Msk               		 /*!< Set to high when non existing address is used.  	*/
#define CLKMON_Pos  		(3U)                                            
#define CLKMON_Msk  		(0x1U << CLKMON_Pos) 	 		 /*!< 0x00000008 */
#define CLKMON 	    		CLKMON_Msk               		 /*!< Set to high when the amount of clock cycles is not correct.  	*/
#define PARITY_Pos  		(2U)                                            
#define PARITY_Msk  		(0x1U << PARITY_Pos) 	 		 /*!< 0x00000004 */
#define PARITY 	    		PARITY_Msk               		 /*!< Set to high when the transmitted parity bit does not match to calculated parity bit.  	*/


/********************Bit definition for System Configuration register********************/

#define RESOLUTION_Pos      (14U)                                            
#define RESOLUTION_Msk     	(0x3U << RESOLUTION_Pos) 	     /*!< 0x0000C000 */
#define RESOLUTION 	      	RESOLUTION_Msk                   /*!< 00 indicates 12 bit resolution.  	*/
#define CHIP_ID_Pos      	(11U)                                            
#define CHIP_ID_Msk     	(0x7U << CHIP_ID_Pos) 	     	 /*!< 0x00003800 */
#define CHIP_ID 	      	CHIP_ID_Msk                   	 /*!< Silicon version 010.  	*/
#define GAIN_Pos      		(5U)                                            
#define GAIN_Msk     		(0x3U << GAIN_Pos) 	     	 	 /*!< 0x00000060 */
#define GAIN 	      		GAIN_Msk                   	 	 /*!< Sets gain setting.  	*/


#endif /* AS5055_H */