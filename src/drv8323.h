/**
 * @file	drv8323.h
 * @brief  	File containing the driver to communicate by SPI with a DRV8323 gate driver
 * 
 * @written by  	Eliot Ferragni
 * @creation date	14.03.2019
 */
 

#ifndef DRV8323_H
#define DRV8323_H


/**
 * @brief   Id of the four gate drivers present on the motor_dev board
 */
typedef enum {
	DRV8232_1 = 0,
	DRV8232_2,
	DRV8232_3,
	DRV8232_4,
	NB_OF_DRV8232,
} DRV8323_ID_t;


/**
 * @brief   Structure representing a DRV8323 Config.
 */
typedef struct {
	uint16_t 	fault_status_1;
	uint16_t	fault_status_2;
	uint16_t	driver_control;
	uint16_t	gate_drive_hs;
	uint16_t	gate_drive_ls;
	uint16_t	ocp_control;
	uint16_t	csa_control;
} DRV8323ConfRegisters;

/**
 * @brief   Structure representing a DRV8323 driver.
 */
typedef struct {
	/**
	 * SPI driver used
	 */
	SPIDriver*				spip;
	/**
	 * SPI config used. 
	 * Note 1 :	The ssline field of spicfg is managed by
	 * 			the drv8323 driver so you can let it uninitialized.
	 * Note 2 :	The spicfg should not be declared as static const.
	 * 			Use static instead.
	 * Note 3 :	The SPI com should be configured to use 16bits words and CPHA=1
	 */
	SPIConfig* 				spicfg;
	/**
	 * Enable line of the DRV8323
	 */
	ioline_t				enline;
	/**
	 * chip select line of the DRV8323
	 */
	ioline_t				ssline;
	/**
	 * Fault line of the DRV8323
	 */
	ioline_t				faultline;
	/**
	 * Values to write to the device to configure it
	 */
	DRV8323ConfRegisters	*registers;

} DRV8323Config;

void drv8323WriteConf(DRV8323Config *drv);
uint16_t drv8323WriteReg(DRV8323Config *drv, uint16_t reg);
uint16_t drv8323ReadReg(DRV8323Config *drv, uint16_t reg);

/********************          DRV8323 REGISTER EXPLANATION        ********************/

/**
 *	bit  [15]		-> 	Write or read command
 *  bits [14..11]	->	Register address
 *	bits [10..0]	->	Values of the register
 *	
 *	With a read command, only the register address is needed and we
 *	receive during the same transmission the content of the register from the device.
 *	
 *	The write command does exactly the same, except it also writes the data we send.
 *
 */

/********************               DRV8323 WRITE/READ             ********************/

#define DRV8323_WR_Pos		(15U)
#define DRV8322_WRITE		(0x0U << DRV8323_WR_Pos)
#define DRV8322_READ		(0x1U << DRV8323_WR_Pos)

/********************               DRV8323 REGISTERS              ********************/

#define DRV8323_REG_Pos				(11U)
#define DRV8323_REG_Msk				(0xFU << DRV8323_REG_Pos)
#define FAULT_STATUS_1_REG			(0x0U << DRV8323_REG_Pos)
#define FAULT_STATUS_2_REG			(0x1U << DRV8323_REG_Pos)
#define DRIVER_CONTROL_REG			(0x2U << DRV8323_REG_Pos)
#define GATE_DRIVE_HS_REG			(0x3U << DRV8323_REG_Pos)
#define GATE_DRIVE_LS_REG			(0x4U << DRV8323_REG_Pos)
#define OCP_CONTROL_REG				(0x5U << DRV8323_REG_Pos)
#define CSA_CONTROL_REG				(0x6U << DRV8323_REG_Pos)

/********************              Configuration bits              ********************/

#define DRV8323_CONF_Pos			(0U)
#define DRV8323_CONF_Msk			(0x7FF << DRV8323_CONF_Pos)

/********************  Bit definition for Fault Status 1 register  ********************/

#define VDS_LC_Pos       (0U)                                            
#define VDS_LC_Msk       (0x1U << VDS_LC_Pos)                      /*!< 0x00000001 */
#define VDS_LC           VDS_LC_Msk                                /*!< Indicates VDS overcurrent fault on the C low-side MOSFET 	*/
#define VDS_HC_Pos       (1U)                                            
#define VDS_HC_Msk       (0x1U << VDS_HC_Pos)                      /*!< 0x00000002 */
#define VDS_HC           VDS_HC_Msk                                /*!< Indicates VDS overcurrent fault on the C high-side MOSFET	*/
#define VDS_LB_Pos       (2U)                                            
#define VDS_LB_Msk       (0x1U << VDS_LB_Pos)                      /*!< 0x00000004 */
#define VDS_LB           VDS_LB_Msk                                /*!< Indicates VDS overcurrent fault on the B low-side MOSFET 	*/
#define VDS_HB_Pos       (3U)                                            
#define VDS_HB_Msk       (0x1U << VDS_HB_Pos)                      /*!< 0x00000008 */
#define VDS_HB           VDS_HB_Msk                                /*!< Indicates VDS overcurrent fault on the B high-side MOSFET	*/
#define VDS_LA_Pos       (4U)                                            
#define VDS_LA_Msk       (0x1U << VDS_LA_Pos)                      /*!< 0x00000010 */
#define VDS_LA           VDS_LA_Msk                                /*!< Indicates VDS overcurrent fault on the A low-side MOSFET 	*/
#define VDS_HA_Pos       (5U)                                            
#define VDS_HA_Msk       (0x1U << VDS_HA_Pos)                      /*!< 0x00000020 */
#define VDS_HA           VDS_HA_Msk                                /*!< Indicates VDS overcurrent fault on the A high-side MOSFET	*/
#define OTSD_Pos         (6U)                                            
#define OTSD_Msk         (0x1U << OTSD_Pos)                        /*!< 0x00000040 */
#define OTSD             OTSD_Msk                                  /*!< Indicates overtemperature shutdown						 	*/
#define UVLO_Pos         (7U)                                            
#define UVLO_Msk         (0x1U << UVLO_Pos)                        /*!< 0x00000080 */
#define UVLO             UVLO_Msk                                  /*!< Indicates undervoltage lockout fault condition			 	*/
#define GDF_Pos          (8U)                                            
#define GDF_Msk          (0x1U << GDF_Pos)                         /*!< 0x00000100 */
#define GDF              GDF_Msk                                   /*!< Indicates gate drive fault condition						*/
#define VDS_OCP_Pos      (9U)                                            
#define VDS_OCP_Msk      (0x1U << VDS_OCP_Pos)                     /*!< 0x00000200 */
#define VDS_OCP          VDS_OCP_Msk                               /*!< Indicates VDS monitor overcurrent fault condition			*/
#define FAULT_Pos        (10U)                                            
#define FAULT_Msk        (0x1U << FAULT_Pos)                       /*!< 0x00000400 */
#define FAULT            FAULT_Msk                                 /*!< Logic OR of FAULT status registers. Mirrors nFAULT pin		*/

/******************** Bit definition for Fault Status 2 register  ********************/

#define VGS_LC_Pos       (0U)                                            
#define VGS_LC_Msk       (0x1U << VGS_LC_Pos)                      /*!< 0x00000001 */
#define VGS_LC           VGS_LC_Msk                                /*!< Indicates gate drive fault on the C low-side MOSFET		 	*/
#define VGS_HC_Pos       (1U)                                            
#define VGS_HC_Msk       (0x1U << VGS_HC_Pos)                      /*!< 0x00000002 */
#define VGS_HC           VGS_HC_Msk                                /*!< Indicates gate drive fault on the C high-side MOSFET		*/
#define VGS_LB_Pos       (2U)                                            
#define VGS_LB_Msk       (0x1U << VGS_LB_Pos)                      /*!< 0x00000004 */
#define VGS_LB           VGS_LB_Msk                                /*!< Indicates gate drive fault on the B low-side MOSFET		 	*/
#define VGS_HB_Pos       (3U)                                            
#define VGS_HB_Msk       (0x1U << VGS_HB_Pos)                      /*!< 0x00000008 */
#define VGS_HB           VGS_HB_Msk                                /*!< Indicates gate drive fault on the B high-side MOSFET		*/
#define VGS_LA_Pos       (4U)                                            
#define VGS_LA_Msk       (0x1U << VGS_LA_Pos)                      /*!< 0x00000010 */
#define VGS_LA           VGS_LA_Msk                                /*!< Indicates gate drive fault on the A low-side MOSFET 		*/
#define VGS_HA_Pos       (5U)                                            
#define VGS_HA_Msk       (0x1U << VGS_HA_Pos)                      /*!< 0x00000020 */
#define VGS_HA           VGS_HA_Msk                                /*!< Indicates gate drive fault on the A high-side MOSFET		*/
#define CPUV_Pos         (6U)                                            
#define CPUV_Msk         (0x1U << CPUV_Pos)                        /*!< 0x00000040 */
#define CPUV             CPUV_Msk                                  /*!< Indicates charge pump undervoltage fault condition			*/
#define OTW_Pos          (7U)                                            
#define OTW_Msk          (0x1U << OTW_Pos)                         /*!< 0x00000080 */
#define OTW              OTW_Msk                                   /*!< Indicates overtemperature warning							*/
#define SC_OC_Pos        (8U)                                            
#define SC_OC_Msk        (0x1U << SC_OC_Pos)                       /*!< 0x00000100 */
#define SC_OC            SC_OC_Msk                                 /*!< Indicates overcurrent on phase C sense amplifier			*/
#define SB_OC_Pos        (9U)                                            
#define SB_OC_Msk        (0x1U << SB_OC_Pos)                       /*!< 0x00000200 */
#define SB_OC            SB_OC_Msk                                 /*!< Indicates overcurrent on phase B sense amplifier			*/
#define SA_OC_Pos        (10U)                                            
#define SA_OC_Msk        (0x1U << SA_OC_Pos)                       /*!< 0x00000400 */
#define SA_OC            SA_OC_Msk                                 /*!< Indicates overcurrent on phase A sense amplifier			*/

/******************** Bit definition for Driver Control register  ********************/

#define CLR_FLT_Pos      (0U)                                            
#define CLR_FLT_Msk      (0x1U << CLR_FLT_Pos)                     /*!< 0x00000001 */
#define CLR_FLT          CLR_FLT_Msk                               /*!< Write a 1 to this bit to clear latched fault bits							*/
#define BRAKE_Pos        (1U)                                            
#define BRAKE_Msk        (0x1U << BRAKE_Pos)                       /*!< 0x00000002 */
#define BRAKE            BRAKE_Msk                                 /*!< Write a 1 to this bit to turn on all three low-side MOSFETs in 1x PWM mode	*/
#define COAST_Pos        (2U)                                            
#define COAST_Msk        (0x1U << COAST_Pos)                       /*!< 0x00000004 */
#define COAST            COAST_Msk                                 /*!< Write a 1 to this bit to put all MOSFETs in the Hi-Z state					*/
#define ONE_PWM_DIR_Pos  (3U)                                            
#define ONE_PWM_DIR_Msk  (0x1U << ONE_PWM_DIR_Pos)                 /*!< 0x00000008 */
#define ONE_PWM_DIR      ONE_PWM_DIR_Msk                           /*!< In 1x PWM mode this bit is ORed with the INHC (DIR) input					*/
#define ONE_PWM_COM_Pos  (4U)                                            
#define ONE_PWM_COM_Msk  (0x1U << ONE_PWM_COM_Pos)                 /*!< 0x00000010 */
#define ONE_PWM_COM      ONE_PWM_COM_Msk                           /*!< Synchronous or asynchronous rectification for 1x PWM mode					*/
#define PWM_MODE_Pos     (5U)                                            
#define PWM_MODE_Msk     (0x3U << PWM_MODE_Pos)                    /*!< 0x00000060 */
#define PWM_MODE         PWM_MODE_Msk                              /*!< Synchronous or asynchronous rectification for 1x PWM mode					*/
#define PWM_MODE_0		 (0x1U << PWM_MODE_Pos)					   /*!< 0x00000020 */
#define PWM_MODE_1		 (0x2U << PWM_MODE_Pos)					   /*!< 0x00000040 */
#define OTW_REP_Pos      (7U)                                            
#define OTW_REP_Msk      (0x1U << OTW_REP_Pos)                     /*!< 0x00000080 */
#define OTW_REP          OTW_REP_Msk                               /*!< Selects if OTW is reported or not on nFault or FAULT bit					*/
#define DIS_GDF_Pos      (8U)                                            
#define DIS_GDF_Msk      (0x1U << DIS_GDF_Pos)                     /*!< 0x00000100 */
#define DIS_GDF          DIS_GDF_Msk                               /*!< Selects if gate drive fault is enabled or not								*/
#define DIS_CPUV_Pos     (9U)                                            
#define DIS_CPUV_Msk     (0x1U << DIS_CPUV_Pos)                    /*!< 0x00000200 */
#define DIS_CPUV         DIS_CPUV_Msk                              /*!< Selects if charge pump UVLO fault is enabled or not							*/
#define DIS_CPUV_Pos     (9U)                                            
#define DIS_CPUV_Msk     (0x1U << DIS_CPUV_Pos)                    /*!< 0x00000200 */
#define DIS_CPUV         DIS_CPUV_Msk                              /*!< Selects if charge pump UVLO fault is enabled or not							*/

/********************  Bit definition for Gate Drive HS register  ********************/

#define IDRIVEN_HS_Pos   (0U)                                            
#define IDRIVEN_HS_Msk   (0xFU << IDRIVEN_HS_Pos)                  /*!< 0x0000000F */
#define IDRIVEN_HS       IDRIVEN_HS_Msk                            /*!< Sink current selection for the high side gates								*/
#define IDRIVEN_HS_0     (0x1U << IDRIVEN_HS_Pos)                  /*!< 0x00000001 */
#define IDRIVEN_HS_1     (0x2U << IDRIVEN_HS_Pos)                  /*!< 0x00000002 */
#define IDRIVEN_HS_2     (0x4U << IDRIVEN_HS_Pos)                  /*!< 0x00000004 */
#define IDRIVEN_HS_3     (0x8U << IDRIVEN_HS_Pos)                  /*!< 0x00000008 */
#define IDRIVEP_HS_Pos   (4U)                                            
#define IDRIVEP_HS_Msk   (0xFU << IDRIVEP_HS_Pos)                  /*!< 0x000000F0 */
#define IDRIVEP_HS       IDRIVEP_HS_Msk                            /*!< Source current selection for the high side gates							*/
#define IDRIVEP_HS_0     (0x1U << IDRIVEP_HS_Pos)                  /*!< 0x00000010 */
#define IDRIVEP_HS_1     (0x2U << IDRIVEP_HS_Pos)                  /*!< 0x00000020 */
#define IDRIVEP_HS_2     (0x4U << IDRIVEP_HS_Pos)                  /*!< 0x00000040 */
#define IDRIVEP_HS_3     (0x8U << IDRIVEP_HS_Pos)                  /*!< 0x00000080 */
#define LOCK_Pos         (8U)                                            
#define LOCK_Msk         (0x7U << LOCK_Pos)                        /*!< 0x00000700 */
#define LOCK             LOCK_Msk                                  /*!< Used to lock every register except this one and BRAKE and COAST				*/
#define LOCK_LOCK        (0x6U << LOCK_Pos)                        /*!< 0x00000600 */
#define LOCK_UNLOCK      (0x3U << LOCK_Pos)                        /*!< 0x00000300 */

/********************  Bit definition for Gate Drive LS register  ********************/

#define IDRIVEN_LS_Pos   (0U)                                            
#define IDRIVEN_LS_Msk   (0xFU << IDRIVEN_LS_Pos)                  /*!< 0x0000000F */
#define IDRIVEN_LS       IDRIVEN_LS_Msk                            /*!< Sink current selection for the low side gates								*/
#define IDRIVEN_LS_0     (0x1U << IDRIVEN_LS_Pos)                  /*!< 0x00000001 */
#define IDRIVEN_LS_1     (0x2U << IDRIVEN_LS_Pos)                  /*!< 0x00000002 */
#define IDRIVEN_LS_2     (0x4U << IDRIVEN_LS_Pos)                  /*!< 0x00000004 */
#define IDRIVEN_LS_3     (0x8U << IDRIVEN_LS_Pos)                  /*!< 0x00000008 */
#define IDRIVEP_LS_Pos   (4U)                                            
#define IDRIVEP_LS_Msk   (0xFU << IDRIVEP_LS_Pos)                  /*!< 0x000000F0 */
#define IDRIVEP_LS       IDRIVEP_LS_Msk                            /*!< Source current selection for the low side gates								*/
#define IDRIVEP_LS_0     (0x1U << IDRIVEP_LS_Pos)                  /*!< 0x00000010 */
#define IDRIVEP_LS_1     (0x2U << IDRIVEP_LS_Pos)                  /*!< 0x00000020 */
#define IDRIVEP_LS_2     (0x4U << IDRIVEP_LS_Pos)                  /*!< 0x00000040 */
#define IDRIVEP_LS_3     (0x8U << IDRIVEP_LS_Pos)                  /*!< 0x00000080 */
#define TDRIVE_Pos       (8U)                                            
#define TDRIVE_Msk       (0x3U << TDRIVE_Pos)                      /*!< 0x00000300 */
#define TDRIVE           TDRIVE_Msk                                /*!< TDRIVE time selection														*/
#define TDRIVE_0         (0x1U << TDRIVE_Pos)                      /*!< 0x00000100 */
#define TDRIVE_1         (0x2U << TDRIVE_Pos)                      /*!< 0x00000200 */
#define CBC_Pos          (10U)                                            
#define CBC_Msk          (0x1U << CBC_Pos)                         /*!< 0x00000400 */
#define CBC              CBC_Msk                                   /*!< In retry OCP_MODE, for VDS_OCP and SEN_OCP, the fault is automatically cleared when a PWM input is given	*/

/********************   Bit definition for OCP Control register   ********************/

#define VDS_LVL_Pos      (0U)                                            
#define VDS_LVL_Msk      (0xFU << VDS_LVL_Pos)                     /*!< 0x0000000F */
#define VDS_LVL          VDS_LVL_Msk                               /*!< Threshold selection for VDS overcurrent detection							*/
#define VDS_LVL_0        (0x1U << VDS_LVL_Pos)                     /*!< 0x00000001 */
#define VDS_LVL_1        (0x2U << VDS_LVL_Pos)                     /*!< 0x00000002 */
#define VDS_LVL_2        (0x4U << VDS_LVL_Pos)                     /*!< 0x00000004 */
#define VDS_LVL_3        (0x8U << VDS_LVL_Pos)                     /*!< 0x00000008 */
#define OCP_DEG_Pos      (4U)                                            
#define OCP_DEG_Msk      (0x3U << OCP_DEG_Pos)                     /*!< 0x00000030 */
#define OCP_DEG          OCP_DEG_Msk                               /*!< Selection of overcurrent deglitch time										*/
#define OCP_DEG_0        (0x1U << OCP_DEG_Pos)                     /*!< 0x00000010 */
#define OCP_DEG_1        (0x2U << OCP_DEG_Pos)                     /*!< 0x00000020 */
#define OCP_MODE_Pos     (6U)                                            
#define OCP_MODE_Msk     (0x3U << OCP_MODE_Pos)                    /*!< 0x000000C0 */
#define OCP_MODE         OCP_MODE_Msk                              /*!< Behavior when an overcurrent detection occurs								*/
#define OCP_MODE_0       (0x1U << OCP_MODE_Pos)                    /*!< 0x00000040 */
#define OCP_MODE_1       (0x2U << OCP_MODE_Pos)                    /*!< 0x00000080 */
#define DEAD_TIME_Pos    (8U)                                            
#define DEAD_TIME_Msk    (0x3U << DEAD_TIME_Pos)                   /*!< 0x00000300 */
#define DEAD_TIME        DEAD_TIME_Msk                             /*!< Dead time selection															*/
#define DEAD_TIME_0      (0x1U << DEAD_TIME_Pos)                   /*!< 0x00000100 */
#define DEAD_TIME_1      (0x2U << DEAD_TIME_Pos)                   /*!< 0x00000200 */
#define TRETRY_Pos       (10U)                                            
#define TRETRY_Msk       (0x1U << TRETRY_Pos)                      /*!< 0x00000400 */
#define TRETRY           TRETRY_Msk                                /*!< Retry time selection for VDS_OCP and SEN_OCP								*/

/********************   Bit definition for CSA Control register   ********************/

#define SEN_LVL_Pos      (0U)                                            
#define SEN_LVL_Msk      (0x3U << SEN_LVL_Pos)                     /*!< 0x00000003 */
#define SEN_LVL          SEN_LVL_Msk                               /*!< Threshold selection for overcurrent detection	(sense resistor)			*/
#define SEN_LVL_0        (0x1U << SEN_LVL_Pos)                     /*!< 0x00000001 */
#define SEN_LVL_1        (0x2U << SEN_LVL_Pos)                     /*!< 0x00000002 */
#define CSA_CAL_C_Pos    (2U)                                            
#define CSA_CAL_C_Msk    (0x1U << CSA_CAL_C_Pos)                   /*!< 0x00000004 */
#define CSA_CAL_C        CSA_CAL_C_Msk                             /*!< Calibration bit for current sense amplifier C								*/
#define CSA_CAL_B_Pos    (3U)                                            
#define CSA_CAL_B_Msk    (0x1U << CSA_CAL_B_Pos)                   /*!< 0x00000008 */
#define CSA_CAL_B        CSA_CAL_B_Msk                             /*!< Calibration bit for current sense amplifier B								*/
#define CSA_CAL_A_Pos    (4U)                                            
#define CSA_CAL_A_Msk    (0x1U << CSA_CAL_A_Pos)                   /*!< 0x00000010 */
#define CSA_CAL_A        CSA_CAL_A_Msk                             /*!< Calibration bit for current sense amplifier A								*/
#define DIS_SEN_Pos      (5U)                                            
#define DIS_SEN_Msk      (0x1U << DIS_SEN_Pos)                     /*!< 0x00000020 */
#define DIS_SEN          DIS_SEN_Msk                               /*!< Selects if sense overcurrent fault is enabled or not						*/
#define CSA_GAIN_Pos     (6U)                                            
#define CSA_GAIN_Msk     (0x3U << CSA_GAIN_Pos)                    /*!< 0x000000C0 */
#define CSA_GAIN         CSA_GAIN_Msk                              /*!< Selection of the gain for the current sense amplifiers						*/
#define CSA_GAIN_0       (0x1U << CSA_GAIN_Pos)                    /*!< 0x00000040 */
#define CSA_GAIN_1       (0x2U << CSA_GAIN_Pos)                    /*!< 0x00000080 */
#define LS_REF_Pos       (8U)                                            
#define LS_REF_Msk       (0x1U << LS_REF_Pos)                      /*!< 0x00000100 */
#define LS_REF           LS_REF_Msk                                /*!< Selects if SPx of SNx is used as negative input for VDS_OCP					*/
#define VREF_DIV_Pos     (9U)                                            
#define VREF_DIV_Msk     (0x1U << VREF_DIV_Pos)                    /*!< 0x00000200 */
#define VREF_DIV         VREF_DIV_Msk                              /*!< Selects if current sense voltage is VREF or VREF/2 for CSA					*/
#define CSA_FET_Pos      (10U)                                            
#define CSA_FET_Msk      (0x1U << CSA_FET_Pos)                     /*!< 0x00000400 */
#define CSA_FET          CSA_FET_Msk                               /*!< Selects if CSA positive input is SPx or SHx									*/



#endif /* DRV8323_H */