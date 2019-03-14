/**
 * @file	drv8323.c
 * @brief  	File containing the driver to communicate by SPI with a DRV8323 gate driver
 * 
 * @written by  	Eliot Ferragni
 * @creation date	14.03.2019
 */
 
/********************               DRV8323 REGISTERS              ********************/

#define FAULT_STATUS_1_REG	0x00
#define FAULT_STATUS_2_REG	0x01
#define DRIVER_CONTROL_REG	0x02
#define GATE_DRIVE_HS_REG	0x03
#define GATE_DRIVE_LS_REG	0x04
#define OCP_CONTROL_REG		0x05
#define CSA_CONTROL_REG		0x06

/********************  Bit definition for FAULT STATUS 1 register  ********************/

#define VDS_LC_Pos       (0U)                                            
#define VDS_LC_Msk       (0x1U << VDS_LC_Pos)                      /*!< 0x00000001 */
#define VDS_LC           VDS_LC_Msk                                /*!< Indicates VDS overcurrent fault on the C low-side MOSFET 	*/
#define VDS_HC_Pos       (1U)                                            
#define VDS_HC_Msk       (0x1U << VDS_HC_Pos)                      /*!< 0x00000001 */
#define VDS_HC           VDS_HC_Msk                                /*!< Indicates VDS overcurrent fault on the C high-side MOSFET	*/
#define VDS_LB_Pos       (2U)                                            
#define VDS_LB_Msk       (0x1U << VDS_LB_Pos)                      /*!< 0x00000001 */
#define VDS_LB           VDS_LB_Msk                                /*!< Indicates VDS overcurrent fault on the B low-side MOSFET 	*/
#define VDS_HB_Pos       (3U)                                            
#define VDS_HB_Msk       (0x1U << VDS_HB_Pos)                      /*!< 0x00000001 */
#define VDS_HB           VDS_HB_Msk                                /*!< Indicates VDS overcurrent fault on the B high-side MOSFET	*/
#define VDS_LA_Pos       (4U)                                            
#define VDS_LA_Msk       (0x1U << VDS_LA_Pos)                      /*!< 0x00000001 */
#define VDS_LA           VDS_LA_Msk                                /*!< Indicates VDS overcurrent fault on the A low-side MOSFET 	*/
#define VDS_HA_Pos       (5U)                                            
#define VDS_HA_Msk       (0x1U << VDS_HA_Pos)                      /*!< 0x00000001 */
#define VDS_HA           VDS_HA_Msk                                /*!< Indicates VDS overcurrent fault on the A high-side MOSFET	*/
#define OTSD_Pos         (6U)                                            
#define OTSD_Msk         (0x1U << OTSD_Pos)                        /*!< 0x00000001 */
#define OTSD             OTSD_Msk                                  /*!< Indicates overtemperature shutdown						 	*/
#define UVLO_Pos         (7U)                                            
#define UVLO_Msk         (0x1U << UVLO_Pos)                        /*!< 0x00000001 */
#define UVLO             UVLO_Msk                                  /*!< Indicates undervoltage lockout fault condition			 	*/
#define GDF_Pos          (8U)                                            
#define GDF_Msk          (0x1U << GDF_Pos)                         /*!< 0x00000001 */
#define GDF              GDF_Msk                                   /*!< Indicates gate drive fault condition						*/
#define VDS_OCP_Pos      (9U)                                            
#define VDS_OCP_Msk      (0x1U << VDS_OCP_Pos)                     /*!< 0x00000001 */
#define VDS_OCP          VDS_OCP_Msk                               /*!< Indicates VDS monitor overcurrent fault condition			*/
#define FAULT_Pos        (10U)                                            
#define FAULT_Msk        (0x1U << FAULT_Pos)                       /*!< 0x00000001 */
#define FAULT            FAULT_Msk                                 /*!< Logic OR of FAULT status registers. Mirrors nFAULT pin		*/

/********************  Bit definition for FAULT STATUS 2 register  ********************/

#define VGS_LC_Pos       (0U)                                            
#define VGS_LC_Msk       (0x1U << VGS_LC_Pos)                      /*!< 0x00000001 */
#define VGS_LC           VGS_LC_Msk                                /*!< Indicates gate drive fault on the C low-side MOSFET		 	*/
#define VGS_HC_Pos       (1U)                                            
#define VGS_HC_Msk       (0x1U << VGS_HC_Pos)                      /*!< 0x00000001 */
#define VGS_HC           VGS_HC_Msk                                /*!< Indicates gate drive fault on the C high-side MOSFET		*/
#define VGS_LB_Pos       (2U)                                            
#define VGS_LB_Msk       (0x1U << VGS_LB_Pos)                      /*!< 0x00000001 */
#define VGS_LB           VGS_LB_Msk                                /*!< Indicates gate drive fault on the B low-side MOSFET		 	*/
#define VGS_HB_Pos       (3U)                                            
#define VGS_HB_Msk       (0x1U << VGS_HB_Pos)                      /*!< 0x00000001 */
#define VGS_HB           VGS_HB_Msk                                /*!< Indicates gate drive fault on the B high-side MOSFET		*/
#define VGS_LA_Pos       (4U)                                            
#define VGS_LA_Msk       (0x1U << VGS_LA_Pos)                      /*!< 0x00000001 */
#define VGS_LA           VGS_LA_Msk                                /*!< Indicates gate drive fault on the A low-side MOSFET 		*/
#define VGS_HA_Pos       (5U)                                            
#define VGS_HA_Msk       (0x1U << VGS_HA_Pos)                      /*!< 0x00000001 */
#define VGS_HA           VGS_HA_Msk                                /*!< Indicates gate drive fault on the A high-side MOSFET		*/




