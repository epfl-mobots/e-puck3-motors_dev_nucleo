/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/*
 * This file has been automatically generated using ChibiStudio board
 * generator plugin. Do not edit manually.
 */

#ifndef BOARD_H
#define BOARD_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/*
 * Setup for STMicroelectronics STM32 Nucleo144-F746ZG board.
 */

/*
 * Board identifier.
 */
#define BOARD_ST_NUCLEO144_F746ZG
#define BOARD_NAME                  "STMicroelectronics STM32 Nucleo144-F746ZG"

/*
 * Ethernet PHY type.
 */
#define BOARD_PHY_ID                MII_LAN8742A_ID
#define BOARD_PHY_RMII

/*
 * Board oscillators-related settings.
 */
#if !defined(STM32_LSECLK)
#define STM32_LSECLK                32768U
#endif

#define STM32_LSEDRV                (3U << 3U)

#if !defined(STM32_HSECLK)
#define STM32_HSECLK                8000000U
#endif

#define STM32_HSE_BYPASS

/*
 * Board voltages.
 * Required for performance limits calculation.
 */
#define STM32_VDD                   300U

/*
 * MCU type as defined in the ST header.
 */
#define STM32F746xx

/*
 * IO pins assignments.
 */
#define GPIOA_ADC3_MOT1_PH1         0U
#define GPIOA_ADC3_MOT1_PH2         1U
#define GPIOA_ADC3_MOT1_PH3         2U
#define GPIOA_ADC3_MOT1_N           3U
#define GPIOA_ADC1_MOT1_PH1_C       4U
#define GPIOA_ADC1_MOT1_PH2_C       5U
#define GPIOA_ADC1_MOT1_PH3_C       6U
#define GPIOA_OUT_MOT4_PH1_N        7U
#define GPIOA_OUT_MOT1_PH1_P        8U
#define GPIOA_OTG_FS_VBUS           9U
#define GPIOA_OUT_MOT1_PH3_P        10U
#define GPIOA_OTG_FS_DM             11U
#define GPIOA_OTG_FS_DP             12U
#define GPIOA_SWDIO                 13U
#define GPIOA_SWCLK                 14U
#define GPIOA_OUT_MOT2_PH1_P        15U

#define GPIOB_OUT_MOT3_PH1_P        0U
#define GPIOB_ADC1_MOT234_PH1_C     1U
#define GPIOB_PIN2                  2U
#define GPIOB_OUT_MOT2_PH1_N        3U
#define GPIOB_OUT_MOT2_PH3_P        4U
#define GPIOB_OUT_MOT2_PH3_N        5U
#define GPIOB_PIN6                  6U
#define GPIOB_PIN7                  7U
#define GPIOB_PIN8                  8U
#define GPIOB_OUT_MOT3_PH3_N        9U
#define GPIOB_OUT_MOT2_PH2_P        10U
#define GPIOB_OUT_MOT2_PH2_N        11U
#define GPIOB_EN_DRIVER_1           12U
#define GPIOB_OUT_MOT1_PH1_N        13U
#define GPIOB_OUT_MOT4_PH2_N        14U
#define GPIOB_OUT_MOT4_PH3_N        15U

#define GPIOC_ADC3_MOT3_PH3         0U
#define GPIOC_ADC3_MOT3_N           1U
#define GPIOC_ADC3_MOT4_PH1         2U
#define GPIOC_ADC3_MOT4_PH2         3U
#define GPIOC_ADC1_MOT234_PH2_C     4U
#define GPIOC_ADC1_MOT234_PH3_C     5U
#define GPIOC_OUT_MOT4_PH1_P        6U
#define GPIOC_OUT_MOT4_PH2_P        7U
#define GPIOC_OUT_MOT4_PH3_P        8U
#define GPIOC_OUT_MOT3_PH1_N        9U
#define GPIOC_SPI3_SCK              10U
#define GPIOC_SPI3_MISO             11U
#define GPIOC_PIN12                 12U
#define GPIOC_PIN13                 13U
#define GPIOC_PIN14                 14U
#define GPIOC_PIN15                 15U

#define GPIOD_PIN0                  0U
#define GPIOD_PIN1                  1U
#define GPIOD_PIN2                  2U
#define GPIOD_INT_ENCODERS_n        3U
#define GPIOD_CS_DRIVER_1_n         4U
#define GPIOD_CS_ENCODERS_n         5U
#define GPIOD_SPI3_MOSI             6U
#define GPIOD_CS_DRIVER_2_n         7U
#define GPIOD_PIN8                  8U
#define GPIOD_PIN9                  9U
#define GPIOD_CS_DRIVER_3_n         10U
#define GPIOD_CS_DRIVER_4_n         11U
#define GPIOD_OUT_MOT3_PH2_P        12U
#define GPIOD_OUT_MOT3_PH2_N        13U
#define GPIOD_OUT_MOT3_PH3_P        14U
#define GPIOD_PIN15                 15U

#define GPIOE_PIN0                  0U
#define GPIOE_PIN1                  1U
#define GPIOE_PIN2                  2U
#define GPIOE_PIN3                  3U
#define GPIOE_PIN4                  4U
#define GPIOE_SWCLK_OUT             5U
#define GPIOE_SWDIO_OUT             6U
#define GPIOE_PIN7                  7U
#define GPIOE_PIN8                  8U
#define GPIOE_PWR_ON_BTN_STATE_n    9U
#define GPIOE_OUT_MOT1_PH2_N        10U
#define GPIOE_OUT_MOT1_PH2_P        11U
#define GPIOE_OUT_MOT1_PH3_N        12U
#define GPIOE_EN_DRIVER_2           13U
#define GPIOE_EN_DRIVER_3           14U
#define GPIOE_EN_DRIVER_4           15U

#define GPIOF_I2C2_SDA              0U
#define GPIOF_I2C2_SCL              1U
#define GPIOF_PIN2                  2U
#define GPIOF_ADC3_MOT3_PH2         3U
#define GPIOF_ADC3_MOT4_PH3         4U
#define GPIOF_ADC3_MOT4_N           5U
#define GPIOF_ADC3_MOT2_PH1         6U
#define GPIOF_ADC3_MOT2_PH2         7U
#define GPIOF_ADC3_MOT2_PH3         8U
#define GPIOF_ADC3_MOT2_N           9U
#define GPIOF_ADC3_MOT3_PH1         10U
#define GPIOF_PIN11                 11U
#define GPIOF_FAULT_DRIVER_1_n      12U
#define GPIOF_FAULT_DRIVER_2_n      13U
#define GPIOF_FAULT_DRIVER_3_n      14U
#define GPIOF_FAULT_DRIVER_4_n      15U

#define GPIOG_STATUS_LED1_RED       0U
#define GPIOG_STATUS_LED1_GREEN     1U
#define GPIOG_STATUS_LED1_BLUE      2U
#define GPIOG_STATUS_LED2_RED       3U
#define GPIOG_STATUS_LED2_GREEN     4U
#define GPIOG_STATUS_LED2_BLUE      5U
#define GPIOG_STATUS_LED3_RED       6U
#define GPIOG_STATUS_LED3_GREEN     7U
#define GPIOG_STATUS_LED3_BLUE      8U
#define GPIOG_PIN9                  9U
#define GPIOG_PWR_PP_STATE          10U
#define GPIOG_INT_PD_CTRL_n         11U
#define GPIOG_PWR_ON                12U
#define GPIOG_PIN13                 13U
#define GPIOG_PIN14                 14U
#define GPIOG_PIN15                 15U

#define GPIOH_OSC_IN                0U
#define GPIOH_OSC_OUT               1U
#define GPIOH_PIN2                  2U
#define GPIOH_PIN3                  3U
#define GPIOH_PIN4                  4U
#define GPIOH_PIN5                  5U
#define GPIOH_PIN6                  6U
#define GPIOH_PIN7                  7U
#define GPIOH_PIN8                  8U
#define GPIOH_PIN9                  9U
#define GPIOH_PIN10                 10U
#define GPIOH_PIN11                 11U
#define GPIOH_PIN12                 12U
#define GPIOH_PIN13                 13U
#define GPIOH_PIN14                 14U
#define GPIOH_PIN15                 15U

#define GPIOI_PIN0                  0U
#define GPIOI_PIN1                  1U
#define GPIOI_PIN2                  2U
#define GPIOI_PIN3                  3U
#define GPIOI_PIN4                  4U
#define GPIOI_PIN5                  5U
#define GPIOI_PIN6                  6U
#define GPIOI_PIN7                  7U
#define GPIOI_PIN8                  8U
#define GPIOI_PIN9                  9U
#define GPIOI_PIN10                 10U
#define GPIOI_PIN11                 11U
#define GPIOI_PIN12                 12U
#define GPIOI_PIN13                 13U
#define GPIOI_PIN14                 14U
#define GPIOI_PIN15                 15U

#define GPIOJ_PIN0                  0U
#define GPIOJ_PIN1                  1U
#define GPIOJ_PIN2                  2U
#define GPIOJ_PIN3                  3U
#define GPIOJ_PIN4                  4U
#define GPIOJ_PIN5                  5U
#define GPIOJ_PIN6                  6U
#define GPIOJ_PIN7                  7U
#define GPIOJ_PIN8                  8U
#define GPIOJ_PIN9                  9U
#define GPIOJ_PIN10                 10U
#define GPIOJ_PIN11                 11U
#define GPIOJ_PIN12                 12U
#define GPIOJ_PIN13                 13U
#define GPIOJ_PIN14                 14U
#define GPIOJ_PIN15                 15U

#define GPIOK_PIN0                  0U
#define GPIOK_PIN1                  1U
#define GPIOK_PIN2                  2U
#define GPIOK_PIN3                  3U
#define GPIOK_PIN4                  4U
#define GPIOK_PIN5                  5U
#define GPIOK_PIN6                  6U
#define GPIOK_PIN7                  7U
#define GPIOK_PIN8                  8U
#define GPIOK_PIN9                  9U
#define GPIOK_PIN10                 10U
#define GPIOK_PIN11                 11U
#define GPIOK_PIN12                 12U
#define GPIOK_PIN13                 13U
#define GPIOK_PIN14                 14U
#define GPIOK_PIN15                 15U

/*
 * IO lines assignments.
 */
#define LINE_ADC3_MOT1_PH1          PAL_LINE(GPIOA, 0U)
#define LINE_ADC3_MOT1_PH2          PAL_LINE(GPIOA, 1U)
#define LINE_ADC3_MOT1_PH3          PAL_LINE(GPIOA, 2U)
#define LINE_ADC3_MOT1_N            PAL_LINE(GPIOA, 3U)
#define LINE_ADC1_MOT1_PH1_C        PAL_LINE(GPIOA, 4U)
#define LINE_ADC1_MOT1_PH2_C        PAL_LINE(GPIOA, 5U)
#define LINE_ADC1_MOT1_PH3_C        PAL_LINE(GPIOA, 6U)
#define LINE_OUT_MOT4_PH1_N         PAL_LINE(GPIOA, 7U)
#define LINE_OUT_MOT1_PH1_P         PAL_LINE(GPIOA, 8U)
#define LINE_OTG_FS_VBUS            PAL_LINE(GPIOA, 9U)
#define LINE_OUT_MOT1_PH3_P         PAL_LINE(GPIOA, 10U)
#define LINE_OTG_FS_DM              PAL_LINE(GPIOA, 11U)
#define LINE_OTG_FS_DP              PAL_LINE(GPIOA, 12U)
#define LINE_SWDIO                  PAL_LINE(GPIOA, 13U)
#define LINE_SWCLK                  PAL_LINE(GPIOA, 14U)
#define LINE_OUT_MOT2_PH1_P         PAL_LINE(GPIOA, 15U)

#define LINE_OUT_MOT3_PH1_P         PAL_LINE(GPIOB, 0U)
#define LINE_ADC1_MOT234_PH1_C      PAL_LINE(GPIOB, 1U)
#define LINE_OUT_MOT2_PH1_N         PAL_LINE(GPIOB, 3U)
#define LINE_OUT_MOT2_PH3_P         PAL_LINE(GPIOB, 4U)
#define LINE_OUT_MOT2_PH3_N         PAL_LINE(GPIOB, 5U)
#define LINE_OUT_MOT3_PH3_N         PAL_LINE(GPIOB, 9U)
#define LINE_OUT_MOT2_PH2_P         PAL_LINE(GPIOB, 10U)
#define LINE_OUT_MOT2_PH2_N         PAL_LINE(GPIOB, 11U)
#define LINE_EN_DRIVER_1            PAL_LINE(GPIOB, 12U)
#define LINE_OUT_MOT1_PH1_N         PAL_LINE(GPIOB, 13U)
#define LINE_OUT_MOT4_PH2_N         PAL_LINE(GPIOB, 14U)
#define LINE_OUT_MOT4_PH3_N         PAL_LINE(GPIOB, 15U)

#define LINE_ADC3_MOT3_PH3          PAL_LINE(GPIOC, 0U)
#define LINE_ADC3_MOT3_N            PAL_LINE(GPIOC, 1U)
#define LINE_ADC3_MOT4_PH1          PAL_LINE(GPIOC, 2U)
#define LINE_ADC3_MOT4_PH2          PAL_LINE(GPIOC, 3U)
#define LINE_ADC1_MOT234_PH2_C      PAL_LINE(GPIOC, 4U)
#define LINE_ADC1_MOT234_PH3_C      PAL_LINE(GPIOC, 5U)
#define LINE_OUT_MOT4_PH1_P         PAL_LINE(GPIOC, 6U)
#define LINE_OUT_MOT4_PH2_P         PAL_LINE(GPIOC, 7U)
#define LINE_OUT_MOT4_PH3_P         PAL_LINE(GPIOC, 8U)
#define LINE_OUT_MOT3_PH1_N         PAL_LINE(GPIOC, 9U)
#define LINE_SPI3_SCK               PAL_LINE(GPIOC, 10U)
#define LINE_SPI3_MISO              PAL_LINE(GPIOC, 11U)

#define LINE_INT_ENCODERS_n         PAL_LINE(GPIOD, 3U)
#define LINE_CS_DRIVER_1_n          PAL_LINE(GPIOD, 4U)
#define LINE_CS_ENCODERS_n          PAL_LINE(GPIOD, 5U)
#define LINE_SPI3_MOSI              PAL_LINE(GPIOD, 6U)
#define LINE_CS_DRIVER_2_n          PAL_LINE(GPIOD, 7U)
#define LINE_CS_DRIVER_3_n          PAL_LINE(GPIOD, 10U)
#define LINE_CS_DRIVER_4_n          PAL_LINE(GPIOD, 11U)
#define LINE_OUT_MOT3_PH2_P         PAL_LINE(GPIOD, 12U)
#define LINE_OUT_MOT3_PH2_N         PAL_LINE(GPIOD, 13U)
#define LINE_OUT_MOT3_PH3_P         PAL_LINE(GPIOD, 14U)

#define LINE_SWCLK_OUT              PAL_LINE(GPIOE, 5U)
#define LINE_SWDIO_OUT              PAL_LINE(GPIOE, 6U)
#define LINE_PWR_ON_BTN_STATE_n     PAL_LINE(GPIOE, 9U)
#define LINE_OUT_MOT1_PH2_N         PAL_LINE(GPIOE, 10U)
#define LINE_OUT_MOT1_PH2_P         PAL_LINE(GPIOE, 11U)
#define LINE_OUT_MOT1_PH3_N         PAL_LINE(GPIOE, 12U)
#define LINE_EN_DRIVER_2            PAL_LINE(GPIOE, 13U)
#define LINE_EN_DRIVER_3            PAL_LINE(GPIOE, 14U)
#define LINE_EN_DRIVER_4            PAL_LINE(GPIOE, 15U)

#define LINE_I2C2_SDA               PAL_LINE(GPIOF, 0U)
#define LINE_I2C2_SCL               PAL_LINE(GPIOF, 1U)
#define LINE_ADC3_MOT3_PH2          PAL_LINE(GPIOF, 3U)
#define LINE_ADC3_MOT4_PH3          PAL_LINE(GPIOF, 4U)
#define LINE_ADC3_MOT4_N            PAL_LINE(GPIOF, 5U)
#define LINE_ADC3_MOT2_PH1          PAL_LINE(GPIOF, 6U)
#define LINE_ADC3_MOT2_PH2          PAL_LINE(GPIOF, 7U)
#define LINE_ADC3_MOT2_PH3          PAL_LINE(GPIOF, 8U)
#define LINE_ADC3_MOT2_N            PAL_LINE(GPIOF, 9U)
#define LINE_ADC3_MOT3_PH1          PAL_LINE(GPIOF, 10U)
#define LINE_FAULT_DRIVER_1_n       PAL_LINE(GPIOF, 12U)
#define LINE_FAULT_DRIVER_2_n       PAL_LINE(GPIOF, 13U)
#define LINE_FAULT_DRIVER_3_n       PAL_LINE(GPIOF, 14U)
#define LINE_FAULT_DRIVER_4_n       PAL_LINE(GPIOF, 15U)

#define LINE_STATUS_LED1_RED        PAL_LINE(GPIOG, 0U)
#define LINE_STATUS_LED1_GREEN      PAL_LINE(GPIOG, 1U)
#define LINE_STATUS_LED1_BLUE       PAL_LINE(GPIOG, 2U)
#define LINE_STATUS_LED2_RED        PAL_LINE(GPIOG, 3U)
#define LINE_STATUS_LED2_GREEN      PAL_LINE(GPIOG, 4U)
#define LINE_STATUS_LED2_BLUE       PAL_LINE(GPIOG, 5U)
#define LINE_STATUS_LED3_RED        PAL_LINE(GPIOG, 6U)
#define LINE_STATUS_LED3_GREEN      PAL_LINE(GPIOG, 7U)
#define LINE_STATUS_LED3_BLUE       PAL_LINE(GPIOG, 8U)
#define LINE_PWR_PP_STATE           PAL_LINE(GPIOG, 10U)
#define LINE_INT_PD_CTRL_n          PAL_LINE(GPIOG, 11U)
#define LINE_PWR_ON                 PAL_LINE(GPIOG, 12U)

#define LINE_OSC_IN                 PAL_LINE(GPIOH, 0U)
#define LINE_OSC_OUT                PAL_LINE(GPIOH, 1U)

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

/*
 * GPIOA setup:
 *
 * PA0  - ADC3_MOT1_PH1             (analog).
 * PA1  - ADC3_MOT1_PH2             (analog).
 * PA2  - ADC3_MOT1_PH3             (analog).
 * PA3  - ADC3_MOT1_N               (analog).
 * PA4  - ADC1_MOT1_PH1_C           (analog).
 * PA5  - ADC1_MOT1_PH2_C           (analog).
 * PA6  - ADC1_MOT1_PH3_C           (analog).
 * PA7  - OUT_MOT4_PH1_N            (alternate 3).
 * PA8  - OUT_MOT1_PH1_P            (alternate 1).
 * PA9  - OTG_FS_VBUS               (input floating).
 * PA10 - OUT_MOT1_PH3_P            (alternate 1).
 * PA11 - OTG_FS_DM                 (alternate 10).
 * PA12 - OTG_FS_DP                 (alternate 10).
 * PA13 - SWDIO                     (alternate 0).
 * PA14 - SWCLK                     (alternate 0).
 * PA15 - OUT_MOT2_PH1_P            (alternate 1).
 */
#define VAL_GPIOA_MODER             (PIN_MODE_ANALOG(GPIOA_ADC3_MOT1_PH1) | \
                                     PIN_MODE_ANALOG(GPIOA_ADC3_MOT1_PH2) | \
                                     PIN_MODE_ANALOG(GPIOA_ADC3_MOT1_PH3) | \
                                     PIN_MODE_ANALOG(GPIOA_ADC3_MOT1_N) |   \
                                     PIN_MODE_ANALOG(GPIOA_ADC1_MOT1_PH1_C) |\
                                     PIN_MODE_ANALOG(GPIOA_ADC1_MOT1_PH2_C) |\
                                     PIN_MODE_ANALOG(GPIOA_ADC1_MOT1_PH3_C) |\
                                     PIN_MODE_ALTERNATE(GPIOA_OUT_MOT4_PH1_N) |\
                                     PIN_MODE_ALTERNATE(GPIOA_OUT_MOT1_PH1_P) |\
                                     PIN_MODE_INPUT(GPIOA_OTG_FS_VBUS) |    \
                                     PIN_MODE_ALTERNATE(GPIOA_OUT_MOT1_PH3_P) |\
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DM) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_OTG_FS_DP) |  \
                                     PIN_MODE_ALTERNATE(GPIOA_SWDIO) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_SWCLK) |      \
                                     PIN_MODE_ALTERNATE(GPIOA_OUT_MOT2_PH1_P))
#define VAL_GPIOA_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOA_ADC3_MOT1_PH1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC3_MOT1_PH2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC3_MOT1_PH3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC3_MOT1_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1_MOT1_PH1_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1_MOT1_PH2_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_ADC1_MOT1_PH3_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OUT_MOT4_PH1_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OUT_MOT1_PH1_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_VBUS) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OUT_MOT1_PH3_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DM) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OTG_FS_DP) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWDIO) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_SWCLK) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOA_OUT_MOT2_PH1_P))
#define VAL_GPIOA_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOA_ADC3_MOT1_PH1) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC3_MOT1_PH2) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC3_MOT1_PH3) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC3_MOT1_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC1_MOT1_PH1_C) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC1_MOT1_PH2_C) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_ADC1_MOT1_PH3_C) |\
                                     PIN_OSPEED_HIGH(GPIOA_OUT_MOT4_PH1_N) |\
                                     PIN_OSPEED_HIGH(GPIOA_OUT_MOT1_PH1_P) |\
                                     PIN_OSPEED_VERYLOW(GPIOA_OTG_FS_VBUS) |\
                                     PIN_OSPEED_HIGH(GPIOA_OUT_MOT1_PH3_P) |\
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DM) |     \
                                     PIN_OSPEED_HIGH(GPIOA_OTG_FS_DP) |     \
                                     PIN_OSPEED_HIGH(GPIOA_SWDIO) |         \
                                     PIN_OSPEED_HIGH(GPIOA_SWCLK) |         \
                                     PIN_OSPEED_HIGH(GPIOA_OUT_MOT2_PH1_P))
#define VAL_GPIOA_PUPDR             (PIN_PUPDR_FLOATING(GPIOA_ADC3_MOT1_PH1) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ADC3_MOT1_PH2) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ADC3_MOT1_PH3) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ADC3_MOT1_N) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1_MOT1_PH1_C) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1_MOT1_PH2_C) |\
                                     PIN_PUPDR_FLOATING(GPIOA_ADC1_MOT1_PH3_C) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OUT_MOT4_PH1_N) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OUT_MOT1_PH1_P) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_VBUS) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OUT_MOT1_PH3_P) |\
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DM) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_OTG_FS_DP) |  \
                                     PIN_PUPDR_FLOATING(GPIOA_SWDIO) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_SWCLK) |      \
                                     PIN_PUPDR_FLOATING(GPIOA_OUT_MOT2_PH1_P))
#define VAL_GPIOA_ODR               (PIN_ODR_HIGH(GPIOA_ADC3_MOT1_PH1) |    \
                                     PIN_ODR_HIGH(GPIOA_ADC3_MOT1_PH2) |    \
                                     PIN_ODR_HIGH(GPIOA_ADC3_MOT1_PH3) |    \
                                     PIN_ODR_HIGH(GPIOA_ADC3_MOT1_N) |      \
                                     PIN_ODR_HIGH(GPIOA_ADC1_MOT1_PH1_C) |  \
                                     PIN_ODR_HIGH(GPIOA_ADC1_MOT1_PH2_C) |  \
                                     PIN_ODR_HIGH(GPIOA_ADC1_MOT1_PH3_C) |  \
                                     PIN_ODR_LOW(GPIOA_OUT_MOT4_PH1_N) |    \
                                     PIN_ODR_LOW(GPIOA_OUT_MOT1_PH1_P) |    \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_VBUS) |      \
                                     PIN_ODR_LOW(GPIOA_OUT_MOT1_PH3_P) |    \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DM) |        \
                                     PIN_ODR_HIGH(GPIOA_OTG_FS_DP) |        \
                                     PIN_ODR_HIGH(GPIOA_SWDIO) |            \
                                     PIN_ODR_HIGH(GPIOA_SWCLK) |            \
                                     PIN_ODR_LOW(GPIOA_OUT_MOT2_PH1_P))
#define VAL_GPIOA_AFRL              (PIN_AFIO_AF(GPIOA_ADC3_MOT1_PH1, 0U) | \
                                     PIN_AFIO_AF(GPIOA_ADC3_MOT1_PH2, 0U) | \
                                     PIN_AFIO_AF(GPIOA_ADC3_MOT1_PH3, 0U) | \
                                     PIN_AFIO_AF(GPIOA_ADC3_MOT1_N, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_ADC1_MOT1_PH1_C, 0U) |\
                                     PIN_AFIO_AF(GPIOA_ADC1_MOT1_PH2_C, 0U) |\
                                     PIN_AFIO_AF(GPIOA_ADC1_MOT1_PH3_C, 0U) |\
                                     PIN_AFIO_AF(GPIOA_OUT_MOT4_PH1_N, 3U))
#define VAL_GPIOA_AFRH              (PIN_AFIO_AF(GPIOA_OUT_MOT1_PH1_P, 1U) |\
                                     PIN_AFIO_AF(GPIOA_OTG_FS_VBUS, 0U) |   \
                                     PIN_AFIO_AF(GPIOA_OUT_MOT1_PH3_P, 1U) |\
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DM, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_OTG_FS_DP, 10U) |    \
                                     PIN_AFIO_AF(GPIOA_SWDIO, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_SWCLK, 0U) |         \
                                     PIN_AFIO_AF(GPIOA_OUT_MOT2_PH1_P, 1U))

/*
 * GPIOB setup:
 *
 * PB0  - OUT_MOT3_PH1_P            (alternate 2).
 * PB1  - ADC1_MOT234_PH1_C         (analog).
 * PB2  - PIN2                      (input pullup).
 * PB3  - OUT_MOT2_PH1_N            (alternate 1).
 * PB4  - OUT_MOT2_PH3_P            (alternate 2).
 * PB5  - OUT_MOT2_PH3_N            (alternate 2).
 * PB6  - PIN6                      (input pullup).
 * PB7  - PIN7                      (input pullup).
 * PB8  - PIN8                      (input pullup).
 * PB9  - OUT_MOT3_PH3_N            (alternate 2).
 * PB10 - OUT_MOT2_PH2_P            (alternate 1).
 * PB11 - OUT_MOT2_PH2_N            (alternate 1).
 * PB12 - EN_DRIVER_1               (output pushpull minimum).
 * PB13 - OUT_MOT1_PH1_N            (alternate 1).
 * PB14 - OUT_MOT4_PH2_N            (alternate 3).
 * PB15 - OUT_MOT4_PH3_N            (alternate 3).
 */
#define VAL_GPIOB_MODER             (PIN_MODE_ALTERNATE(GPIOB_OUT_MOT3_PH1_P) |\
                                     PIN_MODE_ANALOG(GPIOB_ADC1_MOT234_PH1_C) |\
                                     PIN_MODE_INPUT(GPIOB_PIN2) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT2_PH1_N) |\
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT2_PH3_P) |\
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT2_PH3_N) |\
                                     PIN_MODE_INPUT(GPIOB_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOB_PIN8) |           \
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT3_PH3_N) |\
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT2_PH2_P) |\
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT2_PH2_N) |\
                                     PIN_MODE_OUTPUT(GPIOB_EN_DRIVER_1) |   \
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT1_PH1_N) |\
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT4_PH2_N) |\
                                     PIN_MODE_ALTERNATE(GPIOB_OUT_MOT4_PH3_N))
#define VAL_GPIOB_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT3_PH1_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_ADC1_MOT234_PH1_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT2_PH1_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT2_PH3_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT2_PH3_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT3_PH3_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT2_PH2_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT2_PH2_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_EN_DRIVER_1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT1_PH1_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT4_PH2_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOB_OUT_MOT4_PH3_N))
#define VAL_GPIOB_OSPEEDR           (PIN_OSPEED_HIGH(GPIOB_OUT_MOT3_PH1_P) |\
                                     PIN_OSPEED_VERYLOW(GPIOB_ADC1_MOT234_PH1_C) |\
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN2) |       \
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT2_PH1_N) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT2_PH3_P) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT2_PH3_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOB_PIN8) |       \
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT3_PH3_N) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT2_PH2_P) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT2_PH2_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOB_EN_DRIVER_1) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT1_PH1_N) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT4_PH2_N) |\
                                     PIN_OSPEED_HIGH(GPIOB_OUT_MOT4_PH3_N))
#define VAL_GPIOB_PUPDR             (PIN_PUPDR_FLOATING(GPIOB_OUT_MOT3_PH1_P) |\
                                     PIN_PUPDR_FLOATING(GPIOB_ADC1_MOT234_PH1_C) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT2_PH1_N) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT2_PH3_P) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT2_PH3_N) |\
                                     PIN_PUPDR_PULLUP(GPIOB_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOB_PIN8) |         \
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT3_PH3_N) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT2_PH2_P) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT2_PH2_N) |\
                                     PIN_PUPDR_FLOATING(GPIOB_EN_DRIVER_1) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT1_PH1_N) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT4_PH2_N) |\
                                     PIN_PUPDR_FLOATING(GPIOB_OUT_MOT4_PH3_N))
#define VAL_GPIOB_ODR               (PIN_ODR_LOW(GPIOB_OUT_MOT3_PH1_P) |    \
                                     PIN_ODR_HIGH(GPIOB_ADC1_MOT234_PH1_C) |\
                                     PIN_ODR_HIGH(GPIOB_PIN2) |             \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT2_PH1_N) |    \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT2_PH3_P) |    \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT2_PH3_N) |    \
                                     PIN_ODR_HIGH(GPIOB_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOB_PIN8) |             \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT3_PH3_N) |    \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT2_PH2_P) |    \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT2_PH2_N) |    \
                                     PIN_ODR_LOW(GPIOB_EN_DRIVER_1) |       \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT1_PH1_N) |    \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT4_PH2_N) |    \
                                     PIN_ODR_LOW(GPIOB_OUT_MOT4_PH3_N))
#define VAL_GPIOB_AFRL              (PIN_AFIO_AF(GPIOB_OUT_MOT3_PH1_P, 2U) |\
                                     PIN_AFIO_AF(GPIOB_ADC1_MOT234_PH1_C, 0U) |\
                                     PIN_AFIO_AF(GPIOB_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_OUT_MOT2_PH1_N, 1U) |\
                                     PIN_AFIO_AF(GPIOB_OUT_MOT2_PH3_P, 2U) |\
                                     PIN_AFIO_AF(GPIOB_OUT_MOT2_PH3_N, 2U) |\
                                     PIN_AFIO_AF(GPIOB_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_PIN7, 0U))
#define VAL_GPIOB_AFRH              (PIN_AFIO_AF(GPIOB_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOB_OUT_MOT3_PH3_N, 2U) |\
                                     PIN_AFIO_AF(GPIOB_OUT_MOT2_PH2_P, 1U) |\
                                     PIN_AFIO_AF(GPIOB_OUT_MOT2_PH2_N, 1U) |\
                                     PIN_AFIO_AF(GPIOB_EN_DRIVER_1, 0U) |   \
                                     PIN_AFIO_AF(GPIOB_OUT_MOT1_PH1_N, 1U) |\
                                     PIN_AFIO_AF(GPIOB_OUT_MOT4_PH2_N, 3U) |\
                                     PIN_AFIO_AF(GPIOB_OUT_MOT4_PH3_N, 3U))

/*
 * GPIOC setup:
 *
 * PC0  - ADC3_MOT3_PH3             (analog).
 * PC1  - ADC3_MOT3_N               (analog).
 * PC2  - ADC3_MOT4_PH1             (analog).
 * PC3  - ADC3_MOT4_PH2             (analog).
 * PC4  - ADC1_MOT234_PH2_C         (analog).
 * PC5  - ADC1_MOT234_PH3_C         (analog).
 * PC6  - OUT_MOT4_PH1_P            (alternate 3).
 * PC7  - OUT_MOT4_PH2_P            (alternate 3).
 * PC8  - OUT_MOT4_PH3_P            (alternate 3).
 * PC9  - OUT_MOT3_PH1_N            (alternate 2).
 * PC10 - SPI3_SCK                  (alternate 6).
 * PC11 - SPI3_MISO                 (alternate 6).
 * PC12 - PIN12                     (input pullup).
 * PC13 - PIN13                     (input pullup).
 * PC14 - PIN14                     (input pullup).
 * PC15 - PIN15                     (input pullup).
 */
#define VAL_GPIOC_MODER             (PIN_MODE_ANALOG(GPIOC_ADC3_MOT3_PH3) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC3_MOT3_N) |   \
                                     PIN_MODE_ANALOG(GPIOC_ADC3_MOT4_PH1) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC3_MOT4_PH2) | \
                                     PIN_MODE_ANALOG(GPIOC_ADC1_MOT234_PH2_C) |\
                                     PIN_MODE_ANALOG(GPIOC_ADC1_MOT234_PH3_C) |\
                                     PIN_MODE_ALTERNATE(GPIOC_OUT_MOT4_PH1_P) |\
                                     PIN_MODE_ALTERNATE(GPIOC_OUT_MOT4_PH2_P) |\
                                     PIN_MODE_ALTERNATE(GPIOC_OUT_MOT4_PH3_P) |\
                                     PIN_MODE_ALTERNATE(GPIOC_OUT_MOT3_PH1_N) |\
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_SCK) |   \
                                     PIN_MODE_ALTERNATE(GPIOC_SPI3_MISO) |  \
                                     PIN_MODE_INPUT(GPIOC_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOC_PIN15))
#define VAL_GPIOC_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOC_ADC3_MOT3_PH3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC3_MOT3_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC3_MOT4_PH1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC3_MOT4_PH2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC1_MOT234_PH2_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_ADC1_MOT234_PH3_C) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OUT_MOT4_PH1_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OUT_MOT4_PH2_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OUT_MOT4_PH3_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_OUT_MOT3_PH1_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_SCK) |   \
                                     PIN_OTYPE_PUSHPULL(GPIOC_SPI3_MISO) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOC_PIN15))
#define VAL_GPIOC_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOC_ADC3_MOT3_PH3) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_ADC3_MOT3_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_ADC3_MOT4_PH1) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_ADC3_MOT4_PH2) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_ADC1_MOT234_PH2_C) |\
                                     PIN_OSPEED_VERYLOW(GPIOC_ADC1_MOT234_PH3_C) |\
                                     PIN_OSPEED_HIGH(GPIOC_OUT_MOT4_PH1_P) |\
                                     PIN_OSPEED_HIGH(GPIOC_OUT_MOT4_PH2_P) |\
                                     PIN_OSPEED_HIGH(GPIOC_OUT_MOT4_PH3_P) |\
                                     PIN_OSPEED_HIGH(GPIOC_OUT_MOT3_PH1_N) |\
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_SCK) |      \
                                     PIN_OSPEED_HIGH(GPIOC_SPI3_MISO) |     \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOC_PIN15))
#define VAL_GPIOC_PUPDR             (PIN_PUPDR_FLOATING(GPIOC_ADC3_MOT3_PH3) |\
                                     PIN_PUPDR_FLOATING(GPIOC_ADC3_MOT3_N) |\
                                     PIN_PUPDR_FLOATING(GPIOC_ADC3_MOT4_PH1) |\
                                     PIN_PUPDR_FLOATING(GPIOC_ADC3_MOT4_PH2) |\
                                     PIN_PUPDR_FLOATING(GPIOC_ADC1_MOT234_PH2_C) |\
                                     PIN_PUPDR_FLOATING(GPIOC_ADC1_MOT234_PH3_C) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OUT_MOT4_PH1_P) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OUT_MOT4_PH2_P) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OUT_MOT4_PH3_P) |\
                                     PIN_PUPDR_FLOATING(GPIOC_OUT_MOT3_PH1_N) |\
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_SCK) |   \
                                     PIN_PUPDR_FLOATING(GPIOC_SPI3_MISO) |  \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOC_PIN15))
#define VAL_GPIOC_ODR               (PIN_ODR_HIGH(GPIOC_ADC3_MOT3_PH3) |    \
                                     PIN_ODR_HIGH(GPIOC_ADC3_MOT3_N) |      \
                                     PIN_ODR_HIGH(GPIOC_ADC3_MOT4_PH1) |    \
                                     PIN_ODR_HIGH(GPIOC_ADC3_MOT4_PH2) |    \
                                     PIN_ODR_HIGH(GPIOC_ADC1_MOT234_PH2_C) |\
                                     PIN_ODR_HIGH(GPIOC_ADC1_MOT234_PH3_C) |\
                                     PIN_ODR_LOW(GPIOC_OUT_MOT4_PH1_P) |    \
                                     PIN_ODR_LOW(GPIOC_OUT_MOT4_PH2_P) |    \
                                     PIN_ODR_LOW(GPIOC_OUT_MOT4_PH3_P) |    \
                                     PIN_ODR_LOW(GPIOC_OUT_MOT3_PH1_N) |    \
                                     PIN_ODR_HIGH(GPIOC_SPI3_SCK) |         \
                                     PIN_ODR_HIGH(GPIOC_SPI3_MISO) |        \
                                     PIN_ODR_HIGH(GPIOC_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOC_PIN15))
#define VAL_GPIOC_AFRL              (PIN_AFIO_AF(GPIOC_ADC3_MOT3_PH3, 0U) | \
                                     PIN_AFIO_AF(GPIOC_ADC3_MOT3_N, 0U) |   \
                                     PIN_AFIO_AF(GPIOC_ADC3_MOT4_PH1, 0U) | \
                                     PIN_AFIO_AF(GPIOC_ADC3_MOT4_PH2, 0U) | \
                                     PIN_AFIO_AF(GPIOC_ADC1_MOT234_PH2_C, 0U) |\
                                     PIN_AFIO_AF(GPIOC_ADC1_MOT234_PH3_C, 0U) |\
                                     PIN_AFIO_AF(GPIOC_OUT_MOT4_PH1_P, 3U) |\
                                     PIN_AFIO_AF(GPIOC_OUT_MOT4_PH2_P, 3U))
#define VAL_GPIOC_AFRH              (PIN_AFIO_AF(GPIOC_OUT_MOT4_PH3_P, 3U) |\
                                     PIN_AFIO_AF(GPIOC_OUT_MOT3_PH1_N, 2U) |\
                                     PIN_AFIO_AF(GPIOC_SPI3_SCK, 6U) |      \
                                     PIN_AFIO_AF(GPIOC_SPI3_MISO, 6U) |     \
                                     PIN_AFIO_AF(GPIOC_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOC_PIN15, 0U))

/*
 * GPIOD setup:
 *
 * PD0  - PIN0                      (input pullup).
 * PD1  - PIN1                      (input pullup).
 * PD2  - PIN2                      (input pullup).
 * PD3  - INT_ENCODERS_n            (input floating).
 * PD4  - CS_DRIVER_1_n             (output pushpull maximum).
 * PD5  - CS_ENCODERS_n             (output pushpull maximum).
 * PD6  - SPI3_MOSI                 (alternate 5).
 * PD7  - CS_DRIVER_2_n             (output pushpull maximum).
 * PD8  - PIN8                      (input pullup).
 * PD9  - PIN9                      (input pullup).
 * PD10 - CS_DRIVER_3_n             (output pushpull maximum).
 * PD11 - CS_DRIVER_4_n             (output pushpull maximum).
 * PD12 - OUT_MOT3_PH2_P            (alternate 2).
 * PD13 - OUT_MOT3_PH2_N            (alternate 2).
 * PD14 - OUT_MOT3_PH3_P            (alternate 2).
 * PD15 - PIN15                     (input pullup).
 */
#define VAL_GPIOD_MODER             (PIN_MODE_INPUT(GPIOD_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOD_INT_ENCODERS_n) | \
                                     PIN_MODE_OUTPUT(GPIOD_CS_DRIVER_1_n) | \
                                     PIN_MODE_OUTPUT(GPIOD_CS_ENCODERS_n) | \
                                     PIN_MODE_ALTERNATE(GPIOD_SPI3_MOSI) |  \
                                     PIN_MODE_OUTPUT(GPIOD_CS_DRIVER_2_n) | \
                                     PIN_MODE_INPUT(GPIOD_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOD_PIN9) |           \
                                     PIN_MODE_OUTPUT(GPIOD_CS_DRIVER_3_n) | \
                                     PIN_MODE_OUTPUT(GPIOD_CS_DRIVER_4_n) | \
                                     PIN_MODE_ALTERNATE(GPIOD_OUT_MOT3_PH2_P) |\
                                     PIN_MODE_ALTERNATE(GPIOD_OUT_MOT3_PH2_N) |\
                                     PIN_MODE_ALTERNATE(GPIOD_OUT_MOT3_PH3_P) |\
                                     PIN_MODE_INPUT(GPIOD_PIN15))
#define VAL_GPIOD_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOD_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_INT_ENCODERS_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_CS_DRIVER_1_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_CS_ENCODERS_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_SPI3_MOSI) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CS_DRIVER_2_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOD_CS_DRIVER_3_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_CS_DRIVER_4_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_OUT_MOT3_PH2_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_OUT_MOT3_PH2_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_OUT_MOT3_PH3_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOD_PIN15))
#define VAL_GPIOD_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOD_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_INT_ENCODERS_n) |\
                                     PIN_OSPEED_HIGH(GPIOD_CS_DRIVER_1_n) | \
                                     PIN_OSPEED_HIGH(GPIOD_CS_ENCODERS_n) | \
                                     PIN_OSPEED_HIGH(GPIOD_SPI3_MOSI) |     \
                                     PIN_OSPEED_HIGH(GPIOD_CS_DRIVER_2_n) | \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN9) |       \
                                     PIN_OSPEED_HIGH(GPIOD_CS_DRIVER_3_n) | \
                                     PIN_OSPEED_HIGH(GPIOD_CS_DRIVER_4_n) | \
                                     PIN_OSPEED_HIGH(GPIOD_OUT_MOT3_PH2_P) |\
                                     PIN_OSPEED_HIGH(GPIOD_OUT_MOT3_PH2_N) |\
                                     PIN_OSPEED_HIGH(GPIOD_OUT_MOT3_PH3_P) |\
                                     PIN_OSPEED_VERYLOW(GPIOD_PIN15))
#define VAL_GPIOD_PUPDR             (PIN_PUPDR_PULLUP(GPIOD_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_INT_ENCODERS_n) |\
                                     PIN_PUPDR_FLOATING(GPIOD_CS_DRIVER_1_n) |\
                                     PIN_PUPDR_FLOATING(GPIOD_CS_ENCODERS_n) |\
                                     PIN_PUPDR_FLOATING(GPIOD_SPI3_MOSI) |  \
                                     PIN_PUPDR_FLOATING(GPIOD_CS_DRIVER_2_n) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOD_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOD_CS_DRIVER_3_n) |\
                                     PIN_PUPDR_FLOATING(GPIOD_CS_DRIVER_4_n) |\
                                     PIN_PUPDR_FLOATING(GPIOD_OUT_MOT3_PH2_P) |\
                                     PIN_PUPDR_FLOATING(GPIOD_OUT_MOT3_PH2_N) |\
                                     PIN_PUPDR_FLOATING(GPIOD_OUT_MOT3_PH3_P) |\
                                     PIN_PUPDR_PULLUP(GPIOD_PIN15))
#define VAL_GPIOD_ODR               (PIN_ODR_HIGH(GPIOD_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOD_INT_ENCODERS_n) |   \
                                     PIN_ODR_HIGH(GPIOD_CS_DRIVER_1_n) |    \
                                     PIN_ODR_HIGH(GPIOD_CS_ENCODERS_n) |    \
                                     PIN_ODR_HIGH(GPIOD_SPI3_MOSI) |        \
                                     PIN_ODR_HIGH(GPIOD_CS_DRIVER_2_n) |    \
                                     PIN_ODR_HIGH(GPIOD_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOD_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOD_CS_DRIVER_3_n) |    \
                                     PIN_ODR_HIGH(GPIOD_CS_DRIVER_4_n) |    \
                                     PIN_ODR_LOW(GPIOD_OUT_MOT3_PH2_P) |    \
                                     PIN_ODR_LOW(GPIOD_OUT_MOT3_PH2_N) |    \
                                     PIN_ODR_LOW(GPIOD_OUT_MOT3_PH3_P) |    \
                                     PIN_ODR_HIGH(GPIOD_PIN15))
#define VAL_GPIOD_AFRL              (PIN_AFIO_AF(GPIOD_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_INT_ENCODERS_n, 0U) |\
                                     PIN_AFIO_AF(GPIOD_CS_DRIVER_1_n, 0U) | \
                                     PIN_AFIO_AF(GPIOD_CS_ENCODERS_n, 0U) | \
                                     PIN_AFIO_AF(GPIOD_SPI3_MOSI, 5U) |     \
                                     PIN_AFIO_AF(GPIOD_CS_DRIVER_2_n, 0U))
#define VAL_GPIOD_AFRH              (PIN_AFIO_AF(GPIOD_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOD_CS_DRIVER_3_n, 0U) | \
                                     PIN_AFIO_AF(GPIOD_CS_DRIVER_4_n, 0U) | \
                                     PIN_AFIO_AF(GPIOD_OUT_MOT3_PH2_P, 2U) |\
                                     PIN_AFIO_AF(GPIOD_OUT_MOT3_PH2_N, 2U) |\
                                     PIN_AFIO_AF(GPIOD_OUT_MOT3_PH3_P, 2U) |\
                                     PIN_AFIO_AF(GPIOD_PIN15, 0U))

/*
 * GPIOE setup:
 *
 * PE0  - PIN0                      (input pullup).
 * PE1  - PIN1                      (input pullup).
 * PE2  - PIN2                      (input pullup).
 * PE3  - PIN3                      (input pullup).
 * PE4  - PIN4                      (input pullup).
 * PE5  - SWCLK_OUT                 (output pushpull maximum).
 * PE6  - SWDIO_OUT                 (output pushpull maximum).
 * PE7  - PIN7                      (input pullup).
 * PE8  - PIN8                      (input pullup).
 * PE9  - PWR_ON_BTN_STATE_n        (input pullup).
 * PE10 - OUT_MOT1_PH2_N            (alternate 1).
 * PE11 - OUT_MOT1_PH2_P            (alternate 1).
 * PE12 - OUT_MOT1_PH3_N            (alternate 1).
 * PE13 - EN_DRIVER_2               (output pushpull minimum).
 * PE14 - EN_DRIVER_3               (output pushpull minimum).
 * PE15 - EN_DRIVER_4               (output pushpull minimum).
 */
#define VAL_GPIOE_MODER             (PIN_MODE_INPUT(GPIOE_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN4) |           \
                                     PIN_MODE_OUTPUT(GPIOE_SWCLK_OUT) |     \
                                     PIN_MODE_OUTPUT(GPIOE_SWDIO_OUT) |     \
                                     PIN_MODE_INPUT(GPIOE_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOE_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOE_PWR_ON_BTN_STATE_n) |\
                                     PIN_MODE_ALTERNATE(GPIOE_OUT_MOT1_PH2_N) |\
                                     PIN_MODE_ALTERNATE(GPIOE_OUT_MOT1_PH2_P) |\
                                     PIN_MODE_ALTERNATE(GPIOE_OUT_MOT1_PH3_N) |\
                                     PIN_MODE_OUTPUT(GPIOE_EN_DRIVER_2) |   \
                                     PIN_MODE_OUTPUT(GPIOE_EN_DRIVER_3) |   \
                                     PIN_MODE_OUTPUT(GPIOE_EN_DRIVER_4))
#define VAL_GPIOE_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOE_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SWCLK_OUT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_SWDIO_OUT) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOE_PWR_ON_BTN_STATE_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_OUT_MOT1_PH2_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_OUT_MOT1_PH2_P) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_OUT_MOT1_PH3_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_EN_DRIVER_2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_EN_DRIVER_3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOE_EN_DRIVER_4))
#define VAL_GPIOE_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOE_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN4) |       \
                                     PIN_OSPEED_HIGH(GPIOE_SWCLK_OUT) |     \
                                     PIN_OSPEED_HIGH(GPIOE_SWDIO_OUT) |     \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOE_PWR_ON_BTN_STATE_n) |\
                                     PIN_OSPEED_HIGH(GPIOE_OUT_MOT1_PH2_N) |\
                                     PIN_OSPEED_HIGH(GPIOE_OUT_MOT1_PH2_P) |\
                                     PIN_OSPEED_HIGH(GPIOE_OUT_MOT1_PH3_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOE_EN_DRIVER_2) |\
                                     PIN_OSPEED_VERYLOW(GPIOE_EN_DRIVER_3) |\
                                     PIN_OSPEED_VERYLOW(GPIOE_EN_DRIVER_4))
#define VAL_GPIOE_PUPDR             (PIN_PUPDR_PULLUP(GPIOE_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN4) |         \
                                     PIN_PUPDR_FLOATING(GPIOE_SWCLK_OUT) |  \
                                     PIN_PUPDR_FLOATING(GPIOE_SWDIO_OUT) |  \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOE_PWR_ON_BTN_STATE_n) |\
                                     PIN_PUPDR_FLOATING(GPIOE_OUT_MOT1_PH2_N) |\
                                     PIN_PUPDR_FLOATING(GPIOE_OUT_MOT1_PH2_P) |\
                                     PIN_PUPDR_FLOATING(GPIOE_OUT_MOT1_PH3_N) |\
                                     PIN_PUPDR_FLOATING(GPIOE_EN_DRIVER_2) |\
                                     PIN_PUPDR_FLOATING(GPIOE_EN_DRIVER_3) |\
                                     PIN_PUPDR_FLOATING(GPIOE_EN_DRIVER_4))
#define VAL_GPIOE_ODR               (PIN_ODR_HIGH(GPIOE_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOE_SWCLK_OUT) |        \
                                     PIN_ODR_HIGH(GPIOE_SWDIO_OUT) |        \
                                     PIN_ODR_HIGH(GPIOE_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOE_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOE_PWR_ON_BTN_STATE_n) |\
                                     PIN_ODR_LOW(GPIOE_OUT_MOT1_PH2_N) |    \
                                     PIN_ODR_LOW(GPIOE_OUT_MOT1_PH2_P) |    \
                                     PIN_ODR_LOW(GPIOE_OUT_MOT1_PH3_N) |    \
                                     PIN_ODR_LOW(GPIOE_EN_DRIVER_2) |       \
                                     PIN_ODR_LOW(GPIOE_EN_DRIVER_3) |       \
                                     PIN_ODR_LOW(GPIOE_EN_DRIVER_4))
#define VAL_GPIOE_AFRL              (PIN_AFIO_AF(GPIOE_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_SWCLK_OUT, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_SWDIO_OUT, 0U) |     \
                                     PIN_AFIO_AF(GPIOE_PIN7, 0U))
#define VAL_GPIOE_AFRH              (PIN_AFIO_AF(GPIOE_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOE_PWR_ON_BTN_STATE_n, 0U) |\
                                     PIN_AFIO_AF(GPIOE_OUT_MOT1_PH2_N, 1U) |\
                                     PIN_AFIO_AF(GPIOE_OUT_MOT1_PH2_P, 1U) |\
                                     PIN_AFIO_AF(GPIOE_OUT_MOT1_PH3_N, 1U) |\
                                     PIN_AFIO_AF(GPIOE_EN_DRIVER_2, 0U) |   \
                                     PIN_AFIO_AF(GPIOE_EN_DRIVER_3, 0U) |   \
                                     PIN_AFIO_AF(GPIOE_EN_DRIVER_4, 0U))

/*
 * GPIOF setup:
 *
 * PF0  - I2C2_SDA                  (alternate 4).
 * PF1  - I2C2_SCL                  (alternate 4).
 * PF2  - PIN2                      (input pullup).
 * PF3  - ADC3_MOT3_PH2             (analog).
 * PF4  - ADC3_MOT4_PH3             (analog).
 * PF5  - ADC3_MOT4_N               (analog).
 * PF6  - ADC3_MOT2_PH1             (analog).
 * PF7  - ADC3_MOT2_PH2             (analog).
 * PF8  - ADC3_MOT2_PH3             (analog).
 * PF9  - ADC3_MOT2_N               (analog).
 * PF10 - ADC3_MOT3_PH1             (analog).
 * PF11 - PIN11                     (input pullup).
 * PF12 - FAULT_DRIVER_1_n          (input pullup).
 * PF13 - FAULT_DRIVER_2_n          (input pullup).
 * PF14 - FAULT_DRIVER_3_n          (input pullup).
 * PF15 - FAULT_DRIVER_4_n          (input pullup).
 */
#define VAL_GPIOF_MODER             (PIN_MODE_ALTERNATE(GPIOF_I2C2_SDA) |   \
                                     PIN_MODE_ALTERNATE(GPIOF_I2C2_SCL) |   \
                                     PIN_MODE_INPUT(GPIOF_PIN2) |           \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT3_PH2) | \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT4_PH3) | \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT4_N) |   \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT2_PH1) | \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT2_PH2) | \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT2_PH3) | \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT2_N) |   \
                                     PIN_MODE_ANALOG(GPIOF_ADC3_MOT3_PH1) | \
                                     PIN_MODE_INPUT(GPIOF_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOF_FAULT_DRIVER_1_n) |\
                                     PIN_MODE_INPUT(GPIOF_FAULT_DRIVER_2_n) |\
                                     PIN_MODE_INPUT(GPIOF_FAULT_DRIVER_3_n) |\
                                     PIN_MODE_INPUT(GPIOF_FAULT_DRIVER_4_n))
#define VAL_GPIOF_OTYPER            (PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SDA) |  \
                                     PIN_OTYPE_OPENDRAIN(GPIOF_I2C2_SCL) |  \
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT3_PH2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT4_PH3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT4_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT2_PH1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT2_PH2) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT2_PH3) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT2_N) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_ADC3_MOT3_PH1) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOF_FAULT_DRIVER_1_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_FAULT_DRIVER_2_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_FAULT_DRIVER_3_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOF_FAULT_DRIVER_4_n))
#define VAL_GPIOF_OSPEEDR           (PIN_OSPEED_HIGH(GPIOF_I2C2_SDA) |      \
                                     PIN_OSPEED_HIGH(GPIOF_I2C2_SCL) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT3_PH2) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT4_PH3) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT4_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT2_PH1) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT2_PH2) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT2_PH3) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT2_N) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_ADC3_MOT3_PH1) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOF_FAULT_DRIVER_1_n) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_FAULT_DRIVER_2_n) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_FAULT_DRIVER_3_n) |\
                                     PIN_OSPEED_VERYLOW(GPIOF_FAULT_DRIVER_4_n))
#define VAL_GPIOF_PUPDR             (PIN_PUPDR_FLOATING(GPIOF_I2C2_SDA) |   \
                                     PIN_PUPDR_FLOATING(GPIOF_I2C2_SCL) |   \
                                     PIN_PUPDR_PULLUP(GPIOF_PIN2) |         \
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT3_PH2) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT4_PH3) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT4_N) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT2_PH1) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT2_PH2) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT2_PH3) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT2_N) |\
                                     PIN_PUPDR_FLOATING(GPIOF_ADC3_MOT3_PH1) |\
                                     PIN_PUPDR_PULLUP(GPIOF_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOF_FAULT_DRIVER_1_n) |\
                                     PIN_PUPDR_PULLUP(GPIOF_FAULT_DRIVER_2_n) |\
                                     PIN_PUPDR_PULLUP(GPIOF_FAULT_DRIVER_3_n) |\
                                     PIN_PUPDR_PULLUP(GPIOF_FAULT_DRIVER_4_n))
#define VAL_GPIOF_ODR               (PIN_ODR_HIGH(GPIOF_I2C2_SDA) |         \
                                     PIN_ODR_HIGH(GPIOF_I2C2_SCL) |         \
                                     PIN_ODR_HIGH(GPIOF_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT3_PH2) |    \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT4_PH3) |    \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT4_N) |      \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT2_PH1) |    \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT2_PH2) |    \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT2_PH3) |    \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT2_N) |      \
                                     PIN_ODR_HIGH(GPIOF_ADC3_MOT3_PH1) |    \
                                     PIN_ODR_HIGH(GPIOF_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOF_FAULT_DRIVER_1_n) | \
                                     PIN_ODR_HIGH(GPIOF_FAULT_DRIVER_2_n) | \
                                     PIN_ODR_HIGH(GPIOF_FAULT_DRIVER_3_n) | \
                                     PIN_ODR_HIGH(GPIOF_FAULT_DRIVER_4_n))
#define VAL_GPIOF_AFRL              (PIN_AFIO_AF(GPIOF_I2C2_SDA, 4U) |      \
                                     PIN_AFIO_AF(GPIOF_I2C2_SCL, 4U) |      \
                                     PIN_AFIO_AF(GPIOF_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT3_PH2, 0U) | \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT4_PH3, 0U) | \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT4_N, 0U) |   \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT2_PH1, 0U) | \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT2_PH2, 0U))
#define VAL_GPIOF_AFRH              (PIN_AFIO_AF(GPIOF_ADC3_MOT2_PH3, 0U) | \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT2_N, 0U) |   \
                                     PIN_AFIO_AF(GPIOF_ADC3_MOT3_PH1, 0U) | \
                                     PIN_AFIO_AF(GPIOF_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOF_FAULT_DRIVER_1_n, 0U) |\
                                     PIN_AFIO_AF(GPIOF_FAULT_DRIVER_2_n, 0U) |\
                                     PIN_AFIO_AF(GPIOF_FAULT_DRIVER_3_n, 0U) |\
                                     PIN_AFIO_AF(GPIOF_FAULT_DRIVER_4_n, 0U))

/*
 * GPIOG setup:
 *
 * PG0  - STATUS_LED1_RED           (output pushpull maximum).
 * PG1  - STATUS_LED1_GREEN         (output pushpull maximum).
 * PG2  - STATUS_LED1_BLUE          (output pushpull maximum).
 * PG3  - STATUS_LED2_RED           (output pushpull maximum).
 * PG4  - STATUS_LED2_GREEN         (output pushpull maximum).
 * PG5  - STATUS_LED2_BLUE          (output pushpull maximum).
 * PG6  - STATUS_LED3_RED           (output pushpull maximum).
 * PG7  - STATUS_LED3_GREEN         (output pushpull maximum).
 * PG8  - STATUS_LED3_BLUE          (output pushpull maximum).
 * PG9  - PIN9                      (input pullup).
 * PG10 - PWR_PP_STATE              (input floating).
 * PG11 - INT_PD_CTRL_n             (input pullup).
 * PG12 - PWR_ON                    (output pushpull minimum).
 * PG13 - PIN13                     (input pullup).
 * PG14 - PIN14                     (input pullup).
 * PG15 - PIN15                     (input pullup).
 */
#define VAL_GPIOG_MODER             (PIN_MODE_OUTPUT(GPIOG_STATUS_LED1_RED) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED1_GREEN) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED1_BLUE) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED2_RED) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED2_GREEN) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED2_BLUE) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED3_RED) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED3_GREEN) |\
                                     PIN_MODE_OUTPUT(GPIOG_STATUS_LED3_BLUE) |\
                                     PIN_MODE_INPUT(GPIOG_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOG_PWR_PP_STATE) |   \
                                     PIN_MODE_INPUT(GPIOG_INT_PD_CTRL_n) |  \
                                     PIN_MODE_OUTPUT(GPIOG_PWR_ON) |        \
                                     PIN_MODE_INPUT(GPIOG_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOG_PIN15))
#define VAL_GPIOG_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED1_RED) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED1_GREEN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED1_BLUE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED2_RED) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED2_GREEN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED2_BLUE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED3_RED) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED3_GREEN) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_STATUS_LED3_BLUE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PWR_PP_STATE) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_INT_PD_CTRL_n) |\
                                     PIN_OTYPE_PUSHPULL(GPIOG_PWR_ON) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOG_PIN15))
#define VAL_GPIOG_OSPEEDR           (PIN_OSPEED_HIGH(GPIOG_STATUS_LED1_RED) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED1_GREEN) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED1_BLUE) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED2_RED) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED2_GREEN) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED2_BLUE) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED3_RED) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED3_GREEN) |\
                                     PIN_OSPEED_HIGH(GPIOG_STATUS_LED3_BLUE) |\
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOG_PWR_PP_STATE) |\
                                     PIN_OSPEED_VERYLOW(GPIOG_INT_PD_CTRL_n) |\
                                     PIN_OSPEED_VERYLOW(GPIOG_PWR_ON) |     \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOG_PIN15))
#define VAL_GPIOG_PUPDR             (PIN_PUPDR_FLOATING(GPIOG_STATUS_LED1_RED) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED1_GREEN) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED1_BLUE) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED2_RED) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED2_GREEN) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED2_BLUE) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED3_RED) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED3_GREEN) |\
                                     PIN_PUPDR_FLOATING(GPIOG_STATUS_LED3_BLUE) |\
                                     PIN_PUPDR_PULLUP(GPIOG_PIN9) |         \
                                     PIN_PUPDR_FLOATING(GPIOG_PWR_PP_STATE) |\
                                     PIN_PUPDR_PULLUP(GPIOG_INT_PD_CTRL_n) |\
                                     PIN_PUPDR_FLOATING(GPIOG_PWR_ON) |     \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOG_PIN15))
#define VAL_GPIOG_ODR               (PIN_ODR_HIGH(GPIOG_STATUS_LED1_RED) |  \
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED1_GREEN) |\
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED1_BLUE) | \
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED2_RED) |  \
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED2_GREEN) |\
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED2_BLUE) | \
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED3_RED) |  \
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED3_GREEN) |\
                                     PIN_ODR_HIGH(GPIOG_STATUS_LED3_BLUE) | \
                                     PIN_ODR_HIGH(GPIOG_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOG_PWR_PP_STATE) |     \
                                     PIN_ODR_HIGH(GPIOG_INT_PD_CTRL_n) |    \
                                     PIN_ODR_LOW(GPIOG_PWR_ON) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOG_PIN15))
#define VAL_GPIOG_AFRL              (PIN_AFIO_AF(GPIOG_STATUS_LED1_RED, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED1_GREEN, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED1_BLUE, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED2_RED, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED2_GREEN, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED2_BLUE, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED3_RED, 0U) |\
                                     PIN_AFIO_AF(GPIOG_STATUS_LED3_GREEN, 0U))
#define VAL_GPIOG_AFRH              (PIN_AFIO_AF(GPIOG_STATUS_LED3_BLUE, 0U) |\
                                     PIN_AFIO_AF(GPIOG_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOG_PWR_PP_STATE, 0U) |  \
                                     PIN_AFIO_AF(GPIOG_INT_PD_CTRL_n, 0U) | \
                                     PIN_AFIO_AF(GPIOG_PWR_ON, 0U) |        \
                                     PIN_AFIO_AF(GPIOG_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOG_PIN15, 0U))

/*
 * GPIOH setup:
 *
 * PH0  - OSC_IN                    (input floating).
 * PH1  - OSC_OUT                   (input floating).
 * PH2  - PIN2                      (input pullup).
 * PH3  - PIN3                      (input pullup).
 * PH4  - PIN4                      (input pullup).
 * PH5  - PIN5                      (input pullup).
 * PH6  - PIN6                      (input pullup).
 * PH7  - PIN7                      (input pullup).
 * PH8  - PIN8                      (input pullup).
 * PH9  - PIN9                      (input pullup).
 * PH10 - PIN10                     (input pullup).
 * PH11 - PIN11                     (input pullup).
 * PH12 - PIN12                     (input pullup).
 * PH13 - PIN13                     (input pullup).
 * PH14 - PIN14                     (input pullup).
 * PH15 - PIN15                     (input pullup).
 */
#define VAL_GPIOH_MODER             (PIN_MODE_INPUT(GPIOH_OSC_IN) |         \
                                     PIN_MODE_INPUT(GPIOH_OSC_OUT) |        \
                                     PIN_MODE_INPUT(GPIOH_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOH_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOH_PIN15))
#define VAL_GPIOH_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOH_OSC_IN) |     \
                                     PIN_OTYPE_PUSHPULL(GPIOH_OSC_OUT) |    \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOH_PIN15))
#define VAL_GPIOH_OSPEEDR           (PIN_OSPEED_HIGH(GPIOH_OSC_IN) |        \
                                     PIN_OSPEED_HIGH(GPIOH_OSC_OUT) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOH_PIN15))
#define VAL_GPIOH_PUPDR             (PIN_PUPDR_FLOATING(GPIOH_OSC_IN) |     \
                                     PIN_PUPDR_FLOATING(GPIOH_OSC_OUT) |    \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOH_PIN15))
#define VAL_GPIOH_ODR               (PIN_ODR_HIGH(GPIOH_OSC_IN) |           \
                                     PIN_ODR_HIGH(GPIOH_OSC_OUT) |          \
                                     PIN_ODR_HIGH(GPIOH_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOH_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOH_PIN15))
#define VAL_GPIOH_AFRL              (PIN_AFIO_AF(GPIOH_OSC_IN, 0U) |        \
                                     PIN_AFIO_AF(GPIOH_OSC_OUT, 0U) |       \
                                     PIN_AFIO_AF(GPIOH_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN7, 0U))
#define VAL_GPIOH_AFRH              (PIN_AFIO_AF(GPIOH_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOH_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOH_PIN15, 0U))

/*
 * GPIOI setup:
 *
 * PI0  - PIN0                      (input pullup).
 * PI1  - PIN1                      (input pullup).
 * PI2  - PIN2                      (input pullup).
 * PI3  - PIN3                      (input pullup).
 * PI4  - PIN4                      (input pullup).
 * PI5  - PIN5                      (input pullup).
 * PI6  - PIN6                      (input pullup).
 * PI7  - PIN7                      (input pullup).
 * PI8  - PIN8                      (input pullup).
 * PI9  - PIN9                      (input pullup).
 * PI10 - PIN10                     (input pullup).
 * PI11 - PIN11                     (input pullup).
 * PI12 - PIN12                     (input pullup).
 * PI13 - PIN13                     (input pullup).
 * PI14 - PIN14                     (input pullup).
 * PI15 - PIN15                     (input pullup).
 */
#define VAL_GPIOI_MODER             (PIN_MODE_INPUT(GPIOI_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOI_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOI_PIN15))
#define VAL_GPIOI_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOI_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOI_PIN15))
#define VAL_GPIOI_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOI_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOI_PIN15))
#define VAL_GPIOI_PUPDR             (PIN_PUPDR_PULLUP(GPIOI_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOI_PIN15))
#define VAL_GPIOI_ODR               (PIN_ODR_HIGH(GPIOI_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOI_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOI_PIN15))
#define VAL_GPIOI_AFRL              (PIN_AFIO_AF(GPIOI_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN7, 0U))
#define VAL_GPIOI_AFRH              (PIN_AFIO_AF(GPIOI_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOI_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOI_PIN15, 0U))

/*
 * GPIOJ setup:
 *
 * PJ0  - PIN0                      (input pullup).
 * PJ1  - PIN1                      (input pullup).
 * PJ2  - PIN2                      (input pullup).
 * PJ3  - PIN3                      (input pullup).
 * PJ4  - PIN4                      (input pullup).
 * PJ5  - PIN5                      (input pullup).
 * PJ6  - PIN6                      (input pullup).
 * PJ7  - PIN7                      (input pullup).
 * PJ8  - PIN8                      (input pullup).
 * PJ9  - PIN9                      (input pullup).
 * PJ10 - PIN10                     (input pullup).
 * PJ11 - PIN11                     (input pullup).
 * PJ12 - PIN12                     (input pullup).
 * PJ13 - PIN13                     (input pullup).
 * PJ14 - PIN14                     (input pullup).
 * PJ15 - PIN15                     (input pullup).
 */
#define VAL_GPIOJ_MODER             (PIN_MODE_INPUT(GPIOJ_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOJ_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOJ_PIN15))
#define VAL_GPIOJ_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOJ_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOJ_PIN15))
#define VAL_GPIOJ_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOJ_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOJ_PIN15))
#define VAL_GPIOJ_PUPDR             (PIN_PUPDR_PULLUP(GPIOJ_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOJ_PIN15))
#define VAL_GPIOJ_ODR               (PIN_ODR_HIGH(GPIOJ_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOJ_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOJ_PIN15))
#define VAL_GPIOJ_AFRL              (PIN_AFIO_AF(GPIOJ_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN7, 0U))
#define VAL_GPIOJ_AFRH              (PIN_AFIO_AF(GPIOJ_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOJ_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOJ_PIN15, 0U))

/*
 * GPIOK setup:
 *
 * PK0  - PIN0                      (input pullup).
 * PK1  - PIN1                      (input pullup).
 * PK2  - PIN2                      (input pullup).
 * PK3  - PIN3                      (input pullup).
 * PK4  - PIN4                      (input pullup).
 * PK5  - PIN5                      (input pullup).
 * PK6  - PIN6                      (input pullup).
 * PK7  - PIN7                      (input pullup).
 * PK8  - PIN8                      (input pullup).
 * PK9  - PIN9                      (input pullup).
 * PK10 - PIN10                     (input pullup).
 * PK11 - PIN11                     (input pullup).
 * PK12 - PIN12                     (input pullup).
 * PK13 - PIN13                     (input pullup).
 * PK14 - PIN14                     (input pullup).
 * PK15 - PIN15                     (input pullup).
 */
#define VAL_GPIOK_MODER             (PIN_MODE_INPUT(GPIOK_PIN0) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN1) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN2) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN3) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN4) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN5) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN6) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN7) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN8) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN9) |           \
                                     PIN_MODE_INPUT(GPIOK_PIN10) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN11) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN12) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN13) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN14) |          \
                                     PIN_MODE_INPUT(GPIOK_PIN15))
#define VAL_GPIOK_OTYPER            (PIN_OTYPE_PUSHPULL(GPIOK_PIN0) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN1) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN2) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN3) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN4) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN5) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN6) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN7) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN8) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN9) |       \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN10) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN11) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN12) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN13) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN14) |      \
                                     PIN_OTYPE_PUSHPULL(GPIOK_PIN15))
#define VAL_GPIOK_OSPEEDR           (PIN_OSPEED_VERYLOW(GPIOK_PIN0) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN1) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN2) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN3) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN4) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN5) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN6) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN7) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN8) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN9) |       \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN10) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN11) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN12) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN13) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN14) |      \
                                     PIN_OSPEED_VERYLOW(GPIOK_PIN15))
#define VAL_GPIOK_PUPDR             (PIN_PUPDR_PULLUP(GPIOK_PIN0) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN1) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN2) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN3) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN4) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN5) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN6) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN7) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN8) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN9) |         \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN10) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN11) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN12) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN13) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN14) |        \
                                     PIN_PUPDR_PULLUP(GPIOK_PIN15))
#define VAL_GPIOK_ODR               (PIN_ODR_HIGH(GPIOK_PIN0) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN1) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN2) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN3) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN4) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN5) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN6) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN7) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN8) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN9) |             \
                                     PIN_ODR_HIGH(GPIOK_PIN10) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN11) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN12) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN13) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN14) |            \
                                     PIN_ODR_HIGH(GPIOK_PIN15))
#define VAL_GPIOK_AFRL              (PIN_AFIO_AF(GPIOK_PIN0, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN1, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN2, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN3, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN4, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN5, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN6, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN7, 0U))
#define VAL_GPIOK_AFRH              (PIN_AFIO_AF(GPIOK_PIN8, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN9, 0U) |          \
                                     PIN_AFIO_AF(GPIOK_PIN10, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN11, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN12, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN13, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN14, 0U) |         \
                                     PIN_AFIO_AF(GPIOK_PIN15, 0U))


#if !defined(_FROM_ASM_)
#ifdef __cplusplus
extern "C" {
#endif
  void boardInit(void);
#ifdef __cplusplus
}
#endif
#endif /* _FROM_ASM_ */

#endif /* BOARD_H */
