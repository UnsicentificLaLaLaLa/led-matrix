/*
 * Copyright 2023 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */


#ifndef __MM32G0001A_H__
#define __MM32G0001A_H__

#ifdef __cplusplus
extern "C" {
#endif

/* -----------------------------------------------------------------------------
  -- Interrupt vector numbers
  ----------------------------------------------------------------------------- */

/*!
* @addtogroup Interrupt_vector_numbers Interrupt vector numbers

* @{
*/

/** Interrupt Number Definitions */

#define NUMBER_OF_INT_VECTORS 26                 /**< Number of interrupts in the Vector table */

typedef enum IRQ {
    Reset_IRQn                          = -15,                                   /*!< reset */
    NMI_IRQn                            = -14,                                   /*!< Non-Maskable Interrupt/RCC Clock Security System (CSS) connected to NMI vector */
    Hardwarefault_IRQn                  = -13,                                   /*!< All types of failures */
    SVCall_IRQn                         = -5,                                    /*!< System service calls via SWI instructions */
    DebugMonitor_IRQn                   = -4,                                    /*!< debug monitor */
    PendSV_IRQn                         = -2,                                    /*!< Suspendable system services */
    SysTick_IRQn                        = -1,                                    /*!< System tick timer */
    IWDG_IRQn                           = 0,                                     /*!< Watchdog interrupt (IWDG is EXTI17) */
    PVD_IRQn                            = 1,                                     /*!< Supply Voltage Detect (PVD) Interrupt (EXTI16) */
    FLASH_IRQn                          = 3,                                     /*!< Flash Global Interrupt */
    RCC_IRQn                            = 4,                                     /*!< RCC  global interrupt */
    EXTI0_1_IRQn                        = 5,                                     /*!< EXTI line [1: 0] interrupt */
    EXTI2_3_IRQn                        = 6,                                     /*!< EXTI line [3: 2] interrupt */
    EXTI4_15_IRQn                       = 7,                                     /*!< EXTI line [15: 4] interrupt */
    ADC1_IRQn                           = 12,                                    /*!< ADC1 global interrupt */
    TIM1_BRK_UP_TRG_COM_IRQn            = 13,                                    /*!< TIM1 brake, update, trigger, and COM interrupt */
    TIM1_CC_IRQn                        = 14,                                    /*!< TIM1 capture compare interrupt */
    TIM3_IRQn                           = 16,                                    /*!< TIM3 global interrupt */
    TIM14_IRQn                          = 19,                                    /*!< TIM14 global interrupt */
    I2C1_IRQn                           = 23,                                    /*!< I2C1 global interrupt */
    SPI1_IRQn                           = 25,                                    /*!< SPI1 global interrupt */
    USART1_IRQn                         = 27,                                    /*!< USART1 global interrupt */
    USART2_IRQn                         = 28,                                    /*!< USART2 global interrupt */

} IRQn_Type;

/*!

 * @}
 */ /* end of group Interrupt_vector_numbers */


/* -----------------------------------------------------------------------------
 -- Cortex M0 Core Configuration
  ----------------------------------------------------------------------------- */

/*!
 * @addtogroup Cortex_Core_Configuration Cortex M0 Core Configuration
 * @{
 */
#define __CM0_REV                 0x0000U   /* Core revision r0p0 */
#define __MPU_PRESENT             0U        /* no MPU present */
#define __VTOR_PRESENT            0U        /* no VTOR present */
#define __NVIC_PRIO_BITS          2U        /* Number of Bits used for Priority Levels */
#define __Vendor_SysTickConfig    0U        /* Set to 1 if different SysTick Config is used */

#include "core_cm0.h"                /* Core Peripheral Access Layer */

/*!
 * @}
 */ /* end of group Cortex_Core_Configuration */



/* -----------------------------------------------------------------------------
   -- Device Peripheral Access Layer
   ----------------------------------------------------------------------------- */

/*!
 * @addtogroup Peripheral_access_layer Device Peripheral Access Layer
 * @{

 */

/*
** Start of section using anonymous unions
*/

#if   defined (__CC_ARM)
  #pragma push
  #pragma anon_unions
#elif defined (__ICCARM__)
  #pragma language=extended
#elif defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050)
  #pragma clang diagnostic push
  #pragma clang diagnostic ignored "-Wc11-extensions"
  #pragma clang diagnostic ignored "-Wreserved-id-macro"
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

/* Define the base address of the large memory block mapping */

#define BOOT_BASE                   0x00000000u
#define FLASH_ADDR_BASE             0x08000000u
#define SYSTEMMEMORY_BASE           0x1FFFF400u
#define OPTIONBYTES_BASE            0x1FFFF800u
#define SRAM_BASE                   0x20000000u
#define APB1_0_BASE                 0x40000000u
#define APB1_1_BASE                 0x40010000u
#define AHB_0_BASE                  0x40020000u
#define AHB_1_BASE                  0x48000000u


/* Define the mapping base address of the peripheral module */
#define TIM3_BASE                   (APB1_0_BASE + 0x0400)
#define IWDG_BASE                   (APB1_0_BASE + 0x3000)
#define USART2_BASE                 (APB1_0_BASE + 0x4400)
#define I2C1_BASE                   (APB1_0_BASE + 0x5400)
#define PWR_BASE                    (APB1_0_BASE + 0x7000)

#define SYSCFG_BASE                 (APB1_1_BASE + 0x0000)
#define EXTI_BASE                   (APB1_1_BASE + 0x0400)
#define ADC1_BASE                   (APB1_1_BASE + 0x2400)
#define TIM1_BASE                   (APB1_1_BASE + 0x2C00)
#define SPI1_BASE                   (APB1_1_BASE + 0x3000)
#define DBGMCU_BASE                 (APB1_1_BASE + 0x3400)
#define USART1_BASE                 (APB1_1_BASE + 0x3800)
#define TIM14_BASE                  (APB1_1_BASE + 0x4000)

#define RCC_BASE                    (AHB_0_BASE + 0x1000)
#define FLASH_BASE                  (AHB_0_BASE + 0x2000)
#define CRC_BASE                    (AHB_0_BASE + 0x3000)

#define GPIOA_BASE                  (AHB_1_BASE + 0x0000)
#define GPIOB_BASE                  (AHB_1_BASE + 0x0400)



/*!
 * @addtogroup ADC_Peripheral_Access_Layer ADC Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * ADC Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t ADDATA;                                                         ///< Data register                                offset: 0x00
    __IO uint32_t ADCFG;                                                         ///< Configure register                           offset: 0x04
    __IO uint32_t ADCR;                                                           ///< Control register                             offset: 0x08
    __IO uint32_t Reserved0[1];                                                   ///< Reserved
    __IO uint32_t ADCMPR;                                                         ///< Compare register                             offset: 0x10
    __IO uint32_t ADSTA;                                                          ///< Status register                              offset: 0x14
    __IO uint32_t ADDR[9];                                                        ///< Channel data register                        offset: 0x18
    __IO uint32_t Reserved1[8];                                                   ///< Reserved
    __IO uint32_t CHANY0;                                                         ///< Arbitrary channel channel selection register 0offset: 0x5C
    __IO uint32_t CHANY1;                                                         ///< Arbitrary channel channel selection register 1offset: 0x60
    __IO uint32_t ANYCFG;                                                         ///< Arbitrary channel configuration register     offset: 0x64
    __IO uint32_t ANYCR;                                                          ///< Arbitrary channel control register           offset: 0x68
} ADC_Type;

/*!
 * @addtogroup ADC_Register_Masks Register Masks
 * @{ */

/*!
 * @brief ADC_ADDATA Register Bit Definition
 */

#define ADC_ADDATA_VALID_SHIFT                          (21)
#define ADC_ADDATA_VALID_MASK                           (0x01U << ADC_ADDATA_VALID_SHIFT)
#define ADC_ADDATA_VALID(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_ADDATA_VALID_SHIFT)) & ADC_ADDATA_VALID_MASK)

#define ADC_ADDATA_OVERRUN_SHIFT                        (20)
#define ADC_ADDATA_OVERRUN_MASK                         (0x01U << ADC_ADDATA_OVERRUN_SHIFT)
#define ADC_ADDATA_OVERRUN(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_ADDATA_OVERRUN_SHIFT)) & ADC_ADDATA_OVERRUN_MASK)

#define ADC_ADDATA_CHANNELSEL_SHIFT                     (16)
#define ADC_ADDATA_CHANNELSEL_MASK                      (0x0FU << ADC_ADDATA_CHANNELSEL_SHIFT)
#define ADC_ADDATA_CHANNELSEL(x)                        (((uint32_t)(((uint32_t)(x)) << ADC_ADDATA_CHANNELSEL_SHIFT)) & ADC_ADDATA_CHANNELSEL_MASK)

#define ADC_ADDATA_DATA_SHIFT                           (0)
#define ADC_ADDATA_DATA_MASK                            (0xFFFFU << ADC_ADDATA_DATA_SHIFT)
#define ADC_ADDATA_DATA(x)                              (((uint32_t)(((uint32_t)(x)) << ADC_ADDATA_DATA_SHIFT)) & ADC_ADDATA_DATA_MASK)

/*!
 * @brief ADC_ADCFG Register Bit Definition
 */

#define ADC_ADCFG_ADCPREL_SHIFT                        (14)
#define ADC_ADCFG_ADCPREL_MASK                         (0x01U << ADC_ADCFG_ADCPREL_SHIFT)
#define ADC_ADCFG_ADCPREL(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_ADCPREL_SHIFT)) & ADC_ADCFG_ADCPREL_MASK)

#define ADC_ADCFG_SAMCTL_SHIFT                         (10)
#define ADC_ADCFG_SAMCTL_MASK                          (0x0FU << ADC_ADCFG_SAMCTL_SHIFT)
#define ADC_ADCFG_SAMCTL(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_SAMCTL_SHIFT)) & ADC_ADCFG_SAMCTL_MASK)

#define ADC_ADCFG_RSLTCTL_SHIFT                        (7)
#define ADC_ADCFG_RSLTCTL_MASK                         (0x07U << ADC_ADCFG_RSLTCTL_SHIFT)
#define ADC_ADCFG_RSLTCTL(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_RSLTCTL_SHIFT)) & ADC_ADCFG_RSLTCTL_MASK)

#define ADC_ADCFG_ADCPREH_SHIFT                        (4)
#define ADC_ADCFG_ADCPREH_MASK                         (0x07U << ADC_ADCFG_ADCPREH_SHIFT)
#define ADC_ADCFG_ADCPREH(x)                           (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_ADCPREH_SHIFT)) & ADC_ADCFG_ADCPREH_MASK)

#define ADC_ADCFG_VSEN_SHIFT                           (3)
#define ADC_ADCFG_VSEN_MASK                            (0x01U << ADC_ADCFG_VSEN_SHIFT)
#define ADC_ADCFG_VSEN(x)                              (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_VSEN_SHIFT)) & ADC_ADCFG_VSEN_MASK)

#define ADC_ADCFG_AWDEN_SHIFT                          (1)
#define ADC_ADCFG_AWDEN_MASK                           (0x01U << ADC_ADCFG_AWDEN_SHIFT)
#define ADC_ADCFG_AWDEN(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_AWDEN_SHIFT)) & ADC_ADCFG_AWDEN_MASK)

#define ADC_ADCFG_ADEN_SHIFT                           (0)
#define ADC_ADCFG_ADEN_MASK                            (0x01U << ADC_ADCFG_ADEN_SHIFT)
#define ADC_ADCFG_ADEN(x)                              (((uint32_t)(((uint32_t)(x)) << ADC_ADCFG_ADEN_SHIFT)) & ADC_ADCFG_ADEN_MASK)

/*!
 * @brief ADC_ADCR Register Bit Definition
 */

#define ADC_ADCR_TRGEDGE_SHIFT                          (24)
#define ADC_ADCR_TRGEDGE_MASK                           (0x03U << ADC_ADCR_TRGEDGE_SHIFT)
#define ADC_ADCR_TRGEDGE(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_TRGEDGE_SHIFT)) & ADC_ADCR_TRGEDGE_MASK)

#define ADC_ADCR_TRGSHIFT_SHIFT                         (19)
#define ADC_ADCR_TRGSHIFT_MASK                          (0x07U << ADC_ADCR_TRGSHIFT_SHIFT)
#define ADC_ADCR_TRGSHIFT(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_TRGSHIFT_SHIFT)) & ADC_ADCR_TRGSHIFT_MASK)

#define ADC_ADCR_TRGSELH_SHIFT                          (17)
#define ADC_ADCR_TRGSELH_MASK                           (0x03U << ADC_ADCR_TRGSELH_SHIFT)
#define ADC_ADCR_TRGSELH(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_TRGSELH_SHIFT)) & ADC_ADCR_TRGSELH_MASK)

#define ADC_ADCR_CMPCH_SHIFT                            (12)
#define ADC_ADCR_CMPCH_MASK                             (0x0FU << ADC_ADCR_CMPCH_SHIFT)
#define ADC_ADCR_CMPCH(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_CMPCH_SHIFT)) & ADC_ADCR_CMPCH_MASK)

#define ADC_ADCR_ALIGN_SHIFT                            (11)
#define ADC_ADCR_ALIGN_MASK                             (0x01U << ADC_ADCR_ALIGN_SHIFT)
#define ADC_ADCR_ALIGN(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_ALIGN_SHIFT)) & ADC_ADCR_ALIGN_MASK)

#define ADC_ADCR_ADMD_SHIFT                             (9)
#define ADC_ADCR_ADMD_MASK                              (0x03U << ADC_ADCR_ADMD_SHIFT)
#define ADC_ADCR_ADMD(x)                                (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_ADMD_SHIFT)) & ADC_ADCR_ADMD_MASK)

#define ADC_ADCR_ADST_SHIFT                             (8)
#define ADC_ADCR_ADST_MASK                              (0x01U << ADC_ADCR_ADST_SHIFT)
#define ADC_ADCR_ADST(x)                                (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_ADST_SHIFT)) & ADC_ADCR_ADST_MASK)

#define ADC_ADCR_TRGSELL_SHIFT                          (4)
#define ADC_ADCR_TRGSELL_MASK                           (0x07U << ADC_ADCR_TRGSELL_SHIFT)
#define ADC_ADCR_TRGSELL(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_TRGSELL_SHIFT)) & ADC_ADCR_TRGSELL_MASK)

#define ADC_ADCR_TRGEN_SHIFT                            (2)
#define ADC_ADCR_TRGEN_MASK                             (0x01U << ADC_ADCR_TRGEN_SHIFT)
#define ADC_ADCR_TRGEN(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_TRGEN_SHIFT)) & ADC_ADCR_TRGEN_MASK)

#define ADC_ADCR_AWDIE_SHIFT                            (1)
#define ADC_ADCR_AWDIE_MASK                             (0x01U << ADC_ADCR_AWDIE_SHIFT)
#define ADC_ADCR_AWDIE(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_AWDIE_SHIFT)) & ADC_ADCR_AWDIE_MASK)

#define ADC_ADCR_ADIE_SHIFT                             (0)
#define ADC_ADCR_ADIE_MASK                              (0x01U << ADC_ADCR_ADIE_SHIFT)
#define ADC_ADCR_ADIE(x)                                (((uint32_t)(((uint32_t)(x)) << ADC_ADCR_ADIE_SHIFT)) & ADC_ADCR_ADIE_MASK)

/*!
 * @brief ADC_ADCMPR Register Bit Definition
 */

#define ADC_ADCMPR_CMPHDATA_SHIFT                       (16)
#define ADC_ADCMPR_CMPHDATA_MASK                        (0xFFFU << ADC_ADCMPR_CMPHDATA_SHIFT)
#define ADC_ADCMPR_CMPHDATA(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_ADCMPR_CMPHDATA_SHIFT)) & ADC_ADCMPR_CMPHDATA_MASK)

#define ADC_ADCMPR_CMPLDATA_SHIFT                       (0)
#define ADC_ADCMPR_CMPLDATA_MASK                        (0xFFFU << ADC_ADCMPR_CMPLDATA_SHIFT)
#define ADC_ADCMPR_CMPLDATA(x)                          (((uint32_t)(((uint32_t)(x)) << ADC_ADCMPR_CMPLDATA_SHIFT)) & ADC_ADCMPR_CMPLDATA_MASK)

/*!
 * @brief ADC_ADSTA Register Bit Definition
 */

#define ADC_ADSTA_OVERRUN_SHIFT                         (20)
#define ADC_ADSTA_OVERRUN_MASK                          (0x3FFU << ADC_ADSTA_OVERRUN_SHIFT)
#define ADC_ADSTA_OVERRUN(x)                            (((uint32_t)(((uint32_t)(x)) << ADC_ADSTA_OVERRUN_SHIFT)) & ADC_ADSTA_OVERRUN_MASK)

#define ADC_ADSTA_VALID_SHIFT                           (8)
#define ADC_ADSTA_VALID_MASK                            (0x1FFU << ADC_ADSTA_VALID_SHIFT)
#define ADC_ADSTA_VALID(x)                              (((uint32_t)(((uint32_t)(x)) << ADC_ADSTA_VALID_SHIFT)) & ADC_ADSTA_VALID_MASK)

#define ADC_ADSTA_CH_SHIFT                              (4)
#define ADC_ADSTA_CH_MASK                               (0x0FU << ADC_ADSTA_CH_SHIFT)
#define ADC_ADSTA_CH(x)                                 (((uint32_t)(((uint32_t)(x)) << ADC_ADSTA_CH_SHIFT)) & ADC_ADSTA_CH_MASK)

#define ADC_ADSTA_BUSY_SHIFT                            (2)
#define ADC_ADSTA_BUSY_MASK                             (0x01U << ADC_ADSTA_BUSY_SHIFT)
#define ADC_ADSTA_BUSY(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADSTA_BUSY_SHIFT)) & ADC_ADSTA_BUSY_MASK)

#define ADC_ADSTA_AWDIF_SHIFT                           (1)
#define ADC_ADSTA_AWDIF_MASK                            (0x01U << ADC_ADSTA_AWDIF_SHIFT)
#define ADC_ADSTA_AWDIF(x)                              (((uint32_t)(((uint32_t)(x)) << ADC_ADSTA_AWDIF_SHIFT)) & ADC_ADSTA_AWDIF_MASK)

#define ADC_ADSTA_ADIF_SHIFT                            (0)
#define ADC_ADSTA_ADIF_MASK                             (0x01U << ADC_ADSTA_ADIF_SHIFT)
#define ADC_ADSTA_ADIF(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADSTA_ADIF_SHIFT)) & ADC_ADSTA_ADIF_MASK)

/*!
 * @brief ADC_ADDR Register Bit Definition
 */

#define ADC_ADDR_VALID_SHIFT                            (21)
#define ADC_ADDR_VALID_MASK                             (0x01U << ADC_ADDR_VALID_SHIFT)
#define ADC_ADDR_VALID(x)                               (((uint32_t)(((uint32_t)(x)) << ADC_ADDR_VALID_SHIFT)) & ADC_ADDR_VALID_MASK)
#define ADC_ADDR_OVERRUN_SHIFT                          (20)
#define ADC_ADDR_OVERRUN_MASK                           (0x01U << ADC_ADDR_OVERRUN_SHIFT)
#define ADC_ADDR_OVERRUN(x)                             (((uint32_t)(((uint32_t)(x)) << ADC_ADDR_OVERRUN_SHIFT)) & ADC_ADDR_OVERRUN_MASK)
#define ADC_ADDR_DATA_SHIFT                             (0)
#define ADC_ADDR_DATA_MASK                              (0xFFFFU << ADC_ADDR_DATA_SHIFT)
#define ADC_ADDR_DATA(x)                                (((uint32_t)(((uint32_t)(x)) << ADC_ADDR_DATA_SHIFT)) & ADC_ADDR_DATA_MASK)

/*!
 * @brief ADC_CHANY0 Register Bit Definition
 */

#define ADC_CHANY0_CHANYSEL7_SHIFT                      (28)
#define ADC_CHANY0_CHANYSEL7_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL7_SHIFT)
#define ADC_CHANY0_CHANYSEL7(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL7_SHIFT)) & ADC_CHANY0_CHANYSEL7_MASK)

#define ADC_CHANY0_CHANYSEL6_SHIFT                      (24)
#define ADC_CHANY0_CHANYSEL6_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL6_SHIFT)
#define ADC_CHANY0_CHANYSEL6(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL6_SHIFT)) & ADC_CHANY0_CHANYSEL6_MASK)

#define ADC_CHANY0_CHANYSEL5_SHIFT                      (20)
#define ADC_CHANY0_CHANYSEL5_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL5_SHIFT)
#define ADC_CHANY0_CHANYSEL5(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL5_SHIFT)) & ADC_CHANY0_CHANYSEL5_MASK)

#define ADC_CHANY0_CHANYSEL4_SHIFT                      (16)
#define ADC_CHANY0_CHANYSEL4_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL4_SHIFT)
#define ADC_CHANY0_CHANYSEL4(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL4_SHIFT)) & ADC_CHANY0_CHANYSEL4_MASK)

#define ADC_CHANY0_CHANYSEL3_SHIFT                      (12)
#define ADC_CHANY0_CHANYSEL3_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL3_SHIFT)
#define ADC_CHANY0_CHANYSEL3(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL3_SHIFT)) & ADC_CHANY0_CHANYSEL3_MASK)

#define ADC_CHANY0_CHANYSEL2_SHIFT                      (8)
#define ADC_CHANY0_CHANYSEL2_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL2_SHIFT)
#define ADC_CHANY0_CHANYSEL2(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL2_SHIFT)) & ADC_CHANY0_CHANYSEL2_MASK)

#define ADC_CHANY0_CHANYSEL1_SHIFT                      (4)
#define ADC_CHANY0_CHANYSEL1_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL1_SHIFT)
#define ADC_CHANY0_CHANYSEL1(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL1_SHIFT)) & ADC_CHANY0_CHANYSEL1_MASK)

#define ADC_CHANY0_CHANYSEL0_SHIFT                      (0)
#define ADC_CHANY0_CHANYSEL0_MASK                       (0x0FU << ADC_CHANY0_CHANYSEL0_SHIFT)
#define ADC_CHANY0_CHANYSEL0(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY0_CHANYSEL0_SHIFT)) & ADC_CHANY0_CHANYSEL0_MASK)

/*!
 * @brief ADC_CHANY1 Register Bit Definition
 */

#define ADC_CHANY1_CHANYSEL8_SHIFT                      (0)
#define ADC_CHANY1_CHANYSEL8_MASK                       (0x0FU << ADC_CHANY1_CHANYSEL8_SHIFT)
#define ADC_CHANY1_CHANYSEL8(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_CHANY1_CHANYSEL8_SHIFT)) & ADC_CHANY1_CHANYSEL8_MASK)

/*!
 * @brief ADC_ANYCFG Register Bit Definition
 */

#define ADC_ANYCFG_CHANYNUM_SHIFT                      (0)
#define ADC_ANYCFG_CHANYNUM_MASK                       (0x0FU << ADC_ANYCFG_CHANYNUM_SHIFT)
#define ADC_ANYCFG_CHANYNUM(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_ANYCFG_CHANYNUM_SHIFT)) & ADC_ANYCFG_CHANYNUM_MASK)

/*!
 * @brief ADC_ANYCR Register Bit Definition
 */

#define ADC_ANYCR_CHANYMDEN_SHIFT                      (0)
#define ADC_ANYCR_CHANYMDEN_MASK                       (0x01U << ADC_ANYCR_CHANYMDEN_SHIFT)
#define ADC_ANYCR_CHANYMDEN(x)                         (((uint32_t)(((uint32_t)(x)) << ADC_ANYCR_CHANYMDEN_SHIFT)) & ADC_ANYCR_CHANYMDEN_MASK)

/*!
 * @}
 */ /* end of group ADC_Register_Masks */
/******************************************************************************
 * ADC Instance
 ******************************************************************************/

#define ADC1                ((ADC_Type*)ADC1_BASE)

/*!
 * @}
 */ /* end of group ADC_Peripheral_Access_Layer */

/*!
 * @addtogroup CRC_Peripheral_Access_Layer CRC Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * CRC Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t DR;                                                             ///< Data register                                offset: 0x00
    __IO uint32_t IDR;                                                            ///< Independent data register                    offset: 0x04
    __IO uint32_t CTRL;                                                           ///< Control register                             offset: 0x08
} CRC_Type;

/*!
 * @addtogroup CRC_Register_Masks Register Masks
 * @{ */

/*!
 * @brief CRC_DR Register Bit Definition
 */

#define CRC_DR_DR_SHIFT                                 (0)
#define CRC_DR_DR_MASK                                  (0xFFFFFFFFU << CRC_DR_DR_SHIFT)
#define CRC_DR_DR(x)                                    (((uint32_t)(((uint32_t)(x)) << CRC_DR_DR_SHIFT)) & CRC_DR_DR_MASK)

/*!
 * @brief CRC_IDR Register Bit Definition
 */

#define CRC_IDR_IDR_SHIFT                               (0)
#define CRC_IDR_IDR_MASK                                (0xFFU << CRC_IDR_IDR_SHIFT)
#define CRC_IDR_IDR(x)                                  (((uint32_t)(((uint32_t)(x)) << CRC_IDR_IDR_SHIFT)) & CRC_IDR_IDR_MASK)

/*!
 * @brief CRC_CTRL Register Bit Definition
 */

#define CRC_CTRL_RESET_SHIFT                            (0)
#define CRC_CTRL_RESET_MASK                             (0x01U << CRC_CTRL_RESET_SHIFT)
#define CRC_CTRL_RESET(x)                               (((uint32_t)(((uint32_t)(x)) << CRC_CTRL_RESET_SHIFT)) & CRC_CTRL_RESET_MASK)

/*!
 * @}
 */ /* end of group CRC_Register_Masks */
/******************************************************************************
 * CRC Instance
 ******************************************************************************/

#define CRC                ((CRC_Type*)CRC_BASE)

/*!
 * @}
 */ /* end of group CRC_Peripheral_Access_Layer */

/*!
 * @addtogroup DBGMCU_Peripheral_Access_Layer DBG Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * DBGMCU Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t IDCODE;                                                         ///< ID CODE                                      offset: 0x00
    __IO uint32_t CR;                                                             ///< Control Register                             offset: 0x04
} DBGMCU_Type;

/*!
 * @addtogroup DBGMCU_Register_Masks Register Masks
 * @{ */

/*!
 * @brief DBGMCU_IDCODE Register Bit Definition
 */

#define DBGMCU_IDCODE_DEVID_SHIFT                          (0)
#define DBGMCU_IDCODE_DEVID_MASK                           (0xFFFFFFFFU << DBGMCU_IDCODE_DEVID_SHIFT)
#define DBGMCU_IDCODE_DEVID(x)                             (((uint32_t)(((uint32_t)(x)) << DBGMCU_IDCODE_DEVID_SHIFT)) & DBGMCU_IDCODE_DEVID_MASK)

/*!
 * @brief DBGMCU_CR Register Bit Definition
 */

#define DBGMCU_CR_DBGTIM14STOP_SHIFT                       (18)
#define DBGMCU_CR_DBGTIM14STOP_MASK                        (0x01U << DBGMCU_CR_DBGTIM14STOP_SHIFT)
#define DBGMCU_CR_DBGTIM14STOP(x)                          (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGTIM14STOP_SHIFT)) & DBGMCU_CR_DBGTIM14STOP_MASK)

#define DBGMCU_CR_DBGTIM3PWMOFF_SHIFT                      (15)
#define DBGMCU_CR_DBGTIM3PWMOFF_MASK                       (0x01U << DBGMCU_CR_DBGTIM3PWMOFF_SHIFT)
#define DBGMCU_CR_DBGTIM3PWMOFF(x)                         (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGTIM3PWMOFF_SHIFT)) & DBGMCU_CR_DBGTIM3PWMOFF_MASK)

#define DBGMCU_CR_DBGTIM1PWMOFF_SHIFT                      (13)
#define DBGMCU_CR_DBGTIM1PWMOFF_MASK                       (0x01U << DBGMCU_CR_DBGTIM1PWMOFF_SHIFT)
#define DBGMCU_CR_DBGTIM1PWMOFF(x)                         (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGTIM1PWMOFF_SHIFT)) & DBGMCU_CR_DBGTIM1PWMOFF_MASK)

#define DBGMCU_CR_DBGTIM3STOP_SHIFT                        (12)
#define DBGMCU_CR_DBGTIM3STOP_MASK                         (0x01U << DBGMCU_CR_DBGTIM3STOP_SHIFT)
#define DBGMCU_CR_DBGTIM3STOP(x)                           (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGTIM3STOP_SHIFT)) & DBGMCU_CR_DBGTIM3STOP_MASK)

#define DBGMCU_CR_DBGTIM1STOP_SHIFT                        (10)
#define DBGMCU_CR_DBGTIM1STOP_MASK                         (0x01U << DBGMCU_CR_DBGTIM1STOP_SHIFT)
#define DBGMCU_CR_DBGTIM1STOP(x)                           (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGTIM1STOP_SHIFT)) & DBGMCU_CR_DBGTIM1STOP_MASK)

#define DBGMCU_CR_DBGIWDGSTOP_SHIFT                        (8)
#define DBGMCU_CR_DBGIWDGSTOP_MASK                         (0x01U << DBGMCU_CR_DBGIWDGSTOP_SHIFT)
#define DBGMCU_CR_DBGIWDGSTOP(x)                           (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGIWDGSTOP_SHIFT)) & DBGMCU_CR_DBGIWDGSTOP_MASK)

#define DBGMCU_CR_DBGSTOPFORLDO_SHIFT                      (3)
#define DBGMCU_CR_DBGSTOPFORLDO_MASK                       (0x01U << DBGMCU_CR_DBGSTOPFORLDO_SHIFT)
#define DBGMCU_CR_DBGSTOPFORLDO(x)                         (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGSTOPFORLDO_SHIFT)) & DBGMCU_CR_DBGSTOPFORLDO_MASK)

#define DBGMCU_CR_DBGSTOP_SHIFT                            (1)
#define DBGMCU_CR_DBGSTOP_MASK                             (0x01U << DBGMCU_CR_DBGSTOP_SHIFT)
#define DBGMCU_CR_DBGSTOP(x)                               (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGSTOP_SHIFT)) & DBGMCU_CR_DBGSTOP_MASK)

#define DBGMCU_CR_DBGSLEEP_SHIFT                           (0)
#define DBGMCU_CR_DBGSLEEP_MASK                            (0x01U << DBGMCU_CR_DBGSLEEP_SHIFT)
#define DBGMCU_CR_DBGSLEEP(x)                              (((uint32_t)(((uint32_t)(x)) << DBGMCU_CR_DBGSLEEP_SHIFT)) & DBGMCU_CR_DBGSLEEP_MASK)

/*!
 * @}
 */ /* end of group DBGMCU_Register_Masks */
/******************************************************************************
 * DBGMCU Instance
 ******************************************************************************/

#define DBGMCU                ((DBGMCU_Type*)DBGMCU_BASE)

/*!
 * @}
 */ /* end of group DBGMCU_Peripheral_Access_Layer */

/*!
 * @addtogroup DEVICE_Peripheral_Access_Layer DEVICE Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * DEVICE Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t UID1;                                                           ///< Configuration register                       offset: 0x00
    __IO uint32_t UID2;                                                           ///< Configuration register                       offset: 0x04
    __IO uint32_t UID3;                                                           ///< Configuration register                       offset: 0x08
} DEVICE_Type;

/*!
 * @addtogroup DEVICE_Register_Masks Register Masks
 * @{ */

/*!
 * @brief DEVICE_UID1 Register Bit Definition
 */

#define DEVICE_UID1_UID_SHIFT                           (0)
#define DEVICE_UID1_UID_MASK                            (0xFFFFFFFFU << DEVICE_UID1_UID_SHIFT)
#define DEVICE_UID1_UID(x)                              (((uint32_t)(((uint32_t)(x)) << DEVICE_UID1_UID_SHIFT)) & DEVICE_UID1_UID_MASK)

/*!
 * @brief DEVICE_UID2 Register Bit Definition
 */

#define DEVICE_UID2_UID_SHIFT                           (0)
#define DEVICE_UID2_UID_MASK                            (0xFFFFFFFFU << DEVICE_UID2_UID_SHIFT)
#define DEVICE_UID2_UID(x)                              (((uint32_t)(((uint32_t)(x)) << DEVICE_UID2_UID_SHIFT)) & DEVICE_UID2_UID_MASK)

/*!
 * @brief DEVICE_UID3 Register Bit Definition
 */

#define DEVICE_UID3_UID_SHIFT                           (0)
#define DEVICE_UID3_UID_MASK                            (0xFFFFFFFFU << DEVICE_UID3_UID_SHIFT)
#define DEVICE_UID3_UID(x)                              (((uint32_t)(((uint32_t)(x)) << DEVICE_UID3_UID_SHIFT)) & DEVICE_UID3_UID_MASK)

/*!
 * @}
 */ /* end of group DEVICE_Register_Masks */
/******************************************************************************
 * DEVICE Instance
 ******************************************************************************/

#define DEVICE                ((DEVICE_Type*)DEVICE_BASE)

/*!
 * @}
 */ /* end of group DEVICE_Peripheral_Access_Layer */

/*!
 * @addtogroup SYSCFG_Peripheral_Access_Layer SYSCFG Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * SYSCFG Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CFGR;                                                           ///< Configuration register                       offset: 0x00
    __IO uint32_t Reserved0[1];                                                   ///< Reserved
    __IO uint32_t EXTICR1;                                                        ///< External interrupt configuration register 1  offset: 0x08
    __IO uint32_t EXTICR2;                                                        ///< External interrupt configuration register 2  offset: 0x0C
    __IO uint32_t EXTICR3;                                                        ///< External interrupt configuration register 3  offset: 0x10
    __IO uint32_t EXTICR4;                                                        ///< External interrupt configuration register 4  offset: 0x14
    __IO uint32_t PADHYS;                                                         ///< PAD configuration register                   offset: 0x18
} SYSCFG_Type;

/*!
 * @addtogroup SYSCFG_Register_Masks Register Masks
 * @{ */

/*!
 * @brief SYSCFG_CFGR Register Bit Definition
 */

#define SYSCFG_CFGR_MEMMODE_SHIFT                       (0)
#define SYSCFG_CFGR_MEMMODE_MASK                        (0x03U << SYSCFG_CFGR_MEMMODE_SHIFT)
#define SYSCFG_CFGR_MEMMODE(x)                          (((uint32_t)(((uint32_t)(x)) << SYSCFG_CFGR_MEMMODE_SHIFT)) & SYSCFG_CFGR_MEMMODE_MASK)

/*!
 * @brief SYSCFG_EXTICR1 Register Bit Definition
 */

#define SYSCFG_EXTICR1_EXTI3_SHIFT                      (12)
#define SYSCFG_EXTICR1_EXTI3_MASK                       (0x0FU << SYSCFG_EXTICR1_EXTI3_SHIFT)
#define SYSCFG_EXTICR1_EXTI3(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR1_EXTI3_SHIFT)) & SYSCFG_EXTICR1_EXTI3_MASK)

#define SYSCFG_EXTICR1_EXTI2_SHIFT                      (8)
#define SYSCFG_EXTICR1_EXTI2_MASK                       (0x0FU << SYSCFG_EXTICR1_EXTI2_SHIFT)
#define SYSCFG_EXTICR1_EXTI2(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR1_EXTI2_SHIFT)) & SYSCFG_EXTICR1_EXTI2_MASK)

#define SYSCFG_EXTICR1_EXTI1_SHIFT                      (4)
#define SYSCFG_EXTICR1_EXTI1_MASK                       (0x0FU << SYSCFG_EXTICR1_EXTI1_SHIFT)
#define SYSCFG_EXTICR1_EXTI1(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR1_EXTI1_SHIFT)) & SYSCFG_EXTICR1_EXTI1_MASK)

#define SYSCFG_EXTICR1_EXTI0_SHIFT                      (0)
#define SYSCFG_EXTICR1_EXTI0_MASK                       (0x0FU << SYSCFG_EXTICR1_EXTI0_SHIFT)
#define SYSCFG_EXTICR1_EXTI0(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR1_EXTI0_SHIFT)) & SYSCFG_EXTICR1_EXTI0_MASK)

/*!
 * @brief SYSCFG_EXTICR2 Register Bit Definition
 */

#define SYSCFG_EXTICR2_EXTI7_SHIFT                      (12)
#define SYSCFG_EXTICR2_EXTI7_MASK                       (0x0FU << SYSCFG_EXTICR2_EXTI7_SHIFT)
#define SYSCFG_EXTICR2_EXTI7(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR2_EXTI7_SHIFT)) & SYSCFG_EXTICR2_EXTI7_MASK)

#define SYSCFG_EXTICR2_EXTI6_SHIFT                      (8)
#define SYSCFG_EXTICR2_EXTI6_MASK                       (0x0FU << SYSCFG_EXTICR2_EXTI6_SHIFT)
#define SYSCFG_EXTICR2_EXTI6(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR2_EXTI6_SHIFT)) & SYSCFG_EXTICR2_EXTI6_MASK)

#define SYSCFG_EXTICR2_EXTI5_SHIFT                      (4)
#define SYSCFG_EXTICR2_EXTI5_MASK                       (0x0FU << SYSCFG_EXTICR2_EXTI5_SHIFT)
#define SYSCFG_EXTICR2_EXTI5(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR2_EXTI5_SHIFT)) & SYSCFG_EXTICR2_EXTI5_MASK)

#define SYSCFG_EXTICR2_EXTI4_SHIFT                      (0)
#define SYSCFG_EXTICR2_EXTI4_MASK                       (0x0FU << SYSCFG_EXTICR2_EXTI4_SHIFT)
#define SYSCFG_EXTICR2_EXTI4(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR2_EXTI4_SHIFT)) & SYSCFG_EXTICR2_EXTI4_MASK)

/*!
 * @brief SYSCFG_EXTICR3 Register Bit Definition
 */

#define SYSCFG_EXTICR3_EXTI11_SHIFT                     (12)
#define SYSCFG_EXTICR3_EXTI11_MASK                      (0x0FU << SYSCFG_EXTICR3_EXTI11_SHIFT)
#define SYSCFG_EXTICR3_EXTI11(x)                        (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR3_EXTI11_SHIFT)) & SYSCFG_EXTICR3_EXTI11_MASK)

#define SYSCFG_EXTICR3_EXTI10_SHIFT                     (8)
#define SYSCFG_EXTICR3_EXTI10_MASK                      (0x0FU << SYSCFG_EXTICR3_EXTI10_SHIFT)
#define SYSCFG_EXTICR3_EXTI10(x)                        (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR3_EXTI10_SHIFT)) & SYSCFG_EXTICR3_EXTI10_MASK)

#define SYSCFG_EXTICR3_EXTI9_SHIFT                      (4)
#define SYSCFG_EXTICR3_EXTI9_MASK                       (0x0FU << SYSCFG_EXTICR3_EXTI9_SHIFT)
#define SYSCFG_EXTICR3_EXTI9(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR3_EXTI9_SHIFT)) & SYSCFG_EXTICR3_EXTI9_MASK)

#define SYSCFG_EXTICR3_EXTI8_SHIFT                      (0)
#define SYSCFG_EXTICR3_EXTI8_MASK                       (0x0FU << SYSCFG_EXTICR3_EXTI8_SHIFT)
#define SYSCFG_EXTICR3_EXTI8(x)                         (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR3_EXTI8_SHIFT)) & SYSCFG_EXTICR3_EXTI8_MASK)

/*!
 * @brief SYSCFG_EXTICR4 Register Bit Definition
 */

#define SYSCFG_EXTICR4_EXTI15_SHIFT                     (12)
#define SYSCFG_EXTICR4_EXTI15_MASK                      (0x0FU << SYSCFG_EXTICR4_EXTI15_SHIFT)
#define SYSCFG_EXTICR4_EXTI15(x)                        (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR4_EXTI15_SHIFT)) & SYSCFG_EXTICR4_EXTI15_MASK)

#define SYSCFG_EXTICR4_EXTI14_SHIFT                     (8)
#define SYSCFG_EXTICR4_EXTI14_MASK                      (0x0FU << SYSCFG_EXTICR4_EXTI14_SHIFT)
#define SYSCFG_EXTICR4_EXTI14(x)                        (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR4_EXTI14_SHIFT)) & SYSCFG_EXTICR4_EXTI14_MASK)

#define SYSCFG_EXTICR4_EXTI13_SHIFT                     (4)
#define SYSCFG_EXTICR4_EXTI13_MASK                      (0x0FU << SYSCFG_EXTICR4_EXTI13_SHIFT)
#define SYSCFG_EXTICR4_EXTI13(x)                        (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR4_EXTI13_SHIFT)) & SYSCFG_EXTICR4_EXTI13_MASK)

#define SYSCFG_EXTICR4_EXTI12_SHIFT                     (0)
#define SYSCFG_EXTICR4_EXTI12_MASK                      (0x0FU << SYSCFG_EXTICR4_EXTI12_SHIFT)
#define SYSCFG_EXTICR4_EXTI12(x)                        (((uint32_t)(((uint32_t)(x)) << SYSCFG_EXTICR4_EXTI12_SHIFT)) & SYSCFG_EXTICR4_EXTI12_MASK)

/*!
 * @brief SYSCFG_PADHYS Register Bit Definition
 */

#define SYSCFG_PADHYS_I2C1MODESEL_SHIFT                 (16)
#define SYSCFG_PADHYS_I2C1MODESEL_MASK                  (0x01U << SYSCFG_PADHYS_I2C1MODESEL_SHIFT)
#define SYSCFG_PADHYS_I2C1MODESEL(x)                    (((uint32_t)(((uint32_t)(x)) << SYSCFG_PADHYS_I2C1MODESEL_SHIFT)) & SYSCFG_PADHYS_I2C1MODESEL_MASK)

/*!
 * @}
 */ /* end of group SYSCFG_Register_Masks */
/******************************************************************************
 * SYSCFG Instance
 ******************************************************************************/

#define SYSCFG                ((SYSCFG_Type*)SYSCFG_BASE)

/*!
 * @}
 */ /* end of group SYSCFG_Peripheral_Access_Layer */

/*!
 * @addtogroup EXTI_Peripheral_Access_Layer EXTI Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * EXTI Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t IMR;                                                            ///< Interrupt mask register                      offset: 0x00
    __IO uint32_t EMR;                                                            ///< Event mask register                          offset: 0x04
    __IO uint32_t RTSR;                                                           ///< Rising trigger selection register            offset: 0x08
    __IO uint32_t FTSR;                                                           ///< Falling trigger selection register           offset: 0x0C
    __IO uint32_t SWIER;                                                          ///< Software interrupt event register            offset: 0x10
    __IO uint32_t PR;                                                             ///< Pending register                             offset: 0x14
} EXTI_Type;

/*!
 * @addtogroup EXTI_Register_Masks Register Masks
 * @{ */

/*!
 * @brief EXTI_IMR Register Bit Definition
 */

#define EXTI_IMR_IMR17_SHIFT                            (17)
#define EXTI_IMR_IMR17_MASK                             (0x01U << EXTI_IMR_IMR17_SHIFT)
#define EXTI_IMR_IMR17(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR17_SHIFT)) & EXTI_IMR_IMR17_MASK)

#define EXTI_IMR_IMR16_SHIFT                            (16)
#define EXTI_IMR_IMR16_MASK                             (0x01U << EXTI_IMR_IMR16_SHIFT)
#define EXTI_IMR_IMR16(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR16_SHIFT)) & EXTI_IMR_IMR16_MASK)

#define EXTI_IMR_IMR15_SHIFT                            (15)
#define EXTI_IMR_IMR15_MASK                             (0x01U << EXTI_IMR_IMR15_SHIFT)
#define EXTI_IMR_IMR15(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR15_SHIFT)) & EXTI_IMR_IMR15_MASK)

#define EXTI_IMR_IMR14_SHIFT                            (14)
#define EXTI_IMR_IMR14_MASK                             (0x01U << EXTI_IMR_IMR14_SHIFT)
#define EXTI_IMR_IMR14(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR14_SHIFT)) & EXTI_IMR_IMR14_MASK)

#define EXTI_IMR_IMR13_SHIFT                            (13)
#define EXTI_IMR_IMR13_MASK                             (0x01U << EXTI_IMR_IMR13_SHIFT)
#define EXTI_IMR_IMR13(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR13_SHIFT)) & EXTI_IMR_IMR13_MASK)

#define EXTI_IMR_IMR12_SHIFT                            (12)
#define EXTI_IMR_IMR12_MASK                             (0x01U << EXTI_IMR_IMR12_SHIFT)
#define EXTI_IMR_IMR12(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR12_SHIFT)) & EXTI_IMR_IMR12_MASK)

#define EXTI_IMR_IMR11_SHIFT                            (11)
#define EXTI_IMR_IMR11_MASK                             (0x01U << EXTI_IMR_IMR11_SHIFT)
#define EXTI_IMR_IMR11(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR11_SHIFT)) & EXTI_IMR_IMR11_MASK)

#define EXTI_IMR_IMR10_SHIFT                            (10)
#define EXTI_IMR_IMR10_MASK                             (0x01U << EXTI_IMR_IMR10_SHIFT)
#define EXTI_IMR_IMR10(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR10_SHIFT)) & EXTI_IMR_IMR10_MASK)

#define EXTI_IMR_IMR9_SHIFT                             (9)
#define EXTI_IMR_IMR9_MASK                              (0x01U << EXTI_IMR_IMR9_SHIFT)
#define EXTI_IMR_IMR9(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR9_SHIFT)) & EXTI_IMR_IMR9_MASK)

#define EXTI_IMR_IMR8_SHIFT                             (8)
#define EXTI_IMR_IMR8_MASK                              (0x01U << EXTI_IMR_IMR8_SHIFT)
#define EXTI_IMR_IMR8(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR8_SHIFT)) & EXTI_IMR_IMR8_MASK)

#define EXTI_IMR_IMR7_SHIFT                             (7)
#define EXTI_IMR_IMR7_MASK                              (0x01U << EXTI_IMR_IMR7_SHIFT)
#define EXTI_IMR_IMR7(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR7_SHIFT)) & EXTI_IMR_IMR7_MASK)

#define EXTI_IMR_IMR6_SHIFT                             (6)
#define EXTI_IMR_IMR6_MASK                              (0x01U << EXTI_IMR_IMR6_SHIFT)
#define EXTI_IMR_IMR6(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR6_SHIFT)) & EXTI_IMR_IMR6_MASK)

#define EXTI_IMR_IMR5_SHIFT                             (5)
#define EXTI_IMR_IMR5_MASK                              (0x01U << EXTI_IMR_IMR5_SHIFT)
#define EXTI_IMR_IMR5(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR5_SHIFT)) & EXTI_IMR_IMR5_MASK)

#define EXTI_IMR_IMR4_SHIFT                             (4)
#define EXTI_IMR_IMR4_MASK                              (0x01U << EXTI_IMR_IMR4_SHIFT)
#define EXTI_IMR_IMR4(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR4_SHIFT)) & EXTI_IMR_IMR4_MASK)

#define EXTI_IMR_IMR3_SHIFT                             (3)
#define EXTI_IMR_IMR3_MASK                              (0x01U << EXTI_IMR_IMR3_SHIFT)
#define EXTI_IMR_IMR3(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR3_SHIFT)) & EXTI_IMR_IMR3_MASK)

#define EXTI_IMR_IMR2_SHIFT                             (2)
#define EXTI_IMR_IMR2_MASK                              (0x01U << EXTI_IMR_IMR2_SHIFT)
#define EXTI_IMR_IMR2(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR2_SHIFT)) & EXTI_IMR_IMR2_MASK)

#define EXTI_IMR_IMR1_SHIFT                             (1)
#define EXTI_IMR_IMR1_MASK                              (0x01U << EXTI_IMR_IMR1_SHIFT)
#define EXTI_IMR_IMR1(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR1_SHIFT)) & EXTI_IMR_IMR1_MASK)

#define EXTI_IMR_IMR0_SHIFT                             (0)
#define EXTI_IMR_IMR0_MASK                              (0x01U << EXTI_IMR_IMR0_SHIFT)
#define EXTI_IMR_IMR0(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_IMR_IMR0_SHIFT)) & EXTI_IMR_IMR0_MASK)

/*!
 * @brief EXTI_EMR Register Bit Definition
 */

#define EXTI_EMR_EMR17_SHIFT                            (17)
#define EXTI_EMR_EMR17_MASK                             (0x01U << EXTI_EMR_EMR17_SHIFT)
#define EXTI_EMR_EMR17(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR17_SHIFT)) & EXTI_EMR_EMR17_MASK)

#define EXTI_EMR_EMR16_SHIFT                            (16)
#define EXTI_EMR_EMR16_MASK                             (0x01U << EXTI_EMR_EMR16_SHIFT)
#define EXTI_EMR_EMR16(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR16_SHIFT)) & EXTI_EMR_EMR16_MASK)

#define EXTI_EMR_EMR15_SHIFT                            (15)
#define EXTI_EMR_EMR15_MASK                             (0x01U << EXTI_EMR_EMR15_SHIFT)
#define EXTI_EMR_EMR15(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR15_SHIFT)) & EXTI_EMR_EMR15_MASK)

#define EXTI_EMR_EMR14_SHIFT                            (14)
#define EXTI_EMR_EMR14_MASK                             (0x01U << EXTI_EMR_EMR14_SHIFT)
#define EXTI_EMR_EMR14(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR14_SHIFT)) & EXTI_EMR_EMR14_MASK)

#define EXTI_EMR_EMR13_SHIFT                            (13)
#define EXTI_EMR_EMR13_MASK                             (0x01U << EXTI_EMR_EMR13_SHIFT)
#define EXTI_EMR_EMR13(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR13_SHIFT)) & EXTI_EMR_EMR13_MASK)

#define EXTI_EMR_EMR12_SHIFT                            (12)
#define EXTI_EMR_EMR12_MASK                             (0x01U << EXTI_EMR_EMR12_SHIFT)
#define EXTI_EMR_EMR12(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR12_SHIFT)) & EXTI_EMR_EMR12_MASK)

#define EXTI_EMR_EMR11_SHIFT                            (11)
#define EXTI_EMR_EMR11_MASK                             (0x01U << EXTI_EMR_EMR11_SHIFT)
#define EXTI_EMR_EMR11(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR11_SHIFT)) & EXTI_EMR_EMR11_MASK)

#define EXTI_EMR_EMR10_SHIFT                            (10)
#define EXTI_EMR_EMR10_MASK                             (0x01U << EXTI_EMR_EMR10_SHIFT)
#define EXTI_EMR_EMR10(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR10_SHIFT)) & EXTI_EMR_EMR10_MASK)

#define EXTI_EMR_EMR9_SHIFT                             (9)
#define EXTI_EMR_EMR9_MASK                              (0x01U << EXTI_EMR_EMR9_SHIFT)
#define EXTI_EMR_EMR9(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR9_SHIFT)) & EXTI_EMR_EMR9_MASK)

#define EXTI_EMR_EMR8_SHIFT                             (8)
#define EXTI_EMR_EMR8_MASK                              (0x01U << EXTI_EMR_EMR8_SHIFT)
#define EXTI_EMR_EMR8(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR8_SHIFT)) & EXTI_EMR_EMR8_MASK)

#define EXTI_EMR_EMR7_SHIFT                             (7)
#define EXTI_EMR_EMR7_MASK                              (0x01U << EXTI_EMR_EMR7_SHIFT)
#define EXTI_EMR_EMR7(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR7_SHIFT)) & EXTI_EMR_EMR7_MASK)

#define EXTI_EMR_EMR6_SHIFT                             (6)
#define EXTI_EMR_EMR6_MASK                              (0x01U << EXTI_EMR_EMR6_SHIFT)
#define EXTI_EMR_EMR6(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR6_SHIFT)) & EXTI_EMR_EMR6_MASK)

#define EXTI_EMR_EMR5_SHIFT                             (5)
#define EXTI_EMR_EMR5_MASK                              (0x01U << EXTI_EMR_EMR5_SHIFT)
#define EXTI_EMR_EMR5(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR5_SHIFT)) & EXTI_EMR_EMR5_MASK)

#define EXTI_EMR_EMR4_SHIFT                             (4)
#define EXTI_EMR_EMR4_MASK                              (0x01U << EXTI_EMR_EMR4_SHIFT)
#define EXTI_EMR_EMR4(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR4_SHIFT)) & EXTI_EMR_EMR4_MASK)

#define EXTI_EMR_EMR3_SHIFT                             (3)
#define EXTI_EMR_EMR3_MASK                              (0x01U << EXTI_EMR_EMR3_SHIFT)
#define EXTI_EMR_EMR3(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR3_SHIFT)) & EXTI_EMR_EMR3_MASK)

#define EXTI_EMR_EMR2_SHIFT                             (2)
#define EXTI_EMR_EMR2_MASK                              (0x01U << EXTI_EMR_EMR2_SHIFT)
#define EXTI_EMR_EMR2(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR2_SHIFT)) & EXTI_EMR_EMR2_MASK)

#define EXTI_EMR_EMR1_SHIFT                             (1)
#define EXTI_EMR_EMR1_MASK                              (0x01U << EXTI_EMR_EMR1_SHIFT)
#define EXTI_EMR_EMR1(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR1_SHIFT)) & EXTI_EMR_EMR1_MASK)

#define EXTI_EMR_EMR0_SHIFT                             (0)
#define EXTI_EMR_EMR0_MASK                              (0x01U << EXTI_EMR_EMR0_SHIFT)
#define EXTI_EMR_EMR0(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_EMR_EMR0_SHIFT)) & EXTI_EMR_EMR0_MASK)

/*!
 * @brief EXTI_RTSR Register Bit Definition
 */

#define EXTI_RTSR_TR17_SHIFT                            (17)
#define EXTI_RTSR_TR17_MASK                             (0x01U << EXTI_RTSR_TR17_SHIFT)
#define EXTI_RTSR_TR17(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR17_SHIFT)) & EXTI_RTSR_TR17_MASK)

#define EXTI_RTSR_TR16_SHIFT                            (16)
#define EXTI_RTSR_TR16_MASK                             (0x01U << EXTI_RTSR_TR16_SHIFT)
#define EXTI_RTSR_TR16(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR16_SHIFT)) & EXTI_RTSR_TR16_MASK)

#define EXTI_RTSR_TR15_SHIFT                            (15)
#define EXTI_RTSR_TR15_MASK                             (0x01U << EXTI_RTSR_TR15_SHIFT)
#define EXTI_RTSR_TR15(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR15_SHIFT)) & EXTI_RTSR_TR15_MASK)

#define EXTI_RTSR_TR14_SHIFT                            (14)
#define EXTI_RTSR_TR14_MASK                             (0x01U << EXTI_RTSR_TR14_SHIFT)
#define EXTI_RTSR_TR14(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR14_SHIFT)) & EXTI_RTSR_TR14_MASK)

#define EXTI_RTSR_TR13_SHIFT                            (13)
#define EXTI_RTSR_TR13_MASK                             (0x01U << EXTI_RTSR_TR13_SHIFT)
#define EXTI_RTSR_TR13(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR13_SHIFT)) & EXTI_RTSR_TR13_MASK)

#define EXTI_RTSR_TR12_SHIFT                            (12)
#define EXTI_RTSR_TR12_MASK                             (0x01U << EXTI_RTSR_TR12_SHIFT)
#define EXTI_RTSR_TR12(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR12_SHIFT)) & EXTI_RTSR_TR12_MASK)

#define EXTI_RTSR_TR11_SHIFT                            (11)
#define EXTI_RTSR_TR11_MASK                             (0x01U << EXTI_RTSR_TR11_SHIFT)
#define EXTI_RTSR_TR11(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR11_SHIFT)) & EXTI_RTSR_TR11_MASK)

#define EXTI_RTSR_TR10_SHIFT                            (10)
#define EXTI_RTSR_TR10_MASK                             (0x01U << EXTI_RTSR_TR10_SHIFT)
#define EXTI_RTSR_TR10(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR10_SHIFT)) & EXTI_RTSR_TR10_MASK)

#define EXTI_RTSR_TR9_SHIFT                             (9)
#define EXTI_RTSR_TR9_MASK                              (0x01U << EXTI_RTSR_TR9_SHIFT)
#define EXTI_RTSR_TR9(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR9_SHIFT)) & EXTI_RTSR_TR9_MASK)

#define EXTI_RTSR_TR8_SHIFT                             (8)
#define EXTI_RTSR_TR8_MASK                              (0x01U << EXTI_RTSR_TR8_SHIFT)
#define EXTI_RTSR_TR8(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR8_SHIFT)) & EXTI_RTSR_TR8_MASK)

#define EXTI_RTSR_TR7_SHIFT                             (7)
#define EXTI_RTSR_TR7_MASK                              (0x01U << EXTI_RTSR_TR7_SHIFT)
#define EXTI_RTSR_TR7(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR7_SHIFT)) & EXTI_RTSR_TR7_MASK)

#define EXTI_RTSR_TR6_SHIFT                             (6)
#define EXTI_RTSR_TR6_MASK                              (0x01U << EXTI_RTSR_TR6_SHIFT)
#define EXTI_RTSR_TR6(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR6_SHIFT)) & EXTI_RTSR_TR6_MASK)

#define EXTI_RTSR_TR5_SHIFT                             (5)
#define EXTI_RTSR_TR5_MASK                              (0x01U << EXTI_RTSR_TR5_SHIFT)
#define EXTI_RTSR_TR5(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR5_SHIFT)) & EXTI_RTSR_TR5_MASK)

#define EXTI_RTSR_TR4_SHIFT                             (4)
#define EXTI_RTSR_TR4_MASK                              (0x01U << EXTI_RTSR_TR4_SHIFT)
#define EXTI_RTSR_TR4(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR4_SHIFT)) & EXTI_RTSR_TR4_MASK)

#define EXTI_RTSR_TR3_SHIFT                             (3)
#define EXTI_RTSR_TR3_MASK                              (0x01U << EXTI_RTSR_TR3_SHIFT)
#define EXTI_RTSR_TR3(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR3_SHIFT)) & EXTI_RTSR_TR3_MASK)

#define EXTI_RTSR_TR2_SHIFT                             (2)
#define EXTI_RTSR_TR2_MASK                              (0x01U << EXTI_RTSR_TR2_SHIFT)
#define EXTI_RTSR_TR2(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR2_SHIFT)) & EXTI_RTSR_TR2_MASK)

#define EXTI_RTSR_TR1_SHIFT                             (1)
#define EXTI_RTSR_TR1_MASK                              (0x01U << EXTI_RTSR_TR1_SHIFT)
#define EXTI_RTSR_TR1(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR1_SHIFT)) & EXTI_RTSR_TR1_MASK)

#define EXTI_RTSR_TR0_SHIFT                             (0)
#define EXTI_RTSR_TR0_MASK                              (0x01U << EXTI_RTSR_TR0_SHIFT)
#define EXTI_RTSR_TR0(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_RTSR_TR0_SHIFT)) & EXTI_RTSR_TR0_MASK)

/*!
 * @brief EXTI_FTSR Register Bit Definition
 */

#define EXTI_FTSR_TR17_SHIFT                            (17)
#define EXTI_FTSR_TR17_MASK                             (0x01U << EXTI_FTSR_TR17_SHIFT)
#define EXTI_FTSR_TR17(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR17_SHIFT)) & EXTI_FTSR_TR17_MASK)

#define EXTI_FTSR_TR16_SHIFT                            (16)
#define EXTI_FTSR_TR16_MASK                             (0x01U << EXTI_FTSR_TR16_SHIFT)
#define EXTI_FTSR_TR16(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR16_SHIFT)) & EXTI_FTSR_TR16_MASK)

#define EXTI_FTSR_TR15_SHIFT                            (15)
#define EXTI_FTSR_TR15_MASK                             (0x01U << EXTI_FTSR_TR15_SHIFT)
#define EXTI_FTSR_TR15(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR15_SHIFT)) & EXTI_FTSR_TR15_MASK)

#define EXTI_FTSR_TR14_SHIFT                            (14)
#define EXTI_FTSR_TR14_MASK                             (0x01U << EXTI_FTSR_TR14_SHIFT)
#define EXTI_FTSR_TR14(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR14_SHIFT)) & EXTI_FTSR_TR14_MASK)

#define EXTI_FTSR_TR13_SHIFT                            (13)
#define EXTI_FTSR_TR13_MASK                             (0x01U << EXTI_FTSR_TR13_SHIFT)
#define EXTI_FTSR_TR13(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR13_SHIFT)) & EXTI_FTSR_TR13_MASK)

#define EXTI_FTSR_TR12_SHIFT                            (12)
#define EXTI_FTSR_TR12_MASK                             (0x01U << EXTI_FTSR_TR12_SHIFT)
#define EXTI_FTSR_TR12(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR12_SHIFT)) & EXTI_FTSR_TR12_MASK)

#define EXTI_FTSR_TR11_SHIFT                            (11)
#define EXTI_FTSR_TR11_MASK                             (0x01U << EXTI_FTSR_TR11_SHIFT)
#define EXTI_FTSR_TR11(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR11_SHIFT)) & EXTI_FTSR_TR11_MASK)

#define EXTI_FTSR_TR10_SHIFT                            (10)
#define EXTI_FTSR_TR10_MASK                             (0x01U << EXTI_FTSR_TR10_SHIFT)
#define EXTI_FTSR_TR10(x)                               (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR10_SHIFT)) & EXTI_FTSR_TR10_MASK)

#define EXTI_FTSR_TR9_SHIFT                             (9)
#define EXTI_FTSR_TR9_MASK                              (0x01U << EXTI_FTSR_TR9_SHIFT)
#define EXTI_FTSR_TR9(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR9_SHIFT)) & EXTI_FTSR_TR9_MASK)

#define EXTI_FTSR_TR8_SHIFT                             (8)
#define EXTI_FTSR_TR8_MASK                              (0x01U << EXTI_FTSR_TR8_SHIFT)
#define EXTI_FTSR_TR8(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR8_SHIFT)) & EXTI_FTSR_TR8_MASK)

#define EXTI_FTSR_TR7_SHIFT                             (7)
#define EXTI_FTSR_TR7_MASK                              (0x01U << EXTI_FTSR_TR7_SHIFT)
#define EXTI_FTSR_TR7(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR7_SHIFT)) & EXTI_FTSR_TR7_MASK)

#define EXTI_FTSR_TR6_SHIFT                             (6)
#define EXTI_FTSR_TR6_MASK                              (0x01U << EXTI_FTSR_TR6_SHIFT)
#define EXTI_FTSR_TR6(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR6_SHIFT)) & EXTI_FTSR_TR6_MASK)

#define EXTI_FTSR_TR5_SHIFT                             (5)
#define EXTI_FTSR_TR5_MASK                              (0x01U << EXTI_FTSR_TR5_SHIFT)
#define EXTI_FTSR_TR5(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR5_SHIFT)) & EXTI_FTSR_TR5_MASK)

#define EXTI_FTSR_TR4_SHIFT                             (4)
#define EXTI_FTSR_TR4_MASK                              (0x01U << EXTI_FTSR_TR4_SHIFT)
#define EXTI_FTSR_TR4(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR4_SHIFT)) & EXTI_FTSR_TR4_MASK)

#define EXTI_FTSR_TR3_SHIFT                             (3)
#define EXTI_FTSR_TR3_MASK                              (0x01U << EXTI_FTSR_TR3_SHIFT)
#define EXTI_FTSR_TR3(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR3_SHIFT)) & EXTI_FTSR_TR3_MASK)

#define EXTI_FTSR_TR2_SHIFT                             (2)
#define EXTI_FTSR_TR2_MASK                              (0x01U << EXTI_FTSR_TR2_SHIFT)
#define EXTI_FTSR_TR2(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR2_SHIFT)) & EXTI_FTSR_TR2_MASK)

#define EXTI_FTSR_TR1_SHIFT                             (1)
#define EXTI_FTSR_TR1_MASK                              (0x01U << EXTI_FTSR_TR1_SHIFT)
#define EXTI_FTSR_TR1(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR1_SHIFT)) & EXTI_FTSR_TR1_MASK)

#define EXTI_FTSR_TR0_SHIFT                             (0)
#define EXTI_FTSR_TR0_MASK                              (0x01U << EXTI_FTSR_TR0_SHIFT)
#define EXTI_FTSR_TR0(x)                                (((uint32_t)(((uint32_t)(x)) << EXTI_FTSR_TR0_SHIFT)) & EXTI_FTSR_TR0_MASK)

/*!
 * @brief EXTI_SWIER Register Bit Definition
 */

#define EXTI_SWIER_SWIER17_SHIFT                        (17)
#define EXTI_SWIER_SWIER17_MASK                         (0x01U << EXTI_SWIER_SWIER17_SHIFT)
#define EXTI_SWIER_SWIER17(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER17_SHIFT)) & EXTI_SWIER_SWIER17_MASK)

#define EXTI_SWIER_SWIER16_SHIFT                        (16)
#define EXTI_SWIER_SWIER16_MASK                         (0x01U << EXTI_SWIER_SWIER16_SHIFT)
#define EXTI_SWIER_SWIER16(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER16_SHIFT)) & EXTI_SWIER_SWIER16_MASK)

#define EXTI_SWIER_SWIER15_SHIFT                        (15)
#define EXTI_SWIER_SWIER15_MASK                         (0x01U << EXTI_SWIER_SWIER15_SHIFT)
#define EXTI_SWIER_SWIER15(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER15_SHIFT)) & EXTI_SWIER_SWIER15_MASK)

#define EXTI_SWIER_SWIER14_SHIFT                        (14)
#define EXTI_SWIER_SWIER14_MASK                         (0x01U << EXTI_SWIER_SWIER14_SHIFT)
#define EXTI_SWIER_SWIER14(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER14_SHIFT)) & EXTI_SWIER_SWIER14_MASK)

#define EXTI_SWIER_SWIER13_SHIFT                        (13)
#define EXTI_SWIER_SWIER13_MASK                         (0x01U << EXTI_SWIER_SWIER13_SHIFT)
#define EXTI_SWIER_SWIER13(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER13_SHIFT)) & EXTI_SWIER_SWIER13_MASK)

#define EXTI_SWIER_SWIER12_SHIFT                        (12)
#define EXTI_SWIER_SWIER12_MASK                         (0x01U << EXTI_SWIER_SWIER12_SHIFT)
#define EXTI_SWIER_SWIER12(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER12_SHIFT)) & EXTI_SWIER_SWIER12_MASK)

#define EXTI_SWIER_SWIER11_SHIFT                        (11)
#define EXTI_SWIER_SWIER11_MASK                         (0x01U << EXTI_SWIER_SWIER11_SHIFT)
#define EXTI_SWIER_SWIER11(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER11_SHIFT)) & EXTI_SWIER_SWIER11_MASK)

#define EXTI_SWIER_SWIER10_SHIFT                        (10)
#define EXTI_SWIER_SWIER10_MASK                         (0x01U << EXTI_SWIER_SWIER10_SHIFT)
#define EXTI_SWIER_SWIER10(x)                           (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER10_SHIFT)) & EXTI_SWIER_SWIER10_MASK)

#define EXTI_SWIER_SWIER9_SHIFT                         (9)
#define EXTI_SWIER_SWIER9_MASK                          (0x01U << EXTI_SWIER_SWIER9_SHIFT)
#define EXTI_SWIER_SWIER9(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER9_SHIFT)) & EXTI_SWIER_SWIER9_MASK)

#define EXTI_SWIER_SWIER8_SHIFT                         (8)
#define EXTI_SWIER_SWIER8_MASK                          (0x01U << EXTI_SWIER_SWIER8_SHIFT)
#define EXTI_SWIER_SWIER8(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER8_SHIFT)) & EXTI_SWIER_SWIER8_MASK)

#define EXTI_SWIER_SWIER7_SHIFT                         (7)
#define EXTI_SWIER_SWIER7_MASK                          (0x01U << EXTI_SWIER_SWIER7_SHIFT)
#define EXTI_SWIER_SWIER7(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER7_SHIFT)) & EXTI_SWIER_SWIER7_MASK)

#define EXTI_SWIER_SWIER6_SHIFT                         (6)
#define EXTI_SWIER_SWIER6_MASK                          (0x01U << EXTI_SWIER_SWIER6_SHIFT)
#define EXTI_SWIER_SWIER6(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER6_SHIFT)) & EXTI_SWIER_SWIER6_MASK)

#define EXTI_SWIER_SWIER5_SHIFT                         (5)
#define EXTI_SWIER_SWIER5_MASK                          (0x01U << EXTI_SWIER_SWIER5_SHIFT)
#define EXTI_SWIER_SWIER5(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER5_SHIFT)) & EXTI_SWIER_SWIER5_MASK)

#define EXTI_SWIER_SWIER4_SHIFT                         (4)
#define EXTI_SWIER_SWIER4_MASK                          (0x01U << EXTI_SWIER_SWIER4_SHIFT)
#define EXTI_SWIER_SWIER4(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER4_SHIFT)) & EXTI_SWIER_SWIER4_MASK)

#define EXTI_SWIER_SWIER3_SHIFT                         (3)
#define EXTI_SWIER_SWIER3_MASK                          (0x01U << EXTI_SWIER_SWIER3_SHIFT)
#define EXTI_SWIER_SWIER3(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER3_SHIFT)) & EXTI_SWIER_SWIER3_MASK)

#define EXTI_SWIER_SWIER2_SHIFT                         (2)
#define EXTI_SWIER_SWIER2_MASK                          (0x01U << EXTI_SWIER_SWIER2_SHIFT)
#define EXTI_SWIER_SWIER2(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER2_SHIFT)) & EXTI_SWIER_SWIER2_MASK)

#define EXTI_SWIER_SWIER1_SHIFT                         (1)
#define EXTI_SWIER_SWIER1_MASK                          (0x01U << EXTI_SWIER_SWIER1_SHIFT)
#define EXTI_SWIER_SWIER1(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER1_SHIFT)) & EXTI_SWIER_SWIER1_MASK)

#define EXTI_SWIER_SWIER0_SHIFT                         (0)
#define EXTI_SWIER_SWIER0_MASK                          (0x01U << EXTI_SWIER_SWIER0_SHIFT)
#define EXTI_SWIER_SWIER0(x)                            (((uint32_t)(((uint32_t)(x)) << EXTI_SWIER_SWIER0_SHIFT)) & EXTI_SWIER_SWIER0_MASK)

/*!
 * @brief EXTI_PR Register Bit Definition
 */

#define EXTI_PR_PR17_SHIFT                              (17)
#define EXTI_PR_PR17_MASK                               (0x01U << EXTI_PR_PR17_SHIFT)
#define EXTI_PR_PR17(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR17_SHIFT)) & EXTI_PR_PR17_MASK)

#define EXTI_PR_PR16_SHIFT                              (16)
#define EXTI_PR_PR16_MASK                               (0x01U << EXTI_PR_PR16_SHIFT)
#define EXTI_PR_PR16(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR16_SHIFT)) & EXTI_PR_PR16_MASK)

#define EXTI_PR_PR15_SHIFT                              (15)
#define EXTI_PR_PR15_MASK                               (0x01U << EXTI_PR_PR15_SHIFT)
#define EXTI_PR_PR15(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR15_SHIFT)) & EXTI_PR_PR15_MASK)

#define EXTI_PR_PR14_SHIFT                              (14)
#define EXTI_PR_PR14_MASK                               (0x01U << EXTI_PR_PR14_SHIFT)
#define EXTI_PR_PR14(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR14_SHIFT)) & EXTI_PR_PR14_MASK)

#define EXTI_PR_PR13_SHIFT                              (13)
#define EXTI_PR_PR13_MASK                               (0x01U << EXTI_PR_PR13_SHIFT)
#define EXTI_PR_PR13(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR13_SHIFT)) & EXTI_PR_PR13_MASK)

#define EXTI_PR_PR12_SHIFT                              (12)
#define EXTI_PR_PR12_MASK                               (0x01U << EXTI_PR_PR12_SHIFT)
#define EXTI_PR_PR12(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR12_SHIFT)) & EXTI_PR_PR12_MASK)

#define EXTI_PR_PR11_SHIFT                              (11)
#define EXTI_PR_PR11_MASK                               (0x01U << EXTI_PR_PR11_SHIFT)
#define EXTI_PR_PR11(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR11_SHIFT)) & EXTI_PR_PR11_MASK)

#define EXTI_PR_PR10_SHIFT                              (10)
#define EXTI_PR_PR10_MASK                               (0x01U << EXTI_PR_PR10_SHIFT)
#define EXTI_PR_PR10(x)                                 (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR10_SHIFT)) & EXTI_PR_PR10_MASK)

#define EXTI_PR_PR9_SHIFT                               (9)
#define EXTI_PR_PR9_MASK                                (0x01U << EXTI_PR_PR9_SHIFT)
#define EXTI_PR_PR9(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR9_SHIFT)) & EXTI_PR_PR9_MASK)

#define EXTI_PR_PR8_SHIFT                               (8)
#define EXTI_PR_PR8_MASK                                (0x01U << EXTI_PR_PR8_SHIFT)
#define EXTI_PR_PR8(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR8_SHIFT)) & EXTI_PR_PR8_MASK)

#define EXTI_PR_PR7_SHIFT                               (7)
#define EXTI_PR_PR7_MASK                                (0x01U << EXTI_PR_PR7_SHIFT)
#define EXTI_PR_PR7(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR7_SHIFT)) & EXTI_PR_PR7_MASK)

#define EXTI_PR_PR6_SHIFT                               (6)
#define EXTI_PR_PR6_MASK                                (0x01U << EXTI_PR_PR6_SHIFT)
#define EXTI_PR_PR6(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR6_SHIFT)) & EXTI_PR_PR6_MASK)

#define EXTI_PR_PR5_SHIFT                               (5)
#define EXTI_PR_PR5_MASK                                (0x01U << EXTI_PR_PR5_SHIFT)
#define EXTI_PR_PR5(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR5_SHIFT)) & EXTI_PR_PR5_MASK)

#define EXTI_PR_PR4_SHIFT                               (4)
#define EXTI_PR_PR4_MASK                                (0x01U << EXTI_PR_PR4_SHIFT)
#define EXTI_PR_PR4(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR4_SHIFT)) & EXTI_PR_PR4_MASK)

#define EXTI_PR_PR3_SHIFT                               (3)
#define EXTI_PR_PR3_MASK                                (0x01U << EXTI_PR_PR3_SHIFT)
#define EXTI_PR_PR3(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR3_SHIFT)) & EXTI_PR_PR3_MASK)

#define EXTI_PR_PR2_SHIFT                               (2)
#define EXTI_PR_PR2_MASK                                (0x01U << EXTI_PR_PR2_SHIFT)
#define EXTI_PR_PR2(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR2_SHIFT)) & EXTI_PR_PR2_MASK)

#define EXTI_PR_PR1_SHIFT                               (1)
#define EXTI_PR_PR1_MASK                                (0x01U << EXTI_PR_PR1_SHIFT)
#define EXTI_PR_PR1(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR1_SHIFT)) & EXTI_PR_PR1_MASK)

#define EXTI_PR_PR0_SHIFT                               (0)
#define EXTI_PR_PR0_MASK                                (0x01U << EXTI_PR_PR0_SHIFT)
#define EXTI_PR_PR0(x)                                  (((uint32_t)(((uint32_t)(x)) << EXTI_PR_PR0_SHIFT)) & EXTI_PR_PR0_MASK)

/*!
 * @}
 */ /* end of group EXTI_Register_Masks */
/******************************************************************************
 * EXTI Instance
 ******************************************************************************/

#define EXTI                ((EXTI_Type*)EXTI_BASE)

/*!
 * @}
 */ /* end of group EXTI_Peripheral_Access_Layer */

/*!
 * @addtogroup FLASH_Peripheral_Access_Layer FLASH Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * FLASH Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t ACR;                                                            ///< Flash access control register                offset: 0x00
    __IO uint32_t KEYR;                                                           ///< Flash key                                    offset: 0x04
    __IO uint32_t OPTKEYR;                                                        ///< Option byte key                              offset: 0x08
    __IO uint32_t SR;                                                             ///< Flash status register                        offset: 0x0C
    __IO uint32_t CR;                                                             ///< Flash control register                       offset: 0x10
    __IO uint32_t AR;                                                             ///< Flash address register                       offset: 0x14
    __IO uint32_t Reserved0[1];                                                   ///< Reserved
    __IO uint32_t OBR;                                                            ///< Option byte register                         offset: 0x1c
    __IO uint32_t WRPR;                                                           ///< Write protect register                       offset: 0x20
} FLASH_Type;

/*!
 * @addtogroup FLASH_Register_Masks Register Masks
 * @{ */

/*!
 * @brief FLASH_ACR Register Bit Definition
 */

#define FLASH_ACR_PRFTBS_SHIFT                          (5)
#define FLASH_ACR_PRFTBS_MASK                           (0x01U << FLASH_ACR_PRFTBS_SHIFT)
#define FLASH_ACR_PRFTBS(x)                             (((uint32_t)(((uint32_t)(x)) << FLASH_ACR_PRFTBS_SHIFT)) & FLASH_ACR_PRFTBS_MASK)

#define FLASH_ACR_PRFTBE_SHIFT                          (4)
#define FLASH_ACR_PRFTBE_MASK                           (0x01U << FLASH_ACR_PRFTBE_SHIFT)
#define FLASH_ACR_PRFTBE(x)                             (((uint32_t)(((uint32_t)(x)) << FLASH_ACR_PRFTBE_SHIFT)) & FLASH_ACR_PRFTBE_MASK)

#define FLASH_ACR_HLFCYA_SHIFT                          (3)
#define FLASH_ACR_HLFCYA_MASK                           (0x01U << FLASH_ACR_HLFCYA_SHIFT)
#define FLASH_ACR_HLFCYA(x)                             (((uint32_t)(((uint32_t)(x)) << FLASH_ACR_HLFCYA_SHIFT)) & FLASH_ACR_HLFCYA_MASK)

#define FLASH_ACR_LATENCY_SHIFT                         (0)
#define FLASH_ACR_LATENCY_MASK                          (0x07U << FLASH_ACR_LATENCY_SHIFT)
#define FLASH_ACR_LATENCY(x)                            (((uint32_t)(((uint32_t)(x)) << FLASH_ACR_LATENCY_SHIFT)) & FLASH_ACR_LATENCY_MASK)

/*!
 * @brief FLASH_KEYR Register Bit Definition
 */

#define FLASH_KEYR_FKEYR_SHIFT                          (0)
#define FLASH_KEYR_FKEYR_MASK                           (0xFFFFFFFFU << FLASH_KEYR_FKEYR_SHIFT)
#define FLASH_KEYR_FKEYR(x)                             (((uint32_t)(((uint32_t)(x)) << FLASH_KEYR_FKEYR_SHIFT)) & FLASH_KEYR_FKEYR_MASK)

/*!
 * @brief FLASH_OPTKEYR Register Bit Definition
 */

#define FLASH_OPTKEYR_OPTKEYR_SHIFT                     (0)
#define FLASH_OPTKEYR_OPTKEYR_MASK                      (0xFFFFFFFFU << FLASH_OPTKEYR_OPTKEYR_SHIFT)
#define FLASH_OPTKEYR_OPTKEYR(x)                        (((uint32_t)(((uint32_t)(x)) << FLASH_OPTKEYR_OPTKEYR_SHIFT)) & FLASH_OPTKEYR_OPTKEYR_MASK)

/*!
 * @brief FLASH_SR Register Bit Definition
 */

#define FLASH_SR_EOP_SHIFT                              (5)
#define FLASH_SR_EOP_MASK                               (0x01U << FLASH_SR_EOP_SHIFT)
#define FLASH_SR_EOP(x)                                 (((uint32_t)(((uint32_t)(x)) << FLASH_SR_EOP_SHIFT)) & FLASH_SR_EOP_MASK)

#define FLASH_SR_WRPRTERR_SHIFT                         (4)
#define FLASH_SR_WRPRTERR_MASK                          (0x01U << FLASH_SR_WRPRTERR_SHIFT)
#define FLASH_SR_WRPRTERR(x)                            (((uint32_t)(((uint32_t)(x)) << FLASH_SR_WRPRTERR_SHIFT)) & FLASH_SR_WRPRTERR_MASK)

#define FLASH_SR_PGERR_SHIFT                            (2)
#define FLASH_SR_PGERR_MASK                             (0x01U << FLASH_SR_PGERR_SHIFT)
#define FLASH_SR_PGERR(x)                               (((uint32_t)(((uint32_t)(x)) << FLASH_SR_PGERR_SHIFT)) & FLASH_SR_PGERR_MASK)

#define FLASH_SR_BSY_SHIFT                              (0)
#define FLASH_SR_BSY_MASK                               (0x01U << FLASH_SR_BSY_SHIFT)
#define FLASH_SR_BSY(x)                                 (((uint32_t)(((uint32_t)(x)) << FLASH_SR_BSY_SHIFT)) & FLASH_SR_BSY_MASK)

/*!
 * @brief FLASH_CR Register Bit Definition
 */

#define FLASH_CR_EOPIE_SHIFT                            (12)
#define FLASH_CR_EOPIE_MASK                             (0x01U << FLASH_CR_EOPIE_SHIFT)
#define FLASH_CR_EOPIE(x)                               (((uint32_t)(((uint32_t)(x)) << FLASH_CR_EOPIE_SHIFT)) & FLASH_CR_EOPIE_MASK)

#define FLASH_CR_ERRIE_SHIFT                            (10)
#define FLASH_CR_ERRIE_MASK                             (0x01U << FLASH_CR_ERRIE_SHIFT)
#define FLASH_CR_ERRIE(x)                               (((uint32_t)(((uint32_t)(x)) << FLASH_CR_ERRIE_SHIFT)) & FLASH_CR_ERRIE_MASK)

#define FLASH_CR_OPTWRE_SHIFT                           (9)
#define FLASH_CR_OPTWRE_MASK                            (0x01U << FLASH_CR_OPTWRE_SHIFT)
#define FLASH_CR_OPTWRE(x)                              (((uint32_t)(((uint32_t)(x)) << FLASH_CR_OPTWRE_SHIFT)) & FLASH_CR_OPTWRE_MASK)

#define FLASH_CR_LOCK_SHIFT                             (7)
#define FLASH_CR_LOCK_MASK                              (0x01U << FLASH_CR_LOCK_SHIFT)
#define FLASH_CR_LOCK(x)                                (((uint32_t)(((uint32_t)(x)) << FLASH_CR_LOCK_SHIFT)) & FLASH_CR_LOCK_MASK)

#define FLASH_CR_STRT_SHIFT                             (6)
#define FLASH_CR_STRT_MASK                              (0x01U << FLASH_CR_STRT_SHIFT)
#define FLASH_CR_STRT(x)                                (((uint32_t)(((uint32_t)(x)) << FLASH_CR_STRT_SHIFT)) & FLASH_CR_STRT_MASK)

#define FLASH_CR_OPTER_SHIFT                            (5)
#define FLASH_CR_OPTER_MASK                             (0x01U << FLASH_CR_OPTER_SHIFT)
#define FLASH_CR_OPTER(x)                               (((uint32_t)(((uint32_t)(x)) << FLASH_CR_OPTER_SHIFT)) & FLASH_CR_OPTER_MASK)

#define FLASH_CR_OPTPG_SHIFT                            (4)
#define FLASH_CR_OPTPG_MASK                             (0x01U << FLASH_CR_OPTPG_SHIFT)
#define FLASH_CR_OPTPG(x)                               (((uint32_t)(((uint32_t)(x)) << FLASH_CR_OPTPG_SHIFT)) & FLASH_CR_OPTPG_MASK)

#define FLASH_CR_MER_SHIFT                              (2)
#define FLASH_CR_MER_MASK                               (0x01U << FLASH_CR_MER_SHIFT)
#define FLASH_CR_MER(x)                                 (((uint32_t)(((uint32_t)(x)) << FLASH_CR_MER_SHIFT)) & FLASH_CR_MER_MASK)

#define FLASH_CR_PER_SHIFT                              (1)
#define FLASH_CR_PER_MASK                               (0x01U << FLASH_CR_PER_SHIFT)
#define FLASH_CR_PER(x)                                 (((uint32_t)(((uint32_t)(x)) << FLASH_CR_PER_SHIFT)) & FLASH_CR_PER_MASK)

#define FLASH_CR_PG_SHIFT                               (0)
#define FLASH_CR_PG_MASK                                (0x01U << FLASH_CR_PG_SHIFT)
#define FLASH_CR_PG(x)                                  (((uint32_t)(((uint32_t)(x)) << FLASH_CR_PG_SHIFT)) & FLASH_CR_PG_MASK)

/*!
 * @brief FLASH_AR Register Bit Definition
 */

#define FLASH_AR_FAR_SHIFT                              (0)
#define FLASH_AR_FAR_MASK                               (0xFFFFFFFFU << FLASH_AR_FAR_SHIFT)
#define FLASH_AR_FAR(x)                                 (((uint32_t)(((uint32_t)(x)) << FLASH_AR_FAR_SHIFT)) & FLASH_AR_FAR_MASK)

/*!
 * @brief FLASH_OBR Register Bit Definition
 */

#define FLASH_OBR_DATA1_SHIFT                           (18)
#define FLASH_OBR_DATA1_MASK                            (0xFFU << FLASH_OBR_DATA1_SHIFT)
#define FLASH_OBR_DATA1(x)                              (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_DATA1_SHIFT)) & FLASH_OBR_DATA1_MASK)

#define FLASH_OBR_DATA0_SHIFT                           (10)
#define FLASH_OBR_DATA0_MASK                            (0xFFU << FLASH_OBR_DATA0_SHIFT)
#define FLASH_OBR_DATA0(x)                              (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_DATA0_SHIFT)) & FLASH_OBR_DATA0_MASK)

#define FLASH_OBR_NBOOT1_SHIFT                          (6)
#define FLASH_OBR_NBOOT1_MASK                           (0x01U << FLASH_OBR_NBOOT1_SHIFT)
#define FLASH_OBR_NBOOT1(x)                             (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_NBOOT1_SHIFT)) & FLASH_OBR_NBOOT1_MASK)

#define FLASH_OBR_NRSTSTDBY_SHIFT                       (4)
#define FLASH_OBR_NRSTSTDBY_MASK                        (0x01U << FLASH_OBR_NRSTSTDBY_SHIFT)
#define FLASH_OBR_NRSTSTDBY(x)                          (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_NRSTSTDBY_SHIFT)) & FLASH_OBR_NRSTSTDBY_MASK)

#define FLASH_OBR_NRSTSTOP_SHIFT                        (3)
#define FLASH_OBR_NRSTSTOP_MASK                         (0x01U << FLASH_OBR_NRSTSTOP_SHIFT)
#define FLASH_OBR_NRSTSTOP(x)                           (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_NRSTSTOP_SHIFT)) & FLASH_OBR_NRSTSTOP_MASK)

#define FLASH_OBR_WDGSW_SHIFT                           (2)
#define FLASH_OBR_WDGSW_MASK                            (0x01U << FLASH_OBR_WDGSW_SHIFT)
#define FLASH_OBR_WDGSW(x)                              (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_WDGSW_SHIFT)) & FLASH_OBR_WDGSW_MASK)

#define FLASH_OBR_RDPRT_SHIFT                           (1)
#define FLASH_OBR_RDPRT_MASK                            (0x01U << FLASH_OBR_RDPRT_SHIFT)
#define FLASH_OBR_RDPRT(x)                              (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_RDPRT_SHIFT)) & FLASH_OBR_RDPRT_MASK)

#define FLASH_OBR_OPTERR_SHIFT                          (0)
#define FLASH_OBR_OPTERR_MASK                           (0x01U << FLASH_OBR_OPTERR_SHIFT)
#define FLASH_OBR_OPTERR(x)                             (((uint32_t)(((uint32_t)(x)) << FLASH_OBR_OPTERR_SHIFT)) & FLASH_OBR_OPTERR_MASK)

/*!
 * @brief FLASH_WRPR Register Bit Definition
 */

#define FLASH_WRPR_WRP_SHIFT                            (0)
#define FLASH_WRPR_WRP_MASK                             (0x0FU << FLASH_WRPR_WRP_SHIFT)
#define FLASH_WRPR_WRP(x)                               (((uint32_t)(((uint32_t)(x)) << FLASH_WRPR_WRP_SHIFT)) & FLASH_WRPR_WRP_MASK)

/*!
 * @}
 */ /* end of group FLASH_Register_Masks */
/******************************************************************************
 * FLASH Instance
 ******************************************************************************/

#define FLASH                ((FLASH_Type*)FLASH_BASE)

/*!
 * @}
 */ /* end of group FLASH_Peripheral_Access_Layer */

/*!
 * @addtogroup GPIO_Peripheral_Access_Layer GPIO Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * GPIO Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CRL;                                                            ///< configuration low register                   offset: 0x00
    __IO uint32_t CRH;                                                            ///< configuration high register                  offset: 0x04
    __IO uint32_t IDR;                                                            ///< input data register                          offset: 0x08
    __IO uint32_t ODR;                                                            ///< output data register                         offset: 0x0C
    __IO uint32_t BSRR;                                                           ///< bit set/reset register                       offset: 0x10
    __IO uint32_t BRR;                                                            ///< bit reset register                           offset: 0x14
    __IO uint32_t LCKR;                                                           ///< Port configuration lock register             offset: 0x18
    __IO uint32_t DCR;                                                            ///< Port output open drain control register      offset: 0x1C
    __IO uint32_t AFRL;                                                           ///< Port Multiplexing Function Low Register      offset: 0x20
    __IO uint32_t AFRH;                                                           ///< Port Multiplexing Function High Register     offset: 0x24
} GPIO_Type;

/*!
 * @addtogroup GPIO_Register_Masks Register Masks
 * @{ */

/*!
 * @brief GPIO_CRL Register Bit Definition
 */

#define GPIO_CRL_CNF7_SHIFT                             (30)
#define GPIO_CRL_CNF7_MASK                              (0x03U << GPIO_CRL_CNF7_SHIFT)
#define GPIO_CRL_CNF7(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF7_SHIFT)) & GPIO_CRL_CNF7_MASK)

#define GPIO_CRL_MODE7_SHIFT                            (28)
#define GPIO_CRL_MODE7_MASK                             (0x03U << GPIO_CRL_MODE7_SHIFT)
#define GPIO_CRL_MODE7(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE7_SHIFT)) & GPIO_CRL_MODE7_MASK)

#define GPIO_CRL_CNF6_SHIFT                             (26)
#define GPIO_CRL_CNF6_MASK                              (0x03U << GPIO_CRL_CNF6_SHIFT)
#define GPIO_CRL_CNF6(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF6_SHIFT)) & GPIO_CRL_CNF6_MASK)

#define GPIO_CRL_MODE6_SHIFT                            (24)
#define GPIO_CRL_MODE6_MASK                             (0x03U << GPIO_CRL_MODE6_SHIFT)
#define GPIO_CRL_MODE6(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE6_SHIFT)) & GPIO_CRL_MODE6_MASK)

#define GPIO_CRL_CNF5_SHIFT                             (22)
#define GPIO_CRL_CNF5_MASK                              (0x03U << GPIO_CRL_CNF5_SHIFT)
#define GPIO_CRL_CNF5(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF5_SHIFT)) & GPIO_CRL_CNF5_MASK)

#define GPIO_CRL_MODE5_SHIFT                            (20)
#define GPIO_CRL_MODE5_MASK                             (0x03U << GPIO_CRL_MODE5_SHIFT)
#define GPIO_CRL_MODE5(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE5_SHIFT)) & GPIO_CRL_MODE5_MASK)

#define GPIO_CRL_CNF4_SHIFT                             (18)
#define GPIO_CRL_CNF4_MASK                              (0x03U << GPIO_CRL_CNF4_SHIFT)
#define GPIO_CRL_CNF4(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF4_SHIFT)) & GPIO_CRL_CNF4_MASK)

#define GPIO_CRL_MODE4_SHIFT                            (16)
#define GPIO_CRL_MODE4_MASK                             (0x03U << GPIO_CRL_MODE4_SHIFT)
#define GPIO_CRL_MODE4(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE4_SHIFT)) & GPIO_CRL_MODE4_MASK)

#define GPIO_CRL_CNF3_SHIFT                             (14)
#define GPIO_CRL_CNF3_MASK                              (0x03U << GPIO_CRL_CNF3_SHIFT)
#define GPIO_CRL_CNF3(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF3_SHIFT)) & GPIO_CRL_CNF3_MASK)

#define GPIO_CRL_MODE3_SHIFT                            (12)
#define GPIO_CRL_MODE3_MASK                             (0x03U << GPIO_CRL_MODE3_SHIFT)
#define GPIO_CRL_MODE3(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE3_SHIFT)) & GPIO_CRL_MODE3_MASK)

#define GPIO_CRL_CNF2_SHIFT                             (10)
#define GPIO_CRL_CNF2_MASK                              (0x03U << GPIO_CRL_CNF2_SHIFT)
#define GPIO_CRL_CNF2(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF2_SHIFT)) & GPIO_CRL_CNF2_MASK)

#define GPIO_CRL_MODE2_SHIFT                            (8)
#define GPIO_CRL_MODE2_MASK                             (0x03U << GPIO_CRL_MODE2_SHIFT)
#define GPIO_CRL_MODE2(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE2_SHIFT)) & GPIO_CRL_MODE2_MASK)

#define GPIO_CRL_CNF1_SHIFT                             (6)
#define GPIO_CRL_CNF1_MASK                              (0x03U << GPIO_CRL_CNF1_SHIFT)
#define GPIO_CRL_CNF1(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF1_SHIFT)) & GPIO_CRL_CNF1_MASK)

#define GPIO_CRL_MODE1_SHIFT                            (4)
#define GPIO_CRL_MODE1_MASK                             (0x03U << GPIO_CRL_MODE1_SHIFT)
#define GPIO_CRL_MODE1(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE1_SHIFT)) & GPIO_CRL_MODE1_MASK)

#define GPIO_CRL_CNF0_SHIFT                             (2)
#define GPIO_CRL_CNF0_MASK                              (0x03U << GPIO_CRL_CNF0_SHIFT)
#define GPIO_CRL_CNF0(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_CNF0_SHIFT)) & GPIO_CRL_CNF0_MASK)

#define GPIO_CRL_MODE0_SHIFT                            (0)
#define GPIO_CRL_MODE0_MASK                             (0x03U << GPIO_CRL_MODE0_SHIFT)
#define GPIO_CRL_MODE0(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRL_MODE0_SHIFT)) & GPIO_CRL_MODE0_MASK)

/*!
 * @brief GPIO_CRH Register Bit Definition
 */

#define GPIO_CRH_CNF15_SHIFT                            (30)
#define GPIO_CRH_CNF15_MASK                             (0x03U << GPIO_CRH_CNF15_SHIFT)
#define GPIO_CRH_CNF15(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF15_SHIFT)) & GPIO_CRH_CNF15_MASK)

#define GPIO_CRH_MODE15_SHIFT                           (28)
#define GPIO_CRH_MODE15_MASK                            (0x03U << GPIO_CRH_MODE15_SHIFT)
#define GPIO_CRH_MODE15(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE15_SHIFT)) & GPIO_CRH_MODE15_MASK)

#define GPIO_CRH_CNF14_SHIFT                            (26)
#define GPIO_CRH_CNF14_MASK                             (0x03U << GPIO_CRH_CNF14_SHIFT)
#define GPIO_CRH_CNF14(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF14_SHIFT)) & GPIO_CRH_CNF14_MASK)

#define GPIO_CRH_MODE14_SHIFT                           (24)
#define GPIO_CRH_MODE14_MASK                            (0x03U << GPIO_CRH_MODE14_SHIFT)
#define GPIO_CRH_MODE14(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE14_SHIFT)) & GPIO_CRH_MODE14_MASK)

#define GPIO_CRH_CNF13_SHIFT                            (22)
#define GPIO_CRH_CNF13_MASK                             (0x03U << GPIO_CRH_CNF13_SHIFT)
#define GPIO_CRH_CNF13(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF13_SHIFT)) & GPIO_CRH_CNF13_MASK)

#define GPIO_CRH_MODE13_SHIFT                           (20)
#define GPIO_CRH_MODE13_MASK                            (0x03U << GPIO_CRH_MODE13_SHIFT)
#define GPIO_CRH_MODE13(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE13_SHIFT)) & GPIO_CRH_MODE13_MASK)

#define GPIO_CRH_CNF12_SHIFT                            (18)
#define GPIO_CRH_CNF12_MASK                             (0x03U << GPIO_CRH_CNF12_SHIFT)
#define GPIO_CRH_CNF12(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF12_SHIFT)) & GPIO_CRH_CNF12_MASK)

#define GPIO_CRH_MODE12_SHIFT                           (16)
#define GPIO_CRH_MODE12_MASK                            (0x03U << GPIO_CRH_MODE12_SHIFT)
#define GPIO_CRH_MODE12(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE12_SHIFT)) & GPIO_CRH_MODE12_MASK)

#define GPIO_CRH_CNF11_SHIFT                            (14)
#define GPIO_CRH_CNF11_MASK                             (0x03U << GPIO_CRH_CNF11_SHIFT)
#define GPIO_CRH_CNF11(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF11_SHIFT)) & GPIO_CRH_CNF11_MASK)

#define GPIO_CRH_MODE11_SHIFT                           (12)
#define GPIO_CRH_MODE11_MASK                            (0x03U << GPIO_CRH_MODE11_SHIFT)
#define GPIO_CRH_MODE11(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE11_SHIFT)) & GPIO_CRH_MODE11_MASK)

#define GPIO_CRH_CNF10_SHIFT                            (10)
#define GPIO_CRH_CNF10_MASK                             (0x03U << GPIO_CRH_CNF10_SHIFT)
#define GPIO_CRH_CNF10(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF10_SHIFT)) & GPIO_CRH_CNF10_MASK)

#define GPIO_CRH_MODE10_SHIFT                           (8)
#define GPIO_CRH_MODE10_MASK                            (0x03U << GPIO_CRH_MODE10_SHIFT)
#define GPIO_CRH_MODE10(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE10_SHIFT)) & GPIO_CRH_MODE10_MASK)

#define GPIO_CRH_CNF9_SHIFT                             (6)
#define GPIO_CRH_CNF9_MASK                              (0x03U << GPIO_CRH_CNF9_SHIFT)
#define GPIO_CRH_CNF9(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF9_SHIFT)) & GPIO_CRH_CNF9_MASK)

#define GPIO_CRH_MODE9_SHIFT                            (4)
#define GPIO_CRH_MODE9_MASK                             (0x03U << GPIO_CRH_MODE9_SHIFT)
#define GPIO_CRH_MODE9(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE9_SHIFT)) & GPIO_CRH_MODE9_MASK)

#define GPIO_CRH_CNF8_SHIFT                             (2)
#define GPIO_CRH_CNF8_MASK                              (0x03U << GPIO_CRH_CNF8_SHIFT)
#define GPIO_CRH_CNF8(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_CNF8_SHIFT)) & GPIO_CRH_CNF8_MASK)

#define GPIO_CRH_MODE8_SHIFT                            (0)
#define GPIO_CRH_MODE8_MASK                             (0x03U << GPIO_CRH_MODE8_SHIFT)
#define GPIO_CRH_MODE8(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_CRH_MODE8_SHIFT)) & GPIO_CRH_MODE8_MASK)

/*!
 * @brief GPIO_IDR Register Bit Definition
 */

#define GPIO_IDR_IDR_SHIFT                              (0)
#define GPIO_IDR_IDR_MASK                               (0xFFFFU << GPIO_IDR_IDR_SHIFT)
#define GPIO_IDR_IDR(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_IDR_IDR_SHIFT)) & GPIO_IDR_IDR_MASK)

/*!
 * @brief GPIO_ODR Register Bit Definition
 */

#define GPIO_ODR_ODR_SHIFT                              (0)
#define GPIO_ODR_ODR_MASK                               (0xFFFFU << GPIO_ODR_ODR_SHIFT)
#define GPIO_ODR_ODR(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_ODR_ODR_SHIFT)) & GPIO_ODR_ODR_MASK)

/*!
 * @brief GPIO_BSRR Register Bit Definition
 */

#define GPIO_BSRR_BR15_SHIFT                            (31)
#define GPIO_BSRR_BR15_MASK                             (0x01U << GPIO_BSRR_BR15_SHIFT)
#define GPIO_BSRR_BR15(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR15_SHIFT)) & GPIO_BSRR_BR15_MASK)

#define GPIO_BSRR_BR14_SHIFT                            (30)
#define GPIO_BSRR_BR14_MASK                             (0x01U << GPIO_BSRR_BR14_SHIFT)
#define GPIO_BSRR_BR14(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR14_SHIFT)) & GPIO_BSRR_BR14_MASK)

#define GPIO_BSRR_BR13_SHIFT                            (29)
#define GPIO_BSRR_BR13_MASK                             (0x01U << GPIO_BSRR_BR13_SHIFT)
#define GPIO_BSRR_BR13(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR13_SHIFT)) & GPIO_BSRR_BR13_MASK)

#define GPIO_BSRR_BR12_SHIFT                            (28)
#define GPIO_BSRR_BR12_MASK                             (0x01U << GPIO_BSRR_BR12_SHIFT)
#define GPIO_BSRR_BR12(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR12_SHIFT)) & GPIO_BSRR_BR12_MASK)

#define GPIO_BSRR_BR11_SHIFT                            (27)
#define GPIO_BSRR_BR11_MASK                             (0x01U << GPIO_BSRR_BR11_SHIFT)
#define GPIO_BSRR_BR11(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR11_SHIFT)) & GPIO_BSRR_BR11_MASK)

#define GPIO_BSRR_BR10_SHIFT                            (26)
#define GPIO_BSRR_BR10_MASK                             (0x01U << GPIO_BSRR_BR10_SHIFT)
#define GPIO_BSRR_BR10(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR10_SHIFT)) & GPIO_BSRR_BR10_MASK)

#define GPIO_BSRR_BR9_SHIFT                             (25)
#define GPIO_BSRR_BR9_MASK                              (0x01U << GPIO_BSRR_BR9_SHIFT)
#define GPIO_BSRR_BR9(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR9_SHIFT)) & GPIO_BSRR_BR9_MASK)

#define GPIO_BSRR_BR8_SHIFT                             (24)
#define GPIO_BSRR_BR8_MASK                              (0x01U << GPIO_BSRR_BR8_SHIFT)
#define GPIO_BSRR_BR8(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR8_SHIFT)) & GPIO_BSRR_BR8_MASK)

#define GPIO_BSRR_BR7_SHIFT                             (23)
#define GPIO_BSRR_BR7_MASK                              (0x01U << GPIO_BSRR_BR7_SHIFT)
#define GPIO_BSRR_BR7(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR7_SHIFT)) & GPIO_BSRR_BR7_MASK)

#define GPIO_BSRR_BR6_SHIFT                             (22)
#define GPIO_BSRR_BR6_MASK                              (0x01U << GPIO_BSRR_BR6_SHIFT)
#define GPIO_BSRR_BR6(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR6_SHIFT)) & GPIO_BSRR_BR6_MASK)

#define GPIO_BSRR_BR5_SHIFT                             (21)
#define GPIO_BSRR_BR5_MASK                              (0x01U << GPIO_BSRR_BR5_SHIFT)
#define GPIO_BSRR_BR5(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR5_SHIFT)) & GPIO_BSRR_BR5_MASK)

#define GPIO_BSRR_BR4_SHIFT                             (20)
#define GPIO_BSRR_BR4_MASK                              (0x01U << GPIO_BSRR_BR4_SHIFT)
#define GPIO_BSRR_BR4(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR4_SHIFT)) & GPIO_BSRR_BR4_MASK)

#define GPIO_BSRR_BR3_SHIFT                             (19)
#define GPIO_BSRR_BR3_MASK                              (0x01U << GPIO_BSRR_BR3_SHIFT)
#define GPIO_BSRR_BR3(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR3_SHIFT)) & GPIO_BSRR_BR3_MASK)

#define GPIO_BSRR_BR2_SHIFT                             (18)
#define GPIO_BSRR_BR2_MASK                              (0x01U << GPIO_BSRR_BR2_SHIFT)
#define GPIO_BSRR_BR2(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR2_SHIFT)) & GPIO_BSRR_BR2_MASK)

#define GPIO_BSRR_BR1_SHIFT                             (17)
#define GPIO_BSRR_BR1_MASK                              (0x01U << GPIO_BSRR_BR1_SHIFT)
#define GPIO_BSRR_BR1(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR1_SHIFT)) & GPIO_BSRR_BR1_MASK)

#define GPIO_BSRR_BR0_SHIFT                             (16)
#define GPIO_BSRR_BR0_MASK                              (0x01U << GPIO_BSRR_BR0_SHIFT)
#define GPIO_BSRR_BR0(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BR0_SHIFT)) & GPIO_BSRR_BR0_MASK)

#define GPIO_BSRR_BS15_SHIFT                            (15)
#define GPIO_BSRR_BS15_MASK                             (0x01U << GPIO_BSRR_BS15_SHIFT)
#define GPIO_BSRR_BS15(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS15_SHIFT)) & GPIO_BSRR_BS15_MASK)

#define GPIO_BSRR_BS14_SHIFT                            (14)
#define GPIO_BSRR_BS14_MASK                             (0x01U << GPIO_BSRR_BS14_SHIFT)
#define GPIO_BSRR_BS14(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS14_SHIFT)) & GPIO_BSRR_BS14_MASK)

#define GPIO_BSRR_BS13_SHIFT                            (13)
#define GPIO_BSRR_BS13_MASK                             (0x01U << GPIO_BSRR_BS13_SHIFT)
#define GPIO_BSRR_BS13(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS13_SHIFT)) & GPIO_BSRR_BS13_MASK)

#define GPIO_BSRR_BS12_SHIFT                            (12)
#define GPIO_BSRR_BS12_MASK                             (0x01U << GPIO_BSRR_BS12_SHIFT)
#define GPIO_BSRR_BS12(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS12_SHIFT)) & GPIO_BSRR_BS12_MASK)

#define GPIO_BSRR_BS11_SHIFT                            (11)
#define GPIO_BSRR_BS11_MASK                             (0x01U << GPIO_BSRR_BS11_SHIFT)
#define GPIO_BSRR_BS11(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS11_SHIFT)) & GPIO_BSRR_BS11_MASK)

#define GPIO_BSRR_BS10_SHIFT                            (10)
#define GPIO_BSRR_BS10_MASK                             (0x01U << GPIO_BSRR_BS10_SHIFT)
#define GPIO_BSRR_BS10(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS10_SHIFT)) & GPIO_BSRR_BS10_MASK)

#define GPIO_BSRR_BS9_SHIFT                             (9)
#define GPIO_BSRR_BS9_MASK                              (0x01U << GPIO_BSRR_BS9_SHIFT)
#define GPIO_BSRR_BS9(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS9_SHIFT)) & GPIO_BSRR_BS9_MASK)

#define GPIO_BSRR_BS8_SHIFT                             (8)
#define GPIO_BSRR_BS8_MASK                              (0x01U << GPIO_BSRR_BS8_SHIFT)
#define GPIO_BSRR_BS8(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS8_SHIFT)) & GPIO_BSRR_BS8_MASK)

#define GPIO_BSRR_BS7_SHIFT                             (7)
#define GPIO_BSRR_BS7_MASK                              (0x01U << GPIO_BSRR_BS7_SHIFT)
#define GPIO_BSRR_BS7(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS7_SHIFT)) & GPIO_BSRR_BS7_MASK)

#define GPIO_BSRR_BS6_SHIFT                             (6)
#define GPIO_BSRR_BS6_MASK                              (0x01U << GPIO_BSRR_BS6_SHIFT)
#define GPIO_BSRR_BS6(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS6_SHIFT)) & GPIO_BSRR_BS6_MASK)

#define GPIO_BSRR_BS5_SHIFT                             (5)
#define GPIO_BSRR_BS5_MASK                              (0x01U << GPIO_BSRR_BS5_SHIFT)
#define GPIO_BSRR_BS5(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS5_SHIFT)) & GPIO_BSRR_BS5_MASK)

#define GPIO_BSRR_BS4_SHIFT                             (4)
#define GPIO_BSRR_BS4_MASK                              (0x01U << GPIO_BSRR_BS4_SHIFT)
#define GPIO_BSRR_BS4(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS4_SHIFT)) & GPIO_BSRR_BS4_MASK)

#define GPIO_BSRR_BS3_SHIFT                             (3)
#define GPIO_BSRR_BS3_MASK                              (0x01U << GPIO_BSRR_BS3_SHIFT)
#define GPIO_BSRR_BS3(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS3_SHIFT)) & GPIO_BSRR_BS3_MASK)

#define GPIO_BSRR_BS2_SHIFT                             (2)
#define GPIO_BSRR_BS2_MASK                              (0x01U << GPIO_BSRR_BS2_SHIFT)
#define GPIO_BSRR_BS2(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS2_SHIFT)) & GPIO_BSRR_BS2_MASK)

#define GPIO_BSRR_BS1_SHIFT                             (1)
#define GPIO_BSRR_BS1_MASK                              (0x01U << GPIO_BSRR_BS1_SHIFT)
#define GPIO_BSRR_BS1(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS1_SHIFT)) & GPIO_BSRR_BS1_MASK)

#define GPIO_BSRR_BS0_SHIFT                             (0)
#define GPIO_BSRR_BS0_MASK                              (0x01U << GPIO_BSRR_BS0_SHIFT)
#define GPIO_BSRR_BS0(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BSRR_BS0_SHIFT)) & GPIO_BSRR_BS0_MASK)

/*!
 * @brief GPIO_BRR Register Bit Definition
 */

#define GPIO_BRR_BR15_SHIFT                             (15)
#define GPIO_BRR_BR15_MASK                              (0x01U << GPIO_BRR_BR15_SHIFT)
#define GPIO_BRR_BR15(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR15_SHIFT)) & GPIO_BRR_BR15_MASK)

#define GPIO_BRR_BR14_SHIFT                             (14)
#define GPIO_BRR_BR14_MASK                              (0x01U << GPIO_BRR_BR14_SHIFT)
#define GPIO_BRR_BR14(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR14_SHIFT)) & GPIO_BRR_BR14_MASK)

#define GPIO_BRR_BR13_SHIFT                             (13)
#define GPIO_BRR_BR13_MASK                              (0x01U << GPIO_BRR_BR13_SHIFT)
#define GPIO_BRR_BR13(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR13_SHIFT)) & GPIO_BRR_BR13_MASK)

#define GPIO_BRR_BR12_SHIFT                             (12)
#define GPIO_BRR_BR12_MASK                              (0x01U << GPIO_BRR_BR12_SHIFT)
#define GPIO_BRR_BR12(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR12_SHIFT)) & GPIO_BRR_BR12_MASK)

#define GPIO_BRR_BR11_SHIFT                             (11)
#define GPIO_BRR_BR11_MASK                              (0x01U << GPIO_BRR_BR11_SHIFT)
#define GPIO_BRR_BR11(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR11_SHIFT)) & GPIO_BRR_BR11_MASK)

#define GPIO_BRR_BR10_SHIFT                             (10)
#define GPIO_BRR_BR10_MASK                              (0x01U << GPIO_BRR_BR10_SHIFT)
#define GPIO_BRR_BR10(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR10_SHIFT)) & GPIO_BRR_BR10_MASK)

#define GPIO_BRR_BR9_SHIFT                              (9)
#define GPIO_BRR_BR9_MASK                               (0x01U << GPIO_BRR_BR9_SHIFT)
#define GPIO_BRR_BR9(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR9_SHIFT)) & GPIO_BRR_BR9_MASK)

#define GPIO_BRR_BR8_SHIFT                              (8)
#define GPIO_BRR_BR8_MASK                               (0x01U << GPIO_BRR_BR8_SHIFT)
#define GPIO_BRR_BR8(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR8_SHIFT)) & GPIO_BRR_BR8_MASK)

#define GPIO_BRR_BR7_SHIFT                              (7)
#define GPIO_BRR_BR7_MASK                               (0x01U << GPIO_BRR_BR7_SHIFT)
#define GPIO_BRR_BR7(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR7_SHIFT)) & GPIO_BRR_BR7_MASK)

#define GPIO_BRR_BR6_SHIFT                              (6)
#define GPIO_BRR_BR6_MASK                               (0x01U << GPIO_BRR_BR6_SHIFT)
#define GPIO_BRR_BR6(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR6_SHIFT)) & GPIO_BRR_BR6_MASK)

#define GPIO_BRR_BR5_SHIFT                              (5)
#define GPIO_BRR_BR5_MASK                               (0x01U << GPIO_BRR_BR5_SHIFT)
#define GPIO_BRR_BR5(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR5_SHIFT)) & GPIO_BRR_BR5_MASK)

#define GPIO_BRR_BR4_SHIFT                              (4)
#define GPIO_BRR_BR4_MASK                               (0x01U << GPIO_BRR_BR4_SHIFT)
#define GPIO_BRR_BR4(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR4_SHIFT)) & GPIO_BRR_BR4_MASK)

#define GPIO_BRR_BR3_SHIFT                              (3)
#define GPIO_BRR_BR3_MASK                               (0x01U << GPIO_BRR_BR3_SHIFT)
#define GPIO_BRR_BR3(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR3_SHIFT)) & GPIO_BRR_BR3_MASK)

#define GPIO_BRR_BR2_SHIFT                              (2)
#define GPIO_BRR_BR2_MASK                               (0x01U << GPIO_BRR_BR2_SHIFT)
#define GPIO_BRR_BR2(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR2_SHIFT)) & GPIO_BRR_BR2_MASK)

#define GPIO_BRR_BR1_SHIFT                              (1)
#define GPIO_BRR_BR1_MASK                               (0x01U << GPIO_BRR_BR1_SHIFT)
#define GPIO_BRR_BR1(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR1_SHIFT)) & GPIO_BRR_BR1_MASK)

#define GPIO_BRR_BR0_SHIFT                              (0)
#define GPIO_BRR_BR0_MASK                               (0x01U << GPIO_BRR_BR0_SHIFT)
#define GPIO_BRR_BR0(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_BRR_BR0_SHIFT)) & GPIO_BRR_BR0_MASK)

/*!
 * @brief GPIO_LCKR Register Bit Definition
 */

#define GPIO_LCKR_LCKK_SHIFT                            (16)
#define GPIO_LCKR_LCKK_MASK                             (0x01U << GPIO_LCKR_LCKK_SHIFT)
#define GPIO_LCKR_LCKK(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_LCKR_LCKK_SHIFT)) & GPIO_LCKR_LCKK_MASK)

#define GPIO_LCKR_LCK_SHIFT                             (0)
#define GPIO_LCKR_LCK_MASK                              (0xFFFFU << GPIO_LCKR_LCK_SHIFT)
#define GPIO_LCKR_LCK(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_LCKR_LCK_SHIFT)) & GPIO_LCKR_LCK_MASK)

/*!
 * @brief GPIO_DCR Register Bit Definition
 */

#define GPIO_DCR_PX15_SHIFT                             (30)
#define GPIO_DCR_PX15_MASK                              (0x03U << GPIO_DCR_PX15_SHIFT)
#define GPIO_DCR_PX15(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX15_SHIFT)) & GPIO_DCR_PX15_MASK)

#define GPIO_DCR_PX14_SHIFT                             (28)
#define GPIO_DCR_PX14_MASK                              (0x03U << GPIO_DCR_PX14_SHIFT)
#define GPIO_DCR_PX14(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX14_SHIFT)) & GPIO_DCR_PX14_MASK)

#define GPIO_DCR_PX13_SHIFT                             (26)
#define GPIO_DCR_PX13_MASK                              (0x03U << GPIO_DCR_PX13_SHIFT)
#define GPIO_DCR_PX13(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX13_SHIFT)) & GPIO_DCR_PX13_MASK)

#define GPIO_DCR_PX12_SHIFT                             (24)
#define GPIO_DCR_PX12_MASK                              (0x03U << GPIO_DCR_PX12_SHIFT)
#define GPIO_DCR_PX12(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX12_SHIFT)) & GPIO_DCR_PX12_MASK)

#define GPIO_DCR_PX11_SHIFT                             (22)
#define GPIO_DCR_PX11_MASK                              (0x03U << GPIO_DCR_PX11_SHIFT)
#define GPIO_DCR_PX11(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX11_SHIFT)) & GPIO_DCR_PX11_MASK)

#define GPIO_DCR_PX10_SHIFT                             (20)
#define GPIO_DCR_PX10_MASK                              (0x03U << GPIO_DCR_PX10_SHIFT)
#define GPIO_DCR_PX10(x)                                (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX10_SHIFT)) & GPIO_DCR_PX10_MASK)

#define GPIO_DCR_PX9_SHIFT                              (18)
#define GPIO_DCR_PX9_MASK                               (0x03U << GPIO_DCR_PX9_SHIFT)
#define GPIO_DCR_PX9(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX9_SHIFT)) & GPIO_DCR_PX9_MASK)

#define GPIO_DCR_PX8_SHIFT                              (16)
#define GPIO_DCR_PX8_MASK                               (0x03U << GPIO_DCR_PX8_SHIFT)
#define GPIO_DCR_PX8(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX8_SHIFT)) & GPIO_DCR_PX8_MASK)

#define GPIO_DCR_PX7_SHIFT                              (14)
#define GPIO_DCR_PX7_MASK                               (0x03U << GPIO_DCR_PX7_SHIFT)
#define GPIO_DCR_PX7(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX7_SHIFT)) & GPIO_DCR_PX7_MASK)

#define GPIO_DCR_PX6_SHIFT                              (12)
#define GPIO_DCR_PX6_MASK                               (0x03U << GPIO_DCR_PX6_SHIFT)
#define GPIO_DCR_PX6(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX6_SHIFT)) & GPIO_DCR_PX6_MASK)

#define GPIO_DCR_PX5_SHIFT                              (10)
#define GPIO_DCR_PX5_MASK                               (0x03U << GPIO_DCR_PX5_SHIFT)
#define GPIO_DCR_PX5(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX5_SHIFT)) & GPIO_DCR_PX5_MASK)

#define GPIO_DCR_PX4_SHIFT                              (8)
#define GPIO_DCR_PX4_MASK                               (0x03U << GPIO_DCR_PX4_SHIFT)
#define GPIO_DCR_PX4(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX4_SHIFT)) & GPIO_DCR_PX4_MASK)

#define GPIO_DCR_PX3_SHIFT                              (6)
#define GPIO_DCR_PX3_MASK                               (0x03U << GPIO_DCR_PX3_SHIFT)
#define GPIO_DCR_PX3(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX3_SHIFT)) & GPIO_DCR_PX3_MASK)

#define GPIO_DCR_PX2_SHIFT                              (4)
#define GPIO_DCR_PX2_MASK                               (0x03U << GPIO_DCR_PX2_SHIFT)
#define GPIO_DCR_PX2(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX2_SHIFT)) & GPIO_DCR_PX2_MASK)

#define GPIO_DCR_PX1_SHIFT                              (2)
#define GPIO_DCR_PX1_MASK                               (0x03U << GPIO_DCR_PX1_SHIFT)
#define GPIO_DCR_PX1(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX1_SHIFT)) & GPIO_DCR_PX1_MASK)

#define GPIO_DCR_PX0_SHIFT                              (0)
#define GPIO_DCR_PX0_MASK                               (0x03U << GPIO_DCR_PX0_SHIFT)
#define GPIO_DCR_PX0(x)                                 (((uint32_t)(((uint32_t)(x)) << GPIO_DCR_PX0_SHIFT)) & GPIO_DCR_PX0_MASK)

/*!
 * @brief GPIO_AFRL Register Bit Definition
 */

#define GPIO_AFRL_AFR7_SHIFT                            (28)
#define GPIO_AFRL_AFR7_MASK                             (0x0FU << GPIO_AFRL_AFR7_SHIFT)
#define GPIO_AFRL_AFR7(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR7_SHIFT)) & GPIO_AFRL_AFR7_MASK)

#define GPIO_AFRL_AFR6_SHIFT                            (24)
#define GPIO_AFRL_AFR6_MASK                             (0x0FU << GPIO_AFRL_AFR6_SHIFT)
#define GPIO_AFRL_AFR6(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR6_SHIFT)) & GPIO_AFRL_AFR6_MASK)

#define GPIO_AFRL_AFR5_SHIFT                            (20)
#define GPIO_AFRL_AFR5_MASK                             (0x0FU << GPIO_AFRL_AFR5_SHIFT)
#define GPIO_AFRL_AFR5(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR5_SHIFT)) & GPIO_AFRL_AFR5_MASK)

#define GPIO_AFRL_AFR4_SHIFT                            (16)
#define GPIO_AFRL_AFR4_MASK                             (0x0FU << GPIO_AFRL_AFR4_SHIFT)
#define GPIO_AFRL_AFR4(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR4_SHIFT)) & GPIO_AFRL_AFR4_MASK)

#define GPIO_AFRL_AFR3_SHIFT                            (12)
#define GPIO_AFRL_AFR3_MASK                             (0x0FU << GPIO_AFRL_AFR3_SHIFT)
#define GPIO_AFRL_AFR3(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR3_SHIFT)) & GPIO_AFRL_AFR3_MASK)

#define GPIO_AFRL_AFR2_SHIFT                            (8)
#define GPIO_AFRL_AFR2_MASK                             (0x0FU << GPIO_AFRL_AFR2_SHIFT)
#define GPIO_AFRL_AFR2(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR2_SHIFT)) & GPIO_AFRL_AFR2_MASK)

#define GPIO_AFRL_AFR1_SHIFT                            (4)
#define GPIO_AFRL_AFR1_MASK                             (0x0FU << GPIO_AFRL_AFR1_SHIFT)
#define GPIO_AFRL_AFR1(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR1_SHIFT)) & GPIO_AFRL_AFR1_MASK)

#define GPIO_AFRL_AFR0_SHIFT                            (0)
#define GPIO_AFRL_AFR0_MASK                             (0x0FU << GPIO_AFRL_AFR0_SHIFT)
#define GPIO_AFRL_AFR0(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRL_AFR0_SHIFT)) & GPIO_AFRL_AFR0_MASK)

/*!
 * @brief GPIO_AFRH Register Bit Definition
 */

#define GPIO_AFRH_AFR15_SHIFT                           (28)
#define GPIO_AFRH_AFR15_MASK                            (0x0FU << GPIO_AFRH_AFR15_SHIFT)
#define GPIO_AFRH_AFR15(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR15_SHIFT)) & GPIO_AFRH_AFR15_MASK)

#define GPIO_AFRH_AFR14_SHIFT                           (24)
#define GPIO_AFRH_AFR14_MASK                            (0x0FU << GPIO_AFRH_AFR14_SHIFT)
#define GPIO_AFRH_AFR14(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR14_SHIFT)) & GPIO_AFRH_AFR14_MASK)

#define GPIO_AFRH_AFR13_SHIFT                           (20)
#define GPIO_AFRH_AFR13_MASK                            (0x0FU << GPIO_AFRH_AFR13_SHIFT)
#define GPIO_AFRH_AFR13(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR13_SHIFT)) & GPIO_AFRH_AFR13_MASK)

#define GPIO_AFRH_AFR12_SHIFT                           (16)
#define GPIO_AFRH_AFR12_MASK                            (0x0FU << GPIO_AFRH_AFR12_SHIFT)
#define GPIO_AFRH_AFR12(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR12_SHIFT)) & GPIO_AFRH_AFR12_MASK)

#define GPIO_AFRH_AFR11_SHIFT                           (12)
#define GPIO_AFRH_AFR11_MASK                            (0x0FU << GPIO_AFRH_AFR11_SHIFT)
#define GPIO_AFRH_AFR11(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR11_SHIFT)) & GPIO_AFRH_AFR11_MASK)

#define GPIO_AFRH_AFR10_SHIFT                           (8)
#define GPIO_AFRH_AFR10_MASK                            (0x0FU << GPIO_AFRH_AFR10_SHIFT)
#define GPIO_AFRH_AFR10(x)                              (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR10_SHIFT)) & GPIO_AFRH_AFR10_MASK)

#define GPIO_AFRH_AFR9_SHIFT                            (4)
#define GPIO_AFRH_AFR9_MASK                             (0x0FU << GPIO_AFRH_AFR9_SHIFT)
#define GPIO_AFRH_AFR9(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR9_SHIFT)) & GPIO_AFRH_AFR9_MASK)

#define GPIO_AFRH_AFR8_SHIFT                            (0)
#define GPIO_AFRH_AFR8_MASK                             (0x0FU << GPIO_AFRH_AFR8_SHIFT)
#define GPIO_AFRH_AFR8(x)                               (((uint32_t)(((uint32_t)(x)) << GPIO_AFRH_AFR8_SHIFT)) & GPIO_AFRH_AFR8_MASK)

/*!
 * @}
 */ /* end of group GPIO_Register_Masks */
/******************************************************************************
 * GPIO Instance
 ******************************************************************************/

#define GPIOA                ((GPIO_Type*)GPIOA_BASE)
#define GPIOB                ((GPIO_Type*)GPIOB_BASE)

/*!
 * @}
 */ /* end of group GPIO_Peripheral_Access_Layer */

/*!
 * @addtogroup I2C_Peripheral_Access_Layer I2C Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * I2C Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CR;                                                             ///< Control Register                             offset: 0x00
    __IO uint32_t TAR;                                                            ///< Target Register                              offset: 0x04
    __IO uint32_t SAR;                                                            ///< Slave Address Register                       offset: 0x08
    __IO uint32_t Reserved0[1];                                                   ///< Reserved
    __IO uint32_t DR;                                                             ///< Data Command Register                        offset: 0x10
    __IO uint32_t SSHR;                                                           ///< SCL High Period Count for Std. Speed Registeroffset: 0x14
    __IO uint32_t SSLR;                                                           ///< SCL Low Period Count for Std. Speed Register offset: 0x18
    __IO uint32_t FSHR;                                                           ///< SCL High Period Count for Fast Speed Registeroffset: 0x1C
    __IO uint32_t FSLR;                                                           ///< SCL Low Period Count for Fast Speed Register offset: 0x20
    __IO uint32_t Reserved1[2];                                                   ///< Reserved
    __IO uint32_t ISR;                                                            ///< Interrupt Status Register                    offset: 0x2C
    __IO uint32_t IMR;                                                            ///< Interrupt Mask Register                      offset: 0x30
    __IO uint32_t RAWISR;                                                         ///< RAW Interrupt Status Register                offset: 0x34
    __IO uint32_t RXTLR;                                                          ///< Receive FIFO Threshold Level Register        offset: 0x38
    __IO uint32_t TXTLR;                                                          ///< Transmit FIFO Threshold Level Register       offset: 0x3C
    __IO uint32_t ICR;                                                            ///< Clear All Interrupt Register                 offset: 0x40
    __IO uint32_t RXUNDER;                                                        ///< Clear RXUNDER Interrupt Register            offset: 0x44
    __IO uint32_t RXOVER;                                                         ///< Clear RXOVER Interrupt Register             offset: 0x48
    __IO uint32_t TXOVER;                                                        ///< Clear TXOVER Interrupt Register             offset: 0x4C
    __IO uint32_t RDREQ;                                                         ///< Clear RDREQ Interrupt Register              offset: 0x50
    __IO uint32_t TXABRT;                                                        ///< Clear TXABRT Interrupt Register             offset: 0x54
    __IO uint32_t RXDONE;                                                        ///< Clear RXDONE Interrupt Register             offset: 0x58
    __IO uint32_t ACTIV;                                                          ///< Clear ACTIVITY Interrupt Register            offset: 0x5C
    __IO uint32_t STOP;                                                           ///< Clear STOP_DET Interrupt Register            offset: 0x60
    __IO uint32_t START;                                                          ///< Clear START_DET Interrupt Register           offset: 0x64
    __IO uint32_t GC;                                                             ///< Clear GEN_CALL Interrupt Register            offset: 0x68
    __IO uint32_t ENR;                                                            ///< Enable Register                              offset: 0x6C
    __IO uint32_t SR;                                                             ///< Status Register                              offset: 0x70
    __IO uint32_t TXFLR;                                                          ///< Transmit FIFO Level Register                 offset: 0x74
    __IO uint32_t RXFLR;                                                          ///< Receive FIFO Level Register                  offset: 0x78
    __IO uint32_t HOLD;                                                           ///< SDA Hold Time Register                       offset: 0x7C
    __IO uint32_t Reserved2[5];                                                   ///< Reserved
    __IO uint32_t SETUP;                                                          ///< SDA Setup Time Register                      offset: 0x94
    __IO uint32_t GCR;                                                            ///< ACK General Call Register                    offset: 0x98
    __IO uint32_t Reserved3[5];                                                   ///< Reserved
    __IO uint32_t SLVMASK;                                                        ///< Slave mask Register                          offset: 0xB0
    __IO uint32_t SLVRCVADDR;                                                     ///< Slave received address Register              offset: 0xB4
} I2C_Type;

/*!
 * @addtogroup I2C_Register_Masks Register Masks
 * @{ */

/*!
 * @brief I2C_CR Register Bit Definition
 */

#define I2C_CR_EMPINT_SHIFT                             (8)
#define I2C_CR_EMPINT_MASK                              (0x01U << I2C_CR_EMPINT_SHIFT)
#define I2C_CR_EMPINT(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_CR_EMPINT_SHIFT)) & I2C_CR_EMPINT_MASK)

#define I2C_CR_STOPINT_SHIFT                            (7)
#define I2C_CR_STOPINT_MASK                             (0x01U << I2C_CR_STOPINT_SHIFT)
#define I2C_CR_STOPINT(x)                               (((uint32_t)(((uint32_t)(x)) << I2C_CR_STOPINT_SHIFT)) & I2C_CR_STOPINT_MASK)

#define I2C_CR_DISSLAVE_SHIFT                           (6)
#define I2C_CR_DISSLAVE_MASK                            (0x01U << I2C_CR_DISSLAVE_SHIFT)
#define I2C_CR_DISSLAVE(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_CR_DISSLAVE_SHIFT)) & I2C_CR_DISSLAVE_MASK)

#define I2C_CR_REPEN_SHIFT                              (5)
#define I2C_CR_REPEN_MASK                               (0x01U << I2C_CR_REPEN_SHIFT)
#define I2C_CR_REPEN(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_CR_REPEN_SHIFT)) & I2C_CR_REPEN_MASK)

#define I2C_CR_MASTER10_SHIFT                           (4)
#define I2C_CR_MASTER10_MASK                            (0x01U << I2C_CR_MASTER10_SHIFT)
#define I2C_CR_MASTER10(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_CR_MASTER10_SHIFT)) & I2C_CR_MASTER10_MASK)

#define I2C_CR_SLAVE10_SHIFT                            (3)
#define I2C_CR_SLAVE10_MASK                             (0x01U << I2C_CR_SLAVE10_SHIFT)
#define I2C_CR_SLAVE10(x)                               (((uint32_t)(((uint32_t)(x)) << I2C_CR_SLAVE10_SHIFT)) & I2C_CR_SLAVE10_MASK)

#define I2C_CR_SPEED_SHIFT                              (1)
#define I2C_CR_SPEED_MASK                               (0x03U << I2C_CR_SPEED_SHIFT)
#define I2C_CR_SPEED(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_CR_SPEED_SHIFT)) & I2C_CR_SPEED_MASK)

#define I2C_CR_MASTER_SHIFT                             (0)
#define I2C_CR_MASTER_MASK                              (0x01U << I2C_CR_MASTER_SHIFT)
#define I2C_CR_MASTER(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_CR_MASTER_SHIFT)) & I2C_CR_MASTER_MASK)

/*!
 * @brief I2C_TAR Register Bit Definition
 */

#define I2C_TAR_SPECIAL_SHIFT                           (11)
#define I2C_TAR_SPECIAL_MASK                            (0x01U << I2C_TAR_SPECIAL_SHIFT)
#define I2C_TAR_SPECIAL(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_TAR_SPECIAL_SHIFT)) & I2C_TAR_SPECIAL_MASK)

#define I2C_TAR_GC_SHIFT                                (10)
#define I2C_TAR_GC_MASK                                 (0x01U << I2C_TAR_GC_SHIFT)
#define I2C_TAR_GC(x)                                   (((uint32_t)(((uint32_t)(x)) << I2C_TAR_GC_SHIFT)) & I2C_TAR_GC_MASK)

#define I2C_TAR_ADDR_SHIFT                              (0)
#define I2C_TAR_ADDR_MASK                               (0x3FFU << I2C_TAR_ADDR_SHIFT)
#define I2C_TAR_ADDR(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_TAR_ADDR_SHIFT)) & I2C_TAR_ADDR_MASK)

/*!
 * @brief I2C_SAR Register Bit Definition
 */

#define I2C_SAR_ADDR_SHIFT                              (0)
#define I2C_SAR_ADDR_MASK                               (0x3FFU << I2C_SAR_ADDR_SHIFT)
#define I2C_SAR_ADDR(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_SAR_ADDR_SHIFT)) & I2C_SAR_ADDR_MASK)

/*!
 * @brief I2C_DR Register Bit Definition
 */

#define I2C_DR_RESTART_SHIFT                            (10)
#define I2C_DR_RESTART_MASK                             (0x01U << I2C_DR_RESTART_SHIFT)
#define I2C_DR_RESTART(x)                               (((uint32_t)(((uint32_t)(x)) << I2C_DR_RESTART_SHIFT)) & I2C_DR_RESTART_MASK)

#define I2C_DR_STOP_SHIFT                               (9)
#define I2C_DR_STOP_MASK                                (0x01U << I2C_DR_STOP_SHIFT)
#define I2C_DR_STOP(x)                                  (((uint32_t)(((uint32_t)(x)) << I2C_DR_STOP_SHIFT)) & I2C_DR_STOP_MASK)

#define I2C_DR_CMD_SHIFT                                (8)
#define I2C_DR_CMD_MASK                                 (0x01U << I2C_DR_CMD_SHIFT)
#define I2C_DR_CMD(x)                                   (((uint32_t)(((uint32_t)(x)) << I2C_DR_CMD_SHIFT)) & I2C_DR_CMD_MASK)

#define I2C_DR_DAT_SHIFT                                (0)
#define I2C_DR_DAT_MASK                                 (0xFFU << I2C_DR_DAT_SHIFT)
#define I2C_DR_DAT(x)                                   (((uint32_t)(((uint32_t)(x)) << I2C_DR_DAT_SHIFT)) & I2C_DR_DAT_MASK)

/*!
 * @brief I2C_SSHR Register Bit Definition
 */

#define I2C_SSHR_CNT_SHIFT                              (0)
#define I2C_SSHR_CNT_MASK                               (0xFFFFU << I2C_SSHR_CNT_SHIFT)
#define I2C_SSHR_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_SSHR_CNT_SHIFT)) & I2C_SSHR_CNT_MASK)

/*!
 * @brief I2C_SSLR Register Bit Definition
 */

#define I2C_SSLR_CNT_SHIFT                              (0)
#define I2C_SSLR_CNT_MASK                               (0xFFFFU << I2C_SSLR_CNT_SHIFT)
#define I2C_SSLR_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_SSLR_CNT_SHIFT)) & I2C_SSLR_CNT_MASK)

/*!
 * @brief I2C_FSHR Register Bit Definition
 */

#define I2C_FSHR_CNT_SHIFT                              (0)
#define I2C_FSHR_CNT_MASK                               (0xFFFFU << I2C_FSHR_CNT_SHIFT)
#define I2C_FSHR_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_FSHR_CNT_SHIFT)) & I2C_FSHR_CNT_MASK)

/*!
 * @brief I2C_FSLR Register Bit Definition
 */

#define I2C_FSLR_CNT_SHIFT                              (0)
#define I2C_FSLR_CNT_MASK                               (0xFFFFU << I2C_FSLR_CNT_SHIFT)
#define I2C_FSLR_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_FSLR_CNT_SHIFT)) & I2C_FSLR_CNT_MASK)

/*!
 * @brief I2C_ISR Register Bit Definition
 */

#define I2C_ISR_ISR_SHIFT                               (0)
#define I2C_ISR_ISR_MASK                                (0x7FFU << I2C_ISR_ISR_SHIFT)
#define I2C_ISR_ISR(x)                                  (((uint32_t)(((uint32_t)(x)) << I2C_ISR_ISR_SHIFT)) & I2C_ISR_ISR_MASK)

/*!
 * @brief I2C_IMR Register Bit Definition
 */

#define I2C_IMR_IMR_SHIFT                               (0)
#define I2C_IMR_IMR_MASK                                (0x7FFU << I2C_IMR_IMR_SHIFT)
#define I2C_IMR_IMR(x)                                  (((uint32_t)(((uint32_t)(x)) << I2C_IMR_IMR_SHIFT)) & I2C_IMR_IMR_MASK)

/*!
 * @brief I2C_RAWISR Register Bit Definition
 */

#define I2C_RAWISR_HOLD_SHIFT                           (13)
#define I2C_RAWISR_HOLD_MASK                            (0x01U << I2C_RAWISR_HOLD_SHIFT)
#define I2C_RAWISR_HOLD(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_HOLD_SHIFT)) & I2C_RAWISR_HOLD_MASK)

#define I2C_RAWISR_RESTART_SHIFT                        (12)
#define I2C_RAWISR_RESTART_MASK                         (0x01U << I2C_RAWISR_RESTART_SHIFT)
#define I2C_RAWISR_RESTART(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_RESTART_SHIFT)) & I2C_RAWISR_RESTART_MASK)

#define I2C_RAWISR_GC_SHIFT                             (11)
#define I2C_RAWISR_GC_MASK                              (0x01U << I2C_RAWISR_GC_SHIFT)
#define I2C_RAWISR_GC(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_GC_SHIFT)) & I2C_RAWISR_GC_MASK)

#define I2C_RAWISR_START_SHIFT                          (10)
#define I2C_RAWISR_START_MASK                           (0x01U << I2C_RAWISR_START_SHIFT)
#define I2C_RAWISR_START(x)                             (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_START_SHIFT)) & I2C_RAWISR_START_MASK)

#define I2C_RAWISR_STOP_SHIFT                           (9)
#define I2C_RAWISR_STOP_MASK                            (0x01U << I2C_RAWISR_STOP_SHIFT)
#define I2C_RAWISR_STOP(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_STOP_SHIFT)) & I2C_RAWISR_STOP_MASK)

#define I2C_RAWISR_ACTIV_SHIFT                          (8)
#define I2C_RAWISR_ACTIV_MASK                           (0x01U << I2C_RAWISR_ACTIV_SHIFT)
#define I2C_RAWISR_ACTIV(x)                             (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_ACTIV_SHIFT)) & I2C_RAWISR_ACTIV_MASK)

#define I2C_RAWISR_RXDONE_SHIFT                         (7)
#define I2C_RAWISR_RXDONE_MASK                          (0x01U << I2C_RAWISR_RXDONE_SHIFT)
#define I2C_RAWISR_RXDONE(x)                            (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_RXDONE_SHIFT)) & I2C_RAWISR_RXDONE_MASK)

#define I2C_RAWISR_TXABRT_SHIFT                         (6)
#define I2C_RAWISR_TXABRT_MASK                          (0x01U << I2C_RAWISR_TXABRT_SHIFT)
#define I2C_RAWISR_TXABRT(x)                            (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_TXABRT_SHIFT)) & I2C_RAWISR_TXABRT_MASK)

#define I2C_RAWISR_RDREQ_SHIFT                          (5)
#define I2C_RAWISR_RDREQ_MASK                           (0x01U << I2C_RAWISR_RDREQ_SHIFT)
#define I2C_RAWISR_RDREQ(x)                             (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_RDREQ_SHIFT)) & I2C_RAWISR_RDREQ_MASK)

#define I2C_RAWISR_TXEMPTY_SHIFT                        (4)
#define I2C_RAWISR_TXEMPTY_MASK                         (0x01U << I2C_RAWISR_TXEMPTY_SHIFT)
#define I2C_RAWISR_TXEMPTY(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_TXEMPTY_SHIFT)) & I2C_RAWISR_TXEMPTY_MASK)

#define I2C_RAWISR_TXOVER_SHIFT                         (3)
#define I2C_RAWISR_TXOVER_MASK                          (0x01U << I2C_RAWISR_TXOVER_SHIFT)
#define I2C_RAWISR_TXOVER(x)                            (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_TXOVER_SHIFT)) & I2C_RAWISR_TXOVER_MASK)

#define I2C_RAWISR_RXFULL_SHIFT                         (2)
#define I2C_RAWISR_RXFULL_MASK                          (0x01U << I2C_RAWISR_RXFULL_SHIFT)
#define I2C_RAWISR_RXFULL(x)                            (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_RXFULL_SHIFT)) & I2C_RAWISR_RXFULL_MASK)

#define I2C_RAWISR_RXOVER_SHIFT                         (1)
#define I2C_RAWISR_RXOVER_MASK                          (0x01U << I2C_RAWISR_RXOVER_SHIFT)
#define I2C_RAWISR_RXOVER(x)                            (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_RXOVER_SHIFT)) & I2C_RAWISR_RXOVER_MASK)

#define I2C_RAWISR_RXUNDER_SHIFT                        (0)
#define I2C_RAWISR_RXUNDER_MASK                         (0x01U << I2C_RAWISR_RXUNDER_SHIFT)
#define I2C_RAWISR_RXUNDER(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_RAWISR_RXUNDER_SHIFT)) & I2C_RAWISR_RXUNDER_MASK)

/*!
 * @brief I2C_RXTLR Register Bit Definition
 */

#define I2C_RXTLR_TL_SHIFT                              (0)
#define I2C_RXTLR_TL_MASK                               (0xFFU << I2C_RXTLR_TL_SHIFT)
#define I2C_RXTLR_TL(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_RXTLR_TL_SHIFT)) & I2C_RXTLR_TL_MASK)

/*!
 * @brief I2C_TXTLR Register Bit Definition
 */

#define I2C_TXTLR_TL_SHIFT                              (0)
#define I2C_TXTLR_TL_MASK                               (0xFFU << I2C_TXTLR_TL_SHIFT)
#define I2C_TXTLR_TL(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_TXTLR_TL_SHIFT)) & I2C_TXTLR_TL_MASK)

/*!
 * @brief I2C_ICR Register Bit Definition
 */

#define I2C_ICR_ICR_SHIFT                               (0)
#define I2C_ICR_ICR_MASK                                (0x01U << I2C_ICR_ICR_SHIFT)
#define I2C_ICR_ICR(x)                                  (((uint32_t)(((uint32_t)(x)) << I2C_ICR_ICR_SHIFT)) & I2C_ICR_ICR_MASK)

/*!
 * @brief I2C_RXUNDER Register Bit Definition
 */

#define I2C_RXUNDER_RXUNDER_SHIFT                      (0)
#define I2C_RXUNDER_RXUNDER_MASK                       (0x01U << I2C_RXUNDER_RXUNDER_SHIFT)
#define I2C_RXUNDER_RXUNDER(x)                         (((uint32_t)(((uint32_t)(x)) << I2C_RXUNDER_RXUNDER_SHIFT)) & I2C_RXUNDER_RXUNDER_MASK)

/*!
 * @brief I2C_RXOVER Register Bit Definition
 */

#define I2C_RXOVER_RXOVER_SHIFT                        (0)
#define I2C_RXOVER_RXOVER_MASK                         (0x01U << I2C_RXOVER_RXOVER_SHIFT)
#define I2C_RXOVER_RXOVER(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_RXOVER_RXOVER_SHIFT)) & I2C_RXOVER_RXOVER_MASK)

/*!
 * @brief I2C_TXOVER Register Bit Definition
 */

#define I2C_TXOVER_TXOVER_SHIFT                        (0)
#define I2C_TXOVER_TXOVER_MASK                         (0x01U << I2C_TXOVER_TXOVER_SHIFT)
#define I2C_TXOVER_TXOVER(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_TXOVER_TXOVER_SHIFT)) & I2C_TXOVER_TXOVER_MASK)

/*!
 * @brief I2C_RDREQ Register Bit Definition
 */

#define I2C_RDREQ_RDREQ_SHIFT                          (0)
#define I2C_RDREQ_RDREQ_MASK                           (0x01U << I2C_RDREQ_RDREQ_SHIFT)
#define I2C_RDREQ_RDREQ(x)                             (((uint32_t)(((uint32_t)(x)) << I2C_RDREQ_RDREQ_SHIFT)) & I2C_RDREQ_RDREQ_MASK)

/*!
 * @brief I2C_TXABRT Register Bit Definition
 */

#define I2C_TXABRT_TXABRT_SHIFT                        (0)
#define I2C_TXABRT_TXABRT_MASK                         (0x01U << I2C_TXABRT_TXABRT_SHIFT)
#define I2C_TXABRT_TXABRT(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_TXABRT_TXABRT_SHIFT)) & I2C_TXABRT_TXABRT_MASK)

/*!
 * @brief I2C_RXDONE Register Bit Definition
 */

#define I2C_RXDONE_RXDONE_SHIFT                        (0)
#define I2C_RXDONE_RXDONE_MASK                         (0x01U << I2C_RXDONE_RXDONE_SHIFT)
#define I2C_RXDONE_RXDONE(x)                           (((uint32_t)(((uint32_t)(x)) << I2C_RXDONE_RXDONE_SHIFT)) & I2C_RXDONE_RXDONE_MASK)

/*!
 * @brief I2C_ACTIV Register Bit Definition
 */

#define I2C_ACTIV_ACTIV_SHIFT                           (0)
#define I2C_ACTIV_ACTIV_MASK                            (0x01U << I2C_ACTIV_ACTIV_SHIFT)
#define I2C_ACTIV_ACTIV(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_ACTIV_ACTIV_SHIFT)) & I2C_ACTIV_ACTIV_MASK)

/*!
 * @brief I2C_STOP Register Bit Definition
 */

#define I2C_STOP_STOP_SHIFT                             (0)
#define I2C_STOP_STOP_MASK                              (0x01U << I2C_STOP_STOP_SHIFT)
#define I2C_STOP_STOP(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_STOP_STOP_SHIFT)) & I2C_STOP_STOP_MASK)

/*!
 * @brief I2C_START Register Bit Definition
 */

#define I2C_START_START_SHIFT                           (0)
#define I2C_START_START_MASK                            (0x01U << I2C_START_START_SHIFT)
#define I2C_START_START(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_START_START_SHIFT)) & I2C_START_START_MASK)

/*!
 * @brief I2C_GC Register Bit Definition
 */

#define I2C_GC_GC_SHIFT                                 (0)
#define I2C_GC_GC_MASK                                  (0x01U << I2C_GC_GC_SHIFT)
#define I2C_GC_GC(x)                                    (((uint32_t)(((uint32_t)(x)) << I2C_GC_GC_SHIFT)) & I2C_GC_GC_MASK)

/*!
 * @brief I2C_ENR Register Bit Definition
 */

#define I2C_ENR_ABORT_SHIFT                             (1)
#define I2C_ENR_ABORT_MASK                              (0x01U << I2C_ENR_ABORT_SHIFT)
#define I2C_ENR_ABORT(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_ENR_ABORT_SHIFT)) & I2C_ENR_ABORT_MASK)

#define I2C_ENR_ENABLE_SHIFT                            (0)
#define I2C_ENR_ENABLE_MASK                             (0x01U << I2C_ENR_ENABLE_SHIFT)
#define I2C_ENR_ENABLE(x)                               (((uint32_t)(((uint32_t)(x)) << I2C_ENR_ENABLE_SHIFT)) & I2C_ENR_ENABLE_MASK)

/*!
 * @brief I2C_SR Register Bit Definition
 */

#define I2C_SR_SLVACTIV_SHIFT                           (6)
#define I2C_SR_SLVACTIV_MASK                            (0x01U << I2C_SR_SLVACTIV_SHIFT)
#define I2C_SR_SLVACTIV(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_SR_SLVACTIV_SHIFT)) & I2C_SR_SLVACTIV_MASK)

#define I2C_SR_MSTACTIV_SHIFT                           (5)
#define I2C_SR_MSTACTIV_MASK                            (0x01U << I2C_SR_MSTACTIV_SHIFT)
#define I2C_SR_MSTACTIV(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_SR_MSTACTIV_SHIFT)) & I2C_SR_MSTACTIV_MASK)

#define I2C_SR_RFF_SHIFT                                (4)
#define I2C_SR_RFF_MASK                                 (0x01U << I2C_SR_RFF_SHIFT)
#define I2C_SR_RFF(x)                                   (((uint32_t)(((uint32_t)(x)) << I2C_SR_RFF_SHIFT)) & I2C_SR_RFF_MASK)

#define I2C_SR_RFNE_SHIFT                               (3)
#define I2C_SR_RFNE_MASK                                (0x01U << I2C_SR_RFNE_SHIFT)
#define I2C_SR_RFNE(x)                                  (((uint32_t)(((uint32_t)(x)) << I2C_SR_RFNE_SHIFT)) & I2C_SR_RFNE_MASK)

#define I2C_SR_TFE_SHIFT                                (2)
#define I2C_SR_TFE_MASK                                 (0x01U << I2C_SR_TFE_SHIFT)
#define I2C_SR_TFE(x)                                   (((uint32_t)(((uint32_t)(x)) << I2C_SR_TFE_SHIFT)) & I2C_SR_TFE_MASK)

#define I2C_SR_TFNF_SHIFT                               (1)
#define I2C_SR_TFNF_MASK                                (0x01U << I2C_SR_TFNF_SHIFT)
#define I2C_SR_TFNF(x)                                  (((uint32_t)(((uint32_t)(x)) << I2C_SR_TFNF_SHIFT)) & I2C_SR_TFNF_MASK)

#define I2C_SR_ACTIV_SHIFT                              (0)
#define I2C_SR_ACTIV_MASK                               (0x01U << I2C_SR_ACTIV_SHIFT)
#define I2C_SR_ACTIV(x)                                 (((uint32_t)(((uint32_t)(x)) << I2C_SR_ACTIV_SHIFT)) & I2C_SR_ACTIV_MASK)

/*!
 * @brief I2C_TXFLR Register Bit Definition
 */

#define I2C_TXFLR_CNT_SHIFT                             (0)
#define I2C_TXFLR_CNT_MASK                              (0x03U << I2C_TXFLR_CNT_SHIFT)
#define I2C_TXFLR_CNT(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_TXFLR_CNT_SHIFT)) & I2C_TXFLR_CNT_MASK)

/*!
 * @brief I2C_RXFLR Register Bit Definition
 */

#define I2C_RXFLR_CNT_SHIFT                             (0)
#define I2C_RXFLR_CNT_MASK                              (0x03U << I2C_RXFLR_CNT_SHIFT)
#define I2C_RXFLR_CNT(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_RXFLR_CNT_SHIFT)) & I2C_RXFLR_CNT_MASK)

/*!
 * @brief I2C_HOLD Register Bit Definition
 */

#define I2C_HOLD_RXHOLD_SHIFT                           (16)
#define I2C_HOLD_RXHOLD_MASK                            (0xFFU << I2C_HOLD_RXHOLD_SHIFT)
#define I2C_HOLD_RXHOLD(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_HOLD_RXHOLD_SHIFT)) & I2C_HOLD_RXHOLD_MASK)

#define I2C_HOLD_TXHOLD_SHIFT                           (0)
#define I2C_HOLD_TXHOLD_MASK                            (0xFFFFU << I2C_HOLD_TXHOLD_SHIFT)
#define I2C_HOLD_TXHOLD(x)                              (((uint32_t)(((uint32_t)(x)) << I2C_HOLD_TXHOLD_SHIFT)) & I2C_HOLD_TXHOLD_MASK)

/*!
 * @brief I2C_SETUP Register Bit Definition
 */

#define I2C_SETUP_CNT_SHIFT                             (0)
#define I2C_SETUP_CNT_MASK                              (0xFFU << I2C_SETUP_CNT_SHIFT)
#define I2C_SETUP_CNT(x)                                (((uint32_t)(((uint32_t)(x)) << I2C_SETUP_CNT_SHIFT)) & I2C_SETUP_CNT_MASK)

/*!
 * @brief I2C_GCR Register Bit Definition
 */

#define I2C_GCR_GC_SHIFT                                (0)
#define I2C_GCR_GC_MASK                                 (0x01U << I2C_GCR_GC_SHIFT)
#define I2C_GCR_GC(x)                                   (((uint32_t)(((uint32_t)(x)) << I2C_GCR_GC_SHIFT)) & I2C_GCR_GC_MASK)

/*!
 * @brief I2C_SLVMASK Register Bit Definition
 */

#define I2C_SLVMASK_MASK_SHIFT                          (0)
#define I2C_SLVMASK_MASK_MASK                           (0x3FFU << I2C_SLVMASK_MASK_SHIFT)
#define I2C_SLVMASK_MASK(x)                             (((uint32_t)(((uint32_t)(x)) << I2C_SLVMASK_MASK_SHIFT)) & I2C_SLVMASK_MASK_MASK)

/*!
 * @brief I2C_SLVRCVADDR Register Bit Definition
 */

#define I2C_SLVRCVADDR_ADDR_SHIFT                       (0)
#define I2C_SLVRCVADDR_ADDR_MASK                        (0x3FFU << I2C_SLVRCVADDR_ADDR_SHIFT)
#define I2C_SLVRCVADDR_ADDR(x)                          (((uint32_t)(((uint32_t)(x)) << I2C_SLVRCVADDR_ADDR_SHIFT)) & I2C_SLVRCVADDR_ADDR_MASK)

/*!
 * @}
 */ /* end of group I2C_Register_Masks */
/******************************************************************************
 * I2C Instance
 ******************************************************************************/

#define I2C1                ((I2C_Type*)I2C1_BASE)

/*!
 * @}
 */ /* end of group I2C_Peripheral_Access_Layer */

/*!
 * @addtogroup IWDG_Peripheral_Access_Layer IWDG Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * IWDG Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t KR;                                                             ///< Key register                                 offset: 0x00
    __IO uint32_t PR;                                                             ///< Prescaler register                           offset: 0x04
    __IO uint32_t RLR;                                                            ///< Reload register                              offset: 0x08
    __IO uint32_t SR;                                                             ///< Status register                              offset: 0x0c
    __IO uint32_t CR;                                                             ///< Control register                             offset: 0x10
    __IO uint32_t IGEN;                                                           ///< Interruput generate value register           offset: 0x14
    __IO uint32_t CNT;                                                            ///< Counter                                      offset: 0x18
} IWDG_Type;

/*!
 * @addtogroup IWDG_Register_Masks Register Masks
 * @{ */

/*!
 * @brief IWDG_KR Register Bit Definition
 */

#define IWDG_KR_KEY_SHIFT                               (0)
#define IWDG_KR_KEY_MASK                                (0xFFFFU << IWDG_KR_KEY_SHIFT)
#define IWDG_KR_KEY(x)                                  (((uint32_t)(((uint32_t)(x)) << IWDG_KR_KEY_SHIFT)) & IWDG_KR_KEY_MASK)

/*!
 * @brief IWDG_PR Register Bit Definition
 */

#define IWDG_PR_PR_SHIFT                                (0)
#define IWDG_PR_PR_MASK                                 (0x07U << IWDG_PR_PR_SHIFT)
#define IWDG_PR_PR(x)                                   (((uint32_t)(((uint32_t)(x)) << IWDG_PR_PR_SHIFT)) & IWDG_PR_PR_MASK)

/*!
 * @brief IWDG_RLR Register Bit Definition
 */

#define IWDG_RLR_RL_SHIFT                               (0)
#define IWDG_RLR_RL_MASK                                (0xFFFU << IWDG_RLR_RL_SHIFT)
#define IWDG_RLR_RL(x)                                  (((uint32_t)(((uint32_t)(x)) << IWDG_RLR_RL_SHIFT)) & IWDG_RLR_RL_MASK)

/*!
 * @brief IWDG_SR Register Bit Definition
 */

#define IWDG_SR_UPDATE_SHIFT                            (3)
#define IWDG_SR_UPDATE_MASK                             (0x01U << IWDG_SR_UPDATE_SHIFT)
#define IWDG_SR_UPDATE(x)                               (((uint32_t)(((uint32_t)(x)) << IWDG_SR_UPDATE_SHIFT)) & IWDG_SR_UPDATE_MASK)

#define IWDG_SR_IVU_SHIFT                               (2)
#define IWDG_SR_IVU_MASK                                (0x01U << IWDG_SR_IVU_SHIFT)
#define IWDG_SR_IVU(x)                                  (((uint32_t)(((uint32_t)(x)) << IWDG_SR_IVU_SHIFT)) & IWDG_SR_IVU_MASK)

#define IWDG_SR_RVU_SHIFT                               (1)
#define IWDG_SR_RVU_MASK                                (0x01U << IWDG_SR_RVU_SHIFT)
#define IWDG_SR_RVU(x)                                  (((uint32_t)(((uint32_t)(x)) << IWDG_SR_RVU_SHIFT)) & IWDG_SR_RVU_MASK)

#define IWDG_SR_PVU_SHIFT                               (0)
#define IWDG_SR_PVU_MASK                                (0x01U << IWDG_SR_PVU_SHIFT)
#define IWDG_SR_PVU(x)                                  (((uint32_t)(((uint32_t)(x)) << IWDG_SR_PVU_SHIFT)) & IWDG_SR_PVU_MASK)

/*!
 * @brief IWDG_CR Register Bit Definition
 */

#define IWDG_CR_IRQCLR_SHIFT                            (1)
#define IWDG_CR_IRQCLR_MASK                             (0x01U << IWDG_CR_IRQCLR_SHIFT)
#define IWDG_CR_IRQCLR(x)                               (((uint32_t)(((uint32_t)(x)) << IWDG_CR_IRQCLR_SHIFT)) & IWDG_CR_IRQCLR_MASK)

#define IWDG_CR_IRQSEL_SHIFT                            (0)
#define IWDG_CR_IRQSEL_MASK                             (0x01U << IWDG_CR_IRQSEL_SHIFT)
#define IWDG_CR_IRQSEL(x)                               (((uint32_t)(((uint32_t)(x)) << IWDG_CR_IRQSEL_SHIFT)) & IWDG_CR_IRQSEL_MASK)

/*!
 * @brief IWDG_IGEN Register Bit Definition
 */

#define IWDG_IGEN_IGEN_SHIFT                            (0)
#define IWDG_IGEN_IGEN_MASK                             (0xFFFU << IWDG_IGEN_IGEN_SHIFT)
#define IWDG_IGEN_IGEN(x)                               (((uint32_t)(((uint32_t)(x)) << IWDG_IGEN_IGEN_SHIFT)) & IWDG_IGEN_IGEN_MASK)

/*!
 * @brief IWDG_CNT Register Bit Definition
 */

#define IWDG_CNT_CNT_SHIFT                              (8)
#define IWDG_CNT_CNT_MASK                               (0x7FFU << IWDG_CNT_CNT_SHIFT)
#define IWDG_CNT_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << IWDG_CNT_CNT_SHIFT)) & IWDG_CNT_CNT_MASK)

#define IWDG_CNT_PS_SHIFT                               (0)
#define IWDG_CNT_PS_MASK                                (0xFFU << IWDG_CNT_PS_SHIFT)
#define IWDG_CNT_PS(x)                                  (((uint32_t)(((uint32_t)(x)) << IWDG_CNT_PS_SHIFT)) & IWDG_CNT_PS_MASK)

/*!
 * @}
 */ /* end of group IWDG_Register_Masks */
/******************************************************************************
 * IWDG Instance
 ******************************************************************************/

#define IWDG                ((IWDG_Type*)IWDG_BASE)

/*!
 * @}
 */ /* end of group IWDG_Peripheral_Access_Layer */

/*!
 * @addtogroup PWR_Peripheral_Access_Layer PWR Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * PWR Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CR;                                                             ///< CR                                           offset: 0x00
    __IO uint32_t CSR;                                                            ///< CSR                                          offset: 0x04
} PWR_Type;

/*!
 * @addtogroup PWR_Register_Masks Register Masks
 * @{ */

/*!
 * @brief PWR_CR Register Bit Definition
 */

#define PWR_CR_PLS_SHIFT                                (9)
#define PWR_CR_PLS_MASK                                 (0x0FU << PWR_CR_PLS_SHIFT)
#define PWR_CR_PLS(x)                                   (((uint32_t)(((uint32_t)(x)) << PWR_CR_PLS_SHIFT)) & PWR_CR_PLS_MASK)

#define PWR_CR_PVDE_SHIFT                               (4)
#define PWR_CR_PVDE_MASK                                (0x01U << PWR_CR_PVDE_SHIFT)
#define PWR_CR_PVDE(x)                                  (((uint32_t)(((uint32_t)(x)) << PWR_CR_PVDE_SHIFT)) & PWR_CR_PVDE_MASK)

#define PWR_CR_LPDS_SHIFT                               (1)
#define PWR_CR_LPDS_MASK                                (0x01U << PWR_CR_LPDS_SHIFT)
#define PWR_CR_LPDS(x)                                  (((uint32_t)(((uint32_t)(x)) << PWR_CR_LPDS_SHIFT)) & PWR_CR_LPDS_MASK)

/*!
 * @brief PWR_CSR Register Bit Definition
 */

#define PWR_CSR_PVDO_SHIFT                              (2)
#define PWR_CSR_PVDO_MASK                               (0x01U << PWR_CSR_PVDO_SHIFT)
#define PWR_CSR_PVDO(x)                                 (((uint32_t)(((uint32_t)(x)) << PWR_CSR_PVDO_SHIFT)) & PWR_CSR_PVDO_MASK)

/*!
 * @}
 */ /* end of group PWR_Register_Masks */
/******************************************************************************
 * PWR Instance
 ******************************************************************************/

#define PWR                ((PWR_Type*)PWR_BASE)

/*!
 * @}
 */ /* end of group PWR_Peripheral_Access_Layer */

/*!
 * @addtogroup RCC_Peripheral_Access_Layer RCC Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * RCC Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CR;                                                             ///< Control Register                             offset: 0x00
    __IO uint32_t CFGR;                                                           ///< Configuration Register                       offset: 0x04
    __IO uint32_t CIR;                                                            ///< Clock Interrupt Register                     offset: 0x08
    __IO uint32_t Reserved0[1];                                                   ///< Reserved
    __IO uint32_t APB1RSTR;                                                       ///< Advanced Peripheral Bus 1 Reset Register     offset: 0x10
    __IO uint32_t AHBENR;                                                         ///< Advanced High Performance Bus Enable Registeroffset: 0x14
    __IO uint32_t Reserved1[1];                                                   ///< Reserved
    __IO uint32_t APB1ENR;                                                        ///< Advanced Peripheral Bus 1 Enable Register    offset: 0x1C
    __IO uint32_t Reserved2[1];                                                   ///< Reserved
    __IO uint32_t CSR;                                                            ///< Control Status Register                      offset: 0x24
    __IO uint32_t AHBRSTR;                                                        ///< Advanced High Performance Bus Reset Register offset: 0x28
    __IO uint32_t Reserved3[5];                                                   ///< Reserved
    __IO uint32_t SYSCFGR;                                                         ///< System Configuration Register                offset: 0x40
} RCC_Type;

/*!
 * @addtogroup RCC_Register_Masks Register Masks
 * @{ */

/*!
 * @brief RCC_CR Register Bit Definition
 */

#define RCC_CR_HSERDY_SHIFT                             (17)
#define RCC_CR_HSERDY_MASK                              (0x01U << RCC_CR_HSERDY_SHIFT)
#define RCC_CR_HSERDY(x)                                (((uint32_t)(((uint32_t)(x)) << RCC_CR_HSERDY_SHIFT)) & RCC_CR_HSERDY_MASK)

#define RCC_CR_HSEON_SHIFT                              (16)
#define RCC_CR_HSEON_MASK                               (0x01U << RCC_CR_HSEON_SHIFT)
#define RCC_CR_HSEON(x)                                 (((uint32_t)(((uint32_t)(x)) << RCC_CR_HSEON_SHIFT)) & RCC_CR_HSEON_MASK)

#define RCC_CR_HSIRDY_SHIFT                             (1)
#define RCC_CR_HSIRDY_MASK                              (0x01U << RCC_CR_HSIRDY_SHIFT)
#define RCC_CR_HSIRDY(x)                                (((uint32_t)(((uint32_t)(x)) << RCC_CR_HSIRDY_SHIFT)) & RCC_CR_HSIRDY_MASK)

#define RCC_CR_HSION_SHIFT                              (0)
#define RCC_CR_HSION_MASK                               (0x01U << RCC_CR_HSION_SHIFT)
#define RCC_CR_HSION(x)                                 (((uint32_t)(((uint32_t)(x)) << RCC_CR_HSION_SHIFT)) & RCC_CR_HSION_MASK)

/*!
 * @brief RCC_CFGR Register Bit Definition
 */

#define RCC_CFGR_MCO_SHIFT                              (24)
#define RCC_CFGR_MCO_MASK                               (0x07U << RCC_CFGR_MCO_SHIFT)
#define RCC_CFGR_MCO(x)                                 (((uint32_t)(((uint32_t)(x)) << RCC_CFGR_MCO_SHIFT)) & RCC_CFGR_MCO_MASK)

#define RCC_CFGR_PPRE1_SHIFT                            (8)
#define RCC_CFGR_PPRE1_MASK                             (0x07U << RCC_CFGR_PPRE1_SHIFT)
#define RCC_CFGR_PPRE1(x)                               (((uint32_t)(((uint32_t)(x)) << RCC_CFGR_PPRE1_SHIFT)) & RCC_CFGR_PPRE1_MASK)

#define RCC_CFGR_HPRE_SHIFT                             (4)
#define RCC_CFGR_HPRE_MASK                              (0x0FU << RCC_CFGR_HPRE_SHIFT)
#define RCC_CFGR_HPRE(x)                                (((uint32_t)(((uint32_t)(x)) << RCC_CFGR_HPRE_SHIFT)) & RCC_CFGR_HPRE_MASK)

#define RCC_CFGR_SWS_SHIFT                              (2)
#define RCC_CFGR_SWS_MASK                               (0x03U << RCC_CFGR_SWS_SHIFT)
#define RCC_CFGR_SWS(x)                                 (((uint32_t)(((uint32_t)(x)) << RCC_CFGR_SWS_SHIFT)) & RCC_CFGR_SWS_MASK)

#define RCC_CFGR_SW_SHIFT                               (0)
#define RCC_CFGR_SW_MASK                                (0x03U << RCC_CFGR_SW_SHIFT)
#define RCC_CFGR_SW(x)                                  (((uint32_t)(((uint32_t)(x)) << RCC_CFGR_SW_SHIFT)) & RCC_CFGR_SW_MASK)

/*!
 * @brief RCC_CIR Register Bit Definition
 */

#define RCC_CIR_HSERDYC_SHIFT                           (19)
#define RCC_CIR_HSERDYC_MASK                            (0x01U << RCC_CIR_HSERDYC_SHIFT)
#define RCC_CIR_HSERDYC(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CIR_HSERDYC_SHIFT)) & RCC_CIR_HSERDYC_MASK)

#define RCC_CIR_HSIRDYC_SHIFT                           (18)
#define RCC_CIR_HSIRDYC_MASK                            (0x01U << RCC_CIR_HSIRDYC_SHIFT)
#define RCC_CIR_HSIRDYC(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CIR_HSIRDYC_SHIFT)) & RCC_CIR_HSIRDYC_MASK)

#define RCC_CIR_LSIRDYC_SHIFT                           (16)
#define RCC_CIR_LSIRDYC_MASK                            (0x01U << RCC_CIR_LSIRDYC_SHIFT)
#define RCC_CIR_LSIRDYC(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CIR_LSIRDYC_SHIFT)) & RCC_CIR_LSIRDYC_MASK)

#define RCC_CIR_HSERDYIE_SHIFT                          (11)
#define RCC_CIR_HSERDYIE_MASK                           (0x01U << RCC_CIR_HSERDYIE_SHIFT)
#define RCC_CIR_HSERDYIE(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_CIR_HSERDYIE_SHIFT)) & RCC_CIR_HSERDYIE_MASK)

#define RCC_CIR_HSIRDYIE_SHIFT                          (10)
#define RCC_CIR_HSIRDYIE_MASK                           (0x01U << RCC_CIR_HSIRDYIE_SHIFT)
#define RCC_CIR_HSIRDYIE(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_CIR_HSIRDYIE_SHIFT)) & RCC_CIR_HSIRDYIE_MASK)

#define RCC_CIR_LSIRDYIE_SHIFT                          (8)
#define RCC_CIR_LSIRDYIE_MASK                           (0x01U << RCC_CIR_LSIRDYIE_SHIFT)
#define RCC_CIR_LSIRDYIE(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_CIR_LSIRDYIE_SHIFT)) & RCC_CIR_LSIRDYIE_MASK)

#define RCC_CIR_HSERDYF_SHIFT                           (3)
#define RCC_CIR_HSERDYF_MASK                            (0x01U << RCC_CIR_HSERDYF_SHIFT)
#define RCC_CIR_HSERDYF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CIR_HSERDYF_SHIFT)) & RCC_CIR_HSERDYF_MASK)

#define RCC_CIR_HSIRDYF_SHIFT                           (2)
#define RCC_CIR_HSIRDYF_MASK                            (0x01U << RCC_CIR_HSIRDYF_SHIFT)
#define RCC_CIR_HSIRDYF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CIR_HSIRDYF_SHIFT)) & RCC_CIR_HSIRDYF_MASK)

#define RCC_CIR_LSIRDYF_SHIFT                           (0)
#define RCC_CIR_LSIRDYF_MASK                            (0x01U << RCC_CIR_LSIRDYF_SHIFT)
#define RCC_CIR_LSIRDYF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CIR_LSIRDYF_SHIFT)) & RCC_CIR_LSIRDYF_MASK)

/*!
 * @brief RCC_APB1RSTR Register Bit Definition
 */

#define RCC_APB1RSTR_SYSCFG_SHIFT                       (30)
#define RCC_APB1RSTR_SYSCFG_MASK                        (0x01U << RCC_APB1RSTR_SYSCFG_SHIFT)
#define RCC_APB1RSTR_SYSCFG(x)                          (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_SYSCFG_SHIFT)) & RCC_APB1RSTR_SYSCFG_MASK)

#define RCC_APB1RSTR_DBGMCU_SHIFT                       (29)
#define RCC_APB1RSTR_DBGMCU_MASK                        (0x01U << RCC_APB1RSTR_DBGMCU_SHIFT)
#define RCC_APB1RSTR_DBGMCU(x)                          (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_DBGMCU_SHIFT)) & RCC_APB1RSTR_DBGMCU_MASK)

#define RCC_APB1RSTR_PWR_SHIFT                          (28)
#define RCC_APB1RSTR_PWR_MASK                           (0x01U << RCC_APB1RSTR_PWR_SHIFT)
#define RCC_APB1RSTR_PWR(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_PWR_SHIFT)) & RCC_APB1RSTR_PWR_MASK)

#define RCC_APB1RSTR_I2C1_SHIFT                         (21)
#define RCC_APB1RSTR_I2C1_MASK                          (0x01U << RCC_APB1RSTR_I2C1_SHIFT)
#define RCC_APB1RSTR_I2C1(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_I2C1_SHIFT)) & RCC_APB1RSTR_I2C1_MASK)

#define RCC_APB1RSTR_USART2_SHIFT                       (17)
#define RCC_APB1RSTR_USART2_MASK                        (0x01U << RCC_APB1RSTR_USART2_SHIFT)
#define RCC_APB1RSTR_USART2(x)                          (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_USART2_SHIFT)) & RCC_APB1RSTR_USART2_MASK)

#define RCC_APB1RSTR_USART1_SHIFT                       (16)
#define RCC_APB1RSTR_USART1_MASK                        (0x01U << RCC_APB1RSTR_USART1_SHIFT)
#define RCC_APB1RSTR_USART1(x)                          (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_USART1_SHIFT)) & RCC_APB1RSTR_USART1_MASK)

#define RCC_APB1RSTR_SPI1_SHIFT                         (12)
#define RCC_APB1RSTR_SPI1_MASK                          (0x01U << RCC_APB1RSTR_SPI1_SHIFT)
#define RCC_APB1RSTR_SPI1(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_SPI1_SHIFT)) & RCC_APB1RSTR_SPI1_MASK)

#define RCC_APB1RSTR_ADC1_SHIFT                         (9)
#define RCC_APB1RSTR_ADC1_MASK                          (0x01U << RCC_APB1RSTR_ADC1_SHIFT)
#define RCC_APB1RSTR_ADC1(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_ADC1_SHIFT)) & RCC_APB1RSTR_ADC1_MASK)

#define RCC_APB1RSTR_TIM14_SHIFT                        (3)
#define RCC_APB1RSTR_TIM14_MASK                         (0x01U << RCC_APB1RSTR_TIM14_SHIFT)
#define RCC_APB1RSTR_TIM14(x)                           (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_TIM14_SHIFT)) & RCC_APB1RSTR_TIM14_MASK)

#define RCC_APB1RSTR_TIM1_SHIFT                         (2)
#define RCC_APB1RSTR_TIM1_MASK                          (0x01U << RCC_APB1RSTR_TIM1_SHIFT)
#define RCC_APB1RSTR_TIM1(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_TIM1_SHIFT)) & RCC_APB1RSTR_TIM1_MASK)

#define RCC_APB1RSTR_TIM3_SHIFT                         (1)
#define RCC_APB1RSTR_TIM3_MASK                          (0x01U << RCC_APB1RSTR_TIM3_SHIFT)
#define RCC_APB1RSTR_TIM3(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_APB1RSTR_TIM3_SHIFT)) & RCC_APB1RSTR_TIM3_MASK)

/*!
 * @brief RCC_AHBENR Register Bit Definition
 */

#define RCC_AHBENR_GPIOB_SHIFT                          (18)
#define RCC_AHBENR_GPIOB_MASK                           (0x01U << RCC_AHBENR_GPIOB_SHIFT)
#define RCC_AHBENR_GPIOB(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_AHBENR_GPIOB_SHIFT)) & RCC_AHBENR_GPIOB_MASK)

#define RCC_AHBENR_GPIOA_SHIFT                          (17)
#define RCC_AHBENR_GPIOA_MASK                           (0x01U << RCC_AHBENR_GPIOA_SHIFT)
#define RCC_AHBENR_GPIOA(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_AHBENR_GPIOA_SHIFT)) & RCC_AHBENR_GPIOA_MASK)

#define RCC_AHBENR_CRC_SHIFT                            (6)
#define RCC_AHBENR_CRC_MASK                             (0x01U << RCC_AHBENR_CRC_SHIFT)
#define RCC_AHBENR_CRC(x)                               (((uint32_t)(((uint32_t)(x)) << RCC_AHBENR_CRC_SHIFT)) & RCC_AHBENR_CRC_MASK)

#define RCC_AHBENR_FLASH_SHIFT                          (4)
#define RCC_AHBENR_FLASH_MASK                           (0x01U << RCC_AHBENR_FLASH_SHIFT)
#define RCC_AHBENR_FLASH(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_AHBENR_FLASH_SHIFT)) & RCC_AHBENR_FLASH_MASK)

#define RCC_AHBENR_SRAM_SHIFT                           (2)
#define RCC_AHBENR_SRAM_MASK                            (0x01U << RCC_AHBENR_SRAM_SHIFT)
#define RCC_AHBENR_SRAM(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_AHBENR_SRAM_SHIFT)) & RCC_AHBENR_SRAM_MASK)

/*!
 * @brief RCC_APB1ENR Register Bit Definition
 */

#define RCC_APB1ENR_SYSCFG_SHIFT                        (30)
#define RCC_APB1ENR_SYSCFG_MASK                         (0x01U << RCC_APB1ENR_SYSCFG_SHIFT)
#define RCC_APB1ENR_SYSCFG(x)                           (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_SYSCFG_SHIFT)) & RCC_APB1ENR_SYSCFG_MASK)

#define RCC_APB1ENR_DBGMCU_SHIFT                        (29)
#define RCC_APB1ENR_DBGMCU_MASK                         (0x01U << RCC_APB1ENR_DBGMCU_SHIFT)
#define RCC_APB1ENR_DBGMCU(x)                           (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_DBGMCU_SHIFT)) & RCC_APB1ENR_DBGMCU_MASK)

#define RCC_APB1ENR_PWR_SHIFT                           (28)
#define RCC_APB1ENR_PWR_MASK                            (0x01U << RCC_APB1ENR_PWR_SHIFT)
#define RCC_APB1ENR_PWR(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_PWR_SHIFT)) & RCC_APB1ENR_PWR_MASK)

#define RCC_APB1ENR_I2C1_SHIFT                          (21)
#define RCC_APB1ENR_I2C1_MASK                           (0x01U << RCC_APB1ENR_I2C1_SHIFT)
#define RCC_APB1ENR_I2C1(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_I2C1_SHIFT)) & RCC_APB1ENR_I2C1_MASK)

#define RCC_APB1ENR_USART2_SHIFT                        (17)
#define RCC_APB1ENR_USART2_MASK                         (0x01U << RCC_APB1ENR_USART2_SHIFT)
#define RCC_APB1ENR_USART2(x)                           (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_USART2_SHIFT)) & RCC_APB1ENR_USART2_MASK)

#define RCC_APB1ENR_USART1_SHIFT                        (16)
#define RCC_APB1ENR_USART1_MASK                         (0x01U << RCC_APB1ENR_USART1_SHIFT)
#define RCC_APB1ENR_USART1(x)                           (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_USART1_SHIFT)) & RCC_APB1ENR_USART1_MASK)

#define RCC_APB1ENR_SPI1_SHIFT                          (12)
#define RCC_APB1ENR_SPI1_MASK                           (0x01U << RCC_APB1ENR_SPI1_SHIFT)
#define RCC_APB1ENR_SPI1(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_SPI1_SHIFT)) & RCC_APB1ENR_SPI1_MASK)

#define RCC_APB1ENR_ADC1_SHIFT                          (9)
#define RCC_APB1ENR_ADC1_MASK                           (0x01U << RCC_APB1ENR_ADC1_SHIFT)
#define RCC_APB1ENR_ADC1(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_ADC1_SHIFT)) & RCC_APB1ENR_ADC1_MASK)

#define RCC_APB1ENR_TIM14_SHIFT                         (3)
#define RCC_APB1ENR_TIM14_MASK                          (0x01U << RCC_APB1ENR_TIM14_SHIFT)
#define RCC_APB1ENR_TIM14(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_TIM14_SHIFT)) & RCC_APB1ENR_TIM14_MASK)

#define RCC_APB1ENR_TIM1_SHIFT                          (2)
#define RCC_APB1ENR_TIM1_MASK                           (0x01U << RCC_APB1ENR_TIM1_SHIFT)
#define RCC_APB1ENR_TIM1(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_TIM1_SHIFT)) & RCC_APB1ENR_TIM1_MASK)

#define RCC_APB1ENR_TIM3_SHIFT                          (1)
#define RCC_APB1ENR_TIM3_MASK                           (0x01U << RCC_APB1ENR_TIM3_SHIFT)
#define RCC_APB1ENR_TIM3(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_APB1ENR_TIM3_SHIFT)) & RCC_APB1ENR_TIM3_MASK)

/*!
 * @brief RCC_CSR Register Bit Definition
 */

#define RCC_CSR_IWDGRSTF_SHIFT                          (29)
#define RCC_CSR_IWDGRSTF_MASK                           (0x01U << RCC_CSR_IWDGRSTF_SHIFT)
#define RCC_CSR_IWDGRSTF(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_CSR_IWDGRSTF_SHIFT)) & RCC_CSR_IWDGRSTF_MASK)

#define RCC_CSR_SFTRSTF_SHIFT                           (28)
#define RCC_CSR_SFTRSTF_MASK                            (0x01U << RCC_CSR_SFTRSTF_SHIFT)
#define RCC_CSR_SFTRSTF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CSR_SFTRSTF_SHIFT)) & RCC_CSR_SFTRSTF_MASK)

#define RCC_CSR_PORRSTF_SHIFT                           (27)
#define RCC_CSR_PORRSTF_MASK                            (0x01U << RCC_CSR_PORRSTF_SHIFT)
#define RCC_CSR_PORRSTF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CSR_PORRSTF_SHIFT)) & RCC_CSR_PORRSTF_MASK)

#define RCC_CSR_PINRSTF_SHIFT                           (26)
#define RCC_CSR_PINRSTF_MASK                            (0x01U << RCC_CSR_PINRSTF_SHIFT)
#define RCC_CSR_PINRSTF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CSR_PINRSTF_SHIFT)) & RCC_CSR_PINRSTF_MASK)

#define RCC_CSR_RMVF_SHIFT                              (24)
#define RCC_CSR_RMVF_MASK                               (0x01U << RCC_CSR_RMVF_SHIFT)
#define RCC_CSR_RMVF(x)                                 (((uint32_t)(((uint32_t)(x)) << RCC_CSR_RMVF_SHIFT)) & RCC_CSR_RMVF_MASK)

#define RCC_CSR_LOCKUPF_SHIFT                           (23)
#define RCC_CSR_LOCKUPF_MASK                            (0x01U << RCC_CSR_LOCKUPF_SHIFT)
#define RCC_CSR_LOCKUPF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CSR_LOCKUPF_SHIFT)) & RCC_CSR_LOCKUPF_MASK)

#define RCC_CSR_PVDRSTF_SHIFT                           (22)
#define RCC_CSR_PVDRSTF_MASK                            (0x01U << RCC_CSR_PVDRSTF_SHIFT)
#define RCC_CSR_PVDRSTF(x)                              (((uint32_t)(((uint32_t)(x)) << RCC_CSR_PVDRSTF_SHIFT)) & RCC_CSR_PVDRSTF_MASK)

#define RCC_CSR_LOCKUPEN_SHIFT                          (7)
#define RCC_CSR_LOCKUPEN_MASK                           (0x01U << RCC_CSR_LOCKUPEN_SHIFT)
#define RCC_CSR_LOCKUPEN(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_CSR_LOCKUPEN_SHIFT)) & RCC_CSR_LOCKUPEN_MASK)

#define RCC_CSR_PVDRSTEN_SHIFT                          (6)
#define RCC_CSR_PVDRSTEN_MASK                           (0x01U << RCC_CSR_PVDRSTEN_SHIFT)
#define RCC_CSR_PVDRSTEN(x)                             (((uint32_t)(((uint32_t)(x)) << RCC_CSR_PVDRSTEN_SHIFT)) & RCC_CSR_PVDRSTEN_MASK)

#define RCC_CSR_LSIRDY_SHIFT                            (1)
#define RCC_CSR_LSIRDY_MASK                             (0x01U << RCC_CSR_LSIRDY_SHIFT)
#define RCC_CSR_LSIRDY(x)                               (((uint32_t)(((uint32_t)(x)) << RCC_CSR_LSIRDY_SHIFT)) & RCC_CSR_LSIRDY_MASK)

#define RCC_CSR_LSION_SHIFT                             (0)
#define RCC_CSR_LSION_MASK                              (0x01U << RCC_CSR_LSION_SHIFT)
#define RCC_CSR_LSION(x)                                (((uint32_t)(((uint32_t)(x)) << RCC_CSR_LSION_SHIFT)) & RCC_CSR_LSION_MASK)

/*!
 * @brief RCC_AHBRSTR Register Bit Definition
 */

#define RCC_AHBRSTR_GPIOB_SHIFT                         (18)
#define RCC_AHBRSTR_GPIOB_MASK                          (0x01U << RCC_AHBRSTR_GPIOB_SHIFT)
#define RCC_AHBRSTR_GPIOB(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_AHBRSTR_GPIOB_SHIFT)) & RCC_AHBRSTR_GPIOB_MASK)

#define RCC_AHBRSTR_GPIOA_SHIFT                         (17)
#define RCC_AHBRSTR_GPIOA_MASK                          (0x01U << RCC_AHBRSTR_GPIOA_SHIFT)
#define RCC_AHBRSTR_GPIOA(x)                            (((uint32_t)(((uint32_t)(x)) << RCC_AHBRSTR_GPIOA_SHIFT)) & RCC_AHBRSTR_GPIOA_MASK)

/*!
 * @brief RCC_SYSCFGR Register Bit Definition
 */

#define RCC_SYSCFGR_SFTNRSTRMP_SHIFT                     (2)
#define RCC_SYSCFGR_SFTNRSTRMP_MASK                      (0x01U << RCC_SYSCFGR_SFTNRSTRMP_SHIFT)
#define RCC_SYSCFGR_SFTNRSTRMP(x)                        (((uint32_t)(((uint32_t)(x)) << RCC_SYSCFGR_SFTNRSTRMP_SHIFT)) & RCC_SYSCFGR_SFTNRSTRMP_MASK)

#define RCC_SYSCFGR_SECTOR1KCFG_SHIFT                    (1)
#define RCC_SYSCFGR_SECTOR1KCFG_MASK                     (0x01U << RCC_SYSCFGR_SECTOR1KCFG_SHIFT)
#define RCC_SYSCFGR_SECTOR1KCFG(x)                       (((uint32_t)(((uint32_t)(x)) << RCC_SYSCFGR_SECTOR1KCFG_SHIFT)) & RCC_SYSCFGR_SECTOR1KCFG_MASK)

#define RCC_SYSCFGR_PROGCHECKEN_SHIFT                    (0)
#define RCC_SYSCFGR_PROGCHECKEN_MASK                     (0x01U << RCC_SYSCFGR_PROGCHECKEN_SHIFT)
#define RCC_SYSCFGR_PROGCHECKEN(x)                       (((uint32_t)(((uint32_t)(x)) << RCC_SYSCFGR_PROGCHECKEN_SHIFT)) & RCC_SYSCFGR_PROGCHECKEN_MASK)

/*!
 * @}
 */ /* end of group RCC_Register_Masks */
/******************************************************************************
 * RCC Instance
 ******************************************************************************/

#define RCC                ((RCC_Type*)RCC_BASE)

/*!
 * @}
 */ /* end of group RCC_Peripheral_Access_Layer */

/*!
 * @addtogroup SPI_Peripheral_Access_Layer SPI Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * SPI Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t TXREG;                                                          ///< TXREG                                        offset: 0x00
    __IO uint32_t RXREG;                                                          ///< RXREG                                        offset: 0x04
    __IO uint32_t CSTAT;                                                          ///< CSTAT                                        offset: 0x08
    __IO uint32_t INTSTAT;                                                        ///< INTSTAT                                      offset: 0x0C
    __IO uint32_t INTEN;                                                          ///< INTEN                                        offset: 0x10
    __IO uint32_t INTCLR;                                                         ///< INTCLR                                       offset: 0x14
    __IO uint32_t GCTL;                                                           ///< GCTL                                         offset: 0x18
    __IO uint32_t CCTL;                                                           ///< CCTL                                         offset: 0x1C
    __IO uint32_t SPBRG;                                                          ///< SPBRG                                        offset: 0x20
    __IO uint32_t RXDNR;                                                          ///< RXDNR                                        offset: 0x24
    __IO uint32_t NSSR;                                                           ///< NSSR                                         offset: 0x28
    __IO uint32_t EXTCTL;                                                         ///< EXTCTL                                       offset: 0x2C
} SPI_Type;

/*!
 * @addtogroup SPI_Register_Masks Register Masks
 * @{ */

/*!
 * @brief SPI_TXREG Register Bit Definition
 */

#define SPI_TXREG_TXREG_SHIFT                           (0)
#define SPI_TXREG_TXREG_MASK                            (0xFFFFFFFFU << SPI_TXREG_TXREG_SHIFT)
#define SPI_TXREG_TXREG(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_TXREG_TXREG_SHIFT)) & SPI_TXREG_TXREG_MASK)

/*!
 * @brief SPI_RXREG Register Bit Definition
 */

#define SPI_RXREG_RXREG_SHIFT                           (0)
#define SPI_RXREG_RXREG_MASK                            (0xFFFFFFFFU << SPI_RXREG_RXREG_SHIFT)
#define SPI_RXREG_RXREG(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_RXREG_RXREG_SHIFT)) & SPI_RXREG_RXREG_MASK)

/*!
 * @brief SPI_CSTAT Register Bit Definition
 */

#define SPI_CSTAT_RXFADDR_SHIFT                         (8)
#define SPI_CSTAT_RXFADDR_MASK                          (0x07U << SPI_CSTAT_RXFADDR_SHIFT)
#define SPI_CSTAT_RXFADDR(x)                            (((uint32_t)(((uint32_t)(x)) << SPI_CSTAT_RXFADDR_SHIFT)) & SPI_CSTAT_RXFADDR_MASK)

#define SPI_CSTAT_TXFADDR_SHIFT                         (4)
#define SPI_CSTAT_TXFADDR_MASK                          (0x07U << SPI_CSTAT_TXFADDR_SHIFT)
#define SPI_CSTAT_TXFADDR(x)                            (((uint32_t)(((uint32_t)(x)) << SPI_CSTAT_TXFADDR_SHIFT)) & SPI_CSTAT_TXFADDR_MASK)

#define SPI_CSTAT_RXAVL4BYTE_SHIFT                      (3)
#define SPI_CSTAT_RXAVL4BYTE_MASK                       (0x01U << SPI_CSTAT_RXAVL4BYTE_SHIFT)
#define SPI_CSTAT_RXAVL4BYTE(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_CSTAT_RXAVL4BYTE_SHIFT)) & SPI_CSTAT_RXAVL4BYTE_MASK)

#define SPI_CSTAT_TXFULL_SHIFT                          (2)
#define SPI_CSTAT_TXFULL_MASK                           (0x01U << SPI_CSTAT_TXFULL_SHIFT)
#define SPI_CSTAT_TXFULL(x)                             (((uint32_t)(((uint32_t)(x)) << SPI_CSTAT_TXFULL_SHIFT)) & SPI_CSTAT_TXFULL_MASK)

#define SPI_CSTAT_RXAVL_SHIFT                           (1)
#define SPI_CSTAT_RXAVL_MASK                            (0x01U << SPI_CSTAT_RXAVL_SHIFT)
#define SPI_CSTAT_RXAVL(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_CSTAT_RXAVL_SHIFT)) & SPI_CSTAT_RXAVL_MASK)

#define SPI_CSTAT_TXEPT_SHIFT                           (0)
#define SPI_CSTAT_TXEPT_MASK                            (0x01U << SPI_CSTAT_TXEPT_SHIFT)
#define SPI_CSTAT_TXEPT(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_CSTAT_TXEPT_SHIFT)) & SPI_CSTAT_TXEPT_MASK)

/*!
 * @brief SPI_INTSTAT Register Bit Definition
 */

#define SPI_INTSTAT_TXEPTINTF_SHIFT                     (6)
#define SPI_INTSTAT_TXEPTINTF_MASK                      (0x01U << SPI_INTSTAT_TXEPTINTF_SHIFT)
#define SPI_INTSTAT_TXEPTINTF(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_TXEPTINTF_SHIFT)) & SPI_INTSTAT_TXEPTINTF_MASK)

#define SPI_INTSTAT_RXFULLINTF_SHIFT                    (5)
#define SPI_INTSTAT_RXFULLINTF_MASK                     (0x01U << SPI_INTSTAT_RXFULLINTF_SHIFT)
#define SPI_INTSTAT_RXFULLINTF(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_RXFULLINTF_SHIFT)) & SPI_INTSTAT_RXFULLINTF_MASK)

#define SPI_INTSTAT_RXMATCHINTF_SHIFT                   (4)
#define SPI_INTSTAT_RXMATCHINTF_MASK                    (0x01U << SPI_INTSTAT_RXMATCHINTF_SHIFT)
#define SPI_INTSTAT_RXMATCHINTF(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_RXMATCHINTF_SHIFT)) & SPI_INTSTAT_RXMATCHINTF_MASK)

#define SPI_INTSTAT_RXOERRINTF_SHIFT                    (3)
#define SPI_INTSTAT_RXOERRINTF_MASK                     (0x01U << SPI_INTSTAT_RXOERRINTF_SHIFT)
#define SPI_INTSTAT_RXOERRINTF(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_RXOERRINTF_SHIFT)) & SPI_INTSTAT_RXOERRINTF_MASK)

#define SPI_INTSTAT_UNDERRUNINTF_SHIFT                  (2)
#define SPI_INTSTAT_UNDERRUNINTF_MASK                   (0x01U << SPI_INTSTAT_UNDERRUNINTF_SHIFT)
#define SPI_INTSTAT_UNDERRUNINTF(x)                     (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_UNDERRUNINTF_SHIFT)) & SPI_INTSTAT_UNDERRUNINTF_MASK)

#define SPI_INTSTAT_RXINTF_SHIFT                        (1)
#define SPI_INTSTAT_RXINTF_MASK                         (0x01U << SPI_INTSTAT_RXINTF_SHIFT)
#define SPI_INTSTAT_RXINTF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_RXINTF_SHIFT)) & SPI_INTSTAT_RXINTF_MASK)

#define SPI_INTSTAT_TXINTF_SHIFT                        (0)
#define SPI_INTSTAT_TXINTF_MASK                         (0x01U << SPI_INTSTAT_TXINTF_SHIFT)
#define SPI_INTSTAT_TXINTF(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_INTSTAT_TXINTF_SHIFT)) & SPI_INTSTAT_TXINTF_MASK)

/*!
 * @brief SPI_INTEN Register Bit Definition
 */

#define SPI_INTEN_TXEPTIEN_SHIFT                        (6)
#define SPI_INTEN_TXEPTIEN_MASK                         (0x01U << SPI_INTEN_TXEPTIEN_SHIFT)
#define SPI_INTEN_TXEPTIEN(x)                           (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_TXEPTIEN_SHIFT)) & SPI_INTEN_TXEPTIEN_MASK)

#define SPI_INTEN_RXFULLIEN_SHIFT                       (5)
#define SPI_INTEN_RXFULLIEN_MASK                        (0x01U << SPI_INTEN_RXFULLIEN_SHIFT)
#define SPI_INTEN_RXFULLIEN(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_RXFULLIEN_SHIFT)) & SPI_INTEN_RXFULLIEN_MASK)

#define SPI_INTEN_RXMATCHIEN_SHIFT                      (4)
#define SPI_INTEN_RXMATCHIEN_MASK                       (0x01U << SPI_INTEN_RXMATCHIEN_SHIFT)
#define SPI_INTEN_RXMATCHIEN(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_RXMATCHIEN_SHIFT)) & SPI_INTEN_RXMATCHIEN_MASK)

#define SPI_INTEN_RXOERRIEN_SHIFT                       (3)
#define SPI_INTEN_RXOERRIEN_MASK                        (0x01U << SPI_INTEN_RXOERRIEN_SHIFT)
#define SPI_INTEN_RXOERRIEN(x)                          (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_RXOERRIEN_SHIFT)) & SPI_INTEN_RXOERRIEN_MASK)

#define SPI_INTEN_UNDERRUNIEN_SHIFT                     (2)
#define SPI_INTEN_UNDERRUNIEN_MASK                      (0x01U << SPI_INTEN_UNDERRUNIEN_SHIFT)
#define SPI_INTEN_UNDERRUNIEN(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_UNDERRUNIEN_SHIFT)) & SPI_INTEN_UNDERRUNIEN_MASK)

#define SPI_INTEN_RXIEN_SHIFT                           (1)
#define SPI_INTEN_RXIEN_MASK                            (0x01U << SPI_INTEN_RXIEN_SHIFT)
#define SPI_INTEN_RXIEN(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_RXIEN_SHIFT)) & SPI_INTEN_RXIEN_MASK)

#define SPI_INTEN_TXIEN_SHIFT                           (0)
#define SPI_INTEN_TXIEN_MASK                            (0x01U << SPI_INTEN_TXIEN_SHIFT)
#define SPI_INTEN_TXIEN(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_INTEN_TXIEN_SHIFT)) & SPI_INTEN_TXIEN_MASK)

/*!
 * @brief SPI_INTCLR Register Bit Definition
 */

#define SPI_INTCLR_TXEPTICLR_SHIFT                      (6)
#define SPI_INTCLR_TXEPTICLR_MASK                       (0x01U << SPI_INTCLR_TXEPTICLR_SHIFT)
#define SPI_INTCLR_TXEPTICLR(x)                         (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_TXEPTICLR_SHIFT)) & SPI_INTCLR_TXEPTICLR_MASK)

#define SPI_INTCLR_RXFULLICLR_SHIFT                     (5)
#define SPI_INTCLR_RXFULLICLR_MASK                      (0x01U << SPI_INTCLR_RXFULLICLR_SHIFT)
#define SPI_INTCLR_RXFULLICLR(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_RXFULLICLR_SHIFT)) & SPI_INTCLR_RXFULLICLR_MASK)

#define SPI_INTCLR_RXMATCHICLR_SHIFT                    (4)
#define SPI_INTCLR_RXMATCHICLR_MASK                     (0x01U << SPI_INTCLR_RXMATCHICLR_SHIFT)
#define SPI_INTCLR_RXMATCHICLR(x)                       (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_RXMATCHICLR_SHIFT)) & SPI_INTCLR_RXMATCHICLR_MASK)

#define SPI_INTCLR_RXOERRICLR_SHIFT                     (3)
#define SPI_INTCLR_RXOERRICLR_MASK                      (0x01U << SPI_INTCLR_RXOERRICLR_SHIFT)
#define SPI_INTCLR_RXOERRICLR(x)                        (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_RXOERRICLR_SHIFT)) & SPI_INTCLR_RXOERRICLR_MASK)

#define SPI_INTCLR_UNDERRUNICLR_SHIFT                   (2)
#define SPI_INTCLR_UNDERRUNICLR_MASK                    (0x01U << SPI_INTCLR_UNDERRUNICLR_SHIFT)
#define SPI_INTCLR_UNDERRUNICLR(x)                      (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_UNDERRUNICLR_SHIFT)) & SPI_INTCLR_UNDERRUNICLR_MASK)

#define SPI_INTCLR_RXICLR_SHIFT                         (1)
#define SPI_INTCLR_RXICLR_MASK                          (0x01U << SPI_INTCLR_RXICLR_SHIFT)
#define SPI_INTCLR_RXICLR(x)                            (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_RXICLR_SHIFT)) & SPI_INTCLR_RXICLR_MASK)

#define SPI_INTCLR_TXICLR_SHIFT                         (0)
#define SPI_INTCLR_TXICLR_MASK                          (0x01U << SPI_INTCLR_TXICLR_SHIFT)
#define SPI_INTCLR_TXICLR(x)                            (((uint32_t)(((uint32_t)(x)) << SPI_INTCLR_TXICLR_SHIFT)) & SPI_INTCLR_TXICLR_MASK)

/*!
 * @brief SPI_GCTL Register Bit Definition
 */

#define SPI_GCTL_NSSTOG_SHIFT                           (12)
#define SPI_GCTL_NSSTOG_MASK                            (0x01U << SPI_GCTL_NSSTOG_SHIFT)
#define SPI_GCTL_NSSTOG(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_NSSTOG_SHIFT)) & SPI_GCTL_NSSTOG_MASK)

#define SPI_GCTL_DW832_SHIFT                            (11)
#define SPI_GCTL_DW832_MASK                             (0x01U << SPI_GCTL_DW832_SHIFT)
#define SPI_GCTL_DW832(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_DW832_SHIFT)) & SPI_GCTL_DW832_MASK)

#define SPI_GCTL_NSS_SHIFT                              (10)
#define SPI_GCTL_NSS_MASK                               (0x01U << SPI_GCTL_NSS_SHIFT)
#define SPI_GCTL_NSS(x)                                 (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_NSS_SHIFT)) & SPI_GCTL_NSS_MASK)

#define SPI_GCTL_DMAEN_SHIFT                            (9)
#define SPI_GCTL_DMAEN_MASK                             (0x01U << SPI_GCTL_DMAEN_SHIFT)
#define SPI_GCTL_DMAEN(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_DMAEN_SHIFT)) & SPI_GCTL_DMAEN_MASK)

#define SPI_GCTL_TXTLF_SHIFT                            (7)
#define SPI_GCTL_TXTLF_MASK                             (0x03U << SPI_GCTL_TXTLF_SHIFT)
#define SPI_GCTL_TXTLF(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_TXTLF_SHIFT)) & SPI_GCTL_TXTLF_MASK)

#define SPI_GCTL_RXTLF_SHIFT                            (5)
#define SPI_GCTL_RXTLF_MASK                             (0x03U << SPI_GCTL_RXTLF_SHIFT)
#define SPI_GCTL_RXTLF(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_RXTLF_SHIFT)) & SPI_GCTL_RXTLF_MASK)

#define SPI_GCTL_RXEN_SHIFT                             (4)
#define SPI_GCTL_RXEN_MASK                              (0x01U << SPI_GCTL_RXEN_SHIFT)
#define SPI_GCTL_RXEN(x)                                (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_RXEN_SHIFT)) & SPI_GCTL_RXEN_MASK)

#define SPI_GCTL_TXEN_SHIFT                             (3)
#define SPI_GCTL_TXEN_MASK                              (0x01U << SPI_GCTL_TXEN_SHIFT)
#define SPI_GCTL_TXEN(x)                                (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_TXEN_SHIFT)) & SPI_GCTL_TXEN_MASK)

#define SPI_GCTL_MODE_SHIFT                             (2)
#define SPI_GCTL_MODE_MASK                              (0x01U << SPI_GCTL_MODE_SHIFT)
#define SPI_GCTL_MODE(x)                                (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_MODE_SHIFT)) & SPI_GCTL_MODE_MASK)

#define SPI_GCTL_INTEN_SHIFT                            (1)
#define SPI_GCTL_INTEN_MASK                             (0x01U << SPI_GCTL_INTEN_SHIFT)
#define SPI_GCTL_INTEN(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_INTEN_SHIFT)) & SPI_GCTL_INTEN_MASK)

#define SPI_GCTL_SPIEN_SHIFT                            (0)
#define SPI_GCTL_SPIEN_MASK                             (0x01U << SPI_GCTL_SPIEN_SHIFT)
#define SPI_GCTL_SPIEN(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_GCTL_SPIEN_SHIFT)) & SPI_GCTL_SPIEN_MASK)

/*!
 * @brief SPI_CCTL Register Bit Definition
 */

#define SPI_CCTL_HISPD_SHIFT                            (7)
#define SPI_CCTL_HISPD_MASK                             (0x01U << SPI_CCTL_HISPD_SHIFT)
#define SPI_CCTL_HISPD(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_HISPD_SHIFT)) & SPI_CCTL_HISPD_MASK)

#define SPI_CCTL_CPHASEL_SHIFT                          (6)
#define SPI_CCTL_CPHASEL_MASK                           (0x01U << SPI_CCTL_CPHASEL_SHIFT)
#define SPI_CCTL_CPHASEL(x)                             (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_CPHASEL_SHIFT)) & SPI_CCTL_CPHASEL_MASK)

#define SPI_CCTL_TXEDGE_SHIFT                           (5)
#define SPI_CCTL_TXEDGE_MASK                            (0x01U << SPI_CCTL_TXEDGE_SHIFT)
#define SPI_CCTL_TXEDGE(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_TXEDGE_SHIFT)) & SPI_CCTL_TXEDGE_MASK)

#define SPI_CCTL_RXEDGE_SHIFT                           (4)
#define SPI_CCTL_RXEDGE_MASK                            (0x01U << SPI_CCTL_RXEDGE_SHIFT)
#define SPI_CCTL_RXEDGE(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_RXEDGE_SHIFT)) & SPI_CCTL_RXEDGE_MASK)

#define SPI_CCTL_SPILEN_SHIFT                           (3)
#define SPI_CCTL_SPILEN_MASK                            (0x01U << SPI_CCTL_SPILEN_SHIFT)
#define SPI_CCTL_SPILEN(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_SPILEN_SHIFT)) & SPI_CCTL_SPILEN_MASK)

#define SPI_CCTL_LSBFE_SHIFT                            (2)
#define SPI_CCTL_LSBFE_MASK                             (0x01U << SPI_CCTL_LSBFE_SHIFT)
#define SPI_CCTL_LSBFE(x)                               (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_LSBFE_SHIFT)) & SPI_CCTL_LSBFE_MASK)

#define SPI_CCTL_CPOL_SHIFT                             (1)
#define SPI_CCTL_CPOL_MASK                              (0x01U << SPI_CCTL_CPOL_SHIFT)
#define SPI_CCTL_CPOL(x)                                (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_CPOL_SHIFT)) & SPI_CCTL_CPOL_MASK)

#define SPI_CCTL_CPHA_SHIFT                             (0)
#define SPI_CCTL_CPHA_MASK                              (0x01U << SPI_CCTL_CPHA_SHIFT)
#define SPI_CCTL_CPHA(x)                                (((uint32_t)(((uint32_t)(x)) << SPI_CCTL_CPHA_SHIFT)) & SPI_CCTL_CPHA_MASK)

/*!
 * @brief SPI_SPBRG Register Bit Definition
 */

#define SPI_SPBRG_SPBRG_SHIFT                           (0)
#define SPI_SPBRG_SPBRG_MASK                            (0xFFFFU << SPI_SPBRG_SPBRG_SHIFT)
#define SPI_SPBRG_SPBRG(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_SPBRG_SPBRG_SHIFT)) & SPI_SPBRG_SPBRG_MASK)

/*!
 * @brief SPI_RXDNR Register Bit Definition
 */

#define SPI_RXDNR_RXDNR_SHIFT                           (0)
#define SPI_RXDNR_RXDNR_MASK                            (0xFFFFU << SPI_RXDNR_RXDNR_SHIFT)
#define SPI_RXDNR_RXDNR(x)                              (((uint32_t)(((uint32_t)(x)) << SPI_RXDNR_RXDNR_SHIFT)) & SPI_RXDNR_RXDNR_MASK)

/*!
 * @brief SPI_NSSR Register Bit Definition
 */

#define SPI_NSSR_NSS_SHIFT                              (0)
#define SPI_NSSR_NSS_MASK                               (0x01U << SPI_NSSR_NSS_SHIFT)
#define SPI_NSSR_NSS(x)                                 (((uint32_t)(((uint32_t)(x)) << SPI_NSSR_NSS_SHIFT)) & SPI_NSSR_NSS_MASK)

/*!
 * @brief SPI_EXTCTL Register Bit Definition
 */

#define SPI_EXTCTL_EXTLEN_SHIFT                         (0)
#define SPI_EXTCTL_EXTLEN_MASK                          (0x1FU << SPI_EXTCTL_EXTLEN_SHIFT)
#define SPI_EXTCTL_EXTLEN(x)                            (((uint32_t)(((uint32_t)(x)) << SPI_EXTCTL_EXTLEN_SHIFT)) & SPI_EXTCTL_EXTLEN_MASK)

/*!
 * @}
 */ /* end of group SPI_Register_Masks */
/******************************************************************************
 * SPI Instance
 ******************************************************************************/

#define SPI1                ((SPI_Type*)SPI1_BASE)

/*!
 * @}
 */ /* end of group SPI_Peripheral_Access_Layer */

/*!
 * @addtogroup TIM1_Peripheral_Access_Layer TIM1 Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * TIM1 Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CR1;                                                            ///< control register 1                           offset: 0x00
    __IO uint32_t CR2;                                                            ///< control register 2                           offset: 0x04
    __IO uint32_t SMCR;                                                           ///< slave mode control register 1                offset: 0x08
    __IO uint32_t DIER;                                                           ///< DMA/Interrupt enable register                offset: 0x0C
    __IO uint32_t SR;                                                             ///< status register                              offset: 0x10
    __IO uint32_t EGR;                                                            ///< event generation register                    offset: 0x14
    __IO uint32_t CCMR1_OUTPUT;                                                   ///< compare mode register 1 (output mode)        offset: 0x18
    __IO uint32_t CCMR2_OUTPUT;                                                   ///< compare mode register 2(output mode)         offset: 0x1c
    __IO uint32_t CCER;                                                           ///< compare enable register                      offset: 0x20
    __IO uint32_t CNT;                                                            ///< counter                                      offset: 0x24
    __IO uint32_t PSC;                                                            ///< prescaler                                    offset: 0x28
    __IO uint32_t ARR;                                                            ///< auto-reload register                         offset: 0x2c
    __IO uint32_t RCR;                                                            ///< repetition counter register                  offset: 0x30
    __IO uint32_t CCR1;                                                           ///< compare register 1                           offset: 0x34
    __IO uint32_t CCR2;                                                           ///< compare register 2                           offset: 0x38
    __IO uint32_t CCR3;                                                           ///< compare register 3                           offset: 0x3c
    __IO uint32_t CCR4;                                                           ///< compare register 4                           offset: 0x40
    __IO uint32_t BDTR;                                                           ///< break and dead-time register                 offset: 0x44
    __IO uint32_t Reserved0[3];                                                   ///< Reserved
    __IO uint32_t CCMR3_OUTPUT;                                                   ///< compare mode register 3 (output mode)        offset: 0x54
    __IO uint32_t CCR5;                                                           ///< compare register 5                           offset: 0x58
    __IO uint32_t PDER;                                                           ///< PWM/DMA repeat enable register               offset: 0x5C
    __IO uint32_t CCR1FALL;                                                       ///< pwm shift count CCR1 register                offset: 0x60
    __IO uint32_t CCR2FALL;                                                       ///< pwm shift count CCR2 register                offset: 0x64
    __IO uint32_t CCR3FALL;                                                       ///< pwm shift count CCR3 register                offset: 0x68
    __IO uint32_t CCR4FALL;                                                       ///< pwm shift count CCR4 register                offset: 0x6C
    __IO uint32_t CCR5FALL;                                                       ///< pwm shift count CCR5 register                offset: 0x70
} TIM1_Type;

/*!
 * @addtogroup TIM1_Register_Masks Register Masks
 * @{ */

/*!
 * @brief TIM1_CR1 Register Bit Definition
 */

#define TIM1_CR1_CKD_SHIFT                              (8)
#define TIM1_CR1_CKD_MASK                               (0x03U << TIM1_CR1_CKD_SHIFT)
#define TIM1_CR1_CKD(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_CKD_SHIFT)) & TIM1_CR1_CKD_MASK)

#define TIM1_CR1_ARPE_SHIFT                             (7)
#define TIM1_CR1_ARPE_MASK                              (0x01U << TIM1_CR1_ARPE_SHIFT)
#define TIM1_CR1_ARPE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_ARPE_SHIFT)) & TIM1_CR1_ARPE_MASK)

#define TIM1_CR1_CMS_SHIFT                              (5)
#define TIM1_CR1_CMS_MASK                               (0x03U << TIM1_CR1_CMS_SHIFT)
#define TIM1_CR1_CMS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_CMS_SHIFT)) & TIM1_CR1_CMS_MASK)

#define TIM1_CR1_DIR_SHIFT                              (4)
#define TIM1_CR1_DIR_MASK                               (0x01U << TIM1_CR1_DIR_SHIFT)
#define TIM1_CR1_DIR(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_DIR_SHIFT)) & TIM1_CR1_DIR_MASK)

#define TIM1_CR1_OPM_SHIFT                              (3)
#define TIM1_CR1_OPM_MASK                               (0x01U << TIM1_CR1_OPM_SHIFT)
#define TIM1_CR1_OPM(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_OPM_SHIFT)) & TIM1_CR1_OPM_MASK)

#define TIM1_CR1_URS_SHIFT                              (2)
#define TIM1_CR1_URS_MASK                               (0x01U << TIM1_CR1_URS_SHIFT)
#define TIM1_CR1_URS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_URS_SHIFT)) & TIM1_CR1_URS_MASK)

#define TIM1_CR1_UDIS_SHIFT                             (1)
#define TIM1_CR1_UDIS_MASK                              (0x01U << TIM1_CR1_UDIS_SHIFT)
#define TIM1_CR1_UDIS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_UDIS_SHIFT)) & TIM1_CR1_UDIS_MASK)

#define TIM1_CR1_CEN_SHIFT                              (0)
#define TIM1_CR1_CEN_MASK                               (0x01U << TIM1_CR1_CEN_SHIFT)
#define TIM1_CR1_CEN(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR1_CEN_SHIFT)) & TIM1_CR1_CEN_MASK)

/*!
 * @brief TIM1_CR2 Register Bit Definition
 */

#define TIM1_CR2_OIS4_SHIFT                             (14)
#define TIM1_CR2_OIS4_MASK                              (0x01U << TIM1_CR2_OIS4_SHIFT)
#define TIM1_CR2_OIS4(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS4_SHIFT)) & TIM1_CR2_OIS4_MASK)

#define TIM1_CR2_OIS3N_SHIFT                            (13)
#define TIM1_CR2_OIS3N_MASK                             (0x01U << TIM1_CR2_OIS3N_SHIFT)
#define TIM1_CR2_OIS3N(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS3N_SHIFT)) & TIM1_CR2_OIS3N_MASK)

#define TIM1_CR2_OIS3_SHIFT                             (12)
#define TIM1_CR2_OIS3_MASK                              (0x01U << TIM1_CR2_OIS3_SHIFT)
#define TIM1_CR2_OIS3(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS3_SHIFT)) & TIM1_CR2_OIS3_MASK)

#define TIM1_CR2_OIS2N_SHIFT                            (11)
#define TIM1_CR2_OIS2N_MASK                             (0x01U << TIM1_CR2_OIS2N_SHIFT)
#define TIM1_CR2_OIS2N(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS2N_SHIFT)) & TIM1_CR2_OIS2N_MASK)

#define TIM1_CR2_OIS2_SHIFT                             (10)
#define TIM1_CR2_OIS2_MASK                              (0x01U << TIM1_CR2_OIS2_SHIFT)
#define TIM1_CR2_OIS2(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS2_SHIFT)) & TIM1_CR2_OIS2_MASK)

#define TIM1_CR2_OIS1N_SHIFT                            (9)
#define TIM1_CR2_OIS1N_MASK                             (0x01U << TIM1_CR2_OIS1N_SHIFT)
#define TIM1_CR2_OIS1N(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS1N_SHIFT)) & TIM1_CR2_OIS1N_MASK)

#define TIM1_CR2_OIS1_SHIFT                             (8)
#define TIM1_CR2_OIS1_MASK                              (0x01U << TIM1_CR2_OIS1_SHIFT)
#define TIM1_CR2_OIS1(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_OIS1_SHIFT)) & TIM1_CR2_OIS1_MASK)

#define TIM1_CR2_MMS_SHIFT                              (4)
#define TIM1_CR2_MMS_MASK                               (0x07U << TIM1_CR2_MMS_SHIFT)
#define TIM1_CR2_MMS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_MMS_SHIFT)) & TIM1_CR2_MMS_MASK)

#define TIM1_CR2_CCUS_SHIFT                             (2)
#define TIM1_CR2_CCUS_MASK                              (0x01U << TIM1_CR2_CCUS_SHIFT)
#define TIM1_CR2_CCUS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_CCUS_SHIFT)) & TIM1_CR2_CCUS_MASK)

#define TIM1_CR2_CCPC_SHIFT                             (0)
#define TIM1_CR2_CCPC_MASK                              (0x01U << TIM1_CR2_CCPC_SHIFT)
#define TIM1_CR2_CCPC(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_CR2_CCPC_SHIFT)) & TIM1_CR2_CCPC_MASK)

/*!
 * @brief TIM1_SMCR Register Bit Definition
 */

#define TIM1_SMCR_MSM_SHIFT                             (7)
#define TIM1_SMCR_MSM_MASK                              (0x01U << TIM1_SMCR_MSM_SHIFT)
#define TIM1_SMCR_MSM(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SMCR_MSM_SHIFT)) & TIM1_SMCR_MSM_MASK)

#define TIM1_SMCR_TS_SHIFT                              (4)
#define TIM1_SMCR_TS_MASK                               (0x07U << TIM1_SMCR_TS_SHIFT)
#define TIM1_SMCR_TS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_SMCR_TS_SHIFT)) & TIM1_SMCR_TS_MASK)

#define TIM1_SMCR_SMS_SHIFT                             (0)
#define TIM1_SMCR_SMS_MASK                              (0x07U << TIM1_SMCR_SMS_SHIFT)
#define TIM1_SMCR_SMS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SMCR_SMS_SHIFT)) & TIM1_SMCR_SMS_MASK)

/*!
 * @brief TIM1_DIER Register Bit Definition
 */

#define TIM1_DIER_CC5IE_SHIFT                           (16)
#define TIM1_DIER_CC5IE_MASK                            (0x01U << TIM1_DIER_CC5IE_SHIFT)
#define TIM1_DIER_CC5IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_CC5IE_SHIFT)) & TIM1_DIER_CC5IE_MASK)

#define TIM1_DIER_BIE_SHIFT                             (7)
#define TIM1_DIER_BIE_MASK                              (0x01U << TIM1_DIER_BIE_SHIFT)
#define TIM1_DIER_BIE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_BIE_SHIFT)) & TIM1_DIER_BIE_MASK)

#define TIM1_DIER_TIE_SHIFT                             (6)
#define TIM1_DIER_TIE_MASK                              (0x01U << TIM1_DIER_TIE_SHIFT)
#define TIM1_DIER_TIE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_TIE_SHIFT)) & TIM1_DIER_TIE_MASK)

#define TIM1_DIER_COMIE_SHIFT                           (5)
#define TIM1_DIER_COMIE_MASK                            (0x01U << TIM1_DIER_COMIE_SHIFT)
#define TIM1_DIER_COMIE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_COMIE_SHIFT)) & TIM1_DIER_COMIE_MASK)

#define TIM1_DIER_CC4IE_SHIFT                           (4)
#define TIM1_DIER_CC4IE_MASK                            (0x01U << TIM1_DIER_CC4IE_SHIFT)
#define TIM1_DIER_CC4IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_CC4IE_SHIFT)) & TIM1_DIER_CC4IE_MASK)

#define TIM1_DIER_CC3IE_SHIFT                           (3)
#define TIM1_DIER_CC3IE_MASK                            (0x01U << TIM1_DIER_CC3IE_SHIFT)
#define TIM1_DIER_CC3IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_CC3IE_SHIFT)) & TIM1_DIER_CC3IE_MASK)

#define TIM1_DIER_CC2IE_SHIFT                           (2)
#define TIM1_DIER_CC2IE_MASK                            (0x01U << TIM1_DIER_CC2IE_SHIFT)
#define TIM1_DIER_CC2IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_CC2IE_SHIFT)) & TIM1_DIER_CC2IE_MASK)

#define TIM1_DIER_CC1IE_SHIFT                           (1)
#define TIM1_DIER_CC1IE_MASK                            (0x01U << TIM1_DIER_CC1IE_SHIFT)
#define TIM1_DIER_CC1IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_CC1IE_SHIFT)) & TIM1_DIER_CC1IE_MASK)

#define TIM1_DIER_UIE_SHIFT                             (0)
#define TIM1_DIER_UIE_MASK                              (0x01U << TIM1_DIER_UIE_SHIFT)
#define TIM1_DIER_UIE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_DIER_UIE_SHIFT)) & TIM1_DIER_UIE_MASK)

/*!
 * @brief TIM1_SR Register Bit Definition
 */

#define TIM1_SR_CC5IF_SHIFT                             (16)
#define TIM1_SR_CC5IF_MASK                              (0x01U << TIM1_SR_CC5IF_SHIFT)
#define TIM1_SR_CC5IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SR_CC5IF_SHIFT)) & TIM1_SR_CC5IF_MASK)

#define TIM1_SR_BIF_SHIFT                               (7)
#define TIM1_SR_BIF_MASK                                (0x01U << TIM1_SR_BIF_SHIFT)
#define TIM1_SR_BIF(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM1_SR_BIF_SHIFT)) & TIM1_SR_BIF_MASK)

#define TIM1_SR_TIF_SHIFT                               (6)
#define TIM1_SR_TIF_MASK                                (0x01U << TIM1_SR_TIF_SHIFT)
#define TIM1_SR_TIF(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM1_SR_TIF_SHIFT)) & TIM1_SR_TIF_MASK)

#define TIM1_SR_COMIF_SHIFT                             (5)
#define TIM1_SR_COMIF_MASK                              (0x01U << TIM1_SR_COMIF_SHIFT)
#define TIM1_SR_COMIF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SR_COMIF_SHIFT)) & TIM1_SR_COMIF_MASK)

#define TIM1_SR_CC4IF_SHIFT                             (4)
#define TIM1_SR_CC4IF_MASK                              (0x01U << TIM1_SR_CC4IF_SHIFT)
#define TIM1_SR_CC4IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SR_CC4IF_SHIFT)) & TIM1_SR_CC4IF_MASK)

#define TIM1_SR_CC3IF_SHIFT                             (3)
#define TIM1_SR_CC3IF_MASK                              (0x01U << TIM1_SR_CC3IF_SHIFT)
#define TIM1_SR_CC3IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SR_CC3IF_SHIFT)) & TIM1_SR_CC3IF_MASK)

#define TIM1_SR_CC2IF_SHIFT                             (2)
#define TIM1_SR_CC2IF_MASK                              (0x01U << TIM1_SR_CC2IF_SHIFT)
#define TIM1_SR_CC2IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SR_CC2IF_SHIFT)) & TIM1_SR_CC2IF_MASK)

#define TIM1_SR_CC1IF_SHIFT                             (1)
#define TIM1_SR_CC1IF_MASK                              (0x01U << TIM1_SR_CC1IF_SHIFT)
#define TIM1_SR_CC1IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_SR_CC1IF_SHIFT)) & TIM1_SR_CC1IF_MASK)

#define TIM1_SR_UIF_SHIFT                               (0)
#define TIM1_SR_UIF_MASK                                (0x01U << TIM1_SR_UIF_SHIFT)
#define TIM1_SR_UIF(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM1_SR_UIF_SHIFT)) & TIM1_SR_UIF_MASK)

/*!
 * @brief TIM1_EGR Register Bit Definition
 */

#define TIM1_EGR_CC5G_SHIFT                             (16)
#define TIM1_EGR_CC5G_MASK                              (0x01U << TIM1_EGR_CC5G_SHIFT)
#define TIM1_EGR_CC5G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_CC5G_SHIFT)) & TIM1_EGR_CC5G_MASK)

#define TIM1_EGR_BG_SHIFT                               (7)
#define TIM1_EGR_BG_MASK                                (0x01U << TIM1_EGR_BG_SHIFT)
#define TIM1_EGR_BG(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_BG_SHIFT)) & TIM1_EGR_BG_MASK)

#define TIM1_EGR_TG_SHIFT                               (6)
#define TIM1_EGR_TG_MASK                                (0x01U << TIM1_EGR_TG_SHIFT)
#define TIM1_EGR_TG(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_TG_SHIFT)) & TIM1_EGR_TG_MASK)

#define TIM1_EGR_COMG_SHIFT                             (5)
#define TIM1_EGR_COMG_MASK                              (0x01U << TIM1_EGR_COMG_SHIFT)
#define TIM1_EGR_COMG(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_COMG_SHIFT)) & TIM1_EGR_COMG_MASK)

#define TIM1_EGR_CC4G_SHIFT                             (4)
#define TIM1_EGR_CC4G_MASK                              (0x01U << TIM1_EGR_CC4G_SHIFT)
#define TIM1_EGR_CC4G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_CC4G_SHIFT)) & TIM1_EGR_CC4G_MASK)

#define TIM1_EGR_CC3G_SHIFT                             (3)
#define TIM1_EGR_CC3G_MASK                              (0x01U << TIM1_EGR_CC3G_SHIFT)
#define TIM1_EGR_CC3G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_CC3G_SHIFT)) & TIM1_EGR_CC3G_MASK)

#define TIM1_EGR_CC2G_SHIFT                             (2)
#define TIM1_EGR_CC2G_MASK                              (0x01U << TIM1_EGR_CC2G_SHIFT)
#define TIM1_EGR_CC2G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_CC2G_SHIFT)) & TIM1_EGR_CC2G_MASK)

#define TIM1_EGR_CC1G_SHIFT                             (1)
#define TIM1_EGR_CC1G_MASK                              (0x01U << TIM1_EGR_CC1G_SHIFT)
#define TIM1_EGR_CC1G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_CC1G_SHIFT)) & TIM1_EGR_CC1G_MASK)

#define TIM1_EGR_UG_SHIFT                               (0)
#define TIM1_EGR_UG_MASK                                (0x01U << TIM1_EGR_UG_SHIFT)
#define TIM1_EGR_UG(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM1_EGR_UG_SHIFT)) & TIM1_EGR_UG_MASK)

/*!
 * @brief TIM1_CCMR1_OUTPUT Register Bit Definition
 */

#define TIM1_CCMR1_OUTPUT_OC2M_SHIFT                    (12)
#define TIM1_CCMR1_OUTPUT_OC2M_MASK                     (0x07U << TIM1_CCMR1_OUTPUT_OC2M_SHIFT)
#define TIM1_CCMR1_OUTPUT_OC2M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR1_OUTPUT_OC2M_SHIFT)) & TIM1_CCMR1_OUTPUT_OC2M_MASK)

#define TIM1_CCMR1_OUTPUT_OC2PE_SHIFT                   (11)
#define TIM1_CCMR1_OUTPUT_OC2PE_MASK                    (0x01U << TIM1_CCMR1_OUTPUT_OC2PE_SHIFT)
#define TIM1_CCMR1_OUTPUT_OC2PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR1_OUTPUT_OC2PE_SHIFT)) & TIM1_CCMR1_OUTPUT_OC2PE_MASK)

#define TIM1_CCMR1_OUTPUT_OC2FE_SHIFT                   (10)
#define TIM1_CCMR1_OUTPUT_OC2FE_MASK                    (0x01U << TIM1_CCMR1_OUTPUT_OC2FE_SHIFT)
#define TIM1_CCMR1_OUTPUT_OC2FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR1_OUTPUT_OC2FE_SHIFT)) & TIM1_CCMR1_OUTPUT_OC2FE_MASK)

#define TIM1_CCMR1_OUTPUT_OC1M_SHIFT                    (4)
#define TIM1_CCMR1_OUTPUT_OC1M_MASK                     (0x07U << TIM1_CCMR1_OUTPUT_OC1M_SHIFT)
#define TIM1_CCMR1_OUTPUT_OC1M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR1_OUTPUT_OC1M_SHIFT)) & TIM1_CCMR1_OUTPUT_OC1M_MASK)

#define TIM1_CCMR1_OUTPUT_OC1PE_SHIFT                   (3)
#define TIM1_CCMR1_OUTPUT_OC1PE_MASK                    (0x01U << TIM1_CCMR1_OUTPUT_OC1PE_SHIFT)
#define TIM1_CCMR1_OUTPUT_OC1PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR1_OUTPUT_OC1PE_SHIFT)) & TIM1_CCMR1_OUTPUT_OC1PE_MASK)

#define TIM1_CCMR1_OUTPUT_OC1FE_SHIFT                   (2)
#define TIM1_CCMR1_OUTPUT_OC1FE_MASK                    (0x01U << TIM1_CCMR1_OUTPUT_OC1FE_SHIFT)
#define TIM1_CCMR1_OUTPUT_OC1FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR1_OUTPUT_OC1FE_SHIFT)) & TIM1_CCMR1_OUTPUT_OC1FE_MASK)

/*!
 * @brief TIM1_CCMR2_OUTPUT Register Bit Definition
 */

#define TIM1_CCMR2_OUTPUT_OC4CE_SHIFT                   (15)
#define TIM1_CCMR2_OUTPUT_OC4CE_MASK                    (0x01U << TIM1_CCMR2_OUTPUT_OC4CE_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC4CE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC4CE_SHIFT)) & TIM1_CCMR2_OUTPUT_OC4CE_MASK)

#define TIM1_CCMR2_OUTPUT_OC4M_SHIFT                    (12)
#define TIM1_CCMR2_OUTPUT_OC4M_MASK                     (0x07U << TIM1_CCMR2_OUTPUT_OC4M_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC4M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC4M_SHIFT)) & TIM1_CCMR2_OUTPUT_OC4M_MASK)

#define TIM1_CCMR2_OUTPUT_OC4PE_SHIFT                   (11)
#define TIM1_CCMR2_OUTPUT_OC4PE_MASK                    (0x01U << TIM1_CCMR2_OUTPUT_OC4PE_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC4PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC4PE_SHIFT)) & TIM1_CCMR2_OUTPUT_OC4PE_MASK)

#define TIM1_CCMR2_OUTPUT_OC4FE_SHIFT                   (10)
#define TIM1_CCMR2_OUTPUT_OC4FE_MASK                    (0x01U << TIM1_CCMR2_OUTPUT_OC4FE_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC4FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC4FE_SHIFT)) & TIM1_CCMR2_OUTPUT_OC4FE_MASK)

#define TIM1_CCMR2_OUTPUT_OC3CE_SHIFT                   (7)
#define TIM1_CCMR2_OUTPUT_OC3CE_MASK                    (0x01U << TIM1_CCMR2_OUTPUT_OC3CE_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC3CE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC3CE_SHIFT)) & TIM1_CCMR2_OUTPUT_OC3CE_MASK)

#define TIM1_CCMR2_OUTPUT_OC3M_SHIFT                    (4)
#define TIM1_CCMR2_OUTPUT_OC3M_MASK                     (0x07U << TIM1_CCMR2_OUTPUT_OC3M_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC3M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC3M_SHIFT)) & TIM1_CCMR2_OUTPUT_OC3M_MASK)

#define TIM1_CCMR2_OUTPUT_OC3PE_SHIFT                   (3)
#define TIM1_CCMR2_OUTPUT_OC3PE_MASK                    (0x01U << TIM1_CCMR2_OUTPUT_OC3PE_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC3PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC3PE_SHIFT)) & TIM1_CCMR2_OUTPUT_OC3PE_MASK)

#define TIM1_CCMR2_OUTPUT_OC3FE_SHIFT                   (2)
#define TIM1_CCMR2_OUTPUT_OC3FE_MASK                    (0x01U << TIM1_CCMR2_OUTPUT_OC3FE_SHIFT)
#define TIM1_CCMR2_OUTPUT_OC3FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR2_OUTPUT_OC3FE_SHIFT)) & TIM1_CCMR2_OUTPUT_OC3FE_MASK)

/*!
 * @brief TIM1_CCER Register Bit Definition
 */

#define TIM1_CCER_CC4NP_SHIFT                           (15)
#define TIM1_CCER_CC4NP_MASK                            (0x01U << TIM1_CCER_CC4NP_SHIFT)
#define TIM1_CCER_CC4NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC4NP_SHIFT)) & TIM1_CCER_CC4NP_MASK)

#define TIM1_CCER_CC4P_SHIFT                            (13)
#define TIM1_CCER_CC4P_MASK                             (0x01U << TIM1_CCER_CC4P_SHIFT)
#define TIM1_CCER_CC4P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC4P_SHIFT)) & TIM1_CCER_CC4P_MASK)

#define TIM1_CCER_CC4E_SHIFT                            (12)
#define TIM1_CCER_CC4E_MASK                             (0x01U << TIM1_CCER_CC4E_SHIFT)
#define TIM1_CCER_CC4E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC4E_SHIFT)) & TIM1_CCER_CC4E_MASK)

#define TIM1_CCER_CC3NP_SHIFT                           (11)
#define TIM1_CCER_CC3NP_MASK                            (0x01U << TIM1_CCER_CC3NP_SHIFT)
#define TIM1_CCER_CC3NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC3NP_SHIFT)) & TIM1_CCER_CC3NP_MASK)

#define TIM1_CCER_CC3NE_SHIFT                           (10)
#define TIM1_CCER_CC3NE_MASK                            (0x01U << TIM1_CCER_CC3NE_SHIFT)
#define TIM1_CCER_CC3NE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC3NE_SHIFT)) & TIM1_CCER_CC3NE_MASK)

#define TIM1_CCER_CC3P_SHIFT                            (9)
#define TIM1_CCER_CC3P_MASK                             (0x01U << TIM1_CCER_CC3P_SHIFT)
#define TIM1_CCER_CC3P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC3P_SHIFT)) & TIM1_CCER_CC3P_MASK)

#define TIM1_CCER_CC3E_SHIFT                            (8)
#define TIM1_CCER_CC3E_MASK                             (0x01U << TIM1_CCER_CC3E_SHIFT)
#define TIM1_CCER_CC3E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC3E_SHIFT)) & TIM1_CCER_CC3E_MASK)

#define TIM1_CCER_CC2NP_SHIFT                           (7)
#define TIM1_CCER_CC2NP_MASK                            (0x01U << TIM1_CCER_CC2NP_SHIFT)
#define TIM1_CCER_CC2NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC2NP_SHIFT)) & TIM1_CCER_CC2NP_MASK)

#define TIM1_CCER_CC2NE_SHIFT                           (6)
#define TIM1_CCER_CC2NE_MASK                            (0x01U << TIM1_CCER_CC2NE_SHIFT)
#define TIM1_CCER_CC2NE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC2NE_SHIFT)) & TIM1_CCER_CC2NE_MASK)

#define TIM1_CCER_CC2P_SHIFT                            (5)
#define TIM1_CCER_CC2P_MASK                             (0x01U << TIM1_CCER_CC2P_SHIFT)
#define TIM1_CCER_CC2P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC2P_SHIFT)) & TIM1_CCER_CC2P_MASK)

#define TIM1_CCER_CC2E_SHIFT                            (4)
#define TIM1_CCER_CC2E_MASK                             (0x01U << TIM1_CCER_CC2E_SHIFT)
#define TIM1_CCER_CC2E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC2E_SHIFT)) & TIM1_CCER_CC2E_MASK)

#define TIM1_CCER_CC1NP_SHIFT                           (3)
#define TIM1_CCER_CC1NP_MASK                            (0x01U << TIM1_CCER_CC1NP_SHIFT)
#define TIM1_CCER_CC1NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC1NP_SHIFT)) & TIM1_CCER_CC1NP_MASK)

#define TIM1_CCER_CC1NE_SHIFT                           (2)
#define TIM1_CCER_CC1NE_MASK                            (0x01U << TIM1_CCER_CC1NE_SHIFT)
#define TIM1_CCER_CC1NE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC1NE_SHIFT)) & TIM1_CCER_CC1NE_MASK)

#define TIM1_CCER_CC1P_SHIFT                            (1)
#define TIM1_CCER_CC1P_MASK                             (0x01U << TIM1_CCER_CC1P_SHIFT)
#define TIM1_CCER_CC1P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC1P_SHIFT)) & TIM1_CCER_CC1P_MASK)

#define TIM1_CCER_CC1E_SHIFT                            (0)
#define TIM1_CCER_CC1E_MASK                             (0x01U << TIM1_CCER_CC1E_SHIFT)
#define TIM1_CCER_CC1E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCER_CC1E_SHIFT)) & TIM1_CCER_CC1E_MASK)

/*!
 * @brief TIM1_CNT Register Bit Definition
 */

#define TIM1_CNT_CNT_SHIFT                              (0)
#define TIM1_CNT_CNT_MASK                               (0xFFFFU << TIM1_CNT_CNT_SHIFT)
#define TIM1_CNT_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_CNT_CNT_SHIFT)) & TIM1_CNT_CNT_MASK)

/*!
 * @brief TIM1_PSC Register Bit Definition
 */

#define TIM1_PSC_PSC_SHIFT                              (0)
#define TIM1_PSC_PSC_MASK                               (0xFFFFU << TIM1_PSC_PSC_SHIFT)
#define TIM1_PSC_PSC(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_PSC_PSC_SHIFT)) & TIM1_PSC_PSC_MASK)

/*!
 * @brief TIM1_ARR Register Bit Definition
 */

#define TIM1_ARR_ARR_SHIFT                              (0)
#define TIM1_ARR_ARR_MASK                               (0xFFFFU << TIM1_ARR_ARR_SHIFT)
#define TIM1_ARR_ARR(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_ARR_ARR_SHIFT)) & TIM1_ARR_ARR_MASK)

/*!
 * @brief TIM1_RCR Register Bit Definition
 */

#define TIM1_RCR_REPCNT_SHIFT                           (8)
#define TIM1_RCR_REPCNT_MASK                            (0xFFU << TIM1_RCR_REPCNT_SHIFT)
#define TIM1_RCR_REPCNT(x)                              (((uint32_t)(((uint32_t)(x)) << TIM1_RCR_REPCNT_SHIFT)) & TIM1_RCR_REPCNT_MASK)

#define TIM1_RCR_REP_SHIFT                              (0)
#define TIM1_RCR_REP_MASK                               (0xFFU << TIM1_RCR_REP_SHIFT)
#define TIM1_RCR_REP(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM1_RCR_REP_SHIFT)) & TIM1_RCR_REP_MASK)

/*!
 * @brief TIM1_CCR1 Register Bit Definition
 */

#define TIM1_CCR1_CCR1_SHIFT                            (0)
#define TIM1_CCR1_CCR1_MASK                             (0xFFFFU << TIM1_CCR1_CCR1_SHIFT)
#define TIM1_CCR1_CCR1(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCR1_CCR1_SHIFT)) & TIM1_CCR1_CCR1_MASK)

/*!
 * @brief TIM1_CCR2 Register Bit Definition
 */

#define TIM1_CCR2_CCR2_SHIFT                            (0)
#define TIM1_CCR2_CCR2_MASK                             (0xFFFFU << TIM1_CCR2_CCR2_SHIFT)
#define TIM1_CCR2_CCR2(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCR2_CCR2_SHIFT)) & TIM1_CCR2_CCR2_MASK)

/*!
 * @brief TIM1_CCR3 Register Bit Definition
 */

#define TIM1_CCR3_CCR3_SHIFT                            (0)
#define TIM1_CCR3_CCR3_MASK                             (0xFFFFU << TIM1_CCR3_CCR3_SHIFT)
#define TIM1_CCR3_CCR3(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCR3_CCR3_SHIFT)) & TIM1_CCR3_CCR3_MASK)

/*!
 * @brief TIM1_CCR4 Register Bit Definition
 */

#define TIM1_CCR4_CCR4_SHIFT                            (0)
#define TIM1_CCR4_CCR4_MASK                             (0xFFFFU << TIM1_CCR4_CCR4_SHIFT)
#define TIM1_CCR4_CCR4(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCR4_CCR4_SHIFT)) & TIM1_CCR4_CCR4_MASK)

/*!
 * @brief TIM1_BDTR Register Bit Definition
 */

#define TIM1_BDTR_DOE_SHIFT                             (16)
#define TIM1_BDTR_DOE_MASK                              (0x01U << TIM1_BDTR_DOE_SHIFT)
#define TIM1_BDTR_DOE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_DOE_SHIFT)) & TIM1_BDTR_DOE_MASK)

#define TIM1_BDTR_MOE_SHIFT                             (15)
#define TIM1_BDTR_MOE_MASK                              (0x01U << TIM1_BDTR_MOE_SHIFT)
#define TIM1_BDTR_MOE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_MOE_SHIFT)) & TIM1_BDTR_MOE_MASK)

#define TIM1_BDTR_AOE_SHIFT                             (14)
#define TIM1_BDTR_AOE_MASK                              (0x01U << TIM1_BDTR_AOE_SHIFT)
#define TIM1_BDTR_AOE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_AOE_SHIFT)) & TIM1_BDTR_AOE_MASK)

#define TIM1_BDTR_BKP_SHIFT                             (13)
#define TIM1_BDTR_BKP_MASK                              (0x01U << TIM1_BDTR_BKP_SHIFT)
#define TIM1_BDTR_BKP(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_BKP_SHIFT)) & TIM1_BDTR_BKP_MASK)

#define TIM1_BDTR_BKE_SHIFT                             (12)
#define TIM1_BDTR_BKE_MASK                              (0x01U << TIM1_BDTR_BKE_SHIFT)
#define TIM1_BDTR_BKE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_BKE_SHIFT)) & TIM1_BDTR_BKE_MASK)

#define TIM1_BDTR_OSSR_SHIFT                            (11)
#define TIM1_BDTR_OSSR_MASK                             (0x01U << TIM1_BDTR_OSSR_SHIFT)
#define TIM1_BDTR_OSSR(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_OSSR_SHIFT)) & TIM1_BDTR_OSSR_MASK)

#define TIM1_BDTR_OSSI_SHIFT                            (10)
#define TIM1_BDTR_OSSI_MASK                             (0x01U << TIM1_BDTR_OSSI_SHIFT)
#define TIM1_BDTR_OSSI(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_OSSI_SHIFT)) & TIM1_BDTR_OSSI_MASK)

#define TIM1_BDTR_LOCK_SHIFT                            (8)
#define TIM1_BDTR_LOCK_MASK                             (0x03U << TIM1_BDTR_LOCK_SHIFT)
#define TIM1_BDTR_LOCK(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_LOCK_SHIFT)) & TIM1_BDTR_LOCK_MASK)

#define TIM1_BDTR_DTG_SHIFT                             (0)
#define TIM1_BDTR_DTG_MASK                              (0xFFU << TIM1_BDTR_DTG_SHIFT)
#define TIM1_BDTR_DTG(x)                                (((uint32_t)(((uint32_t)(x)) << TIM1_BDTR_DTG_SHIFT)) & TIM1_BDTR_DTG_MASK)

/*!
 * @brief TIM1_CCMR3_OUTPUT Register Bit Definition
 */

#define TIM1_CCMR3_OUTPUT_OC5M_SHIFT                    (4)
#define TIM1_CCMR3_OUTPUT_OC5M_MASK                     (0x07U << TIM1_CCMR3_OUTPUT_OC5M_SHIFT)
#define TIM1_CCMR3_OUTPUT_OC5M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR3_OUTPUT_OC5M_SHIFT)) & TIM1_CCMR3_OUTPUT_OC5M_MASK)

#define TIM1_CCMR3_OUTPUT_OC5PE_SHIFT                   (3)
#define TIM1_CCMR3_OUTPUT_OC5PE_MASK                    (0x01U << TIM1_CCMR3_OUTPUT_OC5PE_SHIFT)
#define TIM1_CCMR3_OUTPUT_OC5PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR3_OUTPUT_OC5PE_SHIFT)) & TIM1_CCMR3_OUTPUT_OC5PE_MASK)

#define TIM1_CCMR3_OUTPUT_OC5FE_SHIFT                   (2)
#define TIM1_CCMR3_OUTPUT_OC5FE_MASK                    (0x01U << TIM1_CCMR3_OUTPUT_OC5FE_SHIFT)
#define TIM1_CCMR3_OUTPUT_OC5FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM1_CCMR3_OUTPUT_OC5FE_SHIFT)) & TIM1_CCMR3_OUTPUT_OC5FE_MASK)

/*!
 * @brief TIM1_CCR5 Register Bit Definition
 */

#define TIM1_CCR5_CCR5_SHIFT                            (0)
#define TIM1_CCR5_CCR5_MASK                             (0xFFFFU << TIM1_CCR5_CCR5_SHIFT)
#define TIM1_CCR5_CCR5(x)                               (((uint32_t)(((uint32_t)(x)) << TIM1_CCR5_CCR5_SHIFT)) & TIM1_CCR5_CCR5_MASK)

/*!
 * @brief TIM1_PDER Register Bit Definition
 */

#define TIM1_PDER_CCR5SHIFTEN_SHIFT                     (5)
#define TIM1_PDER_CCR5SHIFTEN_MASK                      (0x01U << TIM1_PDER_CCR5SHIFTEN_SHIFT)
#define TIM1_PDER_CCR5SHIFTEN(x)                        (((uint32_t)(((uint32_t)(x)) << TIM1_PDER_CCR5SHIFTEN_SHIFT)) & TIM1_PDER_CCR5SHIFTEN_MASK)

#define TIM1_PDER_CCR4SHIFTEN_SHIFT                     (4)
#define TIM1_PDER_CCR4SHIFTEN_MASK                      (0x01U << TIM1_PDER_CCR4SHIFTEN_SHIFT)
#define TIM1_PDER_CCR4SHIFTEN(x)                        (((uint32_t)(((uint32_t)(x)) << TIM1_PDER_CCR4SHIFTEN_SHIFT)) & TIM1_PDER_CCR4SHIFTEN_MASK)

#define TIM1_PDER_CCR3SHIFTEN_SHIFT                     (3)
#define TIM1_PDER_CCR3SHIFTEN_MASK                      (0x01U << TIM1_PDER_CCR3SHIFTEN_SHIFT)
#define TIM1_PDER_CCR3SHIFTEN(x)                        (((uint32_t)(((uint32_t)(x)) << TIM1_PDER_CCR3SHIFTEN_SHIFT)) & TIM1_PDER_CCR3SHIFTEN_MASK)

#define TIM1_PDER_CCR2SHIFTEN_SHIFT                     (2)
#define TIM1_PDER_CCR2SHIFTEN_MASK                      (0x01U << TIM1_PDER_CCR2SHIFTEN_SHIFT)
#define TIM1_PDER_CCR2SHIFTEN(x)                        (((uint32_t)(((uint32_t)(x)) << TIM1_PDER_CCR2SHIFTEN_SHIFT)) & TIM1_PDER_CCR2SHIFTEN_MASK)

#define TIM1_PDER_CCR1SHIFTEN_SHIFT                     (1)
#define TIM1_PDER_CCR1SHIFTEN_MASK                      (0x01U << TIM1_PDER_CCR1SHIFTEN_SHIFT)
#define TIM1_PDER_CCR1SHIFTEN(x)                        (((uint32_t)(((uint32_t)(x)) << TIM1_PDER_CCR1SHIFTEN_SHIFT)) & TIM1_PDER_CCR1SHIFTEN_MASK)

/*!
 * @brief TIM1_CCR1FALL Register Bit Definition
 */

#define TIM1_CCR1FALL_CCR1FALL_SHIFT                    (0)
#define TIM1_CCR1FALL_CCR1FALL_MASK                     (0xFFFFU << TIM1_CCR1FALL_CCR1FALL_SHIFT)
#define TIM1_CCR1FALL_CCR1FALL(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCR1FALL_CCR1FALL_SHIFT)) & TIM1_CCR1FALL_CCR1FALL_MASK)

/*!
 * @brief TIM1_CCR2FALL Register Bit Definition
 */

#define TIM1_CCR2FALL_CCR2FALL_SHIFT                    (0)
#define TIM1_CCR2FALL_CCR2FALL_MASK                     (0xFFFFU << TIM1_CCR2FALL_CCR2FALL_SHIFT)
#define TIM1_CCR2FALL_CCR2FALL(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCR2FALL_CCR2FALL_SHIFT)) & TIM1_CCR2FALL_CCR2FALL_MASK)

/*!
 * @brief TIM1_CCR3FALL Register Bit Definition
 */

#define TIM1_CCR3FALL_CCR3FALL_SHIFT                    (0)
#define TIM1_CCR3FALL_CCR3FALL_MASK                     (0xFFFFU << TIM1_CCR3FALL_CCR3FALL_SHIFT)
#define TIM1_CCR3FALL_CCR3FALL(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCR3FALL_CCR3FALL_SHIFT)) & TIM1_CCR3FALL_CCR3FALL_MASK)

/*!
 * @brief TIM1_CCR4FALL Register Bit Definition
 */

#define TIM1_CCR4FALL_CCR4FALL_SHIFT                    (0)
#define TIM1_CCR4FALL_CCR4FALL_MASK                     (0xFFFFU << TIM1_CCR4FALL_CCR4FALL_SHIFT)
#define TIM1_CCR4FALL_CCR4FALL(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCR4FALL_CCR4FALL_SHIFT)) & TIM1_CCR4FALL_CCR4FALL_MASK)

/*!
 * @brief TIM1_CCR5FALL Register Bit Definition
 */

#define TIM1_CCR5FALL_CCR5FALL_SHIFT                    (0)
#define TIM1_CCR5FALL_CCR5FALL_MASK                     (0xFFFFU << TIM1_CCR5FALL_CCR5FALL_SHIFT)
#define TIM1_CCR5FALL_CCR5FALL(x)                       (((uint32_t)(((uint32_t)(x)) << TIM1_CCR5FALL_CCR5FALL_SHIFT)) & TIM1_CCR5FALL_CCR5FALL_MASK)

/*!
 * @}
 */ /* end of group TIM1_Register_Masks */
/******************************************************************************
 * TIM1 Instance
 ******************************************************************************/

#define TIM1                ((TIM1_Type*)TIM1_BASE)

/*!
 * @}
 */ /* end of group TIM1_Peripheral_Access_Layer */

/*!
 * @addtogroup TIM3_Peripheral_Access_Layer TIM3 Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * TIM3 Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CR1;                                                            ///< control register 1                           offset: 0x00
    __IO uint32_t CR2;                                                            ///< control register 2                           offset: 0x04
    __IO uint32_t SMCR;                                                           ///< slave mode control register 1                offset: 0x08
    __IO uint32_t DIER;                                                           ///< DMA/Interrupt enable register                offset: 0x0C
    __IO uint32_t SR;                                                             ///< status register                              offset: 0x10
    __IO uint32_t EGR;                                                            ///< event generation register                    offset: 0x14
    __IO uint32_t CCMR1;                                                          ///< compare mode register 1 (output/input mode)  offset: 0x18
    __IO uint32_t CCMR2;                                                          ///< compare mode register 2 (output/input mode)  offset: 0x1c
    __IO uint32_t CCER;                                                           ///< compare enable register                      offset: 0x20
    __IO uint32_t CNT;                                                            ///< counter                                      offset: 0x24
    __IO uint32_t PSC;                                                            ///< prescaler                                    offset: 0x28
    __IO uint32_t ARR;                                                            ///< auto-reload register                         offset: 0x2C
    __IO uint32_t Reserved0[1];                                                   ///< Reserved
    __IO uint32_t CCR1;                                                           ///< compare register 1                           offset: 0x34
    __IO uint32_t CCR2;                                                           ///< compare register 2                           offset: 0x38
    __IO uint32_t CCR3;                                                           ///< compare register 3                           offset: 0x3C
    __IO uint32_t CCR4;                                                           ///< compare register 4                           offset: 0x40
    __IO uint32_t Reserved1[3];                                                   ///< Reserved
    __IO uint32_t OR;                                                             ///< option register                              offset: 0x50
} TIM3_Type;

/*!
 * @addtogroup TIM3_Register_Masks Register Masks
 * @{ */

/*!
 * @brief TIM3_CR1 Register Bit Definition
 */

#define TIM3_CR1_CKD_SHIFT                              (8)
#define TIM3_CR1_CKD_MASK                               (0x03U << TIM3_CR1_CKD_SHIFT)
#define TIM3_CR1_CKD(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_CKD_SHIFT)) & TIM3_CR1_CKD_MASK)

#define TIM3_CR1_ARPE_SHIFT                             (7)
#define TIM3_CR1_ARPE_MASK                              (0x01U << TIM3_CR1_ARPE_SHIFT)
#define TIM3_CR1_ARPE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_ARPE_SHIFT)) & TIM3_CR1_ARPE_MASK)

#define TIM3_CR1_CMS_SHIFT                              (5)
#define TIM3_CR1_CMS_MASK                               (0x03U << TIM3_CR1_CMS_SHIFT)
#define TIM3_CR1_CMS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_CMS_SHIFT)) & TIM3_CR1_CMS_MASK)

#define TIM3_CR1_DIR_SHIFT                              (4)
#define TIM3_CR1_DIR_MASK                               (0x01U << TIM3_CR1_DIR_SHIFT)
#define TIM3_CR1_DIR(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_DIR_SHIFT)) & TIM3_CR1_DIR_MASK)

#define TIM3_CR1_OPM_SHIFT                              (3)
#define TIM3_CR1_OPM_MASK                               (0x01U << TIM3_CR1_OPM_SHIFT)
#define TIM3_CR1_OPM(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_OPM_SHIFT)) & TIM3_CR1_OPM_MASK)

#define TIM3_CR1_URS_SHIFT                              (2)
#define TIM3_CR1_URS_MASK                               (0x01U << TIM3_CR1_URS_SHIFT)
#define TIM3_CR1_URS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_URS_SHIFT)) & TIM3_CR1_URS_MASK)

#define TIM3_CR1_UDIS_SHIFT                             (1)
#define TIM3_CR1_UDIS_MASK                              (0x01U << TIM3_CR1_UDIS_SHIFT)
#define TIM3_CR1_UDIS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_UDIS_SHIFT)) & TIM3_CR1_UDIS_MASK)

#define TIM3_CR1_CEN_SHIFT                              (0)
#define TIM3_CR1_CEN_MASK                               (0x01U << TIM3_CR1_CEN_SHIFT)
#define TIM3_CR1_CEN(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR1_CEN_SHIFT)) & TIM3_CR1_CEN_MASK)

/*!
 * @brief TIM3_CR2 Register Bit Definition
 */

#define TIM3_CR2_TI1S_SHIFT                             (7)
#define TIM3_CR2_TI1S_MASK                              (0x01U << TIM3_CR2_TI1S_SHIFT)
#define TIM3_CR2_TI1S(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_CR2_TI1S_SHIFT)) & TIM3_CR2_TI1S_MASK)

#define TIM3_CR2_MMS_SHIFT                              (4)
#define TIM3_CR2_MMS_MASK                               (0x07U << TIM3_CR2_MMS_SHIFT)
#define TIM3_CR2_MMS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CR2_MMS_SHIFT)) & TIM3_CR2_MMS_MASK)

#define TIM3_CR2_CCDS_SHIFT                             (3)
#define TIM3_CR2_CCDS_MASK                              (0x01U << TIM3_CR2_CCDS_SHIFT)
#define TIM3_CR2_CCDS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_CR2_CCDS_SHIFT)) & TIM3_CR2_CCDS_MASK)

/*!
 * @brief TIM3_SMCR Register Bit Definition
 */

#define TIM3_SMCR_MSM_SHIFT                             (7)
#define TIM3_SMCR_MSM_MASK                              (0x01U << TIM3_SMCR_MSM_SHIFT)
#define TIM3_SMCR_MSM(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SMCR_MSM_SHIFT)) & TIM3_SMCR_MSM_MASK)

#define TIM3_SMCR_TS_SHIFT                              (4)
#define TIM3_SMCR_TS_MASK                               (0x07U << TIM3_SMCR_TS_SHIFT)
#define TIM3_SMCR_TS(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_SMCR_TS_SHIFT)) & TIM3_SMCR_TS_MASK)

#define TIM3_SMCR_SMS_SHIFT                             (0)
#define TIM3_SMCR_SMS_MASK                              (0x07U << TIM3_SMCR_SMS_SHIFT)
#define TIM3_SMCR_SMS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SMCR_SMS_SHIFT)) & TIM3_SMCR_SMS_MASK)

/*!
 * @brief TIM3_DIER Register Bit Definition
 */

#define TIM3_DIER_TIE_SHIFT                             (6)
#define TIM3_DIER_TIE_MASK                              (0x01U << TIM3_DIER_TIE_SHIFT)
#define TIM3_DIER_TIE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_DIER_TIE_SHIFT)) & TIM3_DIER_TIE_MASK)

#define TIM3_DIER_CC4IE_SHIFT                           (4)
#define TIM3_DIER_CC4IE_MASK                            (0x01U << TIM3_DIER_CC4IE_SHIFT)
#define TIM3_DIER_CC4IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_DIER_CC4IE_SHIFT)) & TIM3_DIER_CC4IE_MASK)

#define TIM3_DIER_CC3IE_SHIFT                           (3)
#define TIM3_DIER_CC3IE_MASK                            (0x01U << TIM3_DIER_CC3IE_SHIFT)
#define TIM3_DIER_CC3IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_DIER_CC3IE_SHIFT)) & TIM3_DIER_CC3IE_MASK)

#define TIM3_DIER_CC2IE_SHIFT                           (2)
#define TIM3_DIER_CC2IE_MASK                            (0x01U << TIM3_DIER_CC2IE_SHIFT)
#define TIM3_DIER_CC2IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_DIER_CC2IE_SHIFT)) & TIM3_DIER_CC2IE_MASK)

#define TIM3_DIER_CC1IE_SHIFT                           (1)
#define TIM3_DIER_CC1IE_MASK                            (0x01U << TIM3_DIER_CC1IE_SHIFT)
#define TIM3_DIER_CC1IE(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_DIER_CC1IE_SHIFT)) & TIM3_DIER_CC1IE_MASK)

#define TIM3_DIER_UIE_SHIFT                             (0)
#define TIM3_DIER_UIE_MASK                              (0x01U << TIM3_DIER_UIE_SHIFT)
#define TIM3_DIER_UIE(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_DIER_UIE_SHIFT)) & TIM3_DIER_UIE_MASK)

/*!
 * @brief TIM3_SR Register Bit Definition
 */

#define TIM3_SR_CC3OF_SHIFT                             (11)
#define TIM3_SR_CC3OF_MASK                              (0x01U << TIM3_SR_CC3OF_SHIFT)
#define TIM3_SR_CC3OF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC3OF_SHIFT)) & TIM3_SR_CC3OF_MASK)

#define TIM3_SR_CC2OF_SHIFT                             (10)
#define TIM3_SR_CC2OF_MASK                              (0x01U << TIM3_SR_CC2OF_SHIFT)
#define TIM3_SR_CC2OF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC2OF_SHIFT)) & TIM3_SR_CC2OF_MASK)

#define TIM3_SR_CC1OF_SHIFT                             (9)
#define TIM3_SR_CC1OF_MASK                              (0x01U << TIM3_SR_CC1OF_SHIFT)
#define TIM3_SR_CC1OF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC1OF_SHIFT)) & TIM3_SR_CC1OF_MASK)

#define TIM3_SR_TIF_SHIFT                               (6)
#define TIM3_SR_TIF_MASK                                (0x01U << TIM3_SR_TIF_SHIFT)
#define TIM3_SR_TIF(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM3_SR_TIF_SHIFT)) & TIM3_SR_TIF_MASK)

#define TIM3_SR_CC4IF_SHIFT                             (4)
#define TIM3_SR_CC4IF_MASK                              (0x01U << TIM3_SR_CC4IF_SHIFT)
#define TIM3_SR_CC4IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC4IF_SHIFT)) & TIM3_SR_CC4IF_MASK)

#define TIM3_SR_CC3IF_SHIFT                             (3)
#define TIM3_SR_CC3IF_MASK                              (0x01U << TIM3_SR_CC3IF_SHIFT)
#define TIM3_SR_CC3IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC3IF_SHIFT)) & TIM3_SR_CC3IF_MASK)

#define TIM3_SR_CC2IF_SHIFT                             (2)
#define TIM3_SR_CC2IF_MASK                              (0x01U << TIM3_SR_CC2IF_SHIFT)
#define TIM3_SR_CC2IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC2IF_SHIFT)) & TIM3_SR_CC2IF_MASK)

#define TIM3_SR_CC1IF_SHIFT                             (1)
#define TIM3_SR_CC1IF_MASK                              (0x01U << TIM3_SR_CC1IF_SHIFT)
#define TIM3_SR_CC1IF(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_SR_CC1IF_SHIFT)) & TIM3_SR_CC1IF_MASK)

#define TIM3_SR_UIF_SHIFT                               (0)
#define TIM3_SR_UIF_MASK                                (0x01U << TIM3_SR_UIF_SHIFT)
#define TIM3_SR_UIF(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM3_SR_UIF_SHIFT)) & TIM3_SR_UIF_MASK)

/*!
 * @brief TIM3_EGR Register Bit Definition
 */

#define TIM3_EGR_TG_SHIFT                               (6)
#define TIM3_EGR_TG_MASK                                (0x01U << TIM3_EGR_TG_SHIFT)
#define TIM3_EGR_TG(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM3_EGR_TG_SHIFT)) & TIM3_EGR_TG_MASK)

#define TIM3_EGR_CC4G_SHIFT                             (4)
#define TIM3_EGR_CC4G_MASK                              (0x01U << TIM3_EGR_CC4G_SHIFT)
#define TIM3_EGR_CC4G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_EGR_CC4G_SHIFT)) & TIM3_EGR_CC4G_MASK)

#define TIM3_EGR_CC3G_SHIFT                             (3)
#define TIM3_EGR_CC3G_MASK                              (0x01U << TIM3_EGR_CC3G_SHIFT)
#define TIM3_EGR_CC3G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_EGR_CC3G_SHIFT)) & TIM3_EGR_CC3G_MASK)

#define TIM3_EGR_CC2G_SHIFT                             (2)
#define TIM3_EGR_CC2G_MASK                              (0x01U << TIM3_EGR_CC2G_SHIFT)
#define TIM3_EGR_CC2G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_EGR_CC2G_SHIFT)) & TIM3_EGR_CC2G_MASK)

#define TIM3_EGR_CC1G_SHIFT                             (1)
#define TIM3_EGR_CC1G_MASK                              (0x01U << TIM3_EGR_CC1G_SHIFT)
#define TIM3_EGR_CC1G(x)                                (((uint32_t)(((uint32_t)(x)) << TIM3_EGR_CC1G_SHIFT)) & TIM3_EGR_CC1G_MASK)

#define TIM3_EGR_UG_SHIFT                               (0)
#define TIM3_EGR_UG_MASK                                (0x01U << TIM3_EGR_UG_SHIFT)
#define TIM3_EGR_UG(x)                                  (((uint32_t)(((uint32_t)(x)) << TIM3_EGR_UG_SHIFT)) & TIM3_EGR_UG_MASK)

/*!
 * @brief TIM3_CCMR1_OUTPUT Register Bit Definition
 */

#define TIM3_CCMR1_OUTPUT_OC2M_SHIFT                    (12)
#define TIM3_CCMR1_OUTPUT_OC2M_MASK                     (0x07U << TIM3_CCMR1_OUTPUT_OC2M_SHIFT)
#define TIM3_CCMR1_OUTPUT_OC2M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_OC2M_SHIFT)) & TIM3_CCMR1_OUTPUT_OC2M_MASK)

#define TIM3_CCMR1_OUTPUT_OC2PE_SHIFT                   (11)
#define TIM3_CCMR1_OUTPUT_OC2PE_MASK                    (0x01U << TIM3_CCMR1_OUTPUT_OC2PE_SHIFT)
#define TIM3_CCMR1_OUTPUT_OC2PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_OC2PE_SHIFT)) & TIM3_CCMR1_OUTPUT_OC2PE_MASK)

#define TIM3_CCMR1_OUTPUT_OC2FE_SHIFT                   (10)
#define TIM3_CCMR1_OUTPUT_OC2FE_MASK                    (0x01U << TIM3_CCMR1_OUTPUT_OC2FE_SHIFT)
#define TIM3_CCMR1_OUTPUT_OC2FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_OC2FE_SHIFT)) & TIM3_CCMR1_OUTPUT_OC2FE_MASK)

#define TIM3_CCMR1_OUTPUT_CC2S_SHIFT                    (8)
#define TIM3_CCMR1_OUTPUT_CC2S_MASK                     (0x03U << TIM3_CCMR1_OUTPUT_CC2S_SHIFT)
#define TIM3_CCMR1_OUTPUT_CC2S(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_CC2S_SHIFT)) & TIM3_CCMR1_OUTPUT_CC2S_MASK)

#define TIM3_CCMR1_OUTPUT_OC1M_SHIFT                    (4)
#define TIM3_CCMR1_OUTPUT_OC1M_MASK                     (0x07U << TIM3_CCMR1_OUTPUT_OC1M_SHIFT)
#define TIM3_CCMR1_OUTPUT_OC1M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_OC1M_SHIFT)) & TIM3_CCMR1_OUTPUT_OC1M_MASK)

#define TIM3_CCMR1_OUTPUT_OC1PE_SHIFT                   (3)
#define TIM3_CCMR1_OUTPUT_OC1PE_MASK                    (0x01U << TIM3_CCMR1_OUTPUT_OC1PE_SHIFT)
#define TIM3_CCMR1_OUTPUT_OC1PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_OC1PE_SHIFT)) & TIM3_CCMR1_OUTPUT_OC1PE_MASK)

#define TIM3_CCMR1_OUTPUT_OC1FE_SHIFT                   (2)
#define TIM3_CCMR1_OUTPUT_OC1FE_MASK                    (0x01U << TIM3_CCMR1_OUTPUT_OC1FE_SHIFT)
#define TIM3_CCMR1_OUTPUT_OC1FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_OC1FE_SHIFT)) & TIM3_CCMR1_OUTPUT_OC1FE_MASK)

#define TIM3_CCMR1_OUTPUT_CC1S_SHIFT                    (0)
#define TIM3_CCMR1_OUTPUT_CC1S_MASK                     (0x03U << TIM3_CCMR1_OUTPUT_CC1S_SHIFT)
#define TIM3_CCMR1_OUTPUT_CC1S(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_OUTPUT_CC1S_SHIFT)) & TIM3_CCMR1_OUTPUT_CC1S_MASK)

/*!
 * @brief TIM3_CCMR1_INPUT Register Bit Definition
 */

#define TIM3_CCMR1_INPUT_IC2F_SHIFT                     (12)
#define TIM3_CCMR1_INPUT_IC2F_MASK                      (0x0FU << TIM3_CCMR1_INPUT_IC2F_SHIFT)
#define TIM3_CCMR1_INPUT_IC2F(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_INPUT_IC2F_SHIFT)) & TIM3_CCMR1_INPUT_IC2F_MASK)

#define TIM3_CCMR1_INPUT_IC2PSC_SHIFT                   (10)
#define TIM3_CCMR1_INPUT_IC2PSC_MASK                    (0x03U << TIM3_CCMR1_INPUT_IC2PSC_SHIFT)
#define TIM3_CCMR1_INPUT_IC2PSC(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_INPUT_IC2PSC_SHIFT)) & TIM3_CCMR1_INPUT_IC2PSC_MASK)

#define TIM3_CCMR1_INPUT_CC2S_SHIFT                     (8)
#define TIM3_CCMR1_INPUT_CC2S_MASK                      (0x03U << TIM3_CCMR1_INPUT_CC2S_SHIFT)
#define TIM3_CCMR1_INPUT_CC2S(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_INPUT_CC2S_SHIFT)) & TIM3_CCMR1_INPUT_CC2S_MASK)

#define TIM3_CCMR1_INPUT_IC1F_SHIFT                     (4)
#define TIM3_CCMR1_INPUT_IC1F_MASK                      (0x0FU << TIM3_CCMR1_INPUT_IC1F_SHIFT)
#define TIM3_CCMR1_INPUT_IC1F(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_INPUT_IC1F_SHIFT)) & TIM3_CCMR1_INPUT_IC1F_MASK)

#define TIM3_CCMR1_INPUT_IC1PSC_SHIFT                   (2)
#define TIM3_CCMR1_INPUT_IC1PSC_MASK                    (0x03U << TIM3_CCMR1_INPUT_IC1PSC_SHIFT)
#define TIM3_CCMR1_INPUT_IC1PSC(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_INPUT_IC1PSC_SHIFT)) & TIM3_CCMR1_INPUT_IC1PSC_MASK)

#define TIM3_CCMR1_INPUT_CC1S_SHIFT                     (0)
#define TIM3_CCMR1_INPUT_CC1S_MASK                      (0x03U << TIM3_CCMR1_INPUT_CC1S_SHIFT)
#define TIM3_CCMR1_INPUT_CC1S(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR1_INPUT_CC1S_SHIFT)) & TIM3_CCMR1_INPUT_CC1S_MASK)

/*!
 * @brief TIM3_CCMR2_OUTPUT Register Bit Definition
 */

#define TIM3_CCMR2_OUTPUT_OC4M_SHIFT                    (12)
#define TIM3_CCMR2_OUTPUT_OC4M_MASK                     (0x07U << TIM3_CCMR2_OUTPUT_OC4M_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC4M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC4M_SHIFT)) & TIM3_CCMR2_OUTPUT_OC4M_MASK)

#define TIM3_CCMR2_OUTPUT_OC4PE_SHIFT                   (11)
#define TIM3_CCMR2_OUTPUT_OC4PE_MASK                    (0x01U << TIM3_CCMR2_OUTPUT_OC4PE_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC4PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC4PE_SHIFT)) & TIM3_CCMR2_OUTPUT_OC4PE_MASK)

#define TIM3_CCMR2_OUTPUT_OC4FE_SHIFT                   (10)
#define TIM3_CCMR2_OUTPUT_OC4FE_MASK                    (0x01U << TIM3_CCMR2_OUTPUT_OC4FE_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC4FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC4FE_SHIFT)) & TIM3_CCMR2_OUTPUT_OC4FE_MASK)

#define TIM3_CCMR2_OUTPUT_CC4S_SHIFT                    (8)
#define TIM3_CCMR2_OUTPUT_CC4S_MASK                     (0x03U << TIM3_CCMR2_OUTPUT_CC4S_SHIFT)
#define TIM3_CCMR2_OUTPUT_CC4S(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_CC4S_SHIFT)) & TIM3_CCMR2_OUTPUT_CC4S_MASK)

#define TIM3_CCMR2_OUTPUT_OC3CE_SHIFT                   (7)
#define TIM3_CCMR2_OUTPUT_OC3CE_MASK                    (0x01U << TIM3_CCMR2_OUTPUT_OC3CE_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC3CE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC3CE_SHIFT)) & TIM3_CCMR2_OUTPUT_OC3CE_MASK)

#define TIM3_CCMR2_OUTPUT_OC3M_SHIFT                    (4)
#define TIM3_CCMR2_OUTPUT_OC3M_MASK                     (0x07U << TIM3_CCMR2_OUTPUT_OC3M_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC3M(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC3M_SHIFT)) & TIM3_CCMR2_OUTPUT_OC3M_MASK)

#define TIM3_CCMR2_OUTPUT_OC3PE_SHIFT                   (3)
#define TIM3_CCMR2_OUTPUT_OC3PE_MASK                    (0x01U << TIM3_CCMR2_OUTPUT_OC3PE_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC3PE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC3PE_SHIFT)) & TIM3_CCMR2_OUTPUT_OC3PE_MASK)

#define TIM3_CCMR2_OUTPUT_OC3FE_SHIFT                   (2)
#define TIM3_CCMR2_OUTPUT_OC3FE_MASK                    (0x01U << TIM3_CCMR2_OUTPUT_OC3FE_SHIFT)
#define TIM3_CCMR2_OUTPUT_OC3FE(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_OC3FE_SHIFT)) & TIM3_CCMR2_OUTPUT_OC3FE_MASK)

#define TIM3_CCMR2_OUTPUT_CC3S_SHIFT                    (0)
#define TIM3_CCMR2_OUTPUT_CC3S_MASK                     (0x03U << TIM3_CCMR2_OUTPUT_CC3S_SHIFT)
#define TIM3_CCMR2_OUTPUT_CC3S(x)                       (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_OUTPUT_CC3S_SHIFT)) & TIM3_CCMR2_OUTPUT_CC3S_MASK)

/*!
 * @brief TIM3_CCMR2_INPUT Register Bit Definition
 */

#define TIM3_CCMR2_INPUT_CC4S_SHIFT                     (8)
#define TIM3_CCMR2_INPUT_CC4S_MASK                      (0x03U << TIM3_CCMR2_INPUT_CC4S_SHIFT)
#define TIM3_CCMR2_INPUT_CC4S(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_INPUT_CC4S_SHIFT)) & TIM3_CCMR2_INPUT_CC4S_MASK)

#define TIM3_CCMR2_INPUT_IC3F_SHIFT                     (4)
#define TIM3_CCMR2_INPUT_IC3F_MASK                      (0x0FU << TIM3_CCMR2_INPUT_IC3F_SHIFT)
#define TIM3_CCMR2_INPUT_IC3F(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_INPUT_IC3F_SHIFT)) & TIM3_CCMR2_INPUT_IC3F_MASK)

#define TIM3_CCMR2_INPUT_IC3PSC_SHIFT                   (2)
#define TIM3_CCMR2_INPUT_IC3PSC_MASK                    (0x03U << TIM3_CCMR2_INPUT_IC3PSC_SHIFT)
#define TIM3_CCMR2_INPUT_IC3PSC(x)                      (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_INPUT_IC3PSC_SHIFT)) & TIM3_CCMR2_INPUT_IC3PSC_MASK)

#define TIM3_CCMR2_INPUT_CC3S_SHIFT                     (0)
#define TIM3_CCMR2_INPUT_CC3S_MASK                      (0x03U << TIM3_CCMR2_INPUT_CC3S_SHIFT)
#define TIM3_CCMR2_INPUT_CC3S(x)                        (((uint32_t)(((uint32_t)(x)) << TIM3_CCMR2_INPUT_CC3S_SHIFT)) & TIM3_CCMR2_INPUT_CC3S_MASK)

/*!
 * @brief TIM3_CCER Register Bit Definition
 */

#define TIM3_CCER_CC4NP_SHIFT                           (15)
#define TIM3_CCER_CC4NP_MASK                            (0x01U << TIM3_CCER_CC4NP_SHIFT)
#define TIM3_CCER_CC4NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC4NP_SHIFT)) & TIM3_CCER_CC4NP_MASK)

#define TIM3_CCER_CC4P_SHIFT                            (13)
#define TIM3_CCER_CC4P_MASK                             (0x01U << TIM3_CCER_CC4P_SHIFT)
#define TIM3_CCER_CC4P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC4P_SHIFT)) & TIM3_CCER_CC4P_MASK)

#define TIM3_CCER_CC4E_SHIFT                            (12)
#define TIM3_CCER_CC4E_MASK                             (0x01U << TIM3_CCER_CC4E_SHIFT)
#define TIM3_CCER_CC4E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC4E_SHIFT)) & TIM3_CCER_CC4E_MASK)

#define TIM3_CCER_CC3NP_SHIFT                           (11)
#define TIM3_CCER_CC3NP_MASK                            (0x01U << TIM3_CCER_CC3NP_SHIFT)
#define TIM3_CCER_CC3NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC3NP_SHIFT)) & TIM3_CCER_CC3NP_MASK)

#define TIM3_CCER_CC3P_SHIFT                            (9)
#define TIM3_CCER_CC3P_MASK                             (0x01U << TIM3_CCER_CC3P_SHIFT)
#define TIM3_CCER_CC3P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC3P_SHIFT)) & TIM3_CCER_CC3P_MASK)

#define TIM3_CCER_CC3E_SHIFT                            (8)
#define TIM3_CCER_CC3E_MASK                             (0x01U << TIM3_CCER_CC3E_SHIFT)
#define TIM3_CCER_CC3E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC3E_SHIFT)) & TIM3_CCER_CC3E_MASK)

#define TIM3_CCER_CC2NP_SHIFT                           (7)
#define TIM3_CCER_CC2NP_MASK                            (0x01U << TIM3_CCER_CC2NP_SHIFT)
#define TIM3_CCER_CC2NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC2NP_SHIFT)) & TIM3_CCER_CC2NP_MASK)

#define TIM3_CCER_CC2P_SHIFT                            (5)
#define TIM3_CCER_CC2P_MASK                             (0x01U << TIM3_CCER_CC2P_SHIFT)
#define TIM3_CCER_CC2P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC2P_SHIFT)) & TIM3_CCER_CC2P_MASK)

#define TIM3_CCER_CC2E_SHIFT                            (4)
#define TIM3_CCER_CC2E_MASK                             (0x01U << TIM3_CCER_CC2E_SHIFT)
#define TIM3_CCER_CC2E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC2E_SHIFT)) & TIM3_CCER_CC2E_MASK)

#define TIM3_CCER_CC1NP_SHIFT                           (3)
#define TIM3_CCER_CC1NP_MASK                            (0x01U << TIM3_CCER_CC1NP_SHIFT)
#define TIM3_CCER_CC1NP(x)                              (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC1NP_SHIFT)) & TIM3_CCER_CC1NP_MASK)

#define TIM3_CCER_CC1P_SHIFT                            (1)
#define TIM3_CCER_CC1P_MASK                             (0x01U << TIM3_CCER_CC1P_SHIFT)
#define TIM3_CCER_CC1P(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC1P_SHIFT)) & TIM3_CCER_CC1P_MASK)

#define TIM3_CCER_CC1E_SHIFT                            (0)
#define TIM3_CCER_CC1E_MASK                             (0x01U << TIM3_CCER_CC1E_SHIFT)
#define TIM3_CCER_CC1E(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCER_CC1E_SHIFT)) & TIM3_CCER_CC1E_MASK)

/*!
 * @brief TIM3_CNT Register Bit Definition
 */

#define TIM3_CNT_CNT_SHIFT                              (0)
#define TIM3_CNT_CNT_MASK                               (0xFFFFU << TIM3_CNT_CNT_SHIFT)
#define TIM3_CNT_CNT(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_CNT_CNT_SHIFT)) & TIM3_CNT_CNT_MASK)

/*!
 * @brief TIM3_PSC Register Bit Definition
 */

#define TIM3_PSC_PSC_SHIFT                              (0)
#define TIM3_PSC_PSC_MASK                               (0xFFFFU << TIM3_PSC_PSC_SHIFT)
#define TIM3_PSC_PSC(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_PSC_PSC_SHIFT)) & TIM3_PSC_PSC_MASK)

/*!
 * @brief TIM3_ARR Register Bit Definition
 */

#define TIM3_ARR_ARR_SHIFT                              (0)
#define TIM3_ARR_ARR_MASK                               (0xFFFFU << TIM3_ARR_ARR_SHIFT)
#define TIM3_ARR_ARR(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM3_ARR_ARR_SHIFT)) & TIM3_ARR_ARR_MASK)

/*!
 * @brief TIM3_CCR1 Register Bit Definition
 */

#define TIM3_CCR1_CCR1_SHIFT                            (0)
#define TIM3_CCR1_CCR1_MASK                             (0xFFFFU << TIM3_CCR1_CCR1_SHIFT)
#define TIM3_CCR1_CCR1(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCR1_CCR1_SHIFT)) & TIM3_CCR1_CCR1_MASK)

/*!
 * @brief TIM3_CCR2 Register Bit Definition
 */

#define TIM3_CCR2_CCR2_SHIFT                            (0)
#define TIM3_CCR2_CCR2_MASK                             (0xFFFFU << TIM3_CCR2_CCR2_SHIFT)
#define TIM3_CCR2_CCR2(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCR2_CCR2_SHIFT)) & TIM3_CCR2_CCR2_MASK)

/*!
 * @brief TIM3_CCR3 Register Bit Definition
 */

#define TIM3_CCR3_CCR3_SHIFT                            (0)
#define TIM3_CCR3_CCR3_MASK                             (0xFFFFU << TIM3_CCR3_CCR3_SHIFT)
#define TIM3_CCR3_CCR3(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCR3_CCR3_SHIFT)) & TIM3_CCR3_CCR3_MASK)

/*!
 * @brief TIM3_CCR4 Register Bit Definition
 */

#define TIM3_CCR4_CCR4_SHIFT                            (0)
#define TIM3_CCR4_CCR4_MASK                             (0xFFFFU << TIM3_CCR4_CCR4_SHIFT)
#define TIM3_CCR4_CCR4(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_CCR4_CCR4_SHIFT)) & TIM3_CCR4_CCR4_MASK)

/*!
 * @brief TIM3_OR Register Bit Definition
 */

#define TIM3_OR_ETRRMP_SHIFT                            (0)
#define TIM3_OR_ETRRMP_MASK                             (0x03U << TIM3_OR_ETRRMP_SHIFT)
#define TIM3_OR_ETRRMP(x)                               (((uint32_t)(((uint32_t)(x)) << TIM3_OR_ETRRMP_SHIFT)) & TIM3_OR_ETRRMP_MASK)

/*!
 * @}
 */ /* end of group TIM3_Register_Masks */
/******************************************************************************
 * TIM3 Instance
 ******************************************************************************/

#define TIM3                ((TIM3_Type*)TIM3_BASE)

/*!
 * @}
 */ /* end of group TIM3_Peripheral_Access_Layer */

/*!
 * @addtogroup TIM14_Peripheral_Access_Layer TIM14 Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * TIM14 Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t CR1;                                                            ///< control register 1                           offset: 0x00
    __IO uint32_t Reserved0[2];                                                   ///< Reserved
    __IO uint32_t DIER;                                                           ///< DMA/Interrupt enable register                offset: 0x0C
    __IO uint32_t SR;                                                             ///< status register                              offset: 0x10
    __IO uint32_t EGR;                                                            ///< event generation register                    offset: 0x14
    __IO uint32_t CCMR1;                                                          ///< compare mode register 1 (output/input mode)  offset: 0x18
    __IO uint32_t Reserved1[1];                                                   ///< Reserved
    __IO uint32_t CCER;                                                           ///< compare enable register                      offset: 0x20
    __IO uint32_t CNT;                                                            ///< counter                                      offset: 0x24
    __IO uint32_t PSC;                                                            ///< prescaler                                    offset: 0x28
    __IO uint32_t ARR;                                                            ///< auto-reload register                         offset: 0x2c
    __IO uint32_t RCR;                                                            ///< repetition counter register                  offset: 0x30
    __IO uint32_t CCR1;                                                           ///< compare register 1                           offset: 0x34
    __IO uint32_t Reserved2[3];                                                   ///< Reserved
    __IO uint32_t BDTR;                                                           ///< Break and dead time register                 offset: 0x44
} TIM14_Type;

/*!
 * @addtogroup TIM14_Register_Masks Register Masks
 * @{ */

/*!
 * @brief TIM14_CR1 Register Bit Definition
 */

#define TIM14_CR1_CKD_SHIFT                             (8)
#define TIM14_CR1_CKD_MASK                              (0x03U << TIM14_CR1_CKD_SHIFT)
#define TIM14_CR1_CKD(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_CR1_CKD_SHIFT)) & TIM14_CR1_CKD_MASK)

#define TIM14_CR1_ARPE_SHIFT                            (7)
#define TIM14_CR1_ARPE_MASK                             (0x01U << TIM14_CR1_ARPE_SHIFT)
#define TIM14_CR1_ARPE(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_CR1_ARPE_SHIFT)) & TIM14_CR1_ARPE_MASK)

#define TIM14_CR1_URS_SHIFT                             (2)
#define TIM14_CR1_URS_MASK                              (0x01U << TIM14_CR1_URS_SHIFT)
#define TIM14_CR1_URS(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_CR1_URS_SHIFT)) & TIM14_CR1_URS_MASK)

#define TIM14_CR1_UDIS_SHIFT                            (1)
#define TIM14_CR1_UDIS_MASK                             (0x01U << TIM14_CR1_UDIS_SHIFT)
#define TIM14_CR1_UDIS(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_CR1_UDIS_SHIFT)) & TIM14_CR1_UDIS_MASK)

#define TIM14_CR1_CEN_SHIFT                             (0)
#define TIM14_CR1_CEN_MASK                              (0x01U << TIM14_CR1_CEN_SHIFT)
#define TIM14_CR1_CEN(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_CR1_CEN_SHIFT)) & TIM14_CR1_CEN_MASK)

/*!
 * @brief TIM14_DIER Register Bit Definition
 */

#define TIM14_DIER_CC1IE_SHIFT                          (1)
#define TIM14_DIER_CC1IE_MASK                           (0x01U << TIM14_DIER_CC1IE_SHIFT)
#define TIM14_DIER_CC1IE(x)                             (((uint32_t)(((uint32_t)(x)) << TIM14_DIER_CC1IE_SHIFT)) & TIM14_DIER_CC1IE_MASK)

#define TIM14_DIER_UIE_SHIFT                            (0)
#define TIM14_DIER_UIE_MASK                             (0x01U << TIM14_DIER_UIE_SHIFT)
#define TIM14_DIER_UIE(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_DIER_UIE_SHIFT)) & TIM14_DIER_UIE_MASK)

/*!
 * @brief TIM14_SR Register Bit Definition
 */

#define TIM14_SR_CC1OF_SHIFT                            (9)
#define TIM14_SR_CC1OF_MASK                             (0x01U << TIM14_SR_CC1OF_SHIFT)
#define TIM14_SR_CC1OF(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_SR_CC1OF_SHIFT)) & TIM14_SR_CC1OF_MASK)

#define TIM14_SR_CC1IF_SHIFT                            (1)
#define TIM14_SR_CC1IF_MASK                             (0x01U << TIM14_SR_CC1IF_SHIFT)
#define TIM14_SR_CC1IF(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_SR_CC1IF_SHIFT)) & TIM14_SR_CC1IF_MASK)

#define TIM14_SR_UIF_SHIFT                              (0)
#define TIM14_SR_UIF_MASK                               (0x01U << TIM14_SR_UIF_SHIFT)
#define TIM14_SR_UIF(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM14_SR_UIF_SHIFT)) & TIM14_SR_UIF_MASK)

/*!
 * @brief TIM14_EGR Register Bit Definition
 */

#define TIM14_EGR_CC1G_SHIFT                            (1)
#define TIM14_EGR_CC1G_MASK                             (0x01U << TIM14_EGR_CC1G_SHIFT)
#define TIM14_EGR_CC1G(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_EGR_CC1G_SHIFT)) & TIM14_EGR_CC1G_MASK)

#define TIM14_EGR_UG_SHIFT                              (0)
#define TIM14_EGR_UG_MASK                               (0x01U << TIM14_EGR_UG_SHIFT)
#define TIM14_EGR_UG(x)                                 (((uint32_t)(((uint32_t)(x)) << TIM14_EGR_UG_SHIFT)) & TIM14_EGR_UG_MASK)

/*!
 * @brief TIM14_CCMR1_OUTPUT Register Bit Definition
 */

#define TIM14_CCMR1_OUTPUT_OC1M_SHIFT                   (4)
#define TIM14_CCMR1_OUTPUT_OC1M_MASK                    (0x07U << TIM14_CCMR1_OUTPUT_OC1M_SHIFT)
#define TIM14_CCMR1_OUTPUT_OC1M(x)                      (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_OUTPUT_OC1M_SHIFT)) & TIM14_CCMR1_OUTPUT_OC1M_MASK)

#define TIM14_CCMR1_OUTPUT_OC1PE_SHIFT                  (3)
#define TIM14_CCMR1_OUTPUT_OC1PE_MASK                   (0x01U << TIM14_CCMR1_OUTPUT_OC1PE_SHIFT)
#define TIM14_CCMR1_OUTPUT_OC1PE(x)                     (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_OUTPUT_OC1PE_SHIFT)) & TIM14_CCMR1_OUTPUT_OC1PE_MASK)

#define TIM14_CCMR1_OUTPUT_OC1FE_SHIFT                  (2)
#define TIM14_CCMR1_OUTPUT_OC1FE_MASK                   (0x01U << TIM14_CCMR1_OUTPUT_OC1FE_SHIFT)
#define TIM14_CCMR1_OUTPUT_OC1FE(x)                     (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_OUTPUT_OC1FE_SHIFT)) & TIM14_CCMR1_OUTPUT_OC1FE_MASK)

#define TIM14_CCMR1_OUTPUT_CC1S_SHIFT                   (0)
#define TIM14_CCMR1_OUTPUT_CC1S_MASK                    (0x03U << TIM14_CCMR1_OUTPUT_CC1S_SHIFT)
#define TIM14_CCMR1_OUTPUT_CC1S(x)                      (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_OUTPUT_CC1S_SHIFT)) & TIM14_CCMR1_OUTPUT_CC1S_MASK)

/*!
 * @brief TIM14_CCMR1_INPUT Register Bit Definition
 */

#define TIM14_CCMR1_INPUT_IC1F_SHIFT                    (4)
#define TIM14_CCMR1_INPUT_IC1F_MASK                     (0x0FU << TIM14_CCMR1_INPUT_IC1F_SHIFT)
#define TIM14_CCMR1_INPUT_IC1F(x)                       (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_INPUT_IC1F_SHIFT)) & TIM14_CCMR1_INPUT_IC1F_MASK)

#define TIM14_CCMR1_INPUT_IC1PSC_SHIFT                  (2)
#define TIM14_CCMR1_INPUT_IC1PSC_MASK                   (0x03U << TIM14_CCMR1_INPUT_IC1PSC_SHIFT)
#define TIM14_CCMR1_INPUT_IC1PSC(x)                     (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_INPUT_IC1PSC_SHIFT)) & TIM14_CCMR1_INPUT_IC1PSC_MASK)

#define TIM14_CCMR1_INPUT_CC1S_SHIFT                    (0)
#define TIM14_CCMR1_INPUT_CC1S_MASK                     (0x03U << TIM14_CCMR1_INPUT_CC1S_SHIFT)
#define TIM14_CCMR1_INPUT_CC1S(x)                       (((uint32_t)(((uint32_t)(x)) << TIM14_CCMR1_INPUT_CC1S_SHIFT)) & TIM14_CCMR1_INPUT_CC1S_MASK)

/*!
 * @brief TIM14_CCER Register Bit Definition
 */

#define TIM14_CCER_CC1NP_SHIFT                          (3)
#define TIM14_CCER_CC1NP_MASK                           (0x01U << TIM14_CCER_CC1NP_SHIFT)
#define TIM14_CCER_CC1NP(x)                             (((uint32_t)(((uint32_t)(x)) << TIM14_CCER_CC1NP_SHIFT)) & TIM14_CCER_CC1NP_MASK)

#define TIM14_CCER_CC1P_SHIFT                           (1)
#define TIM14_CCER_CC1P_MASK                            (0x01U << TIM14_CCER_CC1P_SHIFT)
#define TIM14_CCER_CC1P(x)                              (((uint32_t)(((uint32_t)(x)) << TIM14_CCER_CC1P_SHIFT)) & TIM14_CCER_CC1P_MASK)

#define TIM14_CCER_CC1E_SHIFT                           (0)
#define TIM14_CCER_CC1E_MASK                            (0x01U << TIM14_CCER_CC1E_SHIFT)
#define TIM14_CCER_CC1E(x)                              (((uint32_t)(((uint32_t)(x)) << TIM14_CCER_CC1E_SHIFT)) & TIM14_CCER_CC1E_MASK)

/*!
 * @brief TIM14_CNT Register Bit Definition
 */

#define TIM14_CNT_CNT_SHIFT                             (0)
#define TIM14_CNT_CNT_MASK                              (0xFFFFU << TIM14_CNT_CNT_SHIFT)
#define TIM14_CNT_CNT(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_CNT_CNT_SHIFT)) & TIM14_CNT_CNT_MASK)

/*!
 * @brief TIM14_PSC Register Bit Definition
 */

#define TIM14_PSC_PSC_SHIFT                             (0)
#define TIM14_PSC_PSC_MASK                              (0xFFFFU << TIM14_PSC_PSC_SHIFT)
#define TIM14_PSC_PSC(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_PSC_PSC_SHIFT)) & TIM14_PSC_PSC_MASK)

/*!
 * @brief TIM14_ARR Register Bit Definition
 */

#define TIM14_ARR_ARR_SHIFT                             (0)
#define TIM14_ARR_ARR_MASK                              (0xFFFFU << TIM14_ARR_ARR_SHIFT)
#define TIM14_ARR_ARR(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_ARR_ARR_SHIFT)) & TIM14_ARR_ARR_MASK)

/*!
 * @brief TIM14_RCR Register Bit Definition
 */

#define TIM14_RCR_REP_SHIFT                             (0)
#define TIM14_RCR_REP_MASK                              (0xFFU << TIM14_RCR_REP_SHIFT)
#define TIM14_RCR_REP(x)                                (((uint32_t)(((uint32_t)(x)) << TIM14_RCR_REP_SHIFT)) & TIM14_RCR_REP_MASK)

/*!
 * @brief TIM14_CCR1 Register Bit Definition
 */

#define TIM14_CCR1_CCR1_SHIFT                           (0)
#define TIM14_CCR1_CCR1_MASK                            (0xFFFFU << TIM14_CCR1_CCR1_SHIFT)
#define TIM14_CCR1_CCR1(x)                              (((uint32_t)(((uint32_t)(x)) << TIM14_CCR1_CCR1_SHIFT)) & TIM14_CCR1_CCR1_MASK)

/*!
 * @brief TIM14_BDTR Register Bit Definition
 */

#define TIM14_BDTR_MOE_SHIFT                            (15)
#define TIM14_BDTR_MOE_MASK                             (0x01U << TIM14_BDTR_MOE_SHIFT)
#define TIM14_BDTR_MOE(x)                               (((uint32_t)(((uint32_t)(x)) << TIM14_BDTR_MOE_SHIFT)) & TIM14_BDTR_MOE_MASK)

/*!
 * @}
 */ /* end of group TIM14_Register_Masks */
/******************************************************************************
 * TIM14 Instance
 ******************************************************************************/

#define TIM14                ((TIM14_Type*)TIM14_BASE)

/*!
 * @}
 */ /* end of group TIM14_Peripheral_Access_Layer */

/*!
 * @addtogroup USART_Peripheral_Access_Layer USART Peripheral Access Layer
 *  @{
 */

/*******************************************************************************
 * USART Type
 ******************************************************************************/
typedef struct {
    __IO uint32_t SR;                                                             ///< Status register                              offset: 0x00
    __IO uint32_t DR;                                                             ///< Data register                                offset: 0x04
    __IO uint32_t BRR;                                                            ///< Baud rate register                           offset: 0x08
    __IO uint32_t CR1;                                                            ///< Control register 1                           offset: 0x0C
    __IO uint32_t CR2;                                                            ///< Control register 2                           offset: 0x10
    __IO uint32_t CR3;                                                            ///< Control register 3                           offset: 0x14
} USART_Type;

/*!
 * @addtogroup USART_Register_Masks Register Masks
 * @{ */

/*!
 * @brief USART_SR Register Bit Definition
 */

#define USART_SR_TXE_SHIFT                              (7)
#define USART_SR_TXE_MASK                               (0x01U << USART_SR_TXE_SHIFT)
#define USART_SR_TXE(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_SR_TXE_SHIFT)) & USART_SR_TXE_MASK)

#define USART_SR_TC_SHIFT                               (6)
#define USART_SR_TC_MASK                                (0x01U << USART_SR_TC_SHIFT)
#define USART_SR_TC(x)                                  (((uint32_t)(((uint32_t)(x)) << USART_SR_TC_SHIFT)) & USART_SR_TC_MASK)

#define USART_SR_RXNE_SHIFT                             (5)
#define USART_SR_RXNE_MASK                              (0x01U << USART_SR_RXNE_SHIFT)
#define USART_SR_RXNE(x)                                (((uint32_t)(((uint32_t)(x)) << USART_SR_RXNE_SHIFT)) & USART_SR_RXNE_MASK)

#define USART_SR_IDLE_SHIFT                             (4)
#define USART_SR_IDLE_MASK                              (0x01U << USART_SR_IDLE_SHIFT)
#define USART_SR_IDLE(x)                                (((uint32_t)(((uint32_t)(x)) << USART_SR_IDLE_SHIFT)) & USART_SR_IDLE_MASK)

#define USART_SR_ORE_SHIFT                              (3)
#define USART_SR_ORE_MASK                               (0x01U << USART_SR_ORE_SHIFT)
#define USART_SR_ORE(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_SR_ORE_SHIFT)) & USART_SR_ORE_MASK)

#define USART_SR_NF_SHIFT                               (2)
#define USART_SR_NF_MASK                                (0x01U << USART_SR_NF_SHIFT)
#define USART_SR_NF(x)                                  (((uint32_t)(((uint32_t)(x)) << USART_SR_NF_SHIFT)) & USART_SR_NF_MASK)

#define USART_SR_FE_SHIFT                               (1)
#define USART_SR_FE_MASK                                (0x01U << USART_SR_FE_SHIFT)
#define USART_SR_FE(x)                                  (((uint32_t)(((uint32_t)(x)) << USART_SR_FE_SHIFT)) & USART_SR_FE_MASK)

#define USART_SR_PE_SHIFT                               (0)
#define USART_SR_PE_MASK                                (0x01U << USART_SR_PE_SHIFT)
#define USART_SR_PE(x)                                  (((uint32_t)(((uint32_t)(x)) << USART_SR_PE_SHIFT)) & USART_SR_PE_MASK)

/*!
 * @brief USART_DR Register Bit Definition
 */

#define USART_DR_DR_SHIFT                               (0)
#define USART_DR_DR_MASK                                (0x1FFU << USART_DR_DR_SHIFT)
#define USART_DR_DR(x)                                  (((uint32_t)(((uint32_t)(x)) << USART_DR_DR_SHIFT)) & USART_DR_DR_MASK)

/*!
 * @brief USART_BRR Register Bit Definition
 */

#define USART_BRR_MFD_SHIFT                             (4)
#define USART_BRR_MFD_MASK                              (0xFFFFU << USART_BRR_MFD_SHIFT)
#define USART_BRR_MFD(x)                                (((uint32_t)(((uint32_t)(x)) << USART_BRR_MFD_SHIFT)) & USART_BRR_MFD_MASK)

#define USART_BRR_FFD_SHIFT                             (0)
#define USART_BRR_FFD_MASK                              (0x0FU << USART_BRR_FFD_SHIFT)
#define USART_BRR_FFD(x)                                (((uint32_t)(((uint32_t)(x)) << USART_BRR_FFD_SHIFT)) & USART_BRR_FFD_MASK)

/*!
 * @brief USART_CR1 Register Bit Definition
 */

#define USART_CR1_SAS_SHIFT                             (17)
#define USART_CR1_SAS_MASK                              (0x01U << USART_CR1_SAS_SHIFT)
#define USART_CR1_SAS(x)                                (((uint32_t)(((uint32_t)(x)) << USART_CR1_SAS_SHIFT)) & USART_CR1_SAS_MASK)

#define USART_CR1_MLS_SHIFT                             (16)
#define USART_CR1_MLS_MASK                              (0x01U << USART_CR1_MLS_SHIFT)
#define USART_CR1_MLS(x)                                (((uint32_t)(((uint32_t)(x)) << USART_CR1_MLS_SHIFT)) & USART_CR1_MLS_MASK)

#define USART_CR1_OVER8_SHIFT                           (15)
#define USART_CR1_OVER8_MASK                            (0x01U << USART_CR1_OVER8_SHIFT)
#define USART_CR1_OVER8(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR1_OVER8_SHIFT)) & USART_CR1_OVER8_MASK)

#define USART_CR1_UE_SHIFT                              (13)
#define USART_CR1_UE_MASK                               (0x01U << USART_CR1_UE_SHIFT)
#define USART_CR1_UE(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_CR1_UE_SHIFT)) & USART_CR1_UE_MASK)

#define USART_CR1_DL_SHIFT                              (12)
#define USART_CR1_DL_MASK                               (0x01U << USART_CR1_DL_SHIFT)
#define USART_CR1_DL(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_CR1_DL_SHIFT)) & USART_CR1_DL_MASK)

#define USART_CR1_PCE_SHIFT                             (10)
#define USART_CR1_PCE_MASK                              (0x01U << USART_CR1_PCE_SHIFT)
#define USART_CR1_PCE(x)                                (((uint32_t)(((uint32_t)(x)) << USART_CR1_PCE_SHIFT)) & USART_CR1_PCE_MASK)

#define USART_CR1_PS_SHIFT                              (9)
#define USART_CR1_PS_MASK                               (0x01U << USART_CR1_PS_SHIFT)
#define USART_CR1_PS(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_CR1_PS_SHIFT)) & USART_CR1_PS_MASK)

#define USART_CR1_PEIEN_SHIFT                           (8)
#define USART_CR1_PEIEN_MASK                            (0x01U << USART_CR1_PEIEN_SHIFT)
#define USART_CR1_PEIEN(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR1_PEIEN_SHIFT)) & USART_CR1_PEIEN_MASK)

#define USART_CR1_TXEIEN_SHIFT                          (7)
#define USART_CR1_TXEIEN_MASK                           (0x01U << USART_CR1_TXEIEN_SHIFT)
#define USART_CR1_TXEIEN(x)                             (((uint32_t)(((uint32_t)(x)) << USART_CR1_TXEIEN_SHIFT)) & USART_CR1_TXEIEN_MASK)

#define USART_CR1_TCIEN_SHIFT                           (6)
#define USART_CR1_TCIEN_MASK                            (0x01U << USART_CR1_TCIEN_SHIFT)
#define USART_CR1_TCIEN(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR1_TCIEN_SHIFT)) & USART_CR1_TCIEN_MASK)

#define USART_CR1_RXNEIEN_SHIFT                         (5)
#define USART_CR1_RXNEIEN_MASK                          (0x01U << USART_CR1_RXNEIEN_SHIFT)
#define USART_CR1_RXNEIEN(x)                            (((uint32_t)(((uint32_t)(x)) << USART_CR1_RXNEIEN_SHIFT)) & USART_CR1_RXNEIEN_MASK)

#define USART_CR1_IDLEIEN_SHIFT                         (4)
#define USART_CR1_IDLEIEN_MASK                          (0x01U << USART_CR1_IDLEIEN_SHIFT)
#define USART_CR1_IDLEIEN(x)                            (((uint32_t)(((uint32_t)(x)) << USART_CR1_IDLEIEN_SHIFT)) & USART_CR1_IDLEIEN_MASK)

#define USART_CR1_TE_SHIFT                              (3)
#define USART_CR1_TE_MASK                               (0x01U << USART_CR1_TE_SHIFT)
#define USART_CR1_TE(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_CR1_TE_SHIFT)) & USART_CR1_TE_MASK)

#define USART_CR1_RE_SHIFT                              (2)
#define USART_CR1_RE_MASK                               (0x01U << USART_CR1_RE_SHIFT)
#define USART_CR1_RE(x)                                 (((uint32_t)(((uint32_t)(x)) << USART_CR1_RE_SHIFT)) & USART_CR1_RE_MASK)

#define USART_CR1_SBK_SHIFT                             (0)
#define USART_CR1_SBK_MASK                              (0x01U << USART_CR1_SBK_SHIFT)
#define USART_CR1_SBK(x)                                (((uint32_t)(((uint32_t)(x)) << USART_CR1_SBK_SHIFT)) & USART_CR1_SBK_MASK)

/*!
 * @brief USART_CR2 Register Bit Definition
 */

#define USART_CR2_SWAP_SHIFT                            (15)
#define USART_CR2_SWAP_MASK                             (0x01U << USART_CR2_SWAP_SHIFT)
#define USART_CR2_SWAP(x)                               (((uint32_t)(((uint32_t)(x)) << USART_CR2_SWAP_SHIFT)) & USART_CR2_SWAP_MASK)

#define USART_CR2_STOP_SHIFT                            (12)
#define USART_CR2_STOP_MASK                             (0x03U << USART_CR2_STOP_SHIFT)
#define USART_CR2_STOP(x)                               (((uint32_t)(((uint32_t)(x)) << USART_CR2_STOP_SHIFT)) & USART_CR2_STOP_MASK)

#define USART_CR2_CPOL_SHIFT                            (10)
#define USART_CR2_CPOL_MASK                             (0x01U << USART_CR2_CPOL_SHIFT)
#define USART_CR2_CPOL(x)                               (((uint32_t)(((uint32_t)(x)) << USART_CR2_CPOL_SHIFT)) & USART_CR2_CPOL_MASK)

#define USART_CR2_CPHA_SHIFT                            (9)
#define USART_CR2_CPHA_MASK                             (0x01U << USART_CR2_CPHA_SHIFT)
#define USART_CR2_CPHA(x)                               (((uint32_t)(((uint32_t)(x)) << USART_CR2_CPHA_SHIFT)) & USART_CR2_CPHA_MASK)

/*!
 * @brief USART_CR3 Register Bit Definition
 */

#define USART_CR3_TXTOG_SHIFT                           (29)
#define USART_CR3_TXTOG_MASK                            (0x01U << USART_CR3_TXTOG_SHIFT)
#define USART_CR3_TXTOG(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR3_TXTOG_SHIFT)) & USART_CR3_TXTOG_MASK)

#define USART_CR3_RXTOG_SHIFT                           (28)
#define USART_CR3_RXTOG_MASK                            (0x01U << USART_CR3_RXTOG_SHIFT)
#define USART_CR3_RXTOG(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR3_RXTOG_SHIFT)) & USART_CR3_RXTOG_MASK)

#define USART_CR3_CKINE_SHIFT                           (16)
#define USART_CR3_CKINE_MASK                            (0x01U << USART_CR3_CKINE_SHIFT)
#define USART_CR3_CKINE(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR3_CKINE_SHIFT)) & USART_CR3_CKINE_MASK)

#define USART_CR3_ONEBIT_SHIFT                          (11)
#define USART_CR3_ONEBIT_MASK                           (0x01U << USART_CR3_ONEBIT_SHIFT)
#define USART_CR3_ONEBIT(x)                             (((uint32_t)(((uint32_t)(x)) << USART_CR3_ONEBIT_SHIFT)) & USART_CR3_ONEBIT_MASK)

#define USART_CR3_HDSEL_SHIFT                           (3)
#define USART_CR3_HDSEL_MASK                            (0x01U << USART_CR3_HDSEL_SHIFT)
#define USART_CR3_HDSEL(x)                              (((uint32_t)(((uint32_t)(x)) << USART_CR3_HDSEL_SHIFT)) & USART_CR3_HDSEL_MASK)

#define USART_CR3_ERRIEN_SHIFT                          (0)
#define USART_CR3_ERRIEN_MASK                           (0x01U << USART_CR3_ERRIEN_SHIFT)
#define USART_CR3_ERRIEN(x)                             (((uint32_t)(((uint32_t)(x)) << USART_CR3_ERRIEN_SHIFT)) & USART_CR3_ERRIEN_MASK)

/*!
 * @}
 */ /* end of group USART_Register_Masks */
/******************************************************************************
 * USART Instance
 ******************************************************************************/

#define USART1                ((USART_Type*)USART1_BASE)
#define USART2                ((USART_Type*)USART2_BASE)


/* --------  End of section using anonymous unions and disabling warnings  -------- */
#if   defined (__CC_ARM)
  #pragma pop
#elif defined (__ICCARM__)
  /* leave anonymous unions enabled */
#elif (defined(__ARMCC_VERSION) && (__ARMCC_VERSION >= 6010050))
  #pragma clang diagnostic pop
#elif defined (__GNUC__)
  /* anonymous unions are enabled by default */
#else
  #warning Not supported compiler type
#endif

#ifdef __cplusplus
}
#endif

#endif /* __MM32G0001_H__ */