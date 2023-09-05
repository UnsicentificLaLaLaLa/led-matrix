/*
 * Copyright 2023 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HAL_RCC_H__
#define __HAL_RCC_H__

#include "hal_common.h"

/*!
 * @addtogroup RCC
 * @{
 */

/*!
 * @addtogroup RCC_AHB1_PERIPH
 * @{
 */

#define RCC_AHB1_PERIPH_SRAM   (1u << 2u)  /*!< AHB1 peripheral SRAM clock 2 bit. */
#define RCC_AHB1_PERIPH_FLASH  (1u << 4u)  /*!< AHB1 peripheral FLASH clock 4 bit. */
#define RCC_AHB1_PERIPH_CRC    (1u << 6u)  /*!< AHB1 peripheral CRC clock 6 bit. */
#define RCC_AHB1_PERIPH_GPIOA  (1u << 17u) /*!< AHB1 peripheral GPIOA clock 17 bit. */
#define RCC_AHB1_PERIPH_GPIOB  (1u << 18u) /*!< AHB1 peripheral GPIOB clock 18 bit. */
/*!
 * @}
 */

/*!
 * @addtogroup RCC_APB1_PERIPH
 * @{
 */
#define RCC_APB1_PERIPH_TIM3   (1u << 1u) /*!< APB1 peripheral TIM3 clock 1 bit. */
#define RCC_APB1_PERIPH_TIM1   (1u << 2u) /*!< APB1 peripheral TIM1 clock 1 bit. */
#define RCC_APB1_PERIPH_TIM14  (1u << 3u) /*!< APB1 peripheral TIM14 clock 1 bit. */
#define RCC_APB1_PERIPH_ADC1   (1u << 9u) /*!< APB1 peripheral ADC1 clock 9 bit. */
#define RCC_APB1_PERIPH_SPI1   (1u << 12u) /*!< APB1 peripheral SPI1 clock 14 bit. */
#define RCC_APB1_PERIPH_USART1 (1u << 16u) /*!< APB1 peripheral USART1 clock 16 bit. */
#define RCC_APB1_PERIPH_USART2 (1u << 17u) /*!< APB1 peripheral USART2 clock 17 bit. */
#define RCC_APB1_PERIPH_I2C1   (1u << 21u) /*!< APB1 peripheral I2C1 clock 21 bit. */
#define RCC_APB1_PERIPH_PWR    (1u << 28u) /*!< APB1 peripheral PWR clock 28 bit. */
#define RCC_APB1_PERIPH_DBG    (1u << 29u) /*!< APB1 peripheral DBG clock 28 bit. */
#define RCC_APB1_PERIPH_SYSCFG (1u << 30u) /*!< APB1 peripheral SYSCFG clock 30 bit. */
/*!
 * @}
 */


/*!
 * @brief Define the clock source for MCO output.
 */
typedef enum
{
    RCC_MCO_NoClock     = 0u, /*!< MCO NoClock 0 bits. */
    RCC_MCO_LSI         = 2u, /*!< MCO LSI 2 bits. */
    RCC_MCO_SYSCLK      = 4u, /*!< MCO SYSCLK 4 bits. */
    RCC_MCO_HSI         = 5u, /*!< MCO HSI/4 5 bits. */
    RCC_MCO_HSE         = 6u, /*!< MCO HSE 6 bits. */
} RCC_MCO_Type;

/*!
 * @brief Enable the RCC AHB1 periphs clock module.
 *
 * @param ahb1_periphs AHB1 Peripheral instance.
 * @param enable 'true' to enable the AHB1Periphs, 'false' to disable the AHB1Periphs.
 * @return None.
 */
void RCC_EnableAHB1Periphs(uint32_t ahb1_periphs, bool enable);

/*!
 * @brief Reset the RCC AHB1 periphs clock module.
 *
 * @param ahb1_periphs AHB1 Peripheral instance.
 * @return None.
 */
void RCC_ResetAHB1Periphs(uint32_t ahb1_periphs);

/*!
 * @brief Enable the RCC APB1 periphs clock module.
 *
 * @param apb1_periphs APB1 Peripheral instance.
 * @param enable 'true' to enable the APB1Periphs, 'false' to disable the APB1Periphs.
 * @return None.
 */
void RCC_EnableAPB1Periphs(uint32_t apb1_periphs, bool enable);

/*!
 * @brief Reset the RCC APB1 periphs clock module.
 *
 * @param apb1_periphs APB1 Peripheral instance.
 * @return None.
 */
void RCC_ResetAPB1Periphs(uint32_t apb1_periphs);

/*!
 * @brief MCO  Configuration clock source.
 *
 * @param source The clock source for MCO Configuration.
 * @return None.
 */
void RCC_MCOConf(RCC_MCO_Type source);

/*!
 *@}
 */

#endif /* __HAL_RCC_H__ */
