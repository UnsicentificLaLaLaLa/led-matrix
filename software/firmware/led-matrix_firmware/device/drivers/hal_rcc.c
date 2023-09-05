/*
 * Copyright 2023 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hal_common.h"
#include "hal_rcc.h"

void RCC_EnableAHB1Periphs(uint32_t ahb1_periphs, bool enable)
{
    (enable) ? (RCC->AHBENR |= ahb1_periphs) : (RCC->AHBENR &= ~ahb1_periphs);
}

void RCC_EnableAPB1Periphs(uint32_t apb1_periphs, bool enable)
{
    (enable) ? (RCC->APB1ENR |= apb1_periphs) : (RCC->APB1ENR &= ~apb1_periphs);
}


void RCC_ResetAHB1Periphs(uint32_t ahb1_periphs)
{
    RCC->AHBRSTR |= ahb1_periphs;
    RCC->AHBRSTR &= ~ahb1_periphs;
}

void RCC_ResetAPB1Periphs(uint32_t apb1_periphs)
{
    RCC->APB1RSTR |= apb1_periphs;
    RCC->APB1RSTR &= ~apb1_periphs;
}

void RCC_MCOConf(RCC_MCO_Type source)
{
    RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_MCO_MASK) | RCC_CFGR_MCO(source);
}

/* EOF. */
