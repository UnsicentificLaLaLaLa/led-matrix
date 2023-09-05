/*
 * Copyright 2022 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hal_pwr.h"

void PWR_EnterStopMode(PWR_Type * PWRx, bool deep_stop)
{   
    if (deep_stop)
    {
        PWRx->CR |= PWR_CR_LPDS_MASK;
    }
    else
    {
        PWRx->CR &= ~PWR_CR_LPDS_MASK;
    }
}

uint32_t PWR_GetStatus(PWR_Type * PWRx)
{
    return PWRx->CSR;
}

/* EOF. */
