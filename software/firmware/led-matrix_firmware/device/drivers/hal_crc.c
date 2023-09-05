/*
 * Copyright 2022 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hal_crc.h"

/* Put data into CRC. */
void CRC_SetData(CRC_Type * CRCx, uint32_t val)
{
    CRCx->DR = val;
}

/* Get the result of CRC. */
uint32_t CRC_GetResult(CRC_Type * CRCx)
{
    return CRCx->DR;
}

/* Reset CRC. */
void CRC_Reset(CRC_Type * CRCx)
{
    CRCx->CTRL |= CRC_CTRL_RESET_MASK;
}

/* Temporarily store 1 byte of data. */
void CRC_SetIDR(CRC_Type * CRCx, uint32_t val)
{
    CRCx->IDR = (uint8_t)val;
}

/* Get 1 byte of stored data. */
uint32_t CRC_GetIDR(CRC_Type * CRCx)
{
    return CRCx->IDR;
}

/* EOF. */
