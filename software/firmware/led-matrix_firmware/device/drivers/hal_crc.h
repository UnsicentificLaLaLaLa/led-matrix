/*
 * Copyright 2022 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HAL_CRC_H__
#define __HAL_CRC_H__

#include "hal_common.h"

/*!
 * @addtogroup CRC
 * @{
 */

/*!
 * @brief Set data for CRC calculation.
 *
 * @param CRCx CRC instance.
 * @param val The value used for CRC calculation.
 * @return None.
 */
void CRC_SetData(CRC_Type * CRCx, uint32_t val);

/*!
 * @brief Get the most recent result of CRC calculation.
 *
 * @param CRCx CRC instance.
 * @return Return the most recent result of CRC calculation.
 */
uint32_t CRC_GetResult(CRC_Type * CRCx);

/*!
 * @brief Reset CRC data.
 *
 * @param CRCx CRC instance.
 * @return None.
 */
void CRC_Reset(CRC_Type * CRCx);

/*!
 * @brief Temporarily store 1 byte.
 *
 * @param CRCx CRC instance.
 * @param val Temporarily store 1 byte data.
 * @return None.
 */
void CRC_SetIDR(CRC_Type * CRCx, uint32_t val);

/*!
 * @brief Get Temporarily store 1 byte.
 *
 * @param CRCx CRC instance.
 * @return Temporarily store 1 byte data.
 */
uint32_t CRC_GetIDR(CRC_Type * CRCx);

/*!
 *@}
 */
 
#endif /*__HAL_CRC_H__. */

