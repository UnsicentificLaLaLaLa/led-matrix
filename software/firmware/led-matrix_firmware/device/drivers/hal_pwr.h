/*
 * Copyright 2022 MindMotion Microelectronics Co., Ltd.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __HAL_PWR_H__
#define __HAL_PWR_H__

#include "hal_common.h"

/*!
 * @addtogroup PWR
 * @{
 */

/*!
 * @brief PWR driver version number.
 */
#define PWR_DRIVER_VERSION 3u /*!< PWR_3. */

/*!
 * @brief Enter stop mode.
 *
 * @param PWRx PWR instance.
 * @param deep_stop 'true' to enable the deep stop mode, 'false' to disable the deep stop mode.
 * @return None.
 */
void PWR_EnterStopMode(PWR_Type * PWRx, bool deep_stop);

/*!
 *@}
 */

#endif /* __HAL_PWR_H__ */
