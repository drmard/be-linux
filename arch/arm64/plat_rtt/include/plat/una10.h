/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (C) 2021 Rusteletech Corporation
 *
 * Author: Sergej Khrumalo <skhrumalo@rusteletech.ru>
 *
 */

#ifndef _LINUX_UNA10_H
#define _LINUX_UNA10_H

#define MACH_TYPE_UNA10       0x251

#include "arch2.h"

#define PCA9555_ADDR01          0x22
#define PCA9555_ADDR02          0x26
#define PCA9546_ADDR01          0x74
#define AB_RTCMC_ADDR01         0x56  // I2C address for RTC AB-RTCMC-32.768KHZ-EOZ9-S3-D-B-T on bus 0x1
#define AT24CS04_ADDR01         0x54  // I2C address for EEPROM AT24CS04-MAHM-T on bus 0x2



#endif /* _LINUX_UNA10_H */

