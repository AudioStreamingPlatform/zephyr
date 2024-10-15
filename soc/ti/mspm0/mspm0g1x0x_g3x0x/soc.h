/*
 * Copyright (c) 2024 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef soc_h
#define soc_h

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

#define POWER_STARTUP_DELAY (16)

#if CONFIG_CAN_USE_HFXT
#define SOC_MSPM0_CAN_USE_HFXT (true)
#else
#define SOC_MSPM0_CAN_USE_HFXT (false)
#endif

#ifdef __cplusplus
}
#endif

#endif /* soc_h */
