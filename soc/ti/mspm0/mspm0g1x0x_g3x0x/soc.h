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

/* Q&D fix until the driver uses clock-control driver*/
#define SOC_MSPM0_HFCLK_FREQ_HZ MHZ(40)
#define SOC_MSPM0_SYSPLL_FREQ_HZ MHZ(40)

#ifdef __cplusplus
}
#endif

#endif /* soc_h */
