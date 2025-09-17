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

/**
 * @brief Register an isr_handler into an interrupt group
 *
 * @param group The targeted interrupt group (i.e. the NVIC interrupt).
 * @param int_idx The interrupt group index (aka IIDX).
 * @param isr_handler  Pointer to the ISR function for the device.
 * @param dev Pointer to the device that will service the interrupt.
 *
 * @return 0 on success, a negative errno otherwise.
 */
int mspm0_register_int_to_group(int group, uint8_t int_idx,
				void (*isr_handler)(const struct device *),
				const struct device *dev);

#ifdef __cplusplus
}
#endif

#endif /* soc_h */
