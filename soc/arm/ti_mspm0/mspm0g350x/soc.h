/*
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

// FIXME: Should be specified in DTS?
#define CPUCLK_FREQ 32000000

#include <ti/devices/msp/msp.h>
#include "ti/driverlib/m0p/sysctl/dl_sysctl_mspm0g1x0x_g3x0x.h"

void SYSCFG_DL_SYSCTL_init(void);

