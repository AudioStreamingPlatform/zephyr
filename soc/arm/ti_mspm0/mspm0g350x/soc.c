/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <soc.h>

SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	DL_SYSCTL_setMCLKDivider(DL_SYSCTL_MCLK_DIVIDER_DISABLE);
	DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_1);

	// Low Power Mode is configured to be SLEEP0
	DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
}

static int ti_mspm0g350x_init(void)
{
	/* SYSCFG_DL_SYSCTL_init(); */
	return 0;
}

SYS_INIT(ti_mspm0g350x_init, PRE_KERNEL_1, 0);
