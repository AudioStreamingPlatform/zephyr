/*
 * Copyright (c) 2024 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <ti/driverlib/m0p/dl_core.h>
#include <ti/driverlib/m0p/dl_interrupt.h>
#include <soc.h>

#include <zephyr/irq.h>
#include <errno.h>

struct mspm0_int_grp_iidx {
	void (*isr_handler)(const struct device *);
	const struct device *dev;
};

static struct mspm0_int_grp_iidx mspm0_int_grp0[8] = { 0 };
static uint32_t mspm0_int_grp0_mask = 0;

static struct mspm0_int_grp_iidx mspm0_int_grp1[8] = { 0 };
static uint32_t mspm0_int_grp1_mask = 0;

static ALWAYS_INLINE void handle_group_isr(int group, uint32_t int_grp_mask,
					   struct mspm0_int_grp_iidx *int_grp)
{
	uint32_t triggered = DL_Interrupt_getStatusGroup(group,
							 int_grp_mask) & 0xff;

	DL_Interrupt_clearGroup(group, triggered);

	/* Groups are priority-ordered: the lower the index,
	 * the higher the priority.
	 */
	while (triggered) {
		if (triggered & 1) {
			int_grp->isr_handler(int_grp->dev);
		}

		int_grp++;
		triggered >>= 1;
	}
}

static void mspm0_int_group_0_isr(const struct device *unused)
{
	ARG_UNUSED(unused);

	handle_group_isr(0, mspm0_int_grp0_mask, mspm0_int_grp0);
}

static void mspm0_int_group_1_isr(const struct device *unused)
{
	ARG_UNUSED(unused);

	handle_group_isr(1, mspm0_int_grp1_mask, mspm0_int_grp1);
}

static int register_group_isr(uint8_t int_idx,
			      uint32_t *int_grp_mask,
			      struct mspm0_int_grp_iidx int_grp[8],
			      void (*isr_handler)(const struct device *),
			      const struct device *dev)
{
	/* IIDX spans from 1 to 8, but it's addressed from 0 to 7 */
	int_idx--;
	if (*int_grp_mask & BIT(int_idx)) {
		return -EALREADY;
	}

	int_grp[int_idx].isr_handler = isr_handler;
	int_grp[int_idx].dev = dev;

	*int_grp_mask |= BIT(int_idx);

	return 0;
}

int mspm0_register_int_to_group(int group, uint8_t int_idx,
				void (*isr_handler)(const struct device *),
				const struct device *dev)
{
	int ret;

	if (group > 1 || int_idx > 8 || int_idx < 1 || isr_handler == NULL) {
		return -EINVAL;
	}

	if (group == 0) {
		ret = register_group_isr(int_idx, &mspm0_int_grp0_mask,
					 mspm0_int_grp0, isr_handler, dev);
	} else {
		ret = register_group_isr(int_idx, &mspm0_int_grp1_mask,
					 mspm0_int_grp1, isr_handler, dev);
	}

	if (ret == 0) {
		irq_enable(group);
	}

	return ret;
}

static int ti_mspm0g_init(void)
{
	/* Reset and enable GPIO banks */
	DL_GPIO_reset(GPIOA);
	DL_GPIO_reset(GPIOB);

	DL_GPIO_enablePower(GPIOA);
	DL_GPIO_enablePower(GPIOB);

	/* Allow delay time to settle */
	delay_cycles(POWER_STARTUP_DELAY);

	/* Low Power Mode is configured to be SLEEP0 */
	DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

	/* INT_GRP0 */
	IRQ_CONNECT(0, 0, mspm0_int_group_0_isr, NULL, 0);
	/* INT_GRP1 */
	IRQ_CONNECT(1, 0, mspm0_int_group_1_isr, NULL, 0);

	return 0;
}

SYS_INIT(ti_mspm0g_init, PRE_KERNEL_1, 0);
