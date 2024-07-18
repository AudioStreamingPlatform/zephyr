/*
 * Copyright (c) 2024 Bang & Olufsen A/S, Denmark
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_pwm

#include <ti/driverlib/dl_timera.h>
#include <ti/driverlib/dl_timerg.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/logging/log.h>

#define REG_TIMA0         (GPTIMER_Regs *)0x40860000
#define REG_TIMA1         (GPTIMER_Regs *)0x40862000
#define IS_TIMA(tim)      (tim == REG_TIMA0 || tim == REG_TIMA1)
// from the datasheet, only TIMA0 has 4 capture compare channels
#define HAS_FOUR_CC(tim)  (tim == REG_TIMA0)
#define NUM_CHANNELS(tim) (HAS_FOUR_CC(tim) ? 4 : 2)

LOG_MODULE_REGISTER(pwm_mspm0, CONFIG_PWM_LOG_LEVEL);

struct pwm_mspm0_config {
	GPTIMER_Regs *timer;
	const struct pinctrl_dev_config *pinctrl;
	const uint32_t frequency;
	const DL_Timer_ClockConfig clock_cfg;
	const DL_TIMER_PWM_MODE pwm_mode;
};

static inline int pwm_mspm0_configure_cc_output(GPTIMER_Regs *clock_cfg, const int cc_index)
{
	int direction = 0;
	switch (cc_index) {
	case DL_TIMER_CC_0_INDEX:
		direction = DL_TIMER_CC0_OUTPUT;
		break;
	case DL_TIMER_CC_1_INDEX:
		direction = DL_TIMER_CC1_OUTPUT;
		break;
	case DL_TIMER_CC_2_INDEX:
		direction = DL_TIMER_CC2_OUTPUT;
		break;
	case DL_TIMER_CC_3_INDEX:
		direction = DL_TIMER_CC3_OUTPUT;
		break;
	default:
		LOG_DBG("Error, invalid CC index: %d", cc_index);
		return -ENOTSUP;
	};

	DL_Timer_setCCPDirection(clock_cfg, direction);

	return 0;
}

static int pwm_mspm0_get_cycles_per_sec(const struct device *dev, uint32_t channel,
					uint64_t *cycles)
{
	const struct pwm_mspm0_config *config = dev->config;
	uint8_t max_allowed_channels = NUM_CHANNELS(config->timer);

	if (channel >= max_allowed_channels) {
		LOG_ERR("Error, invalid channel: %d", channel);
		return -ENXIO;
	}

	*cycles = config->frequency /
		  ((config->clock_cfg.divideRatio + 1) * (config->clock_cfg.prescale + 1));

	return 0;
}

static inline uint32_t pwm_mspm0_polarity_from_flags(pwm_flags_t flags)
{
	if (flags & PWM_POLARITY_INVERTED) {
		return DL_TIMER_CC_OCTL_INV_OUT_ENABLED;
	}

	return DL_TIMER_CC_OCTL_INV_OUT_DISABLED;
}

static int pwm_mspm0_set_cycles(const struct device *dev, uint32_t channel, uint32_t period_cycles,
				uint32_t pulse_cycles, pwm_flags_t flags)
{
	const struct pwm_mspm0_config *cfg = dev->config;
	uint8_t max_allowed_channels = NUM_CHANNELS(cfg->timer);

	if (channel >= max_allowed_channels) {
		LOG_ERR("Error, invalid channel: %d", channel);
		return -ENXIO;
	}

	if (pulse_cycles == 0) {
		DL_Timer_stopCounter(cfg->timer);
		return 0;
	}

	if (IS_TIMA(cfg->timer)) {
		const DL_TimerA_PWMConfig pwm_cfg = {
			.pwmMode = cfg->pwm_mode,
			.isTimerWithFourCC = HAS_FOUR_CC(cfg->timer),
			.period = period_cycles,
		};

		DL_TimerA_initPWMMode(cfg->timer, (DL_TimerA_PWMConfig *)&pwm_cfg);
	} else {
		const DL_TimerG_PWMConfig pwm_cfg = {
			.pwmMode = cfg->pwm_mode,
			.period = period_cycles,
		};

		DL_TimerG_initPWMMode(cfg->timer, (DL_TimerG_PWMConfig *)&pwm_cfg);
	}

	DL_Timer_setCaptureCompareOutCtl(cfg->timer, DL_TIMER_CC_OCTL_INIT_VAL_HIGH,
					 pwm_mspm0_polarity_from_flags(flags),
					 DL_TIMER_CC_OCTL_SRC_FUNCVAL, channel);
	DL_Timer_setCaptureCompareValue(cfg->timer, pulse_cycles, channel);

	int err = pwm_mspm0_configure_cc_output(cfg->timer, channel);
	if (err < 0) {
		DL_Timer_disablePower(cfg->timer);
		return err;
	}

	DL_Timer_startCounter(cfg->timer);

	return 0;
}

static const struct pwm_driver_api pwm_mspm0_driver_api = {
	.set_cycles = pwm_mspm0_set_cycles,
	.get_cycles_per_sec = pwm_mspm0_get_cycles_per_sec,
};

static int pwm_mspm0_init(const struct device *dev)
{
	const struct pwm_mspm0_config *cfg = dev->config;

	int ret = pinctrl_apply_state(cfg->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret) {
		LOG_ERR("Error, failed to apply pinctrl: %d", ret);
		return ret;
	}

	DL_Timer_reset(cfg->timer);
	DL_Timer_enablePower(cfg->timer);
	DL_Timer_setClockConfig(cfg->timer, (DL_Timer_ClockConfig *)&cfg->clock_cfg);
	DL_Timer_enableClock(cfg->timer);

	return 0;
}

#define PWM_MSPM0_INIT(inst)                                                                       \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
                                                                                                   \
	const struct pwm_mspm0_config pwm_mspm0_##inst##_cfg = {                                   \
		.timer = ((GPTIMER_Regs *)DT_INST_REG_ADDR(inst)),                                 \
		.clock_cfg =                                                                       \
			{                                                                          \
				.clockSel = DL_TIMER_CLOCK_BUSCLK,                                 \
				.divideRatio = DT_PROP(DT_DRV_INST(inst), ti_divide_ratio),        \
				.prescale = DT_PROP(DT_DRV_INST(inst), ti_prescaler),              \
			},                                                                         \
		.pwm_mode = DT_PROP(DT_DRV_INST(inst), ti_mode),                                   \
		.frequency = DT_PROP(DT_DRV_INST(inst), ti_clock_frequency),                       \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, pwm_mspm0_init, NULL, NULL, &pwm_mspm0_##inst##_cfg,           \
			      POST_KERNEL, CONFIG_APPLICATION_INIT_PRIORITY,                       \
			      &pwm_mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_MSPM0_INIT)
