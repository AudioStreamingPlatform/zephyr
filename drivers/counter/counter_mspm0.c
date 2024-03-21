/*
 * Copyright (c) 2024 Bang & Olufsen A/S, Denmark
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ti/driverlib/dl_timerg.h>
#include <ti/driverlib/dl_timer.h>
#include <zephyr/logging/log.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/counter.h>

LOG_MODULE_REGISTER(counter_mspm0, CONFIG_LOG_DEFAULT_LEVEL);

#define DT_DRV_COMPAT ti_mspm0_counter
// NOTE: it seems(?) that the clocks support multiple input channels but
// they must be configured with capture mode
#define MAX_CHANNELS  1

struct counter_mspm0_channel_data {
	counter_alarm_callback_t callback;
	void *user_data;
};

struct counter_mspm0_config {
	struct counter_config_info info;
	GPTIMER_Regs *timer;
	const DL_Timer_ClockConfig clock_cfg;
	void (*interrupt_init_function)(const struct device *dev);
};

struct counter_mspm0_data {
	const struct counter_mspm0_config *config;
	struct counter_mspm0_channel_data *channel_data;
	uint32_t freq;
	DL_Timer_TimerConfig time_cfg;
	struct k_sem busy_sem;
};

static int counter_mspm0_start(const struct device *dev)
{
	struct counter_mspm0_data *data = dev->data;
	if (k_sem_take(&data->busy_sem, K_FOREVER) == 0) {
		const struct counter_mspm0_config *cfg = dev->config;
		DL_Timer_reset(cfg->timer);
		DL_Timer_enablePower(cfg->timer);

		DL_Timer_setClockConfig(cfg->timer, (DL_Timer_ClockConfig *)&cfg->clock_cfg);
		DL_Timer_initTimerMode(cfg->timer, &data->time_cfg);

		DL_Timer_enableInterrupt(cfg->timer, DL_TIMER_INTERRUPT_ZERO_EVENT);
		cfg->interrupt_init_function(dev);

		DL_Timer_enableClock(cfg->timer);
		DL_Timer_startCounter(cfg->timer);

		k_sem_give(&data->busy_sem);
	}
	return 0;
}

static int counter_mspm0_stop(const struct device *dev)
{
	struct counter_mspm0_data *data = dev->data;
	if (k_sem_take(&data->busy_sem, K_FOREVER) == 0) {
		const struct counter_mspm0_config *cfg = dev->config;
		DL_Timer_stopCounter(cfg->timer);

		k_sem_give(&data->busy_sem);
	}
	return 0;
}

static int counter_mspm0_set_alarm(const struct device *dev, uint8_t chan,
				   const struct counter_alarm_cfg *alarm_cfg)
{
	// NOTE: we only support only 1 channel
	if (chan != 0) {
		LOG_ERR("Unsupported channel: %d", chan);
		return -EINVAL;
	}

	struct counter_mspm0_data *data = dev->data;
	if (k_sem_take(&data->busy_sem, K_FOREVER) == 0) {

		struct counter_mspm0_channel_data *channel_data = &data->channel_data[chan];
		if (channel_data->callback) {
			LOG_ERR("Callback is already registered");
			k_sem_give(&data->busy_sem);
			return -EBUSY;
		}

		channel_data->callback = alarm_cfg->callback;
		channel_data->user_data = alarm_cfg->user_data;
		data->time_cfg.period = alarm_cfg->ticks;

		k_sem_give(&data->busy_sem);
	}
	return 0;
}

static int counter_mspm0_cancel_alarm(const struct device *dev, uint8_t chan)
{
	// NOTE: we only support only 1 channel
	if (chan != 0) {
		LOG_ERR("Unsupported channel: %d", chan);
		return -1;
	}

	struct counter_mspm0_data *data = dev->data;
	if (k_sem_take(&data->busy_sem, K_FOREVER) == 0) {

		struct counter_mspm0_channel_data *channel_data = &data->channel_data[chan];
		if (channel_data) {
			channel_data[chan].callback = NULL;
		}

		k_sem_give(&data->busy_sem);
	}
	return 0;
}

static uint32_t counter_mspm0_get_freq(const struct device *dev)
{
	struct counter_mspm0_data *data = dev->data;
	const struct counter_mspm0_config *cfg = dev->config;
	const DL_Timer_ClockConfig clock_cfg = cfg->clock_cfg;
	// freq = (timer_clk_source / (div_ratio * (prescale + 1))
	return data->freq / (clock_cfg.divideRatio * (clock_cfg.prescale + 1));
}

static const struct counter_driver_api counter_mspm0_driver_api = {
	.start = counter_mspm0_start,
	.stop = counter_mspm0_stop,
	.set_alarm = counter_mspm0_set_alarm,
	.cancel_alarm = counter_mspm0_cancel_alarm,
	.get_freq = counter_mspm0_get_freq,
};

static void counter_mspm0_isr(const struct device *dev)
{
	struct counter_mspm0_data *data = dev->data;
	// NOTE: can this be problematic (?) when multiple points call a
	// critical function (?)
	if (k_sem_take(&data->busy_sem, K_NO_WAIT) == 0) {
		// NOTE: its hardcoded to always fetch the 1st available channel
		struct counter_mspm0_channel_data *channel_data = &data->channel_data[0];
		counter_alarm_callback_t cb = channel_data->callback;
		if (cb) {
			cb(dev, 0, 0, channel_data->user_data);
		}

		k_sem_give(&data->busy_sem);
	}
}

static int counter_mspm0_init(const struct device *dev)
{
	struct counter_mspm0_data *data = dev->data;
	int err = k_sem_init(&data->busy_sem, 0, 1);
	if (err != 0) {
		LOG_ERR("Failed to create semaphore: %d", err);
		return err;
	}

	k_sem_give(&data->busy_sem);
	return 0;
}

/* Macros to assist with the device-specific initialization */
#define INTERRUPT_INIT_FUNCTION_DECLARATION(inst)                                                  \
	static void counter_mspm0_interrupt_init_##inst(const struct device *dev)

#define INTERRUPT_INIT_FUNCTION(inst)                                                              \
	static void counter_mspm0_interrupt_init_##inst(const struct device *dev)                  \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), counter_mspm0_isr,    \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}

#define COUNTER_MSPM0_INIT(inst)                                                                   \
                                                                                                   \
	INTERRUPT_INIT_FUNCTION_DECLARATION(inst);                                                 \
                                                                                                   \
	static struct counter_mspm0_channel_data counter##inst##_channel_data[MAX_CHANNELS];       \
                                                                                                   \
	const DL_Timer_ClockConfig counter_mspm0_clock_cfg_##inst = {                              \
		.clockSel = DL_TIMER_CLOCK_BUSCLK,                                                 \
		.prescale = DT_PROP(DT_DRV_INST(inst), prescaler),                                 \
		.divideRatio = DT_PROP(DT_DRV_INST(inst), divide_ratio),                           \
	};                                                                                         \
                                                                                                   \
	const struct counter_mspm0_config counter_mspm0_##inst##_cfg = {                           \
		.info =                                                                            \
			{                                                                          \
				.max_top_value = DT_PROP(DT_DRV_INST(inst), resolution) == 32      \
							 ? BIT64_MASK(32)                          \
							 : BIT_MASK(16),                           \
				.flags = 0,                                                        \
				.channels = 1,                                                     \
			},                                                                         \
		.timer = ((GPTIMER_Regs *)DT_INST_REG_ADDR(inst)),                                 \
		.clock_cfg = counter_mspm0_clock_cfg_##inst,                                       \
		.interrupt_init_function = counter_mspm0_interrupt_init_##inst,                    \
	};                                                                                         \
                                                                                                   \
	static struct counter_mspm0_data counter_mspm0_##inst##_data = {                           \
		.config = &counter_mspm0_##inst##_cfg,                                             \
		.channel_data = counter##inst##_channel_data,                                      \
		.freq = DT_PROP(DT_DRV_INST(inst), clock_frequency),                               \
		.time_cfg =                                                                        \
			{                                                                          \
				.timerMode = DT_PROP(DT_DRV_INST(inst), mode),                     \
				.startTimer = DL_TIMER_STOP,                                       \
			},                                                                         \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, counter_mspm0_init, NULL, &counter_mspm0_##inst##_data,        \
			      &counter_mspm0_##inst##_cfg, POST_KERNEL,                            \
			      CONFIG_COUNTER_INIT_PRIORITY, &counter_mspm0_driver_api);            \
                                                                                                   \
	INTERRUPT_INIT_FUNCTION(inst)

DT_INST_FOREACH_STATUS_OKAY(COUNTER_MSPM0_INIT)
