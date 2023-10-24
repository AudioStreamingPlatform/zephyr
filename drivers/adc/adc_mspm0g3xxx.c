/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_mspm0g3xxx_adc

#include <errno.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(adc_mspm0g3xxx);

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/drivers/adc.h>
#include <soc.h>

/* Driverlib includes */
#include <ti/driverlib/dl_adc12.h>
#include <ti/driverlib/dl_gpio.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

/**
 * Maximum number of channels supported per ADC is 17
 */
#define MAX_CHAN_ID 0x11

/** Internal sample time unit conversion entry. */
struct adc_mspm0g3xxx_sample_time_entry {
	uint16_t time_us;
	uint8_t reg_value;
};

/** Maps standard unit sample times (us) to internal (raw hal_ti register) values */
/* static const struct adc_mspm0g3xxx_sample_time_entry adc_g3xxx_sample_times[] = { */
/* 	{ 2, 1 }, */
/* }; */

struct adc_mspm0g3xxx_data {
	struct adc_context ctx;
	const struct device *dev;
	uint32_t ref_source;
	uint8_t sample_time;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
};

struct adc_mspm0g3xxx_cfg {
	unsigned long base;
	void (*irq_cfg_func)(void);
};


static void adc_mspm0g3xxx_isr(const struct device *dev);


static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_mspm0g3xxx_data *data =
		CONTAINER_OF(ctx, struct adc_mspm0g3xxx_data, ctx);

	data->repeat_buffer = data->buffer;
#ifdef UNDO
	AUXADCEnableSync(data->ref_source, data->sample_time, AUXADC_TRIGGER_MANUAL);
	AUXADCGenManualTrigger();
#endif
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat)
{
	struct adc_mspm0g3xxx_data *data =
		CONTAINER_OF(ctx, struct adc_mspm0g3xxx_data, ctx);

	if (repeat) {
		data->buffer = data->repeat_buffer;
	} else {
		data->buffer++;
	}
}

static int adc_mspm0g3xxx_init(const struct device *dev)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	/* const struct adc_mspm0g3xxx_cfg *config = dev->config; */

	data->dev = dev;
#ifdef UNDO
	/* clear any previous events */
	AUXADCDisable();
	HWREG(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR) =
		(AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_IRQ | AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_DONE);

	config->irq_cfg_func();
#endif
	adc_context_unlock_unconditionally(&data->ctx);
	return 0;
}

static int adc_mspm0g3xxx_channel_setup(const struct device *dev,
				    const struct adc_channel_cfg *channel_cfg)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	const uint8_t ch = channel_cfg->channel_id;
	/* uint16_t sample_time_us = 0; */
	/* uint8_t i; */

	if (ch > MAX_CHAN_ID) {
		LOG_ERR("Channel 0x%X is not supported, max 0x%X", ch, MAX_CHAN_ID);
		return -EINVAL;
	}
#ifdef UNDO
	switch (ADC_ACQ_TIME_UNIT(channel_cfg->acquisition_time)) {
	case ADC_ACQ_TIME_TICKS:
		data->sample_time = (uint16_t)ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
		break;
	case ADC_ACQ_TIME_MICROSECONDS:
		sample_time_us = (uint16_t)ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time);
		break;
	case ADC_ACQ_TIME_NANOSECONDS:
		sample_time_us = (uint16_t)(
				ADC_ACQ_TIME_VALUE(channel_cfg->acquisition_time) * 1000);
		break;
	default:
		data->sample_time = AUXADC_SAMPLE_TIME_170_US;
		break;
	}
	if (sample_time_us) {
		/* choose the nearest sample time configuration */
		data->sample_time = adc_g3xxx_sample_times[0].reg_value;
		for (i = 0; i < ARRAY_SIZE(adc_cc13xx_sample_times); i++) {
			if (adc_g3xxx_sample_times[i].time_us >= sample_time_us) {
				break;
			}
			data->sample_time = adc_g3xxx_sample_times[i].reg_value;
		}
		if (i >= ARRAY_SIZE(adc_g3xxx_sample_times)) {
			LOG_ERR("Acquisition time is not valid");
			return -EINVAL;
		}
	}
#endif
	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Gain is not valid");
		return -EINVAL;
	}
#ifdef UNDO
	if (channel_cfg->reference == ADC_REF_INTERNAL) {
		data->ref_source = AUXADC_REF_FIXED;
	} else if (channel_cfg->reference == ADC_REF_VDD_1) {
		data->ref_source = AUXADC_REF_VDDS_REL;
	} else {
		LOG_ERR("Reference is not valid");
		return -EINVAL;
	}
#endif

	LOG_DBG("Setup %d acq time %d", ch, data->sample_time);
#ifdef UNDO
	AUXADCDisable();
	AUXADCSelectInput(ch);
#endif
	return 0;
}

static int mspm0g3xxx_read(const struct device *dev,
		       const struct adc_sequence *sequence,
		       bool asynchronous,
		       struct k_poll_signal *sig)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	int rv;
	size_t exp_size;

	if (sequence->resolution != 12) {
		LOG_ERR("Only 12 Resolution is supported, but %d got",
			sequence->resolution);
		return -EINVAL;
	}

	exp_size = sizeof(uint16_t);
	if (sequence->options) {
		exp_size *= (1 + sequence->options->extra_samplings);
	}

	if (sequence->buffer_size < exp_size) {
		LOG_ERR("Required buffer size is %u, but %u got",
			exp_size, sequence->buffer_size);
		return -ENOMEM;
	}

	data->buffer = sequence->buffer;

	adc_context_lock(&data->ctx, asynchronous, sig);
	adc_context_start_read(&data->ctx, sequence);
	rv = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, rv);
	return rv;
}

static int adc_mspm0g3xxx_read(const struct device *dev,
			   const struct adc_sequence *sequence)
{
	return mspm0g3xxx_read(dev, sequence, false, NULL);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_mspm0g3xxx_read_async(const struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	return mspm0g3xxx_read(dev, sequence, true, async);
}
#endif

/**
 * AUX_ADC_IRQ handler, called for either of these events:
 * - conversion complete or DMA done (if used);
 * - FIFO underflow or overflow;
 */
static void adc_mspm0g3xxx_isr(const struct device *dev)
{
	struct adc_mspm0g3xxx_data *data = dev->data;
	/* get the statuses of ADC_DONE and ADC_IRQ events in order to clear them both */
 #ifdef UNDO	
 	uint32_t ev_status = (
		HWREG(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGS) &
		(AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_IRQ | AUX_EVCTL_EVTOMCUFLAGS_AUX_ADC_DONE)
	);

	uint32_t fifo_status;
	uint32_t adc_value;

	/* clear the AUXADC-related event flags */
	HWREG(AUX_EVCTL_BASE + AUX_EVCTL_O_EVTOMCUFLAGSCLR) = ev_status;
	/* check the ADC FIFO's status */
	fifo_status = AUXADCGetFifoStatus();
	LOG_DBG("ISR flags 0x%08X fifo 0x%08X", ev_status, fifo_status);
	if ((fifo_status & (AUX_ANAIF_ADCFIFOSTAT_OVERFLOW | AUX_ANAIF_ADCFIFOSTAT_UNDERFLOW))) {
		AUXADCFlushFifo();
	}
	if ((fifo_status & AUX_ANAIF_ADCFIFOSTAT_EMPTY_M)) {
		/* no ADC values available */
		return;
	}
	adc_value = AUXADCPopFifo();
	LOG_DBG("ADC buf %04X val %d", (unsigned int)data->buffer, adc_value);
	*data->buffer = adc_value;
	AUXADCDisable();
#endif
	adc_context_on_sampling_done(&data->ctx, dev);
}

static const struct adc_driver_api mspm0g3xxx_driver_api = {
	.channel_setup = adc_mspm0g3xxx_channel_setup,
	.read = adc_mspm0g3xxx_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_mspm0g3xxx_read_async,
#endif
};

#define MSPM0G3XXX_ADC_INIT(index)						 \
	static void adc_mspm0g3xxx_cfg_func_##index(void);			 \
	static const struct adc_mspm0g3xxx_cfg adc_mspm0g3xxx_cfg_##index = { \
		.base = DT_INST_REG_ADDR(index),				 \
		.irq_cfg_func = adc_mspm0g3xxx_cfg_func_##index,		 \
	};									 \
	static struct adc_mspm0g3xxx_data adc_mspm0g3xxx_data_##index = { \
		ADC_CONTEXT_INIT_TIMER(adc_mspm0g3xxx_data_##index, ctx),	 \
		ADC_CONTEXT_INIT_LOCK(adc_mspm0g3xxx_data_##index, ctx),	 \
		ADC_CONTEXT_INIT_SYNC(adc_mspm0g3xxx_data_##index, ctx),	 \
	};									 \
	DEVICE_DT_INST_DEFINE(index,						 \
		&adc_mspm0g3xxx_init, NULL,					 \
		&adc_mspm0g3xxx_data_##index,				 \
		&adc_mspm0g3xxx_cfg_##index, POST_KERNEL,			 \
		CONFIG_ADC_INIT_PRIORITY,					 \
		&mspm0g3xxx_driver_api);					 \
										 \
	static void adc_mspm0g3xxx_cfg_func_##index(void)			 \
	{									 \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority),	 \
				adc_mspm0g3xxx_isr, DEVICE_DT_INST_GET(index), 0); \
		irq_enable(DT_INST_IRQN(index));				 \
	}

DT_INST_FOREACH_STATUS_OKAY(MSPM0G3XXX_ADC_INIT)
