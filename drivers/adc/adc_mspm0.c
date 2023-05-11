/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_adc

#include <errno.h>

#include <zephyr/drivers/adc.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <soc.h>

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/dl_adc12.h>

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
LOG_MODULE_REGISTER(adc_mspm0);

#define ADC0_NODE DT_NODELABEL(adc0)

#define ADC12_0_ADCMEM_0 DL_ADC12_MEM_IDX_0

static const DL_ADC12_ClockConfig gADC12_0ClockConfig = {
	.clockSel = DL_ADC12_CLOCK_ULPCLK,
	.divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
	.freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};

struct adc_mspm0_data {
	const struct device *dev;
	uint16_t *buffer;
	uint16_t *repeat_buffer;
	uint32_t channels;
	size_t active_channels;
	bool readDone;
};

struct adc_mspm0_cfg {
	ADC12_Regs *adc_regs;
	void (*config_func)(const struct device *dev);
};

static int adc_mspm0_init(const struct device *dev)
{
	const struct adc_mspm0_cfg *cfg = dev->config;

	DL_ADC12_reset(cfg->adc_regs);
	DL_ADC12_enablePower(cfg->adc_regs);

	DL_ADC12_setClockConfig(cfg->adc_regs, (DL_ADC12_ClockConfig *)&gADC12_0ClockConfig);
	DL_ADC12_configConversionMem(
		cfg->adc_regs, ADC12_0_ADCMEM_0, DL_ADC12_INPUT_CHAN_2,
		DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0,
		DL_ADC12_AVERAGING_MODE_DISABLED, DL_ADC12_BURN_OUT_SOURCE_DISABLED,
		DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
	DL_ADC12_setPowerDownMode(cfg->adc_regs, DL_ADC12_POWER_DOWN_MODE_MANUAL);
	DL_ADC12_setSampleTime0(cfg->adc_regs, 500);

	/* Enable ADC12 interrupt */
	DL_ADC12_enableInterrupt(cfg->adc_regs, (DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
	DL_ADC12_enableConversions(cfg->adc_regs);

	cfg->config_func(dev);

	return 0;
}

static int adc_mspm0_channel_setup(const struct device *dev,
				   const struct adc_channel_cfg *channel_cfg)
{
	return 0;
}

static int mspm0_read(const struct device *dev, const struct adc_sequence *sequence,
		      bool asynchronous, struct k_poll_signal *sig)
{
	const struct adc_mspm0_cfg *cfg = dev->config;
	struct adc_mspm0_data *data = dev->data;

	data->readDone = false;

	DL_ADC12_startConversion(cfg->adc_regs);

	while (data->readDone == false) {
		__WFE();
	}

	data->buffer = sequence->buffer;

	uint16_t adc_data = DL_ADC12_getMemResult(cfg->adc_regs, DL_ADC12_MEM_IDX_0);
	*(data->buffer) = adc_data;

	// Prepare for next read
	DL_ADC12_enableConversions(cfg->adc_regs);

	return 0;
}

static int adc_mspm0_read(const struct device *dev, const struct adc_sequence *sequence)
{
	return mspm0_read(dev, sequence, false, NULL);
}

static void adc_mspm0_isr(const struct device *dev, int no)
{
	const struct adc_mspm0_cfg *cfg = dev->config;
	struct adc_mspm0_data *data = dev->data;

	switch (DL_ADC12_getPendingInterrupt(cfg->adc_regs)) {
	case DL_ADC12_IIDX_MEM0_RESULT_LOADED:
		data->readDone = true;
		break;
	default:
		break;
	}
}

static const struct adc_driver_api mspm0_driver_api = {
	.channel_setup = adc_mspm0_channel_setup,
	.read = adc_mspm0_read,
	.ref_internal = 1467,
};

#define MSPM0_ADC_INIT(n)                                                                          \
	static void adc_mspm0_config_##n(const struct device *dev)                                 \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(ADC##n##_NODE), DT_IRQ(ADC##n##_NODE, priority),               \
			    adc_mspm0_isr, DEVICE_DT_GET(ADC##n##_NODE), 0);                       \
		irq_enable(DT_IRQN(ADC##n##_NODE));                                                \
	}                                                                                          \
                                                                                                   \
	static const struct adc_mspm0_cfg adc_mspm0_cfg_##n = {                                    \
		.adc_regs = ((ADC12_Regs *)DT_INST_REG_ADDR(n)),                                   \
		.config_func = &adc_mspm0_config_##n,                                              \
	};                                                                                         \
	static struct adc_mspm0_data adc_mspm0_data_##n = {};                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &adc_mspm0_init, NULL, &adc_mspm0_data_##n, &adc_mspm0_cfg_##n,   \
			      POST_KERNEL, CONFIG_ADC_INIT_PRIORITY, &mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPM0_ADC_INIT)
