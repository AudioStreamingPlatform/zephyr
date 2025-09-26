/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_mspm0_uart

/* Zephyr includes */
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/mspm0_clock_control.h>
#include <zephyr/irq.h>
#include <soc.h>

/* Driverlib includes */
#include <ti/driverlib/dl_uart.h>

struct uart_mspm0_config {
	UART_Regs *regs;
	const struct pinctrl_dev_config *pinctrl;
	uint32_t clock_frequency;
	const struct mspm0_clockSys *clock_subsys;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_mspm0_data {
	/* UART clock structure */
	DL_UART_ClockConfig UART_ClockConfig;
	/* Baud Rate */
	uint32_t current_speed;
	/* UART config structure */
	DL_UART_Config UART_Config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb; /* Callback function pointer */
	void *cb_data;                    /* Callback function arg */
	DL_UART_IIDX pending_interrupt;
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_mspm0_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_mspm0_config *config = dev->config;

	return (DL_UART_receiveDataCheck(config->regs, c)) ? 0 : -1;
}

static void uart_mspm0_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_transmitDataBlocking(config->regs, c);
}

static int uart_mspm0_install_configuration(const struct device *dev)
{
	const struct device *const clk_dev = DEVICE_DT_GET(DT_NODELABEL(clkmux));
	const struct uart_mspm0_config *config = dev->config;
	struct uart_mspm0_data *data = dev->data;
	uint32_t clock_rate;
	int ret;

	/* Set UART configs */
	DL_UART_setClockConfig(config->regs, (DL_UART_ClockConfig *)&data->UART_ClockConfig);
	DL_UART_init(config->regs, (DL_UART_Config *)&data->UART_Config);

	/*
	 * Configure baud rate by setting oversampling and baud rate divisor
	 * from the selected current-speed
	 */
	ret = clock_control_get_rate(clk_dev, (clock_control_subsys_t)config->clock_subsys,
				     &clock_rate);
	if (ret < 0) {
		return ret;
	}

	DL_UART_Main_configBaudRate(config->regs, clock_rate, data->current_speed);

	return 0;
}

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

static uint32_t uart_parity_to_mspm0[5] = {
	DL_UART_MAIN_PARITY_NONE,
	DL_UART_MAIN_PARITY_ODD,
	DL_UART_MAIN_PARITY_EVEN,
	DL_UART_MAIN_PARITY_STICK_ONE,
	DL_UART_MAIN_PARITY_STICK_ZERO,
};

static uint32_t uart_stop_bits_to_mspm0[4] = {
	UINT32_MAX,
	DL_UART_MAIN_STOP_BITS_ONE,
	UINT32_MAX,
	DL_UART_MAIN_STOP_BITS_TWO,
};

static uint32_t uart_data_bits_to_mspm0[4] = {
	DL_UART_MAIN_WORD_LENGTH_5_BITS,
	DL_UART_MAIN_WORD_LENGTH_6_BITS,
	DL_UART_MAIN_WORD_LENGTH_7_BITS,
	DL_UART_MAIN_WORD_LENGTH_8_BITS,
};

static uint32_t uart_flow_control_to_mspm0[2] = {
	DL_UART_MAIN_FLOW_CONTROL_NONE,
	DL_UART_MAIN_FLOW_CONTROL_RTS_CTS,
};

static int uart_mspm0_translate_in(uint32_t value_array[],
				   int value_array_length,
				   uint8_t uart_cfg_value,
				   uint32_t *mspm0_cfg_value)
{
	if (uart_cfg_value >= value_array_length) {
		return -EINVAL;
	}

	if (value_array[uart_cfg_value] == UINT32_MAX) {
		return -ENOSYS;
	}

	*mspm0_cfg_value = value_array[uart_cfg_value];

 	return 0;
}

static int uart_mspm0_transtlate_out(uint32_t value_array[],
				     int value_array_length,
				     uint32_t mspm0_cfg_value,
				     uint8_t *uart_cfg_value)
{
	int idx;

	for (idx = 0; idx < value_array_length; idx++) {
		if (value_array[idx] == mspm0_cfg_value) {
			break;
		}
	}

	if (idx == value_array_length ||
	    value_array[idx] == UINT32_MAX) {
		return -EINVAL;
	}

	*uart_cfg_value = (uint8_t)idx;

	return 0;
}

static int uart_mspm0_configure(const struct device *dev,
				const struct uart_config *cfg)
{
	const struct uart_mspm0_config *config = dev->config;
	struct uart_mspm0_data *data = dev->data;
	uint32_t value;
	int ret;

	DL_UART_Main_disable(config->regs);

	data->current_speed = cfg->baudrate;

	ret = uart_mspm0_translate_in(uart_parity_to_mspm0,
				      ARRAY_SIZE(uart_parity_to_mspm0),
				      cfg->parity,
				      &value);
	if (ret != 0) {
		return ret;
	}

	data->UART_Config.parity = value;

	ret = uart_mspm0_translate_in(uart_stop_bits_to_mspm0,
				      ARRAY_SIZE(uart_stop_bits_to_mspm0),
				      cfg->stop_bits,
				      &value);
	if (ret != 0) {
		return ret;
	}

	data->UART_Config.stopBits = value;

	ret = uart_mspm0_translate_in(uart_data_bits_to_mspm0,
				      ARRAY_SIZE(uart_data_bits_to_mspm0),
				      cfg->data_bits,
				      &value);
	if (ret != 0) {
		return ret;
	}

	data->UART_Config.wordLength = value;

	ret = uart_mspm0_translate_in(uart_flow_control_to_mspm0,
				      ARRAY_SIZE(uart_flow_control_to_mspm0),
				      cfg->flow_ctrl,
				      &value);
	if (ret != 0) {
		return ret;
	}

	data->UART_Config.flowControl = value;

	ret = uart_mspm0_install_configuration(dev);
	if (ret != 0) {
		return ret;
	}

	DL_UART_Main_enable(config->regs);

	return 0;
}

static int uart_mspm0_config_get(const struct device *dev,
				 struct uart_config *cfg)
{
	struct uart_mspm0_data *data = dev->data;
	int ret;

	cfg->baudrate = data->current_speed;

	ret = uart_mspm0_transtlate_out(uart_parity_to_mspm0,
					ARRAY_SIZE(uart_parity_to_mspm0),
					data->UART_Config.parity,
					&cfg->parity);
	if (ret != 0) {
		return ret;
	}

	ret = uart_mspm0_transtlate_out(uart_stop_bits_to_mspm0,
					ARRAY_SIZE(uart_stop_bits_to_mspm0),
					data->UART_Config.stopBits,
					&cfg->stop_bits);
	if (ret != 0) {
		return ret;
	}

	ret = uart_mspm0_transtlate_out(uart_data_bits_to_mspm0,
					ARRAY_SIZE(uart_data_bits_to_mspm0),
					data->UART_Config.wordLength,
					&cfg->data_bits);
	if (ret != 0) {
		return ret;
	}

	ret = uart_mspm0_transtlate_out(uart_flow_control_to_mspm0,
					ARRAY_SIZE(uart_flow_control_to_mspm0),
					data->UART_Config.flowControl,
					&cfg->flow_ctrl);
	if (ret != 0) {
		return ret;
	}

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */


static int uart_mspm0_err_check(const struct device *dev)
{
	struct uart_mspm0_data *data = dev->data;

	switch (data->pending_interrupt) {
	case DL_UART_IIDX_BREAK_ERROR:
		return UART_BREAK;
	case DL_UART_IIDX_FRAMING_ERROR:
		return UART_ERROR_FRAMING;
	default:
		return 0;
	}
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define UART_MSPM0_TX_INTERRUPTS (DL_UART_INTERRUPT_TX | DL_UART_INTERRUPT_EOT_DONE)
#define UART_MSPM0_RX_INTERRUPTS (DL_UART_INTERRUPT_RX)

static int uart_mspm0_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_mspm0_config *config = dev->config;

	return (int)DL_UART_fillTXFIFO(config->regs, (uint8_t *)tx_data, size);
}

static int uart_mspm0_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_mspm0_config *config = dev->config;

	return (int)DL_UART_drainRXFIFO(config->regs, rx_data, size);
}

static void uart_mspm0_irq_tx_enable(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_enableInterrupt(config->regs, UART_MSPM0_TX_INTERRUPTS);
}

static void uart_mspm0_irq_tx_disable(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_disableInterrupt(config->regs, UART_MSPM0_TX_INTERRUPTS);
}

static int uart_mspm0_irq_tx_ready(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;
	struct uart_mspm0_data *data = dev->data;
	return (data->pending_interrupt & (DL_UART_IIDX_TX | DL_UART_IIDX_EOT_DONE))
		&& !DL_UART_isTXFIFOFull(config->regs) ? 1 : 0;
}

static void uart_mspm0_irq_rx_enable(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_enableInterrupt(config->regs, UART_MSPM0_RX_INTERRUPTS);
}

static void uart_mspm0_irq_rx_disable(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_disableInterrupt(config->regs, UART_MSPM0_RX_INTERRUPTS);
}

static int uart_mspm0_irq_tx_complete(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	return (DL_UART_isTXFIFOEmpty(config->regs)) ? 1 : 0;
}

static int uart_mspm0_irq_rx_ready(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;
	struct uart_mspm0_data *data = dev->data;
	return (data->pending_interrupt & DL_UART_IIDX_RX) &&
		!DL_UART_isRXFIFOEmpty(config->regs)? 1 : 0;
}

static int uart_mspm0_irq_is_pending(const struct device *dev)
{
	struct uart_mspm0_data *data = dev->data;
	return data->pending_interrupt != DL_UART_IIDX_NO_INTERRUPT;
}

static int uart_mspm0_irq_update(const struct device *dev)
{
	struct uart_mspm0_data *data = dev->data;
	const struct uart_mspm0_config *config = dev->config;
	data->pending_interrupt = DL_UART_getPendingInterrupt(config->regs);
	return 1;
}

static void uart_mspm0_irq_callback_set(const struct device *dev,
					     uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_mspm0_data *const dev_data = dev->data;

	/* Set callback function and data */
	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

#define UART_MSPM0_ERROR_INTERRUPTS                                                                \
	(DL_UART_INTERRUPT_BREAK_ERROR | DL_UART_INTERRUPT_FRAMING_ERROR)

static void uart_mspm0_irq_error_enable(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_enableInterrupt(config->regs, UART_MSPM0_ERROR_INTERRUPTS);
}

static void uart_mspm0_irq_error_disable(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;

	DL_UART_disableInterrupt(config->regs, UART_MSPM0_ERROR_INTERRUPTS);
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
static void uart_mspm0_isr(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;
	struct uart_mspm0_data *const dev_data = dev->data;

	dev_data->pending_interrupt = DL_UART_IIDX_NO_INTERRUPT;
	/* Perform callback if defined */
	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	} else {
		uint32_t int_status;
		/* error, callback necessary in order to make progress. Clear interrupts
		 * temporarily.
		 */
		int_status = DL_UART_getEnabledInterruptStatus(config->regs,
							       UART_MSPM0_TX_INTERRUPTS | UART_MSPM0_RX_INTERRUPTS);
		DL_UART_clearInterruptStatus(config->regs, int_status);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_mspm0_init(const struct device *dev)
{
	const struct uart_mspm0_config *config = dev->config;
	int ret;

	/* Reset power */
	DL_UART_reset(config->regs);
	DL_UART_enablePower(config->regs);
	delay_cycles(POWER_STARTUP_DELAY);

	/* Init UART pins */
	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	ret = uart_mspm0_install_configuration(dev);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	/* Enable UART */
	DL_UART_enable(config->regs);

	return 0;
}

static const struct uart_driver_api uart_mspm0_driver_api = {
	.poll_in = uart_mspm0_poll_in,
	.poll_out = uart_mspm0_poll_out,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_mspm0_configure,
	.config_get = uart_mspm0_config_get,
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */
	.err_check = uart_mspm0_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_mspm0_fifo_fill,
	.fifo_read = uart_mspm0_fifo_read,
	.irq_tx_enable = uart_mspm0_irq_tx_enable,
	.irq_tx_disable = uart_mspm0_irq_tx_disable,
	.irq_tx_ready = uart_mspm0_irq_tx_ready,
	.irq_rx_enable = uart_mspm0_irq_rx_enable,
	.irq_rx_disable = uart_mspm0_irq_rx_disable,
	.irq_tx_complete = uart_mspm0_irq_tx_complete,
	.irq_rx_ready = uart_mspm0_irq_rx_ready,
	.irq_is_pending = uart_mspm0_irq_is_pending,
	.irq_update = uart_mspm0_irq_update,
	.irq_callback_set = uart_mspm0_irq_callback_set,
	.irq_err_enable = uart_mspm0_irq_error_enable,
	.irq_err_disable = uart_mspm0_irq_error_disable,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define MSP_UART_IRQ_DEFINE(inst)                                                                  \
	static void uart_mspm0_##inst##_irq_register(const struct device *dev)                     \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), uart_mspm0_isr,       \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}
#else
#define MSP_UART_IRQ_DEFINE(inst)
#endif

#define MSP_UART_INIT_FN(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	                                                                                           \
	static const struct mspm0_clockSys mspm0_uart_clockSys##inst =                             \
		MSPM0_CLOCK_SUBSYS_FN(inst);                                                       \
	                                                                                           \
	MSP_UART_IRQ_DEFINE(inst);                                                                 \
                                                                                                   \
	static const struct uart_mspm0_config uart_mspm0_cfg_##inst = {                            \
		.regs = (UART_Regs *)DT_INST_REG_ADDR(inst),                                       \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.clock_frequency = DT_PROP(DT_INST_CLOCKS_CTLR(inst), clock_frequency),            \
		.clock_subsys = &mspm0_uart_clockSys##inst,                                        \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                                           \
			   (.irq_config_func = uart_mspm0_##inst##_irq_register, ))};              \
                                                                                                   \
	static struct uart_mspm0_data uart_mspm0_data_##inst = {                                   \
		.UART_ClockConfig = {.clockSel = (DT_INST_CLOCKS_CELL(inst, bus) &                 \
						  MSPM0_CLOCK_SEL_MASK),                           \
				     .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1},            \
		.current_speed = DT_INST_PROP(inst, current_speed),                                \
		.UART_Config =                                                                     \
			{                                                                          \
				.mode = DL_UART_MODE_NORMAL,                                       \
				.direction = DL_UART_DIRECTION_TX_RX,                              \
				.flowControl = DL_UART_FLOW_CONTROL_NONE,                          \
				.parity = DL_UART_PARITY_NONE,                                     \
				.wordLength = DL_UART_WORD_LENGTH_8_BITS,                          \
				.stopBits = DL_UART_STOP_BITS_ONE,                                 \
			},                                                                         \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (.cb = NULL, ))};                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(inst, &uart_mspm0_init, NULL, &uart_mspm0_data_##inst,               \
			      &uart_mspm0_cfg_##inst, PRE_KERNEL_1,                                \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_mspm0_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSP_UART_INIT_FN)
