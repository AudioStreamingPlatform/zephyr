/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_mspm0g3xxx_uart

/* Zephyr includes */
#include <zephyr/kernel.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <soc.h>

/* Driverlib includes */
#include <ti/driverlib/dl_uart.h>

struct uart_mspm0g3xxx_config {
	UART_Regs *regs;
	const struct pinctrl_dev_config *pinctrl;
	uint32_t clock_frequency;
	uint32_t current_speed; /* baud rate */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(const struct device *dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

struct uart_mspm0g3xxx_data {
	/* UART clock structure */
	DL_UART_ClockConfig UART_ClockConfig;
	/* UART config structure */
	DL_UART_Config UART_Config;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t cb; /* Callback function pointer */
	void *cb_data;                    /* Callback function arg */
#endif                                    /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static int uart_mspm0g3xxx_init(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;
	struct uart_mspm0g3xxx_data *data = dev->data;
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

	/* Set UART configs */
	DL_UART_setClockConfig(config->regs, (DL_UART_ClockConfig *)&data->UART_ClockConfig);
	DL_UART_init(config->regs, (DL_UART_Config *)&data->UART_Config);

	DL_UART_configBaudRate(config->regs, config->clock_frequency, config->current_speed);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

	/* Enable UART */
	DL_UART_enable(config->regs);

	return 0;
}

static int uart_mspm0g3xxx_poll_in(const struct device *dev, unsigned char *c)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (DL_UART_receiveDataCheck(config->regs, c)) ? 0 : -1;
}

static void uart_mspm0g3xxx_poll_out(const struct device *dev, unsigned char c)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	DL_UART_transmitDataBlocking(config->regs, c);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN

#define UART_MSPM0_TX_INTERRUPTS (DL_UART_INTERRUPT_TX | DL_UART_INTERRUPT_EOT_DONE)
#define UART_MSPM0_RX_INTERRUPTS (DL_UART_INTERRUPT_RX)

static int uart_mspm0g3xxx_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (int)DL_UART_fillTXFIFO(config->regs, (uint8_t *)tx_data, size);
}

static int uart_mspm0g3xxx_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (int)DL_UART_drainRXFIFO(config->regs, rx_data, size);
}

static void uart_mspm0g3xxx_irq_tx_enable(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	DL_UART_enableInterrupt(config->regs, UART_MSPM0_TX_INTERRUPTS);
}

static void uart_mspm0g3xxx_irq_tx_disable(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	DL_UART_disableInterrupt(config->regs, UART_MSPM0_TX_INTERRUPTS);
}

static int uart_mspm0g3xxx_irq_tx_ready(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (DL_UART_getEnabledInterruptStatus(config->regs, UART_MSPM0_TX_INTERRUPTS)) ? 1 : 0;
}

static void uart_mspm0g3xxx_irq_rx_enable(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	DL_UART_enableInterrupt(config->regs, UART_MSPM0_RX_INTERRUPTS);
}

static void uart_mspm0g3xxx_irq_rx_disable(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	DL_UART_disableInterrupt(config->regs, UART_MSPM0_RX_INTERRUPTS);
}

static int uart_mspm0g3xxx_irq_tx_complete(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (DL_UART_isTXFIFOEmpty(config->regs)) ? 1 : 0;
}

static int uart_mspm0g3xxx_irq_rx_ready(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (DL_UART_getEnabledInterruptStatus(config->regs, UART_MSPM0_RX_INTERRUPTS)) ? 1 : 0;
}

static int uart_mspm0g3xxx_irq_is_pending(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;

	return (DL_UART_getEnabledInterruptStatus(config->regs, UART_MSPM0_RX_INTERRUPTS |
									UART_MSPM0_TX_INTERRUPTS))
		       ? 1
		       : 0;
}

static int uart_mspm0g3xxx_irq_update(const struct device *dev)
{
	ARG_UNUSED(dev);
	return 1;
}

static void uart_mspm0g3xxx_irq_callback_set(const struct device *dev,
					     uart_irq_callback_user_data_t cb, void *cb_data)
{
	struct uart_mspm0g3xxx_data *const dev_data = dev->data;

	/* Set callback function and data */
	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
static void uart_mspm0g3xxx_isr(const struct device *dev)
{
	const struct uart_mspm0g3xxx_config *config = dev->config;
	struct uart_mspm0g3xxx_data *const dev_data = dev->data;

	/* Get the pending interrupt */
	int int_status = DL_UART_getEnabledInterruptStatus(
		config->regs, UART_MSPM0_RX_INTERRUPTS | UART_MSPM0_TX_INTERRUPTS);

	/* Perform callback if defined */
	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}

	/*
	 * Clear interrupts only after cb called, as Zephyr UART clients expect
	 * to check interrupt status during the callback.
	 */
	DL_UART_clearInterruptStatus(config->regs, int_status);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_mspm0g3xxx_driver_api = {
	.poll_in = uart_mspm0g3xxx_poll_in,
	.poll_out = uart_mspm0g3xxx_poll_out,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_mspm0g3xxx_fifo_fill,
	.fifo_read = uart_mspm0g3xxx_fifo_read,
	.irq_tx_enable = uart_mspm0g3xxx_irq_tx_enable,
	.irq_tx_disable = uart_mspm0g3xxx_irq_tx_disable,
	.irq_tx_ready = uart_mspm0g3xxx_irq_tx_ready,
	.irq_rx_enable = uart_mspm0g3xxx_irq_rx_enable,
	.irq_rx_disable = uart_mspm0g3xxx_irq_rx_disable,
	.irq_tx_complete = uart_mspm0g3xxx_irq_tx_complete,
	.irq_rx_ready = uart_mspm0g3xxx_irq_rx_ready,
	.irq_is_pending = uart_mspm0g3xxx_irq_is_pending,
	.irq_update = uart_mspm0g3xxx_irq_update,
	.irq_callback_set = uart_mspm0g3xxx_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define MSP_UART_IRQ_DEFINE(inst)                                                                  \
	static void uart_mspm0g3xxx_##inst##_irq_register(const struct device *dev)                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), uart_mspm0g3xxx_isr,  \
			    DEVICE_DT_INST_GET(inst), 0);                                          \
		irq_enable(DT_INST_IRQN(inst));                                                    \
	}
#else
#define MSP_UART_IRQ_DEFINE(inst)
#endif

#define MSP_UART_INIT_FN(inst)                                                                     \
	PINCTRL_DT_INST_DEFINE(inst);                                                              \
	MSP_UART_IRQ_DEFINE(inst);                                                                 \
                                                                                                   \
	static const struct uart_mspm0g3xxx_config uart_mspm0g3xxx_cfg_##inst = {                  \
		.regs = (UART_Regs *)DT_INST_REG_ADDR(inst),                                       \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(inst),                                   \
		.clock_frequency = DT_PROP(DT_INST_CLOCKS_CTLR(inst), clock_frequency),            \
		.current_speed = DT_INST_PROP(inst, current_speed),                                \
		IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN,                                           \
			   (.irq_config_func = uart_mspm0g3xxx_##inst##_irq_register, ))};         \
                                                                                                   \
	static struct uart_mspm0g3xxx_data uart_mspm0g3xxx_data_##inst = {                         \
		.UART_ClockConfig = {.clockSel = DL_UART_CLOCK_BUSCLK,                             \
				     .divideRatio = DL_UART_CLOCK_DIVIDE_RATIO_1},                 \
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
	DEVICE_DT_INST_DEFINE(inst, &uart_mspm0g3xxx_init, NULL, &uart_mspm0g3xxx_data_##inst,     \
			      &uart_mspm0g3xxx_cfg_##inst, PRE_KERNEL_1,                           \
			      CONFIG_SERIAL_INIT_PRIORITY, &uart_mspm0g3xxx_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSP_UART_INIT_FN)
