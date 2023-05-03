/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_uart

#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <zephyr/drivers/pinctrl.h>

#include <ti/driverlib/m0p/dl_core.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/uart/UARTMSP.h>

struct uart_mspm0_config {
	UART_Regs *uart_reg;
	const struct pinctrl_dev_config *pcfg;
};
struct uart_mspm0_data {
};

/* Defines for UART_0 */
#define UART_0_INST		       UART0
#define GPIO_UART_0_IOMUX_RX	       (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX	       (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE	       (115200)
#define UART_0_IBRD_32_MHZ_115200_BAUD (17)
#define UART_0_FBRD_32_MHZ_115200_BAUD (23)

static const DL_UART_Main_ClockConfig gUART_0ClockConfig = {
	.clockSel = DL_UART_MAIN_CLOCK_BUSCLK, .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1};

static const DL_UART_Main_Config gUART_0Config = {.mode = DL_UART_MAIN_MODE_NORMAL,
						  .direction = DL_UART_MAIN_DIRECTION_TX_RX,
						  .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
						  .parity = DL_UART_MAIN_PARITY_NONE,
						  .wordLength = DL_UART_MAIN_WORD_LENGTH_8_BITS,
						  .stopBits = DL_UART_MAIN_STOP_BITS_ONE};

static int uart_mspm0_init_dev(const struct device *dev)
{
	// Uart init

	DL_UART_Main_setClockConfig(UART_0_INST, (DL_UART_Main_ClockConfig *)&gUART_0ClockConfig);

	DL_UART_Main_init(UART_0_INST, (DL_UART_Main_Config *)&gUART_0Config);
	/*
	 * Configure baud rate by setting oversampling and baud rate divisors.
	 *  Target baud rate: 115200
	 *  Actual baud rate: 115211.52
	 */
	DL_UART_Main_setOversampling(UART_0_INST, DL_UART_OVERSAMPLING_RATE_16X);
	DL_UART_Main_setBaudRateDivisor(UART_0_INST, UART_0_IBRD_32_MHZ_115200_BAUD,
					UART_0_FBRD_32_MHZ_115200_BAUD);

	/* Configure FIFOs */
	DL_UART_Main_enableFIFOs(UART_0_INST);
	DL_UART_Main_setRXFIFOThreshold(UART_0_INST, DL_UART_RX_FIFO_LEVEL_ONE_ENTRY);
	DL_UART_Main_setTXFIFOThreshold(UART_0_INST, DL_UART_TX_FIFO_LEVEL_1_2_EMPTY);

	DL_UART_Main_enableLoopbackMode(UART_0_INST);

	DL_UART_Main_enable(UART_0_INST);

	return 0;
}

static int uart_mspm0_poll_in(const struct device *dev, unsigned char *c)
{
	/* const struct uart_mspm0_config *config = dev->config; */
	*c = DL_UART_receiveDataBlocking(UART_0_INST);

	return 0;
}

static void uart_mspm0_poll_out(const struct device *dev, unsigned char c)
{
	/* const struct uart_mspm0_config *config = dev->config; */

	DL_UART_Main_fillTXFIFO(UART_0_INST, &c, 1);

	/* Wait until all bytes have been transmitted and the TX FIFO is empty */
	while (DL_UART_Main_isBusy(UART_0_INST)) {
		;
	}
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_mspm0_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
	return 0;
}

static int uart_mspm0_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
	return 0;
}

static void uart_mspm0_irq_tx_enable(const struct device *dev)
{
}

static void uart_mspm0_irq_tx_disable(const struct device *dev)
{
}

static int uart_mspm0_irq_tx_ready(const struct device *dev)
{
	return 0;
}

static void uart_mspm0_irq_rx_enable(const struct device *dev)
{
}

static void uart_mspm0_irq_rx_disable(const struct device *dev)
{
}

static int uart_mspm0_irq_tx_complete(const struct device *dev)
{
	return 0;
}

static int uart_mspm0_irq_rx_ready(const struct device *dev)
{
	return 0;
}

static void uart_mspm0_irq_err_enable(const struct device *dev)
{
	/* Not yet used in zephyr */
}

static void uart_mspm0_irq_err_disable(const struct device *dev)
{
	/* Not yet used in zephyr */
}

static int uart_mspm0_irq_is_pending(const struct device *dev)
{
	return 0;
}

static int uart_mspm0_irq_update(const struct device *dev)
{
	return 1;
}

static void uart_mspm0_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
					void *cb_data)
{
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
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_mspm0_driver_api = {
	.poll_in = uart_mspm0_poll_in,
	.poll_out = uart_mspm0_poll_out,
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
	.irq_err_enable = uart_mspm0_irq_err_enable,
	.irq_err_disable = uart_mspm0_irq_err_disable,
	.irq_is_pending = uart_mspm0_irq_is_pending,
	.irq_update = uart_mspm0_irq_update,
	.irq_callback_set = uart_mspm0_irq_callback_set,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#define UART_MSPM0_INIT_FUNC(n)                                                                    \
	static int uart_mspm0_##n##_init(const struct device *dev)                                 \
	{                                                                                          \
		const struct uart_mspm0_config *config = dev->config;                              \
		DL_UART_Main_reset(config->uart_reg);                                              \
		DL_UART_Main_enablePower(config->uart_reg);                                        \
                                                                                                   \
		DL_GPIO_initPeripheralOutputFunction(GPIO_UART_##n##_IOMUX_TX,                     \
						     GPIO_UART_##n##_IOMUX_TX_FUNC);               \
		DL_GPIO_initPeripheralInputFunction(GPIO_UART_##n##_IOMUX_RX,                      \
						    GPIO_UART_##n##_IOMUX_RX_FUNC);                \
                                                                                                   \
		uart_mspm0_init_dev(dev);                                                          \
		return 0;                                                                          \
	}

#define UART_MSPM0_DEVICE_INIT(n)                                                                  \
	DEVICE_DT_INST_DEFINE(n, uart_mspm0_##n##_init, NULL, &uart_mspm0_##n##_data,              \
			      &uart_mspm0_##n##_config, PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY, \
			      (void *)&uart_mspm0_driver_api);

#define UART_MSPM0_INIT(n)                                                                         \
	static const struct uart_mspm0_config uart_mspm0_##n##_config = {                          \
		.uart_reg = ((UART_Regs *)DT_INST_REG_ADDR(n)),                                    \
	};                                                                                         \
	static struct uart_mspm0_data uart_mspm0_##n##_data;                                       \
	UART_MSPM0_INIT_FUNC(n)                                                                    \
	UART_MSPM0_DEVICE_INIT(n)

DT_INST_FOREACH_STATUS_OKAY(UART_MSPM0_INIT)

