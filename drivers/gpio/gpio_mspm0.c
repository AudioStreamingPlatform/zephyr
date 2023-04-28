/*
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ti_mspm0_gpio
#include <errno.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/sys_io.h>

/* Driverlib includes */
#include <ti/driverlib/dl_gpio.h>

// SoC iomux helper
#include <iomux.h>

#include <zephyr/irq.h>

#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/sys/util_macro.h>

struct gpio_mspm0_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	/* GPIO port number */
	uint8_t port_num;
	// Gpio register
	GPIO_Regs *gpio_regs;
};

struct gpio_mspm0_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	/* list of registered callbacks */
	sys_slist_t callbacks;
};

static inline int gpio_mspm0_config(const struct device *port, gpio_pin_t pin, gpio_flags_t flags)
{
	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_INPUT | GPIO_OUTPUT)) == 0) {
		return -ENOTSUP;
	}

	if ((flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) != 0) {
		return -ENOTSUP;
	}

	const struct gpio_mspm0_config *gpio_config = port->config;
	GPIO_Regs *gpios = gpio_config->gpio_regs;

	uint32_t mask = BIT(pin);
	uint32_t config_mask = gpio_config->common.port_pin_mask;
	IOMUX_PINCM iomux = mux_from_gpio(gpios, pin);

	DL_GPIO_clearPins(gpios, mask);

	if ((flags & GPIO_OUTPUT)) {
		DL_GPIO_initDigitalOutput(iomux);
		DL_GPIO_enableOutput(gpios, mask & config_mask);
	}

	return 0;
}

static int gpio_mspm0_port_get_raw(const struct device *port, uint32_t *value)
{
	const struct gpio_mspm0_config *gpio_config = port->config;
	GPIO_Regs *gpios = gpio_config->gpio_regs;

	*value = DL_GPIO_readPins(gpios, 0xFFFF);
	return 0;
}

static int gpio_mspm0_port_set_masked_raw(const struct device *port, uint32_t mask, uint32_t value)
{
	return 0;
}

static int gpio_mspm0_port_set_bits_raw(const struct device *port, uint32_t mask)
{
	const struct gpio_mspm0_config *gpio_config = port->config;
	GPIO_Regs *gpios = gpio_config->gpio_regs;

	DL_GPIO_setPins(gpios, mask);
	return 0;
}

static int gpio_mspm0_port_clear_bits_raw(const struct device *port, uint32_t mask)
{
	const struct gpio_mspm0_config *gpio_config = port->config;
	GPIO_Regs *gpios = gpio_config->gpio_regs;

	DL_GPIO_clearPins(gpios, mask);
	return 0;
}

static int gpio_mspm0_port_toggle_bits(const struct device *port, uint32_t mask)
{
	const struct gpio_mspm0_config *gpio_config = port->config;
	GPIO_Regs *gpios = gpio_config->gpio_regs;

	DL_GPIO_togglePins(gpios, mask);
	return 0;
}

static int gpio_mspm0_pin_interrupt_configure(const struct device *port, gpio_pin_t pin,
					      enum gpio_int_mode mode, enum gpio_int_trig trig)
{
	return 0;
}

static int gpio_mspm0_manage_callback(const struct device *dev, struct gpio_callback *callback,
				      bool set)
{
	return 0;
}

static void gpio_mspm0_port_isr(const struct device *dev)
{
}

static const struct gpio_driver_api api_funcs = {
	.pin_configure = gpio_mspm0_config,
	.port_get_raw = gpio_mspm0_port_get_raw,
	.port_set_masked_raw = gpio_mspm0_port_set_masked_raw,
	.port_set_bits_raw = gpio_mspm0_port_set_bits_raw,
	.port_clear_bits_raw = gpio_mspm0_port_clear_bits_raw,
	.port_toggle_bits = gpio_mspm0_port_toggle_bits,
	.pin_interrupt_configure = gpio_mspm0_pin_interrupt_configure,
	.manage_callback = gpio_mspm0_manage_callback,
};

#define GPIO_MSPM0_INIT_FUNC(n)                                                                    \
	static int gpio_mspm0_##n##_init(const struct device *dev)                                 \
	{                                                                                          \
		ARG_UNUSED(dev);                                                                   \
		DL_GPIO_reset(GPIOA);                                                              \
		DL_GPIO_reset(GPIOB);                                                              \
		DL_GPIO_enablePower(GPIOA);                                                        \
		DL_GPIO_enablePower(GPIOB);                                                        \
		return 0;                                                                          \
	}

#define GPIO_MSPM0_DEVICE_INIT(n)                                                                  \
	DEVICE_DT_INST_DEFINE(n, &gpio_mspm0_##n##_init, NULL, &gpio_mspm0_##n##_data,             \
			      &gpio_mspm0_##n##_config, POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY,    \
			      &api_funcs)

#define GPIO_MSPM0_INIT(n)                                                                         \
	static const struct gpio_mspm0_config gpio_mspm0_##n##_config = {                          \
		.common =                                                                          \
			{                                                                          \
				.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),               \
			},                                                                         \
		.port_num = n,                                                                     \
		.gpio_regs = ((GPIO_Regs *)DT_INST_REG_ADDR(n))};                                  \
                                                                                                   \
	static struct gpio_mspm0_data gpio_mspm0_##n##_data;                                       \
                                                                                                   \
	GPIO_MSPM0_INIT_FUNC(n)                                                                    \
                                                                                                   \
	GPIO_MSPM0_DEVICE_INIT(n);

DT_INST_FOREACH_STATUS_OKAY(GPIO_MSPM0_INIT)

