/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef TI_MSPM0350X_SOC_PINCTRL_H_
#define TI_MSPM0350X_SOC_PINCTRL_H_

#include <zephyr/types.h>
#include <ti/driverlib/dl_gpio.h>

typedef struct pinctrl_soc_pin {
	uint32_t pin;
	uint32_t iofunc;
	uint32_t iomode;
} pinctrl_soc_pin_t;

/* Convert DT flags to SoC flags */
#define MSPM0350X_PIN_FLAGS(node_id)                                                               \
	(DT_PROP(node_id, bias_pull_up) * DL_GPIO_RESISTOR_PULL_UP |                               \
	 DT_PROP(node_id, bias_pull_down) * DL_GPIO_RESISTOR_PULL_DOWN |                           \
	 DT_PROP(node_id, bias_disable) * DL_GPIO_RESISTOR_PULL_NORMAL |

#define MSPM0350X_DT_PIN(node_id)                                                                  \
	{.pin = DT_PROP_BY_IDX(node_id, pinmux, 0),                                                \
	 .iofunc = DT_PROP_BY_IDX(node_id, pinmux, 1),                                             \
	 .iomode = MSPM0350X_PIN_FLAGS(node_id)},

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
	MSPM0350X_DT_PIN(DT_PROP_BY_IDX(node_id, prop, idx))

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
	{                                                                                          \
		DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)                      \
	}

#endif /* TI_MSPM0350X_SOC_PINCTRL_H_ */
