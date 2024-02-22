/* SPDX-License-Identifier: Apache-2.0 */

#define DT_DRV_COMPAT ti_mspm0g3xxx_i2c

/* Zephyr includes */
#include <zephyr/kernel.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>

/* Logging includes */
#define LOG_LEVEL CONFIG_I2C_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_mspm0g3xxx);
#include "i2c-priv.h"

/* Driverlib includes */
#include <ti/driverlib/dl_i2c.h>
#include <ti/driverlib/dl_gpio.h>

#define TI_MSPM0G_TARGET_INTERRUPTS                                                                \
	(DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER | DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |         \
	 DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY | DL_I2C_INTERRUPT_TARGET_START |                    \
	 DL_I2C_INTERRUPT_TARGET_STOP)

enum i2c_mspm0g3xxx_target_type {
	TARGET_TYPE_PRIMARY,
	TARGET_TYPE_ALTERNATE,
	TARGET_TYPE_INVALID,
};

enum i2c_mspm0g3xxx_state {
	I2C_mspm0g3xxx_IDLE,
	I2C_mspm0g3xxx_TX_STARTED,
	I2C_mspm0g3xxx_TX_INPROGRESS,
	I2C_mspm0g3xxx_TX_COMPLETE,
	I2C_mspm0g3xxx_RX_STARTED,
	I2C_mspm0g3xxx_RX_INPROGRESS,
	I2C_mspm0g3xxx_RX_COMPLETE,
	I2C_mspm0g3xxx_TARGET_STARTED,
	I2C_mspm0g3xxx_TARGET_TX_INPROGRESS,
	I2C_mspm0g3xxx_TARGET_RX_INPROGRESS,
	I2C_mspm0g3xxx_TARGET_PREEMPTED,

	I2C_mspm0g3xxx_ERROR
};

struct i2c_mspm0g3xxx_config {
	uint32_t base;
	uint32_t clock_frequency;
	bool target_mode_only;
	DL_I2C_ClockConfig gI2CClockConfig;
	const struct pinctrl_dev_config *pinctrl;
	void (*interrupt_init_function)(const struct device *dev);
};

struct i2c_mspm0g3xxx_data {
	const struct i2c_mspm0g3xxx_config *cfg;
	volatile enum i2c_mspm0g3xxx_state state; /* Current state of I2C transmission */
	struct i2c_msg msg;                       /* Cache msg */
	uint16_t addr;                            /* Cache slave address */
	uint32_t count;                           /* Count for progress in I2C transmission */
	uint32_t dev_config;                      /* Configuration last passed */
	uint32_t is_target;
	struct i2c_target_config *target_config_primary;
	struct i2c_target_config *target_config_alternate;
	int target_tx_valid;
	int target_rx_valid;
	struct k_sem i2c_busy_sem;
};

#ifdef CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT

struct i2c_mspm0g3xxx_target_msg {
	struct i2c_mspm0g3xxx_data *data;
	DL_I2C_IIDX i2c_iidx;
	struct i2c_target_config *tconfig;
};

#define CONFIG_I2C_MSPM0G3XXX_TARGET_THREAD_STACK_SIZE 768
#define CONFIG_I2C_MSPM0G3XXX_TARGET_THREAD_PRIORITY   1

/* NOTE: ring buffer size might require tweaking if multiple i2c-target drivers are enabled */
K_MSGQ_DEFINE(target_msgq, sizeof(struct i2c_mspm0g3xxx_target_msg), 5, 1);
static K_KERNEL_STACK_DEFINE(i2c_mspm0g3xxx_target_stack,
			     CONFIG_I2C_MSPM0G3XXX_TARGET_THREAD_STACK_SIZE);
static struct k_thread i2c_mspm0g3xxx_target_thread;

void i2c_mspm0g3xxx_target_thread_work(void)
{
	struct i2c_mspm0g3xxx_target_msg target_msg;
	while (1) {
		k_msgq_get(&target_msgq, &target_msg, K_FOREVER);

		struct i2c_mspm0g3xxx_data *data = target_msg.data;
		const struct i2c_mspm0g3xxx_config *config = data->cfg;
		struct i2c_target_config *tconfig = target_msg.tconfig;

		if (tconfig == NULL) {
			LOG_ERR("target work invoked on invalid target");
			return;
		}

		switch (target_msg.i2c_iidx) {
		case DL_I2C_IIDX_TARGET_START:
			/* We can receive multiple starts without stop (eg. write-read
			 * operations). So only reset state if we've already received stop */
			if (k_sem_take(&data->i2c_busy_sem, K_NO_WAIT) == 0) {
				/* semaphore has successfully been obtained */
				data->state = I2C_mspm0g3xxx_TARGET_STARTED;

				/* Flush TX FIFO to clear out any stale data */
				DL_I2C_flushTargetTXFIFO((I2C_Regs *)config->base);
			}
			break;
		case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
			if (data->state == I2C_mspm0g3xxx_TARGET_STARTED) {
				data->state = I2C_mspm0g3xxx_TARGET_RX_INPROGRESS;
				if (tconfig->callbacks->write_requested != NULL) {
					data->target_rx_valid =
						tconfig->callbacks->write_requested(tconfig);
				}
			}
			/* Store received data in buffer */
			if (tconfig->callbacks->write_received != NULL) {
				uint8_t nextByte;
				while (DL_I2C_isTargetRXFIFOEmpty((I2C_Regs *)config->base) !=
				       true) {
					if (data->target_rx_valid == 0) {
						nextByte = DL_I2C_receiveTargetData(
							(I2C_Regs *)config->base);
						data->target_rx_valid =
							tconfig->callbacks->write_received(
								tconfig, nextByte);
					} else {
						/* Prevent overflow and just ignore data */
						DL_I2C_receiveTargetData((I2C_Regs *)config->base);
					}
				}
			}

			break;
		case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
			data->state = I2C_mspm0g3xxx_TARGET_TX_INPROGRESS;
			/* Fill TX FIFO if there are more bytes to send */
			if (tconfig->callbacks->read_requested != NULL) {
				uint8_t nextByte;
				data->target_tx_valid =
					tconfig->callbacks->read_requested(tconfig, &nextByte);
				if (data->target_tx_valid == 0) {
					DL_I2C_transmitTargetData((I2C_Regs *)config->base,
								  nextByte);
				} else {
					/* In this case, no new data is desired to be filled, thus
					 * 0's are transmitted */
					DL_I2C_transmitTargetData((I2C_Regs *)config->base, 0x00);
				}
			}
			break;
		case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
			if (tconfig->callbacks->read_processed != NULL) {
				/* still using the FIFO, we call read_processed in order to add
				 * additional data rather than from a buffer. If the write-received
				 * function chooses to return 0 (no more data present), then 0's
				 * will be filled in */
				uint8_t nextByte;
				if (data->target_tx_valid == 0) {
					data->target_tx_valid = tconfig->callbacks->read_processed(
						tconfig, &nextByte);
				}

				if (data->target_tx_valid == 0) {
					DL_I2C_transmitTargetData((I2C_Regs *)config->base,
								  nextByte);
				} else {
					/* In this case, no new data is desired to be filled, thus
					 * 0's are transmitted */
					DL_I2C_transmitTargetData((I2C_Regs *)config->base, 0x00);
				}
			}
			break;
		case DL_I2C_IIDX_TARGET_STOP:
			data->state = I2C_mspm0g3xxx_IDLE;
			k_sem_give(&data->i2c_busy_sem);
			if (tconfig->callbacks->stop) {
				tconfig->callbacks->stop(tconfig);
			}
			break;
		default:
			LOG_WRN("Invalid target work!");
			break;
		}
	}
}

static int i2c_mspm0g3xxx_target_thread_init(void)
{
	k_thread_create(&i2c_mspm0g3xxx_target_thread, i2c_mspm0g3xxx_target_stack,
			CONFIG_I2C_MSPM0G3XXX_TARGET_THREAD_STACK_SIZE,
			(k_thread_entry_t)i2c_mspm0g3xxx_target_thread_work, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_I2C_MSPM0G3XXX_TARGET_THREAD_PRIORITY), 0, K_NO_WAIT);
	k_thread_name_set(&i2c_mspm0g3xxx_target_thread, "i2c_mspm0g3xxx");

	return 0;
}
SYS_INIT(i2c_mspm0g3xxx_target_thread_init, POST_KERNEL, CONFIG_I2C_INIT_PRIORITY);

#endif // CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT

static int i2c_mspm0g3xxx_configure(const struct device *dev, uint32_t dev_config)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;
	uint32_t bitrate;

	k_sem_take(&data->i2c_busy_sem, K_FOREVER);

	/* 10-bit addressing not supported */
	if (dev_config & I2C_MSG_ADDR_10_BITS) {
		k_sem_give(&data->i2c_busy_sem);
		return -EINVAL;
	}

	/* Config I2C speed */
	switch (I2C_SPEED_GET(dev_config)) {
	case I2C_SPEED_STANDARD:
		bitrate = 7;
		break;
	case I2C_SPEED_FAST:
		bitrate = 31;
		break;
	default:
		k_sem_give(&data->i2c_busy_sem);
		return -EINVAL;
	}

	/* Set the I2C speed */
	DL_I2C_setTimerPeriod((I2C_Regs *)config->base, bitrate);

	data->dev_config = dev_config;

	k_sem_give(&data->i2c_busy_sem);
	return 0;
}

static int i2c_mspm0g3xxx_get_config(const struct device *dev, uint32_t *dev_config)
{
	struct i2c_mspm0g3xxx_data *data = dev->data;

	*dev_config = data->dev_config;

	return 0;
}

static int i2c_mspm0g3xxx_receive(const struct device *dev, struct i2c_msg msg, uint16_t addr)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	/* Update cached msg and addr */
	data->msg = msg;
	data->addr = addr;

	/* Send a read request to Target */
	data->count = 0;
	data->state = I2C_mspm0g3xxx_RX_STARTED;
	DL_I2C_startControllerTransfer((I2C_Regs *)config->base, data->addr,
				       DL_I2C_CONTROLLER_DIRECTION_RX, data->msg.len);

	/* Wait for all bytes to be received in interrupt */
	while (data->state != I2C_mspm0g3xxx_RX_COMPLETE && (data->state != I2C_mspm0g3xxx_ERROR))
		;

	/* If error, return error */
	if (DL_I2C_getControllerStatus((I2C_Regs *)config->base) & DL_I2C_CONTROLLER_STATUS_ERROR) {
		return -EIO;
	}

	return 0;
}

static int i2c_mspm0g3xxx_transmit(const struct device *dev, struct i2c_msg msg, uint16_t addr)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	/* Sending address without data is not supported */
	if (msg.len == 0) {
		return -EIO;
	}

	/* Update cached msg and addr */
	data->msg = msg;
	data->addr = addr;

	/* Update the state */
	data->state = I2C_mspm0g3xxx_IDLE;

	/*
	 * Fill the FIFO
	 *  The FIFO is 8-bytes deep, and this function will return number
	 *  of bytes written to FIFO
	 */
	data->count =
		DL_I2C_fillControllerTXFIFO((I2C_Regs *)config->base, data->msg.buf, data->msg.len);

	/* Enable TXFIFO trigger interrupt if there are more bytes to send */
	if (data->count < data->msg.len) {
		DL_I2C_enableInterrupt((I2C_Regs *)config->base,
				       DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	} else {
		DL_I2C_disableInterrupt((I2C_Regs *)config->base,
					DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
	}

	/*
	 * Send the packet to the controller.
	 * This function will send Start + Stop automatically
	 */
	data->state = I2C_mspm0g3xxx_TX_STARTED;
	while (!(DL_I2C_getControllerStatus((I2C_Regs *)config->base) &
		 DL_I2C_CONTROLLER_STATUS_IDLE))
		;
	DL_I2C_startControllerTransfer((I2C_Regs *)config->base, data->addr,
				       DL_I2C_CONTROLLER_DIRECTION_TX, data->msg.len);

	/* Wait until the Controller sends all bytes */
	while ((data->state != I2C_mspm0g3xxx_TX_COMPLETE) && (data->state != I2C_mspm0g3xxx_ERROR))
		;

	/* If error, return error */
	if (DL_I2C_getControllerStatus((I2C_Regs *)config->base) & DL_I2C_CONTROLLER_STATUS_ERROR) {
		return -EIO;
	}

	return 0;
}

static int i2c_mspm0g3xxx_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t num_msgs,
				   uint16_t addr)
{
	struct i2c_mspm0g3xxx_data *data = dev->data;

	int ret = 0;

	/* Sending address with no data not supported */
	if (num_msgs == 0) {
		return -EINVAL;
	}

	k_sem_take(&data->i2c_busy_sem, K_FOREVER);

	if (data->is_target) {
		/* currently target is registered. Controller is disabled */
		k_sem_give(&data->i2c_busy_sem);
		return -EBUSY;
	}

	/* Transmit each message */
	for (int i = 0; i < num_msgs; i++) {
		if ((msgs[i].flags & I2C_MSG_RW_MASK) == I2C_MSG_WRITE) {
			ret = i2c_mspm0g3xxx_transmit(dev, msgs[i], addr);
		} else {
			ret = i2c_mspm0g3xxx_receive(dev, msgs[i], addr);
		}
	}

	k_sem_give(&data->i2c_busy_sem);
	return ret;
}

#ifdef CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT

enum i2c_mspm0g3xxx_target_type i2c_mspm0g3xxx_next_target_type(struct i2c_mspm0g3xxx_data *data)
{
	if (data->target_config_primary == NULL) {
		return TARGET_TYPE_PRIMARY;
	} else if (data->target_config_alternate == NULL) {
		return TARGET_TYPE_ALTERNATE;
	}

	return TARGET_TYPE_INVALID;
}

static int i2c_mspm0g3xxx_target_register(const struct device *dev,
					  struct i2c_target_config *target_config)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	if (target_config == NULL) {
		return -EINVAL;
	}

	if (target_config->flags & I2C_TARGET_FLAGS_ADDR_10_BITS) {
		return -ENOTSUP;
	}

	k_sem_take(&data->i2c_busy_sem, K_FOREVER);

	enum i2c_mspm0g3xxx_target_type target_type = i2c_mspm0g3xxx_next_target_type(data);
	if (target_type == TARGET_TYPE_INVALID) {
		LOG_ERR("Both primary and alternate target are already configured");
		k_sem_give(&data->i2c_busy_sem);
		return -ENODEV;
	}

	LOG_DBG("Register %s target on addr=%u",
		target_type == TARGET_TYPE_PRIMARY ? "primary" : "alternate",
		target_config->address);

	/* device is already configured as a target - simply setup the address and callbacks */
	if (data->is_target == true) {
		DL_I2C_disableInterrupt((I2C_Regs *)config->base, TI_MSPM0G_TARGET_INTERRUPTS);

		if (target_type == TARGET_TYPE_PRIMARY) {
			DL_I2C_setTargetOwnAddress((I2C_Regs *)config->base,
						   target_config->address);
			DL_I2C_enableTargetOwnAddress((I2C_Regs *)config->base);
			data->target_config_primary = target_config;
		} else if (target_type == TARGET_TYPE_ALTERNATE) {
			DL_I2C_setTargetOwnAddressAlternate((I2C_Regs *)config->base,
							    target_config->address);
			DL_I2C_enableTargetOwnAddressAlternate((I2C_Regs *)config->base);
			data->target_config_alternate = target_config;
		}

		if (data->state == I2C_mspm0g3xxx_TARGET_PREEMPTED) {
			DL_I2C_clearInterruptStatus((I2C_Regs *)config->base,
						    TI_MSPM0G_TARGET_INTERRUPTS);
		}

		k_sem_give(&data->i2c_busy_sem);

		DL_I2C_enableInterrupt((I2C_Regs *)config->base, TI_MSPM0G_TARGET_INTERRUPTS);
		return 0;
	}

	data->target_config_primary = target_config;
	data->dev_config &= ~I2C_MODE_CONTROLLER;
	data->is_target = true;
	data->state = I2C_mspm0g3xxx_IDLE;

	DL_I2C_setTargetOwnAddress((I2C_Regs *)config->base, target_config->address);
	DL_I2C_enableTargetOwnAddress((I2C_Regs *)config->base);

	if (!config->target_mode_only) {
		/* Disable the controller and configure the device to run as a target */
		DL_I2C_disableController((I2C_Regs *)config->base);

		/* reconfigure the interrupt to use a slave isr? */
		DL_I2C_disableInterrupt((I2C_Regs *)config->base,
					DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
						DL_I2C_INTERRUPT_CONTROLLER_NACK |
						DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
						DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
						DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

		DL_I2C_setTargetTXFIFOThreshold((I2C_Regs *)config->base,
						DL_I2C_TX_FIFO_LEVEL_BYTES_1);
		DL_I2C_setTargetRXFIFOThreshold((I2C_Regs *)config->base,
						DL_I2C_RX_FIFO_LEVEL_BYTES_1);
		DL_I2C_enableTargetTXTriggerInTXMode((I2C_Regs *)config->base);
		DL_I2C_enableTargetTXEmptyOnTXRequest((I2C_Regs *)config->base);
		DL_I2C_enableTargetClockStretching((I2C_Regs *)config->base);

		DL_I2C_clearInterruptStatus((I2C_Regs *)config->base,
					    (DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
					     DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY));

		DL_I2C_enableInterrupt((I2C_Regs *)config->base, TI_MSPM0G_TARGET_INTERRUPTS);

		DL_I2C_enableTarget((I2C_Regs *)config->base);
	}

	k_sem_give(&data->i2c_busy_sem);
	return 0;
}

static int i2c_mspm0g3xxx_target_unregister(const struct device *dev,
					    struct i2c_target_config *target_config)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	k_sem_take(&data->i2c_busy_sem, K_FOREVER);

	if (data->is_target == false) {
		/* not currently configured as target. Nothing to do. */
		k_sem_give(&data->i2c_busy_sem);
		return 0;
	}

	if (data->target_config_primary != NULL &&
	    data->target_config_primary->address == target_config->address) {
		LOG_DBG("Unregistering primary on addr=%u", target_config->address);
		data->target_config_primary = NULL;
		DL_I2C_disableTargetOwnAddress((I2C_Regs *)config->base);
	} else if (data->target_config_alternate != NULL &&
		   data->target_config_alternate->address == target_config->address) {
		LOG_DBG("Unregistering alternate on addr=%u", target_config->address);
		data->target_config_alternate = NULL;
		DL_I2C_disableTargetOwnAddressAlternate((I2C_Regs *)config->base);
	} else {
		LOG_ERR("Failed to unregister device as none was found at addr=%u",
			target_config->address);
		k_sem_give(&data->i2c_busy_sem);
		return -ENODEV;
	}

	/* Check if no targets are registered - we'll then re-enable controller mode */
	if (data->target_config_primary != NULL || data->target_config_alternate != NULL) {
		k_sem_give(&data->i2c_busy_sem);
		return 0;
	}

	LOG_DBG("Enabling controller mode as no targets are registered");

	DL_I2C_disableTarget((I2C_Regs *)config->base);

	/* reconfigure the interrupt to use a slave isr? */
	DL_I2C_disableInterrupt(
		(I2C_Regs *)config->base,
		DL_I2C_INTERRUPT_TARGET_RXFIFO_TRIGGER | DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
			DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY | DL_I2C_INTERRUPT_TARGET_START |
			DL_I2C_INTERRUPT_TARGET_STOP);

	if (!config->target_mode_only) {
		DL_I2C_enableInterrupt((I2C_Regs *)config->base,
				       DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
					       DL_I2C_INTERRUPT_CONTROLLER_NACK |
					       DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
					       DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
					       DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

		DL_I2C_enableController((I2C_Regs *)config->base);

		data->dev_config |= I2C_MODE_CONTROLLER;
	}

	data->is_target = false;

	k_sem_give(&data->i2c_busy_sem);
	return 0;
}

struct i2c_target_config *i2c_mspm0g3xxx_config_from_addr(const struct i2c_mspm0g3xxx_data *data,
							  uint32_t addr)
{
	if (data->target_config_primary->address == addr) {
		return data->target_config_primary;
	} else if (data->target_config_alternate->address == addr) {
		return data->target_config_alternate;
	}

	return NULL;
}

#endif // CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT

static void i2c_mspm0g3xxx_isr(const struct device *dev)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;

	DL_I2C_IIDX pending_int = DL_I2C_getPendingInterrupt((I2C_Regs *)config->base);
	switch (pending_int) {
	/* controller interrupts */
	case DL_I2C_IIDX_CONTROLLER_RX_DONE:
		data->state = I2C_mspm0g3xxx_RX_COMPLETE;
		break;
	case DL_I2C_IIDX_CONTROLLER_TX_DONE:
		DL_I2C_disableInterrupt((I2C_Regs *)config->base,
					DL_I2C_INTERRUPT_CONTROLLER_TXFIFO_TRIGGER);
		data->state = I2C_mspm0g3xxx_TX_COMPLETE;
		break;
	case DL_I2C_IIDX_CONTROLLER_RXFIFO_TRIGGER:
		if (data->state != I2C_mspm0g3xxx_RX_COMPLETE) {
			/* Fix for RX_DONE happening before the last RXFIFO_TRIGGER */
			data->state = I2C_mspm0g3xxx_RX_INPROGRESS;
		}
		/* Receive all bytes from target */
		while (DL_I2C_isControllerRXFIFOEmpty((I2C_Regs *)config->base) != true) {
			if (data->count < data->msg.len) {
				data->msg.buf[data->count++] =
					DL_I2C_receiveControllerData((I2C_Regs *)config->base);
			} else {
				/* Ignore and remove from FIFO if the buffer is full */
				DL_I2C_receiveControllerData((I2C_Regs *)config->base);
			}
		}
		break;
	case DL_I2C_IIDX_CONTROLLER_TXFIFO_TRIGGER:
		data->state = I2C_mspm0g3xxx_TX_INPROGRESS;
		/* Fill TX FIFO with next bytes to send */
		if (data->count < data->msg.len) {
			data->count += DL_I2C_fillControllerTXFIFO((I2C_Regs *)config->base,
								   &data->msg.buf[data->count],
								   data->msg.len - data->count);
		}
		break;
	case DL_I2C_IIDX_CONTROLLER_ARBITRATION_LOST:
	case DL_I2C_IIDX_CONTROLLER_NACK:
		if ((data->state == I2C_mspm0g3xxx_RX_STARTED) ||
		    (data->state == I2C_mspm0g3xxx_TX_STARTED)) {
			/* NACK interrupt if I2C Target is disconnected */
			data->state = I2C_mspm0g3xxx_ERROR;
		}

	/* Not implemented */
	case DL_I2C_IIDX_CONTROLLER_RXFIFO_FULL:
	case DL_I2C_IIDX_CONTROLLER_TXFIFO_EMPTY:
	case DL_I2C_IIDX_CONTROLLER_START:
	case DL_I2C_IIDX_CONTROLLER_STOP:
	case DL_I2C_IIDX_CONTROLLER_EVENT1_DMA_DONE:
	case DL_I2C_IIDX_CONTROLLER_EVENT2_DMA_DONE:
		break;
	/* target interrupts */
	case DL_I2C_IIDX_TARGET_START:
	case DL_I2C_IIDX_TARGET_RXFIFO_TRIGGER:
	case DL_I2C_IIDX_TARGET_TXFIFO_TRIGGER:
	case DL_I2C_IIDX_TARGET_TXFIFO_EMPTY:
	case DL_I2C_IIDX_TARGET_STOP: {
#ifdef CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT
		uint32_t addr_match = DL_I2C_getTargetAddressMatch((I2C_Regs *)config->base);
		struct i2c_target_config *tconfig =
			i2c_mspm0g3xxx_config_from_addr(data, addr_match);
		struct i2c_mspm0g3xxx_target_msg target_msg = {
			.data = data, .i2c_iidx = pending_int, .tconfig = tconfig};
		if (k_msgq_put(&target_msgq, &target_msg, K_NO_WAIT) != 0) {
			LOG_ERR("Queue full - could not process target request!");
		}
#endif // CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT
	} break;
	/* Not implemented */
	case DL_I2C_IIDX_TARGET_RX_DONE:
	case DL_I2C_IIDX_TARGET_RXFIFO_FULL:
	case DL_I2C_IIDX_TARGET_GENERAL_CALL:
	case DL_I2C_IIDX_TARGET_EVENT1_DMA_DONE:
	case DL_I2C_IIDX_TARGET_EVENT2_DMA_DONE:
	default:
		break;
	}
}

static int i2c_mspm0g3xxx_init(const struct device *dev)
{
	const struct i2c_mspm0g3xxx_config *config = dev->config;
	struct i2c_mspm0g3xxx_data *data = dev->data;
	int ret;

	k_sem_init(&data->i2c_busy_sem, 0, 1);

	/* Init power */
	DL_I2C_reset((I2C_Regs *)config->base);
	DL_I2C_enablePower((I2C_Regs *)config->base);
	delay_cycles(POWER_STARTUP_DELAY);

	/* Init GPIO */
	ret = pinctrl_apply_state(config->pinctrl, PINCTRL_STATE_DEFAULT);
	if (ret < 0) {
		return ret;
	}

	/* Config clocks and analog filter */
	DL_I2C_setClockConfig((I2C_Regs *)config->base,
			      (DL_I2C_ClockConfig *)&config->gI2CClockConfig);
	DL_I2C_disableAnalogGlitchFilter((I2C_Regs *)config->base);

	/* Set frequency */
	uint32_t speed_config = i2c_map_dt_bitrate(config->clock_frequency);

	if (config->target_mode_only) {
		data->dev_config &= ~I2C_MODE_CONTROLLER;
		data->is_target = true;

		DL_I2C_setTargetTXFIFOThreshold((I2C_Regs *)config->base,
						DL_I2C_TX_FIFO_LEVEL_BYTES_1);
		DL_I2C_setTargetRXFIFOThreshold((I2C_Regs *)config->base,
						DL_I2C_RX_FIFO_LEVEL_BYTES_1);
		DL_I2C_enableTargetTXTriggerInTXMode((I2C_Regs *)config->base);
		DL_I2C_enableTargetTXEmptyOnTXRequest((I2C_Regs *)config->base);
		DL_I2C_enableTargetClockStretching((I2C_Regs *)config->base);

		DL_I2C_clearInterruptStatus((I2C_Regs *)config->base,
					    (DL_I2C_INTERRUPT_TARGET_TXFIFO_TRIGGER |
					     DL_I2C_INTERRUPT_TARGET_TXFIFO_EMPTY));

		DL_I2C_enableInterrupt((I2C_Regs *)config->base, TI_MSPM0G_TARGET_INTERRUPTS);

		DL_I2C_enableTarget((I2C_Regs *)config->base);

		k_sem_give(&data->i2c_busy_sem);

		i2c_mspm0g3xxx_configure(dev, speed_config);
	} else {
		/* Configure Controller Mode */
		DL_I2C_resetControllerTransfer((I2C_Regs *)config->base);

		/* Config other settings */
		DL_I2C_setControllerTXFIFOThreshold((I2C_Regs *)config->base,
						    DL_I2C_TX_FIFO_LEVEL_BYTES_1);
		DL_I2C_setControllerRXFIFOThreshold((I2C_Regs *)config->base,
						    DL_I2C_RX_FIFO_LEVEL_BYTES_1);
		DL_I2C_enableControllerClockStretching((I2C_Regs *)config->base);

		k_sem_give(&data->i2c_busy_sem);
		i2c_mspm0g3xxx_configure(dev, speed_config | I2C_MODE_CONTROLLER);

		/* Configure Interrupts */
		DL_I2C_enableInterrupt((I2C_Regs *)config->base,
				       DL_I2C_INTERRUPT_CONTROLLER_ARBITRATION_LOST |
					       DL_I2C_INTERRUPT_CONTROLLER_NACK |
					       DL_I2C_INTERRUPT_CONTROLLER_RXFIFO_TRIGGER |
					       DL_I2C_INTERRUPT_CONTROLLER_RX_DONE |
					       DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);

		/* Enable module */
		DL_I2C_enableController((I2C_Regs *)config->base);
	}

	/* Enable interrupts */
	config->interrupt_init_function(dev);

	return 0;
}

static const struct i2c_driver_api i2c_mspm0g3xxx_driver_api = {
	.configure = i2c_mspm0g3xxx_configure,
	.get_config = i2c_mspm0g3xxx_get_config,
	.transfer = i2c_mspm0g3xxx_transfer,
#ifdef CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT
	.target_register = i2c_mspm0g3xxx_target_register,
	.target_unregister = i2c_mspm0g3xxx_target_unregister,
#endif // CONFIG_I2C_MSPM0G3XXX_TARGET_SUPPORT
};

/* Macros to assist with the device-specific initialization */
#define INTERRUPT_INIT_FUNCTION_DECLARATION(index)                                                 \
	static void i2c_mspm0g3xxx_interrupt_init_##index(const struct device *dev)

#define INTERRUPT_INIT_FUNCTION(index)                                                             \
	static void i2c_mspm0g3xxx_interrupt_init_##index(const struct device *dev)                \
	{                                                                                          \
		IRQ_CONNECT(DT_INST_IRQN(index), DT_INST_IRQ(index, priority), i2c_mspm0g3xxx_isr, \
			    DEVICE_DT_INST_GET(index), 0);                                         \
		irq_enable(DT_INST_IRQN(index));                                                   \
	}

#define MSP_I2C_INIT_FN(index)                                                                     \
                                                                                                   \
	PINCTRL_DT_INST_DEFINE(index);                                                             \
                                                                                                   \
	INTERRUPT_INIT_FUNCTION_DECLARATION(index);                                                \
                                                                                                   \
	static const struct i2c_mspm0g3xxx_config i2c_mspm0g3xxx_cfg_##index = {                   \
		.base = DT_INST_REG_ADDR(index),                                                   \
		.clock_frequency = DT_INST_PROP(index, clock_frequency),                           \
		.target_mode_only = DT_INST_PROP_OR(index, target_mode_only, false),               \
		.pinctrl = PINCTRL_DT_INST_DEV_CONFIG_GET(index),                                  \
		.interrupt_init_function = i2c_mspm0g3xxx_interrupt_init_##index,                  \
		.gI2CClockConfig = {.clockSel = DL_I2C_CLOCK_BUSCLK,                               \
				    .divideRatio = DL_I2C_CLOCK_DIVIDE_1}};                        \
                                                                                                   \
	static struct i2c_mspm0g3xxx_data i2c_mspm0g3xxx_data_##index = {                          \
		.cfg = &i2c_mspm0g3xxx_cfg_##index,                                                \
	};                                                                                         \
                                                                                                   \
	I2C_DEVICE_DT_INST_DEFINE(index, i2c_mspm0g3xxx_init, NULL, &i2c_mspm0g3xxx_data_##index,  \
				  &i2c_mspm0g3xxx_cfg_##index, POST_KERNEL,                        \
				  CONFIG_I2C_INIT_PRIORITY, &i2c_mspm0g3xxx_driver_api);           \
                                                                                                   \
	INTERRUPT_INIT_FUNCTION(index)

DT_INST_FOREACH_STATUS_OKAY(MSP_I2C_INIT_FN)
