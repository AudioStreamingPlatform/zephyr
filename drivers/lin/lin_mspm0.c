#define DT_DRV_COMPAT ti_mspm0_lin

#include <zephyr/drivers/lin.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <ti/driverlib/dl_uart_extend.h>

LOG_MODULE_REGISTER(lin_mspm0, CONFIG_LIN_LOG_LEVEL);

enum lin_mspm0_rx_state {
	LIN_MSPM0_RX_STATE_IDLE,
	LIN_MSPM0_RX_STATE_DATA,
	LIN_MSPM0_RX_STATE_CHECKSUM,
	LIN_MSPM0_RX_STATE_TIMEOUT,
};

struct lin_mspm0_config {
	UART_Regs *regs;
	const struct device *uart_parent;
};

struct lin_mspm0_data {
	const struct device *self;
	struct uart_config uart_cfg;
	enum lin_mode mode;
	enum lin_mspm0_rx_state rx_state;
	struct lin_callbacks callbacks;
	k_timeout_t break_time;
	struct k_timer timeout_timer;

	struct lin_frame rx_frame;
	uint8_t rx_count;
	struct k_sem transfer_busy;
};

void lin_mspm0_send_break(const struct device *dev)
{
	const struct lin_mspm0_config *cfg = dev->config;
	struct lin_mspm0_data *data = dev->data;

	DL_UART_Extend_enableLINSendBreak(cfg->regs);
	k_sleep(data->break_time);
	DL_UART_Extend_disableLINSendBreak(cfg->regs);
}

void lin_mspm0_send_header(const struct device *dev, uint8_t frame_id)
{
	const struct lin_mspm0_config *cfg = dev->config;

	uart_poll_out(cfg->uart_parent, LIN_SYNC_FIELD);
	uart_poll_out(cfg->uart_parent, lin_pid_from_index(frame_id));
}

/* API functions */
static int lin_mspm0_set_mode(const struct device *dev, enum lin_mode mode)
{
	if (mode == LIN_MODE_RESPONDER) {
		return -ENOTSUP;
	}

	struct lin_mspm0_data *data = dev->data;

	k_sem_take(&data->transfer_busy, K_FOREVER);
	data->mode = mode;
	k_sem_give(&data->transfer_busy);

	return 0;
}

static int lin_mspm0_set_bitrate(const struct device *dev, uint32_t bitrate)
{
	struct lin_mspm0_data *data = dev->data;
	const struct lin_mspm0_config *cfg = dev->config;

	k_sem_take(&data->transfer_busy, K_FOREVER);
	data->uart_cfg.baudrate = bitrate;
	int res = uart_configure(cfg->uart_parent, &data->uart_cfg);
	if (res != 0) {
		goto out;
	}

	data->break_time = lin_break_time(bitrate);
out:
	k_sem_give(&data->transfer_busy);
	return res;
}

static int lin_mspm0_set_header_callback(const struct device *dev, lin_header_callback_t callback,
					 void *user_data)
{
	struct lin_mspm0_data *data = dev->data;

	k_sem_take(&data->transfer_busy, K_FOREVER);
	data->callbacks.header = callback;
	data->callbacks.header_data = user_data;
	k_sem_give(&data->transfer_busy);

	return 0;
}

static int lin_mspm0_set_tx_callback(const struct device *dev, lin_tx_callback_t callback,
				     void *user_data)
{
	struct lin_mspm0_data *data = dev->data;

	k_sem_take(&data->transfer_busy, K_FOREVER);
	data->callbacks.tx = callback;
	data->callbacks.tx_data = user_data;
	k_sem_give(&data->transfer_busy);

	return 0;
}

static int lin_mspm0_set_rx_callback(const struct device *dev, lin_rx_callback_t callback,
				     void *user_data)
{
	struct lin_mspm0_data *data = dev->data;

	k_sem_take(&data->transfer_busy, K_FOREVER);
	data->callbacks.rx = callback;
	data->callbacks.rx_data = user_data;
	k_sem_give(&data->transfer_busy);

	return 0;
}

static int lin_mspm0_send(const struct device *dev, const struct lin_frame *frame)
{
	if (frame->type == LIN_CHECKSUM_AUTO || frame->id >= LIN_NUM_ID ||
	    frame->len > LIN_MAX_DLEN || frame->len == 0) {
		return -EINVAL;
	}

	struct lin_mspm0_data *data = dev->data;
	const struct lin_mspm0_config *cfg = dev->config;

	const uint8_t checksum = lin_calculate_checksum(frame, frame->type);

	k_sem_take(&data->transfer_busy, K_FOREVER);

	lin_mspm0_send_break(dev);
	lin_mspm0_send_header(dev, frame->id);

	for (uint8_t i = 0; i < frame->len; i++) {
		uart_poll_out(cfg->uart_parent, frame->data[i]);
	}

	uart_poll_out(cfg->uart_parent, checksum);

	k_sem_give(&data->transfer_busy);
	return 0;
}

static int lin_mspm0_receive(const struct device *dev, uint8_t id, enum lin_checksum type,
			     uint8_t len)
{
	struct lin_mspm0_data *data = dev->data;
	const struct lin_mspm0_config *cfg = dev->config;

	k_sem_take(&data->transfer_busy, K_FOREVER);

	data->rx_state = LIN_MSPM0_RX_STATE_DATA;
	data->rx_count = 0;
	data->rx_frame.len = len;
	data->rx_frame.id = id;
	data->rx_frame.type = type;

	uart_irq_rx_enable(cfg->uart_parent);
	k_timer_start(&data->timeout_timer, K_MSEC(50), K_NO_WAIT);
	lin_mspm0_send_break(dev);
	lin_mspm0_send_header(dev, id);

	return 0;
}

void lin_mspm0_handle_uart_rx(const struct device *dev)
{
	struct lin_mspm0_data *data = dev->data;
	const struct lin_mspm0_config *cfg = dev->config;

	uint8_t rx_data = 0;
	uart_poll_in(cfg->uart_parent, &rx_data);

	switch (data->rx_state) {
	case LIN_MSPM0_RX_STATE_DATA: {
		data->rx_frame.data[data->rx_count++] = rx_data;

		if (data->rx_count >= data->rx_frame.len) {
			data->rx_state = LIN_MSPM0_RX_STATE_CHECKSUM;
		}
	} break;
	case LIN_MSPM0_RX_STATE_CHECKSUM: {
		k_timer_stop(&data->timeout_timer);
		uart_rx_disable(cfg->uart_parent);
		const int err = lin_verify_checksum(&data->rx_frame, rx_data) ? 0 : -EINVAL;
		if (data->callbacks.rx) {
			data->callbacks.rx(dev, err, &data->rx_frame, data->callbacks.rx_data);
		}

		data->rx_state = LIN_MSPM0_RX_STATE_IDLE;

		k_sem_give(&data->transfer_busy);
	} break;
	default:
		LOG_WRN("rx irq while rx_state: %u, rx_data: 0x%02x", data->rx_state, rx_data);
	}
}

void lin_mspm0_uart_irq_callback(const struct device *uart_dev, void *user_data)
{
	const struct device *dev = (const struct device *)user_data;
	const struct lin_mspm0_config *cfg = dev->config;

	if (uart_irq_rx_ready(cfg->uart_parent)) {
		lin_mspm0_handle_uart_rx(dev);
	}
}

static void lin_mspm0_timeout(struct k_timer *timer)
{
	struct lin_mspm0_data *data = CONTAINER_OF(timer, struct lin_mspm0_data, timeout_timer);
	const struct lin_mspm0_config *cfg = data->self->config;

	if (data->rx_state == LIN_MSPM0_RX_STATE_IDLE) {
		return;
	}

	uart_rx_disable(cfg->uart_parent);
	LOG_WRN("timeout, rx_state: %u", data->rx_state);
	data->rx_state = LIN_MSPM0_RX_STATE_TIMEOUT;

	if (data->callbacks.rx) {
		data->callbacks.rx(data->self, -ETIMEDOUT, &data->rx_frame,
				   data->callbacks.rx_data);
	}

	k_sem_give(&data->transfer_busy);
}

static int lin_mspm0_init(const struct device *dev)
{
	const struct lin_mspm0_config *cfg = dev->config;
	struct lin_mspm0_data *data = dev->data;

	if (!device_is_ready(cfg->uart_parent)) {
		LOG_ERR("parent %s is not ready", cfg->uart_parent->name);
		return -ENODEV;
	}

	k_sem_init(&data->transfer_busy, 1, 1);
	k_timer_init(&data->timeout_timer, lin_mspm0_timeout, NULL);

	int err = uart_config_get(cfg->uart_parent, &data->uart_cfg);
	if (err != 0) {
		LOG_ERR("failed to get uart config, dev: %s", cfg->uart_parent->name);
		return err;
	}

	data->break_time = lin_break_time(data->uart_cfg.baudrate);

	DL_UART_Extend_setLINCounterValue(cfg->regs, 0);

	uart_irq_callback_user_data_set(cfg->uart_parent, lin_mspm0_uart_irq_callback, (void *)dev);

	k_sem_give(&data->transfer_busy);

	return 0;
}

static struct lin_driver_api lin_mspm0_api = {
	.set_mode = lin_mspm0_set_mode,
	.set_bitrate = lin_mspm0_set_bitrate,
	.set_header_callback = lin_mspm0_set_header_callback,
	.set_tx_callback = lin_mspm0_set_tx_callback,
	.set_rx_callback = lin_mspm0_set_rx_callback,
	.send = lin_mspm0_send,
	.receive = lin_mspm0_receive,
};

#define LIN_MSPM0_INIT(n)                                                                          \
	static const struct lin_mspm0_config lin_mspm0_config_##n = {                              \
		.regs = (UART_Regs *)DT_REG_ADDR(DT_INST_PARENT(n)),                               \
		.uart_parent = DEVICE_DT_GET(DT_INST_PARENT(n)),                                   \
	};                                                                                         \
                                                                                                   \
	static struct lin_mspm0_data lin_mspm0_data_##n = {                                        \
		.self = DEVICE_DT_INST_GET(n),                                                     \
		.mode = LIN_MODE_COMMANDER,                                                        \
		.rx_state = LIN_MSPM0_RX_STATE_IDLE,                                               \
	};                                                                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &lin_mspm0_init, NULL, &lin_mspm0_data_##n,                       \
			      &lin_mspm0_config_##n, POST_KERNEL, CONFIG_LIN_INIT_PRIORITY,        \
			      &lin_mspm0_api);

DT_INST_FOREACH_STATUS_OKAY(LIN_MSPM0_INIT)
