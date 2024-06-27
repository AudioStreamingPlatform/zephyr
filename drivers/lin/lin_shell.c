/*
 * Copyright (C) 2024 Bang & Olufsen A/S, Denmark
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <zephyr/drivers/lin.h>
#include <zephyr/shell/shell.h>

#define ARGV_DEV  1
#define ARGV_ID   2
#define ARGV_DATA 3

static void lin_shell_rx_cb(const struct device *dev, int error, const struct lin_frame *msg,
			    void *data)
{
	if (error != 0) {
		printk("rx failed, err: %d\n", error);
		goto out;
	}

	printk("%s rx [%u]: ", dev->name, msg->len);
	for (uint8_t i = 0; i < msg->len; i++) {
		printk("0x%02x ", msg->data[i]);
	}
	printk("\n");
out:
	lin_set_rx_callback(dev, NULL, NULL);
}

static int cmd_lin_send(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = device_get_binding(argv[ARGV_DEV]);

	if (!device_is_ready(dev)) {
		shell_error(sh, "%s is not ready", dev->name);
		return -ENODEV;
	}

	const uint8_t id = strtol(argv[ARGV_ID], NULL, 16);
	if (id >= LIN_NUM_ID) {
		shell_error(sh, "invalid id %u >= %u", id, LIN_NUM_ID);
		return -EINVAL;
	}

	struct lin_frame frame = {
		.id = id,
		.len = (argc - ARGV_DATA),
		.type = LIN_CHECKSUM_ENHANCED,
	};

	for (uint8_t i = 0; i < frame.len; i++) {
		frame.data[i] = strtol(argv[ARGV_DATA + i], NULL, 16);
	}

	int res = lin_send(dev, &frame);
	if (res != 0) {
		shell_error(sh, "send failed, err: %d", res);
	} else {
		shell_print(sh, "sent %u bytes to %u", frame.len, frame.id);
	}

	return res;
}

static int cmd_lin_receive(const struct shell *sh, size_t argc, char **argv)
{
	const struct device *dev = device_get_binding(argv[ARGV_DEV]);

	if (!device_is_ready(dev)) {
		shell_error(sh, "%s is not ready", dev->name);
		return -ENODEV;
	}

	const uint8_t id = atoi(argv[ARGV_ID]);
	if (id >= LIN_NUM_ID) {
		shell_error(sh, "invalid id %u >= %u", id, LIN_NUM_ID);
		return -EINVAL;
	}

	int res = lin_set_rx_callback(dev, lin_shell_rx_cb, NULL);

	const uint8_t len = atoi(argv[ARGV_ID]);
	lin_receive(dev, id, LIN_CHECKSUM_ENHANCED, len);

	return res;
}

static void device_name_get(size_t idx, struct shell_static_entry *entry)
{
	const struct device *dev = shell_device_lookup(idx, NULL);

	entry->syntax = (dev != NULL) ? dev->name : NULL;
	entry->handler = NULL;
	entry->help = NULL;
	entry->subcmd = NULL;
}

SHELL_DYNAMIC_CMD_CREATE(sub_lin_dev, device_name_get);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_lin,
			       SHELL_CMD_ARG(send, &sub_lin_dev,
					     "lin send\n"
					     "Usage: lin send <device> <id> <data..8>",
					     cmd_lin_send, 3, 8),
			       SHELL_CMD_ARG(receive, &sub_lin_dev,
					     "lin receive\n"
					     "Usage: lin receive <device> <id> <receive_amount>",
					     cmd_lin_receive, 4, 0),
			       SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(lin, &sub_lin, "LIN commands", NULL);
