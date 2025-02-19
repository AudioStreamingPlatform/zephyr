/*
 * Copyright (c) 2025 Bang & Olufsen.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/fatal.h>

#include <zephyr/debug/coredump.h>

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf)
{
	printk("Rebooting\n");
	sys_reboot(SYS_REBOOT_COLD);
}

void test_backend_in_memory(void)
{
	if (coredump_query(COREDUMP_QUERY_HAS_STORED_DUMP, NULL) == 1) {
		printk("IN-MEMORY COREDUMP FOUND %d\n",
		       coredump_query(COREDUMP_QUERY_GET_STORED_DUMP_SIZE,
				      NULL));
		while (1) {
		}
	}

	printk("No coredump found\n");

	coredump_cmd(COREDUMP_CMD_ERASE_STORED_DUMP, NULL);
}

void test_fatal(void)
{
	unsigned int key = irq_lock();

	k_oops();

	irq_unlock(key);
}

int main(void)
{
	test_backend_in_memory();

	test_fatal();

	return 0;
}
