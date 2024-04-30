# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/lin.h)

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_LIN_UART lin_uart.c)
zephyr_library_sources_ifdef(CONFIG_LIN_MSPM0 lin_mspm0.c)