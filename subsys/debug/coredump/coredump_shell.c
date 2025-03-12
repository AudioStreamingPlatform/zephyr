/*
 * Copyright (c) 2020 Intel Corporation.
 * Copyright (c) 2025 Bang & Olufsen A/S, Denmark
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>

#include <zephyr/debug/coredump.h>

#if defined(CONFIG_DEBUG_COREDUMP_BACKEND_LOGGING)
#define COREDUMP_BACKEND_STR "logging"
#elif defined(CONFIG_DEBUG_COREDUMP_BACKEND_FLASH_PARTITION)
#define COREDUMP_BACKEND_STR "flash partition"
#elif defined(CONFIG_DEBUG_COREDUMP_BACKEND_INTEL_ADSP_MEM_WINDOW)
#define COREDUMP_BACKEND_STR "ADSP memory window"
#elif defined(CONFIG_DEBUG_COREDUMP_BACKEND_IN_MEMORY)
#define COREDUMP_BACKEND_STR "In memory - volatile -"
#elif defined(CONFIG_DEBUG_COREDUMP_BACKEND_OTHER)
#define COREDUMP_BACKEND_STR "other"
#else
#error "Need to select a coredump backend"
#endif

/* Length of buffer of printable size */
#define PRINT_BUF_SZ		64
/* Length of copy buffer */
#define COPY_BUF_SZ		128

/* Length of buffer of printable size plus null character */
#define PRINT_BUF_SZ_RAW	(PRINT_BUF_SZ + 1)

/* Print buffer */
static char print_buf[PRINT_BUF_SZ_RAW];
static off_t print_buf_ptr;

/**
 * Since enum coredump_tgt_code is sequential and starting from 0,
 * let's just have an array
 */
static const char *coredump_target_code2str[] = {
	"Unknown",
	"x86",
	"x86_64",
	"ARM Cortex-m",
	"Risc V",
	"Xtensa",
	"ARM64"
};

/**
 * @brief Shell command to get backend error.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_error_get(const struct shell *sh,
				  size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = coredump_query(COREDUMP_QUERY_GET_ERROR, NULL);
	if (ret == 0) {
		shell_print(sh, "No error.");
	} else {
		shell_print(sh, "Error: %d", ret);
	}

	return 0;
}

/**
 * @brief Shell command to clear backend error.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_error_clear(const struct shell *sh,
				    size_t argc, char **argv)
{
	int ret;

	ret = coredump_cmd(COREDUMP_CMD_CLEAR_ERROR, NULL);
	if (ret != 0) {
		shell_print(sh, "Failed to clear the error: %d", ret);
	} else {
		shell_print(sh, "Error cleared.");
	}

	return 0;
}

/**
 * @brief Shell command to see if there is a stored coredump in flash.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_has_stored_dump(const struct shell *sh,
					size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = coredump_query(COREDUMP_QUERY_HAS_STORED_DUMP, NULL);
	if (ret == 1) {
		shell_print(sh, "Stored coredump found.");
	} else if (ret == 0) {
		shell_print(sh, "Stored coredump NOT found.");
	} else {
		shell_print(sh, "Failed to perform query: %d", ret);
	}

	return 0;
}

/**
 * @brief Shell command to verify if the stored coredump is valid.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_verify_stored_dump(const struct shell *sh,
					   size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = coredump_cmd(COREDUMP_CMD_VERIFY_STORED_DUMP, NULL);
	if (ret == 1) {
		shell_print(sh, "Stored coredump verified.");
	} else if (ret == 0) {
		shell_print(sh, "Stored coredump verification failed "
			    "or there is no stored coredump.");
	} else {
		shell_print(sh, "Failed to perform verify command: %d", ret);
	}

	return 0;
}

/**
 * @brief Flush the print buffer to shell.
 *
 * This prints what is in the print buffer to the shell.
 *
 * @param sh shell instance.
 */
static void flush_print_buf(const struct shell *sh)
{
	shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, print_buf);
	print_buf_ptr = 0;
	(void)memset(print_buf, 0, sizeof(print_buf));
}

/**
 * @brief Helper to print Zephyr coredump header to shell
 *
 * @param sh shell instance
 * @param buf pointer to header
 * @return size of data processed, -EINVAL if error converting data
 */
static int print_coredump_hdr(const struct shell *sh, uint8_t *buf)
{
	struct coredump_hdr_t *hdr = (struct coredump_hdr_t *)buf;

	if (memcmp(hdr->id, "ZE", sizeof(char)*2) != 0) {
		shell_print(sh, "Not a Zephyr coredump header");
		return -EINVAL;
	}

	shell_print(sh, "**** Zephyr Coredump ****");
	shell_print(sh, "\tVersion %u", hdr->hdr_version);
	shell_print(sh, "\tTarget: %s",
		    coredump_target_code2str[sys_le16_to_cpu(
				    hdr->tgt_code)]);
	shell_print(sh, "\tPointer size: %u",
		    (uint16_t)BIT(hdr->ptr_size_bits));
	shell_print(sh, "\tFlag: %u", hdr->flag);
	shell_print(sh, "\tReason: %u", sys_le16_to_cpu(hdr->reason));

	return sizeof(struct coredump_hdr_t);
}

/**
 * @brief Helper to print stored coredump data to shell
 *
 * This converts the binary data in @p buf to hexadecimal digits
 * which can be printed to the shell.
 *
 * @param sh shell instance
 * @param buf binary data buffer
 * @param len number of bytes in buffer to be printed
 * @return 0 if no issues; -EINVAL if error converting data
 */
static int print_stored_dump(const struct shell *sh, uint8_t *buf, size_t len)
{
	int ret = 0;
	size_t i = 0;
	size_t remaining = len;

	if (len == 0) {
		/* Flush print buffer */
		flush_print_buf(sh);
		goto out;
	}

	while (remaining > 0) {
		if (hex2char(buf[i] >> 4, &print_buf[print_buf_ptr]) < 0) {
			ret = -EINVAL;
			break;
		}
		print_buf_ptr++;

		if (hex2char(buf[i] & 0xf, &print_buf[print_buf_ptr]) < 0) {
			ret = -EINVAL;
			break;
		}
		print_buf_ptr++;

		remaining--;
		i++;

		if (print_buf_ptr == PRINT_BUF_SZ) {
			flush_print_buf(sh);
		}
	}

out:
	return ret;
}

static int parse_and_print_coredump(const struct shell *sh,
				    bool *coredump_header,
				    struct coredump_cmd_copy_arg *copy,
				    size_t left_size)
{
	int processed_size = 0;
	int data_size = 0;
	int ret;

	if (!*coredump_header) {
		copy->length = sizeof(struct coredump_hdr_t);
	} else {
		/* thread header is of same size, but memory header */
		copy->length = sizeof(struct coredump_arch_hdr_t);
	}

	if (copy->length > left_size) {
		return -ENOMEM;
	}

	ret = coredump_cmd(COREDUMP_CMD_COPY_STORED_DUMP, copy);
	if (ret != 0) {
		return -ENOMEM;
	}

	if (!*coredump_header) {
		ret = print_coredump_hdr(sh, copy->buffer);
		shell_print(sh, "\tSize of the coredump: %u\n", left_size);

		*coredump_header = true;
		copy->offset += copy->length;

		goto hdr_done;
	}

	/* hdr id */
	switch (copy->buffer[0]) {
	case COREDUMP_ARCH_HDR_ID: {
		struct coredump_arch_hdr_t *hdr =
			(struct coredump_arch_hdr_t *)copy->buffer;

		shell_print(sh, "-> Arch coredump header found");
		shell_print(sh, "\tVersion %u", hdr->hdr_version);
		shell_print(sh, "\tSize %u", hdr->num_bytes);

		data_size = hdr->num_bytes;
		copy->offset += copy->length;
		break;
	}
	case THREADS_META_HDR_ID: {
		struct coredump_threads_meta_hdr_t *hdr =
			(struct coredump_threads_meta_hdr_t *)copy->buffer;

		shell_print(sh, "-> Thread coredump header found");
		shell_print(sh, "\tVersion %u", hdr->hdr_version);
		shell_print(sh, "\tSize %u", hdr->num_bytes);

		data_size = hdr->num_bytes;
		copy->offset += copy->length;
		break;
	}
	case COREDUMP_MEM_HDR_ID: {
		struct coredump_mem_hdr_t *hdr;

		copy->length = sizeof(struct coredump_mem_hdr_t);
		if (copy->length > left_size) {
			return -ENOMEM;
		}

		ret = coredump_cmd(COREDUMP_CMD_COPY_STORED_DUMP, copy);
		if (ret != 0) {
			return -ENOMEM;
		}

		hdr = (struct coredump_mem_hdr_t *)copy->buffer;

		if (sizeof(uintptr_t) == 8) {
			hdr->start = sys_le64_to_cpu(hdr->start);
			hdr->end = sys_le64_to_cpu(hdr->end);
		} else {
			hdr->start = sys_le32_to_cpu(hdr->start);
			hdr->end = sys_le32_to_cpu(hdr->end);
		}

		data_size = hdr->end - hdr->start;

		shell_print(sh, "-> Memory coredump header found");
		shell_print(sh, "\tVersion %u", hdr->hdr_version);
		shell_print(sh, "\tSize %u", data_size);
		shell_print(sh, "\tStarts at %p ends at %p",
			    (void *)hdr->start, (void *)hdr->end);
		copy->offset += copy->length;
		break;
	}
	default:
		return -EINVAL;
	}

	if (data_size > left_size) {
		return -ENOMEM;
	}

hdr_done:
	processed_size += copy->length + data_size;

	if (data_size == 0) {
		goto out;
	}

	copy->length = COPY_BUF_SZ;

	print_buf_ptr = 0;
	(void)memset(print_buf, 0, sizeof(print_buf));

	shell_print(sh, "Data:");
	shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, COREDUMP_BEGIN_STR);

	while (data_size > 0) {
		if (data_size < COPY_BUF_SZ) {
			copy->length = data_size;
		}

		ret = coredump_cmd(COREDUMP_CMD_COPY_STORED_DUMP, copy);
		if (ret != 0) {
			return -EINVAL;
		}

		ret = print_stored_dump(sh, copy->buffer, copy->length);
		if (ret != 0) {
			return -EINVAL;
		}

		if (print_buf_ptr != 0) {
			shell_print(sh, "%s%s", COREDUMP_PREFIX_STR, print_buf);
		}

		copy->offset += copy->length;
		data_size -= copy->length;
	}

	shell_print(sh, "%s%s\n", COREDUMP_PREFIX_STR, COREDUMP_END_STR);

out:
	ret = processed_size;
	return ret;
}


/**
 * @brief Shell command to print stored coredump data to shell
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_print_stored_dump(const struct shell *sh,
					  size_t argc, char **argv)
{
	uint8_t rbuf[COPY_BUF_SZ];
	struct coredump_cmd_copy_arg copy = {
		.offset = 0,
		.buffer = rbuf,
	};
	bool cdump_hdr = false;
	int size;
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	/* Verify first to see if stored dump is valid */
	ret = coredump_cmd(COREDUMP_CMD_VERIFY_STORED_DUMP, NULL);
	if (ret == 0) {
		shell_print(sh, "Stored coredump verification failed "
			    "or there is no stored coredump.");
		goto out;
	} else if (ret != 1) {
		shell_print(sh, "Failed to perform verify command: %d", ret);
		goto out;
	}

	size = coredump_query(COREDUMP_QUERY_GET_STORED_DUMP_SIZE, NULL);
	if (size <= 0) {
		shell_print(sh, "Invalid coredump size: %d", size);
		goto out;
	}

	while (size > 0) {
		ret = parse_and_print_coredump(sh, &cdump_hdr, &copy, size);
		if (ret < 0) {
			goto error;
		}

		if (ret == 0) {
			break;
		}

		size -= ret;
	}

	if (coredump_query(COREDUMP_QUERY_GET_ERROR, NULL) != 0) {
		shell_print(sh, "%s%s", COREDUMP_PREFIX_STR,
			    COREDUMP_ERROR_STR);
	}

	shell_print(sh, "Stored coredump printed");

	goto out;

error:
	shell_print(sh, "Error while retrieving/parsing coredump: %d", ret);
out:
	return 0;
}

/**
 * @brief Shell command to erase stored coredump.
 *
 * @param sh shell instance
 * @param argc (not used)
 * @param argv (not used)
 * @return 0
 */
static int cmd_coredump_erase_stored_dump(const struct shell *sh,
					  size_t argc, char **argv)
{
	int ret;

	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	ret = coredump_cmd(COREDUMP_CMD_ERASE_STORED_DUMP, NULL);
	if (ret == 0) {
		shell_print(sh, "Stored coredump erased.");
	} else {
		shell_print(sh, "Failed to perform erase command: %d", ret);
	}

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_coredump_error,
	SHELL_CMD(clear, NULL, "Clear Coredump error",
		  cmd_coredump_error_clear),
	SHELL_CMD(get, NULL, "Get Coredump error", cmd_coredump_error_get),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_coredump,
	SHELL_CMD(error, &sub_coredump_error,
		  "Get/clear backend error.", NULL),
	SHELL_CMD(erase, NULL,
		  "Erase stored coredump",
		  cmd_coredump_erase_stored_dump),
	SHELL_CMD(find, NULL,
		  "Query if there is a stored coredump",
		  cmd_coredump_has_stored_dump),
	SHELL_CMD(print, NULL,
		  "Print stored coredump to shell",
		  cmd_coredump_print_stored_dump),
	SHELL_CMD(verify, NULL,
		  "Verify stored coredump",
		  cmd_coredump_verify_stored_dump),
	SHELL_SUBCMD_SET_END /* Array terminated. */
);

SHELL_CMD_REGISTER(coredump, &sub_coredump,
		   "Coredump commands (" COREDUMP_BACKEND_STR " backend)",
		   NULL);
