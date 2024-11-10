/*******************************************************************************
*                                                                              *
*   Description: Open Smart Label Bootloader CLI                               *
*                                                                              *
*   Author:  Alexander Herfurtner (mail\alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#include "stdinc.h"
#include "eminc.h"

#include "dbgcfg.h"
#include "bootcfg.h"
#include "bledcfg.h"
#include "sysinfocfg.h"

#include "bled.h"
#include "fatal.h"
/******************************************************************************/
static void cli_print_usage(char* err_msg);

static int cli_cmd_help(int argc, char *argv[]);
static int cli_cmd_erase(int argc, char *argv[]);
static int cli_cmd_write(int argc, char *argv[]);
static int cli_cmd_read(int argc, char *argv[]);
static int cli_cmd_led(int argc, char *argv[]);
static int cli_cmd_reset(int argc, char *argv[]);

/******************************************************************************/
#define CLI_INPUT_BUF_SIZE	64 /* Input buffer size */
#define CLI_MAX_ARGV_CNT	(3 + 1) /* Maximum number of arguments (+1 command) */

#define CLI_USAGE_DESC_MARGIN	30
#define CLI_USAGE_FILL_CHAR		'.'

static bool cli_intro = true;
static char cli_in_buf[CLI_INPUT_BUF_SIZE];

typedef int (*cli_cmd_func_t)(int argc, char *argv[]);

/**
 * \brief Bootloader CLI command structure
 * 
 * This structure defines the CLI command and the corresponding function as a
 * function pointer.
 */
struct cli_cmd {
	const char *cmd;
	const char *usage;
	const char *desc;
	const cli_cmd_func_t func;
};

/**
 * \brief Bootloader CLI commands and their corresponding functions
 */
static const struct cli_cmd cli_cmd_tbl[] = {
	{ "h",  "h",                       "help",         cli_cmd_help  },
	{ "me", "me <sec>",                "memory erase", cli_cmd_erase },
	{ "mw", "mw <b|s|w> <addr> <val>", "memory write", cli_cmd_write },
	{ "mr", "mr <b|s|w> <addr>",       "memory read",  cli_cmd_read  },
	{ "lc", "lc <i> <cmd>",            "led control",  cli_cmd_led   },
	{ "r",  "r <d>",                   "reset",        cli_cmd_reset },
};

/******************************************************************************/
void cli_init(void) {
	; /* Nothing to do */
}

void cli_task(void) {
	char *arg;
	char *argv[CLI_MAX_ARGV_CNT];
	uint8_t in_cnt, arg_cnt;
	cli_cmd_func_t cmd_func;

	if (cli_intro) {
		cli_intro = false;
		dbg_printf("Entering slboot CLI\n");
		goto prompt;
	}

	memset(cli_in_buf, 0, CLI_INPUT_BUF_SIZE);
	in_cnt = SEGGER_RTT_Read(0, (void*)cli_in_buf, CLI_INPUT_BUF_SIZE);
	if (0 == in_cnt)
		return;

	if (NULL == strtok(cli_in_buf, "\n")) {
		goto prompt;
	}

	argv[0] = strtok(cli_in_buf, " ");
	if (NULL == argv[0]) {
		cli_print_usage("Invalid command");
		goto prompt;
	}

	arg_cnt = 0;
	while ((arg = strtok(NULL, " ")) != NULL) {
		argv[++arg_cnt] = arg;
		if (CLI_MAX_ARGV_CNT < arg_cnt) {
			cli_print_usage("Too many arguments");
			goto prompt;
		}
	}

	cmd_func = NULL;
	for (uint8_t i = 0; i < sizeof(cli_cmd_tbl) / sizeof(cli_cmd_tbl[0]); i++) {
		if (strcmp(argv[0], cli_cmd_tbl[i].cmd) == 0) {
			cmd_func = cli_cmd_tbl[i].func;
			break;
		}
	}

	if (cmd_func)
		(void)cmd_func(arg_cnt, &argv[1]);
	else
		cli_print_usage("Unknown command");

	prompt:
		SEGGER_RTT_Write(0, ">", 1); /* Prompt */
}

/**
 * \brief Print CLI usage
 * 
 * \param err_msg Error message
 */
void cli_print_usage(char* err_msg) {
	if (err_msg)
		dbg_printf("Error: %s\n", err_msg);

	for (uint8_t i = 0; i < sizeof(cli_cmd_tbl) / sizeof(cli_cmd_tbl[0]); i++) {
		SEGGER_RTT_printf(0, "%s", cli_cmd_tbl[i].usage);
		for (uint8_t j = strlen(cli_cmd_tbl[i].usage); j < CLI_USAGE_DESC_MARGIN; j++)
			SEGGER_RTT_printf(0, "%c", CLI_USAGE_FILL_CHAR);
		SEGGER_RTT_printf(0, "%s\n", cli_cmd_tbl[i].desc);
	}
}

/**
 * \brief CLI command: help
 * 
 * \param argc Argument count
 * \param argv Argument vector
 * 
 * \return int 0 on success, -1 on error
 */
int cli_cmd_help(int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	cli_print_usage(NULL);
	return 0;
}

/**
 * \brief CLI command: me (memory erase)
 * 
 * \param argc Argument count
 * \param argv Argument vector
 * 
 * \return int 0 on success, -1 on error
 */
int cli_cmd_erase(int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	MSC_Init();
	return (int)MSC_MassErase();
}

/**
 * \brief CLI command: mw (memory write)
 * 
 * \param argc Argument count
 * \param argv Argument vector
 * 
 * \return int 0 on success, -1 on error
 */
int cli_cmd_write(int argc, char *argv[]) {
	char wt;
	uint32_t addr, val;

	if (3 != argc) {
		dbg_printf("Invalid argument count\n");
		return -1;
	}

	wt = argv[0][0];
	val = (uint32_t)strtoul(argv[2], NULL, 16);
	addr = (uint32_t)strtoul(argv[1], NULL, 16);
	switch(wt) {
		case 'b':
			*(uint8_t*)addr = (uint8_t)val;
			break;
		case 's':
			*(uint16_t*)addr = (uint16_t)val;
			break;
		case 'w':
			*(uint32_t*)addr = val;
			break;
		default:
			dbg_printf("Invalid width type\n");
			return -1;
	}

	return 0;
}

/**
 * \brief CLI command: mr (memory read)
 * 
 * \param argc Argument count
 * \param argv Argument vector
 */
int cli_cmd_read(int argc, char *argv[]) {
	char wt;
	uint32_t addr;

	if (2 != argc) {
		dbg_printf("Invalid argument count\n");
		return -1;
	}

	wt = argv[0][0];
	addr = (uint32_t)strtoul(argv[1], NULL, 16);
	switch(wt) {
		case 'b':
			dbg_printf("%02X\n", *(uint8_t*)addr);
			break;
		case 's':
			dbg_printf("%04X\n", *(uint16_t*)addr);
			break;
		case 'w':
			dbg_printf("%08X\n", *(uint32_t*)addr);
			break;
		default:
			dbg_printf("Invalid mem width type\n");
			return -1;
	}

	return 0;
}

/**
 * \brief CLI command: lc (led control)
 * 
 * \param argc Argument count
 * \param argv Argument vector
 * 
 * \return int 0 on success, -1 on error
 */
int cli_cmd_led(int argc, char *argv[]) {
	uint8_t led_id;
	const char *led_cmd;
	
	if (2 != argc) {
		dbg_printf("Invalid argument count\n");
		return -1;
	}

	led_id = (uint8_t)strtoul(argv[0], NULL, 16);
	if (led_id >= BLED_LED_CNT) {
		dbg_printf("Invalid led id\n");
		return -1;
	}

	led_cmd = argv[1];
	if (!strcmp(led_cmd, "on")) {
		bled_ctrl_on(led_id);
	} else if (!strcmp(led_cmd, "off")) {
		bled_ctrl_off(led_id);
	} else if (!strcmp(led_cmd, "tog")) {
		bled_ctrl_toggle(led_id);
	} else if (!strcmp(led_cmd, "fla")) {
		bled_ctrl_flash(led_id, 100);
	} else {
		dbg_printf("Invalid led cmd\n");
		return -1;
	}

	return 0;
}

/**
 * \brief CLI command: r (reset)
 * 
 * \param argc Argument count
 * \param argv Argument vector
 * 
 * \return int 0 on success, -1 on error
 */
int cli_cmd_reset(int argc, char *argv[]) {
	(void)argc;
	(void)argv;

	extern void sys_reset(void);
	sys_reset();
	return 0;
}
