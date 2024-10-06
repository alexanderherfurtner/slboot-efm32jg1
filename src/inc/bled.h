/*******************************************************************************
*                                                                              *
*   Description: Board LED                                                     *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef BLED_H
#define BLED_H

/******************************************************************************/
/**
 * @brief Board LED commands
 */
typedef enum bled_cmd_e {
	BLED_CMD_ON,
	BLED_CMD_OFF,
	BLED_CMD_TOGGLE,
	BLED_CMD_BLINK,
	BLED_CMD_FLASH,
	BLED_CMD_PULSE,
} bled_cmd_t;

/**
 * @brief Board LED hardware configuration
 */
typedef struct bled_hwled_s {
	int8_t gpio_port;
	int8_t gpio_pin;
	void *tmr_base;
	uint8_t tmr_route;
	uint8_t tmr_ch;
	void *ldma_base;
	uint8_t ldma_ch;
} bled_hwled_t;

/**
 * @brief Board LED flash command arguments
 */
typedef struct bled_flash_cmd_arg_s {
	uint32_t delay_ms;
	uint32_t flash_ms;
} bled_flash_cmd_arg_t;

/**
 * @brief Board LED blink command arguments
 */
typedef struct bled_blink_cmd_arg_s {
	uint32_t blink_ms;
	uint32_t blink_ratio;
} bled_blink_cmd_arg_t;

/**
 * @brief Board LED initialization parameter
 */
typedef struct bled_init_param_s {
	bled_hwled_t hwled[BLED_LED_CNT];
} bled_init_param_t;

/**
 * @brief Board LED initialization
 *
 * @param param Initialization parameter
 * @return int 0 on success, -1 on error
 */
int bled_init(bled_init_param_t const * param);

/**
 * @brief Board LED control
 *
 * @param bled_p Pointer to the board LED
 * @param cmd Command to execute
 * @param cmd_arg_p Command argument
 */
int bled_ctrl(bled_led_t bled, bled_cmd_t cmd, void * cmd_arg_p);

/**
 * @brief Board LED on
 *
 * @param bled Board LED
 * @return int 0 on success, -1 on error
 */
static inline int bled_ctrl_on(bled_led_t bled) {
	return bled_ctrl(bled, BLED_CMD_ON, NULL);
}

/**
 * @brief Board LED off
 *
 * @param bled Board LED
 * @return int 0 on success, -1 on error
 */
static inline int bled_ctrl_off(bled_led_t bled) {
	return bled_ctrl(bled, BLED_CMD_OFF, NULL);
}

/**
 * @brief Board LED toggle
 *
 * @param bled Board LED
 * @return int 0 on success, -1 on error
 */
static inline int bled_ctrl_toggle(bled_led_t bled) {
	return bled_ctrl(bled, BLED_CMD_TOGGLE, NULL);
}

/**
 * @brief Board LED blink
 *
 * @param bled Board LED
 * @param blink_ratio Blink ratio in percent
 * @return int 0 on success, -1 on error
 */
static inline int bled_ctrl_flash(bled_led_t bled, uint32_t flash_ms) {
	bled_flash_cmd_arg_t flash_args = {
		.delay_ms = 0,
		.flash_ms = flash_ms
	};
	return bled_ctrl(bled, BLED_CMD_FLASH, &flash_args);
}

/**
 * @brief Board LED blink
 *
 * @param bled Board LED
 * @param blink_ratio Blink ratio in percent
 * @param blink_ms Blink period in milliseconds
 * @return int 0 on success, -1 on error
 */
static inline int bled_ctrl_flash_dly(bled_led_t bled, uint32_t delay_ms, uint32_t flash_ms) {
	bled_flash_cmd_arg_t flash_args = {
		.delay_ms = delay_ms,
		.flash_ms = flash_ms
	};
	return bled_ctrl(bled, BLED_CMD_FLASH, &flash_args);
}

#endif // BLED_H
