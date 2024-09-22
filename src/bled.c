/*******************************************************************************
*                                                                              *
*   Description: Board LED control                                             *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#include "stdinc.h"
#include "eminc.h"

#include "bledcfg.h"

#include "bled.h"
/******************************************************************************/
/**
 * @brief Board LED data
 */
typedef struct bled_data_s {
	struct hw_s {
		uint8_t port;
		uint8_t pin;
	} hw;
} bled_data_t;

/**
 * @brief Board LED private data
 */
typedef struct bled_priv_data_s {
	bled_data_t led[BLED_LED_CNT];
} bled_priv_data_t;

/**
 * @brief Board LED private data instance
 */
static bled_priv_data_t bled_priv_data;

/******************************************************************************/
int bled_init(bled_init_param_t const * param) {
	assert(param != NULL);

	/* Initialize private data */
	memset(&bled_priv_data, 0, sizeof(bled_priv_data));

	/* Initialize board LEDs */
	for (bled_led_t bled = 0; bled < BLED_LED_CNT; bled++) {
		bled_data_t * bled_p = &bled_priv_data.led[bled];
		bled_hwled_t const * hwled_p = &param->hwled[bled];

		bled_p->hw.port = hwled_p->port;
		bled_p->hw.pin = hwled_p->pin;

		GPIO_PinModeSet(bled_p->hw.port, bled_p->hw.pin, gpioModePushPull, 0);
	}

	return 0;
}

int bled_ctrl(bled_led_t bled, bled_cmd_t cmd, void * cmd_arg_p) {
	assert(bled < BLED_LED_CNT);
	bled_data_t * bled_p = &bled_priv_data.led[bled];

	switch (cmd) {
		case BLED_CMD_ON:
			GPIO_PinOutSet(bled_p->hw.port, bled_p->hw.pin);
			break;
		case BLED_CMD_OFF:
			GPIO_PinOutClear(bled_p->hw.port, bled_p->hw.pin);
			break;
		case BLED_CMD_TOGGLE:
			GPIO_PinOutToggle(bled_p->hw.port, bled_p->hw.pin);
			break;
		case BLED_CMD_BLINK:
		case BLED_CMD_FLASH:
		case BLED_CMD_PULSE:
			(void)cmd_arg_p;
			return -1; /* Not supported yet */
			break;
		default:
			return -1; /* Invalid command */
			break;
	}

	return 0;
}
