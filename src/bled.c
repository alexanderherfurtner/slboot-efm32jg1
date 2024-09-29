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

#include "dbgcfg.h"
#include "bledcfg.h"

#include "bled.h"
/******************************************************************************/
/**
 * BLED private function prototypes
 */
static int bled_hw_gpio_init(bled_led_t bled, bled_init_param_t const * param);
static int bled_hw_gpio_deinit(bled_led_t bled) __attribute__((unused));
static int bled_hw_tmr_init(bled_led_t bled, bled_init_param_t const * param);
static int bled_hw_tmr_deinit(bled_led_t bled) __attribute__((unused));
static int bled_hw_ldma_init(bled_led_t bled, bled_init_param_t const * param);
static int bled_hw_ldma_deinit(bled_led_t bled) __attribute__((unused));

static int bled_hw_ctrl_cmd_on(bled_led_t bled);
static int bled_hw_ctrl_cmd_off(bled_led_t bled);
static int bled_hw_ctrl_cmd_toggle(bled_led_t bled);
static int bled_hw_ctrl_cmd_flash(bled_led_t bled, bled_flash_cmd_arg_t* cmd_arg_p);

/******************************************************************************/
/**
 * @brief Board LED data
 */
typedef struct bled_data_s {
	struct hw_s {
		uint8_t port;
		uint8_t pin;
	} hw_gpio;
	struct hw_tmr_s {
		TIMER_TypeDef * base;
		uint32_t ch;
		uint32_t route;
		TIMER_InitCC_TypeDef ch_init;
		uint32_t ldma_buf[2];
	} hw_tmr;
	struct hw_ldma_s {
		LDMA_TypeDef * base;
		uint32_t ch;
		LDMA_TransferCfg_t xfer_cfg;
		LDMA_Descriptor_t desc[2];
	} hw_ldma;
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

	/* Initialize board LEDs hardware */
	for (bled_led_t bled = 0; bled < BLED_LED_CNT; bled++) {
		bled_hw_gpio_init(bled, param);
		bled_hw_tmr_init(bled, param);
		bled_hw_ldma_init(bled, param);
	}

	return 0;
}

int bled_ctrl(bled_led_t bled, bled_cmd_t cmd, void * cmd_arg_p) {
	assert(bled < BLED_LED_CNT);

	switch (cmd) {
		case BLED_CMD_ON:
			return bled_hw_ctrl_cmd_on(bled);
			break;
		case BLED_CMD_OFF:
			return bled_hw_ctrl_cmd_off(bled);
			break;
		case BLED_CMD_TOGGLE:
			return bled_hw_ctrl_cmd_toggle(bled);
			break;
		case BLED_CMD_FLASH:
			return bled_hw_ctrl_cmd_flash(bled, cmd_arg_p);
			break;
		case BLED_CMD_BLINK:
		case BLED_CMD_PULSE:
			(void)cmd_arg_p;
			break;
		default:
			return -1; /* Invalid command */
			break;
	}

	return 0;
}

/**
 * @brief Initialize GPIO peripheral
 *
 * @param bled Board LED
 * @param param Initialization parameter
 */
static int bled_hw_gpio_init(bled_led_t bled, bled_init_param_t const * param) {
	assert(bled < BLED_LED_CNT);
	assert(param != NULL);
	bled_data_t * bled_p = &bled_priv_data.led[bled];
	bled_hwled_t const * hwled_p = &param->hwled[bled];

	bled_p->hw_gpio.port = hwled_p->gpio_port;
	bled_p->hw_gpio.pin = hwled_p->gpio_pin;
	GPIO_PinModeSet(bled_p->hw_gpio.port, bled_p->hw_gpio.pin, gpioModePushPull, 0);

	return 0;
}

/**
 * @brief Deinitialize GPIO peripheral
 *
 * @param bled Board LED
 * @return int 0 on success, -1 on error
 */
static int bled_hw_gpio_deinit(bled_led_t bled) {
	assert(bled < BLED_LED_CNT);
	bled_data_t * bled_p = &bled_priv_data.led[bled];

	GPIO_PinModeSet(bled_p->hw_gpio.port, bled_p->hw_gpio.pin, gpioModeDisabled, 0);

	return 0;
}

/**
 * @brief Initialize TIMER peripheral
 *
 * @param bled Board LED
 * @param param Initialization parameter
 * @return int 0 on success, -1 on error
 */
static int bled_hw_tmr_init(bled_led_t bled, bled_init_param_t const * param) {
	assert(bled < BLED_LED_CNT);
	assert(param != NULL);
	bled_data_t * bled_p = &bled_priv_data.led[bled];
	bled_hwled_t const * hwled_p = &param->hwled[bled];

	bled_p->hw_tmr.base = hwled_p->tmr_base;
	bled_p->hw_tmr.ch = hwled_p->tmr_ch;
	bled_p->hw_tmr.route = hwled_p->tmr_route;

	if (!bled_p->hw_tmr.base)
		return 0;

	bled_p->hw_tmr.ch_init = (TIMER_InitCC_TypeDef)TIMER_INITCC_DEFAULT;
	TIMER_InitCC(bled_p->hw_tmr.base, bled_p->hw_tmr.ch, &bled_p->hw_tmr.ch_init);
	TIMER_RouteCCSet(bled_p->hw_tmr.base, bled_p->hw_tmr.ch, bled_p->hw_tmr.route);

	return 0;
}

/**
 * @brief Deinitialize TIMER peripheral
 *
 * @param bled Board LED
 * @return int 0 on success, -1 on error
 */
static int bled_hw_tmr_deinit(bled_led_t bled) {
	assert(bled < BLED_LED_CNT);

	bled_data_t * bled_p = &bled_priv_data.led[bled];

	if (!bled_p->hw_tmr.base)
		return 0;

	bled_p->hw_tmr.ch_init = (TIMER_InitCC_TypeDef)TIMER_INITCC_DEFAULT;
	TIMER_InitCC(bled_p->hw_tmr.base, bled_p->hw_tmr.ch, &bled_p->hw_tmr.ch_init);
	TIMER_RouteCCClr(bled_p->hw_tmr.base, bled_p->hw_tmr.ch);

	return 0;
}

/**
 * @brief Initialize LDMA peripheral
 *
 * @param bled Board LED
 * @param param Initialization parameter
 */
static int bled_hw_ldma_init(bled_led_t bled, bled_init_param_t const * param) {
	assert(bled < BLED_LED_CNT);
	assert(param != NULL);

	bled_data_t * bled_p = &bled_priv_data.led[bled];
	bled_hwled_t const * hwled_p = &param->hwled[bled];

	bled_p->hw_ldma.base = hwled_p->ldma_base;
	bled_p->hw_ldma.ch = hwled_p->ldma_ch;

	return 0;
}

/**
 * @brief Deinitialize LDMA peripheral
 *
 * @param bled Board LED
 */
static int bled_hw_ldma_deinit(bled_led_t bled) {
	assert(bled < BLED_LED_CNT);

	bled_data_t * bled_p = &bled_priv_data.led[bled];

	if (!bled_p->hw_ldma.base)
		return 0;

	LDMA_StopTransfer(bled_priv_data.led[bled].hw_ldma.ch);
	return 0;
}

/**
 * @brief Turn on command implementation using GPIO peripheral
 *
 * @param bled Board LED
 */
static int bled_hw_ctrl_cmd_on(bled_led_t bled) {
	assert(bled < BLED_LED_CNT);
	bled_data_t * bled_p = &bled_priv_data.led[bled];

	if (bled_p->hw_tmr.base)
		TIMER_RouteCCClr(bled_p->hw_tmr.base, bled_p->hw_tmr.ch);

	GPIO_PinOutSet(bled_p->hw_gpio.port, bled_p->hw_gpio.pin);

	return 0;
}

/**
 * @brief Turn off command implementation
 *
 * @param bled Board LED
 */
static int bled_hw_ctrl_cmd_off(bled_led_t bled) {
	assert(bled < BLED_LED_CNT);
	bled_data_t * bled_p = &bled_priv_data.led[bled];

	if (bled_p->hw_tmr.base)
		TIMER_RouteCCClr(bled_p->hw_tmr.base, bled_p->hw_tmr.ch);

	GPIO_PinOutClear(bled_p->hw_gpio.port, bled_p->hw_gpio.pin);

	return 0;
}

/**
 * @brief Toggle command implementation using GPIO peripheral
 *
 * @param bled Board LED
 */
static int bled_hw_ctrl_cmd_toggle(bled_led_t bled) {
	assert(bled < BLED_LED_CNT);
	bled_data_t * bled_p = &bled_priv_data.led[bled];

	if (bled_p->hw_tmr.base)
		TIMER_RouteCCClr(bled_p->hw_tmr.base, bled_p->hw_tmr.ch);

	GPIO_PinOutToggle(bled_p->hw_gpio.port, bled_p->hw_gpio.pin);

	return 0;
}

/**
 * @brief Flash command implementation using TIMER and LDMA peripherals
 *
 * @param bled Board LED
 * @param cmd_arg_p Command argument
 */
static int bled_hw_ctrl_cmd_flash(bled_led_t bled, bled_flash_cmd_arg_t* cmd_arg_p) {
	assert(bled < BLED_LED_CNT);
	uint32_t tmr_clk_freq;
	uint32_t tmr_delay_ticks,
			 tmr_flash_ticks,
			 tmr_cnt_tick;
	bled_data_t * bled_p = &bled_priv_data.led[bled];

	if (!bled_p->hw_tmr.base || !bled_p->hw_ldma.base)
		return -1; // no TMR or LDMA, but required for flash command

	if (!cmd_arg_p->flash_ms)
		return -1; // invalid flash duration

	if (!cmd_arg_p->delay_ms)
		cmd_arg_p->delay_ms = 1; // add some pre-processing time

	TIMER_RouteCCClr(bled_p->hw_tmr.base, bled_p->hw_tmr.ch);
	GPIO_PinOutClear(bled_p->hw_gpio.port, bled_p->hw_gpio.pin);

	tmr_clk_freq = TIMER_ClockFreqGet(bled_p->hw_tmr.base);
	tmr_delay_ticks = tmr_clk_freq * cmd_arg_p->delay_ms / 1000;
	tmr_flash_ticks = tmr_clk_freq * cmd_arg_p->flash_ms / 1000;

	if (tmr_delay_ticks + tmr_flash_ticks > TIMER_MaxCount(bled_p->hw_tmr.base))
		dbg_print("bled: warning: flash command ticks calculation overflow.\n");

	/* Tmr CC compare control (buffer: ldma_buf[0]) */
	bled_p->hw_ldma.desc[0] = (LDMA_Descriptor_t)
		LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(&bled_p->hw_tmr.ldma_buf[0],
		&bled_p->hw_tmr.base->CC[bled_p->hw_ldma.ch].CCV,
		1, 1);
	bled_p->hw_ldma.desc[0].xfer.size = ldmaCtrlSizeWord;
	bled_p->hw_ldma.desc[0].xfer.doneIfs = 0;

	/* Tmr CC mode control (buffer: ldma_buf[1]) */
	bled_p->hw_ldma.desc[1] = (LDMA_Descriptor_t)
		LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(&bled_p->hw_tmr.ldma_buf[1],
		&bled_p->hw_tmr.base->CC[bled_p->hw_ldma.ch].CTRL,
		1);
	bled_p->hw_ldma.desc[1].xfer.size = ldmaCtrlSizeWord;
	bled_p->hw_ldma.desc[1].xfer.doneIfs = 0;

	uint32_t ldma_ch_srcsel_tmr = LDMA_CH_REQSEL_SOURCESEL_TIMER0;

	if (bled_p->hw_tmr.base == TIMER1)
		ldma_ch_srcsel_tmr = LDMA_CH_REQSEL_SOURCESEL_TIMER1;

	/* Init LDMA transfer */
	bled_p->hw_ldma.xfer_cfg = (LDMA_TransferCfg_t)
		LDMA_TRANSFER_CFG_PERIPHERAL(LDMA_CH_REQSEL_SIGSEL_TIMER0CC0
			<< bled_p->hw_tmr.ch | ldma_ch_srcsel_tmr);

	LDMA_StartTransfer(bled_p->hw_ldma.ch, &bled_p->hw_ldma.xfer_cfg,
		&bled_p->hw_ldma.desc[0]);

	bled_p->hw_tmr.ch_init = (TIMER_InitCC_TypeDef)TIMER_INITCC_DEFAULT;
	bled_p->hw_tmr.ch_init.mode = timerCCModeCompare;
	bled_p->hw_tmr.ch_init.cmoa = timerOutputActionToggle;
	TIMER_InitCC(bled_p->hw_tmr.base, bled_p->hw_tmr.ch,
		&bled_p->hw_tmr.ch_init);

	bled_p->hw_tmr.ldma_buf[1] = _TIMER_CC_CTRL_RESETVALUE;

	tmr_cnt_tick = TIMER_CounterGet(bled_p->hw_tmr.base);
	bled_p->hw_tmr.ldma_buf[0] = tmr_cnt_tick
		+ tmr_delay_ticks
		+ tmr_flash_ticks;
	TIMER_CompareSet(bled_p->hw_tmr.base, bled_p->hw_tmr.ch, tmr_cnt_tick
		+ tmr_delay_ticks);

	TIMER_RouteCCSet(bled_p->hw_tmr.base, bled_p->hw_tmr.ch,
		bled_p->hw_tmr.route);

	return 0;
}
