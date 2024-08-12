/*******************************************************************************
*                                                                              *
*   Description:  Hardware configuration, declaring some default init params   *
*                 for the Silabs EFM32JG1B mcu drivers.                        *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef HWCFG_H
#define HWCFG_H

#include "stdinc.h"
#include "eminc.h"

#include "bootcfg.h"

/**
 * WDG init config
 */
static inline WDOG_Init_TypeDef const* hw_wdg_config(void) {
	static const WDOG_Init_TypeDef config = {
		.enable = true,
		.debugRun = false,
		.em2Run = false,
		.em3Run = false,
		.em4Block = false,
		.swoscBlock = false,
		.lock = true, /* By default lock the config */
		.clkSel = wdogClkSelULFRCO, /* 1 kHz */
		.perSel = wdogPeriod_129, /* 129 ms */
		.warnSel = wdogWarnDisable,
		.winSel = wdogIllegalWindowDisable,
		.resetDisable = true,
	};

	return &config;
}

/**
 * EMU em4 init config
 */
static inline EMU_EM4Init_TypeDef const* hw_emu_em4_config(void) {
	static const EMU_EM4Init_TypeDef config =
	{
		.retainLfxo   = false,
		.retainLfrco  = false,
		.retainUlfrco = false,
		.em4State     = emuEM4Shutoff,
		.pinRetentionMode = emuPinRetentionEm4Exit
	};

	return &config;
}

/**
 * EMU em23 init config
 */
static inline EMU_EM23Init_TypeDef const* hw_emu_em23_config(void) {
	static const EMU_EM23Init_TypeDef config =
	{
		.em23VregFullEn   = false,
	};

	return &config;
}

/**
 * Osczillator configuration
 */
typedef struct hw_osc_cfg_s {
	CMU_Osc_TypeDef osc;
	bool enable;
	bool waitup;
} hw_osc_cfg_t;

static inline const hw_osc_cfg_t* hw_osc_config(void) {
	/* The system use the internal RC High Frequency Oscillator for the
	 * main system and a external low frequency crystal for time sensitive
	 * processes using the peripherals, such as timers.
	 *
	 * The rest is not needed and not active.
	 *
	 * Note: Consider the order of the oscillators configs */
	static const hw_osc_cfg_t config[] =
	{
			{
				.osc = cmuOsc_HFRCO,
				.enable = true,
				.waitup = true
			},
			{
				.osc = cmuOsc_LFXO,
				.enable = true,
				.waitup = true
			},
			{
				.osc = cmuOsc_HFXO,
				.enable = false
			},
			{
				.osc = cmuOsc_AUXHFRCO,
				.enable = false
			},
			{
				.osc = cmuOsc_LFRCO,
				.enable = false
			},
			/* ULFRCO is alywas enabled, so remove it from here
			   to avoid an assert in em_cmu.
			{
				.osc = cmuOsc_ULFRCO,
				.enable = false
			},*/
			{
				.osc = (CMU_Osc_TypeDef)-1
			}
	};

	return config;
}

typedef struct hw_clk_sel_cfg_s {
	CMU_Clock_TypeDef  clk;
	CMU_Select_TypeDef sel;
} hw_clk_sel_cfg_t;

static inline const hw_clk_sel_cfg_t* hw_clk_sel_config(void) {
	static const hw_clk_sel_cfg_t config[] =
	{
		{
			.clk = cmuClock_HF,
			.sel = cmuSelect_HFRCO
		},
		{
			.clk = cmuClock_LFA,
			.sel = cmuSelect_LFXO
		},
		{
			.clk = cmuClock_LFB,
			.sel = cmuSelect_LFXO
		},
		{
			.clk = (CMU_Clock_TypeDef)-1
		}
	};

	return config;
}

typedef struct hw_clk_div_cfg_s {
	CMU_Clock_TypeDef  clk;
	CMU_ClkDiv_TypeDef div;
} hw_clk_div_cfg_t;

static inline const hw_clk_div_cfg_t* hw_clk_div_config(void) {
	static const hw_clk_div_cfg_t config[] =
	{
		{
			.clk = cmuClock_HF,
			.div = cmuClkDiv_1
		},
		{
			.clk = cmuClock_HFPER,
			.div = cmuClkDiv_1
		},
		{
			.clk = cmuClock_CORE,
			.div = cmuClkDiv_1
		},
		{
			.clk = cmuClock_LEUART0,
			.div = cmuClkDiv_1
		},
		{
			.clk = (CMU_Clock_TypeDef)-1
		}
	};

	return config;
}

typedef struct hw_clk_peri_cfg_s {
	CMU_Clock_TypeDef  clk;
	bool enable;
} hw_clk_peri_cfg_t;

static inline const hw_clk_peri_cfg_t* hw_clk_peri_config(void) {
	static const hw_clk_peri_cfg_t config[] =
	{
		{
			.clk = cmuClock_GPIO,
			.enable = true
		},
		{
			.clk = cmuClock_HFLE, /* Low Energy peri clock (incl. WDOG) */
			.enable = true
		},
		{
			.clk = (CMU_Clock_TypeDef)-1
		}
	};

	return config;
}

typedef struct hw_gpio_cfg_s {
	GPIO_Port_TypeDef port;
	uint8_t pin;
	GPIO_Mode_TypeDef mode;
	uint8_t out;
} hw_gpio_cfg_t;

static inline const hw_gpio_cfg_t* hw_gpio_config(void) {
	static const hw_gpio_cfg_t config[] =
	{
#if defined(BOOT_LED1_PORT)
		{
			.port = BOOT_LED1_PORT,
			.pin = BOOT_LED1_PIN,
			.mode = gpioModePushPull,
			.out = 0
		},
#endif
#if defined(BOOT_LED2_PORT)
		{
			.port = BOOT_LED2_PORT,
			.pin = BOOT_LED2_PIN,
			.mode = gpioModePushPull,
			.out = 0
		},
#endif
		{
			.port = (GPIO_Port_TypeDef)-1
		}
	};

	return config;
}

#endif /* HWCFG_H */
