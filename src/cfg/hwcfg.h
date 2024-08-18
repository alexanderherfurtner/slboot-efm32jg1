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
		.intSel = wdogIntTout, /* Enable Timeout Interrupt */
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

typedef struct hw_gpio_port_cfg_s {
	GPIO_Port_TypeDef port;
	uint32_t driveStrength;
	uint32_t slewrate;
	uint32_t slewrateAlt;
	bool dindis;
	bool dindisAlt;
} hw_gpio_port_cfg_t;

/**
 * Hardware GPIO port configuration.
 *
 * The chip supports up to two configuraton for each port. The default
 * is used for the normal operation and the alternate for the alternate
 * function.
 *
 * The following are configured:
 * - Drive strength (including alternate)
 * - Slewrate
 * - SlewrateAlt
 * - Dindis
 * - DindisAlt
 *
 * The pin configuration can select the port configuration (normal or
 * alternate) accordingly.
 *
 * The configuration is done for all ports.
 *
 * @return Pointer to the configuration array.
 */
static inline const hw_gpio_port_cfg_t* hw_gpio_port_config(void) {
	static const hw_gpio_port_cfg_t config [] =
	{
		{
			.port = gpioPortA,
			.driveStrength = gpioDriveStrengthStrongAlternateStrong,
			.slewrate = GPIO_Slewrate(GPIO_SlewrateDefault),
			.slewrateAlt = GPIO_SlewrateAlt(GPIO_SlewrateDefault),
			.dindis = false,
			.dindisAlt = true,
		},
		{
			.port = gpioPortB,
			.driveStrength = gpioDriveStrengthStrongAlternateStrong,
			.slewrate = GPIO_Slewrate(GPIO_SlewrateDefault),
			.slewrateAlt = GPIO_SlewrateAlt(GPIO_SlewrateDefault),
			.dindis = false,
			.dindisAlt = false,
		},
		{
			.port = gpioPortC,
			.driveStrength = gpioDriveStrengthStrongAlternateStrong,
			.slewrate = GPIO_Slewrate(GPIO_SlewrateDefault),
			.slewrateAlt = GPIO_SlewrateAlt(GPIO_SlewrateDefault),
			.dindis = false,
			.dindisAlt = false,
		},
		{
			.port = gpioPortD,
			.driveStrength = gpioDriveStrengthStrongAlternateStrong,
			.slewrate = GPIO_Slewrate(GPIO_SlewrateDefault),
			.slewrateAlt = GPIO_SlewrateAlt(GPIO_SlewrateDefault),
			.dindis = false,
			.dindisAlt = false,
		},
		{
			.port = gpioPortF,
			.driveStrength = gpioDriveStrengthStrongAlternateStrong,
			.slewrate = GPIO_Slewrate(GPIO_SlewrateDefault),
			.slewrateAlt = GPIO_SlewrateAlt(GPIO_SlewrateDefault),
			.dindis = false,
			.dindisAlt = false,
		},
		{
			.port = (GPIO_Port_TypeDef)-1
		}
	};

	return config;
}

typedef struct hw_gpio_pin_cfg_s {
	GPIO_Port_TypeDef port;
	uint8_t pin;
	GPIO_Mode_TypeDef mode;
	uint8_t out;
} hw_gpio_pin_cfg_t;

static inline const hw_gpio_pin_cfg_t* hw_gpio_pin_config(void) {
	static const hw_gpio_pin_cfg_t config[] =
	{
		/* Port A */
		{
			.port = gpioPortA,
			.pin = 0,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		{
			.port = gpioPortA,
			.pin = 1,
			.mode = gpioModeDisabled,
			.out = 0,
		},
#if defined(BOOT_CONFIG_DBG_UART)
		{ /* Debug UART TX */
			.port = gpioPortA,
			.pin = 2,
			.mode = gpioModePushPullAlt,
			.out = 0,
		},
		{ /* Debug UART RX */
			.port = gpioPortA,
			.pin = 3,
			.mode = gpioModeInputPull,
			.out = 1, /* Pull-up */
		},
#else
		{
			.port = gpioPortA,
			.pin = 2,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		{
			.port = gpioPortA,
			.pin = 3,
			.mode = gpioModeDisabled,
			.out = 0,
		},
#endif
		{
			.port = gpioPortA,
			.pin = 4,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		{
			.port = gpioPortA,
			.pin = 5,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		/* Port B */
#if defined(BOOT_CONFIG_BTN1)
		{
			.port = gpioPortB,
			.pin = 11,
			.mode = INPUTPULLFILTER,
			.out = 1, /* Pull-Up */
		},
#else
		{
			.port = gpioPortB,
			.pin = 11,
			.mode = gpioModeDisabled,
			.out = 0,
		},
#endif
		{
			.port = gpioPortB,
			.pin = 12,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		{
			.port = gpioPortB,
			.pin = 13,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		{ /* LFXTAL_N */
			.port = gpioPortB,
			.pin = 14,
			.mode = gpioModeDisabled,
			.out = 0,
		},
		{ /* LFXTAL_P */
			.port = gpioPortB,
			.pin = 15,
			.mode = gpioModeDisabled,
			.out = 0,
		},
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
