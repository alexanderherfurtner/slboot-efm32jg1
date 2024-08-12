/*******************************************************************************
*                                                                              *
*   Description: Smart Label Bootloader main                                   *
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

#include "hwcfg.h"
#include "dbgcfg.h"
#include "bootcfg.h"

#include "fatal.h"
/******************************************************************************/
typedef struct sys_info_s {
	uint32_t reset_cause;
} sys_info_t;

sys_info_t g_sys_info;

/******************************************************************************/
static void sys_init(sys_info_t* sys_info);
static void sys_info(sys_info_t* sys_info);

/******************************************************************************/
void NMI_Handler			(void) { fatal_error_x(0x01); }
void HardFault_Handler		(void) { fatal_error_x(0x02); }
void MemManage_Handler		(void) { fatal_error_x(0x03); }
void BusFault_Handler		(void) { fatal_error_x(0x04); }
void UsageFault_Handler		(void) { fatal_error_x(0x05); }
void SVC_Handler			(void) { fatal_error_x(0x06); }
void DebugMon_Handler		(void) { fatal_error_x(0x07); }
void PendSV_Handler			(void) { fatal_error_x(0x08); }
void SysTick_Handler		(void) { fatal_error_x(0x09); }
/*void EMU_IRQHandler		(void) { fatal_error_x(0x0a); }*/
void WDOG0_IRQHandler		(void) { fatal_error_x(0x0b); }
void LDMA_IRQHandler		(void) { fatal_error_x(0x0c); }
void GPIO_EVEN_IRQHandler	(void) { fatal_error_x(0x0d); }
void TIMER0_IRQHandler		(void) { fatal_error_x(0x0e); }
void USART0_RX_IRQHandler	(void) { fatal_error_x(0x0f); }
void USART0_TX_IRQHandler	(void) { fatal_error_x(0x10); }
void ACMP0_IRQHandler		(void) { fatal_error_x(0x11); }
void ADC0_IRQHandler		(void) { fatal_error_x(0x12); }
void IDAC0_IRQHandler		(void) { fatal_error_x(0x13); }
void I2C0_IRQHandler		(void) { fatal_error_x(0x14); }
void GPIO_ODD_IRQHandler	(void) { fatal_error_x(0x15); }
void TIMER1_IRQHandler		(void) { fatal_error_x(0x16); }
void USART1_RX_IRQHandler	(void) { fatal_error_x(0x17); }
void USART1_TX_IRQHandler	(void) { fatal_error_x(0x18); }
void LEUART0_IRQHandler		(void) { fatal_error_x(0x19); }
void PCNT0_IRQHandler		(void) { fatal_error_x(0x1a); }
void CMU_IRQHandler			(void) { fatal_error_x(0x1b); }
void MSC_IRQHandler			(void) { fatal_error_x(0x1c); }
void CRYPTO_IRQHandler		(void) { fatal_error_x(0x1d); }
void LETIMER0_IRQHandler	(void) { fatal_error_x(0x1e); }
void RTCC_IRQHandler		(void) { fatal_error_x(0x1f); }
void CRYOTIMER_IRQHandler	(void) { fatal_error_x(0x20); }

/******************************************************************************/
/**
 * @brief Entry point of the program.
 *
 * This function initializes the system and toggles the state of a GPIO pin in an infinite loop.
 * It also calls the `sys_info` function to display system information.
 *
 * @return 0 indicating successful program execution.
 */
int main(void) {
	uint32_t loop;
	sys_init(&g_sys_info);
	sys_info(&g_sys_info);

	GPIO_PinOutSet(BOOT_LED1_PORT, BOOT_LED1_PIN);
	GPIO_PinOutClear(BOOT_LED2_PORT, BOOT_LED2_PIN);

	loop = 1000000;
	for (;;) {
		WDOG_Feed();
		loop--;
		if (loop == 0) {
			GPIO_PinOutToggle(BOOT_LED1_PORT, BOOT_LED1_PIN);
			GPIO_PinOutToggle(BOOT_LED2_PORT, BOOT_LED2_PIN);
			loop = 1000000;
		}
	}

	return 0;
}

/******************************************************************************/
/**
 * @brief Initializes the system.
 *
 * This function configures various system components such as the chip, reset unit, watchdog,
 * power management, oscillators, clock selection, clock prescaler, peripheral clock, and LED1.
 * It also handles the erratum related to EMU_E210/216.
 *
 * @param sys_info Pointer to the sys_info_t structure to store system information.
 */
static void sys_init(sys_info_t* sys_info) {
	/* Configure chip */
	CHIP_Init();

	/* Configure Reset Unit */
	sys_info->reset_cause = RMU_ResetCauseGet();
	RMU_ResetCauseClear();
	RMU_ResetControl(rmuResetLockUp, rmuResetModeFull);

	/* Configure EMU2/3/4 Power Management */
	EMU_EM23Init(hw_emu_em23_config());
	EMU_EM4Init(hw_emu_em4_config());

	/* Set HFRCO clock band to 38 MHz (max.) */
	CMU_HFRCOBandSet(CMU_HFRCO_MAX);

	/* Configure system oscillators */
	for (hw_osc_cfg_t const* hw_osc_p = hw_osc_config();
		hw_osc_p->osc != (CMU_Osc_TypeDef)-1; hw_osc_p++) {
		CMU_OscillatorEnable(hw_osc_p->osc, hw_osc_p->enable, hw_osc_p->waitup);
	}

	/* Configure clock selection */
	for (hw_clk_sel_cfg_t const* hw_clk_sel_p = hw_clk_sel_config();
		hw_clk_sel_p->clk != (CMU_Clock_TypeDef)-1; hw_clk_sel_p++) {
		CMU_ClockSelectSet(hw_clk_sel_p->clk, hw_clk_sel_p->sel);
	}

	/* Configure clock presecaler */
	for (hw_clk_div_cfg_t const* hw_clk_div_p = hw_clk_div_config();
		hw_clk_div_p->clk != (CMU_Clock_TypeDef)-1; hw_clk_div_p++) {
		CMU_ClockDivSet(hw_clk_div_p->clk, hw_clk_div_p->div);
	}

	/* Configure peri clocks */
	for (hw_clk_peri_cfg_t const* hw_clk_peri_p = hw_clk_peri_config();
		hw_clk_peri_p->clk != (CMU_Clock_TypeDef)-1; hw_clk_peri_p++) {
		CMU_ClockEnable(hw_clk_peri_p->clk, hw_clk_peri_p->enable);
	}

	/* Configure GPIOs */
	for (hw_gpio_cfg_t const* hw_gpio_p = hw_gpio_config();
		hw_gpio_p->port != (GPIO_Port_TypeDef)-1; hw_gpio_p++) {
		GPIO_PinModeSet(hw_gpio_p->port, hw_gpio_p->pin, hw_gpio_p->mode, hw_gpio_p->out);
	}

	/* Erratum: EMU_E210/216 */
	TEMPDRV_Init();

	/* Segger RTT Init */
	SEGGER_RTT_Init();

	/* Configure Watchdog */
	WDOGn_Init(WDOG0, hw_wdg_config());
	NVIC_EnableIRQ(WDOG0_IRQn);

	/* Enable global interrupts */
	CORE_CriticalEnableIrq();
}

/**
 * @brief Retrieves system information.
 *
 * This function retrieves system information and stores it in the provided
 * `sys_info` structure.
 *
 * @param sys_info Pointer to the `sys_info_t` structure to store the system information.
 */
static void sys_info(sys_info_t* sys_info) {
	/* print the reset cause */
	dbg_print_reset_cause(sys_info->reset_cause);
}
