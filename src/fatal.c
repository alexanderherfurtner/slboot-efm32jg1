/*******************************************************************************
*                                                                              *
*   Description: Fatal error handling                                          *
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

#include "assert.h"
// TODO: ASSERT handling
/******************************************************************************/
struct __attribute__((packed)) irq_stackf_s {
	uint32_t r0;
	uint32_t r1;
	uint32_t r2;
	uint32_t r3;
	uint32_t r12;
	uint32_t lr;
	uint32_t pc;
	uint32_t xpsr;
};

/******************************************************************************/
static void fatal_error_dead() __attribute__((noreturn));

/******************************************************************************/
void fatal_error(const char* file, uint32_t line, uint32_t info0, uint32_t info1,
	uint32_t info2, uint32_t info3) {
	/* Disable all interrupts */
	__disable_irq();

	DBG_PRINTF("Fatal Error\n");
	DBG_PRINTF("File:  %s\n", file);
	DBG_PRINTF("Line:  %ld\n", line);
	DBG_PRINTF("Info0: %08lx\n", info0);
	DBG_PRINTF("Info1: %08lx\n", info1);
	DBG_PRINTF("Info2: %08lx\n", info2);
	DBG_PRINTF("Info3: %08lx\n", info3);

	/* end of the road */
	fatal_error_dead();
}

void __attribute__((optimize("O0"))) fatal_error_trap(void *irq_stackf, uint32_t hdl_nr) {
	struct irq_stackf_s* stackf_p = (struct irq_stackf_s *)irq_stackf;

	/* Disable all interrupts */
	__disable_irq();

	DBG_PRINTF("Fatal Error Trap\n");
	DBG_PRINTF("Hdl:  %ld\n", hdl_nr);
	DBG_PRINTF("Sp:   %p\n", stackf_p);
	DBG_PRINTF("r0:   %08lx\n", stackf_p->r0);
	DBG_PRINTF("r1:   %08lx\n", stackf_p->r1);
	DBG_PRINTF("r2:   %08lx\n", stackf_p->r2);
	DBG_PRINTF("r3:   %08lx\n", stackf_p->r3);
	DBG_PRINTF("r12:  %08lx\n", stackf_p->r12);
	DBG_PRINTF("lr:   %08lx\n", stackf_p->lr);
	DBG_PRINTF("pc:   %08lx\n", stackf_p->pc);
	DBG_PRINTF("xpsr: %08lx\n", stackf_p->xpsr);

	/* end of the road */
	fatal_error_dead();
}

#if defined(DEBUG_EFM) && defined(DEBUG_EFM_USER)
/**
 * @brief EFM/EM library assert overwrite
 *
 * This function overwrites the standard EFM assert function, please see
 * libs/emlib/inc/em_assert.h for more details.
 *
 * @param file The file name where the assert was called
 * @param line The line number where the assert was called
 */
void assertEFM(const char* file, int line) {
	/* Call fatal error */
	fatal_error(file, line, 0, 0, 0, 0);
}
#endif

/******************************************************************************/
/**
 * @brief Fatal error dead hander
 *
 * End of the road for the system. This function is called when a fatal error
 * is detected. It will disable all interrupts and toggle the LEDs in an
 * infinite loop.
 *
 * No return from this function.
 */
static void fatal_error_dead(void) {
	/* Indicate Fatal error */
	GPIO_PinOutSet(BOOT_LED1_PORT, BOOT_LED1_PIN);
	GPIO_PinOutSet(BOOT_LED2_PORT, BOOT_LED2_PIN);

	for(;;) {
		uint32_t delay = 1000000UL;
		/* Toggle all LEDs to indicate the fatal error */
		GPIO_PinOutToggle(BOOT_LED1_PORT, BOOT_LED1_PIN);
		GPIO_PinOutToggle(BOOT_LED2_PORT, BOOT_LED2_PIN);
		while (delay--);
	}
}
