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
#ifndef FATAL_H
#define FATAL_H

#include <stdint.h>

/**
 * @brief Fatal error handler for normal context.
 *
 * Call this function when a abnormal condition is detected. It will print the
 * file name, line number and additional information to the debug console and
 * then enter an infinite loop.
 *
 * @param file		file name
 * @param line		line number
 * @param info0		Info 0
 * @param info1		Info 1
 * @param info2		Info 2
 * @param info3		Info 3
 */
extern void fatal_error(const char* file, uint32_t line, uint32_t info0, uint32_t info1,
	uint32_t info2, uint32_t info3);

/**
 * @brief Fatal error handler for interrupt context/exceptions.
 *
 * Call this function when a fatal error occurs in an interrupt context. It needs
 * the IRQ stack frame and the IRQ number as input. It will print the IRQ number
 * and then enter an infinite loop.
 *
 * This function is called by the fatal_error_x macro.
 *
 * @param irq_stackf	Pointer to the IRQ stack frame
 * @param irq_nr		IRQ number
 */
extern void fatal_error_trap(void *irq_stackf, uint32_t hdl_nr);

/**
 * @brief Macro to call the fatal_error_trap function.
 *
 * This macro is used to call the fatal_error_trap function from the interrupt
 * service routines. It will determine the current stack pointer (PSP or MSP)
 * and then call the fatal_error_trap function.
 *
 * @param x		IRQ number
 */
#define fatal_error_x(x) do { \
    __asm volatile ( \
        "TST LR, #4            \n" /* Test if the exception return state is PSP or MSP */ \
        "ITE EQ                \n" /* If-Then-Else instruction */ \
        "MRSEQ R0, MSP         \n" /* If equal (MSP), move MSP to R0 */ \
        "MRSNE R0, PSP         \n" /* If not equal (PSP), move PSP to R0 */ \
        "MOV R1, %0            \n" /* Move x to R1 */ \
        "B fatal_error_trap    \n" /* Branch to FATAL_ERROR_TRAP */ \
        : /* No output operands */ \
        : "r" (x) /* Input operand */ \
        : "r0", "r1" /* Clobbered registers */ \
    ); \
} while (0)

#endif /* FATAL_H */
