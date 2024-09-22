/*******************************************************************************
*                                                                              *
*   Description: Debug configuration                                           *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef DBGCFG_H
#define DBGCFG_H

#include "eminc.h"
#include <SEGGER_RTT.h>

/**
 * @brief Check if debug probe is active
 *
 * Use Cortex-M specific register to determine if debug probe is active.
 * This can be used to conditionally enable debug output.
 *
 * @return true if debug probe is active
 */
static inline bool dbg_probe_is_active(void) {
	return (CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk);
}

/**
 * @brief Print formatted string to debug output
 *
 * Print formatted string to debug output if debug probe is active.
 *
 * @param fmt format string
 * @param ... arguments
 */
static inline void dbg_printf(const char* fmt, ...) {
	va_list args;
	if (dbg_probe_is_active()){
		va_start(args, fmt);
		SEGGER_RTT_vprintf(0, fmt, &args);
		va_end(args);
	}
}

/**
 * @brief Print string to debug output
 *
 * Print string to debug output if debug probe is active.
 *
 * @param str string
 */
static inline void dbg_print(const char* str) {
	if (dbg_probe_is_active())
		SEGGER_RTT_WriteString(0, str);
}

#define DBG_RSTCAUSE_PRINT(bitf, mask)\
	do {\
		if((bitf)&(mask)) {\
			dbg_printf(" %s", #mask);\
		}\
	} while(0)

static inline void dbg_print_reset_cause(uint32_t bitf) {
	if (bitf) {
		dbg_print("Reset:\n");
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_PORST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_AVDDBOD);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_DVDDBOD);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_DECBOD);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_EXTRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_LOCKUPRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_SYSREQRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_WDOGRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_EM4RST);
		dbg_print("\n");
	}
}

#endif /* DBGCFG_H */

