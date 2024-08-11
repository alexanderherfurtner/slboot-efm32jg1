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

#define DBG_PRINTF(fmt, ...)\
	do {\
		SEGGER_RTT_printf(0, fmt, ##__VA_ARGS__);\
	} while (0)

#define DBG_RSTCAUSE_PRINT(bitf, mask)\
	do {\
		if((bitf)&(mask)) {\
			DBG_PRINTF(" %s", #mask);\
		}\
	} while(0)

static inline void dbg_print_reset_cause(uint32_t bitf) {
	if (bitf) {
		DBG_PRINTF("Reset:");
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_PORST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_AVDDBOD);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_DVDDBOD);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_DECBOD);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_EXTRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_LOCKUPRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_SYSREQRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_WDOGRST);
		DBG_RSTCAUSE_PRINT(bitf, RMU_RSTCAUSE_EM4RST);
		DBG_PRINTF("\n");
	}
}

#endif /* DBGCFG_H */

