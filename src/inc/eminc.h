/*******************************************************************************
*                                                                              *
*   Description: Silabs EM Includes                                            *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef EMINC_H
#define EMINC_H

/*******************************************************************************
 ********************************     EMDRV      *******************************
 ******************************************************************************/
#include "ecode.h"      /* error code */

#include "tempdrv.h"    /* temperature unit */

/*******************************************************************************
 ********************************     EMLIB      *******************************
 ******************************************************************************/
#include "em_version.h" /* emlib version */
#include "em_common.h"  /* emlib common */
#include "em_assert.h"  /* emlib assert */

#include "em_device.h"  /* emgxx device */
#include "em_system.h"  /* emgxx system */
#include "em_chip.h"    /* emgxx chip errata */
#include "em_core.h"    /* emgxx core functions */
#include "em_bus.h"     /* emgxx bus (bit banging) */

#include "em_dbg.h"     /* debug interface */
#include "em_msc.h"     /* memory system controller */
#include "em_rmu.h"     /* reset management unit */
#include "em_emu.h"     /* energy management unit */
#include "em_cmu.h"     /* clock management unit */
#include "em_wdog.h"    /* watchdog */
#include "em_prs.h"     /* peripheral reflex system */
#include "em_gpio.h"    /* general purpose input output unit */

#endif /* EMINC_H */

