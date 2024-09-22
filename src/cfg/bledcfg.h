/*******************************************************************************
*                                                                              *
*   Description: Bootloader configuration                                      *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef BLEDCFG_H
#define BLEDCFG_H

#include "eminc.h"
#include "hwcfg.h"

/**
 * @brief Board LEDs
 * 
 * This enumeration defines the board LEDs. BLED_LED_CNT is used to determine the
 * number of board LEDs, and should always be the last element in the enumeration.
 */
typedef enum bled_led_e {
	BLED_LED_BOOT,
	BLED_LED_DATA,
	BLED_LED_CNT, // Number of board LEDs
} bled_led_t;

#endif /* BLEDCFG_H */