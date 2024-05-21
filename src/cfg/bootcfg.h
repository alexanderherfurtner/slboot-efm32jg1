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
#ifndef BOOTCFG_H
#define BOOTCFG_H

/******************************************************************************/
#define BOOT_FW_VERS_STR_LEN    32
#define BOOT_FW_VERS_STR        "v1.0.0-r0"

#define BOOT_FW_COPY_RIGHT_STR  "(c) 2024 Alexander Herfurtner engineering."
/******************************************************************************/
/**
 * Bootloader debug level selection
 * 0: No Debug
 * 1: Debug log enabled
 */
#define BOOT_DEBUG_LEVEL  0

/**
 * @def BOOT_LED1_PORT
 * @brief GPIO port for boot LED 1.
 */
#define BOOT_LED1_PORT   gpioPortF

/**
 * @def BOOT_LED1_PIN
 * @brief GPIO pin for boot LED 1.
 */
#define BOOT_LED1_PIN    4

/**
 * @def BOOT_LED2_PORT
 * @brief GPIO port for boot LED 2.
 */
#define BOOT_LED2_PORT   gpioPortF

/**
 * @def BOOT_LED2_PIN
 * @brief GPIO pin for boot LED 2.
 */
#define BOOT_LED2_PIN    5

#endif /* BOOTCFG_H */
