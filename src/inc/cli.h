/*******************************************************************************
*                                                                              *
*   Description: Smart Label Bootloader CLI                                    *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef CLI_H
#define CLI_H
/*******************************************************************************/

/**
 * @brief Initialize CLI
 * 
 * This function initializes the CLI and should be called once at startup.
 */
void cli_init(void);

/**
 * @brief CLI task
 * 
 * This function should be called periodically to process CLI commands.
 */
void cli_task(void);

/*******************************************************************************/
#endif // CLI_H
