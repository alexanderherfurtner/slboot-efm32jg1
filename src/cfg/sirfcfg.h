/*******************************************************************************
*                                                                              *
*   Description: Smart Label Bootloader Radio driver configuration             *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef SICFG_H
#define SICFG_H

////////////////////////////////////////////////////////////////////////////////
/**
 * \def SIRF_CONFIG_SLEEP
 * 
 * \details Enable sleep mode. If enabled the radio will enter sleep mode after
 * each operation.
 */
//#define SIRF_CONFIG_SLEEP

/**
 * \def SIRF_CONFIG_IRQ
 * 
 * \details Enable IRQ handling. If enabled the radio driver will use the IRQ
 * handling, otherwise the driver will use polling.
 */
#define SIRF_CONFIG_IRQ

/**
 * \def SIRF_CONFIG_RX_RSSI
 * 
 * \details Enable RSSI measurement. If enabled the RSSI value will be aquired
 * after each receive operation and provided to the user via the event callback.
 */
//#define SIRF_CONFIG_RX_RSSI

/**
 * \def SIRF_CONFIG_RF_EVT_TS
 * 
 * \details Enable radio event timestamping. If enabled the driver will timestamp
 * each radio event.
 */
//#define SIRF_CONFIG_RF_EVT_TS

/**
 * \def SIRF_CONFIG_HW_ADDR
 * 
 * \details Enable hardware address matching. If enabled the radio will use the
 * hardware address matching engine.
 */
//#define SIRF_CONFIG_HW_ADDR

/**
 * \def SIRF_CONFIG_STATS
 * 
 * \details Enable statistics. If enabled the driver will provide statistics
 * about the radio operation.
 */
//#define SIRF_CONFIG_STATS

////////////////////////////////////////////////////////////////////////////////

/**
 * \def SIRF_CONFIG_DEBUG
 * 
 * \details Enable debug output. If enabled the driver will provide debug output
 * via the debug/log interface
 * 
 * \note See sirf header for debug properties
 */
#define SIRF_CONFIG_DEBUG	SIRF_DEBUG_LOG

/**
 * \def SIRF_CONFIG_ASSERT
 * 
 * \details Assert macro. If the expression is false the system will enter an
 * error state.
 * 
 * \param[in] _expr_n Expression to be evaluated
 */
#define SIRF_ASSERT(_expr_n)	(void)0

/**
 * \def SIRF_FATAL_ERROR
 * 
 * \details Fatal error macro. Enter an error state.
 */
#define SIRF_FATAL_ERROR(_msg)	(void)0

/**
 * \def SIRF_CONFIG_TMR_TICK
 * 
 * \details Timestamp macro. Should be used to timestamp events, and provide a
 * resolution of at least 1 ms.
 * 
 * \note This macro is needed if the timestamping is enabled, see \ref SIRF_CONFIG_RF_EVT_TS.
 */
#define SIRF_TMR_TICK()

/**
 * \def SIRF_CONFIG_TMR_OS_START
 * 
 * \details Start timer oneshot operation. Used for timeout handling.
 * 
 * \param[in] _p Timer operation reference
 * \param[in] _to_ms Timeout in milliseconds
 * \param[in] _cb Callback function
 */
#define SIRF_TMR_ONESHOT(_p, _to_ms, _cb) (void)0

/**
 * \def SIRF_TO_STOP
 * 
 * \details Stop timeout.
 * 
 * \param[in] _p Timer operation reference
 */
#define SIRF_TMR_ONESHOT_ABORT(_p)	(void)0

#endif /* SICFG_H */
