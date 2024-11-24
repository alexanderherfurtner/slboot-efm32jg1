/*******************************************************************************
*                                                                              *
*   Description: Smart Label Bootloader Radio driver                           *
*                                                                              *
*   Author:  Alexander Herfurtner (mail@alexanderherfurtner.de)                *
*                                                                              *
*   Copyright (c) 2024 Alexander Herfurtner                                    *
*   All rights reserved.                                                       *
*                                                                              *
*   License: MIT License                                                       *
********************************************************************************/
#ifndef SIRF_H
#define SIRF_H

//////////////////////////////////////////////////////////////////////////////
/**
 * \def SIRF_CONFIG_DEBUG_LOG
 * 
 * \details Enable debug log output.
 * Add to \ref SIRF_CONFIG_DEBUG to enable it.
 */
#define SIRF_DEBUG_LOG	(1 << 0)

/**
 * \def SIRF_CONFIG_DEBUG_IRQ
 * 
 * \details Enable verbose IRQ mode (all radio chip IRQ flags will be handled).
 * Add to \ref SIRF_CONFIG_DEBUG to enable it.
 */
#define SIRF_DEBUG_CHIP_IRQ	(1 << 1)

/**
 * \def SIRF_CONFIG_DEBUG_CHIP_PROPS
 * 
 * \details Enable debug chip properties output.
 * Add to \ref SIRF_CONFIG_DEBUG to enable it.
 */
#define SIRF_DEBUG_CHIP_PROPS	(1 << 2)

/**
 * \def SIRF_DEBUG_CHIP_HW_GPIO
 * 
 * \details Enable debug chip HW GPIO output (RX/TX state reported via GPIO).
 * Add to \ref SIRF_CONFIG_DEBUG to enable it.
 */
#define SIRF_DEBUG_CHIP_HW_GPIO	(1 << 3)

//////////////////////////////////////////////////////////////////////////////

#if defined(SIRF_RF_EVT_TS)
/**
 * \enum sirf_ts_id_e
 * 
 * \details Timestamp IDs, only available if SIRF_CONFIG_RF_EVT_TS is enabled.
 */
enum sirf_ts_id_e {
	SIRF_RTC_TS_ID_RF_EVT,		/** Generic Radio event */
	SIRF_RTC_TS_ID_RX_START,	/** RX start */
	SIRF_RTC_TS_ID_RX_INPROG,	/** RX in progress */
	SIRF_RTC_TS_ID_RX_DONE,		/** RX done */
	SIRF_RTC_TS_ID_TX_START,	/** TX start */
	SIRF_RTC_TS_ID_TX_INPROG,	/** TX in progress */
	SIRF_RTC_TS_ID_TX_DONE,		/** TX done */
	SIRF_RTC_TS_ID_TRX_START,	/** TRX start */
	SIRF_RTC_TS_ID_TRX_DONE,	/** TRX done */

	SIRF_RTC_TS_ID_CNT
};
#endif

/**
 * \enum sirf_fifo_e
 * 
 * \details FIFO reset options
 */
enum sirf_fifo_e {
	SIRF_FIFO_TX_RESET = ( 1 << 0 ),
	SIRF_FIFO_RX_RESET = ( 1 << 1 ),
	SIRF_FIFO_NO_RESET = ( 0 << 0 )
};

/**
 * \enum sirf_ret_e
 * 
 * \details Return values of the Silabs Radio driver
 */
typedef enum sirf_ret_e {
	SIRF_SUCCESS = 0,
	SIRF_ERROR = -1
} sirf_ret_t;

/**
 * \enum sirf_state_e
 *
 * \details State of the radio driver. It should be consitent with the radio chip
 * state.
 * 
 * \note This states are shared with the event callback api. For the future this
 * should be re-worked, seperating the internal states with the event api.
 */
typedef enum sirf_state_e {
	SIRF_STATE_IDLE,	/* The radio drv is in idle */
	SIRF_STATE_RX,		/* The radio drv is in receive state */
	SIRF_STATE_RX_DONE,	/* The radio drv is in receive done state */
	SIRF_STATE_RX_ERR,	/* The radio drv is in receive error state */
	SIRF_STATE_TX,		/* The radio drv is in transmit state */
	SIRF_STATE_TX_DONE,	/* The radio drv is in transmit done state */
	SIRF_STATE_TX_ERR,	/* The radio drv is in transmit error state */
	SIRF_STATE_TRX,		/* The radio drv is in receive/transmit state */
	SIRF_STATE_TRX_ERR,	/* The radio drv is in receive/transmit error state */

	SIRF_STATE_UKNOWN,	/* The radio drv is in unknown error state */
} sirf_state_t;

/**
 * \enum sirf_chip_state_e
 * 
 * \details State of the radio chip. It should be consitent with the radio driver
 * state.
 * 
 * \note Those are used in conjunction with the ezradio driver.
 */
typedef enum sirf_chip_state_e {
	SIRF_CHIP_STATE_SLEEP = 1,	/* Radio sleep state */
	SIRF_CHIP_STATE_BOOT  = 2,	/* Radio boot state */
	SIRF_CHIP_STATE_READY = 3,	/* Radio ready state */
	SIRF_CHIP_STATE_TX = 7,		/* Radio transmit state */
	SIRF_CHIP_STATE_RX = 8		/* Radio receive state */
} sirf_chip_state_t;

/**
 * \typedef event_handle_t
 * 
 * \details Event callback function pointer type.
 */
typedef void (* event_handle_t)(sirf_state_t p_cb_state, void * p_evt_data);

/**
 * \struct sirf_init_para_s
 * 
 * \details Initialization parameters for the Silabs Radio driver.
 */
typedef struct sirf_init_para_s {
	uint8_t rfch;

#if defined(SIRF_CONFIG_HW_ADDR)
	uint8_t   match_en;
	uint16_t  match_addr_bc;
	uint16_t  match_addr;
#endif

	event_handle_t event_cb;
} sirf_init_para_t;

typedef struct sirf_fifo_s {
	uint16_t rx_len;
	uint8_t* rx_buf;
	uint16_t rx_idx;
	uint16_t rx_max;

	uint16_t tx_len;
	uint8_t* tx_buf;
	uint16_t tx_idx;
} sirf_fifo_t;

typedef struct sirf_rx_data_s {
	uint8_t* buf;
	uint8_t  len;
#if defined(SIRF_CONFIG_RX_RSSI)
	uint8_t  rssi;
#endif
} sirf_rx_data_t;

typedef struct sirf_tx_data_s {
	uint8_t* buf;
	uint8_t  len;
} sirf_tx_data_t;

/**
 * \brief Initialize Silabs Radio driver
 *
 * \details This function initializes Silabs Radio driver. It is reasonably re-
 * quired to call this function before any other function of the driver.
 *
 * \param[in] p_sirf_init_para Pointer to the initialization parameters
 * \return sirf_ret_t
 */
extern sirf_ret_t sirf_init(sirf_init_para_t const * p_init_para);

/**
 * \brief Deinitialize Silabs Radio driver
 * 
 * \details This function deinitializes the Silabs Radio driver. It is reasonably
 * required to call this function after the driver is not needed anymore.
 */
extern sirf_ret_t sirf_deinit(void);

/**
 * \brief Silabs Radio driver task
 * 
 * \details This function is the main task of the Silabs Radio driver. It should
 * be called periodically to handle the radio operation. Unless, the driver will
 * use the configuration \ref SIRF_CONFIG_IRQ.
 * 
 * \return sirf_ret_t
 */
extern sirf_ret_t sirf_task(void);

/**
 * \brief Start transmit operation
 * 
 * \details This function starts the transmit operation of a packet.
 * 
 * \param[in] p_tx_buf Pointer to the packet buffer
 * \param[in] p_buf_len Length of the packet
 * \return sirf_ret_t
 */
extern sirf_ret_t sirf_packet_tx(uint8_t* p_tx_buf, uint16_t  p_buf_len);

/**
 * \brief Start receive operation
 *
 * \details This function starts the receive operation.
 *
 * \param[in] p_rx_buf Pointer to the packet buffer
 * \param[in] p_rx_buf_len Length of the packet
 * \param[in] p_rx_to_ms Timeout in milliseconds
 * \return sirf_ret_t 
 */
extern sirf_ret_t sirf_packet_rx(uint8_t* p_rx_buf, uint16_t  p_rx_buf_len,
	uint16_t  p_rx_to_ms);

/**
 * \brief Start transmit and put the radio into receive operation
 * 
 * \details See sirf_packet_rx and sirf_packet_tx. The receive operation is
 * started immediately after the transmit operation is done.
 * 
 * \param[in] p_tx_buf Pointer to the packet buffer
 * \param[in] p_rx_buf Pointer to the packet buffer
 * \param[in] p_tx_buf_len Length of the packet
 * \param[in] p_rx_buf_len Length of the packet
 * \param[in] p_rx_to_ms Timeout in milliseconds
 * \return sirf_ret_t
 */
extern sirf_ret_t sirf_packet_trx(uint8_t * p_tx_buf, uint16_t p_tx_buf_len,
	uint8_t * p_rx_buf, uint16_t p_rx_buf_len, uint16_t p_rx_to_ms);

/**
 * \brief Stop any ongoing receive or transmit operation
 * 
 * \details This function stops any ongoing receive or transmit operation and
 * puts the radio driver into idle state.
 * 
 * \return void
 */
extern void sirf_packet_trx_stop(void);

#endif /* SIRF_H */
