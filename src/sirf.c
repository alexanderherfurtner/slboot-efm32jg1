/*******************************************************************************
*                                                                              *
*   Description: Silabs Radio interface driver                                 *
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

#include "ezradiodrv_config.h"
#include "ezradio_cmd.h"
#include "ezradio_prop.h"
#include "ezradio_comm.h"
#include "ezradio_hal.h"
#include "ezradio_api_lib.h"
#include "ezradio_api_lib_add.h"

#include "sirfcfg.h"
#include "sirf.h"

#define __MODULE__ "SIRF"

////////////////////////////////////////////////////////////////////////////////
/**
 * \def SIRF_CHIP_FIFO_RX_SIZE
 * 
 * \details RX FIFO size - do not change.
 */
#define SIRF_CHIP_FIFO_RX_SIZE	64

/**
 * \def SIRF_CHIP_FIFO_TX_SIZE
 * 
 * \details TX FIFO size - do not change.
 */
#define SIRF_CHIP_FIFO_TX_SIZE	64

/**
 * \def SIRF_CHIP_IRQ_HANDLE_CNT
 * 
 * \details Number of IRQ handlers - do not change.
 */
#define SIRF_CHIP_IRQ_HANDLE_CNT	3

/**
 * \def SIRF_CHIP_CMD_RX_OPT_LATCH
 * 
 * \details Use the given RX configuration and enter RX state.
 */
#define SIRF_CHIP_CMD_RX_OPT_USE	(0 << 3)

/**
 * \def SIRF_CHIP_CMD_RX_OPT_UPD
 * 
 * \details Just update the RX configuration, but do not enter RX state. Will
 * be used by the subsequent packet.
 */
#define SIRF_CHIP_CMD_RX_OPT_UPD	(1 << 3)

/**
 * \def SIRF_CHIP_CMD_RX_OPT_START_IMD
 * 
 * \details Start RX immediately once RX configuration is active.
 */
#define SIRF_CHIP_CMD_RX_OPT_START_IMD	(0 << 0)

/**
 * \def SIRF_CHIP_CMD_RX_OPT_START_WUT
 * 
 * \details Start RX on Wake up timer expieres.
 */
#define SIRF_CHIP_CMD_RX_OPT_START_WUT	(1 << 0)

/**
 * \def SIRF_CHIP_CMD_RX_OPT_LEN_AUTO
 * 
 * \details 0: length is embedded in packet
 */
#define SIRF_CHIP_CMD_RX_OPT_LEN_AUTO	0

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE1_REMAIN
 *
 * \details Next state on preamble timeout: Remain (RX state)
 */
#define SIRF_CHIP_CMD_RX_NSTATE1_REMAIN	0

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE2_SLEEP
 *
 * \details Next state on valid reciption of a packet: Sleep/Standby
 */
#define SIRF_CHIP_CMD_RX_NSTATE2_SLEEP	1

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE2_READY
 *
 * \details Next state on valid reciption of a packet: Ready
 */
#define SIRF_CHIP_CMD_RX_NSTATE2_READY	3

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE3_SLEEP
 *
 * \details: Next state on invalid reciption of a packet: Sleep/Standby
 */
#define SIRF_CHIP_CMD_RX_NSTATE3_SLEEP	1

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE3_READY
 *
 * \details: Next state on invalid reciption of a packet: Ready
 */
#define SIRF_CHIP_CMD_RX_NSTATE3_READY	3

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE3_RETRY
 *
 * \details: Next state on invalid reciption of a packet: Retry
 */
#define SIRF_CHIP_CMD_RX_NSTATE3_RETRY	8

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE1_DFT
 * 
 * \details Default for argument State1 of the RX command.
 * State1: RX state after preamble timeout
 */
#define SIRF_CHIP_CMD_RX_NSTATE1_DFT	SIRF_CHIP_CMD_RX_NSTATE1_REMAIN

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE2_DFT
 * 
 * \details Default for argument State2 of the RX command.
 * State2: RX state after valid reciption. This also applies even if \ref
 * SIRF_CONFIG_SLEEP is configured as it will be controlled by the driver.
 */
#define SIRF_CHIP_CMD_RX_NSTATE2_DFT	SIRF_CHIP_CMD_RX_NSTATE2_READY

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE3_DFT
 * 
 * \details Default for argument State3 of the RX command.
 * State1: RX state after invalid reciption.
 */
#define SIRF_CHIP_CMD_RX_NSTATE3_DFT	SIRF_CHIP_CMD_RX_NSTATE3_READY

/**
 * \def SIRF_CHIP_CMD_RX_NSTATE1_3
 * 
 * \details Default arguments State1 to State3 for RX command.
 */
#define SIRF_CHIP_CMD_RX_NSTATE1_3 \
	SIRF_CHIP_CMD_RX_NSTATE1_DFT, \
	SIRF_CHIP_CMD_RX_NSTATE2_DFT, \
	SIRF_CHIP_CMD_RX_NSTATE3_DFT

/**
 * \def  SIRF_CHIP_CMD_TX_NSTATE_SLEEP
 * 
 * \details Next state after TX complete: Sleep/Standby
 */
#define SIRF_CHIP_CMD_TX_NSTATE_SLEEP	(1 << 4)

/**
 * \def  SIRF_CHIP_CMD_TX_NSTATE_READY
 * 
 * \details Next state after TX complete: Ready
 */
#define SIRF_CHIP_CMD_TX_NSTATE_READY	(3 << 4)

/**
 * \def  SIRF_CHIP_CMD_TX_NSTATE_RX
 * 
 * \details Next state after TX complete: RX
 */
#define SIRF_CHIP_CMD_TX_NSTATE_RX	(8 << 4)

/**
 * \def SIRF_CHIP_INT_PH_MASK_SET
 * 
 * \details Set the PH interrupt mask
 */
#define SIRF_CHIP_INT_PH_MASK_CLR		0

/**
 * \def SIRF_CHIP_INT_MODEM_MASK_CLR
 * 
 * \details Set the PH interrupt mask
 */
#define SIRF_CHIP_INT_MODEM_MASK_CLR	0

/**
 * \def SIRF_CHIP_INT_CHIP_MASK_CLR
 * 
 * \details Set the PH interrupt mask
 */
#define SIRF_CHIP_INT_MODEM_MASK_CLR	0

/**
 * \def SIRF_CHIP_INT_ARG_CLR_ALL
 * 
 * \details Set the PH interrupt mask
 */
#define SIRF_CHIP_INT_MASK_CLR \
	SIRF_CHIP_INT_PH_MASK_CLR, \
	SIRF_CHIP_INT_MODEM_MASK_CLR, \
	SIRF_CHIP_INT_MODEM_MASK_CLR

////////////////////////////////////////////////////////////////////////////////
#if defined(SIRF_CONFIG_SLEEP)
	#define SIRF_SLEEP_ENTER()	sirf_sleep_enter()
	#define SIRF_SLEEP_EXIT()	sirf_sleep_exit()
#else
	#define SIRF_SLEEP_ENTER()
	#define SIRF_SLEEP_EXIT()
#endif

#if defined(SIRF_CONFIG_RF_EVT_TS)
	#define SIRF_TS_TRACE(_ts) \
		sirf_data.ts_event[_ts] = SIRF_TMR_TICK()

	#define SIRF_TS_TRACE_RF(_ts_id) \
		sirf_data.ts_event[_ts] = sirf_data.ts_event[SIRF_RTC_TS_ID_RF_EVT]
#else
	#define SIRF_TS_TRACE(_ts)		(void)0
	#define SIRF_TS_TRACE_RF(_ts)	(void)0
#endif

#if defined(SIRF_CONFIG_HW_ADDR)
	#define SIRF_MATCH_EN	(1 << 6)
	#define SIRF_MATCH_OR	(1 << 6)
	#define SIRF_MATCH_AND	(0 << 6)
#endif

/**
 * \def SIRF_TMR_TIMEOUT_RX_START
 * 
 * \details Start RX timeout timer
 * 
 * \param[in] _to_ms Timeout in milliseconds
 * \param[in] _cb Callback function
 */
#define SIRF_TMR_TIMEOUT_RX_START(_to_ms, _cb) SIRF_TMR_ONESHOT(&sirf_data, _to_ms, _cb)

/**
 * \def SIRF_RX_TO_ABORT
 * 
 * \details Abort RX timeout timer
 */
#define SIRF_TMR_TIMEOUT_RX_ABORT() SIRF_TMR_ONESHOT_ABORT(&sirf_data)

/**s
 * \enum sirf_chip_event_e
 *
 * \brief Only for internal use. Should be placed inside the sirf.c file.
 * Do not use for the event API. This is misleading.
 */
typedef enum sirf_chip_event_e {
	SIRF_EVENT_RX_DONE = 1,
	SIRF_EVENT_TX_DONE,
	SIRF_EVENT_ERR_CMD,
	SIRF_EVENT_ERR_FIFO,
	SIRF_EVENT_ERR_TIMEOUT,
	SIRF_EVENT_ERR_CRC,
	SIRF_EVENT_ERR_SYNC,
	SIRF_EVENT_ERR_SUPR
} sirf_chip_event_t;

enum sirf_fifo_event_e {
	SIRF_FIFO_EVENT_WR_THRES = 1,
	SIRF_FIFO_EVENT_RD_THRES,
	SIRF_FIFO_EVENT_ERROR,
};

////////////////////////////////////////////////////////////////////////////////
#if defined(SIRF_CONFIG_STATS)
/**
 * \struct sirf_debug_s
 * 
 * \details Debug statistics of the Silabs Radio driver
 */
struct sirf_debug_s {
	uint32_t err_pream_cnt;
	uint32_t err_sync_cnt;
	uint32_t err_rssi_jmp_cnt;
	uint32_t err_crc_cnt;

	uint32_t ok_pream_det;
	uint32_t ok_sync_det;
	uint32_t ok_packet_rx;
	uint32_t ok_packet_tx;
	sirf_chip_state_t  state;
};
#endif

/**
 * \struct sirf_data_s
 * 
 * \details Internal data structure of the Silabs Radio driver
 */
struct sirf_data_s {
	uint8_t rfch;
	sirf_fifo_t fifo;
	uint32_t rx_to_ms;
	event_handle_t event_cb;
	enum sirf_state_e	state;
#if defined(SIRF_CONFIG_RF_EVT_TS)
	uint32_t ts_event[SIRF_RTC_TS_ID_CNT];
#endif
#if defined(SIRF_CONFIG_SLEEP)
	bool sleep_active;
#endif
};

/**
 * \typedef Radio IRQ handler type
 */
typedef void (* sirf_irq_handle_t)(ezradio_cmd_reply_t *);

////////////////////////////////////////////////////////////////////////////////
static void sirf_hw_init(void * sirf_irq_handle, void const * sirf_chip_config);

static void sirf_chip_int_handle(void);
static void sirf_gpio_irq_handler(uint8_t p_pin);
static void sirf_irq_chip_handle(ezradio_cmd_reply_t * p_reply_buf);
static void sirf_irq_modem_handle(ezradio_cmd_reply_t * p_reply_buf);
static void sirf_irq_ph_handle(ezradio_cmd_reply_t * p_reply_buf);

static void sirf_trx_event_handle(uint8_t p_sm_event);
static void sirf_trx_timeout_handle(uint32_t rtc_id, void * rtc_cb_data) __attribute__((unused));

static void sirf_fifo_tx_para(uint8_t * p_buf, uint16_t p_buf_len);
static void sirf_fifo_rx_para(uint8_t * p_buf, uint16_t p_buf_len);
static void sirf_fifo_trx_para(uint8_t * p_tx_buf, uint16_t p_tx_buf_len,
	uint8_t * p_rx_buf, uint16_t p_rx_buf_len);

static void sirf_fifo_tx_wr(sirf_fifo_t * p_fifo_data);
static void sirf_fifo_rx_rd(sirf_fifo_t * p_fifo_data);
static void sirf_fifo_tx_reset(sirf_fifo_t * p_fifo_data);
static void sirf_fifo_rx_reset(sirf_fifo_t * p_fifo_data);
static void sirf_fifo_trx_reset(sirf_fifo_t * p_fifo_data);

static void sirf_packet_rx_done(uint8_t p_rf_status);
static void sirf_packet_tx_done(uint8_t p_rf_status);

#if defined(SIRF_CONFIG_DEBUG_PROPS)
static void sirf_chip_props_read(uint8_t group, uint8_t num_props, uint8_t start_prop,
	uint8_t * prop_buf);
#endif

#if defined(SIRF_CONFIG_SLEEP)
static void sirf_slep_enter(void);
static void sirf_slep_exit(void);
#endif

////////////////////////////////////////////////////////////////////////////////
#if defined(SIRF_CONFIG_STATS)
static struct sirf_debug_s sirf_debug;
#endif
static struct sirf_data_s sirf_data;

static uint8_t sirf_irq_ph_mask;
static uint8_t sirf_irq_mo_mask;
static uint8_t sirf_irq_ch_mask;

/**
 * \brief IRQ handler table
 * 
 * \details Sorted semantically from chip, modem to packet handler.
 */
sirf_irq_handle_t sirf_irq_handler_tbl[SIRF_CHIP_IRQ_HANDLE_CNT] = {
	sirf_irq_chip_handle,
	sirf_irq_modem_handle,
	sirf_irq_ph_handle
};

/**
 * \brief Chip configuration
 * 
 * \details This is the configuration of the Silabs Radio chip. Normally created
 * by the Silabs WDS tool.
 */
static const uint8_t sirf_chip_config[] = RADIO_CONFIGURATION_DATA_ARRAY;

////////////////////////////////////////////////////////////////////////////////
sirf_ret_t sirf_init(sirf_init_para_t const * p_init_para) {
	/* Init internal data */
	memset((void *)&sirf_data, 0, sizeof(sirf_data));
#if defined(SIRF_CONFIG_STATS)
	memset((void *)&sirf_debug, 0, sizeof(sirf_debug));
#endif

	/* Set rfch and event callback */
	sirf_data.rfch  = p_init_para->rfch;
	sirf_data.event_cb = p_init_para->event_cb;

	/* init ezradio driver */
	sirf_hw_init(sirf_gpio_irq_handler, sirf_chip_config);

#if defined(SIRF_CONFIG_HW_ADDR)
	// DOWNLINK - Broadcast (10 Bit, Val. 0)
	ezradio_set_property(0x30, 0x03, 0x00,
		(uint8_t)(p_init_para->match_addr_bc), // Value (BC-Addr.: 0-7)
		(uint8_t) 0b11111111, // Mask
		(uint8_t)(0 | SIRF_MATCH_EN));  // Logic is true

	ezradio_set_property(0x30, 0x03, 0x00,
		(uint8_t)((p_init_para->match_addr_bc >> 8) & 0x03), // Value (BC-Addr.: 8-9 + Link Bit)
		(uint8_t) 0b10000011,                           // Mask
		(uint8_t)(8 | SIRF_MATCH_AND));               // Logic is true

	// DOWNLINK - Single Link (10 Bit, Val. 1 - 1023)
	ezradio_set_property(0x30, 0x03, 0x00,
		(uint8_t)(p_init_para->match_addr),    // Value (BC-Addr.: 0-7)
		(uint8_t) 0b11111111,            // Mask
		(uint8_t)(0 | SIRF_MATCH_OR));  // Logic is true

	ezradio_set_property(0x30, 0x03, 0x00,
		(uint8_t)((p_init_para->match_addr >> 8) & 0x03), // Value (BC-Addr.: 8-9 + Link Bit)
		(uint8_t) 0b10000011,                       // Mask
		/* offset */8 | SIRF_MATCH_AND);           // Logic is true
#endif

#if defined(SIRF_CONFIG_DEBUG) \
	&& (SIRF_CONFIG_DEBUG & SIRF_DEBUG_CHIP_IRQ)
	/* Enable all IRQs for DEBUG purpose */
	sirf_irq_ph_mask = 0b11111111;
	sirf_irq_mo_mask = 0b11111111;
	sirf_irq_ch_mask = 0b01111111;

	ezradio_set_property(0x01, 0x04, 0x00,	// Group: 1, Count: 4, offset: 0
		0b00000111,			// PH, MODEM, CHIP
		sirf_irq_ch_mask,	// FIFOs, CRC ERR, RX, TX, MATCH
		sirf_irq_mo_mask,	// PREAM ERR, SYNC ERR
		sirf_irq_ch_mask );	// CMD ERR, FIFO ERR
#else
	sirf_irq_ph_mask = 0b00111111;
	sirf_irq_mo_mask = 0b00000000;
	sirf_irq_ch_mask = 0b00101000;

	ezradio_set_property(0x01, 0x04, 0x00, // Group: 1, Count: 4, offset: 0
		0b00000111,			// PH, MODEM, CHIP
		sirf_irq_ph_mask,	// FIFOs, CRC ERR, RX, TX
		sirf_irq_mo_mask,	// PREAM ERR, SYNC ERR
		sirf_irq_ch_mask );	// CMD ERR, FIFO ERR
#endif

#if defined(SIRF_CONFIG_DEBUG) \
	&& (SIRF_CONFIG_DEBUG & SIRF_DEBUG_CHIP_HW_GPIO)
	uint8_t cmd_gpio_res = 0;
	uint8_t cmd_gpio_req[8] = { RF_GPIO_PIN_CFG };

	cmd_gpio_req[4] = 33; // GPIO3 - Output RX-State
	cmd_gpio_req[5] = 32; // GPIO4 - Output TX-State

	ezradio_comm_SendCmdGetResp(0x08, cmd_gpio_req, 0x01, &cmd_gpio_res);
#endif

	SIRF_SLEEP_ENTER();
	sirf_data.state = SIRF_STATE_IDLE;

	return SIRF_SUCCESS;
}

sirf_ret_t sirf_deinit(void) {
	// TDOO: Implement
	return SIRF_ERROR;
}

sirf_ret_t sirf_task(void) {
#if !defined(SIRF_CONFIG_IRQ)
	sirf_chip_int_handle();
#endif
	return SIRF_SUCCESS;
}

sirf_ret_t sirf_packet_tx(uint8_t* p_tx_buf, uint16_t  p_buf_len) {
	SIRF_ASSERT(p_tx_buf != NULL);

	if (SIRF_STATE_IDLE != sirf_data.state)
		return SIRF_ERROR;

	sirf_data.state = SIRF_STATE_TX;
	SIRF_SLEEP_EXIT();

	sirf_fifo_tx_para(p_tx_buf, p_buf_len);

	SIRF_TS_TRACE(SIRF_RTC_TS_ID_TX_START);
	ezradio_start_tx(sirf_data.rfch, SIRF_CHIP_CMD_TX_NSTATE_READY, p_buf_len);

	return SIRF_SUCCESS;
}

sirf_ret_t sirf_packet_rx(uint8_t* p_rx_buf, uint16_t  p_rx_len,
	uint16_t  p_rx_to_ms) {
	SIRF_ASSERT(p_pkt_buf != NULL);

	if (SIRF_STATE_IDLE != sirf_data.state)
		return SIRF_ERROR;

	sirf_data.state = SIRF_STATE_RX;
	SIRF_SLEEP_EXIT();

	sirf_fifo_rx_para(p_rx_buf, p_rx_len);

	SIRF_TS_TRACE(SIRF_RTC_TS_ID_RX_START);
	ezradio_start_rx(sirf_data.rfch, (SIRF_CHIP_CMD_RX_OPT_USE |
		SIRF_CHIP_CMD_RX_OPT_START_IMD),
		SIRF_CHIP_CMD_RX_OPT_LEN_AUTO,
		SIRF_CHIP_CMD_RX_NSTATE1_3
	);

	if (p_rx_to_ms)
		SIRF_TMR_TIMEOUT_RX_START(p_rx_to_ms, \
			sirf_trx_timeout_handle);

	return SIRF_SUCCESS;
}

/**
 * \brief 
 */
sirf_ret_t sirf_packet_trx(uint8_t * p_tx_buf, uint16_t p_tx_buf_len,
	uint8_t * p_rx_buf, uint16_t p_rx_buf_len, uint16_t p_rx_to_ms) {
	SIRF_ASSERT(p_pkt_tx_buf != NULL);
	SIRF_ASSERT(p_pkt_rx_buf != NULL);

	if (SIRF_STATE_IDLE != sirf_data.state)
		return SIRF_ERROR;

	sirf_data.state = SIRF_STATE_TRX;
	SIRF_SLEEP_EXIT();

	sirf_fifo_trx_para(p_tx_buf, p_tx_buf_len, p_rx_buf, p_rx_buf_len);

	sirf_data.rx_to_ms = 0;
	if (p_rx_to_ms)
		sirf_data.rx_to_ms = p_rx_to_ms;

	SIRF_TS_TRACE(SIRF_RTC_TS_ID_TRX_START);
	ezradio_start_rx(sirf_data.rfch, (SIRF_CHIP_CMD_RX_OPT_UPD |
		SIRF_CHIP_CMD_RX_OPT_START_IMD),
		SIRF_CHIP_CMD_RX_OPT_LEN_AUTO,
		SIRF_CHIP_CMD_RX_NSTATE1_3
	);

	ezradio_start_tx(sirf_data.rfch, SIRF_CHIP_CMD_TX_NSTATE_RX, p_tx_buf_len);

	return SIRF_SUCCESS;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Initialize the Silabs Radio driver
 * 
 * \details This function initializes the Silabs Radio driver. It sets the
 * interrupt handler and the SPI interface. The radio is reset and the
 * configuration is loaded.
 * 
 * \param[in] sirf_irq_handle Pointer to the interrupt handler
 * \param[in] sirf_chip_config Pointer to the radio configuration
 * \return sirf_ret_t
 */
void sirf_hw_init(void * sirf_irq_handle, void const* sirf_chip_config) {
	EZRADIO_ConfigRet_t ret;

	ezradio_hal_GpioInit(sirf_irq_handle, 0);
	ezradio_hal_SpiInit();

	ezradio_reset();
	// TODO USDELAY HERE!
	ret = ezradio_configuration_init((const uint8_t*)sirf_chip_config);
	if (EZRADIO_CONFIG_SUCCESS != ret)
		SIRF_FATAL_ERROR();
	
	ezradio_get_int_status(SIRF_CHIP_INT_MASK_CLR, NULL);
}

#if defined(SIRF_CONFIG_SLEEP)
void sirf_sleep_enter(void) {
	if (sirf_data.sleep_active)
		return; /* already in sleep mode, exit */

	ezradio_change_state(SIRF_CHIP_STATE_SLEEP);

	ezradio_hal_SpiDeinit();
	ezradio_hal_GpioSleep();
	sirf_data.sleep_active = true;
}

void sirf_sleep_exit(void) {
	ezradio_cmd_reply_t radio_reply_buf;

	if (!sirf_data.sleep_active)
		return; /* not in sleep, exit */

	ezradio_hal_GpioWakeUp();
	ezradio_hal_SpiInit();

	ezradio_change_state(SIRF_CHIP_STATE_READY);
	ezradio_get_int_status(SIRF_CHIP_INT_MASK_CLR, &radio_reply_buf);

#if defined(SIRF_CONFIG_IRQ)
	ezradio_hal_GpioIrqEnable(true);
#endif
	sirf_data.sleep_active = false;
}
#endif

/**
 * \brief Transmit package operation done
 * 
 * \details This function is called when the radio has successfully transmitted
 * a package. Provide tx buffer and its length to the user via the event
 * callback.
 * 
 * \param[in] p_rf_status
 * \return void
 */
void sirf_packet_tx_done(uint8_t p_rf_status) {
	sirf_tx_data_t tx_data;

	SIRF_TS_TRACE_RF(SIRF_RTC_TS_ID_TX_DONE);
	tx_data.buf = sirf_data.fifo.tx_buf;
	tx_data.len = sirf_data.fifo.tx_len;

	SIRF_SLEEP_ENTER();
	sirf_data.state = SIRF_STATE_IDLE;

	if (sirf_data.event_cb != NULL)
		sirf_data.event_cb(p_rf_status, &tx_data);
}

/**
 * \brief Receive package operation done
 * 
 * \details This function is called when the radio has successfully received
 * a package. Provide rx buffer and its length to the user via the event callback.
 * 
 * \param[in] p_rf_status
 * \return void
 */
void sirf_packet_rx_done(uint8_t p_rf_status) {
	sirf_rx_data_t rx_data;

	SIRF_TS_TRACE_RF(SIRF_RTC_TS_ID_RX_DONE);

#if defined(SIRF_RX_RSSI)
	ezradio_cmd_reply_t rf_data;
	ezradio_get_modem_status(0xff, &rf_data);
	rx_data.rssi = rf_data.GET_MODEM_STATUS.LATCH_RSSI;
#endif
	rx_data.len = sirf_data.fifo.rx_len;
	rx_data.buf = sirf_data.fifo.rx_buf;

	SIRF_SLEEP_ENTER();
	sirf_data.state = SIRF_STATE_IDLE;

	if (sirf_data.event_cb)
		sirf_data.event_cb(p_rf_status, &rx_data);
}

/**
 * \brief 
 */
void sirf_packet_trx_stop(void) {
	if (SIRF_STATE_IDLE == sirf_data.state)
		return;

	SIRF_TMR_TIMEOUT_RX_ABORT();
	SIRF_SLEEP_EXIT();
	ezradio_change_state(SIRF_CHIP_STATE_READY);
	sirf_fifo_trx_reset(&sirf_data.fifo);
	ezradio_get_int_status(SIRF_CHIP_INT_MASK_CLR, NULL);
	SIRF_SLEEP_ENTER();
	sirf_data.state = SIRF_STATE_IDLE;
}

/**
 * \details Package error done
 * 
 * \ref
 */
void sirf_packet_err_done(uint8_t p_rf_status) {
	if (sirf_data.event_cb != NULL)
		sirf_data.event_cb(p_rf_status, NULL);
}

/**
 * \brief TX/RX timeout handler
 * 
 * \details Should be called when the configured timeout expired. It will call
 * the state machine handler with the event \ref SIRF_EVENT_ERR_TIMEOUT.
 */
void sirf_trx_timeout_handle(uint32_t tmr_id, void* tmr_pref) {
	(void)tmr_id;
	(void)tmr_pref;

	sirf_trx_event_handle(SIRF_EVENT_ERR_TIMEOUT);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Write TX FIFO parameter
 * 
 * \details This function writes the TX FIFO parameter.
 * 
 * \param[in] p_buf	Pointer to the buffer
 * \param[in] p_buf_len Length of the buffer
 * \return void
 */
void sirf_fifo_tx_para(uint8_t * p_buf, uint16_t p_buf_len) {
	SIRF_ASSERT(p_buf != NULL);
	SIRF_ASSERT(p_buf_len != 0);

	sirf_fifo_tx_reset(&sirf_data.fifo);
	sirf_data.fifo.tx_buf = p_buf;
	sirf_data.fifo.tx_len = p_buf_len;
	sirf_data.fifo.tx_idx = 0;

	sirf_fifo_tx_wr(&sirf_data.fifo);
}

/**
 * \brief Write RX FIFO parameter
 * 
 * \details This function writes the RX FIFO parameter.
 * 
 * \param[in] p_buf	Pointer to the buffer
 * \param[in] p_buf_len Length of the buffer
 * \return void
 */
void sirf_fifo_rx_para(uint8_t * p_buf, uint16_t p_buf_len) {
	SIRF_ASSERT(p_buf != NULL);
	SIRF_ASSERT(p_buf_len != 0);

	sirf_fifo_rx_reset(&sirf_data.fifo);
	sirf_data.fifo.rx_buf = p_buf;
	sirf_data.fifo.rx_len = 0;
	sirf_data.fifo.rx_idx = 0;
	sirf_data.fifo.rx_max = p_buf_len;
}

/**
 * \brief Write TX and RX FIFO parameter
 * 
 * \details This function writes the TX and RX FIFO parameter.
 * 
 * \param[in] p_tx_buf	Pointer to the TX buffer
 * \param[in] p_rx_buf	Pointer to the RX buffer
 * \param[in] p_tx_buf_len Length of the TX buffer
 * \param[in] p_rx_buf_len Length of the RX buffer
 * \return void
 */
void sirf_fifo_trx_para(uint8_t * p_tx_buf, uint16_t p_tx_buf_len,
	uint8_t * p_rx_buf, uint16_t p_rx_buf_len) {
	SIRF_ASSERT(p_tx_buf != NULL);
	SIRF_ASSERT(p_rx_buf != NULL);

	sirf_fifo_tx_para(p_tx_buf, p_tx_buf_len);
	sirf_fifo_rx_para(p_rx_buf, p_rx_buf_len);
}

/**
 * \brief Read data to the TX FIFO
 * 
 * \details This function reads data to the TX FIFO.
 * 
 * \param[in] p_fifo_data
 * \return void
 */
static void sirf_fifo_rx_rd(sirf_fifo_t* p_fifo_data) {
	uint8_t fifo_rd_len = 0;
	ezradio_cmd_reply_t rf_cmd_ret;
	SIRF_ASSERT(p_fifo_data->rx_buf != NULL);

	if (!p_fifo_data->rx_idx)
		SIRF_TS_TRACE_RF(SIRF_RTC_TS_ID_RX_INPROG);

	ezradio_fifo_info_fast_read(&rf_cmd_ret);
	fifo_rd_len = rf_cmd_ret.FIFO_INFO.RX_FIFO_COUNT;

	if (fifo_rd_len) {
		if ((p_fifo_data->rx_len + fifo_rd_len) <= p_fifo_data->rx_max) {
			ezradio_read_rx_fifo(fifo_rd_len, p_fifo_data->rx_buf + p_fifo_data->rx_idx);
			p_fifo_data->rx_idx += fifo_rd_len;
			p_fifo_data->rx_len += fifo_rd_len;
		} else {
			sirf_trx_event_handle(SIRF_EVENT_ERR_SUPR);
		}
	} else {
		sirf_trx_event_handle(SIRF_EVENT_ERR_SUPR);
	}
}

/**
 * \brief Write data to the TX FIFO
 * 
 * \details This function writes data to the TX FIFO.
 * 
 * \param[in] p_fifo_data
 * \return void
 */
void sirf_fifo_tx_wr(sirf_fifo_t* p_fifo_data) {
	uint8_t fifo_wr_len;
	ezradio_cmd_reply_t rf_cmd_ret;
	SIRF_ASSERT(p_fifo_data->tx_buf != NULL);

	if (!p_fifo_data->tx_len)
		return;

#if defined(SIRF_CONFIG_RF_EVT_TS)
	if (p_fifo_data->tx_idx == 0)
		SIRF_TS_TRACE_RF(SIRF_RTC_TS_ID_TX_INPROG);
#endif

	ezradio_fifo_info_fast_read(&rf_cmd_ret);
	if (p_fifo_data->tx_len <= rf_cmd_ret.FIFO_INFO.TX_FIFO_SPACE)
		fifo_wr_len = p_fifo_data->tx_len;
	else
		fifo_wr_len = rf_cmd_ret.FIFO_INFO.TX_FIFO_SPACE;

	ezradio_write_tx_fifo(fifo_wr_len, 
		p_fifo_data->tx_buf + p_fifo_data->tx_idx);

	p_fifo_data->tx_idx += fifo_wr_len;
	p_fifo_data->tx_len -= fifo_wr_len;
}

/**
 * \brief Reset the TX and RX FIFO
 * 
 * \details This function resets the TX and RX FIFO.
 * 
 * \param[in] p_fifo_data
 * \return void
 */
void sirf_fifo_trx_reset(sirf_fifo_t* p_fifo_data) {
	p_fifo_data->tx_buf = NULL;
	p_fifo_data->rx_buf = NULL;
	p_fifo_data->tx_len = 0;
	p_fifo_data->rx_len = 0;
	p_fifo_data->tx_idx = 0;
	p_fifo_data->rx_idx = 0;
	p_fifo_data->rx_max = 0;

	ezradio_fifo_info((SIRF_FIFO_TX_RESET|SIRF_FIFO_RX_RESET), NULL);
}

/**
 * \brief Reset the TX FIFO
 * 
 * \details This function resets the TX FIFO.
 * 
 * \param[in] p_fifo_data
 * \return void
 */
void sirf_fifo_tx_reset(sirf_fifo_t* p_fifo_data) {
	p_fifo_data->tx_buf = NULL;
	p_fifo_data->tx_len = 0;
	p_fifo_data->tx_idx = 0;

	ezradio_fifo_info(SIRF_FIFO_TX_RESET, NULL);
}

/**
 * \brief Reset the RX FIFO
 * 
 * \details This function resets the RX FIFO.
 * 
 * \param[in] p_fifo_data
 * \return void
 */
void sirf_fifo_rx_reset(sirf_fifo_t* p_fifo_data) {
	p_fifo_data->rx_buf = NULL;
	p_fifo_data->rx_len = 0;
	p_fifo_data->rx_idx = 0;
	p_fifo_data->rx_max = 0;

	ezradio_fifo_info(SIRF_FIFO_RX_RESET, NULL);
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Handle fifo events
 * 
 * \details This function handles the fifo events.
 * 
 * \param[in] p_fifo_event
 * \return void
 */
void sirf_fifo_event_handle(uint8_t p_fifo_event){
	if (p_fifo_event == SIRF_FIFO_EVENT_WR_THRES) {
		if ((SIRF_STATE_TX != sirf_data.state) && (SIRF_STATE_TRX != sirf_data.state)  ) {
			sirf_trx_event_handle(SIRF_EVENT_ERR_FIFO);
		} else {
			sirf_fifo_tx_wr(&sirf_data.fifo);
		}
	} else if (p_fifo_event == SIRF_FIFO_EVENT_RD_THRES) {
		if ((SIRF_STATE_RX != sirf_data.state) && (SIRF_STATE_TRX != sirf_data.state)  ) {
			sirf_trx_event_handle(SIRF_EVENT_ERR_FIFO);
		} else {
			sirf_fifo_rx_rd(&sirf_data.fifo);
		}
	}
	else
		sirf_trx_event_handle(SIRF_EVENT_ERR_FIFO);
}

/**
 * \brief Handle receive/transmit events
 * 
 * \details This function handles the receive/transmit events.
 * 
 * \param[in] p_sm_event
 * \return void
 */
void sirf_trx_event_handle(uint8_t p_sm_event) {
	struct sirf_data_s * sirf_data_p = &sirf_data;

	switch(sirf_data_p->state) {
		case SIRF_STATE_IDLE:
			sirf_packet_trx_stop();
			sirf_packet_err_done(SIRF_STATE_UKNOWN);
		break;

		case SIRF_STATE_RX:
			SIRF_TMR_TIMEOUT_RX_ABORT();
			if (SIRF_EVENT_RX_DONE == p_sm_event) {
				sirf_fifo_rx_rd(&sirf_data.fifo);
				sirf_packet_rx_done(SIRF_STATE_RX_DONE);
			} else {
				sirf_packet_trx_stop();
				sirf_packet_err_done(SIRF_STATE_RX_ERR);
			}
		break;

		case SIRF_STATE_TX:
			if (SIRF_EVENT_TX_DONE == p_sm_event) {
				sirf_packet_tx_done(SIRF_STATE_TX_DONE);
			} else {
				sirf_packet_trx_stop();
				sirf_packet_err_done(SIRF_STATE_TX_ERR);
			}
		break;

		case SIRF_STATE_TRX:
			if (SIRF_EVENT_TX_DONE == p_sm_event) {
				sirf_packet_tx_done(SIRF_STATE_TX_DONE);
				if (sirf_data.rx_to_ms)
					SIRF_TMR_TIMEOUT_RX_START(sirf_data.rx_to_ms,
						sirf_trx_timeout_handle);
				sirf_data.state = SIRF_STATE_TRX;
			}
			else if (SIRF_EVENT_RX_DONE == p_sm_event) {
				SIRF_TMR_TIMEOUT_RX_ABORT();
				sirf_fifo_rx_rd(&sirf_data.fifo);
				sirf_packet_rx_done(SIRF_STATE_RX_DONE);
			} else {
				sirf_packet_trx_stop();
				sirf_packet_err_done(SIRF_STATE_TRX_ERR);
			}
		break;

		default:
			sirf_packet_trx_stop();
			sirf_packet_err_done(SIRF_STATE_UKNOWN);
		break;
	}
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Handle radio chip IRQs
 * 
 * \details This function handles the radio chip IRQs. It reads the radio chip
 * IRQ status and calls the appropriate IRQ handler.
 * 
 * \param[in] p_reply_buf
 * \return void
 */
void sirf_irq_chip_handle(ezradio_cmd_reply_t * p_reply_buf) {
	ezradio_cmd_reply_t cmd_reply;
	uint8_t chip_irq_pend;

	/* Aquire chip IRQ state */
	chip_irq_pend = p_reply_buf->GET_INT_STATUS.CHIP_PEND;

	/* CMD ERROR pending */
	if (chip_irq_pend & 0b00001000)
		sirf_trx_event_handle(SIRF_EVENT_ERR_CMD);

	/* STATE CHANGED pending */
	if (chip_irq_pend & 0b00010000) {
		ezradio_request_device_state(&cmd_reply);
#if defined(SIRF_CONFIG_STATS)
		sirf_debug.state = cmd_reply.REQUEST_DEVICE_STATE.CURR_STATE;
#endif
	}

	/* FIFO ERROR pending */
	if (chip_irq_pend & 0b00100000)
		sirf_fifo_event_handle(SIRF_FIFO_EVENT_ERROR);

	/* CAL, STATE CHANGED, CHIP READY, LOW BAT */
	if (chip_irq_pend & ~sirf_irq_ch_mask){
		ezradio_cmd_reply_t reply;
		ezradio_get_property(0x01, 0x04, 0x00, &reply);    // CMD ERR, FIFO ERR
	}
}

/**
 * \brief Handle radio modem IRQs
 * 
 * \details This function handles the radio modem IRQs. It reads the radio modem
 * IRQ status and calls the appropriate IRQ handler.
 * 
 * \param[in] p_reply_buf
 * \return void
 */
static void sirf_irq_modem_handle(ezradio_cmd_reply_t * p_reply_buf) {
	uint8_t sirf_irq_pen;

	/* Aquire modem IRQ state */
	sirf_irq_pen = p_reply_buf->GET_INT_STATUS.MODEM_PEND;

#if defined(SIRF_CONFIG_STATS)
	/* PREAM DETECT */
	if (sirf_irq_pen & 0b00000001)
		sirf_debug.ok_sync_det++;

	/* SYNC DETECT */
	if (sirf_irq_pen & 0b00000010)
		sirf_debug.ok_pream_det++;

	/* PREAMBLE ERROR */
	if (sirf_irq_pen & 0b00000100)
		sirf_debug.err_pream_cnt++;

	/* RSSI JUMP */
	if (sirf_irq_pen & 0b00010000)
		sirf_debug.err_rssi_jmp_cnt++;

	/* SYNC ERROR */
	if (sirf_irq_pen & 0b00100000)
		sirf_debug.err_sync_cnt++;
#endif

	/*  SYNC DET, RSSI,
		RSSI JUMP, POSTAMBLE DET, RSSI LATCH */
	if (sirf_irq_pen & ~sirf_irq_mo_mask) {
		ezradio_cmd_reply_t reply;
		ezradio_get_property(0x01, 0x04, 0x00, &reply);    // CMD ERR, FIFO ERR
	}
}

/**
 * \brief Handle radio protocol IRQs
 * 
 * \details This function handles the radio protocol handler IRQs. It reads the
 * radio protocol handler IRQ status and calls the appropriate IRQ handler.
 * 
 * \param[in] p_reply_buf
 * \return void
 */
void sirf_irq_ph_handle(ezradio_cmd_reply_t * p_reply_buf) {
	uint8_t sirf_irq_pen;

	/* Aquire protocol handler IRQ state */
	sirf_irq_pen = p_reply_buf->GET_INT_STATUS.PH_PEND;

	//-- Handle FIFO
	/* FIFO RD THRESHOLD */
	if (sirf_irq_pen & 0b00000001)
		sirf_fifo_event_handle(SIRF_FIFO_EVENT_RD_THRES);

	/* FIFO WR THRESHOLD */
	if (sirf_irq_pen & 0b00000010)
		sirf_fifo_event_handle(SIRF_FIFO_EVENT_WR_THRES);

	//-- Handle ERROR
	/* ALT CRC ERROR 0b00000100 */
	/* CRC ERROR     0b00001000*/
	if (sirf_irq_pen & 0b00001100) {
#if defined(SIRF_CONFIG_STATS)
		sirf_debug.err_crc_cnt++;
#endif
		sirf_trx_event_handle(SIRF_EVENT_ERR_CRC);
	}

	//-- Handle COMM
	/* PACKET RX */
	if (sirf_irq_pen & 0b00010000) {
#if defined(SIRF_CONFIG_STATS)
		sirf_debug.ok_packet_rx++;
#endif
		sirf_trx_event_handle(SIRF_EVENT_RX_DONE);
	}

	/* PACKET TX */
	if (sirf_irq_pen & 0b00100000) {
#if defined(SIRF_CONFIG_STATS)
		sirf_debug.ok_packet_tx++;
#endif
		sirf_trx_event_handle(SIRF_EVENT_TX_DONE);
	}

	//-- Handle unknown IRQs
	if (sirf_irq_pen & ~sirf_irq_ph_mask)
		sirf_trx_event_handle(SIRF_STATE_UKNOWN);
}

/**
 * \brief Handle radio IRQs
 * 
 * \details This function handles the radio IRQ pin. It reads the radio IRQ
 * status and calls the appropriate IRQ handler.
 */
void sirf_chip_int_handle(void) {
	ezradio_cmd_reply_t radio_reply_buf;

	SIRF_TS_TRACE(SIRF_RTC_TS_ID_RF_EVT);
	ezradio_get_int_status(SIRF_CHIP_INT_MASK_CLR, &radio_reply_buf);

	for(uint8_t irq_it = 0; irq_it < SIRF_CHIP_IRQ_HANDLE_CNT; irq_it++) {
		uint8_t irq_mask = (0b100 >> irq_it);
		if (radio_reply_buf.GET_INT_STATUS.INT_PEND & irq_mask)
			sirf_irq_handler_tbl[irq_it](&radio_reply_buf);
	}
}

/**
 * \brief Handle radio chip IRQs
 * 
 * \details This function handles the radio chip interrupt pin IRQs.
 */
void sirf_gpio_irq_handler(uint8_t p_pin) {
	(void)p_pin;
	sirf_chip_int_handle();
}

#if defined(SIRF_CONFIG_DEBUG_PROPS)
////////////////////////////////////////////////////////////////////////////////
/**
 * \brief Read radio chip properties
 * 
 * \param[in] group
 * \param[in] num_props
 * \param[in] start_prop
 * \param[out] prop_buf
 * \return void
 */
void sirf_chip_props_read(uint8_t group, uint8_t num_props, uint8_t start_prop,
	uint8_t * prop_buf) {
	uint8_t ezradioCmd[16u];

	ezradioCmd[0] = EZRADIO_CMD_ID_GET_PROPERTY;
	ezradioCmd[1] = group;
	ezradioCmd[2] = num_props;
	ezradioCmd[3] = start_prop;

	ezradio_comm_SendCmdGetResp(EZRADIO_CMD_ARG_COUNT_GET_PROPERTY, ezradioCmd,
		ezradioCmd[2], ezradioCmd);

	prop_buf[0]   = ezradioCmd[0];
	prop_buf[1]   = ezradioCmd[1];
	prop_buf[2]   = ezradioCmd[2];
	prop_buf[3]   = ezradioCmd[3];
	prop_buf[4]   = ezradioCmd[4];
	prop_buf[5]   = ezradioCmd[5];
	prop_buf[6]   = ezradioCmd[6];
	prop_buf[7]   = ezradioCmd[7];
	prop_buf[8]   = ezradioCmd[8];
	prop_buf[9]   = ezradioCmd[9];
	prop_buf[10]  = ezradioCmd[10];
	prop_buf[11]  = ezradioCmd[11];
	prop_buf[12]  = ezradioCmd[12];
	prop_buf[13]  = ezradioCmd[13];
	prop_buf[14]  = ezradioCmd[14];
	prop_buf[15]  = ezradioCmd[15];
}
#endif
