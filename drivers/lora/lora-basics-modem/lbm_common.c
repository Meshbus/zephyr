/*
 * Copyright (c) 2025 Embeint Inc
 * Copyright (c) 2026 Jakub Rzeszutko <jakub.rzeszutko@verkada.com>
 * Copyright (c) 2026 FoBE Studio
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/lora.h>
#include <zephyr/logging/log.h>

#include "lbm_common.h"

/* LoRa interrupts from the RAL library */
#define RAL_IRQ_LORA                                                                               \
	RAL_IRQ_TX_DONE | RAL_IRQ_RX_DONE | RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR |          \
		RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK

LOG_MODULE_REGISTER(lbm_driver, CONFIG_LORA_LOG_LEVEL);

/* When Symbol Time exceeds 16.38 ms (6.1.1.4 SX1261/2 datasheet), enable LDRO
 * Symbol Rate is bw / (2 ^ sf) so Symbol time is (2 ^ sf) / bw (6.1.1.1 SX1261/2 datasheet)
 * Additionally, enable LDRO in additional situations described in Lora Basic Modem lr1mac
 * where t < 16 from ral_compute_lora_ldro: Bandwidth less than 41 Khz, and SF9 with BW 41 KHz
 */
#define LORA_LDRO(sf, bw) ((((1 << sf) / bw) >= 16 ? 1 : 0) \
	| (bw < BW_41_KHZ || (bw == BW_41_KHZ && sf == SF_9) ? 1 : 0))

#define LBM_RX_DUTY_CYCLE_MAX_TIME_MS 262143U
#define LBM_CAD_TIMEOUT_MS 1000U
#define LBM_CAD_DETECTION_MINIMUM_DEFAULT 10U
#define LBM_CAD_TIMEOUT_MIN_MS 100U
#define LBM_CAD_TIMEOUT_MARGIN_MS 50U
#define LBM_CAD_TIMEOUT_SCALE_NUM 2U

int lbm_lora_cad(const struct device *dev, k_timeout_t timeout);

static int lora_cad_symbol_num_to_ral(enum lora_cad_symbol_num symbol_num,
				      ral_lora_cad_symbs_t *out)
{
	if ((int)symbol_num == 0) {
		*out = RAL_LORA_CAD_02_SYMB;
		return 0;
	}

	switch (symbol_num) {
	case LORA_CAD_SYMB_2:
		*out = RAL_LORA_CAD_02_SYMB;
		return 0;
	case LORA_CAD_SYMB_1:
		*out = RAL_LORA_CAD_01_SYMB;
		return 0;
	case LORA_CAD_SYMB_4:
		*out = RAL_LORA_CAD_04_SYMB;
		return 0;
	case LORA_CAD_SYMB_8:
		*out = RAL_LORA_CAD_08_SYMB;
		return 0;
	case LORA_CAD_SYMB_16:
		*out = RAL_LORA_CAD_16_SYMB;
		return 0;
	}

	return -EINVAL;
}

static uint8_t ral_cad_symbol_num_to_u8(ral_lora_cad_symbs_t symbol_num)
{
	switch (symbol_num) {
	case RAL_LORA_CAD_01_SYMB:
		return 1U;
	case RAL_LORA_CAD_02_SYMB:
		return 2U;
	case RAL_LORA_CAD_04_SYMB:
		return 4U;
	case RAL_LORA_CAD_08_SYMB:
		return 8U;
	case RAL_LORA_CAD_16_SYMB:
		return 16U;
	default:
		return 0U;
	}
}

static uint8_t lbm_cad_symbol_num_to_u8(enum lora_cad_symbol_num symbol_num)
{
	if ((int)symbol_num == 0) {
		return 2U;
	}

	switch (symbol_num) {
	case LORA_CAD_SYMB_1:
		return 1U;
	case LORA_CAD_SYMB_2:
		return 2U;
	case LORA_CAD_SYMB_4:
		return 4U;
	case LORA_CAD_SYMB_8:
		return 8U;
	case LORA_CAD_SYMB_16:
		return 16U;
	}

	return 2U;
}

static int lbm_rx_duty_cycle_period_to_ms(k_timeout_t period, uint32_t *period_ms)
{
	uint64_t ms;

	if (period_ms == NULL || K_TIMEOUT_EQ(period, K_NO_WAIT) ||
	    K_TIMEOUT_EQ(period, K_FOREVER)) {
		return -EINVAL;
	}

	ms = k_ticks_to_ms_ceil64((uint64_t)period.ticks);
	if ((ms == 0U) || (ms > LBM_RX_DUTY_CYCLE_MAX_TIME_MS)) {
		return -EINVAL;
	}

	*period_ms = (uint32_t)ms;
	return 0;
}

static int lbm_lora_bw_to_khz(ral_lora_bw_t bw, uint32_t *bw_khz)
{
	if (bw_khz == NULL) {
		return -EINVAL;
	}

	switch (bw) {
	case RAL_LORA_BW_007_KHZ:
		*bw_khz = 7U;
		return 0;
	case RAL_LORA_BW_010_KHZ:
		*bw_khz = 10U;
		return 0;
	case RAL_LORA_BW_015_KHZ:
		*bw_khz = 15U;
		return 0;
	case RAL_LORA_BW_020_KHZ:
		*bw_khz = 20U;
		return 0;
	case RAL_LORA_BW_031_KHZ:
		*bw_khz = 31U;
		return 0;
	case RAL_LORA_BW_041_KHZ:
		*bw_khz = 41U;
		return 0;
	case RAL_LORA_BW_062_KHZ:
		*bw_khz = 62U;
		return 0;
	case RAL_LORA_BW_125_KHZ:
		*bw_khz = 125U;
		return 0;
	case RAL_LORA_BW_200_KHZ:
		*bw_khz = 200U;
		return 0;
	case RAL_LORA_BW_250_KHZ:
		*bw_khz = 250U;
		return 0;
	case RAL_LORA_BW_400_KHZ:
		*bw_khz = 400U;
		return 0;
	case RAL_LORA_BW_500_KHZ:
		*bw_khz = 500U;
		return 0;
	case RAL_LORA_BW_800_KHZ:
		*bw_khz = 800U;
		return 0;
	case RAL_LORA_BW_1000_KHZ:
		*bw_khz = 1000U;
		return 0;
	case RAL_LORA_BW_1600_KHZ:
		*bw_khz = 1600U;
		return 0;
	}

	return -EINVAL;
}

static uint32_t lbm_cad_timeout_ms_dynamic(const struct device *dev)
{
	struct lbm_lora_data_common *data = dev->data;
	uint32_t bw_khz;
	uint8_t cad_symbols;
	uint64_t cad_symbols_numerator;
	uint64_t cad_est_ms;
	uint64_t timeout_ms;
	int ret;

	if ((data->mod_params.sf < RAL_LORA_SF5) || (data->mod_params.sf > RAL_LORA_SF12)) {
		return LBM_CAD_TIMEOUT_MS;
	}

	ret = lbm_lora_bw_to_khz(data->mod_params.bw, &bw_khz);
	if (ret < 0 || bw_khz == 0U) {
		return LBM_CAD_TIMEOUT_MS;
	}

	cad_symbols = lbm_cad_symbol_num_to_u8(data->cad_symbol_num);
	cad_symbols_numerator = ((uint64_t)cad_symbols << data->mod_params.sf) + 32ULL;
	cad_est_ms = DIV_ROUND_UP(cad_symbols_numerator, bw_khz);
	timeout_ms = (cad_est_ms * LBM_CAD_TIMEOUT_SCALE_NUM) + LBM_CAD_TIMEOUT_MARGIN_MS;
	timeout_ms = MAX(timeout_ms, (uint64_t)LBM_CAD_TIMEOUT_MIN_MS);

	if (timeout_ms > UINT32_MAX) {
		timeout_ms = UINT32_MAX;
	}

	LOG_DBG("CAD timeout auto: sf=%u bw=%uKHz sym=%u est=%ums timeout=%ums",
		(unsigned int)data->mod_params.sf, bw_khz, cad_symbols, (uint32_t)cad_est_ms,
		(uint32_t)timeout_ms);

	return (uint32_t)timeout_ms;
}

static k_timeout_t lbm_cad_timeout_auto(const struct device *dev)
{
	return K_MSEC(lbm_cad_timeout_ms_dynamic(dev));
}

static int lbm_rx_duty_cycle_resolve_periods_ms(k_timeout_t rx_period,
						 k_timeout_t sleep_period,
						 uint32_t *rx_period_ms,
						 uint32_t *sleep_period_ms)
{
	if (rx_period_ms == NULL || sleep_period_ms == NULL) {
		return -EINVAL;
	}

	if (K_TIMEOUT_EQ(rx_period, K_NO_WAIT) || K_TIMEOUT_EQ(sleep_period, K_NO_WAIT)) {
		return -EINVAL;
	}

	if (lbm_rx_duty_cycle_period_to_ms(rx_period, rx_period_ms) < 0) {
		return -EINVAL;
	}

	if (lbm_rx_duty_cycle_period_to_ms(sleep_period, sleep_period_ms) < 0) {
		return -EINVAL;
	}

	return 0;
}

/**
 * @brief Attempt to acquire the modem for operations
 *
 * @param dev modem to acquire
 *
 * @retval true if modem was acquired
 * @retval false otherwise
 */
static inline bool modem_acquire(const struct device *dev)
{
	struct lbm_lora_data_common *data = dev->data;

	return atomic_cas(&data->modem_state, STATE_FREE, STATE_BUSY);
}

/**
 * @brief Safely release the modem from any context
 *
 * This function can be called from any context and guarantees that the
 * release operations will only be run once.
 *
 * @param dev modem to release
 *
 * @retval true if modem was released by this function
 * @retval false otherwise
 */
static bool modem_release(const struct device *dev)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;

	/* Move to cleanup state so both acquire and release will fail */
	if (!atomic_cas(&data->modem_state, STATE_BUSY, STATE_CLEANUP)) {
		return false;
	}
	data->modem_mode = MODE_SLEEP;

	/* Disable DIO1 interrupt immediately so no new work items are scheduled
	 * while the chip is transitioning to sleep. Any work item already queued
	 * will find MODE_SLEEP and return early without touching the hardware.
	 */
	lbm_optional_dio1_irq_configure_dt(&config->dio1, GPIO_INT_DISABLE);

	/* Configure modem for sleep */
	lbm_driver_antenna_configure(dev, MODE_SLEEP);

	/* Put radio back into sleep mode */
	status = ral_set_sleep(&config->ralf.ral, true);
	if (status != RAL_STATUS_OK) {
		LOG_WRN("RAL sleep failed (%d)", status);
	}
	data->rx_started_with_duty_cycle = false;

	/* Completely release modem */
	data->operation_done = NULL;
	atomic_set(&data->modem_state, STATE_FREE);
	return true;
}

static ral_status_t modem_set_rx(const struct device *dev)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;

	data->rx_started_with_duty_cycle = false;

	if (data->duty_cycle_enabled && config->duty_cycle_supported) {
		/* Keep the radio in RX after preamble detection. Otherwise the
		 * duty-cycle timer can expire while the packet arrives.
		 */
		status = ral_stop_timer_on_preamble(&config->ralf.ral, true);
		if (status != RAL_STATUS_OK) {
			return status;
		}

		status = ral_set_rx_duty_cycle(&config->ralf.ral, data->duty_cycle_rx_time_ms,
						   data->duty_cycle_sleep_time_ms);
		if (status == RAL_STATUS_OK) {
			data->rx_started_with_duty_cycle = true;
			return status;
		}

		if (status != RAL_STATUS_UNSUPPORTED_FEATURE) {
			return status;
		}
	}

	return ral_set_rx(&config->ralf.ral, RAL_RX_TIMEOUT_CONTINUOUS_MODE);
}

static int lbm_configure_cad(const struct device *dev, k_timeout_t timeout)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_lora_cad_symbs_t ral_symbs;
	ral_status_t status;
	uint8_t det_peak = 0;
	int ret;

	ret = lora_cad_symbol_num_to_ral(data->cad_symbol_num, &ral_symbs);
	if (ret < 0) {
		return ret;
	}

	data->cad_params = (ral_lora_cad_params_t){
		.cad_symb_nb = ral_symbs,
		.cad_det_peak_in_symb = data->cad_detection_peak,
		.cad_det_min_in_symb = data->cad_detection_minimum == 0U
					       ? LBM_CAD_DETECTION_MINIMUM_DEFAULT
					       : data->cad_detection_minimum,
		.cad_exit_mode = RAL_LORA_CAD_ONLY,
		.cad_timeout_in_ms = k_ticks_to_ms_ceil32((uint64_t)timeout.ticks),
	};

	if (data->cad_params.cad_det_peak_in_symb == 0U) {
		status = ral_get_lora_cad_det_peak(&config->ralf.ral, data->mod_params.sf,
						   data->mod_params.bw, ral_symbs, &det_peak);
		if (status == RAL_STATUS_UNSUPPORTED_FEATURE) {
			return -ENOSYS;
		}
		if (status != RAL_STATUS_OK) {
			return -EIO;
		}
		data->cad_params.cad_det_peak_in_symb = det_peak;
	}

	status = ral_set_lora_cad_params(&config->ralf.ral, &data->cad_params);
	if (status != RAL_STATUS_OK) {
		return -EIO;
	}

	LOG_DBG("CAD configured: sym=%u peak=%u min=%u timeout_ms=%u",
		ral_cad_symbol_num_to_u8(data->cad_params.cad_symb_nb),
		data->cad_params.cad_det_peak_in_symb, data->cad_params.cad_det_min_in_symb,
		data->cad_params.cad_timeout_in_ms);

	return 0;
}

int lbm_lora_config(const struct device *dev, const struct lora_modem_config *lora_config)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ralf_params_lora_t params = {
		.mod_params = {
			.sf = lora_config->datarate,
			.cr = lora_config->coding_rate,
			.ldro = config->force_ldro ? 1 : LORA_LDRO(lora_config->datarate,
								   lora_config->bandwidth),
		},
		.pkt_params = {
			.preamble_len_in_symb = lora_config->preamble_len,
			.header_type = RAL_LORA_PKT_EXPLICIT,
			.pld_len_in_bytes = UINT8_MAX,
			.crc_is_on = !lora_config->packet_crc_disable,
			.invert_iq_is_on = lora_config->iq_inverted,
		},
		.rf_freq_in_hz = lora_config->frequency,
		.output_pwr_in_dbm = lora_config->tx_power,
		.sync_word = 0,
	};
	ral_lora_cad_symbs_t cad_symb_nb;
	ral_status_t status;
	enum lora_rx_boost previous_rx_boosted;
	int ret;

	/* Perform deferred radio initialization on first config */
	if (IS_ENABLED(CONFIG_LORA_BASICS_MODEM_DEFERRED_INIT) && !data->radio_initialized) {
		ret = lbm_driver_radio_init(dev);
		if (ret < 0) {
			return ret;
		}
		data->radio_initialized = true;
	}

	/* Ensure available, decremented after configuration */
	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	if (lora_config->sync_word) {
		params.sync_word = lora_config->sync_word;
	} else {
		params.sync_word = lora_config->public_network ? LBM_LORA_SYNC_WORD_PUBLIC
							       : LBM_LORA_SYNC_WORD_PRIVATE;
	}

	switch (lora_config->bandwidth) {
	case BW_7_KHZ:
		params.mod_params.bw = RAL_LORA_BW_007_KHZ;
		break;
	case BW_10_KHZ:
		params.mod_params.bw = RAL_LORA_BW_010_KHZ;
		break;
	case BW_15_KHZ:
		params.mod_params.bw = RAL_LORA_BW_015_KHZ;
		break;
	case BW_20_KHZ:
		params.mod_params.bw = RAL_LORA_BW_020_KHZ;
		break;
	case BW_31_KHZ:
		params.mod_params.bw = RAL_LORA_BW_031_KHZ;
		break;
	case BW_41_KHZ:
		params.mod_params.bw = RAL_LORA_BW_041_KHZ;
		break;
	case BW_62_KHZ:
		params.mod_params.bw = RAL_LORA_BW_062_KHZ;
		break;
	case BW_125_KHZ:
		params.mod_params.bw = RAL_LORA_BW_125_KHZ;
		break;
	case BW_200_KHZ:
		params.mod_params.bw = RAL_LORA_BW_200_KHZ;
		break;
	case BW_250_KHZ:
		params.mod_params.bw = RAL_LORA_BW_250_KHZ;
		break;
	case BW_400_KHZ:
		params.mod_params.bw = RAL_LORA_BW_400_KHZ;
		break;
	case BW_500_KHZ:
		params.mod_params.bw = RAL_LORA_BW_500_KHZ;
		break;
	case BW_800_KHZ:
		params.mod_params.bw = RAL_LORA_BW_800_KHZ;
		break;
	case BW_1000_KHZ:
		params.mod_params.bw = RAL_LORA_BW_1000_KHZ;
		break;
	case BW_1600_KHZ:
		params.mod_params.bw = RAL_LORA_BW_1600_KHZ;
		break;
	default:
		ret = -EINVAL;
		goto release;
	}

	switch (lora_config->cad.mode) {
	case LORA_CAD_MODE_NONE:
	case LORA_CAD_MODE_RX:
	case LORA_CAD_MODE_LBT:
		break;
	default:
		ret = -EINVAL;
		goto release;
	}

	ret = lora_cad_symbol_num_to_ral(lora_config->cad.symbol_num,
					 &cad_symb_nb);
	if (ret < 0) {
		goto release;
	}

	/* RAL BSP callbacks consult the cached RX boost during setup.
	 * Restore it on failure so an invalid configuration cannot partially
	 * replace the last successful software configuration.
	 */
	previous_rx_boosted = data->rx_boosted;
	data->rx_boosted = lora_config->rx_boosted;
	status = ralf_setup_lora(&config->ralf, &params);
	if (status != RAL_STATUS_OK) {
		data->rx_boosted = previous_rx_boosted;
		ret = -EIO;
		goto release;
	}

	/* Commit cached configuration only after hardware setup succeeds. */
	data->mod_params = params.mod_params;
	data->pkt_params = params.pkt_params;
	data->cad_mode = lora_config->cad.mode;
	data->cad_symbol_num = lora_config->cad.symbol_num;
	data->cad_detection_peak = lora_config->cad.detection_peak;
	data->cad_detection_minimum = lora_config->cad.detection_minimum;
	data->cad_operation = LBM_CAD_OPERATION_SYNC;
	data->cad_state.sync.detected = NULL;
	/* Duty-cycle RX setup is provided by lora_recv_duty_cycle* APIs. */
	data->duty_cycle_enabled = false;
	data->duty_cycle_rx_time_ms = 0U;
	data->duty_cycle_sleep_time_ms = 0U;
	ret = 0;

release:
	modem_release(dev);
	return ret;
}

uint32_t lbm_lora_airtime(const struct device *dev, uint32_t data_len)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;

	/* Updating the internal variable is fine since it is only used by ral_set_lora_pkt_params
	 * in lbm_lora_send_async, and the value is set there immediately before use.
	 */
	data->pkt_params.pld_len_in_bytes = data_len;

	return ral_get_lora_time_on_air_in_ms(&config->ralf.ral, &data->pkt_params,
					      &data->mod_params);
}

int lbm_lora_send_async(const struct device *dev, uint8_t *msg, uint32_t msg_len,
			struct k_poll_signal *async)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;
	int cad_ret;
	int ret = 0;

	if (msg_len > UINT8_MAX) {
		LOG_ERR("Payload too long: %u", msg_len);
		return -EINVAL;
	}

	if (data->cad_mode == LORA_CAD_MODE_LBT) {
		cad_ret = lbm_lora_cad(dev, lbm_cad_timeout_auto(dev));
		if (cad_ret > 0) {
			if (async != NULL) {
				k_poll_signal_raise(async, -EBUSY);
				return 0;
			}
			return -EBUSY;
		}
		if (cad_ret < 0) {
			if (async != NULL) {
				k_poll_signal_raise(async, cad_ret);
				return 0;
			}
			return cad_ret;
		}
	}

	/* Ensure available, freed by TX done callback */
	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	/* Configure modem for TX */
	lbm_driver_antenna_configure(dev, MODE_TX);
	data->modem_mode = MODE_TX;

	/* Validate that we have a TX configuration */
	if (data->mod_params.sf == 0) {
		ret = -EINVAL;
		goto release;
	}

	/* Store signal */
	data->operation_done = async;

	/* Update packet params to override the internal packet length variable.
	 * This has a huge overhead since it performs many register writes, but is the only
	 * generic way to update the variable. Why this isn't just done in ral_set_pkt_payload
	 * is anyones guess.
	 */
	data->pkt_params.pld_len_in_bytes = msg_len;
	status = ral_set_lora_pkt_params(&config->ralf.ral, &data->pkt_params);
	if (status != RAL_STATUS_OK) {
		ret = -EINVAL;
		goto release;
	}

	/* Set the packet payload */
	status = ral_set_pkt_payload(&config->ralf.ral, msg, msg_len);
	if (status != RAL_STATUS_OK) {
		ret = -EINVAL;
		goto release;
	}

	/* Clear any stale IRQ flags and re-arm DIO1 before starting TX so that
	 * the TX-done interrupt is delivered.
	 */
	(void)ral_clear_irq_status(&config->ralf.ral, RAL_IRQ_ALL);
	lbm_optional_dio1_irq_configure_dt(&config->dio1, GPIO_INT_EDGE_TO_ACTIVE);

	/* Start the transmission */
	status = ral_set_tx(&config->ralf.ral);
	if (status != RAL_STATUS_OK) {
		ret = -EINVAL;
		goto release;
	}
	return 0;

release:
	modem_release(dev);
	return ret;
}

int lbm_lora_send(const struct device *dev, uint8_t *msg, uint32_t msg_len)
{
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt =
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);
	uint32_t air_time;
	int ret;

	/* Trigger the asynchronous send */
	ret = lbm_lora_send_async(dev, msg, msg_len, &done);
	if (ret < 0) {
		return ret;
	}

	/* Calculate expected airtime of the packet */
	air_time = lbm_lora_airtime(dev, msg_len);
	LOG_DBG("Expected air time of %u bytes = %u ms", msg_len, air_time);

	/* Wait for the packet to finish transmitting.
	 * Setting up the transaction takes some minimal time, take it into
	 * account to ensure extremely short packets don't incorrectly timeout.
	 * Use twice the tx duration to ensure that we are actually detecting
	 * a failed transmission, and not some minor timing variation between
	 * modem and driver.
	 */
	ret = k_poll(&evt, 1, K_MSEC(10 + (2 * air_time)));
	if (ret < 0) {
		if (modem_release(dev)) {
			LOG_ERR("Packet transmission failed!");
		} else {
			/* TX done interrupt is currently running */
			k_poll(&evt, 1, K_FOREVER);
		}
		return ret;
	}
	return done.result;
}

static int lbm_lora_recv_locked(const struct device *dev, uint8_t *msg, uint8_t msg_len,
				k_timeout_t timeout, int16_t *rssi, int8_t *snr)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt =
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);
	ral_status_t status;
	int ret;

	/* Store signal */
	data->operation_done = &done;
	data->rx_state.sync.msg = msg;
	data->rx_state.sync.msg_len = msg_len;

	/* Configure modem for RX */
	lbm_driver_antenna_configure(dev, MODE_RX);
	data->modem_mode = MODE_RX;

	/* Clear any stale IRQ flags from the previous operation, then re-arm
	 * DIO1 so the RX-done interrupt is delivered.
	 */
	(void)ral_clear_irq_status(&config->ralf.ral, RAL_IRQ_ALL);
	lbm_optional_dio1_irq_configure_dt(&config->dio1, GPIO_INT_EDGE_TO_ACTIVE);

	/* Start the reception according to the active RX mode configuration.
	 * Receive timeouts are handled by the k_poll timeout.
	 */
	status = modem_set_rx(dev);
	if (status != RAL_STATUS_OK) {
		ret = -EINVAL;
		goto release;
	}

	/* Wait for the packet to be received */
	ret = k_poll(&evt, 1, timeout);
	if (ret < 0) {
		if (modem_release(dev)) {
			LOG_INF("Receive timeout");
			return -EAGAIN;
		}
		/* Releasing the modem failed, which means that
		 * the RX callback is currently running. Wait until
		 * the RX callback finishes and we get our packet.
		 */
		k_poll(&evt, 1, K_FOREVER);
	}

	if (done.result != 0) {
		LOG_ERR("Receive error");
		ret = done.result;
		goto release;
	}

	/* Retrieve cached RSSI and SNR */
	if (rssi != NULL) {
		*rssi = data->rx_state.sync.rssi_dbm;
	}
	if (snr != NULL) {
		*snr = data->rx_state.sync.snr_db;
	}
	ret = data->rx_state.sync.msg_len;

release:
	modem_release(dev);
	return ret;
}

static int lbm_lora_recv_async_start(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;

	/* Configure modem for asynchronous RX */
	lbm_driver_antenna_configure(dev, MODE_RX_ASYNC);
	data->modem_mode = MODE_RX_ASYNC;

	/* Store user state */
	data->rx_state.async.rx_cb = cb;
	data->rx_state.async.user_data = user_data;

	/* Clear any stale IRQ flags from the previous operation, then re-arm
	 * DIO1 so packet-received interrupts are delivered.
	 */
	(void)ral_clear_irq_status(&config->ralf.ral, RAL_IRQ_ALL);
	lbm_optional_dio1_irq_configure_dt(&config->dio1, GPIO_INT_EDGE_TO_ACTIVE);

	/* Start the reception according to the active RX mode configuration */
	status = modem_set_rx(dev);
	if (status != RAL_STATUS_OK) {
		modem_release(dev);
		return -EIO;
	}
	return 0;
}

int lbm_lora_recv(const struct device *dev, uint8_t *msg, uint8_t msg_len, k_timeout_t timeout,
		  int16_t *rssi, int8_t *snr)
{
	struct lbm_lora_data_common *data = dev->data;
	int ret;

	if (data->cad_mode == LORA_CAD_MODE_RX) {
		ret = lbm_lora_cad(dev, lbm_cad_timeout_auto(dev));
		if (ret <= 0) {
			return ret;
		}
	}

	/* Ensure available, decremented by op_done_work_handler or on timeout */
	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	return lbm_lora_recv_locked(dev, msg, msg_len, timeout, rssi, snr);
}

int lbm_lora_recv_duty_cycle(const struct device *dev,
			     k_timeout_t rx_period,
			     k_timeout_t sleep_period,
			     uint8_t *msg, uint8_t msg_len,
			     k_timeout_t timeout,
			     int16_t *rssi, int8_t *snr)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	bool prev_duty_cycle_enabled;
	uint32_t prev_duty_cycle_rx_time_ms;
	uint32_t prev_duty_cycle_sleep_time_ms;
	uint32_t rx_period_ms;
	uint32_t sleep_period_ms;
	int ret;

	if (!config->duty_cycle_supported) {
		return -ENOSYS;
	}

	ret = lbm_rx_duty_cycle_resolve_periods_ms(rx_period, sleep_period,
						   &rx_period_ms, &sleep_period_ms);
	if (ret < 0) {
		return ret;
	}

	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	prev_duty_cycle_enabled = data->duty_cycle_enabled;
	prev_duty_cycle_rx_time_ms = data->duty_cycle_rx_time_ms;
	prev_duty_cycle_sleep_time_ms = data->duty_cycle_sleep_time_ms;

	data->duty_cycle_enabled = true;
	data->duty_cycle_rx_time_ms = rx_period_ms;
	data->duty_cycle_sleep_time_ms = sleep_period_ms;

	ret = lbm_lora_recv_locked(dev, msg, msg_len, timeout, rssi, snr);

	data->duty_cycle_enabled = prev_duty_cycle_enabled;
	data->duty_cycle_rx_time_ms = prev_duty_cycle_rx_time_ms;
	data->duty_cycle_sleep_time_ms = prev_duty_cycle_sleep_time_ms;

	return ret;
}

int lbm_lora_recv_async(const struct device *dev, lora_recv_cb cb, void *user_data)
{
	/* Cancel ongoing reception */
	if (cb == NULL) {
		if (!modem_release(dev)) {
			/* Not receiving or already being stopped */
			return -EINVAL;
		}
		return 0;
	}

	/* Ensure available */
	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	return lbm_lora_recv_async_start(dev, cb, user_data);
}

int lbm_lora_recv_duty_cycle_async(const struct device *dev,
				   k_timeout_t rx_period,
				   k_timeout_t sleep_period,
				   lora_recv_cb cb, void *user_data)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	uint32_t rx_period_ms;
	uint32_t sleep_period_ms;
	int ret;

	if (!config->duty_cycle_supported) {
		return -ENOSYS;
	}

	if (cb == NULL) {
		ret = lbm_lora_recv_async(dev, NULL, NULL);
		if (ret == 0) {
			data->duty_cycle_enabled = false;
			data->duty_cycle_rx_time_ms = 0U;
			data->duty_cycle_sleep_time_ms = 0U;
		}
		return ret;
	}

	ret = lbm_rx_duty_cycle_resolve_periods_ms(rx_period, sleep_period,
						   &rx_period_ms, &sleep_period_ms);
	if (ret < 0) {
		return ret;
	}

	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	data->duty_cycle_enabled = true;
	data->duty_cycle_rx_time_ms = rx_period_ms;
	data->duty_cycle_sleep_time_ms = sleep_period_ms;

	ret = lbm_lora_recv_async_start(dev, cb, user_data);
	if (ret < 0) {
		data->duty_cycle_enabled = false;
		data->duty_cycle_rx_time_ms = 0U;
		data->duty_cycle_sleep_time_ms = 0U;
	}

	return ret;
}

int lbm_lora_rssi_inst(const struct device *dev, int16_t *rssi)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;

	if (rssi == NULL) {
		return -EINVAL;
	}

	/* RSSI is only meaningful while in RX mode (typically MODE_RX_ASYNC). */
	if (data->modem_mode != MODE_RX_ASYNC && data->modem_mode != MODE_RX) {
		return -EINVAL;
	}

	status = ral_get_rssi_inst(&config->ralf.ral, rssi);
	return (status == RAL_STATUS_OK) ? 0 : -EIO;
}

int lbm_lora_cad(const struct device *dev, k_timeout_t timeout)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	struct k_poll_signal done = K_POLL_SIGNAL_INITIALIZER(done);
	struct k_poll_event evt =
		K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL, K_POLL_MODE_NOTIFY_ONLY, &done);
	ral_status_t status;
	bool detected = false;
	int ret;

	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	/* Configure modem for CAD */
	lbm_driver_antenna_configure(dev, MODE_CAD);
	data->modem_mode = MODE_CAD;

	/* Validate that we have a LoRa configuration */
	if (data->mod_params.sf == 0) {
		ret = -EINVAL;
		goto release;
	}

	/* Store signal and sync result pointer */
	data->operation_done = &done;
	data->cad_operation = LBM_CAD_OPERATION_SYNC;
	data->cad_state.sync.detected = &detected;

	ret = lbm_configure_cad(dev, timeout);
	if (ret < 0) {
		goto release;
	}

	/* Clear any stale IRQ flags from the previous operation, then re-arm
	 * DIO1 so the CAD-done interrupt is delivered.
	 */
	(void)ral_clear_irq_status(&config->ralf.ral, RAL_IRQ_ALL);
	lbm_optional_dio1_irq_configure_dt(&config->dio1, GPIO_INT_EDGE_TO_ACTIVE);

	status = ral_set_lora_cad(&config->ralf.ral);
	if (status != RAL_STATUS_OK) {
		ret = -EIO;
		goto release;
	}

	ret = k_poll(&evt, 1, timeout);
	if (ret < 0) {
		if (modem_release(dev)) {
			LOG_INF("CAD timeout");
			data->cad_state.sync.detected = NULL;
			return -ETIMEDOUT;
		}
		/* CAD callback is currently running. Wait for completion. */
		k_poll(&evt, 1, K_FOREVER);
	}

	if (done.result != 0) {
		ret = done.result;
		goto release;
	}

	ret = detected ? 1 : 0;

release:
	data->cad_state.sync.detected = NULL;
	data->cad_operation = LBM_CAD_OPERATION_SYNC;
	modem_release(dev);
	return ret;
}

int lbm_lora_cad_async(const struct device *dev, lora_cad_cb cb, void *user_data)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;
	int ret;

	/* Cancel ongoing CAD */
	if (cb == NULL) {
		if (data->modem_mode != MODE_CAD || data->operation_done != NULL ||
		    data->cad_operation != LBM_CAD_OPERATION_ASYNC ||
		    data->cad_state.async.cad_cb == NULL) {
			return -EINVAL;
		}
		/* Prevent any in-flight work from invoking the callback. */
		data->cad_state.async.cad_cb = NULL;
		data->cad_state.async.user_data = NULL;

		if (!modem_release(dev)) {
			return -EINVAL;
		}
		data->cad_operation = LBM_CAD_OPERATION_SYNC;
		return 0;
	}

	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	/* Configure modem for CAD */
	lbm_driver_antenna_configure(dev, MODE_CAD);
	data->modem_mode = MODE_CAD;

	/* Validate that we have a LoRa configuration */
	if (data->mod_params.sf == 0) {
		ret = -EINVAL;
		goto release;
	}

	/* Store user state */
	data->operation_done = NULL;
	data->cad_operation = LBM_CAD_OPERATION_ASYNC;
	data->cad_state.async.cad_cb = cb;
	data->cad_state.async.user_data = user_data;

	ret = lbm_configure_cad(dev, lbm_cad_timeout_auto(dev));
	if (ret < 0) {
		goto release;
	}

	/* Clear any stale IRQ flags from the previous operation, then re-arm
	 * DIO1 so the CAD-done interrupt is delivered.
	 */
	(void)ral_clear_irq_status(&config->ralf.ral, RAL_IRQ_ALL);
	lbm_optional_dio1_irq_configure_dt(&config->dio1, GPIO_INT_EDGE_TO_ACTIVE);

	status = ral_set_lora_cad(&config->ralf.ral);
	if (status != RAL_STATUS_OK) {
		ret = -EIO;
		goto release;
	}

	return 0;

release:
	data->cad_state.async.cad_cb = NULL;
	data->cad_state.async.user_data = NULL;
	data->cad_operation = LBM_CAD_OPERATION_SYNC;
	modem_release(dev);
	return ret;
}

int lbm_lora_test_cw(const struct device *dev, uint32_t frequency, int8_t tx_power,
		     uint16_t duration)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;
	int ret = 0;

	/* Ensure available, freed by op_done_work */
	if (!modem_acquire(dev)) {
		return -EBUSY;
	}

	/* Configure modem for CW */
	lbm_driver_antenna_configure(dev, MODE_CW);
	data->modem_mode = MODE_CW;

	/* Invalidate stored config */
	data->mod_params.sf = 0;

	/* Configure continuous wave */
	status = ral_set_pkt_type(&config->ralf.ral, RAL_PKT_TYPE_LORA);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("RAL set packet type failed (%d)", status);
		ret = -EIO;
		goto release;
	}
	status = ral_set_rf_freq(&config->ralf.ral, frequency);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("RAL set RF frequency failed (%d)", status);
		ret = -EIO;
		goto release;
	}
	status = ral_set_tx_cfg(&config->ralf.ral, tx_power, frequency);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("RAL set TX config failed (%d)", status);
		ret = -EIO;
		goto release;
	}

	/* Start the continuous wave transmission */
	status = ral_set_tx_cw(&config->ralf.ral);
	if (status != RAL_STATUS_OK) {
		ret = -EIO;
		goto release;
	}

	/* Schedule the end of the transmission */
	k_work_reschedule(&data->op_done_work, K_SECONDS(duration));
	return 0;
release:
	modem_release(dev);
	return ret;
}

static int op_done_sync_rx(const struct device *dev)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_lora_rx_pkt_status_t pkt_status;
	ral_status_t status;
	int ret;

	/* Retrieve packet information before putting modem into sleep mode */
	status = ral_get_pkt_payload(&config->ralf.ral, data->rx_state.sync.msg_len,
				     data->rx_state.sync.msg, &data->rx_state.sync.msg_len);
	if (status == RAL_STATUS_OK) {
		LOG_HEXDUMP_DBG(data->rx_state.sync.msg, data->rx_state.sync.msg_len, "RX");
		ret = 0;
	} else {
		LOG_ERR("Failed to retrieve packet payload");
		ret = -EIO;
	}

	status = ral_get_lora_rx_pkt_status(&config->ralf.ral, &pkt_status);
	if (status == RAL_STATUS_OK) {
		data->rx_state.sync.rssi_dbm = pkt_status.signal_rssi_pkt_in_dbm;
		data->rx_state.sync.snr_db = pkt_status.snr_pkt_in_db;
	} else {
		LOG_WRN("Failed to query packet signal stats");
		data->rx_state.sync.rssi_dbm = INT16_MIN;
		data->rx_state.sync.snr_db = INT8_MIN;
	}

	return ret;
}

static void op_done_async_rx(const struct device *dev)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_lora_rx_pkt_status_t pkt_status;
	uint8_t rx_buffer[CONFIG_LORA_BASICS_MODEM_ASYNC_RX_MAX_PAYLOAD];
	ral_status_t status;
	uint16_t size;
	int16_t rssi = INT16_MIN;
	int8_t snr = INT8_MIN;

	/* Retrieve the packet payload */
	status = ral_get_pkt_payload(&config->ralf.ral, sizeof(rx_buffer), rx_buffer, &size);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("Failed to retrieve packet payload");
		return;
	}
	LOG_HEXDUMP_DBG(rx_buffer, size, "RX");

	/* Retrieve packet parameters */
	status = ral_get_lora_rx_pkt_status(&config->ralf.ral, &pkt_status);
	if (status != RAL_STATUS_OK) {
		LOG_WRN("Failed to query packet signal stats");
	} else {
		rssi = IS_ENABLED(CONFIG_LORA_BASICS_MODEM_RSSI_REPORT_TYPE_PACKET)
			       ? pkt_status.rssi_pkt_in_dbm
			       : pkt_status.signal_rssi_pkt_in_dbm;
		snr = pkt_status.snr_pkt_in_db;
	}

	/* Run the user callback */
	data->rx_state.async.rx_cb(dev, rx_buffer, size, rssi, snr,
				   data->rx_state.async.user_data);
}

static void op_done_sync_cad(const struct device *dev, bool cad_detected)
{
	struct lbm_lora_data_common *data = dev->data;

	/* Synchronous CAD: cad_state holds a bool* result. */
	if (data->cad_state.sync.detected != NULL) {
		*data->cad_state.sync.detected = cad_detected;
	}
	data->cad_state.sync.detected = NULL;
}

static void op_done_async_cad(const struct device *dev, bool cad_detected, lora_cad_cb cad_cb,
			      void *cad_user_data)
{
	if (cad_cb != NULL) {
		cad_cb(dev, cad_detected, cad_user_data);
	}
}

static void op_done_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct lbm_lora_data_common *data =
		CONTAINER_OF(dwork, struct lbm_lora_data_common, op_done_work);
	const struct device *dev = data->dev;
	const struct lbm_lora_config_common *config = dev->config;
	struct k_poll_signal *sig_done;
	lora_cad_cb cad_cb = NULL;
	void *cad_user_data = NULL;
	ral_irq_t irq_state;
	ral_status_t status;
	bool cad_detected = false;
	bool notify_cad_async = false;
	bool release = false;
	bool error_irq;
	int ret = 0;

	LOG_DBG("%d", data->modem_mode);

	if (data->modem_mode == MODE_SLEEP) {
		/* DIO1 was disabled by modem_release() before the chip went to
		 * sleep. This work item was already queued when DIO1 fired. The
		 * modem is now idle, just return without touching the hardware.
		 */
		return;
	}

	/* Get and reset the current IRQ state. */
	(void)ral_get_irq_status(&config->ralf.ral, &irq_state);
	(void)ral_clear_irq_status(&config->ralf.ral, RAL_IRQ_ALL);
	error_irq = irq_state & (RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR);

	switch (data->modem_mode) {
	case MODE_SLEEP:
		LOG_WRN("Unexpected modem mode (%d)", data->modem_mode);
		return;
	case MODE_TX:
	case MODE_CW:
		status = ral_handle_tx_done(&config->ralf.ral);
		if (status != RAL_STATUS_OK) {
			LOG_WRN("RAL handle TX done failed (%d)", status);
		}
		release = true;
		break;
	case MODE_RX:
		if (irq_state & (RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
				 RAL_IRQ_RX_CRC_ERROR)) {
			status = ral_handle_rx_done(&config->ralf.ral);
			if (status != RAL_STATUS_OK) {
				LOG_WRN("RAL handle RX done failed (%d)", status);
			}
		}

		if (irq_state & RAL_IRQ_RX_DONE) {
			ret = op_done_sync_rx(dev);
			release = true;
		} else if (data->rx_started_with_duty_cycle &&
			   ((irq_state & RAL_IRQ_RX_TIMEOUT) != 0U) &&
			   ((irq_state & (RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR)) == 0U)) {
			/* Keep waiting across RX duty-cycle windows until k_poll timeout
			 * expires.
			 */
			release = false;
		} else {
			release = true;
		}
		break;
	case MODE_RX_ASYNC: {
		bool rearm_rx_duty_cycle = false;

		if (irq_state & (RAL_IRQ_RX_DONE | RAL_IRQ_RX_TIMEOUT | RAL_IRQ_RX_HDR_ERROR |
				 RAL_IRQ_RX_CRC_ERROR)) {
			status = ral_handle_rx_done(&config->ralf.ral);
			if (status != RAL_STATUS_OK) {
				LOG_WRN("RAL handle RX done failed (%d)", status);
			}
		}

		if ((irq_state & RAL_IRQ_RX_DONE) && (data->rx_state.async.rx_cb != NULL)) {
			op_done_async_rx(dev);
			rearm_rx_duty_cycle = data->rx_started_with_duty_cycle;
		} else if (data->rx_started_with_duty_cycle &&
			   ((irq_state & (RAL_IRQ_RX_HDR_ERROR | RAL_IRQ_RX_CRC_ERROR)) != 0U)) {
			/* In duty-cycle RX mode, RX done/error IRQs end the active receive window.
			 * Rearm duty-cycle RX so asynchronous reception keeps running.
			 */
			rearm_rx_duty_cycle = true;
		}

		if (rearm_rx_duty_cycle && (atomic_get(&data->modem_state) == STATE_BUSY)) {
			status = modem_set_rx(dev);
			if (status != RAL_STATUS_OK) {
				LOG_WRN("RAL rearm RX duty-cycle failed (%d)", status);
				release = true;
			}
		}

		/* Don't release the modem here, RX continues */
		break;
	}
	case MODE_CAD:
		if (irq_state & (RAL_IRQ_CAD_DONE | RAL_IRQ_CAD_OK)) {
			cad_detected = (irq_state & RAL_IRQ_CAD_OK) != 0;

			if ((data->cad_operation == LBM_CAD_OPERATION_SYNC) &&
			    (data->operation_done != NULL)) {
				op_done_sync_cad(dev, cad_detected);
			} else if (data->cad_operation == LBM_CAD_OPERATION_ASYNC) {
				/* Preserve callback state before releasing the modem. */
				cad_cb = data->cad_state.async.cad_cb;
				cad_user_data = data->cad_state.async.user_data;
				data->cad_state.async.cad_cb = NULL;
				data->cad_state.async.user_data = NULL;
				notify_cad_async = true;
			}

			data->cad_operation = LBM_CAD_OPERATION_SYNC;
			release = true;
		}
		break;
	}

	/* Release the modem before running the user callback so that the notified thread can
	 * immediately start another operation before the work item terminates. This requires
	 * preserving the operation_done pointer, since modem_release clears it.
	 */
	sig_done = data->operation_done;

	/* Modem should return to idle */
	if (release) {
		/* Return to sleep mode */
		modem_release(dev);
	}

	/* Notify user that operation has completed */
	if (sig_done && release) {
		k_poll_signal_raise(sig_done, error_irq ? -EAGAIN : ret);
	}

	if (notify_cad_async) {
		op_done_async_cad(dev, cad_detected, cad_cb, cad_user_data);
	}
}

int lbm_lora_common_init(const struct device *dev)
{
	const struct lbm_lora_config_common *config = dev->config;
	struct lbm_lora_data_common *data = dev->data;
	ral_status_t status;

	data->dev = dev;
	data->rx_boosted = RX_BOOST_DEFAULT;
	data->cad_mode = LORA_CAD_MODE_NONE;
	data->cad_symbol_num = 0;
	data->cad_detection_peak = 0U;
	data->cad_detection_minimum = 0U;
	data->cad_operation = LBM_CAD_OPERATION_SYNC;
	data->cad_state.sync.detected = NULL;
	data->duty_cycle_enabled = false;
	data->duty_cycle_rx_time_ms = 0U;
	data->duty_cycle_sleep_time_ms = 0U;
	data->rx_started_with_duty_cycle = false;
	k_work_init_delayable(&data->op_done_work, op_done_work_handler);
	atomic_clear(&data->modem_state);

	/* Initialise the radio abstraction layer */
	status = ral_init(&config->ralf.ral);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("RAL init failure (%d)", status);
		return -EIO;
	}

	/* Enable all relevant interrupts */
	status = ral_set_dio_irq_params(&config->ralf.ral, RAL_IRQ_LORA);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("RAL DIO init failure (%d)", status);
		return -EIO;
	}

	/* Idle in sleep mode */
	status = ral_set_sleep(&config->ralf.ral, true);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("Sleep failure (%d)", status);
		return -EIO;
	}
	return 0;
}

DEVICE_API(lora, lbm_lora_api) = {
	.config = lbm_lora_config,
	.airtime = lbm_lora_airtime,
	.send = lbm_lora_send,
	.send_async = lbm_lora_send_async,
	.recv = lbm_lora_recv,
	.recv_async = lbm_lora_recv_async,
	.cad = lbm_lora_cad,
	.cad_async = lbm_lora_cad_async,
	.recv_duty_cycle_async = lbm_lora_recv_duty_cycle_async,
	.recv_duty_cycle = lbm_lora_recv_duty_cycle,
	.rssi_inst = lbm_lora_rssi_inst,
	.test_cw = lbm_lora_test_cw,
};
