/*
 * Copyright (c) 2026 FoBE Studio
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include "ral.h"

#include "lbm_common.h"
#include "lr11xx_hal.h"
#include "lr11xx_radio.h"
#include "lr11xx_radio_types.h"
#include "lr11xx_system_types.h"
#include "ral_lr11xx_bsp.h"
#include "ralf_lr11xx.h"

#define LR11XX_SYSTEM_SET_SLEEP_OC_MSB 0x01
#define LR11XX_SYSTEM_SET_SLEEP_OC_LSB 0x1B
#define LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC_MSB 0x02
#define LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC_LSB 0x08
#define LR11XX_RADIO_SET_LORA_SYNC_WORD_OC_MSB 0x02
#define LR11XX_RADIO_SET_LORA_SYNC_WORD_OC_LSB 0x2B
#define LR11XX_RADIO_SET_RSSI_CALIBRATION_OC_MSB 0x02
#define LR11XX_RADIO_SET_RSSI_CALIBRATION_OC_LSB 0x29

#define LR11XX_BUSY_TIMEOUT K_SECONDS(1)
#define LR11XX_RESET_READY_DELAY K_MSEC(200)
#define LR11XX_SLEEP_ENTRY_DELAY K_USEC(500)
#define LR11XX_MAX_READ_DATA_LEN 256U
#define LR11XX_MAX_READ_TRANSFER_LEN (LR11XX_MAX_READ_DATA_LEN + 2U)

#define LR11XX_LP_MIN_OUTPUT_POWER -17
#define LR11XX_LP_MAX_OUTPUT_POWER 15
#define LR11XX_HP_MIN_OUTPUT_POWER -9
#define LR11XX_HP_MAX_OUTPUT_POWER 22
#define LR11XX_HF_MIN_OUTPUT_POWER -18
#define LR11XX_HF_MAX_OUTPUT_POWER 13

enum lbm_lr11xx_legacy_lora_network_state {
	LR11XX_LEGACY_LORA_NETWORK_UNKNOWN,
	LR11XX_LEGACY_LORA_NETWORK_PRIVATE,
	LR11XX_LEGACY_LORA_NETWORK_PUBLIC,
};

struct lbm_lr11xx_config {
	struct lbm_lora_config_common lbm_common;
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec ant_enable;
	struct gpio_dt_spec tx_enable;
	struct gpio_dt_spec rx_enable;
	lr11xx_system_rfswitch_cfg_t rfswitch;
	uint8_t tcxo_voltage;
	lr11xx_radio_pa_cfg_t pa_cfg;
	int tcxo_startup_delay_ms;
	int tx_power_offset_db;
	bool crc_over_spi;
	bool rx_boosted;
	bool skip_rssi_calibration;
	bool legacy_lora_public_network;
	bool use_dcdc;
	bool lfclk_in_sleep;
};

struct lbm_lr11xx_data {
	struct lbm_lora_data_common lbm_common;
	const struct device *dev;
	struct gpio_callback dio1_callback;
	enum lbm_lr11xx_legacy_lora_network_state legacy_lora_network_state;
	bool asleep;
};

LOG_MODULE_DECLARE(lbm_driver, CONFIG_LORA_LOG_LEVEL);

static bool lr11xx_is_busy(const struct device *dev)
{
	const struct lbm_lr11xx_config *config = dev->config;

	return gpio_pin_get_dt(&config->busy);
}

static int lr11xx_wait_device_ready(const struct device *dev, k_timeout_t timeout)
{
	k_timepoint_t expiry = sys_timepoint_calc(timeout);

	do {
		if (!lr11xx_is_busy(dev)) {
			return 0;
		}
		k_sleep(K_MSEC(1));
	} while (!sys_timepoint_expired(expiry));

	return -EAGAIN;
}

static int lr11xx_wakeup_sequence(const struct device *dev, k_timeout_t timeout)
{
	const struct lbm_lr11xx_config *config = dev->config;
	const struct spi_cs_control *cs = &config->spi.config.cs;
	int ret;

	if (cs->cs_is_gpio && (cs->gpio.port != NULL)) {
		/* gpio_pin_set_dt uses logical values: 1 = active, 0 = inactive. */
		ret = gpio_pin_set_dt(&cs->gpio, 1);
		if (ret) {
			return ret;
		}

		k_busy_wait(1);
		ret = gpio_pin_set_dt(&cs->gpio, 0);
		if (ret) {
			return ret;
		}
		return lr11xx_wait_device_ready(dev, timeout);
	}

	uint8_t nop = LR11XX_NOP;
	const struct spi_buf tx_bufs[] = {
		{
			.buf = &nop,
			.len = sizeof(nop),
		},
	};
	const struct spi_buf_set tx_buf_set = {.buffers = tx_bufs,
					       .count = ARRAY_SIZE(tx_bufs)};

	ret = spi_write_dt(&config->spi, &tx_buf_set);
	if (ret) {
		return ret;
	}

	return lr11xx_wait_device_ready(dev, timeout);
}

static bool lr11xx_is_sleep_command(const uint8_t *command, uint16_t command_length)
{
	return command_length >= 2U && command[0] == LR11XX_SYSTEM_SET_SLEEP_OC_MSB &&
	       command[1] == LR11XX_SYSTEM_SET_SLEEP_OC_LSB;
}

static int lr11xx_ensure_device_ready(const struct device *dev, k_timeout_t timeout)
{
	const struct lbm_lr11xx_config *config = dev->config;
	struct lbm_lr11xx_data *data = dev->data;
	int ret;

	if (data->asleep) {
		LOG_DBG("SLEEP -> ACTIVE");
		(void)gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
		ret = lr11xx_wakeup_sequence(dev, timeout);
		if (ret) {
			return ret;
		}
		data->asleep = false;
	}

	ret = lr11xx_wait_device_ready(dev, timeout);
	data->asleep = false;
	return ret;
}

static int lr11xx_prepare_for_command(const struct device *dev, const uint8_t *command,
				      uint16_t command_length, k_timeout_t timeout)
{
	struct lbm_lr11xx_data *data = dev->data;
	int ret;

	if (!data->asleep && data->lbm_common.rx_started_with_duty_cycle && lr11xx_is_busy(dev)) {
		LOG_DBG("Waking from RX duty-cycle before command");
		ret = lr11xx_wakeup_sequence(dev, timeout);
		if (ret) {
			return ret;
		}
		data->asleep = false;
	}

	ret = lr11xx_ensure_device_ready(dev, timeout);
	if (ret && lr11xx_is_sleep_command(command, command_length) &&
	    data->lbm_common.rx_started_with_duty_cycle && lr11xx_is_busy(dev)) {
		LOG_DBG("Waking from RX duty-cycle before sleep command");
		ret = lr11xx_wakeup_sequence(dev, timeout);
		if (ret == 0) {
			data->asleep = false;
		}
	}

	return ret;
}

static lr11xx_hal_status_t lr11xx_ret_to_hal_status(int ret)
{
	return ret == 0 ? LR11XX_HAL_STATUS_OK : LR11XX_HAL_STATUS_ERROR;
}

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
				     const uint16_t command_length, const uint8_t *data,
				     const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	struct lbm_lr11xx_data *dev_data = dev->data;
	const uint8_t *write_command = command;
	uint16_t write_command_length = command_length;
	uint8_t legacy_lora_network_cmd[3];
	uint8_t crc;
	int ret;

	if (config->skip_rssi_calibration && command_length >= 2U &&
	    command[0] == LR11XX_RADIO_SET_RSSI_CALIBRATION_OC_MSB &&
	    command[1] == LR11XX_RADIO_SET_RSSI_CALIBRATION_OC_LSB) {
		LOG_DBG("Skipping RSSI calibration command");
		return LR11XX_HAL_STATUS_OK;
	}

	if (config->legacy_lora_public_network && command_length == 3U &&
	    command[0] == LR11XX_RADIO_SET_LORA_SYNC_WORD_OC_MSB &&
	    command[1] == LR11XX_RADIO_SET_LORA_SYNC_WORD_OC_LSB) {
		legacy_lora_network_cmd[0] = LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC_MSB;
		legacy_lora_network_cmd[1] = LR11XX_RADIO_SET_LORA_PUBLIC_NETWORK_OC_LSB;

		if (command[2] == LBM_LORA_SYNC_WORD_PRIVATE) {
			if (dev_data->legacy_lora_network_state !=
			    LR11XX_LEGACY_LORA_NETWORK_PRIVATE) {
				LOG_ERR("Cannot apply legacy private LoRa sync word unless "
					"radio state is private");
				return LR11XX_HAL_STATUS_ERROR;
			}

			LOG_DBG("Skipping legacy private LoRa sync word; radio is already private");
			ret = lr11xx_prepare_for_command(dev, command, command_length,
							 LR11XX_BUSY_TIMEOUT);
			if (ret) {
				LOG_ERR("LR11XX not ready before skipping private "
					"LoRa sync word: %d",
					ret);
				return LR11XX_HAL_STATUS_ERROR;
			}
			return LR11XX_HAL_STATUS_OK;
		} else if (command[2] == LBM_LORA_SYNC_WORD_PUBLIC) {
			legacy_lora_network_cmd[2] = LR11XX_RADIO_LORA_NETWORK_PUBLIC;
		} else {
			LOG_ERR("Unsupported legacy LoRa sync word 0x%02x", command[2]);
			return LR11XX_HAL_STATUS_ERROR;
		}

		LOG_DBG("Remapping LoRa sync word 0x%02x to legacy public-network command",
			command[2]);
		write_command = legacy_lora_network_cmd;
		write_command_length = sizeof(legacy_lora_network_cmd);
	}

	LOG_DBG("CMD=0x%02x%02x CMD_LEN=%d DATA_LEN=%d", write_command[0], write_command[1],
		write_command_length, data_length);

	ret = lr11xx_prepare_for_command(dev, command, command_length, LR11XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR11XX not ready before write CMD=0x%02x%02x: %d", write_command[0],
			write_command[1], ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf tx_bufs[] = {
		{
			.buf = (void *)write_command,
			.len = write_command_length,
		},
		{
			.buf = (void *)data,
			.len = data_length,
		},
		{
			.buf = &crc,
			.len = config->crc_over_spi ? sizeof(crc) : 0U,
		},
	};
	const struct spi_buf_set tx_buf_set = {.buffers = tx_bufs,
					       .count = ARRAY_SIZE(tx_bufs)};

	if (config->crc_over_spi) {
		crc = lr11xx_hal_compute_crc(0xFF, write_command, write_command_length);
		crc = lr11xx_hal_compute_crc(crc, data, data_length);
	}

	ret = spi_write_dt(&config->spi, &tx_buf_set);
	if (ret) {
		LOG_ERR("LR11XX SPI write failed CMD=0x%02x%02x: %d", write_command[0],
			write_command[1], ret);
		return LR11XX_HAL_STATUS_ERROR;
	}
	LOG_DBG("CMD=0x%02x%02x write done", write_command[0], write_command[1]);

	if (config->legacy_lora_public_network && command_length == 3U &&
	    command[0] == LR11XX_RADIO_SET_LORA_SYNC_WORD_OC_MSB &&
	    command[1] == LR11XX_RADIO_SET_LORA_SYNC_WORD_OC_LSB &&
	    command[2] == LBM_LORA_SYNC_WORD_PUBLIC) {
		dev_data->legacy_lora_network_state = LR11XX_LEGACY_LORA_NETWORK_PUBLIC;
	}

	if (lr11xx_is_sleep_command(command, command_length)) {
		LOG_DBG("ACTIVE -> SLEEP");
		(void)gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_DISABLE);
		dev_data->asleep = true;
		k_sleep(LR11XX_SLEEP_ENTRY_DELAY);
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length, uint8_t *data,
				    const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	uint8_t crc;
	int ret;

	LOG_DBG("CMD=0x%02x%02x CMD_LEN=%d DATA_LEN=%d", command[0], command[1], command_length,
		data_length);

	if (data_length > LR11XX_MAX_READ_DATA_LEN) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	ret = lr11xx_ensure_device_ready(dev, LR11XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR11XX not ready before read CMD=0x%02x%02x: %d", command[0], command[1],
			ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf cmd_tx_bufs[] = {
		{
			.buf = (void *)command,
			.len = command_length,
		},
		{
			.buf = &crc,
			.len = config->crc_over_spi ? sizeof(crc) : 0U,
		},
	};
	const struct spi_buf_set cmd_tx_buf_set = {.buffers = cmd_tx_bufs,
						   .count = ARRAY_SIZE(cmd_tx_bufs)};

	if (config->crc_over_spi) {
		crc = lr11xx_hal_compute_crc(0xFF, command, command_length);
	}

	ret = spi_write_dt(&config->spi, &cmd_tx_buf_set);
	if (ret) {
		LOG_ERR("LR11XX SPI read-command write failed CMD=0x%02x%02x: %d", command[0],
			command[1], ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	if (data_length == 0U) {
		return LR11XX_HAL_STATUS_OK;
	}

	ret = lr11xx_ensure_device_ready(dev, LR11XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR11XX not ready before read payload CMD=0x%02x%02x: %d", command[0],
			command[1], ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	uint8_t nop[LR11XX_MAX_READ_TRANSFER_LEN] = {0};
	uint8_t rx[LR11XX_MAX_READ_TRANSFER_LEN] = {0};
	const size_t read_len = data_length + 1U + (config->crc_over_spi ? 1U : 0U);
	const struct spi_buf read_tx_bufs[] = {
		{
			.buf = nop,
			.len = read_len,
		},
	};
	const struct spi_buf read_rx_bufs[] = {
		{
			.buf = rx,
			.len = read_len,
		},
	};
	const struct spi_buf_set read_tx_buf_set = {.buffers = read_tx_bufs,
						    .count = ARRAY_SIZE(read_tx_bufs)};
	const struct spi_buf_set read_rx_buf_set = {.buffers = read_rx_bufs,
						    .count = ARRAY_SIZE(read_rx_bufs)};

	ret = spi_transceive_dt(&config->spi, &read_tx_buf_set, &read_rx_buf_set);
	if (ret) {
		LOG_ERR("LR11XX SPI read payload failed CMD=0x%02x%02x: %d", command[0], command[1],
			ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	memcpy(data, &rx[1], data_length);

	if (config->crc_over_spi) {
		crc = lr11xx_hal_compute_crc(0xFF, rx, data_length + 1U);
		if (crc != rx[data_length + 1U]) {
			LOG_ERR("LR11XX read CRC mismatch");
			return LR11XX_HAL_STATUS_ERROR;
		}
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
					   const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	int ret;

	LOG_DBG("DATA_LEN=%d", data_length);

	if (data_length > LR11XX_MAX_READ_DATA_LEN) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	ret = lr11xx_ensure_device_ready(dev, LR11XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR11XX not ready before direct read: %d", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	uint8_t nop[LR11XX_MAX_READ_TRANSFER_LEN] = {0};
	uint8_t rx[LR11XX_MAX_READ_TRANSFER_LEN] = {0};
	const size_t read_len = data_length + (config->crc_over_spi ? 1U : 0U);
	const struct spi_buf tx_bufs[] = {
		{
			.buf = nop,
			.len = read_len,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = rx,
			.len = read_len,
		},
	};
	const struct spi_buf_set tx_buf_set = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	const struct spi_buf_set rx_buf_set = {.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

	ret = spi_transceive_dt(&config->spi, &tx_buf_set, &rx_buf_set);
	if (ret) {
		LOG_ERR("LR11XX SPI direct read failed: %d", ret);
		return LR11XX_HAL_STATUS_ERROR;
	}

	memcpy(data, rx, data_length);

	if (config->crc_over_spi) {
		uint8_t crc = lr11xx_hal_compute_crc(0xFF, data, data_length);

		if (crc != rx[data_length]) {
			LOG_ERR("LR11XX direct read CRC mismatch");
			return LR11XX_HAL_STATUS_ERROR;
		}
	}

	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	struct lbm_lr11xx_data *data = dev->data;

	LOG_DBG("");

	gpio_pin_set_dt(&config->reset, 1);
	k_sleep(K_MSEC(5));
	gpio_pin_set_dt(&config->reset, 0);
	k_sleep(LR11XX_RESET_READY_DELAY);

	data->asleep = false;
	data->legacy_lora_network_state = LR11XX_LEGACY_LORA_NETWORK_PRIVATE;
	return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
	const struct device *dev = context;

	return lr11xx_ret_to_hal_status(lr11xx_ensure_device_ready(dev, LR11XX_BUSY_TIMEOUT));
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd(const void *context)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	uint8_t command[4] = {0};
	const struct spi_buf tx_bufs[] = {
		{
			.buf = command,
			.len = sizeof(command),
		},
	};
	const struct spi_buf_set tx_buf_set = {.buffers = tx_bufs, .count = ARRAY_SIZE(tx_bufs)};
	int ret;

	ret = spi_write_dt(&config->spi, &tx_buf_set);
	if (ret) {
		return LR11XX_HAL_STATUS_ERROR;
	}

	return lr11xx_ret_to_hal_status(lr11xx_wait_device_ready(dev, LR11XX_BUSY_TIMEOUT));
}

void ral_lr11xx_bsp_get_rf_switch_cfg(const void *context,
				      lr11xx_system_rfswitch_cfg_t *rf_switch_cfg)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;

	*rf_switch_cfg = config->rfswitch;
}

void ral_lr11xx_bsp_get_tx_cfg(const void *context,
			       const ral_lr11xx_bsp_tx_cfg_input_params_t *input_params,
			       ral_lr11xx_bsp_tx_cfg_output_params_t *output_params)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	int16_t power = input_params->system_output_pwr_in_dbm + config->tx_power_offset_db;

	output_params->pa_cfg = config->pa_cfg;
	output_params->pa_ramp_time = LR11XX_RADIO_RAMP_48_US;

	switch (config->pa_cfg.pa_sel) {
	case LR11XX_RADIO_PA_SEL_LP:
		power = CLAMP(power, LR11XX_LP_MIN_OUTPUT_POWER, LR11XX_LP_MAX_OUTPUT_POWER);
		break;
	case LR11XX_RADIO_PA_SEL_HF:
		power = CLAMP(power, LR11XX_HF_MIN_OUTPUT_POWER, LR11XX_HF_MAX_OUTPUT_POWER);
		break;
	case LR11XX_RADIO_PA_SEL_HP:
	default:
		power = CLAMP(power, LR11XX_HP_MIN_OUTPUT_POWER, LR11XX_HP_MAX_OUTPUT_POWER);
		break;
	}

	output_params->chip_output_pwr_in_dbm_configured = power;
	output_params->chip_output_pwr_in_dbm_expected = power - config->tx_power_offset_db;
}

void ral_lr11xx_bsp_get_reg_mode(const void *context, lr11xx_system_reg_mode_t *reg_mode)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;

	*reg_mode = config->use_dcdc ? LR11XX_SYSTEM_REG_MODE_DCDC : LR11XX_SYSTEM_REG_MODE_LDO;
}

void ral_lr11xx_bsp_get_xosc_cfg(const void *context, ral_xosc_cfg_t *xosc_cfg,
				 lr11xx_system_tcxo_supply_voltage_t *supply_voltage,
				 uint32_t *startup_time_in_tick)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;

	if (config->tcxo_voltage > LR11XX_SYSTEM_TCXO_CTRL_3_3V) {
		*xosc_cfg = RAL_XOSC_CFG_XTAL;
		*supply_voltage = LR11XX_SYSTEM_TCXO_CTRL_1_8V;
		*startup_time_in_tick = 0U;
		return;
	}

	*xosc_cfg = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
	*supply_voltage = (lr11xx_system_tcxo_supply_voltage_t)config->tcxo_voltage;
	*startup_time_in_tick =
		lr11xx_radio_convert_time_in_ms_to_rtc_step(config->tcxo_startup_delay_ms);
}

void ral_lr11xx_bsp_get_crc_state(const void *context, bool *crc_is_activated)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;

	*crc_is_activated = config->crc_over_spi;
}

void ral_lr11xx_bsp_get_rssi_calibration_table(
	const void *context, const uint32_t freq_in_hz,
	lr11xx_radio_rssi_calibration_table_t *rssi_calibration_table)
{
	if (freq_in_hz <= 600000000U) {
		*rssi_calibration_table = (lr11xx_radio_rssi_calibration_table_t){
			.gain_tune = {.g4 = 12, .g5 = 12, .g6 = 14, .g7 = 0, .g8 = 1,
				      .g9 = 3, .g10 = 4, .g11 = 4, .g12 = 3, .g13 = 6,
				      .g13hp1 = 6, .g13hp2 = 6, .g13hp3 = 6, .g13hp4 = 6,
				      .g13hp5 = 6, .g13hp6 = 6, .g13hp7 = 6},
			.gain_offset = 0,
		};
	} else if (freq_in_hz <= 2000000000U) {
		*rssi_calibration_table = (lr11xx_radio_rssi_calibration_table_t){
			.gain_tune = {.g4 = 2, .g5 = 2, .g6 = 2, .g7 = 3, .g8 = 3,
				      .g9 = 4, .g10 = 5, .g11 = 4, .g12 = 4, .g13 = 6,
				      .g13hp1 = 5, .g13hp2 = 5, .g13hp3 = 6, .g13hp4 = 6,
				      .g13hp5 = 6, .g13hp6 = 7, .g13hp7 = 6},
			.gain_offset = 0,
		};
	} else {
		*rssi_calibration_table = (lr11xx_radio_rssi_calibration_table_t){
			.gain_tune = {.g4 = 6, .g5 = 7, .g6 = 6, .g7 = 4, .g8 = 3,
				      .g9 = 4, .g10 = 14, .g11 = 12, .g12 = 14,
				      .g13 = 12, .g13hp1 = 12, .g13hp2 = 12,
				      .g13hp3 = 12, .g13hp4 = 8, .g13hp5 = 8,
				      .g13hp6 = 9, .g13hp7 = 9},
			.gain_offset = 2030,
		};
	}
}

void ral_lr11xx_bsp_get_lora_cad_det_peak(const void *context, ral_lora_sf_t sf, ral_lora_bw_t bw,
					  ral_lora_cad_symbs_t nb_symbol,
					  uint8_t *in_out_cad_det_peak)
{
	ARG_UNUSED(context);
	ARG_UNUSED(sf);
	ARG_UNUSED(bw);
	ARG_UNUSED(nb_symbol);
	ARG_UNUSED(in_out_cad_det_peak);
}

void ral_lr11xx_bsp_get_rx_boost_cfg(const void *context, bool *rx_boost_is_activated)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;
	const struct lbm_lr11xx_data *data = dev->data;

	switch (data->lbm_common.rx_boosted) {
	case RX_BOOST_DEFAULT:
		*rx_boost_is_activated = config->rx_boosted;
		break;
	case RX_BOOST_DISABLED:
		*rx_boost_is_activated = false;
		break;
	case RX_BOOST_ENABLED:
		*rx_boost_is_activated = true;
		break;
	default:
		*rx_boost_is_activated = config->rx_boosted;
		break;
	}
}

void ral_lr11xx_bsp_get_lfclk_cfg_in_sleep(const void *context, bool *lfclk_is_running)
{
	const struct device *dev = context;
	const struct lbm_lr11xx_config *config = dev->config;

	*lfclk_is_running = config->lfclk_in_sleep;
}

ral_status_t ral_lr11xx_bsp_get_instantaneous_tx_power_consumption(
	const void *context, const ral_lr11xx_bsp_tx_cfg_output_params_t *tx_cfg,
	lr11xx_system_reg_mode_t radio_reg_mode, uint32_t *pwr_consumption_in_ua)
{
	ARG_UNUSED(context);
	ARG_UNUSED(tx_cfg);
	ARG_UNUSED(radio_reg_mode);
	ARG_UNUSED(pwr_consumption_in_ua);
	return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_bsp_get_instantaneous_gfsk_rx_power_consumption(
	const void *context, lr11xx_system_reg_mode_t radio_reg_mode, bool rx_boosted,
	uint32_t *pwr_consumption_in_ua)
{
	ARG_UNUSED(context);
	ARG_UNUSED(radio_reg_mode);
	ARG_UNUSED(rx_boosted);
	ARG_UNUSED(pwr_consumption_in_ua);
	return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr11xx_bsp_get_instantaneous_lora_rx_power_consumption(
	const void *context, lr11xx_system_reg_mode_t radio_reg_mode, bool rx_boosted,
	uint32_t *pwr_consumption_in_ua)
{
	ARG_UNUSED(context);
	ARG_UNUSED(radio_reg_mode);
	ARG_UNUSED(rx_boosted);
	ARG_UNUSED(pwr_consumption_in_ua);
	return RAL_STATUS_UNSUPPORTED_FEATURE;
}

void lbm_driver_antenna_configure(const struct device *dev, enum lbm_modem_mode mode)
{
	const struct lbm_lr11xx_config *config = dev->config;

	switch (mode) {
	case MODE_SLEEP:
		lbm_optional_gpio_set_dt(&config->ant_enable, 0);
		lbm_optional_gpio_set_dt(&config->rx_enable, 0);
		lbm_optional_gpio_set_dt(&config->tx_enable, 0);
		break;
	case MODE_TX:
	case MODE_CW:
		lbm_optional_gpio_set_dt(&config->rx_enable, 0);
		lbm_optional_gpio_set_dt(&config->tx_enable, 1);
		lbm_optional_gpio_set_dt(&config->ant_enable, 1);
		break;
	case MODE_RX:
	case MODE_RX_ASYNC:
	case MODE_CAD:
		lbm_optional_gpio_set_dt(&config->tx_enable, 0);
		lbm_optional_gpio_set_dt(&config->rx_enable, 1);
		lbm_optional_gpio_set_dt(&config->ant_enable, 1);
		break;
	}
}

static void lr11xx_dio1_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct lbm_lr11xx_data *data = CONTAINER_OF(cb, struct lbm_lr11xx_data, dio1_callback);

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	k_work_schedule(&data->lbm_common.op_done_work, K_NO_WAIT);
}

int lbm_driver_add_dio1_gpio_callback(const struct device *dev, struct gpio_callback *callback,
				      gpio_callback_handler_t handler)
{
	const struct lbm_lr11xx_config *config = dev->config;
	int ret;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	if (callback == NULL || handler == NULL) {
		return -EINVAL;
	}

	gpio_init_callback(callback, handler, BIT(config->irq.pin));

	ret = gpio_add_callback(config->irq.port, callback);
	if (ret < 0) {
		LOG_ERR("Failed to add GPIO callback: %d", ret);
		return ret;
	}

	gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);

	return 0;
}

int lbm_driver_remove_dio1_gpio_callback(const struct device *dev, struct gpio_callback *callback)
{
	const struct lbm_lr11xx_config *config = dev->config;
	int ret;

	if (!device_is_ready(dev)) {
		return -ENODEV;
	}

	if (callback == NULL) {
		return -EINVAL;
	}

	ret = gpio_remove_callback(config->irq.port, callback);
	if (ret < 0) {
		LOG_ERR("Failed to remove GPIO callback: %d", ret);
		return ret;
	}

	return 0;
}

int lbm_driver_radio_init(const struct device *dev)
{
	const struct lbm_lr11xx_config *config = dev->config;
	struct lbm_lr11xx_data *data = dev->data;
	ral_status_t status;
	int ret;

	status = ral_reset(&config->lbm_common.ralf.ral);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("Reset failure (%d)", status);
		return -EIO;
	}

	ret = lr11xx_ensure_device_ready(dev, LR11XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to return to ready after reset");
		return -EIO;
	}

	ret = lbm_lora_common_init(dev);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->dio1_callback, lr11xx_dio1_callback, BIT(config->irq.pin));
	if (gpio_add_callback(config->irq.port, &data->dio1_callback) < 0) {
		LOG_ERR("Could not set GPIO callback for DIO1 interrupt.");
		return -EIO;
	}
	gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);

	LOG_INF("Radio initialized");
	return 0;
}

static int lr11xx_init(const struct device *dev)
{
	const struct lbm_lr11xx_config *config = dev->config;
	struct lbm_lr11xx_data *data = dev->data;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s not ready", config->spi.bus->name);
		return -ENODEV;
	}

	if (gpio_pin_configure_dt(&config->reset, GPIO_OUTPUT_INACTIVE) ||
	    gpio_pin_configure_dt(&config->busy, GPIO_INPUT) ||
	    gpio_pin_configure_dt(&config->irq, GPIO_INPUT)) {
		LOG_ERR("GPIO configuration failed.");
		return -EIO;
	}
	if (config->ant_enable.port) {
		gpio_pin_configure_dt(&config->ant_enable, GPIO_OUTPUT_INACTIVE);
	}
	if (config->tx_enable.port) {
		gpio_pin_configure_dt(&config->tx_enable, GPIO_OUTPUT_INACTIVE);
	}
	if (config->rx_enable.port) {
		gpio_pin_configure_dt(&config->rx_enable, GPIO_OUTPUT_INACTIVE);
	}

	data->dev = dev;

	if (!IS_ENABLED(CONFIG_LORA_BASICS_MODEM_DEFERRED_INIT)) {
		return lbm_driver_radio_init(dev);
	}

	LOG_INF("Device initialized (radio initialization deferred)");
	return 0;
}

#define LR11XX_DEFINE(node_id)                                                                 \
	static const struct lbm_lr11xx_config config_##node_id = {                             \
		.lbm_common.ralf = RALF_LR11XX_INSTANTIATE(DEVICE_DT_GET(node_id)),            \
		.lbm_common.force_ldro = DT_PROP(node_id, force_ldro),                         \
		.lbm_common.dio1 = GPIO_DT_SPEC_GET(node_id, irq_gpios),                       \
		.lbm_common.duty_cycle_supported = true,                                       \
		.spi = SPI_DT_SPEC_GET(node_id,                                                \
				       SPI_WORD_SET(8) | SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB), \
		.reset = GPIO_DT_SPEC_GET(node_id, reset_gpios),                               \
		.busy = GPIO_DT_SPEC_GET(node_id, busy_gpios),                                 \
		.irq = GPIO_DT_SPEC_GET(node_id, irq_gpios),                                   \
		.ant_enable = GPIO_DT_SPEC_GET_OR(node_id, antenna_enable_gpios, {0}),         \
		.tx_enable = GPIO_DT_SPEC_GET_OR(node_id, tx_enable_gpios, {0}),               \
		.rx_enable = GPIO_DT_SPEC_GET_OR(node_id, rx_enable_gpios, {0}),               \
		.rfswitch = {                                                                  \
			.enable = DT_PROP(node_id, rfsw_enable),                               \
			.standby = DT_PROP(node_id, rfsw_standby),                             \
			.rx = DT_PROP(node_id, rfsw_rx),                                       \
			.tx = DT_PROP(node_id, rfsw_tx),                                       \
			.tx_hp = DT_PROP(node_id, rfsw_tx_hp),                                 \
			.tx_hf = DT_PROP(node_id, rfsw_tx_hf),                                 \
			.gnss = DT_PROP(node_id, rfsw_gnss),                                   \
			.wifi = DT_PROP(node_id, rfsw_wifi),                                   \
		},                                                                             \
		.tcxo_voltage = DT_ENUM_IDX(node_id, tcxo_voltage),                            \
		.tcxo_startup_delay_ms = DT_PROP_OR(node_id, tcxo_power_startup_delay_ms, 0),  \
		.pa_cfg = {                                                                    \
			.pa_sel = DT_PROP(node_id, pa_select),                                 \
			.pa_reg_supply = DT_PROP(node_id, pa_reg_supply),                      \
			.pa_duty_cycle = DT_PROP(node_id, pa_duty_cycle),                      \
			.pa_hp_sel = DT_PROP(node_id, pa_hp_sel),                              \
		},                                                                             \
		.tx_power_offset_db = DT_PROP(node_id, tx_power_offset_db),                    \
		.crc_over_spi = DT_PROP(node_id, crc_over_spi),                                \
		.rx_boosted = DT_PROP(node_id, rx_boosted),                                    \
		.skip_rssi_calibration = DT_PROP(node_id, skip_rssi_calibration),              \
		.legacy_lora_public_network = DT_PROP(node_id, legacy_lora_public_network),    \
		.use_dcdc = DT_PROP(node_id, use_dcdc),                                        \
		.lfclk_in_sleep = DT_PROP(node_id, lfclk_in_sleep),                            \
	};                                                                                 \
	static struct lbm_lr11xx_data data_##node_id;                                      \
	DEVICE_DT_DEFINE(node_id, lr11xx_init, NULL, &data_##node_id, &config_##node_id,   \
			 POST_KERNEL, CONFIG_LORA_INIT_PRIORITY, &lbm_lora_api)

DT_FOREACH_STATUS_OKAY(semtech_lr1110, LR11XX_DEFINE);
DT_FOREACH_STATUS_OKAY(semtech_lr1120, LR11XX_DEFINE);
