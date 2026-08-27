/*
 * Copyright (c) 2026 FoBE Studio
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>
#include <stdint.h>
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
#include "lr20xx_hal.h"
#include "lr20xx_radio_common.h"
#include "lr20xx_radio_common_types.h"
#include "lr20xx_system_types.h"
#include "ral_lr20xx_bsp.h"
#include "ralf_lr20xx.h"

#define LR20XX_SYSTEM_SET_SLEEP_OC_MSB 0x01
#define LR20XX_SYSTEM_SET_SLEEP_OC_LSB 0x27

#define LR20XX_BUSY_TIMEOUT K_SECONDS(1)
#define LR20XX_RESET_READY_DELAY K_MSEC(10)
#define LR20XX_WAKEUP_NSS_PULSE_DELAY_US 1000U
#define LR20XX_SLEEP_ENTRY_DELAY K_USEC(500)

#define LR20XX_MAX_COMMAND_LEN 16U
#define LR20XX_MAX_READ_DATA_LEN 256U
#define LR20XX_READ_DUMMY_LEN 2U
#define LR20XX_MAX_READ_TRANSFER_LEN (LR20XX_MAX_READ_DATA_LEN + LR20XX_READ_DUMMY_LEN)
#define LR20XX_RX_BOOST_ENABLED_FALLBACK LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_1

#define LR20XX_DIO_MIN LR20XX_SYSTEM_DIO_5
#define LR20XX_DIO_MAX LR20XX_SYSTEM_DIO_11
#define LR20XX_DIO_COUNT (LR20XX_DIO_MAX - LR20XX_DIO_MIN + 1)

#define LR20XX_LF_MIN_OUTPUT_POWER -10
#define LR20XX_LF_MAX_OUTPUT_POWER 22
#define LR20XX_HF_MIN_OUTPUT_POWER -17
#define LR20XX_HF_MAX_OUTPUT_POWER 12

#define LR20XX_FE_CAL_FREQ0_HZ 470000000U
#define LR20XX_FE_CAL_FREQ1_HZ 897500000U
#define LR20XX_FE_CAL_FREQ2_HZ 2441000000U

struct lr20xx_pa_pwr_cfg {
	int8_t half_power;
	uint8_t pa_duty_cycle;
	uint8_t pa_lf_slices;
};

#include "lr20xx_pa_pwr_cfg.h"

static const struct lr20xx_pa_pwr_cfg pa_lf_cfg_table[] = LR20XX_PA_LF_CFG_TABLE;
static const struct lr20xx_pa_pwr_cfg pa_hf_cfg_table[] = LR20XX_PA_HF_CFG_TABLE;

BUILD_ASSERT(ARRAY_SIZE(pa_lf_cfg_table) ==
	     (LR20XX_LF_MAX_OUTPUT_POWER - LR20XX_LF_MIN_OUTPUT_POWER + 1));
BUILD_ASSERT(ARRAY_SIZE(pa_hf_cfg_table) ==
	     (LR20XX_HF_MAX_OUTPUT_POWER - LR20XX_HF_MIN_OUTPUT_POWER + 1));

struct lbm_lr20xx_config {
	struct lbm_lora_config_common lbm_common;
	struct spi_dt_spec spi;
	struct gpio_dt_spec reset;
	struct gpio_dt_spec busy;
	struct gpio_dt_spec irq;
	struct gpio_dt_spec ant_enable;
	struct gpio_dt_spec tx_enable;
	struct gpio_dt_spec rx_enable;
	lr20xx_system_irq_mask_t irq_mask;
	lr20xx_system_dio_rf_switch_cfg_t rfswitch[LR20XX_DIO_COUNT];
	lr20xx_system_dio_t irq_dio;
	lr20xx_system_dio_drive_t dio_sleep_drive;
	lr20xx_system_lfclk_cfg_t lf_clk;
	uint8_t tcxo_voltage;
	lr20xx_radio_common_rx_path_boost_mode_t rx_boost_mode_lf;
	lr20xx_radio_common_rx_path_boost_mode_t rx_boost_mode_hf;
	uint32_t hf_frequency_threshold_hz;
	uint32_t fe_cal_frequencies_hz[3];
	int tcxo_startup_delay_ms;
	int tx_power_offset_db;
	bool use_dcdc;
};

struct lbm_lr20xx_data {
	struct lbm_lora_data_common lbm_common;
	const struct device *dev;
	struct gpio_callback irq_callback;
	bool asleep;
};

LOG_MODULE_DECLARE(lbm_driver, CONFIG_LORA_LOG_LEVEL);

static bool lr20xx_has_gpio_cs(const struct lbm_lr20xx_config *config)
{
	const struct spi_cs_control *cs = &config->spi.config.cs;

	return cs->cs_is_gpio && (cs->gpio.port != NULL);
}

static uint16_t lr20xx_opcode(const uint8_t *command, uint16_t command_length)
{
	if (command_length < 2U) {
		return 0U;
	}

	return ((uint16_t)command[0] << 8) | command[1];
}

static bool lr20xx_is_busy(const struct device *dev)
{
	const struct lbm_lr20xx_config *config = dev->config;

	return gpio_pin_get_dt(&config->busy);
}

static int lr20xx_wait_device_ready(const struct device *dev, k_timeout_t timeout)
{
	k_timepoint_t expiry = sys_timepoint_calc(timeout);

	do {
		if (!lr20xx_is_busy(dev)) {
			return 0;
		}
		k_sleep(K_MSEC(1));
	} while (!sys_timepoint_expired(expiry));

	return -EAGAIN;
}

static int lr20xx_wakeup_sequence(const struct device *dev, k_timeout_t timeout)
{
	const struct lbm_lr20xx_config *config = dev->config;
	const struct spi_cs_control *cs = &config->spi.config.cs;
	int ret;

	if (!lr20xx_has_gpio_cs(config)) {
		return -ENOTSUP;
	}

	/* gpio_pin_set_dt uses logical values: 1 = active, 0 = inactive. */
	ret = gpio_pin_set_dt(&cs->gpio, 1);
	if (ret) {
		return ret;
	}

	k_busy_wait(LR20XX_WAKEUP_NSS_PULSE_DELAY_US);
	ret = gpio_pin_set_dt(&cs->gpio, 0);
	if (ret) {
		return ret;
	}

	return lr20xx_wait_device_ready(dev, timeout);
}

static bool lr20xx_is_sleep_command(const uint8_t *command, uint16_t command_length)
{
	return command_length >= 2U && command[0] == LR20XX_SYSTEM_SET_SLEEP_OC_MSB &&
	       command[1] == LR20XX_SYSTEM_SET_SLEEP_OC_LSB;
}

static int lr20xx_ensure_device_ready(const struct device *dev, k_timeout_t timeout)
{
	const struct lbm_lr20xx_config *config = dev->config;
	struct lbm_lr20xx_data *data = dev->data;
	int ret;

	if (data->asleep) {
		LOG_DBG("SLEEP -> ACTIVE");
		ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
		if (ret) {
			LOG_ERR("Failed to enable LR20XX interrupt before wakeup: %d", ret);
			return ret;
		}
		ret = lr20xx_wakeup_sequence(dev, timeout);
		if (ret) {
			return ret;
		}
		data->asleep = false;
	}

	ret = lr20xx_wait_device_ready(dev, timeout);
	data->asleep = false;
	return ret;
}

static int lr20xx_prepare_for_command(const struct device *dev, const uint8_t *command,
				      uint16_t command_length, k_timeout_t timeout)
{
	struct lbm_lr20xx_data *data = dev->data;
	int ret;

	if (!data->asleep && data->lbm_common.rx_started_with_duty_cycle && lr20xx_is_busy(dev)) {
		LOG_DBG("Waking from RX duty-cycle before command");
		ret = lr20xx_wakeup_sequence(dev, timeout);
		if (ret) {
			return ret;
		}
		data->asleep = false;
	}

	ret = lr20xx_ensure_device_ready(dev, timeout);
	if (ret && lr20xx_is_sleep_command(command, command_length) &&
	    data->lbm_common.rx_started_with_duty_cycle && lr20xx_is_busy(dev)) {
		LOG_DBG("Waking from RX duty-cycle before sleep command");
		ret = lr20xx_wakeup_sequence(dev, timeout);
		if (ret == 0) {
			data->asleep = false;
		}
	}

	return ret;
}

static lr20xx_hal_status_t lr20xx_ret_to_hal_status(int ret)
{
	return ret == 0 ? LR20XX_HAL_STATUS_OK : LR20XX_HAL_STATUS_ERROR;
}

lr20xx_hal_status_t lr20xx_hal_write(const void *context, const uint8_t *command,
				     const uint16_t command_length, const uint8_t *data,
				     const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;
	struct lbm_lr20xx_data *dev_data = dev->data;
	const uint16_t opcode = lr20xx_opcode(command, command_length);
	int ret;

	LOG_DBG("CMD=0x%04x CMD_LEN=%d DATA_LEN=%d", opcode, command_length, data_length);

	ret = lr20xx_prepare_for_command(dev, command, command_length, LR20XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR20XX not ready before write CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf tx_bufs[] = {
		{
			.buf = (void *)command,
			.len = command_length,
		},
		{
			.buf = (void *)data,
			.len = data_length,
		},
	};
	const struct spi_buf_set tx_buf_set = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};

	ret = spi_write_dt(&config->spi, &tx_buf_set);
	if (ret) {
		LOG_ERR("LR20XX SPI write failed CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	if (lr20xx_is_sleep_command(command, command_length)) {
		LOG_DBG("ACTIVE -> SLEEP");
		ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_DISABLE);
		if (ret) {
			LOG_WRN("Failed to disable LR20XX interrupt before sleep: %d", ret);
		}
		dev_data->asleep = true;
		k_sleep(LR20XX_SLEEP_ENTRY_DELAY);
	}

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length, uint8_t *data,
				    const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;
	const uint16_t opcode = lr20xx_opcode(command, command_length);
	int ret;

	LOG_DBG("CMD=0x%04x CMD_LEN=%d DATA_LEN=%d", opcode, command_length, data_length);

	if (data_length > LR20XX_MAX_READ_DATA_LEN) {
		return LR20XX_HAL_STATUS_ERROR;
	}

	ret = lr20xx_prepare_for_command(dev, command, command_length, LR20XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR20XX not ready before read CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	const struct spi_buf cmd_tx_bufs[] = {
		{
			.buf = (void *)command,
			.len = command_length,
		},
	};
	const struct spi_buf_set cmd_tx_buf_set = {
		.buffers = cmd_tx_bufs,
		.count = ARRAY_SIZE(cmd_tx_bufs),
	};

	ret = spi_write_dt(&config->spi, &cmd_tx_buf_set);
	if (ret) {
		LOG_ERR("LR20XX SPI read-command write failed CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	if (data_length == 0U) {
		return LR20XX_HAL_STATUS_OK;
	}

	ret = lr20xx_ensure_device_ready(dev, LR20XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR20XX not ready before read payload CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	uint8_t nop[LR20XX_MAX_READ_TRANSFER_LEN] = {0};
	uint8_t rx[LR20XX_MAX_READ_TRANSFER_LEN] = {0};
	const size_t read_len = data_length + LR20XX_READ_DUMMY_LEN;
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
	const struct spi_buf_set read_tx_buf_set = {
		.buffers = read_tx_bufs,
		.count = ARRAY_SIZE(read_tx_bufs),
	};
	const struct spi_buf_set read_rx_buf_set = {
		.buffers = read_rx_bufs,
		.count = ARRAY_SIZE(read_rx_bufs),
	};

	ret = spi_transceive_dt(&config->spi, &read_tx_buf_set, &read_rx_buf_set);
	if (ret) {
		LOG_ERR("LR20XX SPI read payload failed CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	memcpy(data, &rx[LR20XX_READ_DUMMY_LEN], data_length);

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read(const void *context, uint8_t *data,
					   const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;
	int ret;

	LOG_DBG("DATA_LEN=%d", data_length);

	if (data_length > LR20XX_MAX_READ_DATA_LEN) {
		return LR20XX_HAL_STATUS_ERROR;
	}

	ret = lr20xx_ensure_device_ready(dev, LR20XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR20XX not ready before direct read: %d", ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	uint8_t nop[LR20XX_MAX_READ_DATA_LEN] = {0};
	uint8_t rx[LR20XX_MAX_READ_DATA_LEN] = {0};
	const struct spi_buf tx_bufs[] = {
		{
			.buf = nop,
			.len = data_length,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = rx,
			.len = data_length,
		},
	};
	const struct spi_buf_set tx_buf_set = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};
	const struct spi_buf_set rx_buf_set = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	ret = spi_transceive_dt(&config->spi, &tx_buf_set, &rx_buf_set);
	if (ret) {
		LOG_ERR("LR20XX SPI direct read failed: %d", ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	memcpy(data, rx, data_length);

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_direct_read_fifo(const void *context, const uint8_t *command,
						const uint16_t command_length, uint8_t *data,
						const uint16_t data_length)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;
	const uint16_t opcode = lr20xx_opcode(command, command_length);
	int ret;

	LOG_DBG("CMD=0x%04x CMD_LEN=%d DATA_LEN=%d", opcode, command_length, data_length);

	if (command_length > LR20XX_MAX_COMMAND_LEN || data_length > LR20XX_MAX_READ_DATA_LEN) {
		return LR20XX_HAL_STATUS_ERROR;
	}

	ret = lr20xx_ensure_device_ready(dev, LR20XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("LR20XX not ready before FIFO read CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	uint8_t nop[LR20XX_MAX_READ_DATA_LEN] = {0};
	uint8_t discard[LR20XX_MAX_COMMAND_LEN] = {0};
	const struct spi_buf tx_bufs[] = {
		{
			.buf = (void *)command,
			.len = command_length,
		},
		{
			.buf = nop,
			.len = data_length,
		},
	};
	const struct spi_buf rx_bufs[] = {
		{
			.buf = discard,
			.len = command_length,
		},
		{
			.buf = data,
			.len = data_length,
		},
	};
	const struct spi_buf_set tx_buf_set = {
		.buffers = tx_bufs,
		.count = ARRAY_SIZE(tx_bufs),
	};
	const struct spi_buf_set rx_buf_set = {
		.buffers = rx_bufs,
		.count = ARRAY_SIZE(rx_bufs),
	};

	ret = spi_transceive_dt(&config->spi, &tx_buf_set, &rx_buf_set);
	if (ret) {
		LOG_ERR("LR20XX SPI FIFO read failed CMD=0x%04x: %d", opcode, ret);
		return LR20XX_HAL_STATUS_ERROR;
	}

	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_reset(const void *context)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;
	struct lbm_lr20xx_data *data = dev->data;

	LOG_DBG("");

	gpio_pin_set_dt(&config->reset, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set_dt(&config->reset, 0);
	k_sleep(LR20XX_RESET_READY_DELAY);

	data->asleep = false;
	return LR20XX_HAL_STATUS_OK;
}

lr20xx_hal_status_t lr20xx_hal_wakeup(const void *context)
{
	const struct device *dev = context;

	return lr20xx_ret_to_hal_status(lr20xx_ensure_device_ready(dev, LR20XX_BUSY_TIMEOUT));
}

static void lr20xx_get_tx_cfg(lr20xx_radio_common_pa_selection_t pa_type,
			      int16_t expected_output_pwr_in_dbm,
			      ral_lr20xx_bsp_tx_cfg_output_params_t *output_params)
{
	int16_t power = expected_output_pwr_in_dbm;
	size_t index;

	output_params->pa_ramp_time = LR20XX_RADIO_COMMON_RAMP_48_US;

	switch (pa_type) {
	case LR20XX_RADIO_COMMON_PA_SEL_LF:
		power = CLAMP(power, LR20XX_LF_MIN_OUTPUT_POWER, LR20XX_LF_MAX_OUTPUT_POWER);
		index = power - LR20XX_LF_MIN_OUTPUT_POWER;
		output_params->pa_cfg.pa_sel = LR20XX_RADIO_COMMON_PA_SEL_LF;
		output_params->pa_cfg.pa_lf_mode = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
		output_params->pa_cfg.pa_lf_slices = pa_lf_cfg_table[index].pa_lf_slices;
		output_params->pa_cfg.pa_lf_duty_cycle = pa_lf_cfg_table[index].pa_duty_cycle;
		output_params->pa_cfg.pa_hf_duty_cycle = 16;
		output_params->chip_output_half_pwr_in_dbm_configured =
			pa_lf_cfg_table[index].half_power;
		output_params->chip_output_pwr_in_dbm_expected = (int8_t)power;
		break;
	case LR20XX_RADIO_COMMON_PA_SEL_HF:
	default:
		power = CLAMP(power, LR20XX_HF_MIN_OUTPUT_POWER, LR20XX_HF_MAX_OUTPUT_POWER);
		index = power - LR20XX_HF_MIN_OUTPUT_POWER;
		output_params->pa_cfg.pa_sel = LR20XX_RADIO_COMMON_PA_SEL_HF;
		output_params->pa_cfg.pa_lf_mode = LR20XX_RADIO_COMMON_PA_LF_MODE_FSM;
		output_params->pa_cfg.pa_lf_slices = pa_hf_cfg_table[index].pa_lf_slices;
		output_params->pa_cfg.pa_lf_duty_cycle = 6;
		output_params->pa_cfg.pa_hf_duty_cycle = pa_hf_cfg_table[index].pa_duty_cycle;
		output_params->chip_output_half_pwr_in_dbm_configured =
			pa_hf_cfg_table[index].half_power;
		output_params->chip_output_pwr_in_dbm_expected = (int8_t)power;
		break;
	}
}

void ral_lr20xx_bsp_get_tx_cfg(const void *context,
			       const ral_lr20xx_bsp_tx_cfg_input_params_t *input_params,
			       ral_lr20xx_bsp_tx_cfg_output_params_t *output_params)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;
	int16_t power = input_params->system_output_pwr_in_dbm + config->tx_power_offset_db;
	int16_t expected_power;
	lr20xx_radio_common_pa_selection_t pa_type = LR20XX_RADIO_COMMON_PA_SEL_LF;

	if (input_params->freq_in_hz >= config->hf_frequency_threshold_hz) {
		pa_type = LR20XX_RADIO_COMMON_PA_SEL_HF;
	}

	lr20xx_get_tx_cfg(pa_type, power, output_params);
	expected_power =
		output_params->chip_output_pwr_in_dbm_expected - config->tx_power_offset_db;
	output_params->chip_output_pwr_in_dbm_expected =
		(int8_t)CLAMP(expected_power, INT8_MIN, INT8_MAX);
}

static void lr20xx_get_rx_cfg(const struct device *dev, const uint32_t freq_in_hz,
			      lr20xx_radio_common_rx_path_t *rx_path,
			      lr20xx_radio_common_rx_path_boost_mode_t *boost_mode)
{
	const struct lbm_lr20xx_config *config = dev->config;
	const struct lbm_lr20xx_data *data = dev->data;
	lr20xx_radio_common_rx_path_boost_mode_t dt_boost;

	if (freq_in_hz >= config->hf_frequency_threshold_hz) {
		*rx_path = LR20XX_RADIO_COMMON_RX_PATH_HF;
		dt_boost = config->rx_boost_mode_hf;
	} else {
		*rx_path = LR20XX_RADIO_COMMON_RX_PATH_LF;
		dt_boost = config->rx_boost_mode_lf;
	}

	switch (data->lbm_common.rx_boosted) {
	case RX_BOOST_DISABLED:
		*boost_mode = LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE;
		break;
	case RX_BOOST_ENABLED:
		*boost_mode = dt_boost == LR20XX_RADIO_COMMON_RX_PATH_BOOST_MODE_NONE
				      ? LR20XX_RX_BOOST_ENABLED_FALLBACK
				      : dt_boost;
		break;
	case RX_BOOST_DEFAULT:
	default:
		*boost_mode = dt_boost;
		break;
	}
}

void ral_lr20xx_bsp_get_rx_cfg(const void *context, const uint32_t freq_in_hz,
			       lr20xx_radio_common_rx_path_t *rx_path,
			       lr20xx_radio_common_rx_path_boost_mode_t *boost_mode)
{
	lr20xx_get_rx_cfg(context, freq_in_hz, rx_path, boost_mode);
}

void ral_lr20xx_bsp_get_front_end_calibration_cfg(
	const void *context,
	lr20xx_radio_common_front_end_calibration_value_t front_end_calibration_structures[3])
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	for (size_t i = 0; i < ARRAY_SIZE(config->fe_cal_frequencies_hz); i++) {
		lr20xx_radio_common_rx_path_t rx_path;
		lr20xx_radio_common_rx_path_boost_mode_t boost_mode;
		uint32_t freq_in_hz = config->fe_cal_frequencies_hz[i];

		lr20xx_get_rx_cfg(dev, freq_in_hz, &rx_path, &boost_mode);
		front_end_calibration_structures[i].rx_path = rx_path;
		front_end_calibration_structures[i].frequency_in_hertz = freq_in_hz;
	}
}

static int lr20xx_dio_index(lr20xx_system_dio_t dio)
{
	if (dio < LR20XX_DIO_MIN || dio > LR20XX_DIO_MAX) {
		return -EINVAL;
	}

	return dio - LR20XX_DIO_MIN;
}

static lr20xx_system_dio_rf_switch_cfg_t lr20xx_get_rfswitch_cfg(const struct device *dev,
								  lr20xx_system_dio_t dio)
{
	const struct lbm_lr20xx_config *config = dev->config;
	int index = lr20xx_dio_index(dio);

	if (index < 0) {
		return 0;
	}

	return config->rfswitch[index];
}

void ral_lr20xx_bsp_get_dio_function(const void *context, lr20xx_system_dio_t dio,
				     lr20xx_system_dio_func_t *function)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	if (dio == config->irq_dio) {
		*function = LR20XX_SYSTEM_DIO_FUNC_IRQ;
	} else if (lr20xx_get_rfswitch_cfg(dev, dio) != 0U) {
		*function = LR20XX_SYSTEM_DIO_FUNC_RF_SWITCH;
	} else {
		*function = LR20XX_SYSTEM_DIO_FUNC_NONE;
	}
}

void ral_lr20xx_bsp_get_dio_sleep_drive(const void *context, lr20xx_system_dio_t dio,
					lr20xx_system_dio_drive_t *drive)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	ARG_UNUSED(dio);

	*drive = config->dio_sleep_drive;
}

void ral_lr20xx_bsp_get_dio_irq_mask(const void *context, lr20xx_system_dio_t dio,
				     lr20xx_system_irq_mask_t *irq_mask)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	if (dio == config->irq_dio) {
		*irq_mask = config->irq_mask;
	} else {
		*irq_mask = LR20XX_SYSTEM_IRQ_NONE;
	}
}

void ral_lr20xx_bsp_get_dio_rf_switch_cfg(const void *context, lr20xx_system_dio_t dio,
					  lr20xx_system_dio_rf_switch_cfg_t *rf_switch_cfg)
{
	*rf_switch_cfg = lr20xx_get_rfswitch_cfg(context, dio);
}

void ral_lr20xx_bsp_get_dio_hf_clk_scaling_cfg(
	const void *context, lr20xx_system_hf_clk_scaling_t *hf_clk_scaling)
{
	ARG_UNUSED(context);

	*hf_clk_scaling = LR20XX_SYSTEM_HF_CLK_SCALING_32_MHZ;
}

void ral_lr20xx_bsp_get_reg_mode(const void *context, lr20xx_system_reg_mode_t *reg_mode)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	*reg_mode = config->use_dcdc ? LR20XX_SYSTEM_REG_MODE_DCDC : LR20XX_SYSTEM_REG_MODE_LDO;
}

void ral_bsp_lr20xx_get_lfclk_cfg(const void *context, lr20xx_system_lfclk_cfg_t *lfclk_cfg)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	*lfclk_cfg = config->lf_clk;
}

void ral_lr20xx_bsp_get_xosc_cfg(const void *context, ral_xosc_cfg_t *xosc_cfg,
				 lr20xx_system_tcxo_supply_voltage_t *supply_voltage,
				 uint32_t *startup_time_in_tick)
{
	const struct device *dev = context;
	const struct lbm_lr20xx_config *config = dev->config;

	if (config->tcxo_voltage > LR20XX_SYSTEM_TCXO_CTRL_3_3V) {
		*xosc_cfg = RAL_XOSC_CFG_XTAL;
		*supply_voltage = LR20XX_SYSTEM_TCXO_CTRL_1_8V;
		*startup_time_in_tick = 0U;
		return;
	}

	*xosc_cfg = RAL_XOSC_CFG_TCXO_RADIO_CTRL;
	*supply_voltage = (lr20xx_system_tcxo_supply_voltage_t)config->tcxo_voltage;
	*startup_time_in_tick =
		lr20xx_radio_common_convert_time_in_ms_to_rtc_step(config->tcxo_startup_delay_ms);
}

void ral_lr20xx_bsp_get_lora_cad_det_peak(const void *context, ral_lora_sf_t sf,
					  ral_lora_cad_symbs_t nb_symbol,
					  uint8_t *in_out_cad_det_peak)
{
	ARG_UNUSED(context);
	ARG_UNUSED(sf);
	ARG_UNUSED(nb_symbol);
	ARG_UNUSED(in_out_cad_det_peak);
}

ral_status_t ral_lr20xx_bsp_get_instantaneous_tx_power_consumption(
	const void *context, const ral_lr20xx_bsp_tx_cfg_output_params_t *tx_cfg,
	lr20xx_system_reg_mode_t radio_reg_mode, uint32_t *pwr_consumption_in_ua)
{
	ARG_UNUSED(context);
	ARG_UNUSED(tx_cfg);
	ARG_UNUSED(radio_reg_mode);
	ARG_UNUSED(pwr_consumption_in_ua);
	return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr20xx_bsp_get_instantaneous_gfsk_rx_power_consumption(
	const void *context, lr20xx_system_reg_mode_t radio_reg_mode, bool rx_boosted,
	uint32_t *pwr_consumption_in_ua)
{
	ARG_UNUSED(context);
	ARG_UNUSED(radio_reg_mode);
	ARG_UNUSED(rx_boosted);
	ARG_UNUSED(pwr_consumption_in_ua);
	return RAL_STATUS_UNSUPPORTED_FEATURE;
}

ral_status_t ral_lr20xx_bsp_get_instantaneous_lora_rx_power_consumption(
	const void *context, const lr20xx_system_reg_mode_t radio_reg_mode,
	const ral_lora_bw_t bw, bool rx_boosted, uint32_t *pwr_consumption_in_ua)
{
	ARG_UNUSED(context);
	ARG_UNUSED(radio_reg_mode);
	ARG_UNUSED(bw);
	ARG_UNUSED(rx_boosted);
	ARG_UNUSED(pwr_consumption_in_ua);
	return RAL_STATUS_UNSUPPORTED_FEATURE;
}

void lbm_driver_antenna_configure(const struct device *dev, enum lbm_modem_mode mode)
{
	const struct lbm_lr20xx_config *config = dev->config;

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

static void lr20xx_irq_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct lbm_lr20xx_data *data = CONTAINER_OF(cb, struct lbm_lr20xx_data, irq_callback);

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	k_work_schedule(&data->lbm_common.op_done_work, K_NO_WAIT);
}

int lbm_driver_add_dio1_gpio_callback(const struct device *dev, struct gpio_callback *callback,
				      gpio_callback_handler_t handler)
{
	const struct lbm_lr20xx_config *config = dev->config;
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

	ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to enable LR20XX user interrupt: %d", ret);
		(void)gpio_remove_callback(config->irq.port, callback);
		return ret;
	}

	return 0;
}

int lbm_driver_remove_dio1_gpio_callback(const struct device *dev, struct gpio_callback *callback)
{
	const struct lbm_lr20xx_config *config = dev->config;
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
	const struct lbm_lr20xx_config *config = dev->config;
	struct lbm_lr20xx_data *data = dev->data;
	ral_status_t status;
	int ret;

	status = ral_reset(&config->lbm_common.ralf.ral);
	if (status != RAL_STATUS_OK) {
		LOG_ERR("Reset failure (%d)", status);
		return -EIO;
	}

	ret = lr20xx_ensure_device_ready(dev, LR20XX_BUSY_TIMEOUT);
	if (ret) {
		LOG_ERR("Failed to return to ready after reset");
		return -EIO;
	}

	ret = lbm_lora_common_init(dev);
	if (ret < 0) {
		return ret;
	}

	gpio_init_callback(&data->irq_callback, lr20xx_irq_callback, BIT(config->irq.pin));
	if (gpio_add_callback(config->irq.port, &data->irq_callback) < 0) {
		LOG_ERR("Could not set GPIO callback for LR20XX interrupt.");
		return -EIO;
	}
	ret = gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret) {
		LOG_ERR("Failed to enable LR20XX interrupt: %d", ret);
		(void)gpio_remove_callback(config->irq.port, &data->irq_callback);
		return ret;
	}

	LOG_INF("Radio initialized");
	return 0;
}

static int lr20xx_init(const struct device *dev)
{
	const struct lbm_lr20xx_config *config = dev->config;
	struct lbm_lr20xx_data *data = dev->data;

	if (!spi_is_ready_dt(&config->spi)) {
		LOG_ERR("SPI bus %s not ready", config->spi.bus->name);
		return -ENODEV;
	}

	if (!lr20xx_has_gpio_cs(config)) {
		LOG_ERR("LR20XX requires GPIO-controlled SPI chip select for wakeup");
		return -ENOTSUP;
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

#define LR20XX_FE_CAL_FREQ_OR(node_id, index, default_value)                                      \
	COND_CODE_1(DT_PROP_HAS_IDX(node_id, front_end_calibration_frequencies_hz, index),        \
		    (DT_PROP_BY_IDX(node_id, front_end_calibration_frequencies_hz, index)),        \
		    (default_value))

#define LR20XX_ASSERT_TX_POWER_OFFSET(node_id)                                                 \
	BUILD_ASSERT(DT_PROP(node_id, tx_power_offset_db) >= -64 &&                            \
		     DT_PROP(node_id, tx_power_offset_db) <= 64,                              \
		     "LR20XX tx-power-offset-db must be in range [-64, 64]")

#define LR20XX_DEFINE(node_id)                                                                  \
	LR20XX_ASSERT_TX_POWER_OFFSET(node_id);                                                 \
	static const struct lbm_lr20xx_config config_##node_id = {                             \
		.lbm_common.ralf = RALF_LR20XX_INSTANTIATE(DEVICE_DT_GET(node_id)),            \
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
		.irq_mask = DT_PROP(node_id, irq_mask),                                        \
		.rfswitch = {                                                                  \
			[LR20XX_SYSTEM_DIO_5 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio5_rfsw), \
			[LR20XX_SYSTEM_DIO_6 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio6_rfsw), \
			[LR20XX_SYSTEM_DIO_7 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio7_rfsw), \
			[LR20XX_SYSTEM_DIO_8 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio8_rfsw), \
			[LR20XX_SYSTEM_DIO_9 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio9_rfsw), \
			[LR20XX_SYSTEM_DIO_10 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio10_rfsw), \
			[LR20XX_SYSTEM_DIO_11 - LR20XX_DIO_MIN] = DT_PROP(node_id, dio11_rfsw), \
		},                                                                             \
		.irq_dio = DT_PROP(node_id, irq_dio),                                          \
		.dio_sleep_drive = DT_PROP(node_id, dio_sleep_drive),                          \
		.lf_clk = DT_ENUM_IDX(node_id, lf_clk),                                        \
		.tcxo_voltage = DT_ENUM_IDX(node_id, tcxo_voltage),                            \
		.rx_boost_mode_lf = DT_PROP(node_id, rx_boost_mode_lf),                        \
		.rx_boost_mode_hf = DT_PROP(node_id, rx_boost_mode_hf),                        \
		.hf_frequency_threshold_hz = DT_PROP(node_id, hf_frequency_threshold_hz),       \
		.fe_cal_frequencies_hz = {                                                     \
			LR20XX_FE_CAL_FREQ_OR(node_id, 0, LR20XX_FE_CAL_FREQ0_HZ),             \
			LR20XX_FE_CAL_FREQ_OR(node_id, 1, LR20XX_FE_CAL_FREQ1_HZ),             \
			LR20XX_FE_CAL_FREQ_OR(node_id, 2, LR20XX_FE_CAL_FREQ2_HZ),             \
		},                                                                             \
		.tcxo_startup_delay_ms = DT_PROP(node_id, tcxo_power_startup_delay_ms),        \
		.tx_power_offset_db = DT_PROP(node_id, tx_power_offset_db),                    \
		.use_dcdc = DT_PROP(node_id, use_dcdc),                                        \
	};                                                                                 \
	static struct lbm_lr20xx_data data_##node_id;                                      \
	DEVICE_DT_DEFINE(node_id, lr20xx_init, NULL, &data_##node_id, &config_##node_id,   \
			 POST_KERNEL, CONFIG_LORA_INIT_PRIORITY, &lbm_lora_api)

DT_FOREACH_STATUS_OKAY(semtech_lr2021, LR20XX_DEFINE);
