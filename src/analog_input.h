/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#ifndef ZEPHYR_INCLUDE_ANALOG_INPUT_H_
#define ZEPHYR_INCLUDE_ANALOG_INPUT_H_

/**
 * @file analog_input.h
 *
 * @brief Header file for the analog_input driver.
 */

#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif

struct analog_input_data {
    const struct device *dev;
    struct adc_sequence as;
    uint16_t *as_buff;
    int32_t *delta;
    struct k_work_delayable init_work;
    int async_init_step;
    bool ready;
    struct k_work sampling_work;
    struct k_timer sampling_timer;
    int err;
};

struct io_channel { 
	struct adc_dt_spec adc_channel;
    uint16_t mv_mid;
    uint16_t mv_min_max;
    uint8_t mv_deadzone;
    bool invert;
    uint16_t scale_multiplier;
    uint16_t scale_divisor;
    uint8_t evt_type;
    uint8_t input_code;
};

struct analog_input_config {
    uint32_t sampling_hz;
    uint8_t input_channel;
    uint8_t io_channels_len;
	struct io_channel io_channels[];
};

enum analog_input_channel {
	ANALOG_INPUT_CHAN_DRIVER_CONFIG = SENSOR_CHAN_PRIV_START,
};

enum analog_input_attribute {
	ANALOG_INPUT_ATTR_INTPUT_CHANNEL = SENSOR_ATTR_PRIV_START,
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_INCLUDE_ANALOG_INPUT_H_ */
