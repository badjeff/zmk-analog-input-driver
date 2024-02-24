/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_analog_input

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include <stdlib.h> //for abs()
#include <zephyr/sys/util.h> // for CLAMP

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ANALOG_INPUT, CONFIG_ANALOG_INPUT_LOG_LEVEL);

#include "analog_input.h"

static int analog_input_report_data(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    const struct analog_input_config *config = dev->config;

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    static int64_t last_smp_time = 0;
    static int64_t last_rpt_time = 0;
    int64_t now = k_uptime_get();
#endif

    struct adc_sequence* as = &data->as;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct io_channel ch_cfg = (struct io_channel)config->io_channels[i];
        const struct device* adc = ch_cfg.adc_channel.dev;

        if (i == 0) {
            int err = adc_read(adc, as);
            if (err < 0) {
                LOG_ERR("AIN%u read returned %d", i, err);
                return err;
            }
        }

        int32_t raw = data->as_buff[i];
        int32_t mv = raw;
        adc_raw_to_millivolts(adc_ref_internal(adc), ADC_GAIN_1_6, as->resolution, &mv);
        LOG_DBG("AIN%u raw: %d mv: %d", ch_cfg.adc_channel.channel_id, raw, mv);

        int16_t v = mv - ch_cfg.mv_mid;
        int16_t dz = ch_cfg.mv_deadzone;
        if (dz) {
            if (v > 0) {
                if (v < dz) v = 0; else v -= dz;
            }
            if (v < 0) {
                if (v > -dz) v = 0; else v += dz;
            }
        }
        uint16_t mm = ch_cfg.mv_min_max;
        if (mm) {
            if (v > 0 && v > mm) v = mm;
            if (v < 0 && v < -mm) v = -mm;
        }

        if (ch_cfg.invert) v *= -1;
        v = (int16_t)((v * ch_cfg.scale_multiplier) / ch_cfg.scale_divisor);

        // accumulate delta until report in next iteration
        int32_t delta = data->delta[i];
        int32_t dv = delta + v;

        data->delta[i] = dv;
    }

    // First read is setup as calibration
    as->calibrate = false;

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // purge accumulated delta, if last sampled had not been reported on last report tick
    if (now - last_smp_time >= CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {
        for (uint8_t i = 0; i < config->io_channels_len; i++) {
            data->delta[i] = 0;
        }
    }
    last_smp_time = now;
#endif

#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
    // strict to report inerval
    if (now - last_rpt_time < CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN) {

        return 0;
    }
#endif

    int8_t idx_to_sync = -1;
    for (uint8_t i = config->io_channels_len - 1; i >= 0; i--) {
        int32_t dv = data->delta[i];
        if (dv != 0) {
            idx_to_sync = i;
            break;
        }
    }

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct io_channel ch_cfg = (struct io_channel)config->io_channels[i];
        // LOG_DBG("AIN%u get delta AGAIN", i);
        int32_t dv = data->delta[i];
        if (dv != 0) {
#if CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN > 0
            last_rpt_time = now;
#endif
            data->delta[i] = 0;
            LOG_DBG("input_report %u rv: %d  e:%d  c:%d", i, dv, ch_cfg.evt_type, ch_cfg.input_code);
            input_report(dev, ch_cfg.evt_type, ch_cfg.input_code, dv, i == idx_to_sync, K_NO_WAIT);
        }
    }
    return 0;
}

K_THREAD_STACK_DEFINE(analog_input_q_stack, CONFIG_ANALOG_INPUT_WORKQUEUE_STACK_SIZE);

static struct k_work_q analog_input_work_q;

static void sampling_work_handler(struct k_work *work) {
    struct analog_input_data *data = CONTAINER_OF(work, struct analog_input_data, sampling_work);
    // LOG_DBG("sampling work triggered");
    analog_input_report_data(data->dev);
}

static void sampling_timer_handler(struct k_timer *timer) {
    struct analog_input_data *data = CONTAINER_OF(timer, struct analog_input_data, sampling_timer);
    // LOG_DBG("sampling timer triggered");
    k_work_submit_to_queue(&analog_input_work_q, &data->sampling_work);
    k_work_submit(&data->sampling_work);
}

static void analog_input_async_init(struct k_work *work) {
    struct k_work_delayable *work_delayable = (struct k_work_delayable *)work;
    struct analog_input_data *data = CONTAINER_OF(work_delayable, 
                                                  struct analog_input_data, init_work);
    const struct device *dev = data->dev;
    const struct analog_input_config *config = dev->config;

    // LOG_DBG("ANALOG_INPUT async init");
    uint32_t ch_mask = 0;

    for (uint8_t i = 0; i < config->io_channels_len; i++) {
        struct io_channel ch_cfg = (struct io_channel)config->io_channels[i];
        const struct device* adc = ch_cfg.adc_channel.dev;
        uint8_t channel_id = ch_cfg.adc_channel.channel_id;
        
        struct adc_channel_cfg channel_cfg = {
            .gain = ADC_GAIN_1_6,
            .reference = ADC_REF_INTERNAL,
            .acquisition_time = ADC_ACQ_TIME_DEFAULT,
            .channel_id = channel_id,
            #ifdef CONFIG_ADC_NRFX_SAADC
            .input_positive = SAADC_CH_PSELP_PSELP_AnalogInput0 + channel_id,
            #else
            .input_positive = channel_id,
            #endif
        };

        ch_mask |= BIT(channel_id);

        int err = adc_channel_setup(adc, &channel_cfg);
        if (err < 0) {
            LOG_ERR("AIN%u setup returned %d", i, err);
        }
    }

    uint16_t delta_size = config->io_channels_len * sizeof(int32_t);
    data->delta = malloc(delta_size);
    memset(data->delta, 0, delta_size);

    uint16_t buff_size = config->io_channels_len * sizeof(uint16_t);
    data->as_buff = malloc(buff_size);
    memset(data->as_buff, 0, buff_size);

    data->as = (struct adc_sequence){
        .channels = ch_mask,
        .buffer = data->as_buff,
        .buffer_size = buff_size,
        .oversampling = 0,
        .resolution = 12,
        .calibrate = true,
    };

    data->ready = true;

    k_work_init(&data->sampling_work, sampling_work_handler);
    k_work_queue_start(&analog_input_work_q,
                        analog_input_q_stack, K_THREAD_STACK_SIZEOF(analog_input_q_stack),
                        CONFIG_ANALOG_INPUT_WORKQUEUE_PRIORITY, NULL);

    k_timer_init(&data->sampling_timer, sampling_timer_handler, NULL);
    if (config->sampling_hz != 0) {
        uint32_t usec = 1000000UL / config->sampling_hz;
        k_timer_start(&data->sampling_timer, K_USEC(usec), K_USEC(usec));
    } else {
        k_timer_start(&data->sampling_timer, K_NO_WAIT, K_NO_WAIT);
    }

}

static int analog_input_init(const struct device *dev) {
    struct analog_input_data *data = dev->data;
    // const struct analog_input_config *config = dev->config;
    int err = 0;

    data->dev = dev;
    k_work_init_delayable(&data->init_work, analog_input_async_init);
    k_work_schedule(&data->init_work, K_MSEC(1));

    return err;
}

static int analog_input_attr_get(const struct device *dev, enum sensor_channel chan,
                                 enum sensor_attribute attr, struct sensor_value *val)
{
    const struct analog_input_config *config = dev->config;
	int ret = 0;
    if ((int)chan == (int)ANALOG_INPUT_CHAN_DRIVER_CONFIG) { // aka SENSOR_CHAN_PRIV_START
        switch (attr) {
        case ANALOG_INPUT_ATTR_INTPUT_CHANNEL: // aka SENSOR_ATTR_PRIV_START
            val->val1 = config->input_channel;
            break;
        default:
            LOG_ERR("ANALOG_INPUT attribute not supported.");
            return -ENOTSUP;
        }
    } else {
        LOG_ERR("ANALOG_INPUT attribute not supported.");
        return -ENOTSUP;
    }
	return ret;
}

static const struct sensor_driver_api analog_input_driver_api = {
	.attr_get = analog_input_attr_get,
};

#define TRANSFORMED_IO_CHANNEL_ENTRY(node_id)                                                      \
    {                                                                                              \
        .adc_channel = ADC_DT_SPEC_GET_BY_IDX(node_id, 0),                                         \
        .mv_mid = DT_PROP(node_id, mv_mid),                                                        \
        .mv_min_max = DT_PROP(node_id, mv_min_max),                                                \
        .mv_deadzone = DT_PROP(node_id, mv_deadzone),                                              \
        .invert = DT_PROP(node_id, invert),                                                        \
        .scale_multiplier = DT_PROP(node_id, scale_multiplier),                                    \
        .scale_divisor = DT_PROP(node_id, scale_divisor),                                          \
        .evt_type = DT_PROP(node_id, evt_type),                                                    \
        .input_code = DT_PROP(node_id, input_code),                                                \
    }

#define ANIN_IOC_CHILD_LEN_PLUS_ONE(node) 1 +

#define ANALOG_INPUT_DEFINE(n)                                                                     \
    static struct analog_input_data data##n = {                                                    \
    };                                                                                             \
    static const struct analog_input_config config##n = {                                          \
        .sampling_hz = DT_PROP(DT_DRV_INST(n), sampling_hz),                                       \
        .input_channel = DT_PROP(DT_DRV_INST(n), input_channel),                                   \
        .io_channels_len = (DT_FOREACH_CHILD(DT_DRV_INST(n), ANIN_IOC_CHILD_LEN_PLUS_ONE) 0),      \
        .io_channels = { DT_INST_FOREACH_CHILD_SEP(n, TRANSFORMED_IO_CHANNEL_ENTRY, (, )) },       \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, analog_input_init, NULL, &data##n, &config##n, POST_KERNEL,           \
                          CONFIG_SENSOR_INIT_PRIORITY, &analog_input_driver_api);

DT_INST_FOREACH_STATUS_OKAY(ANALOG_INPUT_DEFINE)
