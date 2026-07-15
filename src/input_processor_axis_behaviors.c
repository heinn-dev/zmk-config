/*
 * Copyright (c) 2026 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_input_processor_axis_behaviors

#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <drivers/input_processor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/keymap.h>
#include <zmk/behavior.h>
#include <zmk/virtual_key_position.h>

struct axis_behaviors_config {
    uint8_t index;
    uint16_t type;
    uint16_t code;
    int32_t tick;
    const struct zmk_behavior_binding *bindings; /* [0] = negative, [1] = positive */
};

struct axis_behaviors_data {
    struct k_mutex lock;
    int32_t accum;
};

static void tap_binding(const struct axis_behaviors_config *cfg,
                        const struct zmk_behavior_binding *binding,
                        struct zmk_input_processor_state *state) {
    struct zmk_behavior_binding_event event = {
        .position = ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(state->input_device_index,
                                                                       cfg->index),
        .timestamp = k_uptime_get(),
#if IS_ENABLED(CONFIG_ZMK_SPLIT)
        .source = ZMK_POSITION_STATE_CHANGE_SOURCE_LOCAL,
#endif
    };

    /* Unlike the core behaviors input processor, we own both halves of the
     * press/release pair ourselves, since a continuous axis never emits a
     * natural "released" event the way a button does. */
    zmk_behavior_invoke_binding(binding, event, true);
    zmk_behavior_invoke_binding(binding, event, false);
}

static int axis_behaviors_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    const struct axis_behaviors_config *cfg = dev->config;
    struct axis_behaviors_data *data = dev->data;

    if (event->type != cfg->type || event->code != cfg->code) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    k_mutex_lock(&data->lock, K_FOREVER);

    data->accum += event->value;

    while (data->accum >= cfg->tick) {
        tap_binding(cfg, &cfg->bindings[1], state);
        data->accum -= cfg->tick;
    }
    while (data->accum <= -cfg->tick) {
        tap_binding(cfg, &cfg->bindings[0], state);
        data->accum += cfg->tick;
    }

    k_mutex_unlock(&data->lock);

    /* Consume the event - it's a volume/etc tap now, not cursor movement. */
    return ZMK_INPUT_PROC_STOP;
}

static int axis_behaviors_init(const struct device *dev) {
    struct axis_behaviors_data *data = dev->data;
    k_mutex_init(&data->lock);
    return 0;
}

static const struct zmk_input_processor_driver_api axis_behaviors_driver_api = {
    .handle_event = axis_behaviors_handle_event,
};

#define AXIS_BEHAVIORS_INST(n)                                                                     \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == 2,                                               \
                 "bindings must have exactly 2 entries: [negative, positive]");                    \
    static struct axis_behaviors_data axis_behaviors_data_##n = {};                                \
    static const struct zmk_behavior_binding axis_behaviors_bindings_##n[] = {                     \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}; \
    static const struct axis_behaviors_config axis_behaviors_config_##n = {                        \
        .index = n,                                                                                \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .code = DT_INST_PROP(n, code),                                                             \
        .tick = DT_INST_PROP(n, tick),                                                             \
        .bindings = axis_behaviors_bindings_##n,                                                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, axis_behaviors_init, NULL, &axis_behaviors_data_##n,                  \
                          &axis_behaviors_config_##n, POST_KERNEL,                                 \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &axis_behaviors_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_BEHAVIORS_INST)
