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
    int32_t tap_interval_ms;
    const struct zmk_behavior_binding *bindings; /* [0] = negative, [1] = positive */
};

struct axis_behaviors_data {
    const struct device *dev;
    struct k_work_delayable work;
    struct k_spinlock lock;
    int32_t accum;
    uint8_t input_device_index;
};

static void tap_binding(const struct axis_behaviors_config *cfg,
                        const struct zmk_behavior_binding *binding, uint8_t input_device_index) {
    struct zmk_behavior_binding_event event = {
        .position =
            ZMK_VIRTUAL_KEY_POSITION_BEHAVIOR_INPUT_PROCESSOR(input_device_index, cfg->index),
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

/* Runs on the system workqueue - the same context ordinary key events use -
 * and taps at most one binding per run, rescheduling itself while a full
 * tick remains accumulated. Behaviors must NOT be invoked from
 * handle_event: input events for a split input are delivered on the BT RX
 * thread, and pushing HID reports from there can block that thread against
 * a full report queue, deadlocking the whole central (every peripheral
 * appears dead until reset). The tap interval also caps how fast taps can
 * fire, so a quick flick can't flood the HID queue and drop a key release
 * (observable as volume "sticking" and running away to zero). */
static void axis_behaviors_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct axis_behaviors_data *data = CONTAINER_OF(dwork, struct axis_behaviors_data, work);
    const struct axis_behaviors_config *cfg = data->dev->config;

    int dir = 0;
    bool more;
    uint8_t idx;

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    if (data->accum >= cfg->tick) {
        data->accum -= cfg->tick;
        dir = 1;
    } else if (data->accum <= -cfg->tick) {
        data->accum += cfg->tick;
        dir = -1;
    }
    more = (data->accum >= cfg->tick) || (data->accum <= -cfg->tick);
    idx = data->input_device_index;
    k_spin_unlock(&data->lock, key);

    if (dir == 0) {
        return;
    }

    tap_binding(cfg, &cfg->bindings[dir > 0 ? 1 : 0], idx);

    if (more) {
        k_work_schedule(&data->work, K_MSEC(cfg->tap_interval_ms));
    }
}

static int axis_behaviors_handle_event(const struct device *dev, struct input_event *event,
                                       uint32_t param1, uint32_t param2,
                                       struct zmk_input_processor_state *state) {
    const struct axis_behaviors_config *cfg = dev->config;
    struct axis_behaviors_data *data = dev->data;

    if (event->type != cfg->type || event->code != cfg->code) {
        return ZMK_INPUT_PROC_CONTINUE;
    }

    k_spinlock_key_t key = k_spin_lock(&data->lock);
    data->accum += event->value;
    /* Clamp the backlog to two ticks: enough to keep taps flowing between
     * work runs, but movement stops producing taps almost as soon as the
     * ball stops, instead of draining a long queued run. */
    if (data->accum > 2 * cfg->tick) {
        data->accum = 2 * cfg->tick;
    } else if (data->accum < -2 * cfg->tick) {
        data->accum = -2 * cfg->tick;
    }
    data->input_device_index = state->input_device_index;
    k_spin_unlock(&data->lock, key);

    /* No-op if already scheduled, so the interval doubles as a rate limit. */
    k_work_schedule(&data->work, K_MSEC(cfg->tap_interval_ms));

    /* Consume the event - it's a volume/etc tap now, not cursor movement.
     * The listener discards a STOP verdict coming from a layer-override
     * chain (filter_with_input_config() turns it into "continue"), so the
     * value must ALSO be zeroed or the motion still reaches the cursor. */
    event->value = 0;
    return ZMK_INPUT_PROC_STOP;
}

static int axis_behaviors_init(const struct device *dev) {
    struct axis_behaviors_data *data = dev->data;
    data->dev = dev;
    k_work_init_delayable(&data->work, axis_behaviors_work_cb);
    return 0;
}

static const struct zmk_input_processor_driver_api axis_behaviors_driver_api = {
    .handle_event = axis_behaviors_handle_event,
};

#define AXIS_BEHAVIORS_INST(n)                                                                     \
    BUILD_ASSERT(DT_INST_PROP_LEN(n, bindings) == 2,                                               \
                 "bindings must have exactly 2 entries: [negative, positive]");                    \
    BUILD_ASSERT(DT_INST_PROP(n, tick) > 0, "tick must be positive");                              \
    static struct axis_behaviors_data axis_behaviors_data_##n = {};                                \
    static const struct zmk_behavior_binding axis_behaviors_bindings_##n[] = {                     \
        LISTIFY(DT_INST_PROP_LEN(n, bindings), ZMK_KEYMAP_EXTRACT_BINDING, (, ), DT_DRV_INST(n))}; \
    static const struct axis_behaviors_config axis_behaviors_config_##n = {                        \
        .index = n,                                                                                \
        .type = DT_INST_PROP_OR(n, type, INPUT_EV_REL),                                            \
        .code = DT_INST_PROP(n, code),                                                             \
        .tick = DT_INST_PROP(n, tick),                                                             \
        .tap_interval_ms = DT_INST_PROP(n, tap_interval_ms),                                       \
        .bindings = axis_behaviors_bindings_##n,                                                   \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(n, axis_behaviors_init, NULL, &axis_behaviors_data_##n,                  \
                          &axis_behaviors_config_##n, POST_KERNEL,                                 \
                          CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &axis_behaviors_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AXIS_BEHAVIORS_INST)
