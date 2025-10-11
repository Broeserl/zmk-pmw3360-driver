#define DT_DRV_COMPAT zmk_behavior_pmw3360_cpi

#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
#include <zmk/split/bluetooth/peripheral.h>
#endif

#include "../pmw3360_api.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

struct behavior_pmw3360_cpi_config {
    const struct device *sensor;
};

#if IS_ENABLED(CONFIG_ZMK_SPLIT)
// Define a custom split message type
#define ZMK_SPLIT_RUN_BEHAVIOR_PMW3360_CPI 0x60

static int peripheral_cpi_callback(uint8_t source, const uint8_t *data, size_t len) {
    if (len < 2) {
        return -EINVAL;
    }

    // data[0] = device index, data[1] = increase/decrease
    // For simplicity, we'll assume device 0
    const struct device *dev = DEVICE_DT_GET(DT_NODELABEL(trackball));

    if (!device_is_ready(dev)) {
        LOG_ERR("Sensor not ready on peripheral");
        return -ENODEV;
    }

    bool increase = (data[1] != 0);
    return pmw3360_cycle_cpi(dev, increase);
}

// Register the peripheral callback
static int register_split_callback(void) {
    return zmk_split_bt_peripheral_register_callback(
        ZMK_SPLIT_RUN_BEHAVIOR_PMW3360_CPI,
        peripheral_cpi_callback
    );
}
#endif

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_pmw3360_cpi_config *cfg = dev->config;

    bool increase = (binding->param1 != 0);

#if IS_ENABLED(CONFIG_ZMK_SPLIT) && IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // If we're on the central side, send message to peripheral
    if (!cfg->sensor || !device_is_ready(cfg->sensor)) {
        LOG_DBG("Sensor not on this side, sending to peripheral");
        uint8_t data[2] = {0, increase ? 1 : 0};
        return zmk_split_bt_invoke_behavior(
            0, // peripheral index
            ZMK_SPLIT_RUN_BEHAVIOR_PMW3360_CPI,
            data,
            sizeof(data)
        );
    }
#endif

    // If we're on the peripheral side (or unified), execute locally
    if (!cfg->sensor || !device_is_ready(cfg->sensor)) {
        LOG_ERR("PMW3360 sensor not ready");
        return -ENODEV;
    }

    return pmw3360_cycle_cpi(cfg->sensor, increase);
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                       struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw3360_cpi_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

static int behavior_pmw3360_cpi_init(const struct device *dev) {
#if IS_ENABLED(CONFIG_ZMK_SPLIT) && !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
    // Register callback on peripheral
    register_split_callback();
#endif
    return 0;
}

#define PMW3360_CPI_INST(n)                                                        \
    static const struct behavior_pmw3360_cpi_config behavior_pmw3360_cpi_config_##n = { \
        .sensor = COND_CODE_1(DT_NODE_EXISTS(DT_NODELABEL(trackball)),            \
                              (DEVICE_DT_GET(DT_NODELABEL(trackball))),            \
                              (NULL)),                                             \
    };                                                                             \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_pmw3360_cpi_init, NULL,                   \
                            NULL, &behavior_pmw3360_cpi_config_##n,                \
                            POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,      \
                            &behavior_pmw3360_cpi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_CPI_INST)