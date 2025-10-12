#include <zephyr/device.h>
#include <drivers/behavior.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

#include "../pmw3360_api.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define DT_DRV_COMPAT zmk_behavior_pmw3360_cpi_dec

static int on_cpi_dec_pressed(struct zmk_behavior_binding *binding,
                               struct zmk_behavior_binding_event event) {
    const struct device *sensor = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(trackball));

    if (!sensor || !device_is_ready(sensor)) {
        LOG_WRN("PMW3360 sensor not available on this half - ignoring CPI decrease");
        return 0;
    }

    return pmw3360_adjust_cpi_step(sensor, false);
}

static int on_cpi_dec_released(struct zmk_behavior_binding *binding,
                                struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_pmw3360_cpi_dec_driver_api = {
    .binding_pressed = on_cpi_dec_pressed,
    .binding_released = on_cpi_dec_released,
};

static int behavior_pmw3360_cpi_dec_init(const struct device *dev) {
    return 0;
}

BEHAVIOR_DT_INST_DEFINE(0, behavior_pmw3360_cpi_dec_init, NULL, NULL, NULL,
                      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
                      &behavior_pmw3360_cpi_dec_driver_api);