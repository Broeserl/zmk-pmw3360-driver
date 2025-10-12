#pragma once

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * Adjust CPI of the currently active mode by a step amount.
     *
     * @param dev PMW3360 device instance
     * @param increase true to increase CPI, false to decrease
     * @return 0 on success, negative errno on failure
     */
    int pmw3360_adjust_cpi_step(const struct device *dev, bool increase);

    /**
     * Reset CPI of the currently active mode to its Kconfig default.
     *
     * @param dev PMW3360 device instance
     * @return 0 on success, negative errno on failure
     */
    int pmw3360_reset_cpi_to_default(const struct device *dev);

#ifdef __cplusplus
}
#endif