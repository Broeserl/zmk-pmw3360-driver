#pragma once

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

    /**
     * Cycle CPI to the next or previous preset value.
     *
     * @param dev PMW3360 device instance
     * @param increase true to increase CPI, false to decrease
     * @return 0 on success, negative errno on failure
     */
    int pmw3360_cycle_cpi(const struct device *dev, bool increase);

    /**
     * Set CPI to a specific value.
     *
     * @param dev PMW3360 device instance
     * @param cpi Target CPI value (must be valid per PMW3360 specs)
     * @return 0 on success, negative errno on failure
     */
    int pmw3360_set_cpi_direct(const struct device *dev, uint32_t cpi);

    /**
     * Get the current runtime CPI for a specific mode.
     *
     * @param dev PMW3360 device instance
     * @param mode The input mode to query (MOVE, SCROLL, or SNIPE)
     * @return Current CPI value for the mode, or 0 on error
     */
    uint32_t pmw3360_get_mode_cpi(const struct device *dev, enum pixart_input_mode mode);

#ifdef __cplusplus
}
#endif