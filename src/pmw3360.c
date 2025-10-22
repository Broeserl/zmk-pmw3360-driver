/*
 * Copyright (c) 2022 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT pixart_pmw3360

// 12-bit two's complement value to int16_t
// adapted from https://stackoverflow.com/questions/70802306/convert-a-12-bit-signed-number-in-c
#define TOINT16(val, bits) (((struct { int16_t value : bits; }){val}).value)

#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>
#include <zmk/keymap.h>
#include "pmw3360.h"
#include <stdint.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3360, CONFIG_PMW3360_LOG_LEVEL);

/* SROM firmware meta-data, defined in pmw3360_piv.c */
extern const size_t pmw3360_firmware_length;
extern const uint8_t pmw3360_firmware_data[];

#define TABLE_SIZE 72
 
 // Lookup table for sine values (5-degree steps)
static const float sin_table[TABLE_SIZE] = {
     0.0,       0.0872,   0.1736,   0.2588,   0.3420,   0.4226,   0.5000,   0.5736,   0.6428,   0.7071,
     0.7660,    0.8192,   0.8660,   0.9063,   0.9397,   0.9659,   0.9848,   0.9962,   1.0000,   0.9962,
     0.9848,    0.9659,   0.9397,   0.9063,   0.8660,   0.8192,   0.7660,   0.7071,   0.6428,   0.5736,
     0.5000,    0.4226,   0.3420,   0.2588,   0.1736,   0.0872,   0.0,     -0.0872,  -0.1736,  -0.2588,
    -0.3420,   -0.4226,  -0.5000,  -0.5736,  -0.6428,  -0.7071,  -0.7660,  -0.8192,  -0.8660,  -0.9063,
    -0.9397,   -0.9659,  -0.9848,  -0.9962,  -1.0000,  -0.9962,  -0.9848,  -0.9659,  -0.9397,  -0.9063,
    -0.8660,   -0.8192,  -0.7660,  -0.7071,  -0.6428,  -0.5736,  -0.5000,  -0.4226,  -0.3420,  -0.2588,
    -0.1736,   -0.0872
};

// Lookup table for cosine values (5-degree steps)
static const float cos_table[TABLE_SIZE] = {
    1.0,       0.9962,   0.9848,   0.9659,   0.9397,   0.9063,   0.8660,   0.8192,   0.7660,   0.7071,
    0.6428,    0.5736,   0.5000,   0.4226,   0.3420,   0.2588,   0.1736,   0.0872,   0.0,     -0.0872,
   -0.1736,   -0.2588,  -0.3420,  -0.4226,  -0.5000,  -0.5736,  -0.6428,  -0.7071,  -0.7660,  -0.8192,
   -0.8660,   -0.9063,  -0.9397,  -0.9659,  -0.9848,  -0.9962,  -1.0,     -0.9962,  -0.9848,  -0.9659,
   -0.9397,   -0.9063,  -0.8660,  -0.8192,  -0.7660,  -0.7071,  -0.6428,  -0.5736,  -0.5000,  -0.4226,
   -0.3420,   -0.2588,  -0.1736,  -0.0872,   0.0,      0.0872,   0.1736,   0.2588,   0.3420,   0.4226,
    0.5000,    0.5736,   0.6428,   0.7071,   0.7660,   0.8192,   0.8660,   0.9063,   0.9397,   0.9659,
    0.9848,    0.9962
};

/* sensor initialization steps definition */
// init is done in non-bpmw3360_async_initlocking manner (i.e., async), a delayable work is defined for this job
// see pmw3360_init and pmw3360_async_init)

enum async_init_step {
    ASYNC_INIT_STEP_POWER_UP,         // power up reset
    ASYNC_INIT_STEP_FW_LOAD_START,    // clear motion registers, disable REST mode, enable SROM
                                      // register
    ASYNC_INIT_STEP_FW_LOAD_CONTINUE, // start SROM download
    ASYNC_INIT_STEP_FW_LOAD_VERIFY,   // verify SROM pid and fid, enable REST mode
    ASYNC_INIT_STEP_CONFIGURE,        // set cpi and donwshift time (run, rest1, rest2)

    ASYNC_INIT_STEP_COUNT // end flag
};

// delay (ms) in between steps
static const int32_t async_init_delay[ASYNC_INIT_STEP_COUNT] = {
    [ASYNC_INIT_STEP_POWER_UP] = 1,
    [ASYNC_INIT_STEP_FW_LOAD_START] = 50,    // required in spec
    [ASYNC_INIT_STEP_FW_LOAD_CONTINUE] = 10, // required in spec
    [ASYNC_INIT_STEP_FW_LOAD_VERIFY] = 1,
    [ASYNC_INIT_STEP_CONFIGURE] = 0,
};

static int pmw3360_async_init_power_up(const struct device *dev);
static int pmw3360_async_init_configure(const struct device *dev);
static int pmw3360_async_init_fw_load_verify(const struct device *dev);
static int pmw3360_async_init_fw_load_continue(const struct device *dev);
static int pmw3360_async_init_fw_load_start(const struct device *dev);

static int (*const async_init_fn[ASYNC_INIT_STEP_COUNT])(const struct device *dev) = {
    [ASYNC_INIT_STEP_POWER_UP] = pmw3360_async_init_power_up,
    [ASYNC_INIT_STEP_FW_LOAD_START] = pmw3360_async_init_fw_load_start,
    [ASYNC_INIT_STEP_FW_LOAD_CONTINUE] = pmw3360_async_init_fw_load_continue,
    [ASYNC_INIT_STEP_FW_LOAD_VERIFY] = pmw3360_async_init_fw_load_verify,
    [ASYNC_INIT_STEP_CONFIGURE] = pmw3360_async_init_configure,
};

/**
 * Control the SPI chip-select GPIO and enforce required NCS-SCLK timing.
 *
 * Toggles the configured CS GPIO to the specified state and applies the
 * T_NCS_SCLK delay before deasserting and after asserting the line to meet
 * the sensor's timing requirements.
 *
 * @param dev Device instance containing the CS GPIO configuration.
 * @param enable true to assert (enable) CS, false to deassert (disable) CS.
 * @returns 0 on success, or a negative errno value returned by gpio_pin_set_dt on failure.
 */
static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed: %d", err);
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
    int deassert_err = 0;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Write register address. */
    const struct spi_buf tx_buf = {.buf = &reg, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg read failed on SPI write: %d", err);
        goto deassert_cs;
    }

    k_busy_wait(T_SRAD);

    /* Read register value. */
    struct spi_buf rx_buf = {
        .buf = buf,
        .len = 1,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1,
    };

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Reg read failed on SPI read");
        goto deassert_cs;
    }

deassert_cs:
    deassert_err = spi_cs_ctrl(dev, false);
    if (deassert_err && !err) {
        err = deassert_err;
    }

    k_busy_wait(T_SRX);

    if (!err) {
        data->last_read_burst = false;
    }

    return err;
}

static int reg_write(const struct device *dev, uint8_t reg, uint8_t val) {
    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG((reg & SPI_WRITE_BIT) == 0);

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    uint8_t buf[] = {SPI_WRITE_BIT | reg, val};
    const struct spi_buf tx_buf = {.buf = buf, .len = ARRAY_SIZE(buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Reg write failed on SPI write");
        return err;
    }

    k_busy_wait(T_SCLK_NCS_WR);

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SWX);

    data->last_read_burst = false;

    return 0;
}

/**
 * Read a motion burst from the PMW3360 sensor into the provided buffer.
 *
 * @param buf Buffer to receive burst data; must be at least @p burst_size bytes.
 * @param burst_size Number of bytes to read from the motion burst; must be less than or equal to PMW3360_MAX_BURST_SIZE.
 * @returns 0 on success, negative errno on failure.
 *
 * On success fills @p buf with the burst data and sets the device's last_read_burst flag.
 */
static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {

    LOG_DBG("In burst read");
    int err;
    int deassert_err = 0;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    __ASSERT_NO_MSG(burst_size <= PMW3360_MAX_BURST_SIZE);

    /* Write any value to motion burst register only if there have been
     * other SPI transmissions with sensor since last burst read.
     */
    if (!data->last_read_burst) {
        err = reg_write(dev, PMW3360_REG_MOTION_BURST, 0x00);
        if (err) {
            return err;
        }
    }

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    /* Send motion burst address */
    uint8_t reg_buf[] = {PMW3360_REG_MOTION_BURST};
    const struct spi_buf tx_buf = {.buf = reg_buf, .len = ARRAY_SIZE(reg_buf)};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI write");
        goto deassert_cs;
    }

    k_busy_wait(T_SRAD_MOTBR);

    const struct spi_buf rx_buf = {
        .buf = buf,
        .len = burst_size,
    };
    const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    err = spi_read_dt(&config->bus, &rx);
    if (err) {
        LOG_ERR("Motion burst failed on SPI read");
        goto deassert_cs;
    }

deassert_cs:
    deassert_err = spi_cs_ctrl(dev, false);
    if (deassert_err && !err) {
        err = deassert_err;
    }
    k_busy_wait(T_BEXIT);
    if (!err) {
        data->last_read_burst = true;
    }
    return err;
}

/**
 * Write a sequence of bytes to a PMW3360 burst register.
 *
 * Performs a burst write to the specified sensor register and updates the device
 * state to reflect that the last operation was not a burst read.
 *
 * @param dev Pointer to the device instance.
 * @param reg Register address to write the burst to.
 * @param buf Buffer containing the bytes to write.
 * @param size Number of bytes to write from @p buf.
 * @returns 0 on success, or a negative errno code if SPI or chip-select control fails.
 */
static int burst_write(const struct device *dev, uint8_t reg, const uint8_t *buf, size_t size) {
    LOG_DBG("In burst write");
    int err;
    int deassert_err = 0;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    /* Write address of burst register */
    uint8_t write_buf = reg | SPI_WRITE_BIT;
    struct spi_buf tx_buf = {.buf = &write_buf, .len = 1};
    const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    err = spi_cs_ctrl(dev, true);
    if (err) {
        return err;
    }

    err = spi_write_dt(&config->bus, &tx);
    if (err) {
        LOG_ERR("Burst write failed on SPI write");
        goto deassert_cs;
    }

    /* Write data */
    for (size_t i = 0; i < size; i++) {
        write_buf = buf[i];
        tx_buf.buf = &write_buf;
        err = spi_write_dt(&config->bus, &tx);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            goto deassert_cs;
        }

        k_busy_wait(T_BRSEP);
    }

deassert_cs:
    /* Terminate burst mode. */
    deassert_err = spi_cs_ctrl(dev, false);
    if (deassert_err && !err) {
        err = deassert_err;
    }
    k_busy_wait(T_BEXIT);
    data->last_read_burst = false;
    return err;
}

/**
 * Configure the sensor CPI (counts per inch).
 *
 * Accepts a CPI value in 100‑CPI steps between PMW3360_MIN_CPI and PMW3360_MAX_CPI,
 * converts it to the device register encoding, and writes it to PMW3360_REG_CONFIG1.
 *
 * @param cpi Desired CPI; must be a multiple of 100 and within the supported range.
 * @returns 0 on success, negative errno on failure (for example, `-EINVAL` if `cpi` is out of range or if the register write fails).
 */
static int set_cpi(const struct device *dev, uint32_t cpi) {
    /* Set resolution with CPI step of 100 cpi
     * 0x00: 100 cpi (minimum cpi)
     * 0x01: 200 cpi
     * :
     * 0x31: 5000 cpi (default cpi)
     * :
     * 0x77: 12000 cpi (maximum cpi)
     */

    if ((cpi > PMW3360_MAX_CPI) || (cpi < PMW3360_MIN_CPI)) {
        LOG_ERR("CPI value %u out of range", cpi);
        return -EINVAL;
    }

    if ((cpi % 100) != 0) {
        LOG_ERR("CPI value %u must be a multiple of 100", cpi);
        return -EINVAL;
    }

    /* Convert CPI to register value */
    uint8_t value = (cpi / 100) - 1;

    LOG_DBG("Setting CPI to %u (reg value 0x%x)", cpi, value);

    int err = reg_write(dev, PMW3360_REG_CONFIG1, value);
    if (err) {
        LOG_ERR("Failed to change CPI");
    }

    return err;
}

/**
 * Configure a downshift interval (in milliseconds) for a PMW3360 downshift register.
 *
 * Validates that the requested time is within the supported range for the specified
 * downshift register, converts the time to the corresponding register value, and
 * writes it to the sensor.
 *
 * @param dev Pointer to the device instance.
 * @param reg_addr Downshift register address; one of:
 *                 - PMW3360_REG_RUN_DOWNSHIFT
 *                 - PMW3360_REG_REST1_DOWNSHIFT
 *                 - PMW3360_REG_REST2_DOWNSHIFT
 *                 The unit of `time` is milliseconds and each register has its own
 *                 resolution/multiplier.
 * @param time Requested downshift time in milliseconds.
 * @returns 0 on success.
 * @returns -ENOTSUP if `reg_addr` is not a supported downshift register.
 * @returns -EINVAL if `time` is outside the valid range for the selected register.
 * @returns a negative error code returned by the underlying register write on failure.
 */
static int set_downshift_time(const struct device *dev, uint8_t reg_addr, uint32_t time) {
    /* Set downshift time in ms:
     * - Run downshift time (from Run to Rest1 mode), default: 500ms
     * - Rest 1 downshift time (from Rest1 to Rest2 mode), default: 9.92 s
     * - Rest 2 downshift time (from Rest2 to Rest3 mode), default: ~10 min
     */
    uint32_t maxtime;
    uint32_t mintime;

    switch (reg_addr) {
    case PMW3360_REG_RUN_DOWNSHIFT:
        /*
         * Run downshift time = PMW3360_REG_RUN_DOWNSHIFT * 10 ms
         */
        maxtime = 2550;
        mintime = 10;
        break;

    case PMW3360_REG_REST1_DOWNSHIFT:
        /*
         * Rest1 downshift time = PMW3360_REG_RUN_DOWNSHIFT
         *                        * 320 * Rest1 rate (default 1 ms)
         */
        maxtime = 81600;
        mintime = 320;
        break;

    case PMW3360_REG_REST2_DOWNSHIFT:
        /*
         * Rest2 downshift time = PMW3360_REG_REST2_DOWNSHIFT
         *                        * 32 * Rest2 rate (default 100 ms)
         */
        maxtime = 816000;
        mintime = 3200;
        break;

    default:
        LOG_ERR("Not supported");
        return -ENOTSUP;
    }

    if ((time > maxtime) || (time < mintime)) {
        LOG_WRN("Downshift time %u out of range", time);
        return -EINVAL;
    }

    __ASSERT_NO_MSG((mintime > 0) && (maxtime / mintime <= UINT8_MAX));

    /* Convert time to register value (round to nearest) */
    uint32_t q = (time + (mintime / 2)) / mintime;
    if (q < 1) {
        q = 1;
    } else if (q > UINT8_MAX) {
        q = UINT8_MAX;
    }
    uint8_t value = (uint8_t)q;

    LOG_DBG("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

/**
 * Prepare the sensor for firmware download by reading status registers, disabling REST mode,
 * and initializing the SROM interface.
 *
 * Performs reads from registers 0x02–0x06, clears the REST enable bit in CONFIG2, and writes
 * the SROM init value to the SROM_ENABLE register.
 *
 * @returns 0 on success, negative errno on failure.
 */
static int pmw3360_async_init_fw_load_start(const struct device *dev) {
    int err = 0;

    /* Read from registers 0x02-0x06 regardless of the motion pin state. */
    for (uint8_t reg = 0x02; (reg <= 0x06) && !err; reg++) {
        uint8_t buf[1];
        err = reg_read(dev, reg, buf);
    }

    if (err) {
        LOG_ERR("Cannot read from data registers");
        return err;
    }

    /* Write 0 to Rest_En bit of Config2 register to disable Rest mode. */
    err = reg_write(dev, PMW3360_REG_CONFIG2, 0x00);
    if (err) {
        LOG_ERR("Cannot disable REST mode");
        return err;
    }

    /* Write 0x1D in SROM_enable register to initialize the operation */
    err = reg_write(dev, PMW3360_REG_SROM_ENABLE, PMW3360_SROM_INIT_VALUE);
    if (err) {
        LOG_ERR("Cannot initialize SROM");
        return err;
    }

    return err;
}

/**
 * Start the sensor's SROM download and upload the PMW3360 firmware.
 *
 * Initiates SROM download by writing the SROM start value, then uploads the
 * firmware blob to the SROM load burst register.
 *
 * @returns 0 on success, negative errno code on failure.
 */
static int pmw3360_async_init_fw_load_continue(const struct device *dev) {
    int err;

    LOG_DBG("Uploading optical sensor firmware...");

    /* Write 0x18 to SROM_enable to start SROM download */
    err = reg_write(dev, PMW3360_REG_SROM_ENABLE, PMW3360_SROM_START_VALUE);
    if (err) {
        LOG_ERR("Cannot start SROM download");
        return err;
    }

    /* Write SROM file into SROM_Load_Burst register.
     * Data must start with SROM_Load_Burst address.
     */
    err = burst_write(dev, PMW3360_REG_SROM_LOAD_BURST, pmw3360_firmware_data,
                      pmw3360_firmware_length);
    if (err) {
        LOG_ERR("Cannot write firmware to sensor");
    }

    return err;
}

/**
 * Verify sensor firmware and enable REST modes.
 *
 * Reads the SROM_ID and PRODUCT_ID registers to confirm the sensor is running
 * the expected firmware and product. On success writes PMW3360_CONFIG2_VALUE to
 * PMW3360_REG_CONFIG2 to enable REST modes.
 *
 * @returns 0 on success, -EIO if firmware or product ID mismatch, or a negative
 * errno value from register I/O on failure.
 */
static int pmw3360_async_init_fw_load_verify(const struct device *dev) {
    int err;

    /* Read the SROM_ID register to verify the firmware ID before any
     * other register reads or writes
     */

    uint8_t fw_id;
    err = reg_read(dev, PMW3360_REG_SROM_ID, &fw_id);
    if (err) {
        LOG_ERR("Cannot obtain firmware id");
        return err;
    }

    LOG_DBG("Optical chip firmware ID: 0x%x", fw_id);
    if (fw_id != PMW3360_FIRMWARE_ID) {
        LOG_ERR("Chip is not running from SROM!");
        return -EIO;
    }

    uint8_t product_id;
    err = reg_read(dev, PMW3360_REG_PRODUCT_ID, &product_id);
    if (err) {
        LOG_ERR("Cannot obtain product id");
        return err;
    }

    if (product_id != PMW3360_PRODUCT_ID) {
        LOG_ERR("Invalid product id!");
        return -EIO;
    }

    /* Write 0x20 to Config2 register for wireless mouse design.
     * This enables entering rest modes.
     */
    err = reg_write(dev, PMW3360_REG_CONFIG2, PMW3360_CONFIG2_REST_ENABLE);
    if (err) {
        LOG_ERR("Cannot enable REST modes");
    }
    LOG_DBG("Finished firmware load verify");
    return err;
}

/**
 * Configure the device's IRQ GPIO interrupt enable state.
 *
 * Sets the IRQ GPIO described in the device configuration to active level interrupt
 * when `en` is true, or disables the interrupt when `en` is false. Logs an error
 * if configuring the GPIO interrupt fails.
 *
 * @param en true to enable the IRQ interrupt, false to disable it.
 */
static void set_interrupt(const struct device *dev, const bool en) {
    LOG_DBG("In pmw3360_set_interrupt");
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

/**
 * Selects the PixArt input mode based on the currently active keymap layer.
 *
 * @returns `SCROLL` if the active layer is listed in the device's scroll_layers,
 *          `SNIPE` if the active layer is listed in the device's snipe_layers,
 *          `MOVE` otherwise.
 */
static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
    const struct pixart_config *config = dev->config;
    uint8_t curr_layer = zmk_keymap_highest_layer_active();
    for (size_t i = 0; i < config->scroll_layers_len; i++) {
        if (curr_layer == config->scroll_layers[i]) {
            return SCROLL;
        }
    }
    for (size_t i = 0; i < config->snipe_layers_len; i++) {
        if (curr_layer == config->snipe_layers[i]) {
            return SNIPE;
        }
    }
    return MOVE;
}

/**
 * Update the device CPI (counts per inch) only when the requested value differs from the cached value.
 *
 * If the CPI is changed successfully, the cached current CPI is updated.
 *
 * @param dev Pointer to the device instance.
 * @param cpi Requested CPI value to apply.
 * @returns `0` on success, negative errno returned by the underlying CPI set operation on failure.
 */
static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    LOG_DBG("In pmw3360_set_cpi_if_needed");
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) {
        int err = set_cpi(dev, cpi);
        if (0 == err) {
            data->curr_cpi = cpi;
        }
        return err;
    }
    return 0;
}

/**
 * Rotate 2D coordinates by a given angle using precomputed 5-degree-step lookup tables.
 * @param x Input X coordinate.
 * @param y Input Y coordinate.
 * @param angle_degrees Rotation angle in degrees; normalized to the range [0, 359]. Best results when angle is a multiple of 5.
 * @param x_out Pointer to receive the rotated X coordinate (stored as int16_t).
 * @param y_out Pointer to receive the rotated Y coordinate (stored as int16_t).
 */
 static void rotate_coordinates(int16_t x, int16_t y, int angle_degrees, int16_t *x_out, int16_t *y_out) {
    // Ensure the angle is within valid range [0, 360)
    // Since Kconfig enforces 0-355 in 5-degree steps, normalize just in case
    angle_degrees = angle_degrees % 360;
    if (angle_degrees < 0) {
        angle_degrees += 360;
    }

    // Determine the index in the lookup table
    int index = angle_degrees / 5;

    // Safety check (should never happen with proper Kconfig)
    if (index >= TABLE_SIZE) {
        LOG_ERR("Invalid angle %d", angle_degrees);
        index = 0;
    }

    // Get sine and cosine values from the lookup table
    float cos_angle = cos_table[index];
    float sin_angle = sin_table[index];

    // Apply the rotation matrix
    *x_out = (int16_t)(x * cos_angle - y * sin_angle);
    *y_out = (int16_t)(x * sin_angle + y * cos_angle);
}

/**
 * Read motion data from the sensor, transform it according to current mode and
 * configuration, and report relative X/Y movements to the input subsystem.
 *
 * The function selects CPI and scaling based on the current input mode
 * (MOVE/SCROLL/SNIPE), ensures the sensor is configured accordingly, reads a
 * motion burst, applies per-config divisor, rotates and optionally inverts the
 * axes, and emits relative X/Y reports when movement is non-zero.
 *
 * @param dev Device instance for the PMW3360 sensor.
 * @returns `0` on success; a negative errno on failure:
 *          `-EBUSY` if the device is not initialized yet,
 *          `-ENOTSUP` if the input mode is unsupported,
 *          or error codes propagated from CPI setup or motion-burst read operations.
 */
static int pmw3360_report_data(const struct device *dev) {
    LOG_DBG("In pmw3360_report_data");
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3360_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t divisor;
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    int err = 0;

    switch (input_mode) {
    case MOVE:
        LOG_DBG("MOVE mode");
        err = set_cpi_if_needed(dev, data->move_cpi);
        divisor = CONFIG_PMW3360_CPI_DIVISOR;
        break;
    case SCROLL:
        LOG_DBG("SCROLL mode");
        err = set_cpi_if_needed(dev, data->scroll_cpi);
        divisor = CONFIG_PMW3360_SCROLL_CPI_DIVISOR;
        break;
    case SNIPE:
        LOG_DBG("SNIPE mode");
        err = set_cpi_if_needed(dev, data->snipe_cpi);
        divisor = CONFIG_PMW3360_SNIPE_CPI_DIVISOR;
        break;
    default:
        return -ENOTSUP;
    }

    if (err) {
        LOG_ERR("Failed to set CPI: %d", err);
        return err;
    }

    if (unlikely(divisor < 1)) {
        LOG_ERR("Invalid divisor %d", divisor);
        return -EINVAL;
    }

    data->curr_mode = input_mode;

    err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        return err;
    }

    __ASSERT_NO_MSG(divisor > 0);

    // Add bounds checking
    if (PMW3360_DX_POS + 1 >= sizeof(buf) || PMW3360_DY_POS + 1 >= sizeof(buf)) {
        LOG_ERR("Motion data position out of bounds");
        return -EINVAL;
    }

    int16_t raw_x = ((int16_t)TOINT16(buf[PMW3360_DX_POS] | (buf[PMW3360_DX_POS+1] << 8), 12)) / divisor;
    int16_t raw_y = ((int16_t)TOINT16(buf[PMW3360_DY_POS] | (buf[PMW3360_DY_POS+1] << 8), 12)) / divisor;
    int16_t x, y;

    // Rotate the coordinates
    int angle = CONFIG_PMW3360_ROTATION_ANGLE_DEG;
    rotate_coordinates(raw_x, raw_y, angle, &x, &y);

    if (IS_ENABLED(CONFIG_PMW3360_INVERT_X)) {
        x = -x;
    }

    if (IS_ENABLED(CONFIG_PMW3360_INVERT_Y)) {
        y = -y;
    }

    if (x != 0 || y != 0) {
        input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
        input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
    }

    return err;
}

/**
 * Perform a power-up reset of the PMW3360 sensor.
 *
 * @returns 0 on success, negative errno code on failure.
 */
static int pmw3360_async_init_power_up(const struct device *dev) {
    /* Reset sensor */
    LOG_DBG("async_init_power_up");

    return reg_write(dev, PMW3360_REG_POWER_UP_RESET, PMW3360_POWERUP_CMD_RESET);
}

/**
 * Configure the PMW3360 sensor after firmware load.
 *
 * Sets the initial CPI and programs run/rest downshift timing registers; on success
 * caches the configured CPI in the device state.
 *
 * @param dev PMW3360 device instance.
 * @returns 0 on success, negative errno code on failure.
 */
static int pmw3360_async_init_configure(const struct device *dev) {
    LOG_DBG("pmw3360_async_init_configure");
    int err;
    struct pixart_data *data = dev->data;

    // Set initial CPI
    data->move_cpi = CONFIG_PMW3360_CPI;
    data->scroll_cpi = CONFIG_PMW3360_SCROLL_CPI;
    data->snipe_cpi = CONFIG_PMW3360_SNIPE_CPI;

    err = set_cpi(dev, CONFIG_PMW3360_CPI);
    if (err == 0) {
        data->curr_cpi = CONFIG_PMW3360_CPI;
    }

    if (!err) {
        err = set_downshift_time(dev, PMW3360_REG_RUN_DOWNSHIFT,
                                 CONFIG_PMW3360_RUN_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_downshift_time(dev, PMW3360_REG_REST1_DOWNSHIFT,
                                 CONFIG_PMW3360_REST1_DOWNSHIFT_TIME_MS);
    }

    if (!err) {
        err = set_downshift_time(dev, PMW3360_REG_REST2_DOWNSHIFT,
                                 CONFIG_PMW3360_REST2_DOWNSHIFT_TIME_MS);
    }

    return err;
}

/**
 * Perform a single step of the driver's asynchronous initialization state machine.
 *
 * Invoked by the Zephyr workqueue; runs the function for the current async init
 * step, stores any error in the device's runtime state, and either advances and
 * reschedules the next step after its configured delay or marks the device ready
 * and enables interrupts when initialization completes.
 *
 * @param work Work item provided by the Zephyr workqueue; must embed the driver's
 *             init_work member so the containing pixart_data can be retrieved.
 */
static void pmw3360_async_init(struct k_work *work) {
    LOG_DBG("pmw3360_async_init");
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_DBG("async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("initialization failed at step %d: %d", data->async_init_step, data->err);
    } else {
        data->async_init_step++;

        if (data->async_init_step == ASYNC_INIT_STEP_COUNT) {
            data->ready = true; // sensor is ready to work
            LOG_INF("PMW3360 initialized");
            set_interrupt(dev, true);
        } else {
            k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));
        }
    }
}

/**
 * Configure the device's IRQ GPIO pin and register a GPIO callback handler.
 *
 * Initializes the IRQ GPIO as an input, attaches the provided callback to the
 * device's gpio callback structure, and adds that callback to the GPIO port.
 *
 * @param dev Pointer to the device instance containing config and runtime data.
 * @param callback_handler GPIO callback function to invoke on IRQ events.
 * @returns 0 on success, -ENODEV if the IRQ GPIO device is not ready, or a
 *          negative errno value returned by GPIO configuration or callback
 *          registration on failure.
 */
static int pmw3360_init_irq(const struct device *dev, gpio_callback_handler_t callback_handler) {
    LOG_DBG("Configure irq...");

    int err;
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;

    // check readiness of irq gpio pin
    if (!device_is_ready(config->irq_gpio.port)) {
        LOG_ERR("IRQ GPIO device not ready");
        return -ENODEV;
    }

    // init the irq pin
    err = gpio_pin_configure_dt(&config->irq_gpio, GPIO_INPUT);
    if (err) {
        LOG_ERR("Cannot configure IRQ GPIO");
        return err;
    }
    // setup and add the irq callback associated
    gpio_init_callback(&data->irq_gpio_cb, callback_handler, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    LOG_INF("Configure irq done");

    return err;
}

static int pmw3360_init_common(const struct device *dev,
                              gpio_callback_handler_t callback_handler,
                              void (*work_init_func)(const struct device *dev)) {
    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    // check readiness of cs gpio pin and init it to inactive
    if (!device_is_ready(config->cs_gpio.port)) {
        LOG_ERR("SPI CS device not ready");
        return -ENODEV;
    }

    err = gpio_pin_configure_dt(&config->cs_gpio, GPIO_OUTPUT_INACTIVE);
    if (err) {
        LOG_ERR("Cannot configure SPI CS GPIO");
        return err;
    }

    // Let the mode-specific function initialize work/timer structures
    if (work_init_func != NULL) {
        work_init_func(dev);
    }

    // init irq routine
    err = pmw3360_init_irq(dev, callback_handler);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs
    k_work_init_delayable(&data->init_work, pmw3360_async_init);
    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

    return err;
}

#if defined(CONFIG_PMW3360_INTERRUPT_DIRECT)
/**
 * GPIO interrupt callback for direct-interrupt (trigger) mode.
 *
 * Disables further device interrupts, applies a short blocking delay to rate-limit
 * event handling, and enqueues the driver's trigger work to process the event.
 *
 * @param gpiob GPIO device that raised the interrupt.
 * @param cb Pointer to the GPIO callback structure embedded in driver state.
 * @param pins Bitmask of pins that triggered the callback.
 */
static void pmw3360_gpio_callback_direct_mode(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    LOG_DBG("In pmw3360_gpio_callback");
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    // ugly, ugly, ugly hack, to avoid overwhelming the BLE interface
    k_busy_wait(CONFIG_PMW3360_INTERRUPT_DIRECT_DELAY_MS * 1000);
    // submit the real handler work
    k_work_submit(&data->trigger_work);
}

/**
 * Handle scheduled motion-report work for the PMW3360 device.
 *
 * Reads and reports sensor motion data, then re-enables the device interrupt.
 *
 * @param work Work item corresponding to the device's trigger_work.
 */
static void pmw3360_work_callback(struct k_work *work) {
    LOG_DBG("In pmw3360_work_callback");
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3360_report_data(dev);
    set_interrupt(dev, true);
}

/**
 * Initialize the work item used to handle direct-interrupt triggers.
 *
 * @param dev Pointer to the PMW3360 device instance.
 */
static void direct_mode_work_init(const struct device *dev) {
    struct pixart_data *data = dev->data;
    k_work_init(&data->trigger_work, pmw3360_work_callback);
}

/**
 * Initialize the device for direct (hardware) interrupt operation.
 *
 * Performs driver initialization required to operate in direct interrupt mode,
 * including GPIO callback and work setup specific to that mode.
 *
 * @returns 0 on success, negative errno on failure.
 */
static int pmw3360_init_interrupt_direct_mode(const struct device *dev) {
    LOG_DBG("Start initializing direct interrupt mode...");
    return pmw3360_init_common(dev, pmw3360_gpio_callback_direct_mode, direct_mode_work_init);
}

#elif defined(CONFIG_PMW3360_INTERRUPT_POLLING)

/**
 * GPIO callback for polling interrupt mode that disables the GPIO interrupt and starts the polling timer.
 *
 * Disables further hardware interrupts for the device and starts the device's polling timer with the
 * interval CONFIG_PMW3360_POLLING_INTERVAL_MS to perform deferred/polled motion processing.
 *
 * @param gpiob GPIO device that invoked the callback (unused).
 * @param cb Pointer to the GPIO callback structure for this device.
 * @param pins Bitmask of triggered pins (unused).
 */
static void pmw3360_gpio_callback_polling_mode(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    LOG_DBG("In pmw3360_gpio_callback");
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    // start the polling timer
    k_timer_start(&data->poll_timer, K_NO_WAIT, K_MSEC(CONFIG_PMW3360_POLLING_INTERVAL_MS));
}

/**
 * Polling workqueue handler that reports motion data from the PMW3360 device.
 *
 * Retrieves the enclosing pixart_data from the given work item and triggers
 * a data report cycle by calling pmw3360_report_data().
 *
 * @param work Work item corresponding to the device's poll_work (container is pixart_data).
 */
static void trackball_poll_handler(struct k_work *work) {
    LOG_DBG("In polling handler callback");
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, poll_work);
    const struct device *dev = data->dev;

    pmw3360_report_data(dev);
}

/**
 * Handle polling timer expiry by scheduling the polling work or stopping the timer when the max poll count is reached.
 *
 * When the embedded poll timer expires, this function submits the device's poll work to the workqueue and
 * increments the per-device polling counter. If the counter has reached CONFIG_PMW3360_MAX_POLL_COUNT,
 * the timer is stopped instead of scheduling additional work.
 *
 * @param timer Pointer to the expired kernel timer (embedded in struct pixart_data).
 */
static void polling_timer_expiry(struct k_timer *timer) {
    struct pixart_data *data = CONTAINER_OF(timer, struct pixart_data, poll_timer);

    // check whether reaching the polling count limit
    if (data->polling_count < CONFIG_PMW3360_MAX_POLL_COUNT) {
        // submit polling work to mouse work queue
        k_work_submit(&data->poll_work);

        // update status
        data->polling_count++;
    } else {
        // stop timer
        k_timer_stop(&data->poll_timer);
    }
}

/**
 * Stop the polling timer, reset the device's polling state, and re-enable the motion IRQ line.
 *
 * Stops the poll timer associated with the device, sets the device's polling counter to zero,
 * and resumes the motion interrupt by enabling the IRQ.
 *
 * @param timer Pointer to the Zephyr kernel timer instance embedded in the device's pixart_data.
 */
static void polling_timer_stop(struct k_timer *timer) {
    struct pixart_data *data = CONTAINER_OF(timer, struct pixart_data, poll_timer);
    const struct device *dev = data->dev;

    // Disable interrupts to prevent race condition
    unsigned int key = irq_lock();

    // reset polling count
    data->polling_count = 0;

    // resume motion interrupt line
    set_interrupt(dev, true);

    irq_unlock(key);
}

/**
 * Initialize the device's polling-mode timer and work item.
 *
 * Configures the instance's poll timer with the expiry and stop callbacks
 * and initializes the work item to run the trackball polling handler.
 *
 * @param dev Device instance whose polling timer and work are initialized.
 */
static void polling_mode_work_init(const struct device *dev) {
    struct pixart_data *data = dev->data;

    // reset polling count
    data->polling_count = 0;

    k_timer_init(&data->poll_timer, polling_timer_expiry, polling_timer_stop);
    k_work_init(&data->poll_work, trackball_poll_handler);
}

/**
 * Initialize the driver to use timer-based polling for interrupts.
 *
 * @returns 0 on success, or a negative errno value on failure.
 */
static int pmw3360_init_interrupt_polling_mode(const struct device *dev) {
    LOG_DBG("Start initializing timer-based polling mode...");
    return pmw3360_init_common(dev, pmw3360_gpio_callback_polling_mode, polling_mode_work_init);
}
#endif

/**
 * Get the input mode for the currently active layer.
 *
 * @param dev PMW3360 device instance
 * @return Current input mode (MOVE, SCROLL, or SNIPE)
 */
static enum pixart_input_mode pmw3360_get_current_mode(const struct device *dev) {
    return get_input_mode_for_current_layer(dev);
}

/**
 * Adjust CPI of the currently active mode by a step amount.
 *
 * @param dev PMW3360 device instance
 * @param increase true to increase CPI, false to decrease
 * @return 0 on success, negative errno on failure
 */
int pmw3360_adjust_cpi_step(const struct device *dev, bool increase) {
    struct pixart_data *data = dev->data;
    enum pixart_input_mode mode = pmw3360_get_current_mode(dev);

    uint32_t *target_cpi;
    uint32_t step;
    uint32_t min_cpi, max_cpi;

    // Determine which CPI to adjust based on current mode
    switch (mode) {
        case MOVE:
            target_cpi = &data->move_cpi;
            step = CONFIG_PMW3360_CPI_STEP;
            min_cpi = PMW3360_MIN_CPI;
            max_cpi = PMW3360_MAX_CPI;
            LOG_INF("Adjusting MOVE CPI");
            break;
        case SCROLL:
            target_cpi = &data->scroll_cpi;
            step = CONFIG_PMW3360_SCROLL_CPI_STEP;
            min_cpi = PMW3360_MIN_CPI;
            max_cpi = PMW3360_MAX_CPI;
            LOG_INF("Adjusting SCROLL CPI");
            break;
        case SNIPE:
            target_cpi = &data->snipe_cpi;
            step = CONFIG_PMW3360_SNIPE_CPI_STEP;
            min_cpi = 200; // Snipe mode minimum from Kconfig
            max_cpi = 3000; // Snipe mode maximum from Kconfig
            LOG_INF("Adjusting SNIPE CPI");
            break;
        default:
            LOG_ERR("Unknown input mode");
            return -EINVAL;
    }

    uint32_t new_cpi = *target_cpi;

    if (increase) {
        // Increase CPI, but don't exceed maximum
        if (new_cpi + step <= max_cpi) {
            new_cpi += step;
        } else {
            new_cpi = max_cpi;
        }
    } else {
        // Decrease CPI, but don't go below minimum
        if (new_cpi >= min_cpi + step) {
            new_cpi -= step;
        } else {
            new_cpi = min_cpi;
        }
    }

    // Round to nearest 100 (PMW3360 requirement)
    new_cpi = ((new_cpi + 50) / 100) * 100;

    // Clamp to valid range
    if (new_cpi < min_cpi) {
        new_cpi = min_cpi;
    }
    if (new_cpi > max_cpi) {
        new_cpi = max_cpi;
    }

    LOG_INF("CPI: %u -> %u", *target_cpi, new_cpi);

    *target_cpi = new_cpi;

    // If we're currently in this mode, apply the change immediately
    if (data->curr_mode == mode) {
        return set_cpi_if_needed(dev, new_cpi);
    }

    return 0;
}

/**
 * Reset CPI of the currently active mode to its Kconfig default.
 *
 * @param dev PMW3360 device instance
 * @return 0 on success, negative errno on failure
 */
int pmw3360_reset_cpi_to_default(const struct device *dev) {
    struct pixart_data *data = dev->data;
    enum pixart_input_mode mode = pmw3360_get_current_mode(dev);

    uint32_t *target_cpi;
    uint32_t default_cpi;

    // Determine which CPI to reset based on current mode
    switch (mode) {
        case MOVE:
            target_cpi = &data->move_cpi;
            default_cpi = CONFIG_PMW3360_CPI;
            LOG_INF("Resetting MOVE CPI to default: %u", default_cpi);
            break;
        case SCROLL:
            target_cpi = &data->scroll_cpi;
            default_cpi = CONFIG_PMW3360_SCROLL_CPI;
            LOG_INF("Resetting SCROLL CPI to default: %u", default_cpi);
            break;
        case SNIPE:
            target_cpi = &data->snipe_cpi;
            default_cpi = CONFIG_PMW3360_SNIPE_CPI;
            LOG_INF("Resetting SNIPE CPI to default: %u", default_cpi);
            break;
        default:
            LOG_ERR("Unknown input mode");
            return -EINVAL;
    }

    *target_cpi = default_cpi;

    // If we're currently in this mode, apply the change immediately
    if (data->curr_mode == mode) {
        return set_cpi_if_needed(dev, default_cpi);
    }

    return 0;
}


static int pmw3360_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    int err;
#if defined(CONFIG_PMW3360_INTERRUPT_DIRECT)
    err = pmw3360_init_interrupt_direct_mode(dev);
#elif defined(CONFIG_PMW3360_INTERRUPT_POLLING)
    err = pmw3360_init_interrupt_polling_mode(dev);
#endif

    return err;
}

#define PMW3360_DEFINE(n)                                                                          \
    static struct pixart_data data##n;                                                             \
    static int32_t scroll_layers##n[] = DT_PROP(DT_DRV_INST(n), scroll_layers);                    \
    static int32_t snipe_layers##n[] = DT_PROP(DT_DRV_INST(n), snipe_layers);                      \
    static const struct pixart_config config##n = {                                                \
        .irq_gpio = GPIO_DT_SPEC_INST_GET(n, irq_gpios),                                           \
        .bus =                                                                                     \
            {                                                                                      \
                .bus = DEVICE_DT_GET(DT_INST_BUS(n)),                                              \
                .config =                                                                          \
                    {                                                                              \
                        .frequency = DT_INST_PROP(n, spi_max_frequency),                           \
                        .operation =                                                               \
                            SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA,    \
                        .slave = DT_INST_REG_ADDR(n),                                              \
                    },                                                                             \
            },                                                                                     \
        .cs_gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_DRV_INST(n)),                                       \
        .scroll_layers = scroll_layers##n,                                                         \
        .scroll_layers_len = DT_PROP_LEN(DT_DRV_INST(n), scroll_layers),                           \
        .snipe_layers = snipe_layers##n,                                                           \
        .snipe_layers_len = DT_PROP_LEN(DT_DRV_INST(n), snipe_layers),                             \
    };                                                                                             \
                                                                                                   \
    DEVICE_DT_INST_DEFINE(n, pmw3360_init, NULL, &data##n, &config##n, POST_KERNEL,                \
                          CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3360_DEFINE)