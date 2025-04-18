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
//#include <zephyr/keymap.h>
#include "pmw3360.h"
#include <stdint.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3360, CONFIG_PMW3360_LOG_LEVEL);


/* SROM firmware meta-data, defined in pmw3360_piv.c */
extern const size_t pmw3360_firmware_length;
extern const uint8_t pmw3360_firmware_data[];

#define TABLE_SIZE 72
 
 // Lookup table for sine values (5-degree steps)
 const float sin_table[TABLE_SIZE] = {
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
 const float cos_table[TABLE_SIZE] = {
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

static int spi_cs_ctrl(const struct device *dev, bool enable) {
    const struct pixart_config *config = dev->config;
    int err;

    if (!enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    err = gpio_pin_set_dt(&config->cs_gpio, (int)enable);
    if (err) {
        LOG_ERR("SPI CS ctrl failed");
    }

    if (enable) {
        k_busy_wait(T_NCS_SCLK);
    }

    LOG_INF("finished spi_cs_ctrl");
    return err;
}

static int reg_read(const struct device *dev, uint8_t reg, uint8_t *buf) {
    int err;
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
        LOG_ERR("Reg read failed on SPI write");
        return err;
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
        return err;
    }

    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_SRX);

    data->last_read_burst = false;

    return 0;
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

static int motion_burst_read(const struct device *dev, uint8_t *buf, size_t burst_size) {

    LOG_INF("In burst read");
    int err;
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
        return err;
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
        return err;
    }

    /* Terminate burst */
    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_BEXIT);

    data->last_read_burst = true;

    return 0;
}

static int burst_write(const struct device *dev, uint8_t reg, const uint8_t *buf, size_t size) {
    LOG_INF("In burst write");
    int err;
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
        return err;
    }

    /* Write data */
    for (size_t i = 0; i < size; i++) {
        write_buf = buf[i];

        err = spi_write_dt(&config->bus, &tx);
        if (err) {
            LOG_ERR("Burst write failed on SPI write (data)");
            return err;
        }

        k_busy_wait(T_BRSEP);
    }

    /* Terminate burst mode. */
    err = spi_cs_ctrl(dev, false);
    if (err) {
        return err;
    }

    k_busy_wait(T_BEXIT);

    data->last_read_burst = false;

    return 0;
}

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

    /* Convert CPI to register value */
    uint8_t value = (cpi / 100) - 1;

    LOG_INF("Setting CPI to %u (reg value 0x%x)", cpi, value);

    int err = reg_write(dev, PMW3360_REG_CONFIG1, value);
    if (err) {
        LOG_ERR("Failed to change CPI");
    }

    return err;
}

/* unit: ms */
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

    /* Convert time to register value */
    uint8_t value = time / mintime;

    LOG_INF("Set downshift time to %u ms (reg value 0x%x)", time, value);

    int err = reg_write(dev, reg_addr, value);
    if (err) {
        LOG_ERR("Failed to change downshift time");
    }

    return err;
}

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
    err = reg_write(dev, PMW3360_REG_SROM_ENABLE, 0x1D);
    if (err) {
        LOG_ERR("Cannot initialize SROM");
        return err;
    }

    return err;
}

static int pmw3360_async_init_fw_load_continue(const struct device *dev) {
    int err;

    LOG_INF("Uploading optical sensor firmware...");

    /* Write 0x18 to SROM_enable to start SROM download */
    err = reg_write(dev, PMW3360_REG_SROM_ENABLE, 0x18);
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
    err = reg_write(dev, PMW3360_REG_CONFIG2, 0x20);
    if (err) {
        LOG_ERR("Cannot enable REST modes");
    }
    LOG_INF("Finished firmware load verify");
    return err;
}

static void set_interrupt(const struct device *dev, const bool en) {
    LOG_INF("In pwm3360_set_interrupt");
    const struct pixart_config *config = dev->config;
    int ret = gpio_pin_interrupt_configure_dt(&config->irq_gpio,
                                              en ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE);
    if (ret < 0) {
        LOG_ERR("can't set interrupt");
    }
}

static enum pixart_input_mode get_input_mode_for_current_layer(const struct device *dev) {
//    const struct pixart_config *config = dev->config;
//    uint8_t curr_layer = zmk_keymap_highest_layer_active();
//    for (size_t i = 0; i < config->scroll_layers_len; i++) {
//        if (curr_layer == config->scroll_layers[i]) {
//            return SCROLL;
//        }
//    }
//    for (size_t i = 0; i < config->snipe_layers_len; i++) {
//        if (curr_layer == config->snipe_layers[i]) {
//            return SNIPE;
//        }
//    }
    return MOVE;
}

static int set_cpi_if_needed(const struct device *dev, uint32_t cpi) {
    LOG_INF("In pwm3360_set_cpi_if_needed");
    struct pixart_data *data = dev->data;
    if (cpi != data->curr_cpi) {
        return set_cpi(dev, cpi);
    }
    return 0;
}

// Function to rotate coordinates by a specified angle
 static void rotate_coordinates(int16_t x, int16_t y, float angle_degrees, int16_t *x_out, int16_t *y_out) {
     // Ensure the angle is within 0-360 degrees
     //angle_degrees %= 360;
     if (angle_degrees < 0) angle_degrees += 360;

     // Determine the index in the lookup table
     int index = angle_degrees / 5;

     // Get sine and cosine values from the lookup table
     float cos_angle = cos_table[index];
     float sin_angle = sin_table[index];

     // Apply the rotation matrix
     *x_out = (int16_t)(x * cos_angle - y * sin_angle);
     *y_out = (int16_t)(x * sin_angle + y * cos_angle);
}

static int pmw3360_report_data(const struct device *dev) {
    LOG_INF("In pwm3360_report_data");
    struct pixart_data *data = dev->data;
    uint8_t buf[PMW3360_BURST_SIZE];

    if (unlikely(!data->ready)) {
        LOG_WRN("Device is not initialized yet");
        return -EBUSY;
    }

    int32_t dividor;
    enum pixart_input_mode input_mode = get_input_mode_for_current_layer(dev);
    bool input_mode_changed = data->curr_mode != input_mode;
    switch (input_mode) {
    case MOVE:
        set_cpi_if_needed(dev, CONFIG_PMW3360_CPI);
        dividor = CONFIG_PMW3360_CPI_DIVIDOR;
        break;
    case SCROLL:
        set_cpi_if_needed(dev, CONFIG_PMW3360_CPI);
        if (input_mode_changed) {
            data->scroll_delta_x = 0;
            data->scroll_delta_y = 0;
        }
        dividor = 1; // this should be handled with the ticks rather than dividors
        break;
    case SNIPE:
        set_cpi_if_needed(dev, CONFIG_PMW3360_SNIPE_CPI);
        dividor = CONFIG_PMW3360_SNIPE_CPI_DIVIDOR;
        break;
    default:
        return -ENOTSUP;
    }

    data->curr_mode = input_mode;

//#if AUTOMOUSE_LAYER > 0
//    if (input_mode == MOVE &&
//            (automouse_triggered || zmk_keymap_highest_layer_active() != AUTOMOUSE_LAYER)
//    ) {
//        activate_automouse_layer();
//    }
//#endif

    int err = motion_burst_read(dev, buf, sizeof(buf));
    if (err) {
        return err;
    }

    int16_t raw_x = ((int16_t)sys_get_le16(&buf[PMW3360_DX_POS])) / dividor;
    int16_t raw_y = ((int16_t)sys_get_le16(&buf[PMW3360_DY_POS])) / dividor;
    int16_t x, y;

    // Rotate the coordinates
    float angle = CONFIG_PMW3360_ROTATION_ANGLE_DEG;
    rotate_coordinates(raw_x, raw_y, angle, &x, &y);

    if (IS_ENABLED(CONFIG_PMW3360_INVERT_X)) {
        x = -x;
    }

    if (IS_ENABLED(CONFIG_PMW3360_INVERT_Y)) {
        y = -y;
    }

    if (x != 0 || y != 0) {
        if (input_mode != SCROLL) {
            input_report_rel(dev, INPUT_REL_X, x, false, K_FOREVER);
            input_report_rel(dev, INPUT_REL_Y, y, true, K_FOREVER);
        }
    }

    return err;
}

static void pmw3360_gpio_callback(const struct device *gpiob, struct gpio_callback *cb,
                                  uint32_t pins) {
    LOG_INF("In pwm3360_gpio_callback");
    struct pixart_data *data = CONTAINER_OF(cb, struct pixart_data, irq_gpio_cb);
    const struct device *dev = data->dev;

    set_interrupt(dev, false);

    // submit the real handler work
    k_work_submit(&data->trigger_work);
}

static void pmw3360_work_callback(struct k_work *work) {
    LOG_INF("In pwm3360_work_callback");
    struct pixart_data *data = CONTAINER_OF(work, struct pixart_data, trigger_work);
    const struct device *dev = data->dev;

    pmw3360_report_data(dev);
    set_interrupt(dev, true);
}

static int pmw3360_async_init_power_up(const struct device *dev) {
    /* Reset sensor */
    LOG_INF("async_init_power_up");

    return reg_write(dev, PMW3360_REG_POWER_UP_RESET, PMW3360_POWERUP_CMD_RESET);
}

static int pmw3360_async_init_configure(const struct device *dev) {
    LOG_INF("pmw3360_async_init_configure");
    int err;

    err = set_cpi(dev, CONFIG_PMW3360_CPI);

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

static void pmw3360_async_init(struct k_work *work) {
    LOG_INF("pmw3360_async_init");
    struct k_work_delayable *work2 = (struct k_work_delayable *)work;
    struct pixart_data *data = CONTAINER_OF(work2, struct pixart_data, init_work);
    const struct device *dev = data->dev;

    LOG_INF("async init step %d", data->async_init_step);

    data->err = async_init_fn[data->async_init_step](dev);
    if (data->err) {
        LOG_ERR("initialization failed");
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

static int pmw3360_init_irq(const struct device *dev) {
     LOG_INF("Configure irq...");

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
    gpio_init_callback(&data->irq_gpio_cb, pmw3360_gpio_callback, BIT(config->irq_gpio.pin));

    err = gpio_add_callback(config->irq_gpio.port, &data->irq_gpio_cb);
    if (err) {
        LOG_ERR("Cannot add IRQ GPIO callback");
    }

    LOG_INF("Configure irq done");

    return err;
}

static int pmw3360_init(const struct device *dev) {
    LOG_INF("Start initializing...");

    struct pixart_data *data = dev->data;
    const struct pixart_config *config = dev->config;
    int err;

    // init device pointer
    data->dev = dev;

    // init trigger handler work
    k_work_init(&data->trigger_work, pmw3360_work_callback);

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

    // init irq routine
    err = pmw3360_init_irq(dev);
    if (err) {
        return err;
    }

    // Setup delayable and non-blocking init jobs, including following steps:
    // 1. power reset
    // 2. clear motion registers
    // 3. srom firmware download and checking
    // 4. eable rest mode
    // 5. set cpi and downshift time (not sample rate)
    // The sensor is ready to work (i.e., data->ready=true after the above steps are finished)
    k_work_init_delayable(&data->init_work, pmw3360_async_init);

    k_work_schedule(&data->init_work, K_MSEC(async_init_delay[data->async_init_step]));

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
