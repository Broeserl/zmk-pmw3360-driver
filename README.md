# PMW3360 driver implementation for ZMK with at least Zephyr 3.5

THIS driver is ganked from [KOHSUK/zmk-pmw3360-driver](https://github.com/Broeserl/zmk-pmw3360-driver) which is inspired of
[inorichi's PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver) which is based off of [ufan's implementation](https://github.com/inorichi/zmk/tree/support-trackpad) of the driver.
Additionally inspiration from [drorchen/zmk](https://github.com/drorchen/zmk/blob/trackball-support/app/boards/arm/nice_nano/trackball/trackball_new.c)

### Important to note that the PMW3360 is very power hungry compared to the PMW3610. If going fully wireless, you'll need big batteries and, even then, your battery life won't be great.

## Installation

No GitHub actions builds are covered here. Local builds were used to get this working.

Update your `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
&pinctrl {
    spi0_default: spi0_default {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 22)>,
                <NRF_PSEL(SPIM_MOSI, 0, 20)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
        };
    };

    spi0_sleep: spi0_sleep {
        group1 {
            psels = <NRF_PSEL(SPIM_SCK, 0, 22)>,
                <NRF_PSEL(SPIM_MOSI, 0, 20)>,
                <NRF_PSEL(SPIM_MISO, 0, 17)>;
            low-power-enable;
        };
    };
};

&spi0 {
    status = "okay";
    compatible = "nordic,nrf-spim";
    pinctrl-0 = <&spi0_default>;
    pinctrl-1 = <&spi0_sleep>;
    pinctrl-names = "default", "sleep";
    cs-gpios = <&gpio0 24 GPIO_ACTIVE_LOW>;

    trackball: trackball@0 {
        status = "okay";
        compatible = "pixart,pmw3360";
        reg = <0>;
        spi-max-frequency = <2000000>;
        irq-gpios = <&gpio0 8 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
        /* optional features */
        scroll-layers = <3>;
        snipe-layers = <4>;
    };
};

/* Important/Required block */
&trackball_listener {
    status = "okay";
    device = <&trackball>;
    /* Automouse layer provided by the [Temporary Layer Input Processor](https://zmk.dev/docs/keymaps/input-processors/temp-layer#pre-defined-instances) in my case layer 2 with timeout 1000ms */
    input-processors = <&zip_temp_layer 2 1000>;

    /* Scrolling layer provided by the [Code Mapper Input Processor](https://zmk.dev/docs/keymaps/input-processors/code-mapper#pre-defined-instances) in my case layer 3 */
    scroller {
        layers = <3>;
        input-processors = <&zip_xy_to_scroll_mapper>;
        process-next;
    };
};
```

Now enable the driver config in your `board.config` file (read the Kconfig file to find out all possible options):

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_MOUSE=y
CONFIG_PMW3360=y
```

## Features

- **Choice between interrupt only input and interrupt triggered timer polling**

  In the BLE communication use case, I saw the cursor stuttering, I found that it is because events are lost (too many of them appear if interrupt only), from my POV there are 2 solutions:
  - Adding a wait time inside the interrupt to lower the amount of events (default)
  - Using the interrupt as signal to start a timer where polling is active with a certain interval

- **Different CPI values for moving, scrolling, sniping**

  Configure separate CPI values for each input mode. The driver automatically switches between these values based on the active layer.

- **Runtime CPI adjustment behaviors**

  Adjust CPI on-the-fly using keyboard behaviors. Changes are mode-aware and apply to the currently active input mode (MOVE, SCROLL, or SNIPE).

- **Possibility of rotating sensor input in 5 degree steps**

  Compensate for physical sensor mounting angle using the `CONFIG_PMW3360_ROTATION_ANGLE_DEG` option.

More detailed information on configuration can be found in the [Kconfig file](./src/Kconfig).

## CPI Adjustment Behaviors

The driver provides three behaviors for runtime CPI adjustment:

### Available Behaviors

- **`&pmw_cpi_inc`** - Increase CPI of the currently active mode
- **`&pmw_cpi_dec`** - Decrease CPI of the currently active mode
- **`&pmw_cpi_reset`** - Reset CPI of the currently active mode to its Kconfig default

### How It Works

The behaviors are **mode-aware**, meaning they adjust the CPI for whichever mode you're currently in:

- When in **MOVE mode** (normal cursor movement): Adjusts `move_cpi` by `CONFIG_PMW3360_CPI_STEP`
- When in **SCROLL mode** (on scroll layers): Adjusts `scroll_cpi` by `CONFIG_PMW3360_SCROLL_CPI_STEP`
- When in **SNIPE mode** (on snipe layers): Adjusts `snipe_cpi` by `CONFIG_PMW3360_SNIPE_CPI_STEP`

Changes take effect immediately if you're currently in that mode, or will apply the next time you enter that mode.

### Configuration Example

Add to your Kconfig configuration file:

```conf
# Base CPI values for each mode
CONFIG_PMW3360_CPI=1600
CONFIG_PMW3360_SCROLL_CPI=3000
CONFIG_PMW3360_SNIPE_CPI=2000

# Step sizes for CPI adjustment behaviors
CONFIG_PMW3360_CPI_STEP=200
CONFIG_PMW3360_SCROLL_CPI_STEP=200
CONFIG_PMW3360_SNIPE_CPI_STEP=100

# CPI divisors (for fine-tuning effective CPI)
CONFIG_PMW3360_CPI_DIVISOR=1
CONFIG_PMW3360_SCROLL_CPI_DIVISOR=4
CONFIG_PMW3360_SNIPE_CPI_DIVISOR=10
```

### Keymap Example

Add behaviors to your keymap:

```dts
/ {
    keymap {
        compatible = "zmk,keymap";
        
        default_layer {
            bindings = <
                // ... other keys ...
                &pmw_cpi_inc    // Increase CPI
                &pmw_cpi_dec    // Decrease CPI
                &pmw_cpi_reset  // Reset to default
                // ... other keys ...
            >;
        };
    };
};
```

### Usage Tips

1. **Per-Mode Adjustment**: The behaviors adjust CPI for the current mode only. If you're in SCROLL mode and press `&pmw_cpi_inc`, only your scroll sensitivity will change—your normal movement CPI remains unchanged.

2. **Persistent Changes**: CPI adjustments persist until reset or power cycle. They're stored in RAM and reset to Kconfig defaults on device restart.

3. **Valid Ranges**:
- MOVE/SCROLL modes: 100–12000 CPI
- SNIPE mode: 200–3200 CPI
- All values are automatically rounded to the nearest 100 CPI (hardware requirement)

4. **Finding Your Perfect CPI**: Start with the Kconfig defaults, then use the behaviors to fine-tune sensitivity while using your keyboard. Once you find values you like, update your Kconfig to make them permanent.

## Configuration Options

### Operating Modes

Choose between two interrupt handling modes:

- **`CONFIG_PMW3360_INTERRUPT_DIRECT`** (default) - Direct interrupt handling with configurable delay
  - `CONFIG_PMW3360_INTERRUPT_DIRECT_DELAY_MS` - Delay before processing interrupt (default: 15ms)

- **`CONFIG_PMW3360_INTERRUPT_POLLING`** - Timer-based polling mode
  - `CONFIG_PMW3360_POLLING_INTERVAL_MS` - Polling interval (default: 15ms)
  - `CONFIG_PMW3360_MAX_POLL_COUNT` - Maximum polling events per motion interrupt (default: 20)

### CPI Configuration

Configure CPI (counts per inch) for each input mode:

```conf
# MOVE mode (normal cursor movement)
CONFIG_PMW3360_CPI=1600                    # Base CPI value
CONFIG_PMW3360_CPI_DIVISOR=1               # CPI divisor for fine-tuning
CONFIG_PMW3360_CPI_STEP=200                # Inc/dec step size

# SCROLL mode  
CONFIG_PMW3360_SCROLL_CPI=800              # Base CPI value
CONFIG_PMW3360_SCROLL_CPI_DIVISOR=1        # CPI divisor
CONFIG_PMW3360_SCROLL_CPI_STEP=100         # Inc/dec step size

# SNIPE mode (precision aiming)
CONFIG_PMW3360_SNIPE_CPI=2000              # Base CPI value
CONFIG_PMW3360_SNIPE_CPI_DIVISOR=10        # CPI divisor
CONFIG_PMW3360_SNIPE_CPI_STEP=100          # Inc/dec step size
```

**Note**: Effective CPI = Base CPI / Divisor. For most use cases, keep divisor at 1 and adjust the base CPI value directly.

### Power Management

Configure sensor power saving behavior:

```conf
CONFIG_PMW3360_RUN_DOWNSHIFT_TIME_MS=500        # Time before entering REST1 mode
CONFIG_PMW3360_REST1_DOWNSHIFT_TIME_MS=9220     # Time before entering REST2 mode
CONFIG_PMW3360_REST2_DOWNSHIFT_TIME_MS=150000   # Time before entering REST3 mode
```

### Sensor Orientation

```conf
CONFIG_PMW3360_INVERT_X=n                  # Invert X axis
CONFIG_PMW3360_INVERT_Y=n                  # Invert Y axis
CONFIG_PMW3360_ROTATION_ANGLE_DEG=0        # Rotate sensor input (0-355, in 5° steps)
```

## Troubleshooting

### Cursor stuttering or laggy movement

- Try **polling mode** instead of direct interrupt mode
- Adjust `CONFIG_PMW3360_INTERRUPT_DIRECT_DELAY_MS` if using direct mode
- Reduce `CONFIG_PMW3360_POLLING_INTERVAL_MS` if using polling mode

### CPI feels wrong or inconsistent

- Check your CPI divisor values—for most uses, keep them at 1
- Use the `&pmw_cpi_inc`/`&pmw_cpi_dec` behaviors to test different values in real-time
- Remember: Effective CPI = Base CPI / Divisor

### Sensor not responding

- Verify SPI pins are correct in your overlay
- Check that `irq-gpios` is properly configured
- Ensure `CONFIG_PMW3360=y` is set in your config

## Credits

This driver builds upon work from:
- [KOHSUK/zmk-pmw3360-driver](https://github.com/Broeserl/zmk-pmw3360-driver)
- [inorichi's PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver)
- [ufan's implementation](https://github.com/inorichi/zmk/tree/support-trackpad)
- [drorchen/zmk](https://github.com/drorchen/zmk/blob/trackball-support/app/boards/arm/nice_nano/trackball/trackball_new.c)