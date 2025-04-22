# PMW3360 driver implementation for ZMK with at least Zephyr 3.5

THIS driver is ganked from [KOHSUK/zmk-pmw3360-driver](https://github.com/Broeserl/zmk-pmw3360-driver) which is inspired of
[inorichi's PMW3610 driver](https://github.com/inorichi/zmk-pmw3610-driver) which is based off of [ufan's implementation](https://github.com/inorichi/zmk/tree/support-trackpad) of the driver.
Additionally inspiration from [drorchen/zmk](https://github.com/drorchen/zmk/blob/trackball-support/app/boards/arm/nice_nano/trackball/trackball_new.c)

### Important to note that the PMW3360 is very power hungry compared to the PMW3610. If going fully wireless, you'll need big batteries and, even then, your battery life won't be great.

## Installation

No GitHub actions builds are covered here. Local builds where used to get this working.

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
};```

Now enable the driver config in your `board.config` file (read the Kconfig file to find out all possible options):

```conf
CONFIG_SPI=y
CONFIG_INPUT=y
CONFIG_ZMK_MOUSE=y
CONFIG_PMW3360=y
```

## Features
- Choice between interrupt only input and interrupt triggered timer polling
  In the BLE communication use case, I saw the cursor stuttering, I found that it is because events are lost (too many of them appear if interrupt only), from my POV there are 2 solutions:
    - Adding a wait time inside the interrupt to lower the amount of events (default)
    - Using the interrupt as signal to start a timer where polling is active with a certain interval
- Different CPI values for moving, scrolling, sniping
- Possibility of rotating sensor input in 5 degree steps

More detailed information to the configuration can be found [here](https://github.com/Broeserl/zmk-pmw3360-driver/blob/main/src/Kconfig)
