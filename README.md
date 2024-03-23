# Analog Input Driver for ZMK v3.5+

This driver groups ADC io channels into single input event for input subsystem. It provides config for thumbstick input with mid-point alignment, min-mav limitation, deadzone, sampling rate, reporting rate, multiplier, divisor, invertor, etc.

## Installation

Only GitHub actions builds are covered here. Local builds are different for each user, therefore it's not possible to cover all cases.

Include this project on your ZMK's west manifest in `config/west.yml`:

```yml
manifest:
  ...
  projects:
    ...
    - name: zmk-analog-input-driver
      remote: badjeff
      revision: main
    ...
```

Now, update your `board.overlay` adding the necessary bits (update the pins for your board accordingly):

```dts
#define INPUT_EV_KEY 0x01               /**< Key event */
#define INPUT_EV_REL 0x02               /**< Relative coordinate event */
#define INPUT_EV_ABS 0x03               /**< Absolute coordinate event */
#define INPUT_EV_MSC 0x04               /**< Miscellaneous event */
#define INPUT_EV_VENDOR_START 0xf0      /**< Vendor specific event start */
#define INPUT_EV_VENDOR_STOP 0xff       /**< Vendor specific event stop */

#define INPUT_REL_X 0x00                /**< Relative X coordinate */
#define INPUT_REL_Y 0x01                /**< Relative Y coordinate */
#define INPUT_REL_WHEEL 0x08            /**< Relative wheel coordinate */
#define INPUT_REL_HWHEEL 0x06           /**< Relative horizontal wheel coordinate */
#define INPUT_REL_MISC 0x09             /**< Relative misc coordinate */

&adc {
	status = "okay";
};

/ {
	anin0: analog_input_0 {
		compatible = "zmk,analog-input";
		sampling-hz = <100>;
		x-ch {
			io-channels = <&adc 2>;
			mv-mid = <1630>;
			mv-min-max = <1600>;
			mv-deadzone = <10>;
			scale-multiplier = <1>;
			scale-divisor = <70>;
			invert;
			evt-type = <INPUT_EV_REL>;
			input-code = <INPUT_REL_X>;
		};
		y-ch {
			io-channels = <&adc 3>;
			mv-mid = <1630>;
			mv-min-max = <1600>;
			mv-deadzone = <10>;
			scale-multiplier = <1>;
			scale-divisor = <70>;
			invert;
			evt-type = <INPUT_EV_REL>;
			input-code = <INPUT_REL_Y>;
		};
	};
};
```

Now enable the driver config in your `<shield>.config` file (read the Kconfig file to find out all possible options):

```conf
# Enable Analog Input
CONFIG_ADC=y
CONFIG_ADC_ASYNC=y
# ** NOTES** CONFIG_ADC_ASYNC=y is required,
#            if zmk,battery-voltage-divider is chosen for zmk,battery.

# Enable Analog Input Module
CONFIG_ANALOG_INPUT=y
# CONFIG_ANALOG_INPUT_LOG_LEVEL_DBG=y
# CONFIG_ANALOG_INPUT_REPORT_INTERVAL_MIN=22
```
