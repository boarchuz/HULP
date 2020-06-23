# HULP

This is a collection of macros and functions to help get the most out of the Ultra Low Power Co-Processor (ULP) on the ESP32 with legacy macro programming.

## Features

* Methods to easily share data between the ULP and SoC.
* Powerful ULP debugging functionality.
* Macros for easy GPIO interaction, including reading input level, setting output level, setting pullup/pulldown, toggling, etc.
* Wrappers for many timing functions, handling all conversions between milliseconds and ticks, allowing easy and intuitive non-blocking multi-tasking.
* Single APA RGB LED driver for status indication.
* Efficient I2C bitbanging driver with error callbacks, capable of 16-bit reads, supporting multiple devices. All in just 76 instructions.
* Macros to easily control touch pads, hall effect sensor, temperature sensor, etc.
* UART driver, with TX and RX.

## Debugging

* Breakpoint-style debugging aided by the SoC.
* One simple macro to add a breakpoint anywhere in the ULP program array.
* Use an optional callback to receive information about the breakpoint: register values (excluding one scratch), program counter, even the line number in your file.
* Alter register values in the breakpoint, disable or enable breakpoints, change the current program counter, etc. Continue the ULP when ready.
* Only a few lines needed to set up a minimal debugging configuration that will output key information at each breakpoint before automatically continuing.


## Deep Sleep Debugging

* A printf-style function allows the ULP to format the value of R0 to its 5-digit ASCII representation.
* Combined with HULP's UART TX functionality, the ULP can output vital debugging information completely independent of the SoC, even in deep sleep.

### Preview

Share data between ULP and SoC:

```
RTC_DATA_ATTR ulp_var_t ulp_myvariable;

//ULP
I_MOVI(R2, 0),
I_GET(R0, R2, ulp_myvariable),
I_ADDI(R0, R0, 1),
I_PUT(R0, R2, ulp_myvariable),

//SoC
if (ulp_myvariable.updated()) {
  uint16_t val = ulp_myvariable.get();
}
ulp_myvariable.put(1234);
```

Macros for many common IO operations:

```
I_ANALOG_READ(R0, GPIO_NUM_36),
I_GPIO_PULLUP(GPIO_NUM_32, 1),
I_GPIO_READ(GPIO_NUM_32),
I_GPIO_OUTPUT_EN(GPIO_NUM_25),
I_GPIO_SET(GPIO_NUM_25, 1),
I_GPIO_SET(GPIO_NUM_25, 0),

```

Non-blocking multi-tasking:

```
#define LED1_BLINK_MS 1000
#define LED2_BLINK_MS 2000

M_IF_MS_ELAPSED(/*Label:*/ 1, /*Time (ms):*/ LED1_BLINK_MS, /*else goto label:*/ 2),
  M_GPIO_TOGGLE(GPIO_NUM_25),
M_IF_MS_ELAPSED(/*Label:*/ 2, /*Time (ms):*/ LED2_BLINK_MS, /*else goto label:*/ 3),
  M_GPIO_TOGGLE(GPIO_NUM_26),
M_LABEL(3),
  I_HALT(),
```

Helper functions:
```
hulp_configure_pin(GPIO_NUM_2, RTC_GPIO_MODE_INPUT_ONLY);
hulp_configure_i2c_pins(GPIO_NUM_4, GPIO_NUM_15);
hulp_configure_i2c_controller();
hulp_configure_analog_pin(GPIO_NUM_36);
hulp_configure_touch(GPIO_NUM_32);
```


### Examples

Included are examples for hardware I2C, bitbanging I2C, APA driver, UART communication, multi-tasking, touch, hall effect, etc.