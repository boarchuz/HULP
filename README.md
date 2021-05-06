# HULP

HULP is a helper library for the ESP32's Ultra Low Power Co-Processor (ULP).

***

## Features

### Memory Access

Easily access variables to store and retrieve data, or share between the ULP and SoC
```
RTC_SLOW_ATTR ulp_var_t my_ulp_variable;

// ULP:
I_MOVI(R2, 0),
I_GET(R1, R2, my_ulp_variable), // Load my_ulp_variable into R1
I_PUT(R3, R2, my_ulp_variable), // Store R3 into my_ulp_variable

// SoC:
uint16_t temp = my_ulp_variable.val;
my_ulp_variable.val = 123;
```
### GPIO

Functions to configure GPIOs in preparation for the ULP
```
hulp_configure_pin(GPIO_NUM_25, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0);
hulp_configure_pin(GPIO_NUM_26, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1);
hulp_configure_analog_pin(GPIO_NUM_33, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12);
```
Countless macros to simplify ULP GPIO operations
```
I_GPIO_READ(GPIO_NUM_27),       // Digital read
I_GPIO_SET(GPIO_NUM_25, 1),     // Set digital output level high
I_ANALOG_READ(R0, GPIO_NUM_33), // Analog read
I_GPIO_PULLUP(GPIO_NUM_26, 1),  // Enable internal pullup
```
As well as advanced functionality, including interrupts
```
// Configure edge interrupt:
hulp_configure_pin_int(GPIO_NUM_32, GPIO_INTR_ANYEDGE);
// ULP:
I_GPIO_INT_RD(GPIO_NUM_32),   // Check if interrupt has triggered
I_GPIO_INT_CLR(GPIO_NUM_32),  // Clear interrupt
```
### Timing

Macros for basic delays
```
M_DELAY_US_10_100(72),    // Delay 72uS
M_DELAY_MS_20_1000(500),  // Delay 500mS
```
HULP also introduces a concept that makes use of the RTC slow clock for complex, asynchronous timing operations, useful for very power efficient operation and multitasking without the need for blocking delays. See `Timing` example.

### Peripherals

Helpers and drivers for internal peripherals as well as communication protocols, including:

* Capacitive Touch

* Hall Effect

* Temperature

* Hardware I2C

* Software (bitbanged) I2C

* APA RGB LED

* UART

* HX711


### Debugging

* Insert breakpoints, track and manipulate ULP program flow, inspect or alter register contents via the SoC

* Combine the UART driver with included ULP PRINTF subroutines to communicate debugging information independent of the SoC (even in deep sleep!)

### And much more...

Check out the examples for some programs demonstrating the possibilities of the ULP with HULP.

There's a lot more to HULP than what is mentioned above - in lieu of better documentation, please see relevant header files for more features and information.

***

## Compatibility

HULP uses the C macro (legacy) programming method (https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/ulp_macros.html), however you are free to copy and convert any parts for use with the ULP binary toolchain.


ESP-IDF >=4.2.0 is required, and there is partial support for Arduino-ESP32 >=2.0.0.


Only ESP32 is currently supported.