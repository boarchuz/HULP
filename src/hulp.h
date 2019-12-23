#ifndef HULP_H_
#define HULP_H_

#include "sdkconfig.h"

// #define HULP_DEBUG

#ifdef HULP_NO_PATCH
#include "esp32/ulp.h"
#else
#include "cust_ulp.h"
#endif

#include "math.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

#include "hulp_macros.h"
#include "hulp_types.h"


#ifndef CONFIG_ESP32_ULP_COPROC_RESERVE_MEM
    #define CONFIG_ESP32_ULP_COPROC_RESERVE_MEM CONFIG_ULP_COPROC_RESERVE_MEM
#endif

extern const gpio_num_t rtc_to_gpio[18];

/**
 * Initialise and configure a pin for ULP GPIO.
 *
 * pin: GPIO pin (eg. GPIO_NUM_4)
 * mode: RTC_GPIO_MODE_INPUT_ONLY, RTC_GPIO_MODE_OUTPUT_ONLY, RTC_GPIO_MODE_INPUT_OUTPUT or RTC_GPIO_MODE_DISABLED
 */
void hulp_configure_pin(gpio_num_t pin, rtc_gpio_mode_t mode = RTC_GPIO_MODE_INPUT_OUTPUT, bool pullup = false, bool pulldown = false, uint32_t level = 0);

/**
 * Initialise and configure a pin for ULP analog input
 *
 * pin: GPIO pin (eg. GPIO_NUM_32)
 * attenuation: Channel attenuation, one of ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6 or ADC_ATTEN_DB_11
 * width: Bit capture width, one of ADC_WIDTH_BIT_9, ADC_WIDTH_BIT_10, ADC_WIDTH_BIT_11 or ADC_WIDTH_BIT_12
 */
void hulp_configure_analog_pin(gpio_num_t pin, adc_atten_t attenuation = ADC_ATTEN_DB_11, adc_bits_width_t width = ADC_WIDTH_BIT_12);

/**
 * Prepares GPIOs for use with ULP hardware I2C.
 * Do not use this for bitbanging I2C. See examples for bitbanging configuration.
 *  
 * RTC I2C signals can be mapped onto the following pins only:
 *    SCL: GPIO_NUM_2 or GPIO_NUM_4
 *    SDA: GPIO_NUM_0 or GPIO_NUM_15
 */
void hulp_configure_i2c_pins(gpio_num_t scl_pin, gpio_num_t sda_pin, bool scl_pullup = false, bool sda_pullup = false);

/**
 * Prepares I2C control registers in order to use ULP hardware I2C.
 * Do not use this for bitbanging I2C.
 *
 * I2C mode is set to Master
 * scl_low: Number of FAST_CLK cycles for SCL to be low [19b] 
 * scl_high: Number of FAST_CLK cycles for SCL to be high [20b]
 * sda_duty: Number of FAST_CLK cycles SDA will switch after falling edge of SCL [20b]
 * scl_start: Number of FAST_CLK cycles to wait before generating start condition [20b]
 * scl_stop: Number of FAST_CLK cycles to wait before generating stop condition [20b] 
 * timeout: Maximum number of FAST_CLK cycles that the transmission can take [20b]
 * scl_pushpull: SCL is push-pull (true) or open-drain (false)
 * sda_pushpull: SDA is push-pull (true) or open-drain (false)
 * rx_lsbfirst: Receive LSB first
 * tx_lsbfirst: Send LSB first
 */
void hulp_configure_i2c_controller(uint32_t scl_low = 40, uint32_t scl_high = 40, uint32_t sda_duty = 16, uint32_t scl_start = 30, uint32_t scl_stop = 44, uint32_t timeout = 200, bool scl_pushpull = false, bool sda_pushpull = false, bool rx_lsbfirst = false, bool tx_lsbfirst = false);

/**
 * Set the address of an I2C slave for use with ULP hardware I2C.
 * Do not use this for bitbanging I2C.
 *
 * index: the register index in which to store the address (0-7) (SENS_SAR_SLAVE_ADDRx_REG)
 * address: I2C address of the slave
 */
void hulp_register_i2c_slave(uint8_t index, uint8_t address);

/**
 * Prepare a touch pad for ULP control.
 */
void hulp_configure_touch(gpio_num_t touch_gpio);

/**
 * Force RTC peripherals power domain to remain on in sleep.
 * Necessary to maintain some pin states during sleep, internal PU/PD resistors, ULP wakeup interval switching, etc.
 */
void hulp_peripherals_on();

/**
 * Prepare the hall effect sensor for the ULP. 
 * Sensor uses ADC channels on GPIO_36 (SENS_VP) and GPIO_39 (SENS_VN). Nothing should be externally connected to these pins.
 */
void hulp_configure_hall_effect_sensor();

/**
 * Configure the temperature sensor for the ULP
 */
void hulp_tsens_configure(uint8_t clk_div = 3);

/**
 * Zero-out CONFIG_ESP32_ULP_COPROC_RESERVE_MEM bytes of RTC slow mem
 */ 
void hulp_clear_program_memory();

/**
 * Zero-out lower 4K of RTC slow mem
 */
void hulp_clear_rtc_slow_memory();

/**
 * Convert a time (in milliseconds) to an optimally-shifted 16-bit RTC tick count.
 */
uint16_t hulp_ticks(uint32_t time_ms);

/**
 * Variant of hulp_ticks that allows specifying a reference time. 
 * Useful if you have a ticks variable that may change, to ensure that the calculated ticks are always referenced to the same 16 bit range of RTC ticks.
 * Otherwise a min and max value, for example, may resolve to use different tick ranges, resulting in undefined behaviour.
 * Typically, you'd use the maximum possible time as reference_ms.
 */
uint16_t hulp_ticks_ref(uint32_t time_ms, uint32_t reference_ms);

/**
 * Get the optimal bit shift for a given time (in milliseconds), to be used in ranged reading of RTC ticks.
 */
uint8_t hulp_tick_shift(uint32_t time_ms);

/**
 * Returns the offset of a label (after processing) in an array of ULP macros.
 */
// int hulp_get_label_pc(const ulp_insn_t *program, size_t size_of_program, uint16_t label);
uint16_t hulp_label(uint16_t label, const ulp_insn_t *program);


/**
 * Run the ULP program once. It will not be restarted after a I_HALT() instruction.
 */
void hulp_run_once(uint32_t entry_point = 0);

/**
 * Convenience
 * Process, load, set wakeup interval, run.
 * Provide size_of_program in bytes (ie. sizeof(program))
 */
void hulp_start(const ulp_insn_t *program, size_t size_of_program, uint32_t period_us, uint32_t entry_point = 0);

/**
 * Disables the timer so that the ULP will not wake up again. Equivalent to I_END()
 * If it is currently running, the ULP will continue until the next I_HALT() instruction.
 * Use ulp_run() to restart.
 */
void hulp_end();

/**
 * Internal
 */
adc_unit_t hulp_adc_get_unit(gpio_num_t pin);
/**
 * Internal
 */
adc1_channel_t hulp_adc1_get_channel(gpio_num_t pin);
/**
 * Internal
 */
adc2_channel_t hulp_adc2_get_channel(gpio_num_t pin);
/**
 * Internal
 */
uint8_t hulp_adc_get_channel_num(gpio_num_t pin);

#endif