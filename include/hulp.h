#ifndef HULP_H
#define HULP_H

#include <math.h>

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "hal/gpio_types.h"

#include "hulp_ulp.h"

#include "hulp_macros.h"
#include "hulp_types.h"

#include "sdkconfig.h"

enum ulp_state_t {
    ULP_STATE_IDLE,      //Default state.
    ULP_STATE_WAKING,    //Sleep timer expired. Starting ULP.
    ULP_STATE_RUNNING,   //ULP is running.
    ULP_STATE_HALTED,    //ULP just halted. Starting sleep timer.
    ULP_STATE_SLEEPING,  //ULP is sleeping. Sleep timer is running.
    ULP_STATE_DONE,      //ULP has finished and sleep timer is not running.
    ULP_STATE_UNKNOWN,   //Unable to determine.
};

/**
 * Initialise and configure a pin for ULP GPIO.
 *
 * pin: GPIO pin (eg. GPIO_NUM_4)
 * mode: RTC_GPIO_MODE_INPUT_ONLY, RTC_GPIO_MODE_OUTPUT_ONLY, RTC_GPIO_MODE_INPUT_OUTPUT or RTC_GPIO_MODE_DISABLED
 * pull_mode: GPIO_PULLUP_ONLY, GPIO_PULLDOWN_ONLY, GPIO_PULLUP_PULLDOWN, GPIO_FLOATING
 */
esp_err_t hulp_configure_pin(gpio_num_t pin, rtc_gpio_mode_t mode = RTC_GPIO_MODE_INPUT_OUTPUT, gpio_pull_mode_t pull_mode = GPIO_FLOATING, uint32_t level = 0);

/**
 * Initialise and configure a pin for ULP analog input
 *
 * pin: GPIO pin (eg. GPIO_NUM_32)
 * attenuation: Channel attenuation, one of ADC_ATTEN_DB_0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6 or ADC_ATTEN_DB_11
 * width: Bit capture width, one of ADC_WIDTH_BIT_9, ADC_WIDTH_BIT_10, ADC_WIDTH_BIT_11 or ADC_WIDTH_BIT_12
 */
esp_err_t hulp_configure_analog_pin(gpio_num_t pin, adc_atten_t attenuation = ADC_ATTEN_DB_11, adc_bits_width_t width = ADC_WIDTH_BIT_12);

/**
 * Prepares GPIOs for use with ULP hardware I2C.
 * Do not use this for bitbanging I2C. See examples for bitbanging configuration.
 *  
 * RTC I2C signals can be mapped onto the following pins only:
 *    SCL: GPIO_NUM_2 or GPIO_NUM_4
 *    SDA: GPIO_NUM_0 or GPIO_NUM_15
 */
esp_err_t hulp_configure_i2c_pins(gpio_num_t scl_pin, gpio_num_t sda_pin, bool scl_pullup = false, bool sda_pullup = false);

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
esp_err_t hulp_register_i2c_slave(uint8_t index, uint8_t address);

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
 * Zero-out ULP_RESERVE_MEM bytes of RTC slow mem
 */ 
void hulp_clear_program_memory();

/**
 * Zero-out lower 4K of RTC slow mem
 */
void hulp_clear_rtc_slow_memory();

/**
 * Convert a time (in milliseconds) to an optimally-shifted 16-bit RTC tick count.
 */
uint16_t hulp_ms_to_ulp_ticks(uint32_t time_ms);

/**
 * Variant of hulp_ms_to_ulp_ticks that allows specifying a reference shift. 
 *  Useful if you have a changing time period or multiple time periods that must be referenced to the same range of ticks.
 *  Otherwise, for example, a shorter time period may resolve to a larger 16-bit tick count using a *different* bit range.
 *  Typically, you'd use the maximum possible time to calulcate the shift using hulp_ms_to_ulp_tick_shift()
 */
uint16_t hulp_ms_to_ulp_ticks_with_shift(uint32_t time_ms, uint8_t shift);

/**
 * Get the current 16-bit shifted value of the RTC ticks register.
 * Use hulp_ms_to_ulp_tick_shift() to get the shift associated with a given time. 
 */
uint16_t hulp_get_current_ulp_ticks(uint8_t shift);

/**
 * Get the optimal bit shift for a given time (in milliseconds), to be used in ranged 16-bit reading of RTC ticks.
 */
uint8_t hulp_ms_to_ulp_tick_shift(uint32_t time_ms);

/**
 * Returns the offset of a label (after processing) in an array of ULP macros.
 */
uint16_t hulp_get_label_pc(uint16_t label, const ulp_insn_t *program);

/**
 * Start the ULP coprocessor. Upon a I_HALT() instruction, the ULP will power down for the interval period before restarting.
 * This simply wraps ulp_run; it is provided for consistency with hulp_ulp_run_once.
 */
esp_err_t hulp_ulp_run(uint32_t entry_point = 0);

/**
 * Run the ULP program once. It will not be restarted after a I_HALT() instruction.
 */
esp_err_t hulp_ulp_run_once(uint32_t entry_point = 0);

/**
 * Process program macros and load it into RTC memory, and set the wakeup interval.
 * This is typically followed by hulp_ulp_run or hulp_ulp_run_once to start the ULP coprocessor.
 * For simplicity, expects program_size in bytes (not words).
 */
esp_err_t hulp_ulp_load(const ulp_insn_t *program, size_t program_size, uint32_t period_us, uint32_t entry_point = 0);

/**
 * Disables the timer so that the ULP will not wake up again. Equivalent to I_END()
 * If it is currently running, the ULP will continue until the next I_HALT() instruction.
 * Use hulp_ulp_run/hulp_ulp_run_once to restart.
 */
void hulp_ulp_end();

/**
 * True if most recent reset was a wake from deep sleep (to distinguish from power-on reset, for example), false if any other cause.
 */
bool hulp_is_deep_sleep_wakeup();

/**
 * True if deep sleep wakeup was triggered by the ULP, false if any other cause.
 */
bool hulp_is_ulp_wakeup();

/**
 * Get the current ULP state.
 * Note: when using hulp_ulp_run_once, only IDLE or DONE may be returned
 */
ulp_state_t hulp_get_state();

/**
 * Register an ISR for ULP interrupts.
 * ULP interrupts will not be enabled until hulp_ulp_interrupt_en() is called.
 */
esp_err_t hulp_ulp_isr_register(intr_handler_t handler, void* handler_arg = nullptr);

/**
 * Deregister a ULP ISR. 
 * Simply calls rtc_isr_deregister. Included for consistency with hulp_ulp_isr_register.
 */
esp_err_t hulp_ulp_isr_deregister(intr_handler_t handler, void* handler_arg = nullptr);

/**
 * Enable ULP interrupts.
 * The ULP can trigger interrupts with the I_WAKE() instruction.
 */
void hulp_ulp_interrupt_en();

/**
 * Disable ULP interrupts.
 */
void hulp_ulp_interrupt_dis();

/**
 * Configure RTC pin interrupt.
 * The ESP32 ULP has no ISR functionality, however it can read a register to determine if this interrupt has been triggered.
 * Assuming latency is acceptable, this allows flexibility to sleep or do other work without missing activity on a pin that 
 * would otherwise need to be polled, as well as enabling the handling of pulses that may be too short to practically detect 
 * using the ULP.
 * intr_type: GPIO_INTR_DISABLE, GPIO_INTR_ANYEDGE, GPIO_INTR_LOW_LEVEL, or GPIO_INTR_HIGH_LEVEL
 * RTC peripherals domain may need to be forced on for this to function correctly: hulp_peripherals_on()
 * See: I_GPIO_INT_RD, I_GPIO_INT_CLR, I_GPIO_INT_SET_TYPE
 */
esp_err_t hulp_configure_pin_int(gpio_num_t gpio_num, gpio_int_type_t intr_type);

/**
 * @brief Get the frequency of RTC Fast Clock
 * @param slow_clk_cycles Number of RTC Slow Clock cycles for calibration
 */
uint32_t hulp_get_fast_clk_freq(uint32_t slow_clk_cycles = 100);

/**
 * Internal. Do not use directly.
 */
#define hulp_gtr(gpio_num) ((uint8_t)rtc_io_number_get(gpio_num))

/**
 * Internal. Do not use directly.
 */
int hulp_adc_get_periph_index(gpio_num_t pin);

/**
 * Internal. Do not use directly.
 */
int hulp_adc_get_channel_num(gpio_num_t pin);

#endif