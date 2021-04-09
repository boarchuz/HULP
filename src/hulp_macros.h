#ifndef HULP_MACROS_H
#define HULP_MACROS_H

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/rtc_io_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "soc/rtc_i2c_reg.h"
#include "soc/soc_memory_layout.h"
#include "esp_assert.h"

#include "hulp_compat.h"

#define hulp_log2(x) (31 - __builtin_clz(x))

#define RTC_WORD_OFFSET(x) ({ \
            uint32_t* ptr_ = (uint32_t*)(&(x)); \
            TRY_STATIC_ASSERT((uint32_t)(ptr_) % sizeof(uint32_t) == 0, (Not aligned)); \
            TRY_STATIC_ASSERT(esp_ptr_in_rtc_slow(ptr_), (Not in RTC Slow Mem)); \
            ((uint16_t)(ptr_ - RTC_SLOW_MEM)); \
        })

#define RTCIO_HAS_DEFAULT_PULLUP(x) ((x == GPIO_NUM_0) || (x == GPIO_NUM_14) || (x == GPIO_NUM_15))
#define RTCIO_HAS_DEFAULT_PULLDOWN(x) ((x == GPIO_NUM_2) || (x == GPIO_NUM_4) || (x == GPIO_NUM_12) || (x == GPIO_NUM_13))

/**
 * Move the offset (in words) of a ulp_var_t or similar into reg_dest.
 *  You may then use it with regular I_LD and I_ST instructions, for example.
 *  It is strongly suggested to use I_GET and I_PUT methods provided by HULP instead.
 * 
 *  reg_dest: General purpose register to hold offset value (R0-R3).
 *  var: A variable of type ulp_var_t or similar.
 * 
 *  eg.
 *      I_MOVO(R3, ulp_my_variable),    // R3 = word offset of ulp_my_variable
 *      I_LD(R0, R3, 0),                // Load the value at the address in R3 into R0
 *      I_ADDI(R0, R0, 1),
 *      I_ST(R0, R3, 0),                // Store the value of R0 into the address in R3
 */
#define I_MOVO(reg_dest, var) \
    I_MOVI(reg_dest, RTC_WORD_OFFSET(var))

/**
 * Load the value of a ulp_var_t into a general purpose register.
 * 
 *  reg_dest: General purpose register to load value into (R0-R3).
 *  reg_zero: General purpose register containing 0. May be the same as reg_dest.
 *  var: A variable of type ulp_var_t or similar.
 * 
 *  eg.
 *      I_MOVI(R3, 0),                  // A register must have known value 0 to use I_GET/I_PUT.
 *      I_GET(R0, R3, ulp_my_variable), // Get ulp_my_variable's value into R0.
 *      I_ADDI(R0, R0, 1),              // R0 = R0 + 1
 *      I_PUT(R0, R3, ulp_my_variable), // Put the value of R0 into ulp_my_variable.
 */
#define I_GET(reg_dest, reg_zero, var) \
    I_GETO(reg_dest, reg_zero, 0, var)

/**
 * A more flexible variant of I_GET that allows specifying the value (val_offset) of the offset register (reg_offset).
 * 
 *  reg_dest: General purpose register to load value into (R0-R3).
 *  reg_offset: General purpose register containing a known value. May be the same as reg_src.
 *  val_offset: The known value of reg_offset.
 *  var: A 4-byte, word-aligned variable in RTC_SLOW_MEM. Use HULP's ulp_var_t family for convenience.
 * 
 */
#define I_GETO(reg_dest, reg_offset, val_offset, var) \
    I_LD(reg_dest, reg_offset, (uint16_t)(RTC_WORD_OFFSET(var) - (val_offset)))

/**
 * Store the value of a general purpose register into ulp_var_t.
 * 
 *  reg_src: General purpose register containing the value to be stored (R0-R3)
 *  reg_zero: General purpose register containing 0. May be the same as reg_src (if storing 0).
 *  var: a ulp_var_t or similar.
 * 
 *  eg.
 *      I_MOVI(R3, 0),                          // A register must have known value 0 to use I_GET/I_PUT.
 *      I_MOVI(R0, 123),
 *      I_PUT(R0, R3, ulp_my_variable),         // Put R0's value in ulp_my_variable.
 *      I_MOVI(R1, 1),
 *      I_PUT(R1, R3, ulp_another_variable),    // Put R1's value in ulp_another_variable.
 */
#define I_PUT(reg_src, reg_zero, var) \
    I_PUTO(reg_src, reg_zero, 0, var)

/**
 * A more flexible variant of I_PUT that allows specifying the value (val_offset) of the offset register (reg_offset).
 * 
 *  reg_src: General purpose register containing the value to be stored (R0-R3)
 *  reg_offset: General purpose register containing a known value. May be the same as reg_src.
 *  val_offset: The known value of reg_offset.
 *  var: A 4-byte, word-aligned variable in RTC_SLOW_MEM. Use HULP's ulp_var_t family for convenience.
 * 
 */
#define I_PUTO(reg_src, reg_offset, val_offset, var) \
    I_ST(reg_src, reg_offset, (uint16_t)(RTC_WORD_OFFSET(var) - (val_offset)))

/**
 * Request and block until the RTC ticks register is updated.
 */
#define M_UPDATE_TICKS() \
    I_FLAG_UPDATE_TICKS(), \
    I_GET_TICKS_VALID(), \
    I_BL(-1,1)

/**
 * Convenience macro to prepare a register with return point and then branch (eg. to subroutine)
 */
#define M_RETURN(label_return_point, reg, label_goto) \
    M_MOVL(reg, label_return_point), \
    M_BX(label_goto), \
    M_LABEL(label_return_point)

/**
 * Init GPIO as RTCIO
 */
#define M_GPIO_INIT(gpio_num) \
     M_RTCIO_INIT(hulp_gtr(gpio_num))

/**
 * Init RTCIO
 */
#define M_RTCIO_INIT(rtcio_num) \
    I_WR_REG_BIT(rtc_io_desc[(rtcio_num)].reg, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].mux), 1), \
    I_WR_REG(rtc_io_desc[(rtcio_num)].reg, rtc_io_desc[(rtcio_num)].func, rtc_io_desc[(rtcio_num)].func + 1, 0)

/**
 * Read GPIO analog value into reg_dest
 * This converts the GPIO_NUM into an ADC_UNIT and ADC_CHANNEL for ULP instruction I_ADC
 */
#define I_ANALOG_READ(reg_dest, pin) \
    I_ADC(reg_dest, (uint32_t)hulp_adc_get_periph_index(pin), (uint32_t)(hulp_adc_get_channel_num(pin)))

/**
 * Read GPIO input value into R0 lsb
 * ie. R0 == 0 (low) or R0 == 1 (high)
 */
#define I_GPIO_READ(gpio_num) \
    I_RTCIO_READ(hulp_gtr(gpio_num))

/**
 * Read RTCIO input value into R0 lsb
 * ie. R0 == 0 (low) or R0 == 1 (high)
 */
#define I_RTCIO_READ(rtcio_num) \
    I_RTCIOS_READ((rtcio_num), 1)

/**
 * Set GPIO output level.
 */
#define I_GPIO_SET(gpio_num, level) \
    I_RTCIO_SET(hulp_gtr(gpio_num), level)

/**
 * Set RTCIO output level.
 */
#define I_RTCIO_SET(rtcio_num, level) \
    I_WR_REG_BIT( ( (level) ? RTC_GPIO_OUT_W1TS_REG : RTC_GPIO_OUT_W1TC_REG ), (uint8_t)( (level) ? (RTC_GPIO_OUT_DATA_W1TS_S + (rtcio_num)) : (RTC_GPIO_OUT_DATA_W1TC_S + (rtcio_num)) ), 1)
    // I_WR_REG_BIT( RTC_GPIO_OUT_REG, (uint8_t)( RTC_GPIO_OUT_DATA_S + (rtcio_num)), (level))

/**
 * Enable GPIO output.
 */
#define I_GPIO_OUTPUT_EN(gpio_num) \
    I_RTCIO_OUTPUT_EN(hulp_gtr(gpio_num))

/**
 * Enable RTCIO output.
 */
#define I_RTCIO_OUTPUT_EN(rtcio_num) \
    I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TS_REG, (uint8_t)(RTC_GPIO_ENABLE_W1TS_S + rtcio_num), 1)

/**
 * Disable GPIO output.
 */
#define I_GPIO_OUTPUT_DIS(gpio_num) \
    I_RTCIO_OUTPUT_DIS(hulp_gtr(gpio_num))

/**
 * Disable RTCIO output.
 */
#define I_RTCIO_OUTPUT_DIS(rtcio_num) \
    I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TC_REG, (uint8_t)(RTC_GPIO_ENABLE_W1TC_S + rtcio_num), 1)

/**
 * Set GPIO internal pullup.
 */
#define I_GPIO_PULLUP(gpio_num, enable) \
    I_RTCIO_PULLUP(hulp_gtr(gpio_num), enable)

/**
 * Set RTCIO internal pullup.
 */
#define I_RTCIO_PULLUP(rtcio_num, enable) \
    I_WR_REG_BIT(rtc_io_desc[(rtcio_num)].reg, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].pullup), ( (enable) ? 1 : 0) )

/**
 * Set GPIO internal pulldown.
 */
#define I_GPIO_PULLDOWN(gpio_num, enable) \
    I_RTCIO_PULLDOWN(hulp_gtr(gpio_num), enable)

/**
 * Set RTCIO internal pulldown.
 */
#define I_RTCIO_PULLDOWN(rtcio_num, enable) \
    I_WR_REG_BIT(rtc_io_desc[(rtcio_num)].reg, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].pulldown), ( (enable) ? 1 : 0) )

/**
 * Set RTCIO internal pull mode.
 * pull_mode: GPIO_PULLUP_ONLY, GPIO_PULLDOWN_ONLY, GPIO_PULLUP_PULLDOWN, GPIO_FLOATING
 */
#define I_RTCIO_PULL_MODE(rtcio_num, pull_mode) \
    I_WR_REG(rtc_io_desc[(rtcio_num)].reg, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].pullup), (uint8_t)(hulp_log2(rtc_io_desc[(rtcio_num)].pullup) + 1), (uint8_t)( ((pull_mode) == GPIO_PULLUP_ONLY ? 1 : ((pull_mode) == GPIO_PULLDOWN_ONLY ? 2 : ((pull_mode) == GPIO_PULLUP_PULLDOWN ? 3 : 0))) ) )

/**
 * Set GPIO internal pull mode.
 * pull_mode: GPIO_PULLUP_ONLY, GPIO_PULLDOWN_ONLY, GPIO_PULLUP_PULLDOWN, GPIO_FLOATING
 */
#define I_GPIO_PULL_MODE(gpio_num, pull_mode) \
    I_RTCIO_PULL_MODE(hulp_gtr(gpio_num), pull_mode)


/**
 * Toggle GPIO output value.
 * This is a simple implementation and should not be used if timing requirements are strict.
 */
#define M_GPIO_TOGGLE(gpio_num) \
    M_RTCIO_TOGGLE(hulp_gtr(gpio_num))

/**
 * Toggle RTCIO output value.
 * This is a simple implementation and should not be used if timing requirements are strict.
 */
#define M_RTCIO_TOGGLE(rtcio_num) \
    I_RD_REG_BIT(RTC_GPIO_OUT_REG, (uint8_t)(RTC_GPIO_OUT_DATA_S + (rtcio_num))), \
    I_RTCIO_SET((rtcio_num), 0), \
    I_BGE(2,1), \
    I_RTCIO_SET((rtcio_num), 1)

/**
 * Read RTCIO input values into R0.
 * If low_rtcio <= 15, up to 16 bits may be returned [RTCIO_15:RTCIO_0] from the lower RTCIO input register
 * If low_rtcio > 15, up to 2 bits may be returned [RTCIO_17:RTCIO_16] from the upper RTCIO input register
 */
#define I_RTCIOS_READ(low_rtcio, num) \
    I_RD_REG(RTC_GPIO_IN_REG, (uint8_t)(RTC_GPIO_IN_NEXT_S + (low_rtcio)), \
        (uint8_t)(RTC_GPIO_IN_NEXT_S + (( ((low_rtcio) + (num) - 1) < 17) ? ((low_rtcio) + (num) - 1) : (17 - low_rtcio))))

/**
 * Latch GPIO config. Use before returning to deep sleep to maintain state.
 * The state will not change (eg. output level) until a later I_GPIO_UNHOLD.
 */
#define I_GPIO_HOLD_EN(gpio_num) \
    I_RTCIO_HOLD_EN(hulp_gtr(gpio_num))

/**
 * Latch individual RTCIO config. Use before returning to deep sleep to maintain state.
 */
#define I_RTCIO_HOLD_EN(rtcio_num) \
    I_WR_REG_BIT(rtc_io_desc[(rtcio_num)].reg, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].hold), 1)

/**
 * Unlatch individual GPIO config.
 */
#define I_GPIO_HOLD_DIS(gpio_num) \
    I_RTCIO_HOLD_DIS(hulp_gtr(gpio_num))

/**
 * Unlatch individual RTCIO config.
 */
#define I_RTCIO_HOLD_DIS(rtcio_num) \
    I_WR_REG_BIT(rtc_io_desc[(rtcio_num)].reg, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].hold), 0)

/**
 * Unlatch RTCIO config. Use before returning to deep sleep to maintain state.
 * The state will not change (eg. output level) until a later I_GPIO_UNHOLD.
 * Note: This uses a different reg to I_GPIO_HOLD_EN. Do not confuse the two.
 */
#define I_GPIO_FORCE_HOLD_EN(gpio_num) \
    I_RTCIO_FORCE_HOLD_EN(hulp_gtr(gpio_num))

/**
 * Latch RTCIO config. Use before returning to deep sleep to maintain state.
 * Note: This uses a different reg to I_RTCIO_HOLD_EN. Do not confuse the two.
 */
#define I_RTCIO_FORCE_HOLD_EN(rtcio_num) \
    I_WR_REG_BIT(RTC_CNTL_HOLD_FORCE_REG, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].hold_force), 1)

/**
 * Unlatch GPIO config.
 */
#define I_GPIO_FORCE_HOLD_DIS(gpio_num) \
    I_RTCIO_FORCE_HOLD_DIS(hulp_gtr(gpio_num))

/**
 * Unlatch RTCIO config.
 */
#define I_RTCIO_FORCE_HOLD_DIS(rtcio_num) \
    I_WR_REG_BIT(RTC_CNTL_HOLD_FORCE_REG, (uint8_t)hulp_log2(rtc_io_desc[(rtcio_num)].hold_force), 0)

/**
 * Latch all RTCIOs configs.
 * Use before I_HALT to maintain state while sleeping.
 * This is not necessary if RTC peripherals are force on: hulp_peripherals_on()
 */
#define M_FORCE_HOLD_ALL_EN() \
    I_WR_REG(RTC_CNTL_HOLD_FORCE_REG, 0, 7, 0xFF), \
    I_WR_REG(RTC_CNTL_HOLD_FORCE_REG, 8, 15, 0xFF), \
    I_WR_REG(RTC_CNTL_HOLD_FORCE_REG, 16, SOC_RTC_IO_PIN_COUNT - 1, 0xFF)

/**
 * Unlatch all RTCIOs.
 * Use this to allow altering RTCIO config after a previous M_FORCE_HOLD_ALL_EN().
 * Typically, this would be at the start of a ULP program that uses M_FORCE_HOLD_ALL_EN() before I_HALT().
 */
#define M_FORCE_HOLD_ALL_DIS() \
    I_WR_REG(RTC_CNTL_HOLD_FORCE_REG, 0, 7, 0), \
    I_WR_REG(RTC_CNTL_HOLD_FORCE_REG, 8, 15, 0), \
    I_WR_REG(RTC_CNTL_HOLD_FORCE_REG, 16, SOC_RTC_IO_PIN_COUNT - 1, 0)

/**
 * Set GPIO output strength. Ensure pin is output capable!
 */
#define I_GPIO_SET_DRIVE_CAP(gpio_num, drive_cap) \
    I_RTCIO_SET_DRIVE_CAP(hulp_gtr(gpio_num), drive_cap)

/**
 * Set GPIO output strength. Ensure pin is output capable!
 */
#define I_RTCIO_SET_DRIVE_CAP(rtcio_num, drive_cap) \
    I_WR_REG(rtc_io_desc[(rtcio_num)].reg, rtc_io_desc[(rtcio_num)].drv_s, rtc_io_desc[(rtcio_num)].drv_s + 1, (uint8_t)(drive_cap))

/**
 * Set RTC peripherals automatic power down
 */
#define I_PWR_PERI_ENABLE_PD(enable) \
    I_WR_REG_BIT(RTC_CNTL_PWC_REG, RTC_CNTL_PD_EN_S, (enable) ? 1 : 0)

/**
 * Set RTC peripherals force powered down
 */
#define I_PWR_PERI_FORCE_PD(enable) \
    I_WR_REG_BIT(RTC_CNTL_PWC_REG, RTC_CNTL_PWC_FORCE_PD_S, (enable) ? 1 : 0)

/**
 * Set RTC peripherals force powered up
 */
#define I_PWR_PERI_FORCE_PU(enable) \
    I_WR_REG_BIT(RTC_CNTL_PWC_REG, RTC_CNTL_PWC_FORCE_PU_S, (enable) ? 1 : 0)

/**
 * Read a bit from peripheral register into R0
 *
 * This instruction can access RTC_CNTL_, RTC_IO_, SENS_, and RTC_I2C peripheral registers.
 */
#define I_RD_REG_BIT(reg, shift) I_RD_REG(reg, shift, shift)

/**
 * Ensure num_bits is >= bits in val (eg. 0b111 must have num_bits >=3; higher bits will be written regardless)
 */
#define I_I2C_WRITE_BITS(sub_addr, val, shift, num_bits, slave_sel) { .i2c = {\
        .i2c_addr = sub_addr, \
        .data = val, \
        .low_bits = shift, \
        .high_bits = ((shift) + (num_bits) - 1), \
        .i2c_sel = slave_sel, \
        .unused = 0, \
        .rw = SUB_OPCODE_I2C_WR, \
        .opcode = OPCODE_I2C } }

/**
 * Power up ADC
 */
#define I_ADC_POWER_ON() I_WR_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, SENS_FORCE_XPD_SAR_S + 1, SENS_FORCE_XPD_SAR_PU)

/**
 * Power down ADC
 */
#define I_ADC_POWER_OFF() I_WR_REG(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR_S, SENS_FORCE_XPD_SAR_S + 1, SENS_FORCE_XPD_SAR_PD)

/**
 * Check if RTC controller is ready to wake the SoC
 */
#define I_GET_WAKE_READY() I_RD_REG_BIT(RTC_CNTL_LOW_POWER_ST_REG, RTC_CNTL_RDY_FOR_WAKEUP_S)

/**
 * I_WAKE when the SoC is ready, waiting if necessary
 */
#define M_WAKE_WHEN_READY() \
    I_GET_WAKE_READY(), \
    I_BL(-1, 1), \
    I_WAKE()

/**
 * Read up to 16 bits of the 48-bit RTC tick registers, from low_bit.
 */
#define I_RD_TICKS_REG(low_bit) \
    I_RD_REG( ( (low_bit) < 32 ? RTC_CNTL_TIME0_REG : RTC_CNTL_TIME1_REG ), (uint8_t)((low_bit) < 32 ? (low_bit) : ((low_bit) - 32)), 31)

/**
 * Get 16 bits of RTC ticks register into R0, range optimised for the provided time (in milliseconds).
 */
#define I_RD_TICKS(timebase_ms) I_RD_TICKS_REG(hulp_ms_to_ulp_tick_shift(timebase_ms))

/**
 * Set bit to update the RTC ticks register
 */
#define I_FLAG_UPDATE_TICKS() I_WR_REG_BIT(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE_S, 1)

/**
 * Get the bit indicating if RTC ticks register is valid
 */
#define I_GET_TICKS_VALID() I_RD_REG_BIT(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_VALID_S)

/**
 * This is a simple way to repeat a task every interval_ms. Useful if you have multiple tasks that run at different intervals.
 *  eg. Suppose the ULP needed to wake every 30ms to check button input, but also needed to blink an LED every 500ms and communicate
 *  with an i2c slave every 10 seconds.
 *   Example program flow:
 *          //Handle Button
 *      M_UPDATE_TICKS(), //Update the current time
 *      M_IF_MS_ELAPSED(MY_LED_LABEL,   500, MY_I2C_LABEL),
 *          //Handle LED
 *      M_IF_MS_ELAPSED(MY_I2C_LABEL, 10000, MY_HALT_LABEL),
 *          //Handle I2C
 *      M_LABEL(MY_HALT_LABEL),
 *          I_HALT(),
 * 
 * id_label: A unique label number for this block
 * interval_ms: Time (ms) between each execution of this task
 * else_goto_label: The label number to branch to if interval_ms has not elapsed (typically this would be at the very end of the task)
 */
#define M_IF_MS_ELAPSED(id_label, interval_ms, else_goto_label) \
    M_IF_TICKS_ELAPSED(id_label, hulp_ms_to_ulp_ticks((interval_ms)), hulp_ms_to_ulp_tick_shift((interval_ms)), else_goto_label)

#define M_IF_TICKS_ELAPSED(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label) \
    M_IF_TICKS_ELAPSED_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, R1, R2)

/*After, reg_scr1 will always be non-zero (unless this is at RTC_SLOW_MEM[0], which it shouldn't be) */
#define M_IF_TICKS_ELAPSED_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, reg_scr1, reg_scr2) \
    M_LABEL(id_label), \
        I_RD_TICKS_REG(ticks_reg_low_bit), \
        I_MOVR(reg_scr2, R0), \
        M_MOVL(reg_scr1, (id_label)), \
        I_LD(R0, reg_scr1, 7), \
        I_SUBR(R0, reg_scr2, R0), \
        I_BGE(3, (interval_ticks)), \
        M_BX((else_goto_label)), \
        I_BXI(0), /*Reserved word for storing previous tick count*/ \
        I_ST(reg_scr2, reg_scr1, 7)

/**
 * As per M_IF_MS_ELAPSED, with logic reversed.
 * ie. Branches to else_goto_label every time interval_ms elapses; meanwhile continues.
 */
#define M_IF_MS_WAITING(id_label, interval_ms, else_goto_label) \
    M_IF_TICKS_WAITING(id_label, hulp_ms_to_ulp_ticks((interval_ms)), hulp_ms_to_ulp_tick_shift((interval_ms)), else_goto_label)

#define M_IF_TICKS_WAITING(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label) \
    M_IF_TICKS_WAITING_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, R1, R2)

/*After, reg_scr1 will always be non-zero (unless this is at RTC_SLOW_MEM[0], which it shouldn't be) */
#define M_IF_TICKS_WAITING_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, reg_scr1, reg_scr2) \
    M_LABEL(id_label), \
        I_RD_TICKS_REG(ticks_reg_low_bit), \
        I_MOVR(reg_scr2, R0), \
        M_MOVL(reg_scr1, (id_label)), \
        I_LD(R0, reg_scr1, 8), \
        I_SUBR(R0, reg_scr2, R0), \
        I_BL(4, (interval_ticks)), \
        I_ST(reg_scr2, reg_scr1, 8), \
        M_BX((else_goto_label)), \
        I_BXI(0) /*Reserved word for storing previous tick count*/ \

/**
 * Set the entry point (in words) next time the ULP runs.
 */
#define M_SET_ENTRY(word_entry) \
    I_WR_REG(SENS_SAR_START_FORCE_REG, SENS_PC_INIT_S + 8, SENS_PC_INIT_S + 10, (uint8_t)(((word_entry) >> 8) & 0x7)), \
    I_WR_REG(SENS_SAR_START_FORCE_REG, SENS_PC_INIT_S + 0, SENS_PC_INIT_S +  7, (uint8_t)(word_entry) & 0xFF)

/**
 * Set the entry point to the given label. Next time the ULP runs, it will begin from that point.
 */
#define M_SET_ENTRY_LBL(label_entry, program_ptr, program_size) \
    M_SET_ENTRY((uint16_t)hulp_get_label_pc(label_entry, program_ptr, program_size))

/**
 * Get interrupt triggered bits for RTCIO
 */
#define I_RTCIO_INT_RD(low_rtcio, num) I_RD_REG(RTC_GPIO_STATUS_REG, (uint8_t)(RTC_GPIO_STATUS_INT_S + (low_rtcio)), \
        (uint8_t)(RTC_GPIO_STATUS_INT_S + (( ((low_rtcio) + (num) - 1) < 17) ? ((low_rtcio) + (num) - 1) : (17 - low_rtcio))))

#define I_GPIO_INT_RD(gpio_num) I_RTCIO_INT_RD(hulp_gtr(gpio_num), 1)

/**
 * Clear interrupt triggered bits for RTCIO
 */
#define I_RTCIO_INT_CLR(low_rtcio, num) I_WR_REG(RTC_GPIO_STATUS_W1TC_REG, (uint8_t)(RTC_GPIO_STATUS_INT_W1TC_S + (low_rtcio)), \
        (uint8_t)(RTC_GPIO_STATUS_INT_W1TC_S + (low_rtcio) + (num) - 1), \
        (uint8_t)((1 << (num)) - 1))

/**
 * Clear interrupt triggered bits for GPIO
 */
#define I_GPIO_INT_CLR(gpio_num) I_RTCIO_INT_CLR(hulp_gtr(gpio_num), 1)

/**
 * Set interrupt type for RTCIO (intr_type: GPIO_INTR_DISABLE, GPIO_INTR_ANYEDGE, GPIO_INTR_LOW_LEVEL, or GPIO_INTR_HIGH_LEVEL)
 */
#define I_RTCIO_INT_SET_TYPE(rtcio_num, intr_type) I_WR_REG((RTC_GPIO_PIN0_REG + ((rtcio_num) * sizeof(uint32_t))), \
    RTC_GPIO_PIN0_INT_TYPE_S, (uint8_t)(RTC_GPIO_PIN0_INT_TYPE_S + 2), (uint8_t)(intr_type))


/**
 * Set interrupt type for GPIO (intr_type: GPIO_INTR_DISABLE, GPIO_INTR_ANYEDGE, GPIO_INTR_LOW_LEVEL, or GPIO_INTR_HIGH_LEVEL)
 */
#define I_GPIO_INT_SET_TYPE(gpio_num, intr_type) I_RTCIO_INT_SET_TYPE(hulp_gtr(gpio_num), intr_type)

#define I_SET_EXT0_RTCIO(rtcio_num) I_WR_REG(RTC_IO_EXT_WAKEUP0_REG, RTC_IO_EXT_WAKEUP0_SEL_S, RTC_IO_EXT_WAKEUP0_SEL_S, (rtcio_num))

/**
 * Set EXT0 wakeup GPIO
 */
#define I_SET_EXT0_GPIO(gpio_num) I_SET_EXT0_RTCIO(hulp_gtr(gpio_num))

/**
 * Set EXT0 wakeup level (0 or 1)
 */
#define I_SET_EXT0_LEVEL(level) I_WR_REG(RTC_CNTL_EXT_WAKEUP_CONF_REG, RTC_CNTL_EXT_WAKEUP0_LV_S, RTC_CNTL_EXT_WAKEUP0_LV_S, (level) ? 1 : 0)

#define I_CLR_EXT1_INT() I_WR_REG(RTC_CNTL_EXT_WAKEUP1_REG, RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_S, RTC_CNTL_EXT_WAKEUP1_STATUS_CLR_S, 1)

#define I_RD_EXT1_RTCIO_INT(low_rtcio, num) I_RD_REG(RTC_CNTL_EXT_WAKEUP1_STATUS_REG, (uint8_t)(RTC_CNTL_EXT_WAKEUP1_STATUS_S + (low_rtcio)), \
        (uint8_t)(RTC_CNTL_EXT_WAKEUP1_STATUS_S + (( ((low_rtcio) + (num) - 1) < 17) ? ((low_rtcio) + (num) - 1) : (17 - low_rtcio))))

#define I_RD_EXT1_GPIO_INT(gpio_num) I_RD_EXT1_RTCIO_INT(hulp_gtr(gpio_num), 1)

#define I_EXT1_RTCIO_EN(rtcio_num) I_WR_REG(RTC_CNTL_EXT_WAKEUP1_REG, (uint8_t)(RTC_CNTL_EXT_WAKEUP1_SEL_S + (rtcio_num)), (uint8_t)(RTC_CNTL_EXT_WAKEUP1_SEL_S + (rtcio_num)), 1)
#define I_EXT1_RTCIO_DIS(rtcio_num) I_WR_REG(RTC_CNTL_EXT_WAKEUP1_REG, (uint8_t)(RTC_CNTL_EXT_WAKEUP1_SEL_S + (rtcio_num)), (uint8_t)(RTC_CNTL_EXT_WAKEUP1_SEL_S + (rtcio_num)), 0)

/**
 * Add GPIO to EXT1 wakeup group
 */
#define I_EXT1_GPIO_EN(gpio_num) I_EXT1_RTCIO_EN(hulp_gtr(gpio_num))

/**
 * Remove GPIO from EXT1 wakeup group
 */
#define I_EXT1_GPIO_DIS(gpio_num) I_EXT1_RTCIO_DIS(hulp_gtr(gpio_num))

/**
 * Set EXT1 wakeup trigger level (ESP_EXT1_WAKEUP_ALL_LOW or ESP_EXT1_WAKEUP_ANY_HIGH)
 */
#define I_SET_EXT1_LEVEL(level) I_WR_REG(RTC_CNTL_EXT_WAKEUP_CONF_REG, RTC_CNTL_EXT_WAKEUP1_LV_S, RTC_CNTL_EXT_WAKEUP1_LV_S, (level) ? 1 : 0)

/**
 * Enable EXT0 wakeup
 */
#define I_EXT0_EN() I_WR_REG_BIT(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA_S + 0, 1)

/**
 * Disable EXT0 wakeup
 */
#define I_EXT0_DIS() I_WR_REG_BIT(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA_S + 0, 0)

/**
 * Enable EXT1 wakeup
 */
#define I_EXT1_EN() I_WR_REG_BIT(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA_S + 1, 1)

/**
 * Disable EXT1 wakeup
 */
#define I_EXT1_DIS() I_WR_REG_BIT(RTC_CNTL_WAKEUP_STATE_REG, RTC_CNTL_WAKEUP_ENA_S + 1, 0)


#define MIN_ULP_SLEEP_US (rtc_time_slowclk_to_us(ULP_FSM_PREPARE_SLEEP_CYCLES + ULP_FSM_WAKEUP_SLEEP_CYCLES + REG_GET_FIELD(RTC_CNTL_TIMER2_REG, RTC_CNTL_ULPCP_TOUCH_START_WAIT), esp_clk_slowclk_cal_get()))

/**
 * Do not use this directly, use M_SET_WAKEUP_PERIOD for more accurate timing.
 */
#define M_SET_WAKEUP_PERIOD_REG(index, period_us)   \
    I_WR_REG((uint32_t)(SENS_ULP_CP_SLEEP_CYC0_REG + (index) * sizeof(uint32_t)), 0, 7, (uint8_t)((rtc_time_us_to_slowclk((period_us), esp_clk_slowclk_cal_get()) >> 0) & 0xFF)), \
    I_WR_REG((uint32_t)(SENS_ULP_CP_SLEEP_CYC0_REG + (index) * sizeof(uint32_t)), 8, 15, (uint8_t)((rtc_time_us_to_slowclk((period_us), esp_clk_slowclk_cal_get()) >> 8) & 0xFF)), \
    I_WR_REG((uint32_t)(SENS_ULP_CP_SLEEP_CYC0_REG + (index) * sizeof(uint32_t)), 16, 23, (uint8_t)((rtc_time_us_to_slowclk((period_us), esp_clk_slowclk_cal_get()) >> 16) & 0xFF)), \
    I_WR_REG((uint32_t)(SENS_ULP_CP_SLEEP_CYC0_REG + (index) * sizeof(uint32_t)), 24, 31, (uint8_t)((rtc_time_us_to_slowclk((period_us), esp_clk_slowclk_cal_get()) >> 24) & 0xFF))

/**
 * Set the ULP wakeup period in microseconds (equivalent to ulp_set_wakeup_period).
 */
#define M_SET_WAKEUP_PERIOD(index, period_us)   \
    M_SET_WAKEUP_PERIOD_REG((index), (period_us) > (MIN_ULP_SLEEP_US) ? ((period_us) - MIN_ULP_SLEEP_US) : 0)

/**
 * Increment the value in a register
 */
#define M_REG_INC(reg, shift) \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 0)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 0), 0b1), \
    I_BL(23, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 1)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 1), 0b10), \
    I_BL(20, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 2)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 2), 0b100), \
    I_BL(17, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 3)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 3), 0b1000), \
    I_BL(14, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 4)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 4), 0b10000), \
    I_BL(11, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 5)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 5), 0b100000), \
    I_BL(8, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 6)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 6), 0b1000000), \
    I_BL(5, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 7)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 7), 0b10000000), \
    I_BL(2, 1), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 7), 0)

/**
 * Decrement the value in a register
 */
#define M_REG_DEC(reg, shift) \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 0)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 0), 0b0), \
    I_BGE(23, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 1)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 1), 0b01), \
    I_BGE(20, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 2)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 2), 0b011), \
    I_BGE(17, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 3)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 3), 0b0111), \
    I_BGE(14, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 4)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 4), 0b01111), \
    I_BGE(11, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 5)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 5), 0b011111), \
    I_BGE(8, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 6)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 6), 0b0111111), \
    I_BGE(5, 1), \
    I_RD_REG_BIT(reg, (uint8_t)((shift) + 7)), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 7), 0b01111111), \
    I_BGE(2, 1), \
    I_WR_REG(reg, (uint8_t)((shift) + 0), (uint8_t)((shift) + 7), 0b011111111)

#endif // HULP_MACROS_H