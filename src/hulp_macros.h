#ifndef HULP_MACROS_H_
#define HULP_MACROS_H_

#include "hulp.h"
#include "soc/sens_reg.h"

#define RTC_WORD_OFFSET(x) ((uint32_t*)&(x) - RTC_SLOW_MEM)
#define HAS_DEFAULT_PULLUP(x) ((x == GPIO_NUM_0) || (x == GPIO_NUM_14) || (x == GPIO_NUM_15))
#define HAS_DEFAULT_PULLDOWN(x) ((x == GPIO_NUM_2) || (x == GPIO_NUM_4) || (x == GPIO_NUM_12) || (x == GPIO_NUM_13))



#define I_MOVO(reg_dest, var) \
    I_MOVI(reg_dest, (uint16_t)(RTC_WORD_OFFSET(var)))

#define I_GET(reg_dest, reg_zero, var) \
    I_GETO(reg_dest, reg_zero, 0, var)

#define I_GETO(reg_dest, reg_offset, val_offset, var) \
    I_LD(reg_dest, reg_offset, (uint16_t)(RTC_WORD_OFFSET(var) - (val_offset)))

#define I_PUT(reg_dest, reg_zero, var) \
    I_PUTO(reg_dest, reg_zero, 0, var)

#define I_PUTO(reg_dest, reg_offset, val_offset, var) \
    I_ST(reg_dest, reg_offset, (uint16_t)(RTC_WORD_OFFSET(var) - (val_offset)))

/**
 * Request and wait for the RTC ticks register to update.
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


#define M_GPIO_INIT(gpio_num) \
    I_WR_REG_BIT(rtc_gpio_desc[gpio_num].reg, (uint8_t)log2(rtc_gpio_desc[gpio_num].mux), 1), \
    I_WR_REG(rtc_gpio_desc[gpio_num].reg, rtc_gpio_desc[gpio_num].func, rtc_gpio_desc[gpio_num].func + 1, 0)

#define M_RTCIO_INIT(rtcio_num) \
    M_GPIO_INIT(rtc_to_gpio[(rtcio_num)])

#define I_ANALOG_READ(reg_dest, pin) \
    I_ADC(reg_dest, (uint32_t)(hulp_adc_get_unit(pin) - 1), (uint32_t)(hulp_adc_get_channel_num(pin)))

#define I_GPIO_READ(gpio_num) \
    I_RTCIO_READ(rtc_gpio_desc[gpio_num].rtc_num)

#define I_RTCIO_READ(rtcio_num) \
    I_RTCIOS_READ(rtcio_num, 1)

#define I_GPIO_SET(gpio_num, level) \
    I_RTCIO_SET(rtc_gpio_desc[(gpio_num_t)gpio_num].rtc_num, level)

#define I_RTCIO_SET(rtcio_num, level) \
    I_WR_REG_BIT( ( (level) ? RTC_GPIO_OUT_W1TS_REG : RTC_GPIO_OUT_W1TC_REG ), (uint8_t)( (level) ? (RTC_GPIO_OUT_DATA_W1TS_S + (rtcio_num)) : (RTC_GPIO_OUT_DATA_W1TC_S + (rtcio_num)) ), 1)
    // I_WR_REG_BIT( RTC_GPIO_OUT_REG, (uint8_t)( RTC_GPIO_OUT_DATA_S + (rtcio_num)), (level))

#define I_GPIO_OUTPUT_EN(gpio_num) \
    I_RTCIO_OUTPUT_EN(rtc_gpio_desc[gpio_num].rtc_num)

#define I_RTCIO_OUTPUT_EN(rtcio_num) \
    I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TS_REG, (uint8_t)(RTC_GPIO_ENABLE_W1TS_S + rtcio_num), 1)

#define I_GPIO_OUTPUT_DIS(gpio_num) \
    I_RTCIO_OUTPUT_DIS(rtc_gpio_desc[gpio_num].rtc_num)

#define I_RTCIO_OUTPUT_DIS(rtcio_num) \
    I_WR_REG_BIT(RTC_GPIO_ENABLE_W1TC_REG, (uint8_t)(RTC_GPIO_ENABLE_W1TC_S + rtcio_num), 1)

#define I_GPIO_PULLUP(gpio_num, enable) \
    I_WR_REG_BIT(rtc_gpio_desc[(gpio_num)].reg, (uint8_t)log2(rtc_gpio_desc[(gpio_num)].pullup), ( (enable) ? 1 : 0) )

#define I_RTCIO_PULLUP(rtcio_num, enable) \
    I_GPIO_PULLUP(rtc_to_gpio[(rtcio_num)],(enable))

#define I_GPIO_PULLDOWN(gpio_num, enable) \
    I_WR_REG_BIT(rtc_gpio_desc[(gpio_num)].reg, (uint8_t)log2(rtc_gpio_desc[(gpio_num)].pulldown), ( (enable) ? 1 : 0) )

#define I_RTCIO_PULLDOWN(rtcio_num, enable) \
    I_GPIO_PULLDOWN(rtc_to_gpio[(rtcio_num)],(enable))

#define M_GPIO_TOGGLE(gpio_num) \
    M_RTCIO_TOGGLE(rtc_gpio_desc[gpio_num].rtc_num)

#define M_RTCIO_TOGGLE(rtcio_num) \
    I_RD_REG_BIT(RTC_GPIO_OUT_REG, (uint8_t)(RTC_GPIO_OUT_DATA_S + (rtcio_num))), \
    I_RTCIO_SET((rtcio_num), 0), \
    I_BGE(2,1), \
    I_RTCIO_SET((rtcio_num), 1)

#define I_RTCIOS_READ(low_rtcio, num) \
    I_RD_REG(RTC_GPIO_IN_REG, (uint8_t)(RTC_GPIO_IN_NEXT_S + (low_rtcio)), \
        (uint8_t)(RTC_GPIO_IN_NEXT_S + (( ((low_rtcio) + (num) - 1) < 17) ? ((low_rtcio) + (num) - 1) : (17 - low_rtcio))))

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
 * Convert a time (in milliseconds) to an optimally-shifted 16-bit RTC tick count.
 * Equivalent to hulp_ticks()
 */
#define I_HULP_TICKS(reg_dest, time_ms) \
    I_MOVI(reg_dest, hulp_ticks(time_ms))

/**
 * Get 16 bits of RTC ticks register into R0, range optimised for the provided time (in milliseconds).
 */
#define I_RD_TICKS(timebase_ms) I_RD_TICKS_REG(hulp_tick_shift(timebase_ms))

/**
 * Read up to 16 bits of the 48-bit RTC tick registers, from low_bit.
 */
#define I_RD_TICKS_REG(low_bit) \
    I_RD_REG( ( (low_bit) < 32 ? RTC_CNTL_TIME0_REG : RTC_CNTL_TIME1_REG ), (uint8_t)((low_bit) < 32 ? (low_bit) : ((low_bit) - 32)), 31)

/**
 * Set bit to update the RTC ticks register
 */
#define I_FLAG_UPDATE_TICKS() I_WR_REG_BIT(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_UPDATE_S, 1)

/**
 * Get the bit indicating if RTC ticks register is valid
 */
#define I_GET_TICKS_VALID() I_RD_REG_BIT(RTC_CNTL_TIME_UPDATE_REG, RTC_CNTL_TIME_VALID_S)

#define M_EVERY_X_MS(time_ms,timestamp_offset,else_go_to) \
    I_MOVI(R2,0), \
    I_LD(R1,R2,timestamp_offset), \
    M_CHECK_MS_ELAPSED(time_ms, R1, else_go_to), \
    I_RD_TICKS((time_ms)), \
    I_ST(R0,R2,timestamp_offset)

#define M_IF_MS_ELAPSED(id_label, interval_ms, else_goto_label) \
    M_IF_TICKS_ELAPSED(id_label, hulp_ticks((interval_ms)), hulp_tick_shift((interval_ms)), else_goto_label)

#define M_IF_TICKS_ELAPSED(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label) \
    M_IF_TICKS_ELAPSED_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, R1, R2)

/*After, reg_scr1 will always be non-zero (unless this is at RTC_SLOW_MEM[0], which it shouldn't be) */
#define M_IF_TICKS_ELAPSED_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, reg_scr1, reg_scr2) \
    M_LABEL(id_label), \
        I_RD_TICKS_REG(ticks_reg_low_bit), \
        I_MOVR(reg_scr2,R0), \
        M_MOVL(reg_scr1, (id_label)), \
        I_LD(R0,reg_scr1,7), \
        I_SUBR(R0,reg_scr2,R0), \
        I_BGE(3, (interval_ticks)), \
        M_BX((else_goto_label)), \
        I_BXI(0), /*Reserved word for storing previous tick count*/ \
        I_ST(reg_scr2,reg_scr1,7)

#define M_IF_MS_WAITING(id_label, interval_ms, timebase_ms, else_goto_label) \
    M_IF_TICKS_ELAPSED(id_label, hulp_ticks((interval_ms)), hulp_tick_shift((interval_ms)), else_goto_label)

#define M_IF_TICKS_WAITING(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label) \
    M_IF_TICKS_ELAPSED_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, R1, R2)

/*After, reg_scr1 will always be non-zero (unless this is at RTC_SLOW_MEM[0], which it shouldn't be) */
#define M_IF_TICKS_WAITING_(id_label, interval_ticks, ticks_reg_low_bit, else_goto_label, reg_scr1, reg_scr2) \
    M_LABEL(id_label), \
        I_RD_TICKS_REG(ticks_reg_low_bit), \
        I_MOVR(reg_scr2,R0), \
        M_MOVL(reg_scr1, (id_label)), \
        I_LD(R0,reg_scr1,8), \
        I_SUBR(R0,reg_scr2,R0), \
        I_BL(4, (interval_ticks)), \
        I_ST(reg_scr2,reg_scr1,8), \
        M_BX((else_goto_label)), \
        I_BXI(65535/2) /*Reserved word for storing previous tick count*/ \

/* A minimal variant (3 insn) offering more control, requiring a reg (R1-R3) preloaded with the previous ticks. You need to handle get and save current ticks if == true */
#define M_IF_REG_MS_ELAPSED(reg_ticks_previous, interval_ms, timebase_ms, else_goto_label)  \
        M_IF_REG_TICKS_ELAPSED(reg_ticks_previous, hulp_ticks((interval_ms)), hulp_tick_shift((timebase_ms)), else_goto_label)

#define M_IF_REG_TICKS_ELAPSED(reg_previous_ticks, interval_ticks, ticks_reg_low_bit, else_goto_label) \
    I_RD_TICKS_REG(ticks_reg_low_bit), \
    I_SUBR(R0, R0, reg_previous_ticks), \
    M_BL(else_goto_label, (interval_ticks))

#define M_IF_REG_MS_WAITING(reg_ticks_previous, interval_ms, timebase_ms, else_goto_label)  \
    M_IF_REG_TICKS_WAITING_(reg_ticks_previous, hulp_ticks((interval_ms)), hulp_tick_shift((timebase_ms)), else_goto_label)

#define M_IF_REG_TICKS_WAITING_(reg_previous_ticks, interval_ticks, ticks_reg_low_bit, else_goto_label) \
    I_RD_TICKS_REG(ticks_reg_low_bit), \
    I_SUBR(R0, R0, reg_previous_ticks), \
    M_BGE(else_goto_label, (interval_ticks))

#define M_CHECK_MS_ELAPSED(time_ms,reg_previous,else_go_to) \
    I_RD_TICKS((time_ms)), \
    I_SUBR(R0,R0,reg_previous), \
    M_BL(else_go_to,hulp_ticks((time_ms)))

/**
 * Set the entry point (in words) next time the ULP runs.
 */
#define I_SET_ENTRY(word_entry) \
    I_WR_REG(SENS_SAR_START_FORCE_REG, SENS_PC_INIT_S, SENS_PC_INIT_S + 10, (word_entry))

/**
 * Set the entry point to the given label. Next time the ULP runs, it will begin from that point.
 */
#define I_SET_ENTRY_LBL(label_entry, ulp_program_ptr) \
    I_SET_ENTRY(hulp_label(label_entry, ulp_program_ptr))


#endif