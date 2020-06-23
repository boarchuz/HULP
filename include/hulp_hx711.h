#ifndef HULP_HX711_H
#define HULP_HX711_H

#include "hulp.h"

/**
 * Suggested pin config:
 *  hulp_configure_pin(gpio_data,  RTC_GPIO_MODE_INPUT_ONLY,  false, false);
 *  hulp_configure_pin(gpio_clock, RTC_GPIO_MODE_OUTPUT_ONLY, false, false, 0);
 */

/**
 * Busy wait until data is ready (ie. until HX711 sets data low)
 */
#define M_HX711_WAIT_READY(gpio_data) \
    I_GPIO_READ(gpio_data), \
    I_BGE(-1, 1)

/**
 * Read 24-bit value from a HX711
 * 
 * Two destination registers are required for the output - high16 and low16.
 * As the name implies, these will overlap:
 *      high16: [23:8]
 *      low16:  [15:0]
 * 
 * The SoC may simply OR the outputs together for the full value if required: uint32_t val = ((uint32_t)high16 << 8) | (low16);
 * 
 * reg_high16: Destination reg (R1-R3) to hold the upper 16 bits of the 24-bit value
 * reg_low16: Destination reg (R1-R3) to hold the lower 16 bits of the 24-bit value
 * 
 * For example, to read the upper 16 bits into R1, lower 16 bits into R2:
 *      M_HX711_READ(R1, R2, GPIO_NUM_25, GPIO_NUM_26),
 */
#define M_HX711_READ(reg_high16, reg_low16, gpio_data, gpio_clock) \
    I_STAGE_RST(), \
    I_MOVR(reg_high16, reg_low16), \
    I_GPIO_SET(gpio_clock, 1), \
    I_STAGE_INC(1), \
    I_GPIO_SET(gpio_clock, 0), \
    I_JUMPS(6, 25, JUMPS_GE), \
    I_LSHI(reg_low16, reg_low16, 1), \
    I_GPIO_READ(gpio_data), \
    I_ORR(reg_low16, reg_low16, R0), \
    I_JUMPS(-8, 17, JUMPS_LT), \
    I_BGE(-8, 0)

/**
 * Read upper 16 bits from a HX711 (lower bits will be clocked but ignored)
 * 
 * reg_high16: Destination reg (R1-R3) to hold the upper 16 bits [23:8] of the 24-bit value
 * 
 * For example, to read the upper 16 bits into R1:
 *      M_HX711_READ(R1, GPIO_NUM_25, GPIO_NUM_26),
 */
#define M_HX711_READ16(reg_high16, gpio_data, gpio_clock) \
    I_STAGE_RST(), \
    I_GPIO_SET(gpio_clock, 1), \
    I_STAGE_INC(1), \
    I_GPIO_SET(gpio_clock, 0), \
    I_JUMPS(6, 25, JUMPS_GE), \
    I_GPIO_READ(gpio_data), \
    I_JUMPS(-5, 17, JUMPS_GE), \
    I_LSHI(reg_high16, reg_high16, 1), \
    I_ORR(reg_high16, reg_high16, R0), \
    I_BGE(-8, 0)


#endif