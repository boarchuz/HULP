#ifndef HULP_APA_H_
#define HULP_APA_H_

/**
 * Basic APA-style RGB LED driver.
 * Only one LED is supported.
 * Useful for status indication, for example.
 */

#include "hulp.h"

#define M_APA1_SET(label_return_point, label_apa1_entry, brightness, red, green, blue) \
    I_MOVI(R1, ((brightness) & 0x1F) << 8 | (blue)), \
    I_MOVI(R2, (green) << 8 | (red)), \
    M_MOVL(R3, label_return_point), \
    M_BX(label_apa1_entry), \
    M_LABEL(label_return_point)

#define M_INCLUDE_APA1(label_entry, clk_gpio, data_gpio) \
    M_INCLUDE_APA1_(label_entry, clk_gpio, data_gpio, R1, R2, R3)

#define M_INCLUDE_APA1_(label_entry, clk_gpio, data_gpio, reg_br_blue, reg_green_red, reg_return) \
    M_LABEL(label_entry), \
        I_STAGE_RST(), \
        I_GPIO_SET(data_gpio, 0), \
        I_GPIO_SET(clk_gpio, 1), \
        I_STAGE_INC(1), \
        I_LSHI(R0, R0, 1), \
        I_GPIO_SET(clk_gpio, 0), \
        I_JUMPS(7,48,JUMPS_GE), \
        I_JUMPS(-5,32,JUMPS_LT), \
        I_JUMPS(2,33,JUMPS_GE), \
        I_ORI(R0, reg_br_blue, 0xE000), \
        I_BL(-9, 32768), \
        I_GPIO_SET(data_gpio, 1), \
        I_BGE(-10,0), \
        I_JUMPS(4,64,JUMPS_GE), \
        I_JUMPS(-4,49,JUMPS_GE), \
        I_MOVR(R0,reg_green_red), \
        I_BGE(-6,0), \
        I_JUMPS(3,96,JUMPS_GE), \
        I_GPIO_SET(data_gpio, 1), \
        I_BGE(-17,0), \
        I_GPIO_SET(data_gpio, 0), \
        I_BXR(reg_return)

#endif