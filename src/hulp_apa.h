#ifndef HULP_APA_H
#define HULP_APA_H

/**
 * APA-style (serial clock/data) RGB LED driver.
 */

#include "driver/gpio.h"
#include "driver/rtc_io.h"

#include "hulp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Structured APA data for ULP/SoC access
 */
typedef struct {
    union {
        ulp_var_t msb;
        struct {
            uint32_t blue : 8;          /*!< 8-bit blue */
            uint32_t brightness : 5;    /*!< 5-bit brightness (0-31) */
            uint32_t unused1 : 3;
            uint32_t unused2 : 16;
        };
    };
    union {
        ulp_var_t lsb;
        struct {
            uint32_t red : 8;           /*!< 8-bit red */
            uint32_t green : 8;         /*!< 8-bit green */
            uint32_t unused3 : 16;
        };
    };
} ulp_apa_t;

_Static_assert(sizeof(ulp_apa_t) == (2 * sizeof(uint32_t)), "ulp_apa_t size should be 2 words" );

/**
 * @brief Transmit a buffer (array of ulp_apa_t) to a string of APA RGBs
 * @param label_entry A new unique label for this subroutine. Branch to this label to transmit.
 * @param clk_gpio Clock pin (GPIO_NUM_x).
 * @param data_gpio Data pin (GPIO_NUM_x).
 * @param ulp_apa_var The array of ``ulp_apa_t`` in RTC Slow Memory.
 * @param num_apas The number of APAs in the array.
 * @param reg_scr General purpose scratch register (R1-R3).
 * @param reg_return The general purpose scratch register containing the return address (R1-R3).
 * 
 * @code{c}
 *      #define NUM_APAS 10
 *      RTC_DATA_ATTR ulp_apa_t apas[NUM_APAS];
 * 
 *      M_MOVL(R3, LBL_RETURN_POINT), //Prepare return
 *      M_BX(LBL_APA_ENTRY), //Branch to the label given as APA entry
 *      M_LABEL(LBL_RETURN_POINT), //Will return here after TX
 * 
 *      M_APA_TX(LBL_APA_ENTRY, GPIO_NUM_26, GPIO_NUM_27, apas, NUM_APAS, R1, R3),
 * @endcode
 */
#define M_APA_TX(label_entry, clk_gpio, data_gpio, ulp_apa_var, num_apas, reg_scr, reg_return) \
    M_LABEL(label_entry), \
        I_STAGE_RST(), \
        I_GPIO_SET(data_gpio, 0), \
        I_GPIO_SET(clk_gpio, 1), \
        I_STAGE_INC(1), \
        I_GPIO_SET(clk_gpio, 0), \
        I_JUMPS(-3, 32, JUMPS_LT), \
        I_MOVI(reg_scr, 0), \
        I_STAGE_RST(), \
        I_LD(R0, reg_scr, RTC_WORD_OFFSET(ulp_apa_var)), \
        I_ORI(R0, R0, 0xE000), \
        I_GPIO_SET(data_gpio, 1), \
        I_GPIO_SET(clk_gpio, 1), \
        I_STAGE_INC(1), \
        I_LSHI(R0, R0, 1), \
        I_GPIO_SET(clk_gpio, 0), \
        I_JUMPS(8, 32, JUMPS_GE), \
        I_JUMPS(4, 16, JUMPS_GE), \
        I_BGE(-7, 1 << 15), \
        I_GPIO_SET(data_gpio, 0), \
        I_BGE(-8, 0), \
        I_JUMPS(-3, 17, JUMPS_GE), \
        I_LD(R0, reg_scr, (uint16_t)(RTC_WORD_OFFSET(ulp_apa_var) + 1)), \
        I_BGE(-5, 0), \
        I_ADDI(reg_scr, reg_scr, 2), \
        I_MOVR(R0, reg_scr), \
        I_BL(-18, NUM_LEDS * 2), \
        I_GPIO_SET(data_gpio, 1), \
        I_GPIO_SET(clk_gpio, 1), \
        I_STAGE_INC(1), \
        I_GPIO_SET(clk_gpio, 0), \
        I_JUMPS(-3, 64, JUMPS_LT), \
        I_GPIO_SET(data_gpio, 0), \
        I_BXR(reg_return)

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


#ifdef __cplusplus
}
#endif

#endif // HULP_APA_H