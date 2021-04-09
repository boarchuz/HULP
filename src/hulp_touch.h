#ifndef HULP_TOUCH_H
#define HULP_TOUCH_H

#include "driver/gpio.h"
#include "driver/touch_pad.h"

#include "hulp.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SWAPPED_TOUCH_INDEX(x) ((x) == TOUCH_PAD_NUM9 ? TOUCH_PAD_NUM8 : ((x) == TOUCH_PAD_NUM8 ? TOUCH_PAD_NUM9 : (x)))

typedef struct {
    uint16_t fastclk_meas_cycles;
    touch_high_volt_t high_voltage;
    touch_low_volt_t low_voltage;
    touch_volt_atten_t attenuation;
} hulp_touch_controller_config_t;

#define HULP_TOUCH_CONTROLLER_CONFIG_DEFAULT() { \
    .fastclk_meas_cycles = TOUCH_PAD_MEASURE_CYCLE_DEFAULT, \
    .high_voltage = TOUCH_HVOLT_2V4, \
    .low_voltage = TOUCH_LVOLT_0V8, \
    .attenuation = TOUCH_HVOLT_ATTEN_1V5, \
}

/**
 * Prepare touch controller for ULP control.
 * Do this once, then configure each pin using hulp_configure_touch_pin.
 * fastclk_meas_cycles: measurement time in fastclk (8MHz) cycles. (65535 = 8.19ms)
 * Shorter = lower power consumption; Longer = higher counts (better possible signal/noise filtering)
 */
esp_err_t hulp_configure_touch_controller(const hulp_touch_controller_config_t *config);

typedef struct {
    touch_cnt_slope_t slope;
    touch_tie_opt_t tie_opt;
} hulp_touch_pin_config_t;

#define HULP_TOUCH_PIN_CONFIG_DEFAULT() { \
    .slope = TOUCH_PAD_SLOPE_DEFAULT, \
    .tie_opt = TOUCH_PAD_TIE_OPT_DEFAULT, \
}

/**
 * Initialise and configure a pin for touch function.
 */
esp_err_t hulp_configure_touch_pin(gpio_num_t touch_gpio, const hulp_touch_pin_config_t *config);


#define I_TOUCH_GET_PAD_THRESHOLD(touch_num) \
    I_RD_REG((SENS_SAR_TOUCH_THRES1_REG + (4 * ((uint8_t)SWAPPED_TOUCH_INDEX(touch_num)/2))), (uint8_t)(SWAPPED_TOUCH_INDEX(touch_num)%2 ? 0 : 16), (uint8_t)(SWAPPED_TOUCH_INDEX(touch_num)%2 ? 15 : 31))

#define I_TOUCH_GET_GPIO_THRESHOLD(gpio_num) \
    I_TOUCH_GET_PAD_THRESHOLD(hulp_touch_get_pad_num(gpio_num))

#define I_TOUCH_GET_PAD_VALUE(touch_num) \
    I_RD_REG((SENS_SAR_TOUCH_OUT1_REG + (4 * ((uint8_t)SWAPPED_TOUCH_INDEX(touch_num)/2))), (uint8_t)((SWAPPED_TOUCH_INDEX(touch_num)%2) ? 0 : 16), (uint8_t)((SWAPPED_TOUCH_INDEX(touch_num)%2) ? 15 : 31))

#define I_TOUCH_GET_GPIO_VALUE(gpio_num) \
    I_TOUCH_GET_PAD_VALUE(hulp_touch_get_pad_num(gpio_num))

#define I_TOUCH_GET_DONE_BIT() \
    I_RD_REG_BIT(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_MEAS_DONE_S)

#define M_TOUCH_WAIT_DONE() \
    I_TOUCH_GET_DONE_BIT(), \
    I_BL(-1,1)

#define M_TOUCH_BEGIN() \
    I_WR_REG_BIT(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_S, 0), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_S, 1)

//Junk:
#define I_TOUCH_EN(gpio_num, enable) \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_WORKEN_S + SWAPPED_TOUCH_INDEX(hulp_touch_get_pad_num(gpio_num))), enable ? 1 : 0)

#define I_TOUCH_INT_SET1_EN(gpio_num, enable) \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_OUTEN1_S + SWAPPED_TOUCH_INDEX(hulp_touch_get_pad_num(gpio_num))), enable ? 1 : 0)

#define I_TOUCH_INT_SET2_EN(gpio_num, enable) \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_OUTEN2_S + SWAPPED_TOUCH_INDEX(hulp_touch_get_pad_num(gpio_num))), enable ? 1 : 0)

#define I_TOUCH_INT_SET_SOURCE(touch_trigger_source) /*TOUCH_TRIGGER_SOURCE_BOTH / TOUCH_TRIGGER_SOURCE_SET1*/ \
    I_WR_REG_BIT(SENS_SAR_TOUCH_CTRL1_REG, SENS_TOUCH_OUT_1EN_S, TOUCH_TRIGGER_SOURCE)

#define M_TOUCH_SW_READ_PAD_END(touch_num) \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_OUTEN1_S + SWAPPED_TOUCH_INDEX(touch_num)), 0), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_OUTEN2_S + SWAPPED_TOUCH_INDEX(touch_num)), 0), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_WORKEN_S + SWAPPED_TOUCH_INDEX(touch_num)), 0)

#define M_TOUCH_SW_READ_GPIO_END(gpio_num) \
    M_TOUCH_SW_READ_PAD_END(hulp_touch_get_pad_num(gpio_num))

#define M_TOUCH_SW_READ_PAD_BEGIN_V(touch_num) \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_OUTEN1_S + SWAPPED_TOUCH_INDEX(touch_num)), 1), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_OUTEN2_S + SWAPPED_TOUCH_INDEX(touch_num)), 1), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_ENABLE_REG, (uint8_t)(SENS_TOUCH_PAD_WORKEN_S + SWAPPED_TOUCH_INDEX(touch_num)), 1), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_S, 0), \
    I_WR_REG_BIT(SENS_SAR_TOUCH_CTRL2_REG, SENS_TOUCH_START_EN_S, 1)

#define M_TOUCH_SW_READ_GPIO_BEGIN_V(gpio_num) \
    M_TOUCH_SW_READ_PAD_BEGIN_V(hulp_touch_get_pad_num(gpio_num))

/**
 * Internal. Do not use directly.
 */
int hulp_touch_get_pad_num(gpio_num_t pin);

#ifdef __cplusplus
}
#endif

#endif // HULP_TOUCH_H