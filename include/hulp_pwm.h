#ifndef HULP_PWM_H
#define HULP_PWM_H



#include "hulp.h"


/**
 * Disable I2C clock high output (pin will be hi-z during on time)
 */
#define I_PWM_HIGH_DIS() I_WR_REG_BIT(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT_S, 0)

/**
 * Enable I2C clock high output (pin will be driven high during on time)
 */
#define I_PWM_HIGH_EN()  I_WR_REG_BIT(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT_S, 1)

#define M_PWM_SET8(clk_freq, pwm_freq, duty8) \
    I_PWM_SET8_H(clk_freq, pwm_freq, duty8), \
    I_PWM_SET8_L(clk_freq, pwm_freq, duty8)

/**
 * Enable pin I2C clock function
 */
#define I_PWM_ATTACH(scl_pin) \
    I_WR_REG(rtc_io_desc[hulp_gtr(scl_pin)].reg, RTC_IO_TOUCH_PAD0_FUN_SEL_S, RTC_IO_TOUCH_PAD0_FUN_SEL_S + 1, 0x3)

/**
 * Disable pin I2C clock function (ie. reset to normal RTC IO function)
 */
#define I_PWM_DETACH(scl_pin) \
    I_WR_REG(rtc_io_desc[hulp_gtr(scl_pin)].reg, RTC_IO_TOUCH_PAD0_FUN_SEL_S, RTC_IO_TOUCH_PAD0_FUN_SEL_S + 1, 0x0)


esp_err_t hulp_pwm_configure(gpio_num_t scl_pin, gpio_num_t sda_pin);

void hulp_pwm_start();

void hulp_pwm_attach(gpio_num_t scl_pin);

void hulp_pwm_detach(gpio_num_t scl_pin);

void hulp_pwm_set_cycles(uint32_t high_cycles, uint32_t low_cycles);

void hulp_pwm_set(uint32_t clk_freq, uint32_t pwm_freq, float duty);

void hulp_pwm_set8(uint32_t clk_freq, uint32_t pwm_freq, uint8_t duty8);

uint8_t hulp_pwm_shift8(uint32_t clk_freq, uint32_t pwm_freq);

ulp_insn_t I_PWM_SET8_H(uint32_t clk_freq, uint32_t pwm_freq, uint8_t duty8);
ulp_insn_t I_PWM_SET8_L(uint32_t clk_freq, uint32_t pwm_freq, uint8_t duty8);

#endif // HULP_PWM_H