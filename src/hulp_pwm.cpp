#include "hulp_pwm.h"

#include "soc/sens_reg.h"
#include "soc/rtc_io_reg.h"
#include "hal/gpio_types.h"
#include "driver/rtc_io.h"
#include "hal/rtc_io_ll.h"
#include "soc/rtc_io_caps.h"
#include "esp_log.h"
#include "hulp.h"

#include "sdkconfig.h"

static const char* TAG = "HULP-PWM";

esp_err_t hulp_pwm_configure(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    assert((scl_pin == GPIO_NUM_2 || scl_pin == GPIO_NUM_4) && "invalid i2c SCL pin, must be 2 or 4");
    assert((sda_pin == GPIO_NUM_0 || sda_pin == GPIO_NUM_15) && "invalid i2c SDA pin, must be 0 or 15");

    int scl_rtcio = rtc_io_number_get(scl_pin);
    int sda_rtcio = rtc_io_number_get(sda_pin);

    //SCL
    rtc_gpio_init(scl_pin);
	rtc_gpio_set_level(scl_pin, 0);
	rtc_gpio_pulldown_dis(scl_pin);
    rtc_gpio_pullup_dis(scl_pin);
    rtc_gpio_set_direction(scl_pin, RTC_GPIO_MODE_INPUT_OUTPUT);

    //SDA
    //Bus can lock up if input is enabled and data pin is low, so disable input here
    rtcio_ll_input_disable(sda_rtcio);
    
    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SCL_SEL, scl_pin == GPIO_NUM_4 ? 0 : 1);
    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SDA_SEL, sda_pin == GPIO_NUM_0 ? 0 : 1);

    //RTC I2C FUN_SEL = 3
    SET_PERI_REG_BITS(rtc_io_desc[scl_rtcio].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_io_desc[scl_rtcio].func);

    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT, 1);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SDA_FORCE_OUT, 0);
    // REG_SET_FIELD(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD, 0);
    // REG_SET_FIELD(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD, 0);
    REG_SET_FIELD(RTC_I2C_SDA_DUTY_REG, RTC_I2C_SDA_DUTY, 0);
    REG_SET_FIELD(RTC_I2C_SCL_START_PERIOD_REG, RTC_I2C_SCL_START_PERIOD, 0);
    REG_SET_FIELD(RTC_I2C_SCL_STOP_PERIOD_REG, RTC_I2C_SCL_STOP_PERIOD, 0);
    REG_SET_FIELD(RTC_I2C_TIMEOUT_REG, RTC_I2C_TIMEOUT, RTC_I2C_TIMEOUT_V);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE, 1);

    REG_SET_FIELD(SENS_SAR_I2C_CTRL_REG, SENS_SAR_I2C_CTRL, SENS_SAR_I2C_CTRL);

    return ESP_OK;
}

void hulp_pwm_start()
{
    REG_SET_BIT(SENS_SAR_I2C_CTRL_REG, SENS_SAR_I2C_START_FORCE);
    REG_SET_BIT(SENS_SAR_I2C_CTRL_REG, SENS_SAR_I2C_START);
}

void hulp_pwm_attach(gpio_num_t scl_pin)
{
    assert((scl_pin == GPIO_NUM_2 || scl_pin == GPIO_NUM_4));
    SET_PERI_REG_BITS(rtc_io_desc[rtc_io_number_get(scl_pin)].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_io_desc[rtc_io_number_get(scl_pin)].func);
}

void hulp_pwm_detach(gpio_num_t scl_pin)
{
    assert((scl_pin == GPIO_NUM_2 || scl_pin == GPIO_NUM_4));
    SET_PERI_REG_BITS(rtc_io_desc[rtc_io_number_get(scl_pin)].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, SOC_PIN_FUNC_RTC_IO, rtc_io_desc[rtc_io_number_get(scl_pin)].func);
}

void hulp_pwm_set_cycles(uint32_t high_cycles, uint32_t low_cycles)
{
    REG_SET_FIELD(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD, high_cycles);
    REG_SET_FIELD(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD, low_cycles);
}

void hulp_pwm_set(uint32_t clk_freq, uint32_t pwm_freq, float duty)
{
    uint32_t cycles = clk_freq / pwm_freq;
    uint32_t high_cycles = duty * cycles;
    uint32_t low_cycles = cycles - high_cycles;
    hulp_pwm_set_cycles(high_cycles, low_cycles);
}

uint8_t hulp_pwm_shift8(uint32_t clk_freq, uint32_t pwm_freq)
{
    uint32_t cycles = clk_freq / pwm_freq;
    uint8_t shift = 0;
    if(cycles > 255)
    {
        shift = (log2((clk_freq) / (pwm_freq)) + 1) - 8;
        if(shift > 12) shift = 12; // high/low cycles are 20 bits
    }
    return shift;
}

void hulp_pwm_set8(uint32_t clk_freq, uint32_t pwm_freq, uint8_t duty8)
{
    uint8_t shift = hulp_pwm_shift8(clk_freq, pwm_freq);
    uint32_t cycles = (clk_freq / pwm_freq) >> shift;
    uint8_t high_cycles = cycles * duty8 / 255;
    uint8_t low_cycles = cycles - high_cycles;
    hulp_pwm_set_cycles((uint32_t)high_cycles << shift, (uint32_t)low_cycles << shift);
}

ulp_insn_t I_PWM_SET8_H(uint32_t clk_freq, uint32_t pwm_freq, uint8_t duty8)
{
    uint8_t shift = hulp_pwm_shift8(clk_freq, pwm_freq);
    uint32_t cycles = (clk_freq / pwm_freq) >> shift;
    uint8_t high_cycles = cycles * duty8 / 255;
    return (ulp_insn_t)I_WR_REG(RTC_I2C_SCL_HIGH_PERIOD_REG, (uint8_t)(RTC_I2C_SCL_HIGH_PERIOD_S + shift), (uint8_t)(RTC_I2C_SCL_HIGH_PERIOD_S + shift + 7), high_cycles);
}
ulp_insn_t I_PWM_SET8_L(uint32_t clk_freq, uint32_t pwm_freq, uint8_t duty8)
{
    uint8_t shift = hulp_pwm_shift8(clk_freq, pwm_freq);
    uint32_t cycles = (clk_freq / pwm_freq) >> shift;
    uint8_t low_cycles = cycles - (cycles * duty8 / 255);
    return (ulp_insn_t)I_WR_REG(RTC_I2C_SCL_LOW_PERIOD_REG, (uint8_t)(RTC_I2C_SCL_LOW_PERIOD_S + shift), (uint8_t)(RTC_I2C_SCL_LOW_PERIOD_S + shift + 7), low_cycles);
}