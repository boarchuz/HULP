#include "hulp.h"

#include <string.h>
#include <math.h>

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
extern "C" {
    #include "esp_clk.h"
}
extern "C" {
	#include "driver/rtc_cntl.h"
}
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"
#include "soc/syscon_reg.h"

#include "hulp_ulp.h"

#include "sdkconfig.h"

static const char* TAG = "HULP";

const gpio_num_t rtc_to_gpio[18] = {GPIO_NUM_36,GPIO_NUM_37,GPIO_NUM_38,GPIO_NUM_39,GPIO_NUM_34,GPIO_NUM_35,GPIO_NUM_25,GPIO_NUM_26,GPIO_NUM_33,GPIO_NUM_32,GPIO_NUM_4,GPIO_NUM_0,GPIO_NUM_2,GPIO_NUM_15,GPIO_NUM_13,GPIO_NUM_12,GPIO_NUM_14,GPIO_NUM_27};

esp_err_t hulp_configure_pin(gpio_num_t pin, rtc_gpio_mode_t mode, bool pullup, bool pulldown, uint32_t level)
{
    esp_err_t err = rtc_gpio_init(pin);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "rtcio (gpio %d) init failed", pin);
        return err;
    }
    if(pullup)
    {
        rtc_gpio_pullup_en(pin);
    }
    else
    {
        rtc_gpio_pullup_dis(pin);
    }
    if(pulldown)
    {
        rtc_gpio_pulldown_en(pin);
    }
    else
    {
        rtc_gpio_pulldown_dis(pin);
    }
    rtc_gpio_set_level(pin, level);
    rtc_gpio_set_direction(pin, mode);
    return ESP_OK;
}

static esp_err_t hulp_adc1_get_channel(gpio_num_t pin, adc1_channel_t *channel)
{
    gpio_num_t search_pin;
    for(uint8_t i = ADC1_CHANNEL_0; i < ADC1_CHANNEL_MAX; ++i)
    {
        if (adc1_pad_get_io_num((adc1_channel_t)i, &search_pin) == ESP_OK && pin == search_pin)
        {
            if(channel)
            {
                *channel = (adc1_channel_t)i;
            }
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_ARG;
}

static esp_err_t hulp_adc2_get_channel(gpio_num_t pin, adc2_channel_t* channel)
{
    gpio_num_t search_pin;
    for(uint8_t i = ADC2_CHANNEL_0; i < ADC2_CHANNEL_MAX; ++i)
    {
        if (adc2_pad_get_io_num((adc2_channel_t)i, &search_pin) == ESP_OK && pin == search_pin)
        {
            if(channel)
            {
                *channel = (adc2_channel_t)i;
            }
            return ESP_OK;
        }
    }
    return ESP_ERR_INVALID_ARG;
}

adc_unit_t hulp_adc_get_unit(gpio_num_t pin)
{
    //Must return value as used directly in ULP macros
    esp_err_t err = hulp_adc1_get_channel(pin, nullptr);
    if(err == ESP_OK)
    {
        return ADC_UNIT_1;
    }
    err = hulp_adc2_get_channel(pin, nullptr);
    if(err == ESP_OK)
    {
        return ADC_UNIT_2;
    }
    ESP_LOGE(TAG, "no adc unit for pin %d", pin);
    return ADC_UNIT_MAX;
}

uint8_t hulp_adc_get_channel_num(gpio_num_t pin)
{
    //Must return value as used directly in ULP macros
    esp_err_t err;
    union {
        adc1_channel_t one;
        adc2_channel_t two;
    } adc_channel;

    err = hulp_adc1_get_channel(pin, &(adc_channel.one));
    if(err == ESP_OK)
    {
        return (uint8_t)adc_channel.one;
    }
    err = hulp_adc2_get_channel(pin, &(adc_channel.two));
    if(err == ESP_OK)
    {
        return (uint8_t)adc_channel.two;
    }
    ESP_LOGE(TAG, "no adc channel for pin %d", pin);
    return -1;
}

esp_err_t hulp_configure_analog_pin(gpio_num_t pin, adc_atten_t attenuation, adc_bits_width_t width)
{
    adc_unit_t adc_unit = hulp_adc_get_unit(pin);
    if(adc_unit == ADC_UNIT_1)
    {
        adc1_channel_t channel;
        hulp_adc1_get_channel(pin, &channel);
        adc1_config_channel_atten(channel, attenuation); //Does adc_gpio_init() internally
        adc1_config_width(width);
        adc1_ulp_enable();
        return ESP_OK;
    }
    else if(adc_unit == ADC_UNIT_2)
    {
        //Not sure how much of this is necessary. Probably only needs to be inverted.
        adc2_channel_t channel;
        hulp_adc2_get_channel(pin, &channel);
        adc2_config_channel_atten(channel, attenuation); //Does adc2_pad_init() internally
        //Do a read in order to set some regs
        int adc2val;
        adc2_get_raw(channel, width, &adc2val);
        //Equivalent of adc_set_controller( ADC_UNIT_2, ADC_CTRL_ULP ) : (unsafe?)
        REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE);
        REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE);
        REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);
        REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_PWDET_FORCE);
        REG_SET_BIT(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_MUX);
        return ESP_OK;
    }
    ESP_LOGE(TAG, "unknown ADC unit for pin %d", pin);
    return ESP_ERR_INVALID_ARG;
}

esp_err_t hulp_configure_i2c_pins(gpio_num_t scl_pin, gpio_num_t sda_pin, bool scl_pullup, bool sda_pullup)
{
    if( !(scl_pin == GPIO_NUM_2 || scl_pin == GPIO_NUM_4) )
    {
        ESP_LOGE(TAG, "invalid i2c hw SCL pin, must be 2 or 4");
        return ESP_ERR_INVALID_ARG;
    }
    if( !(sda_pin == GPIO_NUM_0 || sda_pin == GPIO_NUM_15) )
    {
        ESP_LOGE(TAG, "invalid i2c hw SDA pin, must be 0 or 15");
        return ESP_ERR_INVALID_ARG;
    }

    //SCL
    rtc_gpio_init(scl_pin);
	rtc_gpio_set_level(scl_pin, 0);
	rtc_gpio_pulldown_dis(scl_pin); //GPIO_NUM_2 and GPIO_NUM_4 have default pulldown
	if(scl_pullup)
	{
		rtc_gpio_pullup_en(scl_pin);
	}
	else
	{
		rtc_gpio_pullup_dis(scl_pin);
	}
    rtc_gpio_set_direction(scl_pin, RTC_GPIO_MODE_INPUT_OUTPUT);
    
    //SDA
    rtc_gpio_init(sda_pin);
	rtc_gpio_set_level(sda_pin, 0);
	rtc_gpio_pulldown_dis(sda_pin);
	if(sda_pullup)
	{
		rtc_gpio_pullup_en(sda_pin);
	}
	else
	{
		rtc_gpio_pullup_dis(sda_pin);
	}
    rtc_gpio_set_direction(sda_pin, RTC_GPIO_MODE_INPUT_OUTPUT);

    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SCL_SEL, scl_pin == GPIO_NUM_4 ? 0 : 1);
    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SDA_SEL, sda_pin == GPIO_NUM_0 ? 0 : 1);

    //RTC I2C FUN_SEL = 3
    SET_PERI_REG_BITS(rtc_gpio_desc[scl_pin].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_gpio_desc[scl_pin].func);
    SET_PERI_REG_BITS(rtc_gpio_desc[sda_pin].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, 0x3, rtc_gpio_desc[sda_pin].func);
    return ESP_OK;
}

void hulp_configure_i2c_controller(uint32_t scl_low, uint32_t scl_high, uint32_t sda_duty, uint32_t scl_start, uint32_t scl_stop, uint32_t timeout, bool scl_pushpull, bool sda_pushpull, bool rx_lsbfirst, bool tx_lsbfirst)
{
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_RX_LSB_FIRST, rx_lsbfirst ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_TX_LSB_FIRST, tx_lsbfirst ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT, scl_pushpull ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SDA_FORCE_OUT, sda_pushpull ? 1 : 0);

    REG_SET_FIELD(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD, scl_low);
    REG_SET_FIELD(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD, scl_high);
    REG_SET_FIELD(RTC_I2C_SDA_DUTY_REG, RTC_I2C_SDA_DUTY, sda_duty);
    REG_SET_FIELD(RTC_I2C_SCL_START_PERIOD_REG, RTC_I2C_SCL_START_PERIOD, scl_start);
    REG_SET_FIELD(RTC_I2C_SCL_STOP_PERIOD_REG, RTC_I2C_SCL_STOP_PERIOD, scl_stop);
    REG_SET_FIELD(RTC_I2C_TIMEOUT_REG, RTC_I2C_TIMEOUT, timeout);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE, 1);
}

esp_err_t hulp_register_i2c_slave(uint8_t index, uint8_t address)
{
    if(index > 7)
    {
        ESP_LOGE(TAG, "invalid i2c slave index (%u), range 0-7", index);
        return ESP_ERR_INVALID_ARG;
    }
    SET_PERI_REG_BITS(SENS_SAR_SLAVE_ADDR1_REG + (index / 2) * 4, 0x000007FF, address, (index % 2) ? 0 : 11);
    return ESP_OK;
}

void hulp_tsens_configure(uint8_t clk_div)
{
    REG_SET_FIELD(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, clk_div);
    REG_SET_FIELD(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, SENS_FORCE_XPD_SAR_PU);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
}

void hulp_peripherals_on()
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}
void hulp_configure_hall_effect_sensor()
{
    //GPIO 36
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
    //GPIO 39
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();
    REG_SET_BIT(SENS_SAR_TOUCH_CTRL1_REG, SENS_HALL_PHASE_FORCE);
    REG_SET_BIT(SENS_SAR_TOUCH_CTRL1_REG, SENS_XPD_HALL_FORCE);
    //Connect sensor to 36 and 39
    REG_SET_BIT(RTC_IO_HALL_SENS_REG, RTC_IO_XPD_HALL);
}

void hulp_clear_program_memory()
{
    memset(RTC_SLOW_MEM, 0, ULP_RESERVE_MEM);
}

void hulp_clear_rtc_slow_memory()
{
    memset(RTC_SLOW_MEM, 0, 0x1000);
}

static uint64_t hulp_us_to_ticks(uint64_t time_us)
{
    return rtc_time_us_to_slowclk(time_us, esp_clk_slowclk_cal_get());
}

uint16_t hulp_ms_to_ulp_ticks_with_shift(uint32_t time_ms, uint8_t shift)
{
    return (uint16_t)((hulp_us_to_ticks(1000ULL * time_ms) >> shift) & 0xFFFF);
}

uint16_t hulp_ms_to_ulp_ticks(uint32_t time_ms)
{
    return hulp_ms_to_ulp_ticks_with_shift(time_ms, hulp_ms_to_ulp_tick_shift(time_ms));
}

uint16_t hulp_get_current_ulp_ticks(uint8_t shift)
{
    return (uint16_t)((rtc_time_get() >> shift) & 0xFFFF);
}

uint8_t hulp_ms_to_ulp_tick_shift(uint32_t time_ms)
{
    uint64_t rtc_slow_ticks = hulp_us_to_ticks(1000ULL * time_ms);
    uint8_t high_bit = log2(rtc_slow_ticks);
    if(high_bit >= 32)
    {
        high_bit = 47;  // [47:32]
    }
    else if(high_bit < 16)
    {
        high_bit = 16; // if high_bit < 16, get all lower 16 bits. Note: tick count is updated every 2 ticks, so bit 0 is not interesting, therefore [16:1] rather than [15:0]
    } // else 32 > high_bit > 16. ie. [31:16] - [16:1]
    ESP_LOGV(TAG, "%s ms: %u, ticks: %llu, range: [%u:%u], overflow: %llu ms, resolution: %llu uS", __func__, time_ms, rtc_slow_ticks, high_bit, high_bit - 15, rtc_time_slowclk_to_us((1ULL << (high_bit+1))-1,esp_clk_slowclk_cal_get()) / 1000, rtc_time_slowclk_to_us(1ULL << (high_bit-15),esp_clk_slowclk_cal_get()));
    return (high_bit - 15);
}

uint16_t hulp_get_label_pc(uint16_t label, const ulp_insn_t *program)
{
    size_t counter = 0;
    while(counter < (0x1000 / sizeof(ulp_insn_t))) //Unsafe but simple
    {
        if (program->macro.opcode == OPCODE_MACRO)
        {
            if(program->macro.sub_opcode == SUB_OPCODE_MACRO_LABEL && program->macro.label == label)
            {
                return counter;
            }
        }
        else
        {
            ++counter;
        }
        ++program;
    }
    assert(false && "label not found");
}

esp_err_t hulp_ulp_run(uint32_t entry_point)
{
    return ulp_run(entry_point);
}

esp_err_t hulp_ulp_run_once(uint32_t entry_point)
{
    // disable ULP timer
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    // wait for at least 1 RTC_SLOW_CLK cycle
    ets_delay_us(10);
    // set entry point
    REG_SET_FIELD(SENS_SAR_START_FORCE_REG, SENS_PC_INIT, entry_point);
    // enable SW start
    SET_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_FORCE_START_TOP);
    // make sure voltage is raised when RTC 8MCLK is enabled
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_I2C_FOLW_8M);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_CORE_FOLW_8M);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_SLEEP_FOLW_8M);
    // start
    CLEAR_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_START_TOP);
    SET_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_START_TOP);
    return ESP_OK;
}

esp_err_t hulp_ulp_load(const ulp_insn_t *program, size_t size_of_program, uint32_t period_us, uint32_t entry_point)
{
    size_t num_words = size_of_program / sizeof(ulp_insn_t);
    esp_err_t err = ulp_process_macros_and_load(entry_point, program, &num_words);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to load program");
        return err;
    }
    ulp_set_wakeup_period(0, period_us);
    return ESP_OK;
}

void hulp_ulp_end()
{
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
}

bool hulp_is_deep_sleep_wakeup()
{
    return (esp_reset_reason() == ESP_RST_DEEPSLEEP);
}

bool hulp_is_ulp_wakeup()
{
    return (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP);
}

esp_err_t hulp_ulp_isr_register(intr_handler_t handler, void* handler_arg)
{
    esp_err_t err = rtc_isr_register(handler, handler_arg, RTC_CNTL_SAR_INT_ST_M);
    if(err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t hulp_ulp_isr_deregister(intr_handler_t handler, void* handler_arg)
{
    return rtc_isr_deregister(handler, handler_arg);
}

void hulp_ulp_interrupt_en()
{
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
}

void hulp_ulp_interrupt_dis()
{
    REG_CLR_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
}