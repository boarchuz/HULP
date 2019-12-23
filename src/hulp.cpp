#include "hulp.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"

#ifdef HULP_NO_PATCH
#include "esp32/ulp.h"
#else
#include "cust_ulp.h"
#endif

#include "esp_sleep.h"

#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"

#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"
#include "soc/syscon_reg.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include <string.h>
#include "math.h"
extern "C" {
    #include "esp_clk.h"
}

#include "soc/touch_channel.h"

#include "hulp_touch.h"

static const char* TAG = "HULP";

const gpio_num_t rtc_to_gpio[18] = {GPIO_NUM_36,GPIO_NUM_37,GPIO_NUM_38,GPIO_NUM_39,GPIO_NUM_34,GPIO_NUM_35,GPIO_NUM_25,GPIO_NUM_26,GPIO_NUM_33,GPIO_NUM_32,GPIO_NUM_4,GPIO_NUM_0,GPIO_NUM_2,GPIO_NUM_15,GPIO_NUM_13,GPIO_NUM_12,GPIO_NUM_14,GPIO_NUM_27};

void hulp_configure_pin(gpio_num_t pin, rtc_gpio_mode_t mode, bool pullup, bool pulldown, uint32_t level)
{
    ESP_ERROR_CHECK(rtc_gpio_init(pin));
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
}

adc1_channel_t hulp_adc1_get_channel(gpio_num_t pin)
{
    gpio_num_t search_pin;
    for(uint8_t i = ADC1_CHANNEL_0; i < ADC1_CHANNEL_MAX; i++)
    {
        if (adc1_pad_get_io_num((adc1_channel_t)i, &search_pin) == ESP_OK && pin == search_pin)
        {
            return (adc1_channel_t)i;
        }
    }
    return ADC1_CHANNEL_MAX;
}

adc2_channel_t hulp_adc2_get_channel(gpio_num_t pin)
{
    gpio_num_t search_pin;
    for(uint8_t i = ADC2_CHANNEL_0; i < ADC2_CHANNEL_MAX; ++i)
    {
        if (adc2_pad_get_io_num((adc2_channel_t)i, &search_pin) == ESP_OK && pin == search_pin)
        {
            return (adc2_channel_t)i;
        }
    }
    return ADC2_CHANNEL_MAX;
}

adc_unit_t hulp_adc_get_unit(gpio_num_t pin)
{
    if(hulp_adc1_get_channel(pin) < ADC1_CHANNEL_MAX)
    {
        return ADC_UNIT_1;
    }
    if(hulp_adc2_get_channel(pin) < ADC2_CHANNEL_MAX)
    {
        return ADC_UNIT_2;
    }
    assert (false && "ADC invalid");
}

uint8_t hulp_adc_get_channel_num(gpio_num_t pin)
{
    adc1_channel_t adc1channel;
    adc1channel = hulp_adc1_get_channel(pin);
    if(adc1channel < ADC1_CHANNEL_MAX)
    {
        return adc1channel;
    }
    adc2_channel_t adc2channel;
    adc2channel = hulp_adc2_get_channel(pin);
    if(adc2channel < ADC2_CHANNEL_MAX)
    {
        return adc2channel;
    }
    assert (false && "ADC invalid");
}

void hulp_configure_analog_pin(gpio_num_t pin, adc_atten_t attenuation, adc_bits_width_t width)
{
    adc_unit_t adc_unit = hulp_adc_get_unit(pin);
    if(adc_unit == ADC_UNIT_1)
    {
        adc1_config_channel_atten(hulp_adc1_get_channel(pin), attenuation); //Does adc_gpio_init() internally
        adc1_config_width(width);
        adc1_ulp_enable();
        return;
    }
    else if(adc_unit == ADC_UNIT_2)
    {
        //Not sure how much of this is necessary. Probably only needs to be inverted.
        adc2_config_channel_atten(hulp_adc2_get_channel(pin), attenuation); //Does adc2_pad_init() internally
        //Do a read in order to set some regs
        int adc2val;
        adc2_get_raw(hulp_adc2_get_channel(pin), width, &adc2val);
        //Equivalent of adc_set_controller( ADC_UNIT_2, ADC_CTRL_ULP ) : (unsafe?)
        REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE);
        REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE);
        REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);
        REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_PWDET_FORCE);
        REG_SET_BIT(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_MUX);
    }
}

void hulp_configure_i2c_pins(gpio_num_t scl_pin, gpio_num_t sda_pin, bool scl_pullup, bool sda_pullup)
{
    assert((scl_pin == GPIO_NUM_2 || scl_pin == GPIO_NUM_4) && "SCL invalid");
    assert((sda_pin == GPIO_NUM_0 || sda_pin == GPIO_NUM_15) && "SDA invalid");

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
}

void hulp_configure_i2c_controller(uint32_t scl_low, uint32_t scl_high, uint32_t sda_duty, uint32_t scl_start, uint32_t scl_stop, uint32_t timeout, bool scl_pushpull, bool sda_pushpull, bool rx_lsbfirst, bool tx_lsbfirst)
{
    REG_SET_BIT(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_RX_LSB_FIRST, rx_lsbfirst ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_TX_LSB_FIRST, tx_lsbfirst ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT, scl_pushpull ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SDA_FORCE_OUT, sda_pushpull ? 1 : 0);
	REG_WRITE(RTC_I2C_SCL_HIGH_PERIOD_REG, scl_high);
    REG_WRITE(RTC_I2C_SCL_LOW_PERIOD_REG, scl_low);
	REG_WRITE(RTC_I2C_SDA_DUTY_REG, sda_duty);
	REG_WRITE(RTC_I2C_SCL_START_PERIOD_REG, scl_start);
	REG_WRITE(RTC_I2C_SCL_STOP_PERIOD_REG, scl_stop);
	REG_WRITE(RTC_I2C_TIMEOUT_REG, timeout);
}

void hulp_register_i2c_slave(uint8_t index, uint8_t address)
{
    SET_PERI_REG_BITS(SENS_SAR_SLAVE_ADDR1_REG + (index / 2) * 4, 0x000007FF, address, (index % 2) ? 0 : 11);
}

void hulp_tsens_configure(uint8_t clk_div)
{
    REG_SET_FIELD(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, clk_div);
    REG_SET_FIELD(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, SENS_FORCE_XPD_SAR_PU);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
}

void hulp_configure_touch(gpio_num_t touch_gpio)
{
    touch_pad_t pad = gpio_to_touch_num[touch_gpio];
    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    touch_pad_io_init(pad);
    touch_pad_set_cnt_mode(pad, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_LOW );
    touch_pad_set_group_mask(0,0,1<<pad);
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
    memset(RTC_SLOW_MEM, 0, CONFIG_ESP32_ULP_COPROC_RESERVE_MEM);
}

void hulp_clear_rtc_slow_memory()
{
    memset(RTC_SLOW_MEM, 0, 0x1000);
}

uint16_t hulp_ticks(uint32_t time_ms)
{
#ifdef HULP_DEBUG
    printf("Ticks: %u\n", (uint16_t)((rtc_time_us_to_slowclk(1000ULL * time_ms,esp_clk_slowclk_cal_get()) >> hulp_tick_shift(time_ms)) & 0xFFFF));
#endif
    return (uint16_t)((rtc_time_us_to_slowclk(1000ULL * time_ms, esp_clk_slowclk_cal_get()) >> hulp_tick_shift(time_ms)) & 0xFFFF);
}

uint16_t hulp_ticks_ref(uint32_t time_ms, uint32_t reference_ms)
{
#ifdef HULP_DEBUG
    printf("TickR: %u\n", (uint16_t)((rtc_time_us_to_slowclk(1000ULL * time_ms,esp_clk_slowclk_cal_get()) >> hulp_tick_shift(reference_ms)) & 0xFFFF));
#endif
    return (uint16_t)((rtc_time_us_to_slowclk(1000ULL * time_ms,esp_clk_slowclk_cal_get()) >> hulp_tick_shift(reference_ms)) & 0xFFFF);
}

uint8_t hulp_tick_shift(uint32_t time_ms)
{
    uint64_t ticks = rtc_time_us_to_slowclk(1000ULL * time_ms, esp_clk_slowclk_cal_get());
    uint8_t high_bit = log2(ticks);
    if(high_bit > 31)
    {
#ifdef HULP_DEBUG
        printf("Tick shift for %ums: %llu ticks, final range: [%u:%u], overflow: %llu ms, resolution: %llu uS\n", time_ms, ticks, 47, 47-15, rtc_time_slowclk_to_us((1ULL << (47+1))-1,esp_clk_slowclk_cal_get()) / 1000, rtc_time_slowclk_to_us(1ULL << (47-15),esp_clk_slowclk_cal_get()));
#endif
        return 32; // [47:32]
    }
    else if(high_bit < 16)
    {
#ifdef HULP_DEBUG
        printf("Tick shift for %ums: %llu ticks, final range: [%u:%u], overflow: %llu ms, resolution: %llu uS\n", time_ms, ticks, 16, 16-15, rtc_time_slowclk_to_us((1ULL << (16+1))-1,esp_clk_slowclk_cal_get()) / 1000, rtc_time_slowclk_to_us(1ULL << (16-15),esp_clk_slowclk_cal_get()));
#endif
        return 1; // [16:1]
    }
#ifdef HULP_DEBUG
    printf("Tick shift for %ums: %llu ticks, final range: [%u:%u], overflow: %llu ms, resolution: %llu uS\n", time_ms, ticks, high_bit, high_bit-15, rtc_time_slowclk_to_us((1ULL << (high_bit+1))-1,esp_clk_slowclk_cal_get()) / 1000, rtc_time_slowclk_to_us(1ULL << (high_bit-15),esp_clk_slowclk_cal_get()));
#endif
    return high_bit-15;
}

uint16_t hulp_label(uint16_t label, const ulp_insn_t *program)
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

void hulp_run_once(uint32_t entry_point)
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
}

void hulp_start(const ulp_insn_t *program, size_t size_of_program, uint32_t period_us, uint32_t entry_point)
{
    size_t num_words = size_of_program / sizeof(ulp_insn_t);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(entry_point, program, &num_words));
    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, period_us));
    ESP_ERROR_CHECK(ulp_run(entry_point));
}

void hulp_end()
{
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
}