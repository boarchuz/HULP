#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_pwm.h"

static const char* TAG = "pwmtest";

// PWM output pin: (eg. connect LED)
#define PIN_CLOCK GPIO_NUM_4
// Ensure nothing connected to this:
#define PIN_DATA GPIO_NUM_15

#define PWM_FREQUENCY 5000

static RTC_DATA_ATTR uint8_t ulp_program_index = 0;
static RTC_DATA_ATTR ulp_var_t ulp_run_count;

static esp_err_t load_program_reg_continuous()
{
    // This ULP program will increment and decrement clock cycle registers for smooth 8-bit PWM pulses, sleeping 3s between each.
    static RTC_DATA_ATTR ulp_var_t decrementing;

#define ULP_PWM_CHANGE_INTERVAL_MS 5

    // Cycle counts are 20 bits, but using 8-bit PWM values for convenience here.
    // This function will return a sensible range for 8-bit settings @ approx the desired frequency.
    uint8_t reg_shift = hulp_pwm_shift8(hulp_get_fast_clk_freq(), PWM_FREQUENCY);
    uint16_t max_run_count = 5;

    enum {
        LBL_LOOP,
        LBL_CHECK_INTERVAL,
        LBL_DECREMENTING,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R3, 0),
        I_PUT(R3, R3, decrementing),

        I_PWM_ATTACH(PIN_CLOCK),

        M_LABEL(LBL_LOOP),
        M_UPDATE_TICKS(),
        M_IF_MS_ELAPSED(LBL_CHECK_INTERVAL, ULP_PWM_CHANGE_INTERVAL_MS, LBL_LOOP),
        
        I_GET(R0, R3, decrementing),
        M_BGE(LBL_DECREMENTING, 1),

        // Incrementing
        M_REG_INC(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD_S + reg_shift),
        M_REG_DEC(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD_S + reg_shift),
        // Check if low period has reached 0 (ie. ~100% duty)
        I_RD_REG(RTC_I2C_SCL_LOW_PERIOD_REG, (uint8_t)(RTC_I2C_SCL_LOW_PERIOD_S + reg_shift), (uint8_t)(RTC_I2C_SCL_LOW_PERIOD_S + reg_shift + 7)),
        M_BGE(LBL_LOOP, 1),
        // Now switch to decrementing.
        I_MOVI(R0, 1),
        I_PUT(R0, R3, decrementing),
        M_BX(LBL_LOOP),

        // Decrementing
        M_LABEL(LBL_DECREMENTING),
        M_REG_INC(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD_S + reg_shift),
        M_REG_DEC(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD_S + reg_shift),
        // Check if high period has reached 0 (ie. ~0% duty)
        I_RD_REG(RTC_I2C_SCL_HIGH_PERIOD_REG, (uint8_t)(RTC_I2C_SCL_HIGH_PERIOD_S + reg_shift), (uint8_t)(RTC_I2C_SCL_HIGH_PERIOD_S + reg_shift + 7)),
        M_BGE(LBL_LOOP, 1),
        // Done
        I_PWM_DETACH(PIN_CLOCK),
        I_GET(R0, R3, ulp_run_count),
        I_ADDI(R0, R0, 1),
        I_PUT(R0, R3, ulp_run_count),
        I_BL(3, max_run_count),
        I_END(),
        I_WAKE(),
        I_HALT(),
    };

    // Prepare registers with the base values
    hulp_pwm_set8(hulp_get_fast_clk_freq(), PWM_FREQUENCY, 0);

    return hulp_ulp_load(program, sizeof(program), 3 * 1000 * 1000, 0);
}

static esp_err_t load_program_discrete()
{
    // This ULP program will set a particular PWM duty each time it wakes. 
    // Could by any values, but basically a low-res pulse here.

    // note that this sleeps between PWM changes, breaking PWM clock frequency. Not noticeable with LED though.
    // For stable frequency, ensure ULP stays awake while PWM active (as in previous program)

    static RTC_DATA_ATTR ulp_var_t pwm_stage = {0};

    uint32_t fast_clk_freq = hulp_get_fast_clk_freq();
    uint16_t max_run_count = 5;

    enum {
        LBL_BRANCH_ROOT,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R3, 0),

        I_GET(R0, R3, pwm_stage),
        M_MOVL(R1, LBL_BRANCH_ROOT),
        I_ADDR(R1, R0, R1),
        I_ADDI(R0, R0, 3),
        I_PUT(R0, R3, pwm_stage),
        I_BXR(R1),

        M_LABEL(LBL_BRANCH_ROOT),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 0),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 31),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 63),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 95),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 127),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 159),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 191),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 223),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 255),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 127),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 255),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 127),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 255),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 127),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 255),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 223),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 191),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 159),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 127),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 95),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 63),
            I_HALT(),
            M_PWM_SET8(fast_clk_freq, PWM_FREQUENCY, 31),
            I_PUT(R3, R3, pwm_stage), //Reset
            I_GET(R0, R3, ulp_run_count),
            I_ADDI(R0, R0, 1),
            I_PUT(R0, R3, ulp_run_count),
            I_BL(3, max_run_count),
            I_END(),
            I_WAKE(),
            I_HALT(),
    };

    pwm_stage.val = 0;
    return hulp_ulp_load(program, sizeof(program), 250 * 1000, 0);
}

static void pulse_pwm()
{
    uint32_t fast_clk_freq = hulp_get_fast_clk_freq();
    for(int i = 0; i <= 255; ++i)
    {
        hulp_pwm_set8(fast_clk_freq, PWM_FREQUENCY, i);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    for(int i = 254; i >= 0; --i)
    {
        hulp_pwm_set8(fast_clk_freq, PWM_FREQUENCY, i);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    esp_err_t (*ulp_program_loaders[])() = {load_program_reg_continuous, load_program_discrete};

    hulp_pwm_configure(PIN_CLOCK, PIN_DATA);
    hulp_pwm_start();
    
    ESP_LOGI(TAG, "SoC Pulsing");
    for(int i = 0; i < 1; ++i)
    {
        pulse_pwm();
    }

    ESP_LOGI(TAG, "Loading ULP program %u", ulp_program_index);
    ulp_program_loaders[ulp_program_index]();
    ulp_run_count.val = 0;
    ulp_run(0);

    //Increment ULP program index for next wakeup
    ulp_program_index = (ulp_program_index + 1) % (sizeof(ulp_program_loaders) / sizeof(ulp_program_loaders)[0]);

    hulp_peripherals_on();
    esp_sleep_enable_ulp_wakeup();

    // for some reason, pwm is broken if immediately sleep here...
    
    // eg. pin_clock output becomes basically digital
    // and/or clock frequency is very low (150khz?) 
    // and/or lots of jitter (switching 8mhz/150khz?)

    // setting RTC_CNTL_ANA_CONF_REG->RTC_CNTL_TXRF_I2C_PU works, but much higher deep sleep current and may break other stuff (eg. rf)

    // waiting until the ULP is asleep before deep sleeping works for demonstration but not a great solution
    while(hulp_get_state() != ULP_STATE_SLEEPING) ets_delay_us(1);

    ESP_LOGI(TAG, "Sleeping");
    esp_deep_sleep_start();
}