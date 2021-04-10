#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_hall.h"

#define HALL_THRESHOLD 100

RTC_DATA_ATTR ulp_var_t ulp_adcvp0;
RTC_DATA_ATTR ulp_var_t ulp_adcvn0;
RTC_DATA_ATTR ulp_var_t ulp_adcvp1;
RTC_DATA_ATTR ulp_var_t ulp_adcvn1;
RTC_DATA_ATTR ulp_var_t ulp_adcvpdiff;
RTC_DATA_ATTR ulp_var_t ulp_adcvndiff;
RTC_DATA_ATTR ulp_var_t ulp_adcvdiff;

RTC_DATA_ATTR ulp_var_t ulp_adcbaseline;
RTC_DATA_ATTR ulp_var_t ulp_adcvbaselinediff;

void init_ulp()
{

    enum {
        LBL_HALT,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R2,0),

        I_ADC_POWER_ON(),

        I_HALL_POLARITY_FORWARD(),

        I_ADC(R0, 0, 0),
        I_PUT(R0, R2, ulp_adcvp0),
        I_ADC(R0, 0, 3),
        I_PUT(R0, R2, ulp_adcvn0),

        I_HALL_POLARITY_REVERSE(),

        I_ADC(R0, 0, 0),
        I_PUT(R0, R2, ulp_adcvp1),
        I_ADC(R0, 0, 3),
        I_PUT(R0, R2, ulp_adcvn1),

        I_ADC_POWER_OFF(),

        //+ diff
        I_GET(R0, R2, ulp_adcvp0),
        I_GET(R1, R2, ulp_adcvp1),
        I_SUBR(R0, R0, R1),
        I_PUT(R0, R2, ulp_adcvpdiff),

        //- diff
        I_GET(R0, R2, ulp_adcvn0),
        I_GET(R1, R2, ulp_adcvn1),
        I_SUBR(R0, R0, R1),
        I_PUT(R0, R2, ulp_adcvndiff),

        //diff
        I_GET(R0, R2, ulp_adcvpdiff),
        I_GET(R1, R2, ulp_adcvndiff),
        I_SUBR(R0, R0, R1),
        I_PUT(R0, R2, ulp_adcvdiff),

        I_GET(R1, R2, ulp_adcbaseline),
        I_SUBR(R0, R0, R1),
        I_SUBI(R1, R0, HALL_THRESHOLD),
        M_BXF(LBL_HALT),
        I_ADDI(R1, R0, HALL_THRESHOLD),
        M_BXF(LBL_HALT),
        I_PUT(R0, R2, ulp_adcvbaselinediff),
        I_WAKE(),

        M_LABEL(LBL_HALT),
            I_HALT(),
    };

    hulp_peripherals_on();

    hulp_configure_hall_effect_sensor();

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 50 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

extern "C" void app_main()
{
    if(hulp_is_ulp_wakeup())
    {
        printf("Woken: %d\n", (int16_t)ulp_adcvbaselinediff.val);
        printf("ADC: P: %u - %u (%d), N: %u - %u (%d), Diff: %d\n", ulp_adcvp0.val, ulp_adcvp1.val, (int16_t)ulp_adcvpdiff.val, ulp_adcvn0.val, ulp_adcvn1.val, (int16_t)ulp_adcvndiff.val, (int16_t)ulp_adcvdiff.val);
    }
    else
    {
        adc1_config_width( ADC_WIDTH_BIT_12);
        int baseline = hall_sensor_read();
        printf("Baseline: %d\n", baseline);
        ulp_adcbaseline.val = baseline;
        
        init_ulp();

        //Print some values for debugging
        TickType_t start_time = xTaskGetTickCount();
        while(xTaskGetTickCount() - start_time < (1000 / portTICK_PERIOD_MS))
        {
            printf("ADC: P: %u - %u (%d), N: %u - %u (%d), Diff: %d\n", ulp_adcvp0.val, ulp_adcvp1.val, (int16_t)ulp_adcvpdiff.val, ulp_adcvn0.val, ulp_adcvn1.val, (int16_t)ulp_adcvndiff.val, (int16_t)ulp_adcvdiff.val);
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
    }

    fflush(stdout);
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}