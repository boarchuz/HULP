#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

#include "hulp.h"

#define PIN_LED1 GPIO_NUM_12
#define LED1_TOGGLE_MS 250
#define PIN_LED2 GPIO_NUM_13
#define LED2_TOGGLE_MS 500
#define PIN_LED3 GPIO_NUM_14
#define LED3_TOGGLE_MS 1000
#define PIN_LED4 GPIO_NUM_15
#define LED4_TOGGLE_MS 5000

void init_ulp()
{
    enum {
        LBL_INTERVAL1,
        LBL_INTERVAL2,
        LBL_INTERVAL3,
        LBL_INTERVAL4,
        LBL_HALT,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R2,0),

        M_UPDATE_TICKS(),
        
        M_IF_MS_ELAPSED(LBL_INTERVAL1,LED1_TOGGLE_MS,LBL_INTERVAL2),
            M_GPIO_TOGGLE(PIN_LED1),

        M_IF_MS_ELAPSED(LBL_INTERVAL2,LED2_TOGGLE_MS,LBL_INTERVAL3),
            M_GPIO_TOGGLE(PIN_LED2),

        M_IF_MS_ELAPSED(LBL_INTERVAL3,LED3_TOGGLE_MS,LBL_INTERVAL4),
            M_GPIO_TOGGLE(PIN_LED3),

        M_IF_MS_ELAPSED(LBL_INTERVAL4,LED4_TOGGLE_MS,LBL_HALT),
            M_GPIO_TOGGLE(PIN_LED4),

        M_LABEL(LBL_HALT),
            I_HALT(),
    };

    ESP_ERROR_CHECK(hulp_configure_pin(PIN_LED1, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_LED2, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_LED3, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_LED4, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));

    hulp_peripherals_on();

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1ULL * 10 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

extern "C" void app_main()
{
    init_ulp();
    esp_deep_sleep_start();
}