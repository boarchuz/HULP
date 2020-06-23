#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_apa.h"

#define SCL_PIN GPIO_NUM_14
#define SDA_PIN GPIO_NUM_13

//Brightness is 5 bit (0-31)
#define BRIGHTNESS 5

void init_ulp()
{
    enum {
        LBL_RED_INTERVAL,
        LBL_RED_RETURN,
        LBL_BLUE_INTERVAL,
        LBL_BLUE_RETURN,
        LBL_GREEN_INTERVAL,
        LBL_GREEN_RETURN,
        LBL_YELLOW_INTERVAL,
        LBL_YELLOW_RETURN,
        LBL_WHITE_INTERVAL,
        LBL_WHITE_RETURN,

        LBL_HALT,

        LBL_APA_ENTRY,
    };

    const ulp_insn_t program[] = {
        M_UPDATE_TICKS(),
        
        M_IF_MS_ELAPSED(LBL_RED_INTERVAL, 1000, LBL_GREEN_INTERVAL),
            M_APA1_SET(LBL_RED_RETURN, LBL_APA_ENTRY, BRIGHTNESS, 255, 0, 0),

        M_IF_MS_ELAPSED(LBL_GREEN_INTERVAL, 1050, LBL_YELLOW_INTERVAL),
            M_APA1_SET(LBL_GREEN_RETURN, LBL_APA_ENTRY, BRIGHTNESS, 0, 255, 0),

        M_IF_MS_ELAPSED(LBL_YELLOW_INTERVAL, 1100, LBL_BLUE_INTERVAL),
            M_APA1_SET(LBL_YELLOW_RETURN, LBL_APA_ENTRY, BRIGHTNESS, 255, 255, 0),

        M_IF_MS_ELAPSED(LBL_BLUE_INTERVAL, 1150, LBL_WHITE_INTERVAL),
            M_APA1_SET(LBL_BLUE_RETURN, LBL_APA_ENTRY, BRIGHTNESS, 0, 0, 255),

        M_IF_MS_ELAPSED(LBL_WHITE_INTERVAL, 1200, LBL_HALT),
            M_APA1_SET(LBL_WHITE_RETURN, LBL_APA_ENTRY, BRIGHTNESS, 255, 255, 255),

        M_LABEL(LBL_HALT),
            I_HALT(),

        M_INCLUDE_APA1(LBL_APA_ENTRY, SCL_PIN, SDA_PIN),
    };

    hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_PULLUP_ONLY, 1);
    hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_PULLUP_ONLY, 1);

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    hulp_ulp_load(program, sizeof(program), 1ULL * 50 * 1000);
    hulp_ulp_run();
}

extern "C" void app_main()
{
    init_ulp();
    esp_deep_sleep_start();
}