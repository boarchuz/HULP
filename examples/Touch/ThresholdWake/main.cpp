#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_touch.h"

#define PIN_TOUCH GPIO_NUM_32

#define ULP_TOUCH_INTERVAL_MS 200
//The read value will be printed out for a few seconds on reset/wake. Adjust this threshold accordingly:
#define ULP_TOUCH_WAKEUP_THRESHOLD 250

RTC_DATA_ATTR ulp_var_t ulp_touch_val;

void init_ulp()
{
    enum {
        LBL_TOUCH_INTERVAL,
        LBL_HALT,
    };

    const ulp_insn_t program[] = {

        M_UPDATE_TICKS(),

        M_IF_MS_ELAPSED(LBL_TOUCH_INTERVAL, ULP_TOUCH_INTERVAL_MS, LBL_HALT),
            //Begin a new touch read
            M_TOUCH_BEGIN(),
            //Wait for it to complete
            M_TOUCH_WAIT_DONE(),
            //Get the value for selected pin
            I_TOUCH_GET_GPIO_VALUE(PIN_TOUCH),
            //Update the variable with the new value
            I_MOVI(R2, 0),
            I_PUT(R0, R2, ulp_touch_val),
            //If it's higher than threshold, go to halt; else trigger a wake
            M_BGE(LBL_HALT, ULP_TOUCH_WAKEUP_THRESHOLD),
                I_WAKE(),

        M_LABEL(LBL_HALT),
            I_HALT(),
    };

    hulp_configure_touch_controller();
    hulp_configure_touch_pin(PIN_TOUCH);

    hulp_ulp_load(program, sizeof(program), 1ULL * 10 * 1000);
    hulp_ulp_run();
}

extern "C" void app_main()
{

    if(!hulp_is_ulp_wakeup())
    {
        init_ulp();
    }
    
    TickType_t start_time = xTaskGetTickCount();
    while(xTaskGetTickCount() - start_time < (5000 / portTICK_PERIOD_MS))
    {
        printf("ulp_touch_val: %u\n", ulp_touch_val.val);
        vTaskDelay(ULP_TOUCH_INTERVAL_MS / portTICK_PERIOD_MS);
    }

    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}