#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

#include "hulp.h"

#define MEAS_INTERVAL_MS 1000

RTC_DATA_ATTR ulp_var_t ulp_tsens_val;

void init_ulp()
{
    const ulp_insn_t program[] = {
        I_TSENS(R0, 1000),
        I_MOVI(R2,0),
        I_PUT(R0, R2, ulp_tsens_val),
        I_HALT(),
    };

    hulp_tsens_configure();
    hulp_ulp_load(program, sizeof(program), MEAS_INTERVAL_MS * 1000, 0);
    hulp_ulp_run();
}

extern "C" void app_main()
{
    init_ulp();
    
    for(;;)
    {
        vTaskDelay(MEAS_INTERVAL_MS / portTICK_PERIOD_MS);
        printf("TSENS: %u\n", ulp_tsens_val.val);
    }
}