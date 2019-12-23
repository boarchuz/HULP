#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/rtc.h"

#include "hulp.h"


RTC_DATA_ATTR ulp_var_t ulp_tsens_val;

void startulp()
{
    const ulp_insn_t program[]{
        I_TSENS(R0, 1000),
        I_MOVI(R2,0),
        I_PUT(R0, R2, ulp_tsens_val),
        I_HALT(),
    };

    hulp_tsens_configure();
    hulp_start(program, sizeof(program), 1ULL * 1000 * 1000);
}

extern "C" void app_main()
{
    startulp();
    
    for(;;)
    {
        if(ulp_tsens_val.updated())
        {
            ulp_tsens_val.clearUpdated();
            printf("TSENS: %u\n", ulp_tsens_val.get());
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}