#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hulp.h"
#include "hulp_debug.h"

//A ulp_debug_bp_data_t struct in RTC memory:
RTC_DATA_ATTR ulp_debug_bp_data_t ulp_debug_data;

RTC_DATA_ATTR ulp_var_t ulp_some_counter;

void init_ulp()
{
    enum {
        LAB_BREAKPOINT_1 = 100,
        LAB_BREAKPOINT_2,
        LAB_DEBUG_ENTRY,
    };

    const ulp_insn_t program[] {

            I_ADDI(R3, R3, 1),

        //A breakpoint with a label:
        M_LABEL(LAB_BREAKPOINT_1), M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),
            
            M_UPDATE_TICKS(),
            I_RD_TICKS_REG(1),
        
        //Another one:
        M_LABEL(LAB_BREAKPOINT_2), M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),

            I_MOVI(R2, 0),
            I_GET(R1, R2, ulp_some_counter),
            I_SUBI(R1, R1, 3),
            I_PUT(R1, R2, ulp_some_counter),

        //A simple breakpoint without a label:
        M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),

            I_HALT(),

        //Breakpoint dependency:
        M_INCLUDE_DEBUG_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),
    };

    hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000);

    //Basic debugging initialisation:
    hulp_debug_bp_config_t debug_config = HULP_DEBUG_BP_CONFIG_DEFAULT(ulp_debug_data, program, sizeof(program));
    hulp_debug_bp_init(&debug_config);

    hulp_ulp_run();
}

extern "C" void app_main()
{
    init_ulp();

    vTaskDelay(portMAX_DELAY);
}