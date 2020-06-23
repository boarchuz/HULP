#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hulp.h"
#include "hulp_debug.h"

//Debugging requires a ulp_debug_bp_data_t in RTC memory:
RTC_DATA_ATTR ulp_debug_bp_data_t ulp_debug_data;

enum {
    LAB_BREAKPOINT_1 = 100,
    LAB_BREAKPOINT_2,
    LAB_BREAKPOINT_3,
    LAB_BREAKPOINT_4,
    LAB_SOMEWHERE_ELSE,
    LAB_FINISHED,
    LAB_DEBUG_ENTRY,
};

void ulp_breakpoint_handler(hulp_debug_bp_cb_data_t* bp_data, void*)
{
    //This is an ISR. No lengthy operations, logging, printf, etc.

    //Use hulp_debug_bp_print_info for a simple dump of key breakpoint data:
    hulp_debug_bp_print_info(bp_data);

    //Some examples of things that may be checked/manipulated in breakpoint ISR:

        //Set R1 to 1234:
        // hulp_debug_bp_alter_reg(bp_data, R1, 1234);

        //Get and set register values.
        //eg. if R3 == 4 then R0++
        if(bp_data->regs.r3 == 4)
        {
            hulp_debug_bp_alter_reg(bp_data, R0, bp_data->regs.r0 + 1);
        }

        //Change where the ULP will continue execution via a label.
        //eg. If it hit breakpoint with the label LAB_BREAKPOINT_1, change execution to LAB_SOMEWHERE_ELSE:
        if(bp_data->bp.label.valid && bp_data->bp.label.num == LAB_BREAKPOINT_1)
        {
            hulp_debug_bp_set_continue_label(bp_data, LAB_SOMEWHERE_ELSE);
        }

        //Enable a breakpoint by label number (this or another)
        //eg. if this breakpoint has label LAB_BREAKPOINT_3, enable the breakpoint with label LAB_BREAKPOINT_1
        if(bp_data->bp.label.valid && bp_data->bp.label.num == LAB_BREAKPOINT_3)
        {
            hulp_debug_bp_enable_by_label(bp_data->meta.handle, LAB_BREAKPOINT_1);
        }

        //Disable this breakpoint using its pc
        //eg. if R0==7, disable this breakpoint
        if(bp_data->regs.r0 == 7)
        {
            hulp_debug_bp_disable_by_pc(bp_data->bp.pc);
        }

        //If some condition is met, choose not to restart ULP
        //eg. if R0 < 1000 at LAB_BREAKPOINT_4
        if(bp_data->bp.label.valid && bp_data->bp.label.num == LAB_BREAKPOINT_4)
        {
            if(bp_data->regs.r0 < 1000)
            {
                ets_printf("Oh no, a low R0 value at breakpoint 4?! ULP will not be restarted.\n");
                return;
            }
        }

        //eg. Stop after certain duration
        if(esp_log_timestamp() > (5 * 60 * 1000))
        {
            ets_printf("Time expired, ULP will not be restarted\n");
            return;
        }

    //When done, continue the ULP
    hulp_debug_bp_continue(bp_data);
}

void init_ulp()
{
    const ulp_insn_t program[] {
            I_ADDI(R3, R3, 1),
            I_MOVI(R1, 0),

            I_ADDI(R1, R1, 1),         
        M_LABEL(LAB_BREAKPOINT_1), M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),
            I_ADDI(R1, R1, 1),
        M_LABEL(LAB_SOMEWHERE_ELSE),
            I_ADDI(R1, R1, 1),
        M_LABEL(LAB_BREAKPOINT_2), M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),
            I_ANDR(R0, R3, 1),
            M_BGE(LAB_FINISHED, 1),
            I_ADDI(R1, R1, 1),
        M_LABEL(LAB_BREAKPOINT_3), M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),
            M_UPDATE_TICKS(),
            I_RD_TICKS_REG(1),
            I_ADDI(R1, R1, 1),
        M_LABEL(LAB_BREAKPOINT_4), M_DEBUG_SET_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),

        M_LABEL(LAB_FINISHED),
            I_HALT(),

        M_INCLUDE_DEBUG_BP(LAB_DEBUG_ENTRY, R2, ulp_debug_data),
    };

    hulp_debug_bp_config_t debug_cfg = {
        .data = &ulp_debug_data,
        .program = {
            .ptr = program,
            .num_words = (sizeof(program) / sizeof(program)[0]),
        },
        .callback = {
            .fn = ulp_breakpoint_handler,
            .ctx = nullptr,
        },
    };

    hulp_debug_bp_handle_t dbg_handle;
    ESP_ERROR_CHECK( hulp_debug_bp_init(&debug_cfg, &dbg_handle) );

    hulp_ulp_load(program, sizeof(program), 1ULL * 1000 * 1000);

    //Breakpoints may be disabled/enabled once the program is loaded (default: enabled)
    hulp_debug_bp_disable_by_label(dbg_handle, LAB_BREAKPOINT_1);

    //Everything is ready, start the ULP.
    hulp_ulp_run();
}

extern "C" void app_main()
{
    init_ulp();

    vTaskDelay(portMAX_DELAY);
}