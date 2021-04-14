/**
 * Demonstrates using the ULP to write registers using the value in R2 into any register, able to be changed at runtime.
 * 
 * Limitations:
 *  Up to 6 bits may be written from the MSB of R2
 *  Valid low bits are 0, 8, 16, 24
 *  If more than 6 bits are to be written, the instruction will pad with zeros the bits above the sixth bit
 *  Some very specific instructions must be set at very specific places.
 *      These are at very high offsets (~830 and ~1024). Ensure reserved ULP mem is >= 4108 bytes.
 *  Each combination of high:low requires 3 instructions reserved at a particular place as its 'work area'.
 *      These are all contained between HULP_REGWR_WORK_AREA_START and HULP_REGWR_WORK_AREA_END (approx words 128-256).
 *      For example, if you write [3:1] of any register at any time then it will require HULP_REGWR_WORK_OFFSET(1,3) to HULP_REGWR_WORK_OFFSET(1,3)+2
 *      If you do not use a combination then you are free to use that work area as normal.
 *      It is up to you to ensure that nothing else is placed in work areas.
 *
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "hulp.h"
#include "hulp_regwr.h"

#include "sdkconfig.h"

static const char* TAG = "HULP_REGWR";

// Pick a test register for this example.
// SENS_ULP_CP_SLEEP_CYC2_REG is unused in this application so there's no harm writing anything to it.
#define ULP_WR_TEST_REG SENS_ULP_CP_SLEEP_CYC2_REG

static void init_ulp()
{
    // Load special instructions to return to your program from any register write
    ESP_ERROR_CHECK(hulp_regwr_load_generate_ret());
    // Load special instructions to generate variable register writes at run time
    ESP_ERROR_CHECK(hulp_regwr_load_generate_wr());

    enum {
        LBL_INIT_DONE,
        LBL_REGWR_RETURN,
    };

    const ulp_insn_t program[] = {
    // Initialise some values
        // R2 is used for the register, and the 6 bit value.
        // Initialise the value to 0 here, and set the register. Of course any of this can be changed at run time.
        I_MOVI(R2, HULP_REGWR_IMM_VAL(ULP_WR_TEST_REG, 0)),
        // The address of the work area for the particular combination of high:low must be set in R0
        // In this example, we're writing 6 bits [5:0] so it looks like this:
        I_MOVI(R0, HULP_REGWR_WORK_OFFSET(0, 5)),
        // Where to return to after the register write
        M_MOVL(R3, LBL_REGWR_RETURN),
    // Do that once, and now set the entry point for next run to here:
    M_LABEL(LBL_INIT_DONE),
    M_SET_ENTRY_LBL(LBL_INIT_DONE, program),

    // Loop:
        // The value that will be written to the register is in the upper 6 bits of R2. So to increment by one, add (1 << HULP_REGWR_VAL_SHIFT)
        I_ADDI(R2, R2, 1 << HULP_REGWR_VAL_SHIFT),
        // Branch to this address to do the configured register write.
        // It will ensure return instruction exists, generate register write instruction, execute the register write, and return to R3
        I_BXI(HULP_WR_REG_GEN_ENTRY),
    M_LABEL(LBL_REGWR_RETURN),
        // Returns here.
        I_HALT(),
    };

    // Clear whatever is in our test register
    REG_WRITE(ULP_WR_TEST_REG, 0);

    ESP_ERROR_CHECK( hulp_ulp_load(program, sizeof(program), 1 * 1000 * 1000, 0) );
    ESP_ERROR_CHECK( hulp_ulp_run(0) );

    // Print something every time the ULP writes to the register
    uint32_t reg_current = 0;
    for(;;)
    {
        uint32_t reg_new = REG_READ(ULP_WR_TEST_REG);
        if(reg_new != reg_current)
        {
            reg_current = reg_new;
            ESP_LOGI(TAG, "ULP Wrote: %u", reg_current);
        }
        vTaskDelay(1);
    }
}

void app_main(void)
{
    init_ulp();
    vTaskDelete(NULL);   
}