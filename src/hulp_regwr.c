#include "hulp_regwr.h"

#include "hulp.h"

esp_err_t hulp_regwr_load_generate_ret()
{
/**
 * Generates a I_BXR(R3) instruction at the end of the provided work area
 * 
 * Executing at 1025 generates a I_BXR instruction
 *      -> I_BXR(x)
 * [1:0] determines the register
 *      -> I_BXR(R3)
 */
    const ulp_insn_t program[] = {
        I_MOVI(R1, R3), // Constant == 3
        I_ST(R1, R0, (HULP_REGWR_WORK_COUNT - 1)), // MUST BE AT 1025
        I_BXI(HULP_WR_REG_GEN_ENTRY_HAS_RET),
    };
    _Static_assert(sizeof(program) / sizeof(program[0]) == HULP_WR_REG_GEN_ENTRY_COUNT, "program size != reserved");

    size_t program_size = sizeof(program) / sizeof(program[0]);
    return ulp_process_macros_and_load(HULP_WR_REG_GEN_ENTRY, program, &program_size);
}

esp_err_t hulp_regwr_load_generate_wr()
{
/**
 * Generates a ST instruction at the provided work area address in R0, then branches to it to execute
 * 
 * Executing at 832 generates a ST instruction
 *      -> I_ST(x, x, x)
 * [15:10] determine the offset value of the generated ST instruction, ie. (1 << 10) -> offset 1
 *      -> I_ST(x, x, 1)
 * [3:2] is Rdest
 *      -> I_ST(x, R0, 1)
 * [1:0] is Rsrc
 *      -> I_ST(R2, R0, 1)
 * 
 * Executing this generated ST at the correct PC produces a WR_REG with the required bit range, and R2's value determines its register/val
 */
    const ulp_insn_t program[] = {
        I_MOVI(R1, (1 << 10) | (R0 << 2) | (R2 << 0)),
        I_ST(R1, R0, 0), // MUST BE AT 832
        I_BXR(R0),
    };
    _Static_assert(sizeof(program) / sizeof(program[0]) == HULP_WR_REG_GEN_ENTRY_HAS_RET_COUNT, "program size != reserved");

    size_t program_size = sizeof(program) / sizeof(program[0]);
    return ulp_process_macros_and_load(HULP_WR_REG_GEN_ENTRY_HAS_RET, program, &program_size);
}

esp_err_t hulp_regwr_prepare_offset(uint32_t offset)
{
    const ulp_insn_t r3_return = I_BXR(R3);
    RTC_SLOW_MEM[offset + HULP_REGWR_WORK_COUNT - 1] = r3_return.instruction;
    return ESP_OK;
}