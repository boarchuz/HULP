#include "hulp_regwr.h"

#include "hulp.h"

esp_err_t hulp_regwr_load_generate_ret()
{
    const ulp_insn_t program[] = {
        I_MOVI(R1, R3), // Constant == 3
        I_ST(R1, R0, 2), // MUST BE AT 1025
        I_BXI(HULP_WR_REG_GEN_ENTRY_HAS_RET),
    };
    size_t program_size = sizeof(program) / sizeof(program[0]);
    return ulp_process_macros_and_load(HULP_WR_REG_GEN_ENTRY, program, &program_size);
}

esp_err_t hulp_regwr_load_generate_wr()
{
    const ulp_insn_t program[] = {
        I_MOVI(R1, (1 << 10) | (R0 << 2) | (R2 << 0)),
        I_ST(R1, R0, 0), // MUST BE AT 832
        I_BXR(R0),
    };
    size_t program_size = sizeof(program) / sizeof(program[0]);
    return ulp_process_macros_and_load(HULP_WR_REG_GEN_ENTRY_HAS_RET, program, &program_size);
}

esp_err_t hulp_regwr_prepare_offset(uint32_t offset)
{
    const ulp_insn_t r3_return = I_BXR(R3);
    RTC_SLOW_MEM[offset + 2] = r3_return.instruction;
    return ESP_OK;
}