#include "hulp_regwr.h"

#include "hulp.h"

#include "hulp_config.h"

// Layout of RTC Slow Memory if using regwr
struct hulp_regwr_rtc_slow_map_check {
    // Normal usage
    ulp_insn_t normal1[HULP_REGWR_WORK_AREA_START];
    // Each register write bit range will write to 3 particular (possibly overlapping) instructions in this region.
    // May be used as temporary storage between register writes (register write will modify the corresponding region with its required instructions)
    // Unused regions may be used as normal.
    //      eg. if only one range [18:16] is used, then almost all of this is free for normal usage. If all ranges are used, all is reserved.
    ulp_insn_t regwr_work_area[HULP_REGWR_WORK_AREA_END - HULP_REGWR_WORK_AREA_START];
    // Normal usage
    ulp_insn_t normal2[HULP_WR_REG_GEN_ENTRY_HAS_RET - HULP_REGWR_WORK_AREA_END];
    // Reserved
    ulp_insn_t regwr_gen_wr[HULP_WR_REG_GEN_ENTRY_HAS_RET_COUNT];
    // Normal usage
    ulp_insn_t normal3[(HULP_WR_REG_GEN_ENTRY) - (HULP_WR_REG_GEN_ENTRY_HAS_RET + HULP_WR_REG_GEN_ENTRY_HAS_RET_COUNT)];
    // Reserved
    ulp_insn_t regwr_gen_ret[HULP_WR_REG_GEN_ENTRY_COUNT];
    // Normal usage
    ulp_insn_t normal4[((CONFIG_ESP32_ULP_COPROC_RESERVE_MEM / 4) > (HULP_WR_REG_GEN_ENTRY + HULP_WR_REG_GEN_ENTRY_COUNT)) ? (CONFIG_ESP32_ULP_COPROC_RESERVE_MEM / sizeof(ulp_insn_t)) - (HULP_WR_REG_GEN_ENTRY + HULP_WR_REG_GEN_ENTRY_COUNT) : 0];
};

_Static_assert(offsetof(struct hulp_regwr_rtc_slow_map_check, regwr_work_area) == (HULP_REGWR_WORK_AREA_START * sizeof(ulp_insn_t)), "wrong offset: work area");
_Static_assert(offsetof(struct hulp_regwr_rtc_slow_map_check, regwr_gen_wr) == (HULP_WR_REG_GEN_ENTRY_HAS_RET * sizeof(ulp_insn_t)), "wrong offset: gen wr");
_Static_assert(offsetof(struct hulp_regwr_rtc_slow_map_check, regwr_gen_ret) == (HULP_WR_REG_GEN_ENTRY * sizeof(ulp_insn_t)), "wrong offset: gen ret");

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