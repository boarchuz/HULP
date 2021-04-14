#ifndef HULP_REGWR_H
#define HULP_REGWR_H

#include "hulp.h"

/**
 * Each combination of high_bit and low_bit require a few words at a specific PC
 *  
 * Due to limitations of this method, low_bit may only be 0, 8, 16, or 24
 */
#define HULP_REGWR_WORK_OFFSET(low_bit, high_bit) \
    (128 + (high_bit) * 4 + (low_bit) / 8)

/**
 * At each of these PCs, this is the number of instructions required to implement the register write
 */
#define HULP_REGWR_WORK_COUNT 3

#define HULP_REGWR_WORK_AREA_START (HULP_REGWR_WORK_OFFSET(0,0))
#define HULP_REGWR_WORK_AREA_END (HULP_REGWR_WORK_OFFSET(24,31) + HULP_REGWR_WORK_COUNT)

#define HULP_REGWR_VAL_SHIFT 10
#define HULP_REGWR_IMM_VAL(register, value) \
    (((value) << HULP_REGWR_VAL_SHIFT) | (SOC_REG_TO_ULP_PERIPH_SEL(register) << 8) | ((register & 0xFF) / sizeof(uint32_t)))

/**
 * Branch here to write the register with the prepared parameters
 */
#define HULP_WR_REG_GEN_ENTRY 1024

/**
 * Branch here to write the register with the prepared parameters.
 * 
 * Return instruction must exist. Generally, prefer HULP_WR_REG_GEN_ENTRY.
 */
#define HULP_WR_REG_GEN_ENTRY_HAS_RET 831

/**
 * Sets instructions to generate a register write instruction
 */
esp_err_t hulp_regwr_load_generate_wr();

/**
 * Sets instructions to generate a return instruction from any register write range
 */
esp_err_t hulp_regwr_load_generate_ret();

/**
 * Prepares the area required by a particular [high:low] register write with a return instruction so it may be used
 */
esp_err_t hulp_regwr_prepare_offset(uint32_t offset);

#endif // HULP_REGWR_H