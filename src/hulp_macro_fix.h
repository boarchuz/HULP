#ifndef HULP_MACRO_FIX_H
#define HULP_MACRO_FIX_H

#include "hulp_compat.h"

// Fix for register macros bitmask (see https://github.com/espressif/esp-idf/pull/11652)
    #if !defined(I_WR_REG)
    #   error "!I_WR_REG"
    #endif
    #undef I_WR_REG
    #define I_WR_REG(reg, low_bit, high_bit, val) {.wr_reg = {\
        .addr = ((reg) / sizeof(uint32_t)) & 0xff, \
        .periph_sel = SOC_REG_TO_ULP_PERIPH_SEL(reg), \
        .data = val, \
        .low = low_bit, \
        .high = high_bit, \
        .opcode = OPCODE_WR_REG } }

    #if !defined(I_RD_REG)
    #   error "!I_RD_REG"
    #endif
    #undef I_RD_REG
    #define I_RD_REG(reg, low_bit, high_bit) {.rd_reg = {\
        .addr = ((reg) / sizeof(uint32_t)) & 0xff, \
        .periph_sel = SOC_REG_TO_ULP_PERIPH_SEL(reg), \
        .unused = 0, \
        .low = low_bit, \
        .high = high_bit, \
        .opcode = OPCODE_RD_REG } }

#endif /* HULP_MACRO_FIX_H */
