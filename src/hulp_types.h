#ifndef HULP_TYPES_H
#define HULP_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef union {
    struct {
        uint32_t data : 16;
        uint32_t reg_off : 2;
        uint32_t st : 3;
        uint32_t pc : 11;
    };
    struct {
        uint16_t val;
        uint16_t meta;
    };
    struct {
        uint8_t val_bytes[2];
        uint8_t meta_bytes[2];
    };
    struct {
        uint8_t bytes[4];
    };
    ulp_insn_t insn;
    uint32_t word;
} ulp_var_t;

_Static_assert(sizeof(ulp_var_t) == 4, "ulp_var_t size should be 4 bytes");

#ifdef __cplusplus
}
#endif

#endif /* HULP_TYPES_H */
