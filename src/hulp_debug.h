#ifndef HULP_DEBUG_H
#define HULP_DEBUG_H

#include "hulp.h"

typedef struct hulp_debug_bp_state_t *hulp_debug_bp_handle_t;

/**
 * Type for ULP to communicate breakpoint data to SoC handler. Do not access this directly.
 * Object must be in RTC_SLOW_MEM for ULP access (declare with RTC_DATA_ATTR).
 * 
 * Member order is fixed; ULP BP macros expect these offsets.
 */
struct ulp_debug_bp_data_t {
    ulp_var_t marker;
    ulp_var_t scr;
    ulp_var_t reg[3];
};

/**
 * Structured ULP breakpoint information.
 * Generally, 'meta' should be left to hulp_debug internal functions.
 */
struct hulp_debug_bp_cb_data_t {
    struct {
        uint16_t r0; //Value of R0
        uint16_t r1; //Value of R1
        uint16_t r2; //Value of R2
        uint16_t r3; //Value of R3
    } regs;
    struct {
        uint16_t pc; //PC of breakpoint
        uint16_t line; //File line number of breakpoint macro
        struct {
            bool valid; //Whether or not label.num is valid or should be ignored
            uint16_t num; //Label number of breakpoint
        } label;
    } bp;
    struct {
        hulp_debug_bp_handle_t handle;
        uint8_t reg_scr; //Which register is scratch for this breakpoint
        uint16_t return_addr; //Address to continue to after breakpoint is processed and ULP state restored
        struct {
            bool timer_en; //Whether or not ULP wakeup timer was enabled before being disabled by breakpoint.
            uint32_t entry_point; //Original ULP program entry point to be restored upon continue.
        } config_backup;
    } meta;
};

/**
 * ULP breakpoint callback ISR function type.
 * Short, no printf style functions, etc.
 */
typedef void (*hulp_debug_bp_cb_t)(hulp_debug_bp_cb_data_t* info, void* ctx);

/**
 * ULP breakpoint debugging config for initialisation.
 * 
 * Providing an optional pointer to the program array will allow the debugger to associate breakpoints with label numbers for improved debugging information.
 */
struct hulp_debug_bp_config_t {
    ulp_debug_bp_data_t* data;
    struct {
        const ulp_insn_t* ptr;
        size_t num_words;
    } program;
    struct {
        hulp_debug_bp_cb_t fn;
        void* ctx;
    } callback;
};

#define HULP_DEBUG_BP_CONFIG_DEFAULT(bp_data, ulp_program, program_size) { \
            .data = &(bp_data), \
            .program = { \
                .ptr = (ulp_program), \
                .num_words = (program_size) / sizeof(ulp_insn_t), \
            }, \
            .callback = { \
                .fn = hulp_debug_bp_callback_default, \
                .ctx = nullptr, \
            }, \
        }

#define HULP_DEBUG_BP_CONFIG_DEFAULT_NO_LABELS(bp_data) \
    HULP_DEBUG_BP_CONFIG_DEFAULT(bp_data, nullptr, 0)

/**
 * Initialise ULP breakpoint debugging with the provided configuration. 
 * 
 * A pointer to a handle is optional if deinitialisation is desired later to free resources.
 * The ULP interrupt must be enabled with hulp_ulp_interrupt_en() to begin.
 */
esp_err_t hulp_debug_bp_init(const hulp_debug_bp_config_t* config, hulp_debug_bp_handle_t* handle = nullptr);

/**
 * Free resources associated with the provided ULP breakpoint debugging handle.
 */
esp_err_t hulp_debug_bp_deinit(hulp_debug_bp_handle_t handle);

/**
 * Prints basic breakpoint debug info and continues ULP execution.
 * May be used in callback ISR, or used directly as the callback in debug initialisation config.
 */
void hulp_debug_bp_callback_default(hulp_debug_bp_cb_data_t* bp_data, void*);

/**
 * Basic dump of breakpoint info.
 * May be used in callback ISR.
 */
void hulp_debug_bp_print_info(hulp_debug_bp_cb_data_t* bp_data);

/**
 * Restores the ULP state from a breakpoint and continues execution.
 * May be used in callback ISR.
 */
void hulp_debug_bp_continue(hulp_debug_bp_cb_data_t* bp_data);

/**
 * Enable the BP at the provided pc.
 * May be used in callback ISR.
 */
esp_err_t hulp_debug_bp_enable_by_pc(uint16_t pc);

/**
 * Disable the BP at the provided pc.
 * May be used in callback ISR.
 */
esp_err_t hulp_debug_bp_disable_by_pc(uint16_t pc);

/**
 * Enable the BP using the identifying label immediately before the M_DEBUG_SET_BP macro.
 * This expects the instruction in RTC_SLOW_MEM so the program must already be loaded.
 * May be used in callback ISR.
 */
esp_err_t hulp_debug_bp_enable_by_label(hulp_debug_bp_handle_t handle, uint16_t label_num);

/**
 * Disable the BP using the identifying label immediately before the M_DEBUG_SET_BP macro.
 * This expects the instruction in RTC_SLOW_MEM so the program must already be loaded.
 * May be used in callback ISR.
 */
esp_err_t hulp_debug_bp_disable_by_label(hulp_debug_bp_handle_t handle, uint16_t label_num);

/**
 * In a breakpoint, change where the ULP will continue from using the provided label.
 * When hulp_debug_bp_continue is called, the ULP will branch to this label instead of continuing from the breakpoint.
 * This is useful for conditionally controlling the flow of execution based on the current state.
 * May be used in callback ISR.
 */
esp_err_t hulp_debug_bp_set_continue_label(hulp_debug_bp_cb_data_t* bp_data, uint16_t label_num);

/**
 * In a breakpoint, change the value of a general purpose register (R0-R3, excluding the scratch register).
 * The ULP will load this updated value into the specified register when it continues.
 */
esp_err_t hulp_debug_bp_alter_reg(hulp_debug_bp_cb_data_t* bp_data, uint8_t reg, uint16_t val);

/**
 * Set a breakpoint in a ULP program.
 *  
 * If breakpoints have been initialised in HULP, interrupts are enabled, and this BP is enabled, the ULP will log information and halt at this point.
 * The current state can then be debugged and manipulated via hulp_debug methods before continuing execution when ready.
 * 
 *  label_bp_dep_entry: The same label number provided to M_INCLUDE_DEBUG_BP
 *  reg_scr: Unfortunately, 1 register must be used as a scratch register (its value will be overwritten before breaking).
 *  bp_data: The same ulp_debug_bp_data_t provided to M_INCLUDE_DEBUG_BP
 */
#define M_DEBUG_SET_BP(label_bp_dep_entry, reg_scr, bp_data) \
            I_BGE(1, 0), /* <- allows debugger to enable/disable breakpoint by skipping or continuing into bp instructions. default: enabled */ \
            I_MOVI(reg_scr, __LINE__), \
            I_PUTO(reg_scr, reg_scr, __LINE__, (bp_data)), /* <- store the scr_reg and current PC for debugger */ \
            M_BX(label_bp_dep_entry)

//These must be kept in sync with above macro for offset calculations. Used internally. Do not use.
// #define HULP_DEBUG_SET_BP_INSN_CHECK_FIRST I_BGE(1,0) //for rudimentary verification of M_DEBUG_SET_BP
#define HULP_DEBUG_SET_BP_START_ST_OFFSET 2 //num instructions (excl macros) from start of M_DEBUG_SET_BP to I_ST
#define HULP_DEBUG_SET_BP_INSN_NUM 4   //num instructions (excl macros) in a M_DEBUG_SET_BP block


/**
 * Include subroutine necessary for ULP breakpoints.
 * Typically this macro is at the end of your program (eg. after I_HALT).
 * 
 *  label_entry: A unique label number for this subroutine. This label should be provided to every M_DEBUG_SET_BP so the breakpoint knows where to find this subroutine.
 *  reg_scr: Unfortunately, 1 register must be used as a scratch register (its value will be overwritten before breaking).
 *  bp_data: A ulp_debug_bp_data_t in RTC_SLOW_MEM
 */
#define M_INCLUDE_DEBUG_BP(label_entry, reg_scr, bp_data) \
    M_LABEL(label_entry), \
        I_MOVI(reg_scr, reg_scr), \
        I_PUTO(reg_scr, reg_scr, (reg_scr) - 1, ((bp_data))), \
        I_PUTO( ((reg_scr) == R0 ? R1 : R0), reg_scr, (reg_scr) - 2, ((bp_data)) ), \
        I_PUTO( ((reg_scr) == R1 ? R2 : R1), reg_scr, (reg_scr) - 3, ((bp_data)) ), \
        I_PUTO( ((reg_scr) == R2 ? R3 : R2), reg_scr, (reg_scr) - 4, ((bp_data)) ), \
        I_WAKE(), \
        I_HALT(), \
        I_GETO( ((reg_scr) == R0 ? R1 : R0), reg_scr, (reg_scr) - 2, ((bp_data)) ), /* <- dirty reentry point */ \
        I_GETO( ((reg_scr) == R1 ? R2 : R1), reg_scr, (reg_scr) - 3, ((bp_data)) ), \
        I_GETO( ((reg_scr) == R2 ? R3 : R2), reg_scr, (reg_scr) - 4, ((bp_data)) ), \
        M_SET_ENTRY(0),  /* <- default reentry point */ \
        I_BXI(0)

#define HULP_DEBUG_BP_INC_ST_REENTRY_DIRTY_OFFSET 6   // num instructions (excl macros) from reg_scr I_ST to the dirty reentry (I_GET)
#define HULP_DEBUG_BP_INC_ST_REENTRY_DEFAULT_OFFSET 9 // num instructions (excl macros) from reg_scr I_ST to default reentry (M_SET_ENTRY)

#endif // HULP_DEBUG_H