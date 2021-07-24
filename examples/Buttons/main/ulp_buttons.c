#include "ulp_buttons.h"

#include <sys/param.h>

#include "hulp.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

static esp_err_t load_ulp_button_handler(uint32_t *load_addr, int debounce_ms, int double_click_timeout_ms, int hold_ms, int reinterrupt_ms)
{
    // ULP Button FSM:
    enum {
        ULP_BUTTON_IDLE,            // Idle, waiting for: press -> Pressed
        ULP_BUTTON_PRESSED,         // Just pressed, debouncing -> Down
        ULP_BUTTON_DOWN,            // Pressed down, waiting for: release -> Released /OR/ hold timer expiry -> Wait_Release
        ULP_BUTTON_RELEASED,        // Just released, debouncing -> Up
        ULP_BUTTON_UP,              // Click pending, waiting for: double click -> Wait_Release /OR/ double click timer expiry (ie. single click) -> Idle
        ULP_BUTTON_WAIT_RELEASE,    // Double/hold already processed, waiting for: release -> Debounce
        ULP_BUTTON_DEBOUNCE,        // Debouncing -> Idle
    };

    enum {
        LBL_BUTTON_IDLE,
        LBL_BUTTON_PRESSED,
        LBL_BUTTON_DOWN,
        LBL_BUTTON_RELEASED,
        LBL_BUTTON_UP,
        LBL_BUTTON_WAIT_RELEASE,
        LBL_BUTTON_DEBOUNCE,

        LBL_BUTTON_HANDLE_RELEASE,
        LBL_BUTTON_HANDLE_DOUBLE_CLICK,

        LBL_BUTTON_CHECK_INT_INTERVAL,
        LBL_BUTTON_CHECK_INTS,
        LBL_BUTTON_CHECK_INTS_DOUBLE,
        LBL_BUTTON_CHECK_INTS_HOLD,
        LBL_BUTTON_INT,

        LBL_BUTTON_DONE,
    };

    // The double click timeout and debounce timer use same timestamp, so ensure they use same range:
    uint8_t double_and_debounce_shift = hulp_ms_to_ulp_tick_shift(MAX(debounce_ms, double_click_timeout_ms));
    uint16_t debounce_ticks = hulp_ms_to_ulp_ticks_with_shift(debounce_ms, double_and_debounce_shift);
    uint16_t double_click_timeout_ticks = hulp_ms_to_ulp_ticks_with_shift(double_click_timeout_ms, double_and_debounce_shift);

    uint8_t hold_shift = hulp_ms_to_ulp_tick_shift(hold_ms);
    uint16_t hold_ticks = hulp_ms_to_ulp_ticks_with_shift(hold_ms, hold_shift);

    uint8_t reint_shift = hulp_ms_to_ulp_tick_shift(reinterrupt_ms);
    uint16_t reint_ticks = hulp_ms_to_ulp_ticks_with_shift(reinterrupt_ms, reint_shift);

    const ulp_insn_t program[] = {
        // For clarity, this is deliberately not optimised
        
        // Subroutine entry
        // Expects current GPIO level in R0 (0/1).

        // Prepare ALU so that we can branch based on current pin state later. By subtracting 1, ALU overflow == pin low; ALU zero == pin high
        I_SUBI(R2, R0, 1),

        // FSM: Load current state and branch
        I_LD(R0, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
        M_BL(LBL_BUTTON_IDLE, ULP_BUTTON_IDLE+1),
        M_BL(LBL_BUTTON_PRESSED, ULP_BUTTON_PRESSED+1),
        M_BL(LBL_BUTTON_DOWN, ULP_BUTTON_DOWN+1),
        M_BL(LBL_BUTTON_RELEASED, ULP_BUTTON_RELEASED+1),
        M_BL(LBL_BUTTON_UP, ULP_BUTTON_UP+1),
        M_BL(LBL_BUTTON_WAIT_RELEASE, ULP_BUTTON_WAIT_RELEASE+1),
        
        M_LABEL(LBL_BUTTON_DEBOUNCE),
            // Check debounce expiry
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_LD(R2, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_CHECK_INT_INTERVAL, debounce_ticks),
            // Debounce expired, reset to idle
            I_MOVI(R2, ULP_BUTTON_IDLE),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INT_INTERVAL),

        M_LABEL(LBL_BUTTON_WAIT_RELEASE),
            // If pin still low, do nothing
            M_BXF(LBL_BUTTON_CHECK_INT_INTERVAL),
            // Else released -> begin debounce timer
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_ST(R0, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_MOVI(R2, ULP_BUTTON_DEBOUNCE),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INT_INTERVAL),

        M_LABEL(LBL_BUTTON_UP),
            // If pin is low again, process double click
            M_BXF(LBL_BUTTON_HANDLE_DOUBLE_CLICK),
            // Else check if double click timeout expired
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_LD(R2, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_CHECK_INT_INTERVAL, double_click_timeout_ticks),
            // Double click timeout expired, process single click
            I_LD(R2, R1, offsetof(ulp_button_t, raw.single_clicks) / sizeof(ulp_var_t)),
            I_ADDI(R2, R2, 1),
            I_ST(R2, R1, offsetof(ulp_button_t, raw.single_clicks) / sizeof(ulp_var_t)),
            I_MOVI(R2, ULP_BUTTON_IDLE),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INTS),

            M_LABEL(LBL_BUTTON_HANDLE_DOUBLE_CLICK),
            I_LD(R2, R1, offsetof(ulp_button_t, raw.double_clicks) / sizeof(ulp_var_t)),
            I_ADDI(R2, R2, 1),
            I_ST(R2, R1, offsetof(ulp_button_t, raw.double_clicks) / sizeof(ulp_var_t)),
            I_MOVI(R2, LBL_BUTTON_WAIT_RELEASE),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INTS),

        M_LABEL(LBL_BUTTON_RELEASED),
            // Check debounce expiry
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_LD(R2, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_CHECK_INT_INTERVAL, debounce_ticks),
            // Debounce expired, set to UP
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_ST(R0, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_MOVI(R2, ULP_BUTTON_UP),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INT_INTERVAL),

        M_LABEL(LBL_BUTTON_DOWN),
            // If high, begin released debounce
            M_BXZ(LBL_BUTTON_HANDLE_RELEASE),
            // Else still low, check hold time
            I_RD_TICKS_REG(hold_shift),
            I_LD(R2, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_CHECK_INT_INTERVAL, hold_ticks),
            // Hold time expired, process hold
            I_LD(R2, R1, offsetof(ulp_button_t, raw.holds) / sizeof(ulp_var_t)),
            I_ADDI(R2, R2, 1),
            I_ST(R2, R1, offsetof(ulp_button_t, raw.holds) / sizeof(ulp_var_t)),
            I_MOVI(R2, LBL_BUTTON_WAIT_RELEASE),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INTS),

            M_LABEL(LBL_BUTTON_HANDLE_RELEASE),
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_ST(R0, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_MOVI(R2, ULP_BUTTON_RELEASED),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INT_INTERVAL),

        M_LABEL(LBL_BUTTON_PRESSED),
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_LD(R2, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_CHECK_INT_INTERVAL, debounce_ticks),
            // Else debounce expired, set to UP
            I_RD_TICKS_REG(hold_shift),
            I_ST(R0, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_MOVI(R2, ULP_BUTTON_DOWN),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),
            M_BX(LBL_BUTTON_CHECK_INT_INTERVAL),

        M_LABEL(LBL_BUTTON_IDLE),
            // If still high, do nothing
            M_BXZ(LBL_BUTTON_CHECK_INT_INTERVAL),
            // Else begin pressed debounce
            I_RD_TICKS_REG(double_and_debounce_shift),
            I_ST(R0, R1, offsetof(ulp_button_t, priv.ts) / sizeof(ulp_var_t)),
            I_MOVI(R2, ULP_BUTTON_PRESSED),
            I_ST(R2, R1, offsetof(ulp_button_t, priv.state) / sizeof(ulp_var_t)),

        // Check reinterrupt interval
        M_LABEL(LBL_BUTTON_CHECK_INT_INTERVAL),
            I_RD_TICKS_REG(reint_shift),
            I_LD(R2, R1, offsetof(ulp_button_t, priv.ts_int) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_DONE, reint_ticks),

            M_LABEL(LBL_BUTTON_CHECK_INTS),

            I_LD(R0, R1, offsetof(ulp_button_t, interrupts_en) / sizeof(ulp_var_t)),
            I_ANDI(R0, R0, ULP_BUTTON_INT_SINGLE),
            M_BXZ(LBL_BUTTON_CHECK_INTS_DOUBLE),
            I_LD(R0, R1, offsetof(ulp_button_t, processed.single_clicks) / sizeof(ulp_var_t)),
            I_LD(R2, R1, offsetof(ulp_button_t, raw.single_clicks) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BGE(LBL_BUTTON_INT, 1),

            M_LABEL(LBL_BUTTON_CHECK_INTS_DOUBLE),
            I_LD(R0, R1, offsetof(ulp_button_t, interrupts_en) / sizeof(ulp_var_t)),
            I_ANDI(R0, R0, ULP_BUTTON_INT_DOUBLE),
            M_BXZ(LBL_BUTTON_CHECK_INTS_HOLD),
            I_LD(R0, R1, offsetof(ulp_button_t, processed.double_clicks) / sizeof(ulp_var_t)),
            I_LD(R2, R1, offsetof(ulp_button_t, raw.double_clicks) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BGE(LBL_BUTTON_INT, 1),

            M_LABEL(LBL_BUTTON_CHECK_INTS_HOLD),
            I_LD(R0, R1, offsetof(ulp_button_t, interrupts_en) / sizeof(ulp_var_t)),
            I_ANDI(R0, R0, ULP_BUTTON_INT_HOLD),
            M_BXZ(LBL_BUTTON_DONE),
            I_LD(R0, R1, offsetof(ulp_button_t, processed.holds) / sizeof(ulp_var_t)),
            I_LD(R2, R1, offsetof(ulp_button_t, raw.holds) / sizeof(ulp_var_t)),
            I_SUBR(R0, R0, R2),
            M_BL(LBL_BUTTON_DONE, 1),

            M_LABEL(LBL_BUTTON_INT),
            I_WAKE(),
            I_RD_TICKS_REG(reint_shift),
            I_ST(R0, R1, offsetof(ulp_button_t, priv.ts_int) / sizeof(ulp_var_t)),

        M_LABEL(LBL_BUTTON_DONE),
            I_BXR(R3),
    };

    uint32_t program_size = ARRAY_SIZE(program);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(*load_addr, program, &program_size));
    *load_addr += program_size;
    return ESP_OK;
}

static esp_err_t load_ulp_program_begin(uint32_t *load_addr)
{
    const ulp_insn_t program[] = {
        I_FLAG_UPDATE_TICKS(),
    };
    uint32_t program_size = ARRAY_SIZE(program);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(*load_addr, program, &program_size));
    *load_addr += program_size;
    return ESP_OK;
}

static esp_err_t load_ulp_button_block(gpio_num_t gpio, uint16_t state_rtc_offset, uint32_t *load_addr)
{
    const ulp_insn_t program[] = {
        I_MOVI(R3, *load_addr + 4),
        I_MOVI(R1, state_rtc_offset),
        I_GPIO_READ(gpio),
        I_BXI(0),
    };
    uint32_t program_size = ARRAY_SIZE(program);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(*load_addr, program, &program_size));
    *load_addr += program_size;
    return ESP_OK;
}

static esp_err_t load_ulp_program_end(uint32_t *load_addr)
{
    const ulp_insn_t program[] = {
        I_HALT(),
    };
    uint32_t program_size = ARRAY_SIZE(program);
    ESP_ERROR_CHECK(ulp_process_macros_and_load(*load_addr, program, &program_size));
    *load_addr += program_size;
    return ESP_OK;
}

esp_err_t ulp_buttons_init(const gpio_num_t *button_gpios, ulp_button_t *buttons, size_t num_buttons, uint32_t period_us, int debounce_ms, int double_click_timeout_ms, int hold_ms, int reinterrupt_ms)
{
    uint32_t program_size = 0;

    // Load the handler at offset 0
    ESP_ERROR_CHECK(load_ulp_button_handler(&program_size, debounce_ms, double_click_timeout_ms, hold_ms, reinterrupt_ms));

    // Program entry will be at this point (ie. after handler routine) so keep a copy of current pc
    const uint32_t program_entry = program_size;    

    ESP_ERROR_CHECK(load_ulp_program_begin(&program_size));

    for(int i = 0; i < num_buttons; ++i)
    {
        ESP_ERROR_CHECK(load_ulp_button_block(button_gpios[i], RTC_WORD_OFFSET(buttons[i]), &program_size));
    }

    ESP_ERROR_CHECK(load_ulp_program_end(&program_size));

    for(int i = 0; i < num_buttons; ++i)
    {
        ESP_ERROR_CHECK(hulp_configure_pin(button_gpios[i], RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0));
    }

    ESP_ERROR_CHECK(ulp_set_wakeup_period(0, period_us));
    ESP_ERROR_CHECK(hulp_ulp_run(program_entry));
    return ESP_OK;
}

static uint16_t process_button_evt(ulp_var_t* processed, ulp_var_t *raw)
{
    uint16_t diff = raw->val - processed->val;
    if(diff)
    {
        processed->val = raw->val;
    }
    return diff;
}
uint16_t ulp_button_process_single_clicks(ulp_button_t *button)
{
    return process_button_evt(&button->processed.single_clicks, &button->raw.single_clicks);
}
uint16_t ulp_button_process_double_clicks(ulp_button_t *button)
{
    return process_button_evt(&button->processed.double_clicks, &button->raw.double_clicks);
}
uint16_t ulp_button_process_holds(ulp_button_t *button)
{
    return process_button_evt(&button->processed.holds, &button->raw.holds);
}

void ulp_button_interrupt_config(ulp_button_t *button, uint16_t interrupt_en)
{
    button->interrupts_en.val = interrupt_en;
}