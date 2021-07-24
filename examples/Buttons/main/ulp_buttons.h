#ifndef ULP_BUTTONS_H
#define ULP_BUTTONS_H

#include "hulp.h"

enum {
    ULP_BUTTON_INT_SINGLE       = (1 << 0),
    ULP_BUTTON_INT_DOUBLE       = (1 << 1),
    ULP_BUTTON_INT_HOLD         = (1 << 2),
};

typedef struct {
    ulp_var_t single_clicks;
    ulp_var_t double_clicks;
    ulp_var_t holds;
} ulp_button_actions_t;

// Struct containing config/state required for each button (in RTC slow memory)
typedef struct {
    ulp_var_t interrupts_en;        // interrupt enabled bits (ULP_BUTTON_INT_x)
    ulp_button_actions_t raw;       // raw events queued by ULP (SoC: read only)
    ulp_button_actions_t processed; // events acknowledged by SoC (ULP: read only)
    struct {                        // private ULP data
        ulp_var_t state;
        ulp_var_t ts;
        ulp_var_t ts_int;
    } priv;
} ulp_button_t;

esp_err_t ulp_buttons_init(const gpio_num_t *button_gpios, ulp_button_t *buttons, size_t num_buttons, uint32_t period_us, int debounce_ms, int double_click_timeout_ms, int hold_ms, int reinterrupt_ms);

uint16_t ulp_button_process_single_clicks(ulp_button_t *button);
uint16_t ulp_button_process_double_clicks(ulp_button_t *button);
uint16_t ulp_button_process_holds(ulp_button_t *button);
void ulp_button_interrupt_config(ulp_button_t *button, uint16_t interrupt_en);

#endif // ULP_BUTTONS_H