#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_touch.h"
#include "hulp_uart.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#define PIN_TOUCH GPIO_NUM_32
#define TOUCH_HYSTERESIS 1000

#define PIN_UART_TX GPIO_NUM_25

#define BAUD_RATE 9600

#define HULP_PRINTF_X_MIN_CAP 4
#define HULP_PRINTF_U_MIN_CAP 5

// Need a buffer for formatting hex/dec to ASCII
#define ULP_BUFFER_CAPACITY MAX(HULP_PRINTF_X_MIN_CAP, HULP_PRINTF_U_MIN_CAP)
RTC_DATA_ATTR ulp_var_t ulp_buffer HULP_UART_STRING_RESERVE(ULP_BUFFER_CAPACITY);

// And some strings to format the output, eg. "Pin 00032: 04779 (0x12AB)\n"
#define ULP_STRING_FMT_1 "Pin "
#define ULP_STRING_FMT_2 ": "
#define ULP_STRING_FMT_3 " (0x"
#define ULP_STRING_FMT_4 ")\n"
#define ULP_STRING_DETECTION "Detection!\n"

RTC_DATA_ATTR ulp_var_t ulp_str_fmt_1 HULP_UART_STRING_RESERVE(ULP_STRING_FMT_1);
RTC_DATA_ATTR ulp_var_t ulp_str_fmt_2 HULP_UART_STRING_RESERVE(ULP_STRING_FMT_2);
RTC_DATA_ATTR ulp_var_t ulp_str_fmt_3 HULP_UART_STRING_RESERVE(ULP_STRING_FMT_3);
RTC_DATA_ATTR ulp_var_t ulp_str_fmt_4 HULP_UART_STRING_RESERVE(ULP_STRING_FMT_4);
RTC_DATA_ATTR ulp_var_t ulp_str_detection HULP_UART_STRING_RESERVE(ULP_STRING_DETECTION);

//Somewhere to store the previous touch val for comparison:
RTC_DATA_ATTR ulp_var_t ulp_previous_touch_val;

void init_ulp()
{
    enum
    {
        LBL_FMT_1,
        LBL_PRINTF_PIN_RETURN,
        LBL_UART_PIN_RETURN,
        LBL_FMT_2,
        LBL_PRINTF_DEC_RETURN,
        LBL_UART_DEC_RETURN,
        LBL_FMT_3,
        LBL_PRINTF_HEX_RETURN,
        LBL_UART_HEX_RETURN,
        LBL_FMT_4,

        LBL_ALERT_OUT_RETURN,

        LBL_FINISHED,

        LBL_UART_TX_ENTRY,

        LBL_PRINTF_DEC_ENTRY,
        LBL_PRINTF_HEX_ENTRY,
    };

    const ulp_insn_t program[] = {
        // Start a touch measurement and wait until it finishes
            M_TOUCH_BEGIN(),
            M_TOUCH_WAIT_DONE(),

        // TX the start of the output
            I_MOVO(R1, ulp_str_fmt_1),
            M_RETURN(LBL_FMT_1, R3, LBL_UART_TX_ENTRY),

        // Format and TX the pin number
            // printf needs value in R0 and buffer pointer in R1
            I_MOVI(R0, PIN_TOUCH),
            I_MOVO(R1, ulp_buffer),
            // Go to printf subroutine
            M_RETURN(LBL_PRINTF_PIN_RETURN, R3, LBL_PRINTF_DEC_ENTRY),
            // And now TX it out over UART (R1 will not be altered by printf, so the buffer pointer is still valid):
            M_RETURN(LBL_UART_PIN_RETURN, R3, LBL_UART_TX_ENTRY),

        // TX the next part of the output
            I_MOVO(R1, ulp_str_fmt_2),
            M_RETURN(LBL_FMT_2, R3, LBL_UART_TX_ENTRY),

        // Format and TX the touch measurement (decimal)
            I_TOUCH_GET_GPIO_VALUE(PIN_TOUCH),
            I_MOVO(R1, ulp_buffer),
            M_RETURN(LBL_PRINTF_DEC_RETURN, R3, LBL_PRINTF_DEC_ENTRY),
            M_RETURN(LBL_UART_DEC_RETURN, R3, LBL_UART_TX_ENTRY),

        // TX the next part of the output
            I_MOVO(R1, ulp_str_fmt_3),
            M_RETURN(LBL_FMT_3, R3, LBL_UART_TX_ENTRY),

        // Format and TX the touch measurement (hex)
            I_TOUCH_GET_GPIO_VALUE(PIN_TOUCH),
            I_MOVO(R1, ulp_buffer),
            M_RETURN(LBL_PRINTF_HEX_RETURN, R3, LBL_PRINTF_HEX_ENTRY),
            M_RETURN(LBL_UART_HEX_RETURN, R3, LBL_UART_TX_ENTRY),

        // TX the last part of the output
            I_MOVO(R1, ulp_str_fmt_4),
            M_RETURN(LBL_FMT_4, R3, LBL_UART_TX_ENTRY),


        // Now check if this touch < (previous - hysteresis), in which case TX an alert string
            I_TOUCH_GET_GPIO_VALUE(PIN_TOUCH),
            // Load the previous value into R1
            I_MOVI(R2, 0),
            I_GET(R1, R2, ulp_previous_touch_val),
            // Store it as the 'previous' value for next time
            I_PUT(R0, R2, ulp_previous_touch_val),
            I_SUBR(R0, R1, R0), // R0 = previous - current
            // If that overflowed then current > previous so halt:
            M_BXF(LBL_FINISHED),
            // Else current <= previous:
            I_SUBI(R0, R0, TOUCH_HYSTERESIS), // R0 = difference - hysteresis
            // If that overflowed then difference < hysteresis so halt:
            M_BXF(LBL_FINISHED),
            // Else difference >= hysteresis so alert!
            I_MOVO(R1, ulp_str_detection),
            M_RETURN(LBL_ALERT_OUT_RETURN, R3, LBL_UART_TX_ENTRY),

        // Sleep
            M_LABEL(LBL_FINISHED),
                I_HALT(),

        //Include subroutine for UART TX:
            M_INCLUDE_UART_TX(LBL_UART_TX_ENTRY, BAUD_RATE, PIN_UART_TX),

        // Include subroutines for printf:
            M_INCLUDE_PRINTF_X(LBL_PRINTF_HEX_ENTRY),
            // By default, the dec printf will put a newline in the final byte. Useful if you want to dump values without any formatting.
                // M_INCLUDE_PRINTF_U(LBL_PRINTF_DEC_ENTRY),
            // In this case, we'll leave the last byte unused instead with a custom declaration:
            M_INCLUDE_PRINTF_U_(LBL_PRINTF_DEC_ENTRY, R1, R2, R3, 0, '\0'),
    };

    // Set the contents of string buffers
    if(
        hulp_uart_string_set(ulp_str_fmt_1, ARRAY_SIZE(ulp_str_fmt_1), ULP_STRING_FMT_1) < 0 ||
        hulp_uart_string_set(ulp_str_fmt_2, ARRAY_SIZE(ulp_str_fmt_2), ULP_STRING_FMT_2) < 0 ||
        hulp_uart_string_set(ulp_str_fmt_3, ARRAY_SIZE(ulp_str_fmt_3), ULP_STRING_FMT_3) < 0 ||
        hulp_uart_string_set(ulp_str_fmt_4, ARRAY_SIZE(ulp_str_fmt_4), ULP_STRING_FMT_4) < 0 ||
        hulp_uart_string_set(ulp_str_detection, ARRAY_SIZE(ulp_str_detection), ULP_STRING_DETECTION) < 0
    )
    {
        abort();
    }

    const hulp_touch_controller_config_t controller_config = HULP_TOUCH_CONTROLLER_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(hulp_configure_touch_controller(&controller_config));
    
    const hulp_touch_pin_config_t pin_config = HULP_TOUCH_PIN_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(hulp_configure_touch_pin(PIN_TOUCH, &pin_config));

#if CONFIG_HULP_UART_TX_OD
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_UART_TX, RTC_GPIO_MODE_DISABLED, GPIO_PULLUP_ONLY, 0));
#else
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_UART_TX, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1));
#endif

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 200ULL * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

extern "C" void app_main()
{
    if(!hulp_is_ulp_wakeup())
    {
        init_ulp();
    }

    esp_deep_sleep_start();
}