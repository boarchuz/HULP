#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_log.h"

#include "hulp.h"
#include "hulp_uart.h"

#include "sdkconfig.h"

#define PIN_ULP_TX GPIO_NUM_25
#define PIN_ULP_RX GPIO_NUM_26

#define BAUD_RATE 115200

#define ULP_STRING_GREETING     "Hello, what's your name?\n"
#define ULP_STRING_REPLY_START  "Nice to meet you, "
#define ULP_STRING_REPLY_END    "! I'm a ULP Coprocessor.\n"
#define ULP_RX_MAX_LEN          32

RTC_DATA_ATTR ulp_var_t ulp_greeting HULP_UART_STRING_RESERVE(ULP_STRING_GREETING);
RTC_DATA_ATTR ulp_var_t ulp_reply_start HULP_UART_STRING_RESERVE(ULP_STRING_REPLY_START);
RTC_DATA_ATTR ulp_var_t ulp_reply_end HULP_UART_STRING_RESERVE(ULP_STRING_REPLY_END);

RTC_DATA_ATTR ulp_var_t ulp_rx_buffer HULP_UART_STRING_BUFFER(ULP_RX_MAX_LEN);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(array) (sizeof(array) / sizeof((array)[0]))
#endif

void init_ulp()
{
    enum
    {
        LBL_TX_GREET,
        LBL_RX_RESPONSE,
        LBL_TX_REPLY_START,
        LBL_TX_RESPONSE,
        LBL_TX_REPLY_END,

        LBL_SUBROUTINE_TX_ENTRY,
        LBL_SUBROUTINE_RX_ENTRY,
    };

    const ulp_insn_t program[] = {
        //Request name
        I_MOVO(R1, ulp_greeting),
        M_RETURN(LBL_TX_GREET, R3, LBL_SUBROUTINE_TX_ENTRY),
        //Read response (name) into buffer
        I_MOVO(R1, ulp_rx_buffer),
        M_RETURN(LBL_RX_RESPONSE, R3, LBL_SUBROUTINE_RX_ENTRY),
        //Start intro
        I_MOVO(R1, ulp_reply_start),
        M_RETURN(LBL_TX_REPLY_START, R3, LBL_SUBROUTINE_TX_ENTRY),
        //Echo the name that was just received
        I_MOVO(R1, ulp_rx_buffer),
        M_RETURN(LBL_TX_RESPONSE, R3, LBL_SUBROUTINE_TX_ENTRY),
        //End intro
        I_MOVO(R1, ulp_reply_end),
        M_RETURN(LBL_TX_REPLY_END, R3, LBL_SUBROUTINE_TX_ENTRY),
        //Sleep
        I_WAKE(),
        I_HALT(),

        //Dependencies for UART
        M_INCLUDE_UART_TX(LBL_SUBROUTINE_TX_ENTRY, BAUD_RATE, PIN_ULP_TX),
        M_INCLUDE_UART_RX(LBL_SUBROUTINE_RX_ENTRY, BAUD_RATE, PIN_ULP_RX, '\n'),
    };

    // Set the contents of string buffers
    if(
        hulp_uart_string_set(ulp_greeting, ARRAY_SIZE(ulp_greeting), ULP_STRING_GREETING) < 0 ||
        hulp_uart_string_set(ulp_reply_start, ARRAY_SIZE(ulp_reply_start), ULP_STRING_REPLY_START) < 0 ||
        hulp_uart_string_set(ulp_reply_end, ARRAY_SIZE(ulp_reply_end), ULP_STRING_REPLY_END) < 0
    )
    {
        abort();
    }

#if CONFIG_HULP_UART_TX_OD
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_ULP_TX, RTC_GPIO_MODE_DISABLED, GPIO_PULLUP_ONLY, 0));
#else
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_ULP_TX, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 1));
#endif
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_ULP_RX, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0));
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 10ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR(*(TaskHandle_t*)task_handle_ptr, 0, eNoAction, NULL);
}

extern "C" void app_main()
{
    // ULP will trigger an interrupt when a new UART string is received. Set up here.
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    hulp_ulp_isr_register(&ulp_isr, &main_handle);
    hulp_ulp_interrupt_en();

    // Load and start the program.
    init_ulp();

    for (;;)
    {
        // Block until notified by ULP+ISR
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        // Get the contents of the buffer and display
        char name[ULP_RX_MAX_LEN + 1];
        int len = hulp_uart_string_get(ulp_rx_buffer, name, sizeof(name), false);
        if(len < 0)
        {
            abort();
        }
        printf("ULP RX String (%d): %s\n", len, name);
    }
}