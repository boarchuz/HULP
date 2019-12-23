#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"
#include "hulp_uart.h"

#define PIN_ULP_TX GPIO_NUM_14
#define PIN_ULP_RX GPIO_NUM_13
#define BAUD_RATE 9600

RTC_DATA_ATTR ulp_string_t<26> ulp_greeting("Hello, what's your name?\n");
RTC_DATA_ATTR ulp_string_t<19> ulp_introStart("Nice to meet you, ");
RTC_DATA_ATTR ulp_string_t<26> ulp_introEnd("! I'm a ULP Coprocessor.\n");
RTC_DATA_ATTR ulp_string_t<32> ulp_receiveBuffer;

void startulp()
{
    enum
    {
        LBL_GREET,
        LBL_WAIT_RESPONSE,
        LBL_INTRO_START,
        LBL_ECHO_NAME,
        LBL_INTRO_END,
        LBL_UART_TX_ENTRY,
        LBL_UART_RX_ENTRY,
    };

    const ulp_insn_t program[]{
        //Request name
        I_MOVO(R1, ulp_greeting),
        M_RETURN(LBL_GREET, R3, LBL_UART_TX_ENTRY),
        //Read response (name) into buffer
        I_MOVO(R1, ulp_receiveBuffer),
        M_RETURN(LBL_WAIT_RESPONSE, R3, LBL_UART_RX_ENTRY),
        //Start intro
        I_MOVO(R1, ulp_introStart),
        M_RETURN(LBL_INTRO_START, R3, LBL_UART_TX_ENTRY),
        //Echo the name that was just received
        I_MOVO(R1, ulp_receiveBuffer),
        M_RETURN(LBL_ECHO_NAME, R3, LBL_UART_TX_ENTRY),
        //End intro
        I_MOVO(R1, ulp_introEnd),
        M_RETURN(LBL_INTRO_END, R3, LBL_UART_TX_ENTRY),
        //Sleep
        I_HALT(),

        //Dependencies for UART
        M_INCLUDE_UART_TX(LBL_UART_TX_ENTRY, BAUD_RATE, PIN_ULP_TX),
        M_INCLUDE_UART_RX(LBL_UART_RX_ENTRY, BAUD_RATE, PIN_ULP_RX, '\n'),
    };

    hulp_configure_pin(PIN_ULP_TX, RTC_GPIO_MODE_DISABLED, true);
    hulp_configure_pin(PIN_ULP_RX, RTC_GPIO_MODE_INPUT_ONLY, true);

    hulp_start(program, sizeof(program), 10ULL * 1000 * 1000);
}

extern "C" void app_main()
{
    startulp();

    for (;;)
    {
        if (ulp_receiveBuffer.updated())
        {
            ulp_receiveBuffer.clearUpdated();
            char name[ulp_receiveBuffer.len()];
            uint8_t nameLength = ulp_receiveBuffer.peek(name, sizeof(name));
            printf("ULP RX String: %.*s\n", nameLength, name);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}