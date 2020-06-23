#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"
#include "hulp_uart.h"

#define PIN_ULP_TX GPIO_NUM_14
#define PIN_ULP_RX GPIO_NUM_13

RTC_DATA_ATTR ulp_string_t<26> ulp_greeting("Hello, what's your name?\n");
RTC_DATA_ATTR ulp_string_t<19> ulp_introStart("Nice to meet you, ");
RTC_DATA_ATTR ulp_string_t<26> ulp_introEnd("! I'm a ULP Coprocessor.\n");
RTC_DATA_ATTR ulp_string_t<32> ulp_receiveBuffer;

#define BAUD_RATE 9600

static EventGroupHandle_t events;

void init_ulp()
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

    const ulp_insn_t program[] = {
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
        I_WAKE(),
        I_HALT(),

        //Dependencies for UART
        M_INCLUDE_UART_TX(LBL_UART_TX_ENTRY, BAUD_RATE, PIN_ULP_TX),
        M_INCLUDE_UART_RX(LBL_UART_RX_ENTRY, BAUD_RATE, PIN_ULP_RX, '\n'),
    };

    hulp_configure_pin(PIN_ULP_TX, RTC_GPIO_MODE_DISABLED, GPIO_PULLUP_ONLY);
    hulp_configure_pin(PIN_ULP_RX, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY);
    hulp_ulp_load(program, sizeof(program), 10ULL * 1000 * 1000);
    hulp_ulp_run();
}

void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR(*(TaskHandle_t*)task_handle_ptr, 0, eNoAction, NULL);
}

extern "C" void app_main()
{
    events = xEventGroupCreate();

    //ULP will trigger an interrupt when a new UART string is received. Set up here.
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    hulp_ulp_isr_register(&ulp_isr, &main_handle);
    hulp_ulp_interrupt_en();

    //Load and start the program.
    init_ulp();

    for (;;)
    {
        //Block until notified by ULP+ISR
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        
        //Get the contents of the buffer and display
        char name[ulp_receiveBuffer.len()];
        uint8_t nameLength = ulp_receiveBuffer.peek(name, sizeof(name));
        printf("ULP RX String: %.*s\n", nameLength, name);
    }
}