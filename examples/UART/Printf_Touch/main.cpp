#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"

#include "hulp.h"
#include "hulp_touch.h"
#include "hulp_uart.h"

#define PIN_TOUCH GPIO_NUM_32
#define TOUCH_HYSTERESIS 10

#define PIN_UART_TX GPIO_NUM_13
#define BAUD_RATE 57600

//For ULP printf, a ulp_string_t<6> is required:
RTC_DATA_ATTR ulp_string_t<6> ulp_printf_buffer;

//Some other strings that will be used in ULP program:
RTC_DATA_ATTR ulp_string_t<4> ulp_str_prefix_start("Pin ");
RTC_DATA_ATTR ulp_string_t<13> ulp_str_prefix_end("Touch Val: ");
RTC_DATA_ATTR ulp_string_t<12> ulp_str_detection("Detection!\n");

//Somewhere to store the previous touch val for comparison:
RTC_DATA_ATTR ulp_var_t ulp_previous_touch_val;

void init_ulp()
{
    enum
    {
        LBL_PREFIX_START_OUT_RETURN,
        LBL_PRINTF_PIN_RETURN,
        LBL_PIN_OUT_RETURN,
        LBL_PREFIX_END_OUT_RETURN,
        LBL_PRINTF_VAL_RETURN,
        LBL_VAL_OUT_RETURN,
        LBL_ALERT_OUT_RETURN,
        LBL_FINISHED,

        LBL_UART_TX_ENTRY,

        LBL_PRINTF_ENTRY,
        LBL_PRINTF_CUSTOM_ENTRY,
    };

    const ulp_insn_t program[]{
        //Start a touch measurement and wait until it finishes
            M_TOUCH_BEGIN(),
            M_TOUCH_WAIT_DONE(),

        //TX the start of the prefix string
            I_MOVO(R1, ulp_str_prefix_start),
            M_RETURN(LBL_PREFIX_START_OUT_RETURN, R3, LBL_UART_TX_ENTRY),

        //TX the pin number
            //First, set R0 with the desired value.
            I_MOVI(R0, PIN_TOUCH),
            //Prepare R1 with the address of the ulp_string_t to use as the printf buffer
            I_MOVO(R1, ulp_printf_buffer),
            //By default, printf will append a line break (\n). In this case, for demonstration, a custom one defined below with a space is used instead:
            M_RETURN(LBL_PRINTF_PIN_RETURN, R3, LBL_PRINTF_CUSTOM_ENTRY),
            //And now TX it out over UART (R1 will not be altered by printf, so the address is still valid):
            M_RETURN(LBL_PIN_OUT_RETURN, R3, LBL_UART_TX_ENTRY),

        //TX the rest of the prefix string
            I_MOVO(R1, ulp_str_prefix_end),
            M_RETURN(LBL_PREFIX_END_OUT_RETURN, R3, LBL_UART_TX_ENTRY),

        //Now check if this touch < (previous - hysteresis), in which case TX an alert string
            //Get the measurement value of the touch pin into R0
            I_TOUCH_GET_GPIO_VALUE(PIN_TOUCH),
            //Prepare R1 with the buffer for printf
            I_MOVO(R1, ulp_printf_buffer),
            //Branch to the printf subroutine and return. This time use the default one (with line break):
            M_RETURN(LBL_PRINTF_VAL_RETURN, R3, LBL_PRINTF_ENTRY),
            //R1 won't be altered so can now branch immediately to the TX function
            M_RETURN(LBL_VAL_OUT_RETURN, R3, LBL_UART_TX_ENTRY),

            //Prepare R2 with zero so it can be used for I_GET/I_PUT instructions
            I_MOVI(R2, 0),

            //Load the previous value into R1
            I_GET(R1, R2, ulp_previous_touch_val),

    //Note: The value of R0 will be destroyed by the printf subroutine. If you need it afterwards, you should store it (eg. in a ulp_var_t).
    //In this case, a register holds the touch measurement value so it can simply be read into R0 again.

            //Reload the touch value into R0
            I_TOUCH_GET_GPIO_VALUE(PIN_TOUCH),
            //Store it as the 'previous' value for next time
            I_PUT(R0, R2, ulp_previous_touch_val),

            //R0 = previous - current
            I_SUBR(R0, R1, R0),
            //If that overflowed then current > previous so halt:
                M_BXF(LBL_FINISHED),
            //Else current <= previous:
                //R0 = difference - hysteresis
                I_SUBI(R0, R0, TOUCH_HYSTERESIS),
                //If that overflowed then difference < hysteresis so halt:
                    M_BXF(LBL_FINISHED),
                //Else: difference >= hysteresis so alert!
                    //Prepare R1 with the address of the detection string, and TX the message
                    I_MOVO(R1, ulp_str_detection),
                    M_RETURN(LBL_ALERT_OUT_RETURN, R3, LBL_UART_TX_ENTRY),

        //Sleep:
            M_LABEL(LBL_FINISHED),
                I_HALT(),



        
        //Include subroutine for UART TX:
            M_INCLUDE_UART_TX(LBL_UART_TX_ENTRY, BAUD_RATE, PIN_UART_TX),

        //Include subroutine for printf:
            //The default will set the last byte with a line break (\n):
            M_INCLUDE_PRINTF_U(LBL_PRINTF_ENTRY),
            //For demonstration, here's how to customise it to specify what the last byte ought to be.
            //This will set the last byte to a space ' ' instead:
            M_INCLUDE_PRINTF_U_(LBL_PRINTF_CUSTOM_ENTRY, R1, R2, R3, 1, ' '),
    };

    hulp_configure_touch(PIN_TOUCH);
    hulp_configure_pin(PIN_UART_TX, RTC_GPIO_MODE_DISABLED, true);

    hulp_ulp_load(program, sizeof(program), 200ULL * 1000);
    hulp_ulp_run();
}

extern "C" void app_main()
{
    if(!hulp_is_ulp_wakeup())
    {
        init_ulp();
    }

    esp_deep_sleep_start();
}