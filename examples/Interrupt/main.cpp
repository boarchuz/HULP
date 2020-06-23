/* HULP RTCIO Interrupt Example

   Demonstrates the use of RTCIO interrupts to detect changes in pin state, even while the ULP is sleeping or busy. This also
   allows handling short pulses which would otherwise be difficult to detect using the ULP.

   The interrupt is processed the next time the ULP wakes (every 500ms here), toggling an LED if triggered in the previous 
   interval.
*/
#include "esp_sleep.h"

#include "hulp.h"

#define BUTTON_PIN GPIO_NUM_0
#define LED_PIN GPIO_NUM_26

#define ULP_INTERVAL_MS 500

void init_ulp()
{
    enum {
        LAB_INTERRUPT_TRIGGERED,
    };

    const ulp_insn_t program[] = {
        //Read the interrupt bit
        I_GPIO_INT_RD(BUTTON_PIN),
        //If it's set, goto LAB_INTERRUPT_TRIGGERED
        M_BGE(LAB_INTERRUPT_TRIGGERED, 1),
        //Else turn off the LED and HALT
            I_GPIO_SET(LED_PIN, 0),
            I_HALT(),
        M_LABEL(LAB_INTERRUPT_TRIGGERED),
            //Clear the interrupt bit
            I_GPIO_INT_CLR(BUTTON_PIN),
            //Turn on the LED and halt
            I_GPIO_SET(LED_PIN, 1),
            I_HALT(),
    };

    hulp_peripherals_on();
    hulp_configure_pin(BUTTON_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_PULLUP_ONLY, 0);
    hulp_configure_pin(LED_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0);
    hulp_configure_pin_int(BUTTON_PIN, GPIO_INTR_ANYEDGE);
    hulp_ulp_load(program, sizeof(program), ULP_INTERVAL_MS * 1000, 0);
    hulp_ulp_run();
}

extern "C" void app_main(void)
{
    init_ulp();
    esp_deep_sleep_start();
}
