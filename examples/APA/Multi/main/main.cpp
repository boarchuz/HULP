#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"
#include "soc/rtc.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_apa.h"

#define SCL_PIN GPIO_NUM_14
#define SDA_PIN GPIO_NUM_13
#define NUM_LEDS 5

//The ULP will set the first LED to these values:
#define ULP_SET_BRIGHTNESS 10
#define ULP_SET_BLUE 0
#define ULP_SET_RED 255
#define ULP_SET_GREEN 255

//Declare the array of ulp_apa_t in RTC Slow Memory
RTC_DATA_ATTR ulp_apa_t leds[NUM_LEDS];

void init_ulp()
{
    enum {
        LBL_APA_TX_RETURN,
        LBL_APA_ENTRY,
    };

    const ulp_insn_t program[] = {
        I_MOVI(R1, 0),
        //Brightness and blue go in MSB
        I_MOVI(R0, (ULP_SET_BRIGHTNESS << 8 | ULP_SET_BLUE)),
        I_PUT(R0, R1, leds[0].msb),
        //Green and red go in LSB
        I_MOVI(R0, (ULP_SET_GREEN << 8 | ULP_SET_RED)),
        I_PUT(R0, R1, leds[0].lsb),

        //Ready to transmit, prepare R3 with the return address.
        M_MOVL(R3, LBL_APA_TX_RETURN),
        //Then branch to the APA TX subroutine entry
        M_BX(LBL_APA_ENTRY),
        //Returns here
        M_LABEL(LBL_APA_TX_RETURN),

        //Notify SoC that the TX is done, and sleep
        I_WAKE(),
        I_HALT(),

        //Subroutine:
        M_APA_TX(LBL_APA_ENTRY, SCL_PIN, SDA_PIN, leds, NUM_LEDS, R1, R3),
    };

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 250 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR(*(TaskHandle_t*)task_handle_ptr, 0, eNoAction, NULL);
}

extern "C" void app_main()
{
    //ULP will trigger an interrupt when it finishes transmitting. Randomise the leds each time.
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    hulp_ulp_isr_register(&ulp_isr, &main_handle);
    hulp_ulp_interrupt_en();

    init_ulp();

    for(;;)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        esp_fill_random(leds, sizeof(leds));
    }
}