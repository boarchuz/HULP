/* HULP Buttons Example

   Example of using the ULP to handle more advanced button input, including multiple input events - single click, double click, and long hold -
   configurable interrupts, multiple buttons simultaneously, and debouncing of pin changes.

   Attach four buttons (or equivalent) across GND to GPIOs 0, 25, 26 and 27.
   Click, double click, and hold to log different events. Hold 0 to enter/exit deep sleep.
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "ulp_buttons.h"

#include "sdkconfig.h"

static const char *TAG = "buttons";

// Array of RTC GPIOs with an attached button for ULP to monitor
const gpio_num_t button_gpios[] = {GPIO_NUM_0, GPIO_NUM_25, GPIO_NUM_26, GPIO_NUM_27};

#define BUTTON_DEBOUNCE_MS 20
#define BUTTON_DOUBLE_CLICK_TIMEOUT_MS 200
#define BUTTON_HOLD_MS 5000
// To ensure edge race conditions are handled, ULP will interrupt the SoC repeatedly at this interval until events are acknowledged:
#define BUTTON_REINTERRUPT_MS 100
#define ULP_WAKEUP_INTERVAL_US (5 * 1000)

#define NUM_BUTTONS (sizeof(button_gpios)/sizeof(button_gpios[0]))

// Button states (in RTC slow memory)
static RTC_SLOW_ATTR ulp_button_t ulp_button_states[NUM_BUTTONS];

void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR(*(TaskHandle_t*)task_handle_ptr, 0, eNoAction, NULL);
}

void app_main()
{
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    ESP_ERROR_CHECK(hulp_ulp_isr_register(&ulp_isr, &main_handle));
    hulp_ulp_interrupt_en();

    if(!hulp_is_ulp_wakeup())
    {
        ESP_ERROR_CHECK(ulp_buttons_init(button_gpios, ulp_button_states, NUM_BUTTONS, ULP_WAKEUP_INTERVAL_US, BUTTON_DEBOUNCE_MS, BUTTON_DOUBLE_CLICK_TIMEOUT_MS, BUTTON_HOLD_MS, BUTTON_REINTERRUPT_MS));
    }
    else
    {
        ESP_LOGI(TAG, "Woken up! Sleep events:");
        for(int i = 0; i < NUM_BUTTONS; ++i)
        {
            ESP_LOGI(TAG, "\tButton %d:", button_gpios[i]);
            ESP_LOGI(TAG, "\t\tClicks: %u", ulp_button_process_single_clicks(&ulp_button_states[i]));
            ESP_LOGI(TAG, "\t\tDouble clicks: %u", ulp_button_process_double_clicks(&ulp_button_states[i]));
            ESP_LOGI(TAG, "\t\tHolds: %u", ulp_button_process_holds(&ulp_button_states[i]));
        }
    }

    // Enable interrupts for all button events
    for(int i = 0; i < NUM_BUTTONS; ++i)
    {
        ulp_button_interrupt_config(&ulp_button_states[i], UINT16_MAX);
    }

    // Log events until 0 is held to go to sleep
    ESP_LOGI(TAG, "Hold %d to sleep.", button_gpios[0]);
    bool goto_sleep = false;
    for(;;)
    {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        for(int i = 0; i < NUM_BUTTONS; ++i)
        {
            if(ulp_button_process_single_clicks(&ulp_button_states[i]))
            {
                ESP_LOGI(TAG, "%d click", button_gpios[i]);
            }
            if(ulp_button_process_double_clicks(&ulp_button_states[i]))
            {
                ESP_LOGI(TAG, "%d double click", button_gpios[i]);
            }
            if(ulp_button_process_holds(&ulp_button_states[i]))
            {
                ESP_LOGI(TAG, "%d hold", button_gpios[i]);
                if(i == 0)
                {
                    goto_sleep = true;
                }
            }
        }
        if(goto_sleep)
        {
            ESP_LOGI(TAG, "Going to sleep. Hold %d to wake up.", button_gpios[0]);
            // Enable only hold interrupt for 0
            ulp_button_interrupt_config(&ulp_button_states[0], ULP_BUTTON_INT_HOLD);
            // Disable all other interrupts
            for(int i = 1; i < NUM_BUTTONS; ++i)
            {
                ulp_button_interrupt_config(&ulp_button_states[i], 0);
            }
            esp_sleep_enable_ulp_wakeup();
            esp_deep_sleep_start();
        }
    }
}
