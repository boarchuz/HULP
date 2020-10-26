/* ADC Example

    This example will perform some basic ADC operations with the ULP, using 3 pins:
        PIN1: Simply read and store the value
        PIN2: Oversample and store the average value
        PIN3: Wake from deep sleep if value is below threshold
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "hulp.h"

static const char *TAG = "HULP_ADC";

#define PIN_ADC_PIN1    GPIO_NUM_32
#define PIN_ADC_PIN2    GPIO_NUM_33
#define PIN_ADC_PIN3    GPIO_NUM_25

// Pin 2 Settings
//      Set the log2 of the desired number of samples here.
//      In this example, it will be 2^3 = 8 samples. The ULP can then shift right 3 (ie. divide by 8) to find the average.
//      **Caution: Do not set higher than 4 (16 samples) else ULP arithmetic may overflow
#define PIN2_OVERSAMPLE_SHIFT 3

// Pin 3 settings
//      The internal pullup for ADC PIN3 is enabled in this example, so the idle value is ~4095 (using default 12 bit attenuation).
//      Set the threshold just a little lower, so even a small analog change will trigger a wakeup. (Try a resistor to GND, for example.)
#define PIN3_WAKE_THRESHOLD (4090)

#define ULP_WAKEUP_INTERVAL_MS (20)

RTC_DATA_ATTR struct {
    ulp_var_t pin1;
    ulp_var_t pin2;
    struct {
        ulp_var_t debug;
        ulp_var_t armed;
        struct {
            ulp_var_t reason;
            ulp_var_t counter;
        } wakeup;
    } pin3;
} ulp_vars;

void ulp_init()
{
    enum {
        LBL_PIN2_OVERSAMPLE_LOOP,
        LBL_PIN3_WAKEUP_TRIGGERED,
        LBL_HALT,
    };

    const ulp_insn_t program[] = {
        // Set a register to 0 for use with I_GET and I_PUT
        I_MOVI(R3, 0),

        // #1 : Read and store
            I_ANALOG_READ(R1, PIN_ADC_PIN1),
            I_PUT(R1, R3, ulp_vars.pin1),

        // #2 : Oversample
            I_STAGE_RST(),
            I_MOVI(R0, 0),
            M_LABEL(LBL_PIN2_OVERSAMPLE_LOOP),
                I_ANALOG_READ(R1, PIN_ADC_PIN2),
                I_ADDR(R0, R0, R1),
                I_STAGE_INC(1),
                M_BSLT(LBL_PIN2_OVERSAMPLE_LOOP, (1 << PIN2_OVERSAMPLE_SHIFT)),
            I_RSHI(R0, R0, PIN2_OVERSAMPLE_SHIFT),
            I_PUT(R0, R3, ulp_vars.pin2),
        
        // #3 : Wake if below threshold
            I_ANALOG_READ(R1, PIN_ADC_PIN3),
            I_PUT(R1, R3, ulp_vars.pin3.debug),
            // Skip if the trigger isn't 'armed'. SoC enables this before going to sleep (see below).
            I_GET(R0, R3, ulp_vars.pin3.armed),
            M_BL(LBL_HALT, 1),
            // Subtract the threshold. If it overflows, the value must be < threshold so process the event.
            I_SUBI(R0, R1, PIN3_WAKE_THRESHOLD),
            M_BXF(LBL_PIN3_WAKEUP_TRIGGERED),
            M_BX(LBL_HALT),
            M_LABEL(LBL_PIN3_WAKEUP_TRIGGERED),
                // Store the value that caused the wakeup so SoC can inspect when it's ready.
                I_PUT(R1, R3, ulp_vars.pin3.wakeup.reason),
                // Increment the counter.
                I_GET(R0, R3, ulp_vars.pin3.wakeup.counter),
                I_ADDI(R0, R0, 1),
                I_PUT(R0, R3, ulp_vars.pin3.wakeup.counter),
                // Clear 'armed'. SoC can set it again when it goes to sleep.
                I_PUT(R3, R3, ulp_vars.pin3.armed),
                M_WAKE_WHEN_READY(),

        M_LABEL(LBL_HALT),
            I_HALT(),
    };

    ESP_ERROR_CHECK(hulp_configure_analog_pin(PIN_ADC_PIN1));
    ESP_ERROR_CHECK(hulp_configure_analog_pin(PIN_ADC_PIN2));
    ESP_ERROR_CHECK(hulp_configure_analog_pin(PIN_ADC_PIN3));
    ESP_ERROR_CHECK(rtc_gpio_pullup_en(PIN_ADC_PIN3));

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1000UL * ULP_WAKEUP_INTERVAL_MS, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

extern "C" void app_main(void)
{
    if(hulp_is_deep_sleep_wakeup())
    {
        ESP_LOGI(TAG, "Woken up! PIN3: Wakeup value: %u, counter: %u", ulp_vars.pin3.wakeup.reason.val, ulp_vars.pin3.wakeup.counter.val); 
    }
    else
    {
        ulp_init();
    }
    
    // Print some values for a few seconds
    for(int i = 0; i < 5; ++i)
    {
        ESP_LOGI(TAG, "PIN1: Value: %u", ulp_vars.pin1.val);
        ESP_LOGI(TAG, "PIN2: Value: %u", ulp_vars.pin2.val);
        ESP_LOGI(TAG, "PIN3: Value: %u", ulp_vars.pin3.debug.val);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Don't go to sleep if PIN3 is still in a triggered state
    ESP_LOGI(TAG, "Waiting for PIN 3 idle...");
    while(ulp_vars.pin3.debug.val < PIN3_WAKE_THRESHOLD)
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    // Tell ULP that PIN3 is now armed, and sleep
    ESP_LOGI(TAG, "Sleeping...");
    ulp_vars.pin3.armed.val = 1;
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}
