/* HX711 Example

   This example will read from a HX711 load cell amplifier, waking from deep sleep if the set weight threshold is met.

   Have a weight ready to use for calibration (set HX711_CALIBRATION_WEIGHT_G). Ensure there is nothing on the scales
   until prompted, then place the calibration weight down.
   Set HX711_THRESHOLD_WEIGHT_G to the minimum weight required to trigger a ULP deep sleep wakeup.
*/

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_sleep.h"

#include "hulp.h"
#include "hulp_hx711.h"

static const char *TAG = "HULP_HX711";

#define PIN_HX711_SDA   GPIO_NUM_25
#define PIN_HX711_SCL   GPIO_NUM_26

/* Specify the calibration weight here */
#define HX711_CALIBRATION_WEIGHT_G 1000.0f

/* Specify the threshold weight here. The ULP will wake the SoC when the weight exceeds this value. */
#define HX711_THRESHOLD_WEIGHT_G 500.0f

// Helpers to pass structured HX711 values between ULP and SoC
typedef struct {
    ulp_var_t high;
    ulp_var_t low;
} ulp_hx711_data_t;

static uint32_t hx711_ulp_to_val(const ulp_hx711_data_t *data)
{
    return (((uint32_t)(data->high.val) << 8) | (data->low.val));
}

static void hx711_val_to_ulp(ulp_hx711_data_t *data, const uint32_t value)
{
    data->high.val = (value >> 8) & 0xFFFF;
    data->low.val  = (value >> 0) & 0xFFFF;
}

// For the ULP to store readings
RTC_DATA_ATTR ulp_hx711_data_t hx711_val;
// For the SoC to set a wake threshold, accessible to ULP
RTC_DATA_ATTR ulp_hx711_data_t hx711_threshold;
// Keep these in RTC memory as they are required on deep sleep wakeup to convert values to weights.
RTC_DATA_ATTR uint32_t idle_value;
RTC_DATA_ATTR float cal_factor;

static void ulp_isr(void *task_handle_ptr)
{
    xTaskNotifyFromISR((TaskHandle_t)task_handle_ptr, 0, eNoAction, NULL);
}

static void init_ulp(void)
{
    // Configure pin
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_HX711_SDA,  RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(PIN_HX711_SCL, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_FLOATING, 0));
    // For the calibration phase, the ULP will send interrupts when data is ready. Prepare ISR here.
    TaskHandle_t main_handle =  xTaskGetCurrentTaskHandle();
    hulp_ulp_isr_register(&ulp_isr, main_handle);
    hulp_ulp_interrupt_en();
}

static void load_hx711_single_read_program(void)
{
    // A simple program to read the value from the HX711 when ready, then interrupt the SoC
    const ulp_insn_t program[] = {
        // Read from HX711
        M_HX711_WAIT_READY(PIN_HX711_SDA),
        M_HX711_READ(R1, R2, PIN_HX711_SDA, PIN_HX711_SCL),
        // Store and alert SoC that a new value is ready
        I_MOVI(R3, 0),
        I_PUT(R1, R3, hx711_val.high),
        I_PUT(R2, R3, hx711_val.low),
        I_WAKE(),
        // Done
        I_HALT(),
    };
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 1000, 0));
}

static void load_hx711_wake_above_threshold_program(void)
{
    // A more advanced program to compare the current value to a set threshold, waking the SoC if exceeded.
    // The HX711 is powered down when not in use for lower power consumption.
    enum {
        LBL_WAKEUP,
        LBL_SLEEP,
    };
    const ulp_insn_t program[] = {
        I_HX711_POWER_UP(PIN_HX711_SCL),

        M_HX711_WAIT_READY(PIN_HX711_SDA),
        M_HX711_READ(R1, R2, PIN_HX711_SDA, PIN_HX711_SCL),

        I_HX711_POWER_DOWN(PIN_HX711_SCL),

        I_MOVI(R3, 0),
        I_PUT(R1, R3, hx711_val.high),
        I_PUT(R2, R3, hx711_val.low),

        // Compare to threshold
        // 16-bit arithmetic, so first compare upper 16 bits
        I_GET(R0, R3, hx711_threshold.high),
        I_SUBR(R0, R1, R0),
        M_BXF(LBL_SLEEP), // if < threshold, sleep
        M_BGE(LBL_WAKEUP, 1), // if > threshold, wake
        // Else upper 16 bits equal, so need to compare lower 16 bits
        I_GET(R0, R3, hx711_threshold.low),
        I_SUBR(R0, R2, R0),
        M_BXF(LBL_SLEEP), // if < threshold, sleep
        // Else >= threshold, so wake
        M_LABEL(LBL_WAKEUP),
            M_WAKE_WHEN_READY(),
        M_LABEL(LBL_SLEEP),
            I_HALT(),
    };
    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 250 * 1000, 0));
}

static float hx711_val_to_g(uint32_t val, uint32_t idle_val, float calibration_factor)
{
    return (((int64_t)val - idle_val) / calibration_factor);
}

static uint32_t hx711_ulp_read(int samples)
{
    uint32_t total = 0;
    for(int i = 0; i < samples; ++i)
    {
        ESP_ERROR_CHECK(hulp_ulp_run_once(0));
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        uint32_t this_val = hx711_ulp_to_val(&hx711_val);
        ESP_LOGD(TAG, "Val %d/%d: %" PRIu32, i+1, samples, this_val);
        total += this_val;
        assert(total >= this_val);
    }
    return (uint32_t)((total + (samples / 2)) / samples);
}

void app_main(void)
{
    if(!hulp_is_deep_sleep_wakeup())
    {
        init_ulp();

        // Load a simple program for the ULP to read values from HX711
        load_hx711_single_read_program();

        // Take a few readings from the HX711 to set the 'idle' value when no load is on the scales
        idle_value = hx711_ulp_read(5);
        ESP_LOGI(TAG, "Tare Value: %" PRIu32, idle_value);

        // Now is the time to place the calibration weight
        ESP_LOGI(TAG, "Place the known weight (%.2fg) now. Calibrating in...", HX711_CALIBRATION_WEIGHT_G);
        for(int i = 5; i > 0; --i)
        {
            ESP_LOGI(TAG, "%d...", i);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // Get some readings from HX711 now that it is loaded with the calibration weight, and use the difference to calibrate.
        ESP_LOGI(TAG, "Calibrating...");
        uint32_t loaded_value = hx711_ulp_read(5);
        ESP_LOGI(TAG, "Loaded Value (%.2fg): %" PRIu32, HX711_CALIBRATION_WEIGHT_G, loaded_value);
        cal_factor = ((int64_t)loaded_value - idle_value) / HX711_CALIBRATION_WEIGHT_G;
        ESP_LOGI(TAG, "Calibration complete: val = %.2fg + %" PRIu32, cal_factor, idle_value);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        // Print the current value for debugging/testing for a little while
        for(int i = 0; i < 10; ++i)
        {
            ESP_LOGI(TAG, "Current Weight: %.2fg", hx711_val_to_g(hx711_ulp_read(1), idle_value, cal_factor));
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        // Set the ULP wake threshold to the threshold weight.
        uint32_t wake_threshold_value = idle_value + cal_factor * HX711_THRESHOLD_WEIGHT_G;
        ESP_LOGI(TAG, "Wake threshold (%.2fg): %" PRIu32, HX711_THRESHOLD_WEIGHT_G, wake_threshold_value);
        hx711_val_to_ulp(&hx711_threshold, wake_threshold_value);

        // Load deep sleep wakeup program
        load_hx711_wake_above_threshold_program();
    }
    else
    {
        ESP_LOGI(TAG, "Woken up! Recent: %.2fg", hx711_val_to_g(hx711_ulp_to_val(&hx711_val), idle_value, cal_factor));
    }

    ESP_ERROR_CHECK(hulp_ulp_run(0));
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    while(hx711_ulp_to_val(&hx711_val) > hx711_ulp_to_val(&hx711_threshold))
    {
        ESP_LOGI(TAG, "Waiting for weight to be released...");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Sleeping");
    hulp_peripherals_on();
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}
