#include <esp_log.h>
#include <esp_sleep.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hulp.h>
#include <hulp_i2cbb.h>

static const char *TAG = "MAIN";

#define SCL_PIN GPIO_NUM_26
#define SDA_PIN GPIO_NUM_25
#define SLAVE_ADDR 0x44     // should be either 0x44 or 0x45
static gpio_num_t LED_ERR = GPIO_NUM_13;
static const uint16_t max_temp_difference_raw = 37;     // actually 37.4485 which is about 0.1C

static RTC_SLOW_ATTR ulp_var_t ulp_write_cmd[] = {
        // write 0x24, 0x00 which is high repeatability with no clock stretching
        HULP_I2C_CMD_HDR(SLAVE_ADDR, 0x24, 1),
        HULP_I2C_CMD_1B(0x00),
};
static RTC_SLOW_ATTR ulp_var_t ulp_read_cmd[HULP_I2C_CMD_BUF_SIZE(6)] = {
        HULP_I2C_CMD_HDR_NO_PTR(SLAVE_ADDR, 6),
};
static RTC_DATA_ATTR ulp_var_t last_temp;

void init_ulp() {
    enum {
        LABEL_I2C_READ,
        LABEL_I2C_READ_RETURN,
        LABEL_I2C_WRITE,
        LABEL_I2C_WRITE_RETURN,
        LABEL_I2C_ERROR,
        LABEL_NEGATIVE,
        LABEL_WAKE,
    };

    const ulp_insn_t program[] = {
            // write i2c to request measurement
            I_MOVO(R1, ulp_write_cmd),
            M_MOVL(R3, LABEL_I2C_WRITE_RETURN),
            M_BX(LABEL_I2C_WRITE),
            M_LABEL(LABEL_I2C_WRITE_RETURN),
            M_BGE(LABEL_I2C_ERROR, 1),

            // delay 13ms to get result
            M_DELAY_US_5000_20000(13000),

            // read i2c
            I_MOVO(R1, ulp_read_cmd),
            M_MOVL(R3, LABEL_I2C_READ_RETURN),
            M_BX(LABEL_I2C_READ),
            M_LABEL(LABEL_I2C_READ_RETURN),
            M_BGE(LABEL_I2C_ERROR, 1),

            // read first 2 bytes (temperature) to R1
            I_MOVI(R1,0),
            I_GET(R1, R1, ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET]),

            // read last_temp to R2
            I_MOVI(R2,0),
            I_GET(R2, R2, last_temp),

            // get difference and store in R0
            I_SUBR(R0, R1, R2),

            // deal with negative
            M_BGE(LABEL_NEGATIVE, 0x8000),

            M_BGE(LABEL_WAKE, max_temp_difference_raw),
            I_HALT(),

            M_LABEL(LABEL_NEGATIVE),
                M_BL(LABEL_WAKE, 0xFFFF - max_temp_difference_raw),
                I_HALT(),

            M_LABEL(LABEL_WAKE),
				I_WAKE(),
				I_HALT(),

            M_LABEL(LABEL_I2C_ERROR),
                I_GPIO_SET(LED_ERR, 0), // turn on led
                I_END(),                     // end ulp program so it won't run again
                I_HALT(),

            M_INCLUDE_I2CBB_CMD(LABEL_I2C_READ, LABEL_I2C_WRITE, SCL_PIN, SDA_PIN)
    };

    ESP_ERROR_CHECK(hulp_configure_pin(SCL_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(SDA_PIN, RTC_GPIO_MODE_INPUT_ONLY, GPIO_FLOATING, 0));
    ESP_ERROR_CHECK(hulp_configure_pin(LED_ERR, RTC_GPIO_MODE_OUTPUT_ONLY, GPIO_PULLDOWN_ONLY, 0));

    hulp_peripherals_on();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_ERROR_CHECK(hulp_ulp_load(program, sizeof(program), 10ULL * 1000 * 1000, 0));
    ESP_ERROR_CHECK(hulp_ulp_run(0));
}

/**
 * formulas for conversion of the sensor signals, optimized for fixed point
 * algebra:
 * Temperature = 175 * S_T / 2^16 - 45
 * Relative Humidity = 100 * S_RH / 2^16
 */
int32_t word_to_temperature(uint16_t raw) {
    return ((21875 * (int32_t) raw) >> 13) - 45000;
}

int32_t word_to_humidity(uint16_t raw) {
    return ((12500 * (int32_t) raw) >> 13);
}

void print_result() {
    const uint16_t temp = ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET].val;
    const uint16_t hum_msb = ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 1].val;
    const uint16_t hum_lsb = ulp_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 2].val;
    const uint16_t hum = (hum_msb << 8) | (hum_lsb >> 8);
    const double temp_c = word_to_temperature(temp) / 1000.0;
    const double last_temp_c = word_to_temperature(last_temp.val) / 1000.0;
    const double hum_rh = word_to_humidity(hum) / 1000.0;

    ESP_LOGI(TAG, "temp: %.1f, last_temp: %.1f, difference: %.2f, hum: %.1f",
             temp_c, last_temp_c, temp_c - last_temp_c, hum_rh);

    // would normally be done after sending e.g. using ESP-NOW
    last_temp.val = temp;
}

extern "C" void app_main(void) {

    if (hulp_is_deep_sleep_wakeup()) {
        print_result();
    } else {
        init_ulp();
    }

    ESP_LOGI(TAG, "Sleeping");
    hulp_peripherals_on();
    esp_sleep_enable_ulp_wakeup();
    esp_deep_sleep_start();
}
