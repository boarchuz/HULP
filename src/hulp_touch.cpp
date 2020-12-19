#include "hulp_touch.h"

#include "esp_log.h"
#include "driver/touch_pad.h"
#include "soc/touch_sensor_channel.h"
#include "soc/touch_sensor_periph.h"

#include "sdkconfig.h"

static const char* TAG = "HULP-TCH";

int hulp_touch_get_pad_num(gpio_num_t pin)
{
    for(int i = 0; i < SOC_TOUCH_SENSOR_NUM; ++i)
    {
        if(touch_sensor_channel_io_map[i] == pin) return i;
    }
    ESP_LOGE(TAG, "no touch pad for gpio %d", pin);
    return -1;
}

esp_err_t hulp_configure_touch_controller(uint16_t fastclk_meas_cycles, touch_high_volt_t high_voltage, touch_low_volt_t low_voltage, touch_volt_atten_t attenuation)
{
    if( ESP_OK != touch_pad_init() ||
        ESP_OK != touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW) ||
        ESP_OK != touch_pad_set_voltage(high_voltage, low_voltage, attenuation) ||
        //sw control so sleep_cycle is irrelevant here; set to default
        ESP_OK != touch_pad_set_meas_time(TOUCH_PAD_SLEEP_CYCLE_DEFAULT, fastclk_meas_cycles)
    )
    {
        ESP_LOGI(TAG, "failed configuring touch controller");
        return ESP_FAIL;
    }
    return ESP_OK;
}

esp_err_t hulp_configure_touch_pin(gpio_num_t touch_gpio, touch_cnt_slope_t slope, touch_tie_opt_t tie_opt)
{
    int touch_pad_num = hulp_touch_get_pad_num(touch_gpio);
    if(touch_pad_num < 0)
    {
        ESP_LOGE(TAG, "invalid touch pin (%d)", touch_gpio);
        return ESP_ERR_INVALID_ARG;
    }

    if( ESP_OK != touch_pad_io_init((touch_pad_t)touch_pad_num) ||
        ESP_OK != touch_pad_set_cnt_mode((touch_pad_t)touch_pad_num, slope, tie_opt)
    )
    {
        ESP_LOGE(TAG, "failed to configure touch pin %d", touch_gpio);
        return ESP_FAIL;
    }

    return touch_pad_set_group_mask(0, 0, 1 << touch_pad_num);
}