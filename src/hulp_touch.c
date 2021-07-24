#include "hulp_touch.h"

#include "esp_log.h"
#include "driver/touch_pad.h"
#include "soc/touch_sensor_channel.h"
#include "soc/touch_sensor_periph.h"

#include "hulp_config.h"

static const char* TAG = "HULP-TCH";

int hulp_touch_get_pad_num(gpio_num_t pin)
{
    for(int i = 0; i < SOC_TOUCH_SENSOR_NUM; ++i)
    {
        if(touch_sensor_channel_io_map[i] == pin) return i;
    }
    ESP_LOGW(TAG, "no touch pad for gpio %d", pin);
    return -1;
}

esp_err_t hulp_configure_touch_controller(const hulp_touch_controller_config_t *config)
{
    if(!config)
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    if( 
        ESP_OK != (err = touch_pad_init()) ||
        ESP_OK != (err = touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW)) ||
        ESP_OK != (err = touch_pad_set_voltage(config->high_voltage, config->low_voltage, config->attenuation)) ||
        //sw control so sleep_cycle is irrelevant here; set to default
        ESP_OK != (err = touch_pad_set_meas_time(TOUCH_PAD_SLEEP_CYCLE_DEFAULT, config->fastclk_meas_cycles))
    )
    {
        ESP_LOGE(TAG, "[%s] err (0x%x)", __func__, err);
        return err;
    }
    return ESP_OK;
}

esp_err_t hulp_configure_touch_pin(gpio_num_t touch_gpio, const hulp_touch_pin_config_t *config)
{
    if(!config)
    {
        return ESP_ERR_INVALID_ARG;
    }

    int touch_pad_num = hulp_touch_get_pad_num(touch_gpio);
    if(touch_pad_num < 0)
    {
        ESP_LOGE(TAG, "invalid touch pin (%d)", touch_gpio);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err;
    if(
        ESP_OK != (err = touch_pad_io_init(touch_pad_num)) ||
        ESP_OK != (err = touch_pad_set_cnt_mode(touch_pad_num, config->slope, config->tie_opt)) ||
        ESP_OK != (err = touch_pad_set_group_mask(0, 0, 1 << touch_pad_num))
    )
    {
        ESP_LOGE(TAG, "[%s] err (0x%x)", __func__, err);
        return err;
    }
    return ESP_OK;
}