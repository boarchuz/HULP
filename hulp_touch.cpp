#include "hulp_touch.h"

#include "esp_log.h"
#include "driver/touch_pad.h"
#include "soc/touch_channel.h"

static const char* TAG = "HULP-TCH";

esp_err_t hulp_configure_touch(gpio_num_t touch_gpio)
{
    touch_pad_t pad = gpio_to_touch_num[touch_gpio];
    if(pad == TOUCH_PAD_NO_CHANNEL)
    {
        ESP_LOGE(TAG, "invalid touch pad gpio (%d)", touch_gpio);
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = touch_pad_init();
    if(err != ESP_OK)
    {
        return err;
    }
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    touch_pad_io_init(pad);
    touch_pad_set_cnt_mode(pad, TOUCH_PAD_SLOPE_7, TOUCH_PAD_TIE_OPT_LOW );
    err = touch_pad_set_group_mask(0, 0, (1 << pad));
    return err;
}