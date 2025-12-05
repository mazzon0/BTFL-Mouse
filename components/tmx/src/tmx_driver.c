#include "tmx_driver.h"

static const char *TAG = "tmx_driver";

esp_err_t tmx_driver_init(void)
{
    ESP_ERROR_CHECK(touch_pad_init());

    for (int i = 0; i < TMX_NUM_TOUCH_PADS; i++) {
        ESP_ERROR_CHECK(touch_pad_config(tmx_touch_pads[i]));
    }

    ESP_ERROR_CHECK(touch_pad_set_voltage(
        TOUCH_HVOLT_2V7, 
        TOUCH_LVOLT_0V5, 
        TOUCH_HVOLT_ATTEN_1V
    ));
    ESP_ERROR_CHECK(touch_pad_set_charge_discharge_times(0x0400));

    ESP_ERROR_CHECK(touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER));
    ESP_ERROR_CHECK(touch_pad_fsm_start());

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Touch driver initialized");
    return ESP_OK;
}

void tmx_driver_read_raw(uint32_t* dest)
{
    for (int i = 0; i < TMX_NUM_TOUCH_PADS; i++) {
        touch_pad_read_raw_data(tmx_touch_pads[i], &dest[i]);
    }
}
