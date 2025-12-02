#include "tmx.h"
#include "tmx_driver.h"

esp_err_t tmx_init(void)
{
    return tmx_driver_init();
}

esp_err_t tmx_read_raw(uint32_t* dest, int len)
{
    if (len < TMX_NUM_TOUCH_PADS) return ESP_ERR_INVALID_SIZE;

    tmx_driver_read_raw();

    uint32_t* src = tmx_driver_get_raw_data();
    for (int i = 0; i < TMX_NUM_TOUCH_PADS; i++) {
        dest[i] = src[i];
    }

    return ESP_OK;
}
