#include "tmx.h"
#include "tmx_driver.h"

esp_err_t tmx_init(void)
{
    return tmx_driver_init();
}

esp_err_t tmx_read_raw(uint32_t* dest, int len)
{
    if (len < TMX_NUM_TOUCH_PADS) return ESP_ERR_INVALID_SIZE;

    tmx_driver_read_raw(dest);

    return ESP_OK;
}
