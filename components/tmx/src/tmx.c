#include "tmx.h"
#include "tmx_processing.h"
esp_err_t tmx_init(void)
{
    return tmx_processing_init();
}

esp_err_t tmx_read_raw()
{
    tmx_processing_print();

    return ESP_OK;
}
