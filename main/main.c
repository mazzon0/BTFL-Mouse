#include "hogp.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"

void app_main(void) {
    // Init the NVS flash (required for bonding)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE("my_project", "Failed to initialize nvs flash, error code: %d ", ret);
    }

    // Init the HOGP component
    hogp_init_info_t hogp_init_info = {
        .device_data = (hogp_device_data_t) {
            .device_name = "BTFL Mouse",
            .appearance = HOGP_APPEARANCE_MOUSE,
        },
        .update_period_ms = 10,
    };

    if (hogp_setup(&hogp_init_info) == HOGP_OK) {
        printf("HOGP Initialized and Advertising...\n");
    }

    // Send events to the Bluetooth host
    hogp_data_event_t event;
    event.type = HOGP_DEVT_CURSOR_MOTION;
    event.x = 16;
    event.y = 16;

    for (int i = 0; i < 64; i++) {
        hogp_send(&event);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    // Shutdown HOGP component
    hogp_shutdown();
}
