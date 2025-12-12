#include "hogp.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"


void app_main(void) {
    hogp_init_info_t hogp_init_info = {
        .device_data = (hogp_device_data_t) {
            .device_name = "BTFL Mouse",
            .appearance = HOGP_APPEARANCE_MOUSE,
        },
        .update_period_ms = 10,
    };

    hogp_setup(&hogp_init_info);

    hogp_data_event_t event;
    event.type = HOGP_DEVT_CURSOR_MOTION;
    event.x = 16;
    event.y = 16;

    for (int i = 0; i < 64; i++) {
        hogp_send_data(&event);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /*for (int i = 0; i < 64; i++) {
        hogp_data_event_t event = {0};
        event.type = HOGP_DEVT_MOUSE_BUTTON_PRESSED;
        event.button = 2;
        hogp_send_data(&event);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        event.type = HOGP_DEVT_MOUSE_BUTTON_RELEASED;
        event.button = 2;
        hogp_send_data(&event);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }*/

    ESP_LOGI("main", "finishing");

    hogp_shutdown();

    ESP_LOGI("main", "finished");
}
