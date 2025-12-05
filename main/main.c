#include "hogp.h"
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

    for (int i = 0; i < 32; i++) {
        hogp_send_data(&event);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    hogp_shutdown();
}
