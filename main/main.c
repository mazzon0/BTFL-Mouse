#include "hogp.h"

void app_main(void) {
    hogp_init_info_t hogp_init_info = {
        .device_name = "BTFL Mouse",
        .appearance = HOGP_APPEARANCE_MOUSE,
        .update_period_ms = 10,
        .n_batteries = 1,
        .flags = 0x01,
    };

    hogp_setup(&hogp_init_info);

    vTaskDelay(100000 / portTICK_PERIOD_MS);

    hogp_shutdown();
}
