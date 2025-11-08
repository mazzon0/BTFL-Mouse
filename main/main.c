#include "hogp.h"

void app_main(void) {
    hogp_init_info_t hogp_init_info = {
        .n_mice = 1,
        .n_keyboards = 0,
        .n_customs = 0,
        .update_period_ms = 10,
        .appearance = HOGP_APPEARANCE_MOUSE,
        .device_name = "BTFL Mouse"
    };

    hogp_setup(&hogp_init_info);

    vTaskDelay(100000 / portTICK_PERIOD_MS);

    hogp_shutdown();
}
