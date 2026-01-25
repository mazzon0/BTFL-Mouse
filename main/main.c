#include "hogp.h"
#include "tmx.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"

void tmx_callback(tmx_gesture_t gesture){
    // Handle the gesture event
    hogp_data_event_t event;

    switch(gesture.type){
        case TMX_GESTURE_BUTTON_PRESSED:
            event.type = HOGP_DEVT_MOUSE_BUTTON_PRESSED;
            event.button = gesture.button;
            ESP_LOGI("Mouse", "Sending button pressed: %d", event.button);
            hogp_send(&event);
            break;

        case TMX_GESTURE_BUTTON_RELEASED:
            event.type = HOGP_DEVT_MOUSE_BUTTON_RELEASED;
            event.button = gesture.button;
            ESP_LOGI("Mouse", "Sending button released: %d", event.button);
            hogp_send(&event);
            break;

        case TMX_GESTURE_SCROLL:
            event.type = HOGP_DEVT_SCROLL_MOTION;
            event.x = gesture.dx;
            event.y = gesture.dy;
            ESP_LOGI("Mouse", "Sending scroll: %d, %d", event.x, event.y);
            hogp_send(&event);
            break;

        default:
            break; 
    }
}



void app_main(void) {
    // Init the NVS flash (required for Bluetooth bonding)
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

    hogp_result_t res = hogp_setup(&hogp_init_info);
    if (res != HOGP_OK) {
        ESP_LOGE("my_project", "Failed to initialize HOGP, error code: %d ", res);
    }

    // Init touch driver and task
    tmx_init();
    xTaskCreate(tmx_task, "tmx_event_task", 4096,(void *) tmx_callback, 5, NULL);

    

    // Shutdown HOGP component
    //hogp_shutdown();
}