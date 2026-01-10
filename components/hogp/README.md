# HOGP (HID Over GATT Profile) ESP-IDF Component
An easy-to-use ESP-IDF component for implementing Bluetooth Low Energy (BLE) HID peripherals. This library abstracts the complexities of the NimBLE stack and HID report management into a simple, event-driven API.

## Features
- Simplified Initialization: Set up a fully functional BLE HID device with a single configuration struct.
- Thread-Safe API: hogp_send_data can be called from any task or ISR.
- Power efficient: only sends updates if there are changes and the host is listening.
- Mouse Input: Supports relative cursor movement, scroll wheels, and up to 8 mouse buttons.

## Installation
Create a components folder in your ESP-IDF project (if it doesn't exist). Clone or copy this repository into ```components/hogp```. The component will be automatically detected by the ESP-IDF build system via the provided CMakeLists.txt.

## Requirements
The mouse works only on Linux and Android.

ESP-IDF v5.5 or newer. It may work on older versions, but they have not been tested.

## Quick Start
Here is an simple example of a component using the HOGP component.
```main.c
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

    hogp_result_t res = hogp_setup(&hogp_init_info);
    if (res != HOGP_OK) {
        ESP_LOGE("my_project", "Failed to initialize HOGP, error code: %d ", res);
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
```
The provided component needs this CMakeLists.txt.
```CMakeLists.txt
idf_component_register(SRCS "main.c"
                       REQUIRES nvs_flash hogp)
```

## Incoming Features
Future features will be:
- support for Windows and MacOS
- selecting the actual frequency of messages (right now only the frequency of the FSM can be chosen by the user)
- composing multiple events in a single message, in order to save energy
- support for keyboard events and other types of HID events
- send the battery level to the host
- boot protocol implementation (enabling to send mouse and keyboard events to simple hosts)
- add callbacks to inform the system of upcoming events from the host