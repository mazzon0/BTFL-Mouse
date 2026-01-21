# TITLE

An easy-to-use ESP_IDF component for implementing multitouch surface. This components work either at an hardware level, setting up the required drivers to make the touch pin of the ESP32-S3 working, and at a signal processing leve, implementing a pipeline to identify tipical mouse interaction gestures.

## Features
Main features
- Finger rejection
- Mouse input: Supports scroll wheels, right and left click

Signal processing features
- noise cancelling
- blob detection
- finger traking

# Installation
Create a components floder in your ESP-IDF project (if it doesn't exist). Clone or copy this repository into `component/tmx`. The component will be automatically detected by the ESP-IDF build system via the provided `CmakeList.txt`

# Requirements
ESP-IDF v5.5 or newer. It may work on older versions, but they have not been tested

# Quick Start
Here is a simple example of component usage.

```c
#include "tmx.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

void tmx_callback(tmx_gesture_t gesture){
    // Handle the gesture event
    switch(gesture.type){
        case TMX_GESTURE_BUTTON_PRESSED:
            //implement
            break;

        case TMX_GESTURE_BUTTON_RELEASED:
            //implement
            break;

        case TMX_GESTURE_SCROLL:
            //implement
            break;

        default:
            break;
    }
}

void app_main(void) {

    // Init touch driver and task
    tmx_init();
    xTaskCreate(tmx_task, "tmx_event_task", 4096,(void *) tmx_callback, 5, NULL);

}
```