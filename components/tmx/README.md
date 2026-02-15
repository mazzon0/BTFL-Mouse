# Touch Gesture Recognition driver for ESP32s3

An easy-to-use ESP_IDF component for implementing multitouch surface. This components work either at an hardware level, setting up the required drivers to make the touch pin of the ESP32-S3 working, and at a signal processing level, implementing a pipeline to identify tipical mouse interaction gestures.

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

>[!WARNING]
>
>The pin used for this driver are hardcoded in the driver to pin GPIO2 to GPIO13. This because the mutual capacitive touch driver is included with the esp32s3 with a limited number of pin. The pin GPIO1 is left free if the ADC1 is needed.
>

## TEST
In the test folder can be found two test written in python to view in realtime the pipeline at work. The aim of the test is to visually see if the data are filtered correctly ( in `raw_viewer.py`) and if blob detection is working (in `blob_viewer.py`). To run this two test Python 3.12 and some additional python module. The moudle can be found in `requirements.txt` and installed using ```pip install -r requirements.txt```
