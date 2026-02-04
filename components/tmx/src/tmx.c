#include "tmx.h"
#include "tmx_processing.h"
esp_err_t tmx_init(void)
{
    return tmx_processing_init();
}

esp_err_t tmx_print(void)
{
    tmx_gesture_t gesture = tmx_pipeline_process();
    if(gesture.type != TMX_GESTURE_NONE){
        // print gesture info
        switch(gesture.type){
            case TMX_GESTURE_BUTTON_PRESSED:
                printf("GESTURE, BUTTON_PRESSED, %d\n", gesture.button);
                break;
            case TMX_GESTURE_BUTTON_RELEASED:
                printf("GESTURE, BUTTON_RELEASED, %d\n", gesture.button);
                break;
            case TMX_GESTURE_SCROLL:
                printf("GESTURE, SCROLL, dx=%d, dy=%d\n", gesture.dx, gesture.dy);
                break;
            default:
                break;
        }
    }

    return ESP_OK;
}

tmx_gesture_t tmx_pipeline_process(void)
{
    tmx_processing_raw_read();
    tmx_processing_filtering();
    tmx_processing_oversampling();
    tmx_processing_blob_detection();
    finger_rejection_filtering();
    tmx_processing_associate_blobs(get_current_time_ms());
    tmx_processing_tracker_FSM();
    tmx_gesture_t gesture = tmx_processing_detect_gestures();

    return gesture;
}

void tmx_task(void *param){
    void (*callback)(tmx_gesture_t gesture) = (void (*)(tmx_gesture_t)) param;
    while (1){
        tmx_gesture_t gesture = tmx_pipeline_process();
        if(gesture.type != TMX_GESTURE_NONE){
            callback(gesture);
        }
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}