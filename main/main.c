#include <stdio.h>
#include "tmx.h"

void tmx_callback(tmx_gesture_t gesture){
    // Handle the gesture event
    switch(gesture.type){
        case TMX_GESTURE_BUTTON_PRESSED:
            printf("Callback: BUTTON_PRESSED, %d\n", gesture.button);
            break;
        case TMX_GESTURE_BUTTON_RELEASED:
            printf("Callback: BUTTON_RELEASED, %d\n", gesture.button);
            break;
        case TMX_GESTURE_SCROLL:
            printf("Callback: SCROLL, dx=%d, dy=%d\n", gesture.dx, gesture.dy);
            break;
        default:
            break;
    }
}
void app_main(void)
{
    tmx_init();
    xTaskCreate(tmx_task, "tmx_event_task", 4096,(void *) tmx_callback, 5, NULL);
    
    
}