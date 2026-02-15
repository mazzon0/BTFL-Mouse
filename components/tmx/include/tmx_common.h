#ifndef TMX_COMMON_H
#define TMX_COMMON_H

#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "esp_log.h"

#define TMX_M 4
#define TMX_N 3
#define TMX_NUM_TOUCH_PADS (TMX_M * TMX_N)


/**
 * @brief Mapping of TMX touch pads to ESP32-S3 touch pad channels.
 */
static const touch_pad_t tmx_touch_pads[TMX_NUM_TOUCH_PADS] = {
    TOUCH_PAD_NUM2,
    TOUCH_PAD_NUM3,
    TOUCH_PAD_NUM4,
    TOUCH_PAD_NUM5,
    TOUCH_PAD_NUM6,
    TOUCH_PAD_NUM7,
    TOUCH_PAD_NUM8,
    TOUCH_PAD_NUM9,
    TOUCH_PAD_NUM10,
    TOUCH_PAD_NUM11,
    TOUCH_PAD_NUM12,
    TOUCH_PAD_NUM13
};

/**
 * @brief Enumeration for different gesture types.
 */
typedef enum {
    TMX_GESTURE_NONE,
    TMX_GESTURE_BUTTON_PRESSED,
    TMX_GESTURE_BUTTON_RELEASED,
    TMX_GESTURE_SCROLL
} tmx_gesture_type_t;

/**
 * @brief Data structure for a detected gesture.
 */
typedef struct {
    tmx_gesture_type_t type;
    
    union{
        struct {
            uint16_t dx;
            uint16_t dy;
        };

        uint16_t button;
    };
} tmx_gesture_t;

/**
 * @brief Callback function type for touch events
 */
typedef void (*tmx_callback_t)(tmx_gesture_t gesture);

#endif
