#ifndef HOGP_DATA_EVENTS_H
#define HOGP_DATA_EVENTS_H
#include "hogp_user_common.h"

typedef enum {
    HOGP_DEVT_CURSOR_MOTION,
    HOGP_DEVT_SCROLL_MOTION,
    HOGP_DEVT_MOUSE_BUTTON_PRESSED,
    HOGP_DEVT_MOUSE_BUTTON_RELEASED,
} hogp_data_event_type_t;

typedef struct {
    hogp_data_event_type_t type;

    union {
        struct {
            uint16_t x;
            uint16_t y;
        };
        uint16_t button;
    };
} hogp_data_event_t;

#endif
