#ifndef HOGP_CONTROL_EVENTS_H
#define HOGP_CONTROL_EVENTS_H
#include "hogp_common.h"

typedef enum {
    HOGP_CEVT_SHUTDOWN,
    HOGP_CEVT_SUSPEND,
    HOGP_CEVT_SET_PROTOCOL,
    HOGP_CEVT_CONNECT,
    HOGP_CEVT_DISCONNECT,
    HOGP_CEVT_ADV_COMPLETE,
    HOGP_CEVT_NOTIFY_TX,
    HOGP_CEVT_SUBSCRIBE,
    HOGP_CEVT_MTU,
    HOGP_CEVT_BLE_READY,
} hogp_control_event_type_t;

typedef struct {
    hogp_control_event_type_t type;

    union {
        uint8_t suspended;
        hogp_protocol_t protocol;
        uint8_t reason;
        uint8_t mtu;
        struct {
            uint16_t sub_handle;
            uint8_t sub;    // bit 0: indicate, bit 1: notify
        };
    };
} hogp_control_event_t;

#endif
