#ifndef HOGP_BLE_H
#define HOGP_BLE_H
#include "hogp_common.h"

typedef enum {  // the order of the values MUST match the order of the boolean values is hogp_common.h -> hogp_sub_t -> anonymous struct
    MOUSE_REPORT,
    MOUSE_BOOT,
} hogp_characteristics_t;

hogp_error_t hogp_gap_init(void);

hogp_error_t hogp_gatt_init(void);

hogp_error_t hogp_nimble_config(void);

hogp_error_t hogp_start_advertising(void);

hogp_error_t hogp_connect(uint16_t handle);

hogp_error_t hogp_notify(uint8_t *message, uint8_t size, hogp_characteristics_t chr);    // TODO add indicate

hogp_error_t hogp_subscribe(uint16_t handle, uint8_t subscription_type);

#endif
