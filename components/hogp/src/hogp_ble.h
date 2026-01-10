#ifndef HOGP_BLE_H
#define HOGP_BLE_H
#include <stdint.h>

int hogp_gap_init(void);

int hogp_gatt_init(void);

int hogp_nimble_config(void);

int hogp_start_advertising(void);

int hogp_notify(uint8_t *message, uint8_t size);    // TODO add indicate

int hogp_subscribe(uint16_t handle, uint8_t subscription_type);

#endif
