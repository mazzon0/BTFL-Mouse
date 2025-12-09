#ifndef HOGP_DEFINES_H
#define HOGP_DEFINES_H

#define HOGP_DEVICE_NAME_MAX_CHARACTERS 31

#define HOGP_APPEARANCE_MOUSE       0x03c2
#define HOGP_APPEARANCE_KEYBOARD    0x03c1
#define HOGP_APPEARANCE_CUSTOM      0x03c0

#include <stdint.h>
#include "hogp_error_codes.h"

typedef struct {
    char device_name[HOGP_DEVICE_NAME_MAX_CHARACTERS];
    uint16_t appearance;
} hogp_device_data_t;

typedef struct {
    hogp_device_data_t device_data;         /**< Appearance of the Bluetooth device to other devices. Possible values are HOGP_APPEARANCE_MOUSE, HOGP_APPEARANCE_KEYBOARD and HOGP_APPEARANCE_CUSTOM */
    uint16_t update_period_ms;              /**< Update period of the Bluetooth connection task (FreeRTOS task) in milliseconds */
} hogp_init_info_t;

#endif
