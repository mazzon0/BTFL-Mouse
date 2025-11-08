#ifndef HOGP_COMMON_H
#define HOGP_COMMON_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#define HID_TAG "BLE HID Device"
#define BLE_UUID_HID_SERVICE 0x1124

#define HOGP_MAX_DEVICE_NAME_CHARACTERS 31

#define HOGP_APPEARANCE_MOUSE       0x03c2
#define HOGP_APPEARANCE_KEYBOARD    0x03c1
#define HOGP_APPEARANCE_CUSTOM      0x03c0

//inline void hogp_format_addr(char* addr_str, uint8_t addr[]);

static inline void hogp_format_addr(char* addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}

#endif
