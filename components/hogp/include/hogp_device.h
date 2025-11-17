#ifndef HOGP_DEVICE_H
#define HOGP_DEVICE_H

#include "hogp_common.h"

/**
 * @brief This file defines an abstraction over an HID Device and its GATT services.
 */

#define N_CHARACTERISTICS   16
#define MAX_SERVICES        16

/**
 * @brief Data for a BLE Characteristics.
 */
typedef struct {
    uint16_t handle;
    bool subscribed;
} hogp_characteristic_t;

/**
 * @brief Data for an HID device.
 */
typedef struct {
    hogp_characteristic_t characteristics[N_CHARACTERISTICS];
    struct ble_gatt_svc_def *services;
    uint16_t *service_uuids;
    int (*suspend_cb)(bool);
    uint16_t conn_handle;
    uint8_t n_services;
    uint8_t n_batteries;
    uint8_t flags;
} hogp_hid_device_t;

/**
 * @brief Data for initializing an HID device.
 */
typedef struct {
    int (*suspend_cb)(bool);
    uint16_t conn_handle;
    uint8_t n_batteries;
    uint8_t flags;
} hogp_hid_device_init_info_t;

/**
 * @brief Masks for the flags field of hogp_hid_device_init_into_t.
 */
#define HOGP_MOUSE_DEVICE       1   /**< 0 if the device should contain no mouse data, 1 otherwise */
#define HOGP_KEYBOARD_DEVICE    2   /**< 0 if the device should contain no keyboard data, 1 otherwise */
#define HOGP_CUSTOM_DEVICE      4   /**< 0 if the device should contain no custom hid data, 1 otherwise */

/**
 * @brief Init HID device data.
 */
int hogp_hid_device_setup(hogp_hid_device_init_info_t *init_info);

/**
 * @brief Shutdown HID device data.
 */
int hogp_hid_device_shutdown(void);

/**
 * @brief Get the array of UUIDs representing the services the device can provide.
 * @return The array of UUIDs (to understand the meaning of the UUIDs, check the SIG Assigned Numbers)
 */
const ble_uuid16_t *const hogp_device_get_services_uuids(void);

/**
 * @brief Get the length of the array returned from hogp_device_get_service.
 * @return Unsigned integer with the count of services.
 */
uint16_t hogp_device_get_services_count(void);

/**
 * @brief Get the array of GATT services definitions.
 */
const struct ble_gatt_svc_def *const hogp_device_get_services_defs(void);

#endif
