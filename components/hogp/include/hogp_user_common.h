#ifndef HOGP_DEFINES_H
#define HOGP_DEFINES_H

/**
 * @brief Maximum length for the device name string (including terminator).
 * * Limits the size of the buffer used for the advertising name.
 */
#define HOGP_DEVICE_NAME_MAX_CHARACTERS 31

/** @brief BLE Appearance value for a generic Mouse. */
#define HOGP_APPEARANCE_MOUSE       0x03c2
/** @brief BLE Appearance value for a generic Keyboard. */
#define HOGP_APPEARANCE_KEYBOARD    0x03c1
/** @brief BLE Appearance value for a custom HID device. */
#define HOGP_APPEARANCE_CUSTOM      0x03c0

#include <stdint.h>
#include <stdbool.h>
#include "hogp_result.h"

/**
 * @brief Pointer type to a function that handles Bluetooth connection event
 */
typedef void (*hogp_connected_fn)(bool connected);

/**
 * @brief Pointer type to a function that handles Bluetooth suspension event
 */
typedef void (*hogp_suspended_fn)(bool suspended);

/**
 * @brief Configuration data defining the device's appearance and name.
 * This struct holds the static information presented to the host during scanning.
 */
typedef struct {
    char device_name[HOGP_DEVICE_NAME_MAX_CHARACTERS]; /**< The name advertised over BLE. */
    uint16_t appearance;                               /**< The BLE Appearance characteristic value. */
} hogp_device_data_t;

/**
 * @brief Initialization information structure.
 * Passed to `hogp_setup` or `hogp_context_init` to configure the library on startup.
 */
typedef struct {
    hogp_device_data_t device_data;         /**< Device appearance settings (name, category like Mouse/Keyboard). */
    uint16_t register_period_ms;            /**< The cycle time (in ms) for the HOGP FSM task (how frequently messages are registered to be sent) */
    uint16_t transmit_period_ms;            /**< The cycle time (in ms) for the NimBLE stack task (how frequently messages are sent to the host) */
    hogp_connected_fn connected_cb;         /**< Callback to handle Bluetooth connection events (set to NULL for no callback) */
    hogp_suspended_fn suspended_cb;         /**< Callback to handle Bluetooth suspension events (set to NULL for no callback) */
} hogp_init_info_t;

#endif /* HOGP_DEFINES_H */
