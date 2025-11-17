#ifndef HOGP_H
#define HOGP_H

#include "hogp_common.h"

/**
 * @brief Data needed to initialize the HID over GATT Bluetooth Profile.
 * The device will present itself with a name and an appearance, that are not restrictive of the services it will provide (can be chosen freely).
 */
typedef struct {
    char device_name[HOGP_MAX_DEVICE_NAME_CHARACTERS + 1];  /**< Name of the Bluetooth device */
    uint16_t appearance;        /**< Appearance of the Bluetooth device to other devices. Possible values are HOGP_APPEARANCE_MOUSE, HOGP_APPEARANCE_KEYBOARD and HOGP_APPEARANCE_CUSTOM */
    uint16_t update_period_ms;  /**< Update period of the Bluetooth connection task (FreeRTOS task) in milliseconds */
    uint8_t n_batteries;
    uint8_t flags;
} hogp_init_info_t;

/**
 * @brief Setup HID over GATT Bluetooth Profile.
 * This function setup all structures necessary for the Bluetooth connection and the HID over GATT Profile (as described by the parameter init_info).
 * Then it will create a FreeRTOS task that manages connections and messages.
 * Do not call this function if hogp_shutdown(void) was called and has not terminated.
 * @param init_info Pointer to a hogp_init_info_t struct, containing all information necessary to setup the Bluetooth connection and HOG Profile.
 * @return ESP error code. In order to havo more details about errors, check the logs.
 */
esp_err_t hogp_setup(hogp_init_info_t *init_info);  // TODO add callbacks

/**
 * @brief Shutdown HID over GATT Bluetooth Profile.
 * Do not call this function if hogp_setup(hogp_init_info_t *init_info) was called and has not terminated.
 * @return ESP error code. In order to havo more details about errors, check the logs.
 */
esp_err_t hogp_shutdown(void);

#endif
