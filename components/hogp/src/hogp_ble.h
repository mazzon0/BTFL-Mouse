#ifndef HOGP_BLE_H
#define HOGP_BLE_H
#include "hogp_common.h"

/**
 * @brief Enumeration of HOGP characteristics supported by the application.
 * Used to identify which characteristic to notify or update.
 * @note The order of these values MUST match the internal boolean order in 
 * `hogp_common.h` (hogp_sub_t) to ensure correct subscription mapping.
 */
typedef enum {
    MOUSE_REPORT,   /**< The Report characteristic (Input Report) for mouse data. */
    MOUSE_BOOT,     /**< The Boot Mouse Input Report characteristic. */
} hogp_characteristics_t;

/**
 * @brief Initializes the BLE Generic Access Profile (GAP).
 * Sets the device name and appearance based on the initialization info 
 * stored in the global context.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_INTERNAL_FAIL if the device name or appearance could not be set.
 */
hogp_result_t hogp_gap_init(void);

/**
 * @brief Initializes the BLE Generic Attribute Profile (GATT).
 * Defines the services and characteristics (HID Service, Report Map, etc.) 
 * and registers them with the NimBLE stack.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_INTERNAL_FAIL if the services have an invalid definition or registration fails.
 */
hogp_result_t hogp_gatt_init(void);

/**
 * @brief Configures the NimBLE stack parameters.
 * Sets up stack callbacks (reset, sync), security capabilities (No IO), 
 * bonding flags, and Man-In-The-Middle (MITM) protection.
 * @return HOGP_OK on success.
 */
hogp_result_t hogp_nimble_config(void);

/**
 * @brief Configures and starts BLE advertising.
 * Sets advertising fields (UUIDs, Appearance) and scan response data (Device Name).
 * Uses generic discovery mode and undirected connectable mode.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_INTERNAL_FAIL if setting fields or starting advertising fails.
 */
hogp_result_t hogp_start_advertising(void);

/**
 * @brief Finalizes the connection setup when a peer connects.
 * Updates the connection parameters (interval, latency, supervision timeout) 
 * to meet the requirements for a HID device.
 * @param handle The connection handle provided by the BLE stack event.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_INTERNAL_FAIL if the connection handle is invalid or params update fails.
 */
hogp_result_t hogp_connect(uint16_t handle);

/**
 * @brief Sends a notification for a specific characteristic.
 * @param message Pointer to the raw byte array containing the report data.
 * @param size The number of bytes in the message.
 * @param chr The specific characteristic to notify (e.g., MOUSE_REPORT).
 * @return HOGP_OK on success.
 * @return HOGP_ERR_NO_MEM if memory for the mbuf could not be allocated.
 * @return HOGP_ERR_INTERNAL_FAIL if the notification could not be sent.
 */
hogp_result_t hogp_notify(uint8_t *message, uint8_t size, hogp_characteristics_t chr);    // TODO add indicate

/**
 * @brief Updates the internal subscription state for a characteristic.
 * Called when the central writes to a Client Characteristic Configuration Descriptor (CCCD).
 * @param handle The attribute handle of the characteristic being subscribed to.
 * @param subscription_type Bitmask value: 0x01 = Indicate, 0x02 = Notify.
 * @return HOGP_OK on success.
 */
hogp_result_t hogp_subscribe(uint16_t handle, uint8_t subscription_type);

#endif /* HOGP_BLE_H */

