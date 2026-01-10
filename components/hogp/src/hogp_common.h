#ifndef HOGP_COMMON_H
#define HOGP_COMMON_H

#include <assert.h>
#include <stdbool.h>

#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/util/util.h"
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"

#include "hogp_user_common.h"
#include "hogp_results.h"

/** @brief Tag used for ESP_LOG macros. */
#define HID_TAG "BLE HID Device"

/** @brief Helper macro to get the filename without path for logging. */
#define FILE_BASENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

/** @brief Log an error with file, line, and function context. */
#define ERROR(fmt, ...) \
    ESP_LOGE(HID_TAG, "ERROR %s:%d (%s): " fmt, \
             FILE_BASENAME, __LINE__, __func__, ##__VA_ARGS__)

/** @brief Log a warning with file, line, and function context. */
#define WARN(fmt, ...) \
    ESP_LOGW(HID_TAG, "WARN %s:%d (%s): " fmt, \
             FILE_BASENAME, __LINE__, __func__, ##__VA_ARGS__)

/** @brief Log information with file, line, and function context. */
#define INFO(fmt, ...) \
    ESP_LOGI(HID_TAG, "INFO %s:%d (%s): " fmt, \
             FILE_BASENAME, __LINE__, __func__, ##__VA_ARGS__)

/** @brief Total number of services exposed by the GATT server. */
#define HOGP_NUM_SERVICES 2  // HID Service, Device Information Service (TODO battery service)

/** @brief Total number of characteristics handles tracked. */
#define HOGP_HANDLE_COUNT 3  // Report, Boot Mouse, Keyboard (future), Battery (future)

/**
 * @brief  Structure holding the attribute handles for HOGP characteristics.
 * This union allows accessing handles either by index (for iteration) 
 * or by name (for specific logic).
 */
typedef union {
    uint16_t values[HOGP_HANDLE_COUNT]; /**< Array access to handles. */

    struct {
        uint16_t mouse_report;      /**< Handle for the Mouse Input Report. */
        uint16_t mouse_boot;        /**< Handle for the Mouse Boot Report. */
        uint16_t control_point;     /**< Handle for the HID Control Point. */
    };
} hogp_handles_t;

/**
 * @brief Union representing subscription status for characteristics.
 * Tracks which characteristics the host has enabled notifications/indications for.
 * Accessible as a boolean array or named bit-fields.
 */
typedef union { // TODO optimize booleans to single bits
    bool subs[HOGP_HANDLE_COUNT]; /**< Array access to subscription flags. */

    struct {    // this struct should respect the hogp_handles_t anonymous struct
        bool mouse_report;    /**< True if Mouse Report notifications are enabled. */
        bool mouse_boot;      /**< True if Mouse Boot notifications are enabled. */
        bool control_point; /**< True if Keyboard notifications are enabled. */
    };
} hogp_sub_t;

/**
 * @brief  HOGP Protocol Modes.
 * Defined by the HID over GATT Profile.
 */
typedef enum {
    HOGP_PROTOCOL_BOOT,   /**< Boot Protocol Mode (Simplified report format). */
    HOGP_PROTOCOL_REPORT, /**< Report Protocol Mode (Full report descriptor format). */
} hogp_protocol_t;

/**
 * @brief  Active connection context.
 * Stores all dynamic state related to the current BLE connection.
 */
typedef struct {
    ble_uuid16_t svc_uuids[HOGP_NUM_SERVICES]; /**< UUIDs of the advertised services. */
    //hogp_handles_t handles;   // Moved global in hogp_ble.c due to NimBLE stack issues
    
    hogp_sub_t indicate_sub; /**< Flags for enabled Indications. */
    hogp_sub_t notify_sub;   /**< Flags for enabled Notifications. */

    uint16_t conn_handle;    /**< The handle of the active BLE connection. */
    uint16_t mtu;            /**< Maximum Transmission Unit size for the connection. */
    uint8_t addr_val[6];     /**< Device's own MAC address. */
    uint8_t own_addr_type;   /**< Address type (Public/Random). */

    hogp_protocol_t protocol; /**< Currently active HID protocol mode. */
    bool tx_arrived;          /**< Flow control flag: True if the previous packet was acknowledged. */
} hogp_conn_t;

/**
 * @brief Device configuration data.
 * Static configuration usually loaded from init structure.
 */
typedef struct {
    char device_name[32];    /**< The name advertised by the device. */
    uint16_t appearance;     /**< The BLE appearance value (see include/hogp_user_common.h) */
} hogp_device_t;

/**
 * @brief Finite State Machine (FSM) states.
 */
typedef enum {
    HOGP_STATE_IDLE,         /**< Initial state, waiting for stack init. */
    HOGP_STATE_START,        /**< Stack ready, preparing to advertise. */
    HOGP_STATE_ADVERTISING,  /**< Advertising packets, waiting for connection. */
    HOGP_STATE_CONNECTED,    /**< Connected to a host, ready to send data. */
    HOGP_STATE_SUSPENDED,    /**< Host requested suspend (low power mode). */
    HOGP_STATE_CLOSED        /**< Application is shutting down. */
} hogp_state_t;

/**
 * @brief Stores the HID data with a current state (for example keys and buttons)
 */
typedef struct {
    uint8_t buttons;        /**< State of the buttons (1 pressed, 0 not pressed) */
} hogp_hid_state_t;

/**
 * @brief  Global HOGP Context.
 * The central singleton structure holding the entire state of the application.
 */
typedef struct {
    hogp_conn_t connection;        /**< Active connection state. */
    hogp_device_t device;          /**< Device configuration. */
    hogp_state_t state;            /**< Current FSM state. */
    QueueHandle_t control_queue;   /**< Queue for internal control events (connect, disconnect). */
    QueueHandle_t data_queue;      /**< Queue for user data events (mouse motion, clicks). */
    uint32_t update_period_ms;     /**< Main task loop delay in milliseconds. */
    hogp_hid_state_t hid_state;    /**< Current HID state */
} hogp_context_t;


/**
 * @brief  Retrieves the global HOGP context.
 * Implemented as a singleton pattern.
 * @return Pointer to the static `hogp_context_t` instance.
 */
hogp_context_t *hogp_get_context(void);

#endif /* HOGP_COMMON_H */