#ifndef HOGP_COMMON_H
#define HOGP_COMMON_H

#include <assert.h>
#include <stdbool.h>

#include "esp_log.h"
#include "nvs_flash.h"

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

#define HID_TAG "BLE HID Device"
#define FILE_BASENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define ERROR(fmt, ...) \
    ESP_LOGE(HID_TAG, "ERROR %s:%d (%s): " fmt, \
             FILE_BASENAME, __LINE__, __func__, ##__VA_ARGS__)
#define WARN(fmt, ...) \
    ESP_LOGW(HID_TAG, "WARN %s:%d (%s): " fmt, \
             FILE_BASENAME, __LINE__, __func__, ##__VA_ARGS__)
#define INFO(fmt, ...) \
    ESP_LOGI(HID_TAG, "INFO %s:%d (%s): " fmt, \
             FILE_BASENAME, __LINE__, __func__, ##__VA_ARGS__)

#define HOGP_NUM_SERVICES 1
#define HOGP_HANDLE_COUNT 4

typedef union {
    uint16_t values[HOGP_HANDLE_COUNT]; 
    
    struct {
        uint16_t mouse_report;
        uint16_t mouse_boot;
        uint16_t keyboard_report;
        uint16_t battery_level;
    };
} hogp_handles_t;

_Static_assert(sizeof(hogp_handles_t) == (HOGP_HANDLE_COUNT * sizeof(uint16_t)), "hogp_sub_t can store up to 16 handles, but HOGP_HANDLE_COUNT has been defined higher");

typedef union { // TODO optimize booleans to single bits
    bool subs[HOGP_HANDLE_COUNT];

    struct {
        bool mouse_report;
        bool mouse_boot;
        bool keyboard_report;
        bool battery_level;
    };
} hogp_sub_t;

typedef enum {
    HOGP_PROTOCOL_BOOT,
    HOGP_PROTOCOL_REPORT,
} hogp_protocol_t;

typedef struct {
    ble_uuid16_t svc_uuids[HOGP_NUM_SERVICES];
    //hogp_handles_t handles;   // Moved global in hogp_ble.c, due to issued with the NimBLE stack, but conceptually they chould stay here
    hogp_sub_t indicate_sub;
    hogp_sub_t notify_sub;

    uint16_t conn_handle;
    uint16_t mtu;
    uint8_t addr_val[6];
    uint8_t own_addr_type;

    hogp_protocol_t protocol;
    bool tx_arrived;
} hogp_conn_t;

typedef enum {
    HOGP_STATE_IDLE,
    HOGP_STATE_START,
    HOGP_STATE_ADVERTISING,
    HOGP_STATE_CONNECTED,
    HOGP_STATE_SUSPENDED,
    HOGP_STATE_CLOSED
} hogp_state_t;

typedef struct {
    // OS Resources
    QueueHandle_t control_queue;
    QueueHandle_t data_queue;

    // Internal state
    hogp_conn_t connection;
    hogp_device_data_t device;
    hogp_state_t state;
    uint16_t update_period_ms;
} hogp_context_t;

hogp_context_t *hogp_get_context(void); // TODO inline?

#endif
