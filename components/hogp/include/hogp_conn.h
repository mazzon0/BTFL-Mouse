#ifndef HOGP_CONN_H
#define HOGP_CONN_H

#include "hogp.h"
#include "hogp_device.h"

/**
 * @brief Protocols for HID messages.
 */
typedef enum {
    HOGP_BOOT,
    HOGP_REPORT
} hogp_protocol_t;

/**
 * @brief Bluetooth connection data and HID over GATT Profile info.
 */
typedef struct {
    hogp_mouse_t *mice;
    hogp_keyboard_t *keyboards;
    hogp_custom_t *customs;

    volatile hogp_protocol_t protocol;
    volatile uint16_t conn_handle;
    volatile uint16_t mtu;
    uint16_t update_period_ms;
    uint16_t appearance;

    uint16_t n_mice;
    uint16_t n_keyboards;
    uint16_t n_customs;

    char device_name[HOGP_MAX_DEVICE_NAME_CHARACTERS + 1];

    volatile uint8_t flags;

    uint8_t own_addr_type;
    uint8_t addr_val[6];
} hogp_conn_t;

/**
 * @brief Masks for the connection flags.
 */
#define HOGP_CONNECTED_FLAG     1   /**< 0 if not connected, 1 if connected */
#define HOGP_CAN_SEND_FLAG      2   /**< 0 if TX message has not been received yet, 1 if it has been received */
#define HOGP_NEED_ADV_FLAG      4   /**< 0 if we don't need to start advertising, 1 if we need to start advertising */
#define HOGP_MUST_CLOSE_FLAG    8   /**< 0 in normal condition, 1 if a task requested to close the connection */
#define HOGP_UPDATED_FLAG       16  /**< 0 if no updates were received from any tasks, 1 if we have some updates to send to the HID host */
#define HOGP_TERMINATED_FLAG    32  /**< 0 if the hogp task is running, 1 if it terminated */

/**
 * @brief Initialize Bluetooth connection and HID over GATT Profile data.
 */
int hogp_conn_setup(hogp_init_info_t *init_info);

/**
 * @brief FreeRTOS task that manages the Bluetooth connection.
 */
void hogp_conn_task(void *params);

/**
 * @brief Shutdown Bluetooth connection and clear HID over GATT Profile data.
 */
int hogp_conn_shutdown(void);

#endif
