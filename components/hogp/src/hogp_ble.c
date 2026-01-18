#include "hogp_ble.h"
#include "hogp_common.h"
#include "hogp_control_events.h"
#include <string.h>
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

// UUIDs from the SIG Assigned Numbers
#define BLE_UUID_SVC_HID                        0x1812
#define BLE_UUID_SVC_DEVINFO                    0x180A
#define BLE_UUID_CHR_HID_INFORMATION            0x2A4A
#define BLE_UUID_CHR_REPORT_MAP                 0x2A4B
#define BLE_UUID_CHR_HID_CONTROL_POINT          0x2A4C
#define BLE_UUID_CHR_REPORT                     0x2A4D
#define BLE_UUID_CHR_PROTOCOL_MODE              0x2A4E
#define BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT    0x2A33
#define BLE_UUID_CHR_PNP_ID                     0x2A50
#define BLE_UUID_DSC_REPORT_REF                 0x2908

const ble_uuid16_t hid_service_uuid = BLE_UUID16_INIT(BLE_UUID_SVC_HID);
const ble_uuid16_t device_info_service_uuid = BLE_UUID16_INIT(BLE_UUID_SVC_DEVINFO);
const ble_uuid16_t hid_info_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_HID_INFORMATION);
const ble_uuid16_t report_map_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_REPORT_MAP);
const ble_uuid16_t hid_control_point_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_HID_CONTROL_POINT);
const ble_uuid16_t protocol_mode_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_PROTOCOL_MODE);
const ble_uuid16_t mouse_report_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_REPORT);
const ble_uuid16_t mouse_boot_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT);
const ble_uuid16_t pnp_id_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_PNP_ID);
const ble_uuid16_t report_ref_mouse_in_uuid = BLE_UUID16_INIT(BLE_UUID_DSC_REPORT_REF);

hogp_handles_t handles;     // handles for the ble definitions

// Protocol definitions
#define HOGP_USB_VERSION_MAJOR_BCD  0x01
#define HOGP_USB_VERSION_MINOR_BCD  0x11

// NimBLE Prototypes
void ble_store_config_init(void);

// NimBLE callbacks
static void ble_stack_reset_callback(int reason);
static void ble_stack_sync_callback(void);
static int gap_event_callback(struct ble_gap_event *event, void *arg);

// Utility functions
static void set_service_uuids(void);

// Host access callbacks
static int hid_info_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_control_point_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_protocol_mode_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_map_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_boot_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int pnp_id_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_ref_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Services definition
static const struct ble_gatt_svc_def services[] = {
    // HID Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *) &hid_service_uuid,
        .characteristics = (struct ble_gatt_chr_def[]) {
            // HID Information
            {
                .uuid = (const ble_uuid_t *) &hid_info_uuid,
                .access_cb = hid_info_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_READ_ENC,
            },
            // HID Control Point
            {
                .uuid = (const ble_uuid_t *) &hid_control_point_uuid,
                .access_cb = hid_control_point_access_cb,
                .flags = BLE_GATT_CHR_F_WRITE_NO_RSP | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_ENC,
                .val_handle = &handles.control_point,
            },
            // Protocol Mode
            {
                .uuid = (const ble_uuid_t *) &protocol_mode_uuid,
                .access_cb = hid_protocol_mode_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            // Report Map
            {
                .uuid = (const ble_uuid_t *) &report_map_uuid,
                .access_cb = hid_report_map_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            // Report Mouse
            {
                .uuid = (const ble_uuid_t *) &mouse_report_uuid,
                .access_cb = hid_report_mouse_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &handles.mouse_report,
                .descriptors = (struct ble_gatt_dsc_def[]) {
                    {
                        .uuid = (const ble_uuid_t *) &report_ref_mouse_in_uuid,
                        .access_cb = hid_report_ref_mouse_access_cb,
                        .att_flags = BLE_ATT_F_READ,
                    },
                    { 0 }
                }
            },
            // Boot Mouse
            {
                .uuid = (const ble_uuid_t *) &mouse_boot_uuid,
                .access_cb = hid_boot_mouse_access_cb,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &handles.mouse_boot,
            },
            // End of characteristics
            { 0 }
        },
    },
    // Device Information Service
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *) &device_info_service_uuid,
        .characteristics = (struct ble_gatt_chr_def[]) {
            // PnP ID
            {
                .uuid = (const ble_uuid_t *) &pnp_id_uuid,
                .access_cb = pnp_id_access_cb,
                .flags = BLE_GATT_CHR_F_READ,
            },
            {
                0
            }
        }
    },
    {
        0
    }
};


// Public functions

hogp_result_t hogp_gap_init(void) {
    int rc = 0;
    hogp_context_t *ctx = hogp_get_context();

    INFO("Initializing GAP");

    ble_svc_gap_init();

    rc = ble_svc_gap_device_name_set(ctx->device.device_name);
    if (rc != 0) {
        ERROR("Failed to set the device name to '%s'. Got the error code %d", ctx->device.device_name, rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    rc = ble_svc_gap_device_appearance_set(ctx->device.appearance);
    if (rc != 0) {
        ERROR("Failed to set the device appearance to %d. Got the error code %d", ctx->device.appearance, rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    return HOGP_OK;
}

hogp_result_t hogp_gatt_init(void) {
    hogp_context_t *ctx = hogp_get_context();
    int rc = 0;

    INFO("Initializing GATT");
    ble_svc_gatt_init();

    set_service_uuids();

    rc = ble_gatts_count_cfg(services);
    if (rc != 0) {
        ERROR("GATT services have an invalid definition. Error code: %d", rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        ERROR("GATT services not registered. Error code: %d", rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    return HOGP_OK;
}

hogp_result_t hogp_nimble_config(void) {
    ble_hs_cfg.reset_cb = ble_stack_reset_callback;
    ble_hs_cfg.sync_cb = ble_stack_sync_callback;
    //ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb; // TODO
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; // or BLE_SM_IO_CAP_DISPLAY_YESNO for more security
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;

    ble_store_config_init();

    return HOGP_OK;
}

hogp_result_t hogp_start_advertising(void) {
    int rc = 0;
    const char *name;

    hogp_context_t *ctx = hogp_get_context();

    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.appearance = ctx->device.appearance;
    adv_fields.appearance_is_present = 1;
    adv_fields.uuids16 = ctx->connection.svc_uuids;
    adv_fields.num_uuids16 = HOGP_NUM_SERVICES;
    adv_fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0) {
        ERROR("Failed to set advertising data, error core %d", rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ERROR("Failed to set scan response data, error code: %d", rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    rc = ble_gap_adv_start(ctx->connection.own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_callback, NULL);
    if (rc != 0) {
        ERROR("Start advertising failed, error code: %d", rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    INFO("Advertising started successfully");

    return HOGP_OK;
}

hogp_result_t hogp_connect(uint16_t handle) {
    hogp_context_t *ctx = hogp_get_context();
    struct ble_gap_conn_desc desc;
    int rc = ble_gap_conn_find(handle, &desc);
    if (rc != 0) {
        ERROR("Failed to find connection by handle, error code: %d", rc);
        return HOGP_ERR_INTERNAL_FAIL;
    }

    ctx->connection.conn_handle = handle;
    INFO("Connection parameters: itvl (%d), superivsion timeout: (%d)", desc.conn_itvl, desc.supervision_timeout);

    return HOGP_OK;
}

hogp_result_t hogp_notify(uint8_t *message, uint8_t size, hogp_characteristics_t chr) {
    hogp_context_t *ctx = hogp_get_context();
    struct os_mbuf *om = ble_hs_mbuf_from_flat(message, sizeof(uint8_t) * size);
    
    // Check for allocation failure
    if (om == NULL) {
        ERROR("No memory available for notification mbuf (size: %d)", size);
        return HOGP_ERR_NO_MEM;
    }
    
    // Send notification
    int rc = ble_gatts_notify_custom(ctx->connection.conn_handle, handles.values[(int)chr], om);
    
    if (rc == 0) {
        INFO("Notification sent successfully (Type: %d)", chr);
    } else {
        WARN("Notification send failed. Error code: %d, Handle: %d", rc, ctx->connection.conn_handle);
        //os_mbuf_free_chain(om); // NimBLE frees on error for notify_custom? Check API docs usually
        return HOGP_ERR_INTERNAL_FAIL;
    }
    return HOGP_OK;
}

hogp_result_t hogp_subscribe(uint16_t handle, uint8_t subscription_type) {
    hogp_context_t *ctx = hogp_get_context();

    int i;
    for (i = 0; i < HOGP_HANDLE_COUNT; i++) {
        if (handles.values[i] == handle) break;
    }

    if (i == HOGP_HANDLE_COUNT) {
        WARN("Received subscription for unknown handle: %d", handle);
        return HOGP_OK; 
    }

    if (subscription_type & 1)  ctx->connection.indicate_sub.subs[i] = true;
    else                        ctx->connection.indicate_sub.subs[i] = false;
    
    if (subscription_type & 2)  ctx->connection.notify_sub.subs[i]   = true;
    else                        ctx->connection.notify_sub.subs[i]   = false;

    return HOGP_OK;
}



// Utility functions

/**
 * @brief Populates the service UUIDs array in the context.
 * Currently sets the HID Service UUID.
 */
static void set_service_uuids(void) {
    hogp_context_t *ctx = hogp_get_context();

    ctx->connection.svc_uuids[0] = (ble_uuid16_t) BLE_UUID16_INIT(BLE_UUID_SVC_HID);
    ctx->connection.svc_uuids[1] = (ble_uuid16_t) BLE_UUID16_INIT(BLE_UUID_SVC_DEVINFO);
}

/**
 * @brief Helper to format a raw 6-byte MAC address into a readable string.
 * @param addr_str Buffer to hold the resulting string (must be at least 18 chars).
 * @param addr Array of 6 bytes representing the MAC address.
 */
static inline void hogp_format_addr(char* addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}




// NimBLE callbacks

/**
 * @brief Callback invoked by NimBLE when the stack resets.
 * Log useful information for debugging (e.g., stack crash or manual reset).
 * @param  reason  The error code indicating why the reset occurred.
 */
static void ble_stack_reset_callback(int reason) {
    INFO("NimBLE stack reset triggered. Reason: %d", reason);
}

/**
 * @brief Callback invoked when the NimBLE stack is synced and ready.
 * Infers the device's own address.
 * Sends the HOGP_CEVT_BLE_READY event to the FSM to trigger the application start.
 */
static void ble_stack_sync_callback(void) {
    hogp_context_t *ctx = hogp_get_context();
    int rc;
    char addr_str[18] = {0};

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ERROR("No available Bluetooth address found on device!");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &ctx->connection.own_addr_type);
    if (rc != 0) {
        ERROR("Failed to infer address type. Error code: %d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(ctx->connection.own_addr_type, ctx->connection.addr_val, NULL);
    if (rc != 0) {
        ERROR("Failed to copy device address. Error code: %d", rc);
        return;
    }
    hogp_format_addr(addr_str, ctx->connection.addr_val);
    INFO("Device Address Synced: %s", addr_str);

    // Send BLE ready event
    hogp_control_event_t event;
    event.type = HOGP_CEVT_BLE_READY;

    if (xQueueSendToBackFromISR(ctx->control_queue, &event, 0) != pdPASS) {
        ERROR("Critical: Failed to enqueue BLE_READY event (Queue full)");
    }
}

/**
 * @brief  The central GAP event handler.
 * Handles connection, disconnection, advertising completion, and subscription updates.
 * Translates raw BLE events into `hogp_control_event_t` for the FSM queue.
 * @param  event  The event structure provided by NimBLE.
 * @param  arg    User argument (unused).
 * @return 0 on success, or a BLE error code.
 */
static int gap_event_callback(struct ble_gap_event *event, void *arg) {
    int rc = 0;
    struct ble_gap_conn_desc desc;
    hogp_control_event_t e;
    hogp_context_t *ctx = hogp_get_context();

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        INFO("Connection attempt %s; Status=%d, Handle=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status,
                 event->connect.conn_handle);

        if(event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ERROR("Failed to find connection descriptor by handle. Error code: %d", rc);
                return rc;
            }

            // Connect event
            e.type = HOGP_CEVT_CONNECT;
            e.conn_handle = event->connect.conn_handle;
            if (xQueueSendToBackFromISR(ctx->control_queue, &e, 0) != pdPASS) {
                ERROR("Failed to enqueue CONNECT event (Queue full)");
                return -1;
            }
        }
        return rc;

    case BLE_GAP_EVENT_DISCONNECT:
        INFO("Disconnected from peer. Reason=%d", event->disconnect.reason);

        e.type = HOGP_CEVT_DISCONNECT;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 0) != pdPASS) {
            ERROR("Failed to enqueue DISCONNECT event (Queue full)");
            return -1;
        }
        
        return rc;

    case BLE_GAP_EVENT_CONN_UPDATE:
        INFO("Connection parameters updated. Status=%d", event->conn_update.status);

        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);  // TODO handle in the FSM or remove
        if (rc != 0) {
            ERROR("Failed to find connection descriptor after update. Error code: %d", rc);
            return rc;
        }

        return rc;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        INFO("Advertising completed/stopped. Reason=%d", event->adv_complete.reason);

        e.type = HOGP_CEVT_ADV_COMPLETE;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 0) != pdPASS) {
            ERROR("Failed to enqueue ADV_COMPLETE event (Queue full)");
            return -1;
        }
        return rc;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status == 0) {
            e.type = HOGP_CEVT_NOTIFY_TX;
            if (xQueueSendToBackFromISR(ctx->control_queue, &e, 0) != pdPASS) {
                ERROR("Failed to enqueue NOTIFY_TX event (Queue full)");
                return -1;
            }
        }
        return rc;

    case BLE_GAP_EVENT_SUBSCRIBE:
        INFO("Subscribe event: Handle=%d, Attr=%d, Reason=%d. Notify: %d->%d, Indicate: %d->%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        e.type = HOGP_CEVT_SUBSCRIBE;
        e.sub_handle = event->subscribe.attr_handle;
        e.sub = 0;
        if (event->subscribe.cur_indicate != 0) e.sub |= 1;
        if (event->subscribe.cur_notify != 0)   e.sub |= 2;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 0) != pdPASS) {
            ERROR("Failed to enqueue SUBSCRIBE event (Queue full)");
            return -1;
        }

        return rc;

    case BLE_GAP_EVENT_MTU:
        INFO("MTU Update: Handle=%d, Channel=%d, New MTU=%d",
                 event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
        
        e.type = HOGP_CEVT_MTU;
        e.mtu = event->mtu.value;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 0) != pdPASS) {
            ERROR("Failed to enqueue MTU event (Queue full)");
            return -1;
        }

        return rc;
    }

    return rc;
}




// Host access callbacks

static int hid_info_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();

    INFO("HID Info characteristic read request received");
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        WARN("Invalid operation %d on HID Info characteristic (Conn: %d)", ctxt->op, conn_handle);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // HID Information structure (4 bytes)
    //    [0]: bcdHID LSB
    //    [1]: bcdHID MSB
    //    [2]: bCountryCode (0x00 for not localized)
    //    [3]: Flags (0x02 for Normally Connectable)
    const uint8_t hid_info[4] = { HOGP_USB_VERSION_MINOR_BCD, HOGP_USB_VERSION_MAJOR_BCD, 0x00, 0x02 };
    int rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
    if (rc != 0) { WARN("HID Info response appended. Unsuccessfull: %d", rc); }
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_control_point_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();
    INFO("HID Control Point write request received");
    
    int rc;
    uint8_t command;

    if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) {
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    if (OS_MBUF_PKTLEN(ctxt->om) != 1) {
        return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
    }

    rc = ble_hs_mbuf_to_flat(ctxt->om, &command, 1, NULL);
    if (rc != 0) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    hogp_control_event_t event;
    event.type = HOGP_CEVT_SUSPEND;
    if (command == 0x00) {          // Suspend
        event.suspended = true;
    } 
    else if (command == 0x01) {     // Exit suspend
        event.suspended = false;
    } 
    else {
        WARN("Invalid HID Control Point command: 0x%02X", command);
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    
    if (xQueueSendToBackFromISR(ctx->control_queue, &event, 0) != pdPASS) {
        ERROR("Failed to enqueue SUSPEND event (Queue full)");
        return -1;
    }

    return 0;
}

static int hid_protocol_mode_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();
    int rc;
    uint8_t command;

    INFO("Protocol Mode characteristic access request received");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (OS_MBUF_PKTLEN(ctxt->om) != 1)  return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        rc = ble_hs_mbuf_to_flat(ctxt->om, &command, 1, NULL);
        if (rc != 0) return BLE_ATT_ERR_UNLIKELY;

        hogp_control_event_t event;
        event.type = HOGP_CEVT_SET_PROTOCOL;
        if (command == 0x00) {          // Boot protocol
            event.protocol = HOGP_PROTOCOL_BOOT;
        }
        else if (command == 0x01) {     // Report protocol
            event.protocol = HOGP_PROTOCOL_REPORT;
        }

        if (xQueueSendToBackFromISR(ctx->control_queue, &event, 0) != pdPASS) {
            ERROR("Failed to enqueue SET_PROTOCOL event (Queue full)");
            return -1;
        }

        return 0;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        INFO("Protocol Mode read. Current protocol: %s", ctx->connection.protocol == HOGP_PROTOCOL_REPORT ? "Report" : "Boot");
        // set 0x00 for boot and 0x01 for report protocol
        command = ctx->connection.protocol == HOGP_PROTOCOL_REPORT ? 0x01 : 0x00;
        const uint8_t hid_info[1] = { command };
        rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));  // TODO check OS_MBUF_PKTLEN
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
}

static int hid_report_map_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    INFO("HID Report Map read request received");
    
    static const uint8_t hid_report_map[] = {
        0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
        0x09, 0x02,        // Usage (Mouse)
        0xA1, 0x01,        // Collection (Application)
        0x09, 0x01,        //   Usage (Pointer)
        0xA1, 0x00,        //   Collection (Physical)
        
        // --- Buttons (3 buttons) ---
        0x05, 0x09,        //     Usage Page (Button)
        0x19, 0x01,        //     Usage Minimum (0x01)
        0x29, 0x03,        //     Usage Maximum (0x03)
        0x15, 0x00,        //     Logical Minimum (0)
        0x25, 0x01,        //     Logical Maximum (1)
        0x95, 0x03,        //     Report Count (3)
        0x75, 0x01,        //     Report Size (1)
        0x81, 0x02,        //     Input (Data,Var,Abs,No Wrap,Linear,...)

        // --- Padding (5 bits to complete the byte) ---
        0x95, 0x01,        //     Report Count (1)
        0x75, 0x05,        //     Report Size (5)
        0x81, 0x03,        //     Input (Const,Var,Abs,No Wrap,Linear,...)

        // --- X, Y, Wheel (3 bytes) ---
        0x05, 0x01,        //     Usage Page (Generic Desktop Ctrls)
        0x09, 0x30,        //     Usage (X)
        0x09, 0x31,        //     Usage (Y)
        0x09, 0x38,        //     Usage (Wheel)
        0x15, 0x81,        //     Logical Minimum (-127)
        0x25, 0x7F,        //     Logical Maximum (127)
        0x75, 0x08,        //     Report Size (8)
        0x95, 0x03,        //     Report Count (3)
        0x81, 0x06,        //     Input (Data,Var,Rel,No Wrap,Linear,...)

        0xC0,              //   End Collection
        0xC0               // End Collection
    };
    int rc = os_mbuf_append(ctxt->om, hid_report_map, sizeof(hid_report_map));
    
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_report_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();

    INFO("Mouse Report characteristic read request received");
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        WARN("Invalid operation %d on mouse report characteristic (Conn: %d)", ctxt->op, conn_handle);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // Mouse report structure (4 bytes)
    //    [0]: buttons
    //    [1]: cursor motion x
    //    [2]: cursor motion y
    //    [3]: scroll motion y
    const uint8_t hid_info[4] = { ctx->hid_state.buttons, 0, 0, 0 };
    int rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
    if (rc != 0) { WARN("Mouse Report response appended. Unsuccessfull: %d", rc); }
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_boot_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    WARN("Tried to access the mouse boot report (not implemented)");
    return 0;
}

static int pnp_id_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();

    INFO("Plug and Play ID characteristic read request received");
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        WARN("Invalid operation %d on mouse report characteristic (Conn: %d)", ctxt->op, conn_handle);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // Mouse report structure (7 bytes)
    //    [0]: vendor source id
    //    [1]: vendor id (msb)
    //    [2]: vendor id (lsb)
    //    [3]: product id (msb)
    //    [4]: product id (lsb)
    //    [5]: version id (msb)
    //    [6]: version id (lsb)
    const uint8_t hid_info[7] = { 0x01, 0xFF, 0xFF, 0x00, 0x01, 0x00, 0x01 };   // SIG ID, None, Product 1, Version 0.0.1
    int rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
    if (rc != 0) { WARN("Mouse Report response appended. Unsuccessfull: %d", rc); }
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    return 0;
}

static int hid_report_ref_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    uint8_t hid_report_reference[2] = { 0x01, 0x01 };
    INFO("Mouse Report reference descriptor read request received");

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_DSC) {
        int rc = os_mbuf_append(ctxt->om, hid_report_reference, sizeof(hid_report_reference));
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }

    return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
}
