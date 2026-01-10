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
#define BLE_UUID_CHR_HID_INFORMATION            0x2A4A
#define BLE_UUID_CHR_REPORT_MAP                 0x2A4B
#define BLE_UUID_CHR_HID_CONTROL_POINT          0x2A4C
#define BLE_UUID_CHR_REPORT                     0x2A4D
#define BLE_UUID_CHR_PROTOCOL_MODE              0x2A4E
#define BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT    0x2A33
#define BLE_UUID_DSC_REPORT_REF                 0x2908

const ble_uuid16_t hid_service_uuid = BLE_UUID16_INIT(BLE_UUID_SVC_HID);
const ble_uuid16_t hid_info_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_HID_INFORMATION);
const ble_uuid16_t report_map_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_REPORT_MAP);
const ble_uuid16_t hid_control_point_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_HID_CONTROL_POINT);
const ble_uuid16_t protocol_mode_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_PROTOCOL_MODE);
const ble_uuid16_t mouse_report_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_REPORT);
const ble_uuid16_t mouse_boot_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT);
const ble_uuid16_t report_ref_mouse_in_uuid = BLE_UUID16_INIT(BLE_UUID_DSC_REPORT_REF);

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
static int hid_report_ref_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Services definition
static const struct ble_gatt_svc_def services[] = {
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
                .val_handle = NULL,
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
                .val_handle = NULL,
            },
            // End of characteristics
            { 0 }
        },
    },
};


// Public functions

int hogp_gap_init(void) {
    int rc = 0;
    hogp_context_t *ctx = hogp_get_context();

    ble_svc_gap_init();

    rc = ble_svc_gap_device_name_set(ctx->device.device_name);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to set device name to %s, error code: %d", ctx->device.device_name, rc);
        return rc;
    }

    rc = ble_svc_gap_device_appearance_set(ctx->device.appearance);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to set device appearance, error code: %d", rc);
        return rc;
    }

    return rc;
}

int hogp_gatt_init(void) {
    hogp_context_t *ctx = hogp_get_context();
    int rc = 0;

    ble_svc_gatt_init();
    ESP_LOGI(HID_TAG, "Initializing GATT");

    set_service_uuids();

    rc = ble_gatts_count_cfg(services);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "Error configuring the GATT services");
        return rc;
    }
    ESP_LOGI(HID_TAG, "GATT services configured");

    rc = ble_gatts_add_svcs(services);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "Error adding the GATT services");
        return rc;
    }
    ESP_LOGI(HID_TAG, "GATT services added");

    // Get handles TODO check uuid vs uuid16
    /*const ble_uuid_t HID_SVC_UUID = BLE_UUID128_INIT(BLE_UUID_SVC_HID);
    const ble_uuid_t MOUSE_REPORT_UUID = BLE_UUID128_INIT(BLE_UUID_CHR_REPORT);
    const ble_uuid_t MOUSE_BOOT_UUID = BLE_UUID128_INIT(BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT);
    if (ble_gatts_find_chr(&HID_SVC_UUID, &MOUSE_REPORT_UUID, NULL, &ctx->connection.handles.mouse_report) != 0) {
        // TODO error handling
    }
    if (ble_gatts_find_chr(&HID_SVC_UUID, &MOUSE_BOOT_UUID, NULL, &ctx->connection.handles.mouse_boot) != 0) {

    }*/
   if (ble_gatts_find_chr((const ble_uuid_t *)&hid_service_uuid, (const ble_uuid_t *)&mouse_report_uuid, NULL, &ctx->connection.handles.mouse_report) != 0) {
        // TODO error handling
    }
    if (ble_gatts_find_chr((const ble_uuid_t *)&hid_service_uuid, (const ble_uuid_t *)&mouse_boot_uuid, NULL, &ctx->connection.handles.mouse_boot) != 0) {

    }

    return rc;
}

int hogp_nimble_config(void) {
    ble_hs_cfg.reset_cb = ble_stack_reset_callback;
    ble_hs_cfg.sync_cb = ble_stack_sync_callback;
    //ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb; // TODO
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = BLE_SM_IO_CAP_NO_IO; // or BLE_SM_IO_CAP_DISPLAY_YESNO for more security
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 1;
    ble_hs_cfg.sm_sc = 1;

    ble_store_config_init();

    return 0;
}

int hogp_start_advertising(void) {
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
        ESP_LOGE(HID_TAG, "failed to set advertising data, error code: %d", rc);
        return rc;
    }

    name = ble_svc_gap_device_name();
    rsp_fields.name = (uint8_t *)name;
    rsp_fields.name_len = strlen(name);
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to set scan response data, error code: %d", rc);
        return rc;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    rc = ble_gap_adv_start(ctx->connection.own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_callback, NULL);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to start advertising, error code: %d", rc);
        return rc;
    }

    ESP_LOGI(HID_TAG, "advertising started!");    

    return rc;
}

int hogp_notify(uint8_t *message, uint8_t size) {
    hogp_context_t *ctx = hogp_get_context();
    struct os_mbuf *om = ble_hs_mbuf_from_flat(message, sizeof(uint8_t) * size);
    
    // Check for allocation failure
    if (om == NULL) {
        ESP_LOGE(HID_TAG, "Error: No memory for mbuf!");
        return -1;
    }
    
    // Send notification
    int rc = 0;
    if (ctx->connection.protocol == HOGP_PROTOCOL_REPORT) {
        rc = ble_gatts_notify_custom(ctx->connection.conn_handle, ctx->connection.handles.mouse_report, om);
    }
    else if (ctx->connection.protocol == HOGP_PROTOCOL_BOOT) {
        rc = ble_gatts_notify_custom(ctx->connection.conn_handle, ctx->connection.handles.mouse_boot, om);
    }
    
    if (rc == 0) {
        ESP_LOGI(HID_TAG, "Test: Sent Mouse Movement");
    } else {
        ESP_LOGW(HID_TAG, "Test: Failed to send (Error %d)", rc);
        os_mbuf_free_chain(om);
    }
    return 0;
}

int hogp_subscribe(uint16_t handle, uint8_t subscription_type) {
    hogp_context_t *ctx = hogp_get_context();

    int i;
    for (i = 0; i < HOGP_HANDLE_COUNT; i++) {
        if (ctx->connection.handles.values[i] == handle) break;
    }

    if (subscription_type & 1)  ctx->connection.indicate_sub.subs[i] = true;
    else                        ctx->connection.indicate_sub.subs[i] = false;
    if (subscription_type & 2)  ctx->connection.notify_sub.subs[i]   = true;
    else                        ctx->connection.notify_sub.subs[i]   = false;

    return 0;
}



// Utility functions

static void set_service_uuids(void) {
    hogp_context_t *ctx = hogp_get_context();

    ctx->connection.svc_uuids[0] = (ble_uuid16_t) BLE_UUID16_INIT(BLE_UUID_SVC_HID);
}

static inline void hogp_format_addr(char* addr_str, uint8_t addr[]) {
    sprintf(addr_str, "%02X:%02X:%02X:%02X:%02X:%02X", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);
}




// NimBLE callbacks

static void ble_stack_reset_callback(int reason) {
    ESP_LOGI(HID_TAG, "NimBLE stack reset: reason %d", reason);
}

static void ble_stack_sync_callback(void) { // TODO refactor
    // BLE stack has been setupped, now we can setup advertising data and start advertising
    hogp_context_t *ctx = hogp_get_context();
    int rc;
    char addr_str[18] = {0};

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "device does not have any available bt address!");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &ctx->connection.own_addr_type);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(ctx->connection.own_addr_type, ctx->connection.addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    hogp_format_addr(addr_str, ctx->connection.addr_val);
    ESP_LOGI(HID_TAG, "device address: %s", addr_str);

    // Send BLE ready event
    hogp_control_event_t event;
    event.type = HOGP_CEVT_BLE_READY;

    if (xQueueSendToBackFromISR(ctx->control_queue, &event, 1 / portTICK_PERIOD_MS) != pdPASS) {
        ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
    }
}

static int gap_event_callback(struct ble_gap_event *event, void *arg) {
    int rc = 0;
    struct ble_gap_conn_desc desc;
    hogp_control_event_t e;
    hogp_context_t *ctx = hogp_get_context();

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(HID_TAG, "connection %s; status=%d",
                 event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);

        if(event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(HID_TAG, "failed to find connection by handle, error code: %d", rc);
                return rc;
            }

            // Connect event
            e.type = HOGP_CEVT_CONNECT;
            if (xQueueSendToBackFromISR(ctx->control_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
                ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
                return -1;
            }

            struct ble_gap_upd_params params = {
                .itvl_min = desc.conn_itvl,
                .itvl_max = desc.conn_itvl,
                .latency = 3,
                .supervision_timeout = desc.supervision_timeout,
            };

            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0) {
                ESP_LOGE(HID_TAG, "failed to update connection parameters, error code: %d", rc);
                return rc;
            }
        }
        return rc;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(HID_TAG, "disconnected from peer; reason=%d", event->disconnect.reason);

        e.type = HOGP_CEVT_DISCONNECT;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
            ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
            return -1;
        }
        
        return rc;

    case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(HID_TAG, "connection updated; status=%d", event->conn_update.status);

        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0) {
            ESP_LOGE(HID_TAG, "failed to find connection by handle, error code: %d", rc);
            return rc;
        }
        return rc;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(HID_TAG, "advertise complete; reason=%d", event->adv_complete.reason);

        e.type = HOGP_CEVT_ADV_COMPLETE;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
            ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
            return -1;
        }
        return rc;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status == 0) {
            e.type = HOGP_CEVT_NOTIFY_TX;
            if (xQueueSendToBackFromISR(ctx->control_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
                ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
                return -1;
            }
        }
        return rc;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(HID_TAG,
                 "subscribe event; conn_handle=%d attr_handle=%d reason=%d prevn=%d curn=%d previ=%d curi=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        e.type = HOGP_CEVT_SUBSCRIBE;
        e.sub_handle = event->subscribe.attr_handle;
        e.sub = 0;
        if (event->subscribe.cur_indicate != 0) e.sub |= 1;
        if (event->subscribe.cur_notify != 0)   e.sub |= 2;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
            ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
            return -1;
        }

        return rc;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(HID_TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d",
                 event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
        
        e.type = HOGP_CEVT_MTU;
        e.mtu = event->mtu.value;
        if (xQueueSendToBackFromISR(ctx->control_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
            ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
            return -1;
        }

        return rc;
    }

    return rc;
}




// Host access callbacks

static int hid_info_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();

    ESP_LOGI(HID_TAG, "hid info callback");
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) {
        ESP_LOGI(HID_TAG, "Connection %d tried to perform the operation %d, which is not allowed", conn_handle, ctxt->op);
        return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
    }

    // HID Information structure (4 bytes)
    //    [0]: bcdHID LSB
    //    [1]: bcdHID MSB
    //    [2]: bCountryCode (0x00 for not localized)
    //    [3]: Flags (0x02 for Normally Connectable)
    const uint8_t hid_info[4] = { HOGP_USB_VERSION_MINOR_BCD, HOGP_USB_VERSION_MAJOR_BCD, 0x00, 0x02 };
    int rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));
    ESP_LOGI(HID_TAG, "hid info response returned: %d", rc);
    return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int hid_control_point_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();
    ESP_LOGI(HID_TAG, "hid control point callback");
    
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
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }
    
    if (xQueueSendToBackFromISR(ctx->control_queue, &event, 1 / portTICK_PERIOD_MS) != pdPASS) {
        ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
        return -1;
    }

    return 0;
}

static int hid_protocol_mode_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_context_t *ctx = hogp_get_context();
    int rc;
    uint8_t command;

    ESP_LOGI(HID_TAG, "protocol mode callback");

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

        if (xQueueSendToBackFromISR(ctx->control_queue, &event, 1 / portTICK_PERIOD_MS) != pdPASS) {
            ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
            return -1;
        }

        return 0;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ESP_LOGI(HID_TAG, "protocol set");
        // set 0x00 for boot and 0x01 for report protocol
        command = ctx->connection.protocol == HOGP_PROTOCOL_REPORT ? 0x01 : 0x00;
        const uint8_t hid_info[1] = { command };
        rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));  // TODO check OS_MBUF_PKTLEN
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
}

static int hid_report_map_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    ESP_LOGI(HID_TAG, "Reading Report Map");
    
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
    return 0;
}

static int hid_boot_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int hid_report_ref_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}
