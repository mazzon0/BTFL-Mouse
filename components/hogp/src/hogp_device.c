#include "hogp_device.h"

// UUIDs from the SIG Assigned Numbers
#define BLE_UUID_SVC_HID                        0x1812
#define BLE_UUID_CHR_HID_INFORMATION            0x2A4A
#define BLE_UUID_CHR_REPORT_MAP                 0x2A4B
#define BLE_UUID_CHR_HID_CONTROL_POINT          0x2A4C
#define BLE_UUID_CHR_REPORT                     0x2A4D
#define BLE_UUID_CHR_PROTOCOL_MODE              0x2A4E
#define BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT    0x2A33
#define BLE_UUID_DSC_REPORT_REF                 0x2908

// Protocol definitions
#define HOGP_USB_VERSION_MAJOR_BCD  0x01
#define HOGP_USB_VERSION_MINOR_BCD  0x11

// Indices in the characteristics array (0-15)
#define HOGP_CHR_HID_INFO           0
#define HOGP_CHR_REPORT_MAP         1
#define HOGP_CHR_CONTROL_POINT      2
#define HOGP_CHR_PROTOCOL_MODE      3
#define HOGP_CHR_REPORT_MOUSE       4
#define HOGP_CHR_BOOT_MOUSE         5
#define HOGP_CHR_MAX                6

/*static*/ hogp_hid_device_t device;

// Private functions to setup services
static int init_hid_service(hogp_hid_device_init_info_t *init_info, uint8_t i);
static int init_battery_services(hogp_hid_device_init_info_t *init_info, uint8_t i);
static int init_device_info_service(hogp_hid_device_init_info_t *init_info, uint8_t i);

// Callbacks for GATT characteristics
static int hid_info_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_map_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_control_point_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_protocol_mode_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_boot_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Callbacks for GATT descriptors
static int hid_report_ref_mouse_in_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_ref_kbd_in_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);
static int hid_report_ref_kbd_out_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg);

// Public functions implementation

int hogp_hid_device_setup(hogp_hid_device_init_info_t *init_info) {
    uint8_t n_services = 1 + 1 + init_info->n_batteries + 1;    // 1 hid service, 1 device info service, n battery services, 1 empty service
    uint8_t current_service_id = 0;
    int rc = 0;

    // Memory allocation for services
    device.service_uuids = malloc(sizeof(uint16_t) * n_services);
    device.services = malloc(sizeof(struct ble_gatt_svc_def) * n_services);
    if (!device.services || !device.service_uuids) {
        ESP_LOGE(HID_TAG, "Failed to allocate data for initializing GATT services");
        return -1;
    }
    device.n_services = n_services;

    // Init data from init info
    device.n_batteries = init_info->n_batteries;
    device.flags = init_info->flags;
    device.flags |= HOGP_HID_PROTOCOL_FLAG;     // starting with Report protocol is mandatory
    device.conn_handle = init_info->conn_handle;
    device.suspend_cb = init_info->suspend_cb;

    // Initializing services
    rc = init_hid_service(init_info, current_service_id);
    if (rc != 0) return rc;
    current_service_id += 1;

    /*rc = init_device_info_service(init_info, current_service_id);
    if (rc != 0) return rc;
    current_service_id += 1;

    rc = init_battery_services(init_info, current_service_id);
    if (rc != 0) return rc;
    current_service_id += init_info->n_batteries;*/

    // Adding null terminator service
    device.services[current_service_id] = (struct ble_gatt_svc_def) {0};

    // Setting characteristics to unsubscribed
    for (int i = 0; i < N_CHARACTERISTICS; i++) {
        device.characteristics[i].subscribed = false;
    }

    return rc;
}

int hogp_hid_device_shutdown(void) {
    int rc = 0;

    if (device.services == NULL) {
        ESP_LOGE(HID_TAG, "The device was not shutted down, since it was NULL");
        return -1;
    }

    free(device.services);
    free(device.service_uuids);
    return rc;
}

const ble_uuid16_t *const hogp_device_get_services_uuids(void) {
    static ble_uuid16_t svcs[MAX_SERVICES];

    if (device.n_services > MAX_SERVICES) {
        ESP_LOGE(HID_TAG, "The maximum number of services is %d, but the device was intiliazed with %d services", MAX_SERVICES, device.n_services);
        return NULL;
    }

    for (int i = 0; i < device.n_services; i++) {
        svcs[i] = (ble_uuid16_t) BLE_UUID16_INIT(device.service_uuids[i]);
    }

    return svcs;
}

uint16_t hogp_device_get_services_count(void) {
    return device.n_services;
}

const struct ble_gatt_svc_def *const hogp_device_get_services_defs(void) {
    return device.services;
}

uint8_t hogp_device_subscribe_event(struct ble_gap_event *event) {
    for(uint8_t i = 0; i < HOGP_CHR_MAX; i++) {
        if (device.characteristics[i].handle == event->subscribe.attr_handle) {
            ESP_LOGI(HID_TAG, "chr id=%d", i);
            if (event->subscribe.cur_notify)    device.characteristics[i].subscribed |= SUBSCRIBE_NOTIFY;
            else                                device.characteristics[i].subscribed &= ~SUBSCRIBE_NOTIFY;
            
            if (event->subscribe.cur_indicate)  device.characteristics[i].subscribed |= SUBSCRIBE_INDICATE;
            else                                device.characteristics[i].subscribed &= ~SUBSCRIBE_INDICATE;

            return 0;
        }
    }

    return 1;
}


// Private functions implementation

static int init_hid_service(hogp_hid_device_init_info_t *init_info, uint8_t i) {

    device.service_uuids[i] = BLE_UUID_SVC_HID;

    // UUIDs
    static const ble_uuid16_t report_ref_mouse_in_uuid = BLE_UUID16_INIT(BLE_UUID_DSC_REPORT_REF);
    static const ble_uuid16_t hid_info_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_HID_INFORMATION);
    static const ble_uuid16_t report_map_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_REPORT_MAP);
    static const ble_uuid16_t hid_control_point_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_HID_CONTROL_POINT);
    static const ble_uuid16_t protocol_mode_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_PROTOCOL_MODE);
    static const ble_uuid16_t mouse_report_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_REPORT);
    static const ble_uuid16_t mouse_boot_uuid = BLE_UUID16_INIT(BLE_UUID_CHR_BOOT_MOUSE_INPUT_REPORT);
    static const ble_uuid16_t hid_service_uuid = BLE_UUID16_INIT(BLE_UUID_SVC_HID);

    // Descriptors
    static const struct ble_gatt_dsc_def mouse_input_report_descriptors[] = {
        {
            .uuid = (const ble_uuid_t *) &report_ref_mouse_in_uuid,
            .access_cb = hid_report_ref_mouse_in_access_cb,
            .att_flags = BLE_ATT_F_READ,
        },
        { 0 }
    };

    // Characteristics
    static const struct ble_gatt_chr_def characteristics[] = {
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
            .arg = &device,
        },
        // Report Map
        {
            .uuid = (const ble_uuid_t *) &report_map_uuid,
            .access_cb = hid_report_map_access_cb,
            .flags = BLE_GATT_CHR_F_READ,
            .arg = &device,
        },
        // Input Report (Mouse)
        {
            .uuid = (const ble_uuid_t *) &mouse_report_uuid,
            .access_cb = hid_report_access_cb,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &device.characteristics[HOGP_CHR_REPORT_MOUSE].handle,
            .arg = &device,
            .descriptors = mouse_input_report_descriptors
        },
        // Boot Mouse Input
        {
            .uuid = (const ble_uuid_t *) &mouse_boot_uuid,
            .access_cb = hid_boot_mouse_access_cb,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
            .val_handle = &device.characteristics[HOGP_CHR_BOOT_MOUSE].handle,
            .arg = &device,
        },
        // End of characteristics
        { 0 }
    };

    // Init the GATT service data
    device.services[i] = (struct ble_gatt_svc_def) {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (const ble_uuid_t *) &hid_service_uuid,
        .characteristics = characteristics,
    };

    return 0;
}

static int init_battery_services(hogp_hid_device_init_info_t *init_info, uint8_t i) {
    return 0;
}

static int init_device_info_service(hogp_hid_device_init_info_t *init_info, uint8_t i) {
    return 0;
}


// Callbacks

static int hid_info_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
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

static int hid_control_point_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_hid_device_t *device = (hogp_hid_device_t *)arg;
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

    if (command == 0x00) {          // Suspend
        device->suspend_cb(true);
    } 
    else if (command == 0x01) {     // Exit suspend
        device->suspend_cb(false);
    } 
    else {
        return BLE_ATT_ERR_VALUE_NOT_ALLOWED;
    }

    return 0;
}

static int hid_protocol_mode_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    hogp_hid_device_t *device = (hogp_hid_device_t *)arg;
    int rc;
    uint8_t command;

    ESP_LOGI(HID_TAG, "protocol mode callback");

    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (OS_MBUF_PKTLEN(ctxt->om) != 1)  return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;

        rc = ble_hs_mbuf_to_flat(ctxt->om, &command, 1, NULL);
        if (rc != 0) return BLE_ATT_ERR_UNLIKELY;

        if (command == 0x00) {          // Boot protocol
            device->flags &= ~HOGP_HID_PROTOCOL_FLAG;
        }
        else if (command == 0x01) {     // Report protocol
            device->flags |= HOGP_HID_PROTOCOL_FLAG;
        }

        return 0;
    }
    else if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        ESP_LOGI(HID_TAG, "protocol set at %d", device->flags & HOGP_HID_PROTOCOL_FLAG);
        // set 0x00 for boot and 0x01 for report protocol
        command = (device->flags & HOGP_HID_PROTOCOL_FLAG) ? 0x01 : 0x00;
        const uint8_t hid_info[1] = { command };
        rc = os_mbuf_append(ctxt->om, hid_info, sizeof(hid_info));  // TODO check OS_MBUF_PKTLEN
        return (rc == 0) ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    else return BLE_ATT_ERR_WRITE_NOT_PERMITTED;
}

static int hid_report_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int hid_boot_mouse_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int hid_report_ref_mouse_in_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int hid_report_ref_kbd_in_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}

static int hid_report_ref_kbd_out_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    return 0;
}
