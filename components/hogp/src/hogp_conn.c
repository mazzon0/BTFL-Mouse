#include "hogp_conn.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include <string.h>

static hogp_conn_t connection;

// Private functions for the setup
static int gap_init(hogp_init_info_t *init_info);
static int gatt_init(hogp_init_info_t *init_info);
static int check_init_info(hogp_init_info_t *init_info);
static int init_connection_data(hogp_init_info_t *init_info);
static void set_ble_callbacks(void);
static void ble_stack_reset_callback(int reason);
static void ble_stack_sync_callback(void);

// Private functions for the task
static int gap_event_callback(struct ble_gap_event *event, void *arg);
static int start_advertising();
static int send();

// Prototypes for BLE functions
void ble_store_config_init(void);

// Public HOGP functions

int hogp_conn_setup(hogp_init_info_t *init_info) {
    int rc = 0;
    
    // init connection data
    rc = check_init_info(init_info);
    if (rc != 0) return rc;
    init_connection_data(init_info);
    
    // setup ble connection
    rc = gap_init(init_info);
    if (rc != 0) return rc;
    rc = gatt_init(init_info);
    if (rc != 0) return rc;

    set_ble_callbacks();

    return rc;
}

void hogp_conn_task(void *params) {

    while (1) {

        xSemaphoreTake(connection.mutex, portMAX_DELAY);
        
        // check not terminated
        if (connection.flags & HOGP_TERMINATED_FLAG) {
            ESP_LOGE(HID_TAG, "Bluetooth task was set as terminated, but still running");
            xSemaphoreGive(connection.mutex);
            break;
        }
        
        // check if must terminate
        if (connection.flags & HOGP_MUST_CLOSE_FLAG) {
            ESP_LOGI(HID_TAG, "Shutting down the bluetooth task");
            xSemaphoreGive(connection.mutex);
            break;
        }
        
        // need advertising and not connected
        if (!(connection.flags & HOGP_CONNECTED_FLAG) && connection.flags & HOGP_NEED_ADV_FLAG) {
            start_advertising();
            connection.flags &= ~HOGP_NEED_ADV_FLAG;
        }

        // connected, there are updates to send, can send
        if (connection.flags & HOGP_CONNECTED_FLAG && connection.flags & HOGP_UPDATED_FLAG && connection.flags & HOGP_CAN_SEND_FLAG) {
            send();
            connection.flags &= ~(HOGP_UPDATED_FLAG | HOGP_CAN_SEND_FLAG);  // set updated and can send to false
        }

        xSemaphoreGive(connection.mutex);

        vTaskDelay(connection.update_period_ms / portTICK_PERIOD_MS);
    }

    connection.flags |= HOGP_TERMINATED_FLAG;

    vTaskDelete(NULL);
}

int hogp_conn_shutdown(void) {
    int rc = 0;

    xSemaphoreTake(connection.mutex, portMAX_DELAY);
    connection.flags |= HOGP_MUST_CLOSE_FLAG;
    bool connected = connection.flags & HOGP_CONNECTED_FLAG;
    xSemaphoreGive(connection.mutex);

    while (!(connection.flags & HOGP_TERMINATED_FLAG)) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // free all allocated memory
    if (connection.mice != NULL && connection.n_mice > 0) free(connection.mice);
    if (connection.keyboards != NULL && connection.n_keyboards) free(connection.keyboards);
    if (connection.customs != NULL && connection.n_customs) free(connection.customs);

    // disconnect if connected
    if (connected) {
        rc = ble_gap_terminate(connection.conn_handle, BLE_ERR_RD_CONN_TERM_PWROFF);    // FIX: this returns an error code
        if (rc != 0) {
            ESP_LOGE(HID_TAG, "Failed to disconnect, error: %d", rc);
        }
    }

    return rc;
}


// ------------- Private functions -------------

static int gap_init(hogp_init_info_t *init_info) {
    int rc = 0;

    ble_svc_gap_init();

    rc = ble_svc_gap_device_name_set(connection.device_name);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to set device name to %s, error code: %d", connection.device_name, rc);
        return rc;
    }

    rc = ble_svc_gap_device_appearance_set(connection.appearance);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to set device appearance, error code: %d", rc);
        return rc;
    }

    return rc;
}

static int gatt_init(hogp_init_info_t *init_info) {
    return 0;
}

static int check_init_info(hogp_init_info_t *init_info) {
    if (init_info == NULL) {
        ESP_LOGE(HID_TAG, "null hogp init info");
        return -1;
    }
    if (init_info->n_mice == 0 && init_info->n_keyboards == 0 && init_info->n_customs == 0) {
        ESP_LOGE(HID_TAG, "hogp init info contains 0 HID devices");
        return -1;
    }
    if (init_info->update_period_ms == 0) {
        ESP_LOGE(HID_TAG, "hogp init info: member update_period_ms must be greater than 0");
        return -1;
    }
    if (init_info->appearance != HOGP_APPEARANCE_CUSTOM &&
        init_info->appearance != HOGP_APPEARANCE_KEYBOARD &&
        init_info->appearance != HOGP_APPEARANCE_MOUSE) {
        ESP_LOGE(HID_TAG, "hogp init info: unknown appearance value: %d", init_info->appearance);
        return -1;
    }
    if (strlen(init_info->device_name) == 0) {
        ESP_LOGE(HID_TAG, "empty device name");
        return -1;
    }
    return 0;
}

static int init_connection_data(hogp_init_info_t *init_info) {
    int rc = 0;

    connection = (hogp_conn_t) {
        .mice = NULL,
        .keyboards = NULL,
        .customs = NULL,
        .mutex = xSemaphoreCreateMutex(),
        .protocol = HOGP_REPORT,
        .conn_handle = 0,
        .mtu = 23,
        .update_period_ms = init_info->update_period_ms,
        .appearance = init_info->appearance,
        .n_mice = init_info->n_mice,
        .n_keyboards = init_info->n_keyboards,
        .n_customs = init_info->n_customs,
        .flags = 0b00000000
    };

    strcpy(connection.device_name, init_info->device_name);

    if (connection.n_mice > 0) connection.mice = malloc(sizeof(hogp_mouse_t) * init_info->n_mice);
    if (connection.n_keyboards > 0) connection.keyboards = malloc(sizeof(hogp_keyboard_t) * init_info->n_keyboards);
    if (connection.n_customs > 0) connection.customs = malloc(sizeof(hogp_custom_t) * init_info->n_customs);

    // setup mice
    if (connection.mice == NULL && connection.n_mice > 0) {
        ESP_LOGE(HID_TAG, "unable to allocate data for %d mouse services", connection.n_mice);
        return -1;
    }
    for (uint16_t i = 0; i < connection.n_mice; i++) {
        rc = hogp_mouse_init(&connection.mice[i]);
        if (rc != 0) {
            ESP_LOGE(HID_TAG, "unable to inialize mouse service data for mouse at index %d", i);
            return rc;
        }
    }

    // setup keyboards
    if (connection.keyboards == NULL && connection.n_keyboards > 0) {
        ESP_LOGE(HID_TAG, "unable to allocate data for %d keyboard services", connection.n_keyboards);
        return -1;
    }
    for (uint16_t i = 0; i < connection.n_keyboards; i++) {
        rc = hogp_keyboard_init(&connection.keyboards[i]);
        if (rc != 0) {
            ESP_LOGE(HID_TAG, "unable to inialize keyboard service data for keyboard at index %d", i);
            return rc;
        }
    }

    // setup customs
    if (connection.customs == NULL && connection.n_customs > 0) {
        ESP_LOGE(HID_TAG, "unable to allocate data for %d custom HID services", connection.n_customs);
        return -1;
    }
    for (uint16_t i = 0; i < connection.n_customs; i++) {
        rc = hogp_custom_init(&connection.customs[i]);
        if (rc != 0) {
            ESP_LOGE(HID_TAG, "unable to inialize custom HID service data for custom HID at index %d", i);
            return rc;
        }
    }

    return rc;
}

static void set_ble_callbacks(void) {
    ble_hs_cfg.reset_cb = ble_stack_reset_callback;
    ble_hs_cfg.sync_cb = ble_stack_sync_callback;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;
    //ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;

    ble_store_config_init();
}

static void ble_stack_reset_callback(int reason) {
    ESP_LOGI(HID_TAG, "BLE stack reset: reason %d", reason);
}

static void ble_stack_sync_callback(void) {
    // BLE stack has been setupped, now we can setup advertising data and start advertising
    int rc;
    char addr_str[18] = {0};

    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "device does not have any available bt address!");
        return;
    }

    rc = ble_hs_id_infer_auto(0, &connection.own_addr_type);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to infer address type, error code: %d", rc);
        return;
    }

    rc = ble_hs_id_copy_addr(connection.own_addr_type, connection.addr_val, NULL);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to copy device address, error code: %d", rc);
        return;
    }
    hogp_format_addr(addr_str, connection.addr_val);
    ESP_LOGI(HID_TAG, "device address: %s", addr_str);

    // set that we can start advertising
    connection.flags |= HOGP_NEED_ADV_FLAG;
}

static int gap_event_callback(struct ble_gap_event *event, void *arg) {
    int rc = 0;
    struct ble_gap_conn_desc desc;

    switch (event->type) {

    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(HID_TAG, "connection %s; status=%d", event->connect.status == 0 ? "established" : "failed", event->connect.status);

        if(event->connect.status == 0) {
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0) {
                ESP_LOGE(HID_TAG, "failed to find connection by handle, error code: %d", rc);
                return rc;
            }

            // set as connected + can send
            xSemaphoreTake(connection.mutex, portMAX_DELAY);
            connection.conn_handle = event->connect.conn_handle;
            connection.flags |= HOGP_CONNECTED_FLAG;
            connection.flags |= HOGP_CAN_SEND_FLAG;
            xSemaphoreGive(connection.mutex);

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
        else {
            // set as not connected + need advertising
            xSemaphoreTake(connection.mutex, portMAX_DELAY);
            connection.flags &= ~HOGP_CONNECTED_FLAG;
            connection.flags |= HOGP_NEED_ADV_FLAG;
            xSemaphoreGive(connection.mutex);
        }
        return rc;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(HID_TAG, "disconnected from peer; reason=%d", event->disconnect.reason);

        // set as not connected + need advertising
        xSemaphoreTake(connection.mutex, portMAX_DELAY);
        connection.flags &= ~HOGP_CONNECTED_FLAG;
        connection.flags |= HOGP_NEED_ADV_FLAG;
        xSemaphoreGive(connection.mutex);
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

        // set as not connected + need advertising
        xSemaphoreTake(connection.mutex, portMAX_DELAY);
        connection.flags &= ~HOGP_CONNECTED_FLAG;
        connection.flags |= HOGP_NEED_ADV_FLAG;
        xSemaphoreGive(connection.mutex);
        return rc;

    case BLE_GAP_EVENT_NOTIFY_TX:
        if (event->notify_tx.status == 0) {
            xSemaphoreTake(connection.mutex, portMAX_DELAY);
            connection.flags |= HOGP_CAN_SEND_FLAG;
            xSemaphoreGive(connection.mutex);
        }
        return rc;

    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(HID_TAG, "subscribe event; conn_handle=%d attr_handle=%d reason=%d prevn=%d curn=%d previ=%d curi=%d",
                 event->subscribe.conn_handle, event->subscribe.attr_handle,
                 event->subscribe.reason, event->subscribe.prev_notify,
                 event->subscribe.cur_notify, event->subscribe.prev_indicate,
                 event->subscribe.cur_indicate);

        // TODO gatt_svr_subscribe_cb(event);
        return rc;

    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(HID_TAG, "mtu update event; conn_handle=%d cid=%d mtu=%d", event->mtu.conn_handle, event->mtu.channel_id, event->mtu.value);
        xSemaphoreTake(connection.mutex, portMAX_DELAY);
        connection.mtu = event->mtu.value;
        xSemaphoreGive(connection.mutex);
        return rc;
    }

    return rc;
}

static int start_advertising() {
    int rc = 0;
    const char *name;

    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};
    struct ble_gap_adv_params adv_params = {0};

    /*name = ble_svc_gap_device_name();
    adv_fields.name = (uint8_t *)name;
    adv_fields.name_len = strlen(name);
    adv_fields.name_is_complete = 1;*/
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    /*adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.tx_pwr_lvl_is_present = 1;*/
    adv_fields.appearance = connection.appearance;
    adv_fields.appearance_is_present = 1;
    /*adv_fields.le_role = BLE_GAP_LE_ROLE_PERIPHERAL;
    adv_fields.le_role_is_present = 1;*/
    adv_fields.uuids16 = (ble_uuid16_t[]) { BLE_UUID16_INIT(BLE_UUID_HID_SERVICE) };
    adv_fields.num_uuids16 = 1;
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
    /*rsp_fields.device_addr = connection.addr_val;
    rsp_fields.device_addr_type = connection.own_addr_type;
    rsp_fields.device_addr_is_present = 1;
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(500);
    rsp_fields.adv_itvl_is_present = 1;*/

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to set scan response data, error code: %d", rc);
        return rc;
    }

    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(500);
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(510);

    rc = ble_gap_adv_start(connection.own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, gap_event_callback, NULL);
    if (rc != 0) {
        ESP_LOGE(HID_TAG, "failed to start advertising, error code: %d", rc);
        return rc;
    }

    ESP_LOGI(HID_TAG, "advertising started!");    

    return rc;
}

static int send() {
    return 0;
}
