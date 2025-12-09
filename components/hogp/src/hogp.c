#include "hogp.h"
#include "hogp_common.h"
#include "hogp_ble.h"
#include "hogp_context.h"
#include "hogp_fsm.h"

// NimBLE task
static void nimble_run_task(void *param);

int hogp_setup(const hogp_init_info_t *const init_info) {
    esp_err_t ret = ESP_OK;
    hogp_error_t rc = 0;

    // Init flash drive TODO should be done in main?
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ERROR("Failed to initialize nvs flash, error code: %d ", ret);
        return HOGP_OPERATION_FAILED;
    }

    // Init ble stack (NimBLE)
    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ERROR("Failed to initialize nimble stack, error code: %d", ret);
        return HOGP_OPERATION_FAILED;
    }

    // Initialize HOGP context based on the init_info
    rc = hogp_context_init(init_info);
    if (rc != HOGP_OK) return rc;

    // Initialize BLE GAP (Generic Access Profile)
    rc = hogp_gap_init();
    if (rc != HOGP_OK) return rc;

    // Initialize BLE GATT (Generic Attribute Profile)
    rc = hogp_gatt_init();
    if (rc != HOGP_OK) return rc;

    // Config NimBLE (callbacks, security, conding with devices)
    hogp_nimble_config();

    // Schedule NimBLE task
    xTaskCreate(nimble_run_task, "NimBLE Stack", 4 * 1024, NULL, 5, NULL);

    // Schedule HOGP task
    xTaskCreate(hogp_task, "HOGP FSM", 4 * 1024, NULL, 5, NULL);

    return HOGP_OK;
}

int hogp_shutdown(void) {
    // TODO stop HOGP task
    // handle the case if the task do not delete itself

    hogp_context_shutdown();
    return HOGP_OK;
}

int hogp_send_data(const hogp_data_event_t *event) {
    hogp_context_t *ctx = hogp_get_context();

    if (xQueueSendToBackFromISR(ctx->data_queue, event, 0) != pdPASS) { // TODO should be used in ISR?
        ERROR("Push to data queue failed");
        return HOGP_PUSH_FAILED;
    }
    return HOGP_OK;
}

static void nimble_run_task(void *param) {
    INFO("Nimble run task has been started");
    nimble_port_run();
    vTaskDelete(NULL);
}
