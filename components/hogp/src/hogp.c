#include "hogp.h"
#include "hogp_conn.h"

static void nimble_run_task(void *param);

esp_err_t hogp_setup(hogp_init_info_t *init_info) {
    esp_err_t ret = ESP_OK;
    int rc = 0;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(HID_TAG, "failed to initialize nvs flash, error code: %d ", ret);
        return ret;
    }

    ret = nimble_port_init();
    if (ret != ESP_OK) {
        ESP_LOGE(HID_TAG, "failed to initialize nimble stack, error code: %d", ret);
        return ret;
    }

    rc = hogp_conn_setup(init_info);
    if (rc != 0) return ESP_FAIL;

    // schedule nimble run task
    xTaskCreate(nimble_run_task, "NimBLE Run", 4 * 1024, NULL, 5, NULL);

    // schedule hogp run task
    xTaskCreate(hogp_conn_task, "HOGP Connection Manager", 4 * 1024, NULL, 5, NULL);

    return ESP_OK;
}

esp_err_t hogp_shutdown(void) {
    int rc = hogp_conn_shutdown();
    if (rc != 0) return ESP_FAIL;

    return ESP_OK;
}

static void nimble_run_task(void *param) {
    ESP_LOGI(HID_TAG, "nimble run task has been started");
    nimble_port_run();
    vTaskDelete(NULL);
}
