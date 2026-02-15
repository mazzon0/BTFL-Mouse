#include "mouse_fsm.h"
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "tmx.h"
#include "hogp.h"
#include "pmw3389.h"
#include "nvs_flash.h"
#include "power_modes.h"
#include "battery_monitor.h"

#define INACTIVITY_TIMEOUT_MS_LPM 300000	// 5 min
#define INACTIVITY_TIMEOUT_MS_DS 600000 // 10 min

const char *TAG = "BTFL Mouse";
int64_t last_event_time = 0;
bool high_performance;
pmw3389_handle_t sensor_handle = NULL;
TaskHandle_t tmx_task_handle = NULL;

extern State_t cur_state;

// Button codes to convert tmx gestures to hogp events
hogp_mouse_button_t button_codes[2] = {HOGP_MOUSE_BLEFT, HOGP_MOUSE_BRIGHT};

void send_battery_level(void) {
    // Check battery level
    uint16_t battery_level = check_battery();
    ESP_LOGI(TAG, "sending battery level: %d \n", battery_level);
    hogp_data_event_t event;
    event.type = HOGP_DEVT_BATTERY_LEVEL_UPDATE;
    event.battery_level = battery_level;

    // Send many messages with the update
    const int NUM_NOTIFICATIONS = 16;
    const int DELAY = 50; // ms
    for (int i = 0; i < NUM_NOTIFICATIONS; i++) {
        hogp_send(&event);
        vTaskDelay(pdMS_TO_TICKS(DELAY));   // needed to not pack the messages together during serialization
    }
}

/**
 * Callback for handling disconnection events from Bluetooth
 */
void bt_connection_cb(bool connected) {
    if (connected) {
        ESP_LOGI(TAG, "Connected");
        send_battery_level();
    }
    else ESP_LOGI(TAG, "Disconnected");
}

/**
 * Callback for handling suspension events from Bluetooth
 */
void bt_suspension_cb(bool suspended) {
    if (suspended) ESP_LOGI(TAG, "Suspended");
    else ESP_LOGI(TAG, "Not suspended");
}

void tmx_callback(tmx_gesture_t gesture) {
    /* Whenever a touch gesture is detected, reset the inactivity timer */
    last_event_time = esp_timer_get_time() / 1000;
    
    /* Handle the gesture event */
    hogp_data_event_t event;

    switch(gesture.type){
        case TMX_GESTURE_BUTTON_PRESSED:
            event.type = HOGP_DEVT_MOUSE_BUTTON_PRESSED;
            event.button = button_codes[gesture.button];
            ESP_LOGI(TAG, "Sending button pressed: %d", event.button);
            hogp_send(&event);
            break;

        case TMX_GESTURE_BUTTON_RELEASED:
            event.type = HOGP_DEVT_MOUSE_BUTTON_RELEASED;
            event.button = button_codes[gesture.button];
            ESP_LOGI(TAG, "Sending button released: %d", event.button);
            hogp_send(&event);
            break;

        case TMX_GESTURE_SCROLL:
            event.type = HOGP_DEVT_SCROLL_MOTION;
            event.x = gesture.dx;
            event.y = gesture.dy;
            ESP_LOGI(TAG, "Sending scroll: %d, %d", event.x, event.y);
            hogp_send(&event);
            break;

        default:
            break; 
    }
}

/**
 * @brief Check inactivity and trigger sleep if needed
 * 
 * @param void
 * 
 * @return void
 * 
 * @details If the sensor is active, the function calulates the inactive_time.
 * If the time since the last motion detected exceedes 5 minutes, enter Light Sleep mode,
 * if it exceedes 10 minutes, enter Deep Sleep mode. Else it goes back to high performance state.
 */
static void check_inactivity(void) {
    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    int64_t inactive_time = current_time - last_event_time;
   
    if (inactive_time > INACTIVITY_TIMEOUT_MS_LPM && inactive_time < INACTIVITY_TIMEOUT_MS_DS) {
        /* low power consumption mode */
        cur_state = LOW_POWER_CONSUMPTION;
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        /* deep sleep mode */
        cur_state = DEEP_SLEEP;
    } else {
        cur_state = WORKING;
    }
}

/**
 * @brief Motion callback
 * @param
 * @return
 * 
 * @details This functoin "translates" the sensor' movements into Bluetooth
 * commands and resets the timer.
 */
static void pmw3389_callback(const pmw3389_motion_data_t *motion, void *used_data) {
    last_event_time = esp_timer_get_time() / 1000;

    hogp_data_event_t event;
    event.type = HOGP_DEVT_CURSOR_MOTION;
    event.x = motion->delta_x;
    event.y = motion->delta_y;

    hogp_send(&event);
}

/**
 * @brief Sensor task definition
 * @param
 * @return
 * 
 * @details This function handles interrupts from the sensor internally.
 */
static void sensor_task(void *pvParameters) {
    ESP_LOGI(TAG, "Sensor task started");
    
    pmw3389_start_motion_tracking_interrupt(sensor_handle, 3200);
    vTaskDelete(NULL);
}

static void battery_task(void *params) {
    ESP_LOGI(TAG, "Battery task started");
    
    while(1) {
        // Send battery level and speel for 30 seconds
        send_battery_level();
        vTaskDelay(pdMS_TO_TICKS(30 * 1000));
    }
    vTaskDelete(NULL);
}

/* State functions */
void fn_START(void) {
    high_performance = true;
    config_low_power_consumption(high_performance);

    // Init NVS flash (required for bonding)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize nvs flash, error code: %d ", ret);
    }

    // Init HOGP component
    hogp_init_info_t hogp_init_info = {
        .device_data = (hogp_device_data_t) {
            .device_name = "BTFL Mouse",
            .appearance = HOGP_APPEARANCE_MOUSE,
        },
        .connected_cb = bt_connection_cb,
        .suspended_cb = bt_suspension_cb,
        .register_period_ms = 10,
        .transmit_period_ms = 20,
    };

    hogp_result_t res = hogp_setup(&hogp_init_info);
    if (res != HOGP_OK) {
        ESP_LOGE(TAG, "Failed to initialize HOGP, error code: %d ", res);
    }

    /* Init touch driver and task */
    tmx_init();
    xTaskCreate(tmx_task, "tmx_event_task", 4096, (void *) tmx_callback, 5, &tmx_task_handle);

    /* Init optical sensor and task */
    /* Configure sensor pins */
    pmw3389_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_miso = GPIO_NUM_37,
        .pin_mosi = GPIO_NUM_35,
        .pin_sclk = GPIO_NUM_36,
        .pin_cs = GPIO_NUM_45,
        .pin_motion = GPIO_NUM_18,
        .pin_reset = GPIO_NUM_21,
        .spi_clock_speed_hz = 2000000,    // 2MHz max
    };

    ret = pmw3389_init_and_configure(&config, 3200, &sensor_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMW3389 init failed: %s", esp_err_to_name(ret));
    }

    /* Function to register the callback */
    ret = pmw3389_register_callback(sensor_handle, &pmw3389_callback, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register motion callback");
    }

    /* Create task that will run in the background to listen for movements */
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 10, NULL);

    // Start battery monitoring task
    xTaskCreate(battery_task, "Battery Task", 4096, NULL, 10, NULL);
    
    last_event_time = esp_timer_get_time()/1000;

    cur_state = WORKING;
}

void fn_WORKING(void) {
    check_inactivity();
    vTaskDelay(pdMS_TO_TICKS(10000));
}

void fn_DEEP_SLEEP(void) {
    enter_deep_sleep(); 
    /* Following line never executed */
    cur_state = START; 
}

void fn_LOW_POWER_CONSUMPTION(void) {
    config_low_power_consumption(!high_performance);
    check_inactivity();
}

void fn_OFF(void) {
    /* Codice batteria ??? */

    cur_state = START;
}
