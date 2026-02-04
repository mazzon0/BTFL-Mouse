#include "hogp.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "tmx.h"
#include "pmw3389.h"

#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"

//le prossime servono??
#include "esp_pm.h"
//#include "esp_bt.h"
//#include "esp_bt_main.h"
//#include "driver/touch_pad.h"

//Pin configuration
/**
 * MOTION: GPIO18 (RTC_GPIO for wake-up)
*/
#define PIN_MOTION GPIO_NUM_18 /*RTC_GPIO for wake-up*/
#define WAKEUP_LEVEL 1 /*HIGH LEVEL on motion detected*/

//Configuration
#define SPI_FREQUENCY 2000000	// 2 MHz
#define INACTIVITY_TIMEOUT_MS_LPM 300000	// 5 min
#define INACTIVITY_TIMEOUT_MS_DS 600000 // 10 min
#define MOTION_THRESHOLD 5	// Minimum movement threshold
#define POLLING_RATE_MS 10	// 100Hz polling

//Global variables
static const char *TAG = "PowerSave";
static int64_t last_event_time;
static bool high_performance;
static pmw3389_handle_t sensor_handle = NULL; // Handle to communicate with the sensor

static TaskHandle_t tmx_task_handle = NULL;

/**
 * Define states and state machine structure
 */
typedef enum {
    START,
    WORKING,
    DEEP_SLEEP,
    LOW_POWER_CONSUMPTION,
    OFF,
    NUM_STATES
} State_t;

/* Variable that holds the current state */
State_t cur_state;

 /**
 * Power mode functions
 */
/**
 * @brief Configure Low Power Mode
 * 
 * @param bool high_performance
 * @return void
 * @details The function is called after 5 minutes of inactivity. It lowers the
 * CPU frequency and disables bluetooth and touch features.
 */
void config_low_power_consumption(bool high_performance) {
    if (high_performance) { /* Active state*/
        /* Set CPU frequency range*/
        esp_pm_config_t pm_config = {
            .max_freq_mhz = 240,       /* Maximum frequence */
            .min_freq_mhz = 140,       /* Minimum frequence*/
            .light_sleep_enable = false /* Optional: allows automated light sleep*/
        };
        esp_err_t err = esp_pm_configure(&pm_config); /* Apply the frequency limits*/
        if (err == ESP_OK) {
            ESP_LOGE(TAG, "CPU frequence configured (Max:240MHz, Min:160MHz)");
        }

    } else { /* Inactive state*/
        /* 1. Lower CPU frequency*/
        esp_pm_config_t pm_config = {
            .max_freq_mhz = 80,       /* Maximum frequence */
            .min_freq_mhz = 40,       /* Minimum frequence*/
            .light_sleep_enable = false /* Optional: allows automated light sleep*/
            };
            esp_err_t err = esp_pm_configure(&pm_config); /* Apply the frequency limits*/
            ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "CPU frequence configured (Max:80MHz, Min:40MHz)");
            }

        /* 2. Disable Bluetooth */
        hogp_shutdown();
        ESP_LOGI(TAG, "Bluetooth disabled");

        /* 3. Disable Touch Sensor */
        /* If the touch sensor is active, we turn it off to save power in the RTC domain */
        vTaskDelete(tmx_task_handle);
        
        ESP_LOGI(TAG, "Touch sensor disabled");
    }
}

/**
 * @brief Enter Deep Sleep mode
 * 
 * @param void
 * 
 * @return void
 * 
 * @details This function prepares the ESP32 and the PMW3389 sensor for Deep Sleep.
 * This power-saving mode turns off both the CPU and RAM. When the device is
 * woke up it starts the program execution from the very beginning, so it behaves as a reset.
 * GPIO18 is configurd as RTC GPIO, is set as an input, so it can listen for the
 * sensor's signal, and its internal resistors are disabled - to leave the pin in a
 * high-impedance state (floating), preventing leakage  currents that could drain
 * the battery during deep sleep. The pin is then configured as
 * a wake-up source.
 * A 100ms delay is set to ensure all SPI communcations are closed and to alow the
 * electrical signals on the PCB to stabilize, so that there won't be any "spurious" wakeups.
 * The ESP32 enters Deep Sleep: the CPU powers down and the RAM is wiped. On wake up, a System
 * Reset will be triggered.
 */
static void enter_deep_sleep(void) {      
    /* Configure GPIO18 as RTC GPIO */
    rtc_gpio_init(PIN_MOTION);
    rtc_gpio_set_direction(PIN_MOTION, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(PIN_MOTION);
    rtc_gpio_pullup_dis(PIN_MOTION);
   
    /* Configure wake-up */
    esp_sleep_enable_ext0_wakeup(PIN_MOTION, 1);  /* Wake on HIGH */
   
    vTaskDelay(pdMS_TO_TICKS(100));
   
    /* Enter Deep Sleep (will cause RESET on wake) */
    esp_deep_sleep_start();
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

void bt_connection_cb(bool connected) {
    if (connected) ESP_LOGI("my_project", "Connected");
    else ESP_LOGI("my_project", "Disconnected");
}

void bt_suspension_cb(bool suspended) {
    if (suspended) ESP_LOGI("my_project", "Suspended");
    else ESP_LOGI("my_project", "Not suspended");
}

void tmx_callback(tmx_gesture_t gesture) {
    /* Whenever a touch gesture is detected, reset the inactivity timer */
    last_event_time = esp_timer_get_time() / 1000;
    
    /* Handle the gesture event */
    hogp_data_event_t event;

    switch(gesture.type){
        case TMX_GESTURE_BUTTON_PRESSED:
            event.type = HOGP_DEVT_MOUSE_BUTTON_PRESSED;
            event.button = gesture.button;
            ESP_LOGI("Mouse", "Sending button pressed: %d", event.button);
            hogp_send(&event);
            break;

        case TMX_GESTURE_BUTTON_RELEASED:
            event.type = HOGP_DEVT_MOUSE_BUTTON_RELEASED;
            event.button = gesture.button;
            ESP_LOGI("Mouse", "Sending button released: %d", event.button);
            hogp_send(&event);
            break;

        case TMX_GESTURE_SCROLL:
            event.type = HOGP_DEVT_SCROLL_MOTION;
            event.x = gesture.dx;
            event.y = gesture.dy;
            ESP_LOGI("Mouse", "Sending scroll: %d, %d", event.x, event.y);
            hogp_send(&event);
            break;

        default:
            break; 
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
    /* 1. Whenever a movement is detected, reset the inactivity timer */
    last_event_time = esp_timer_get_time() / 1000;

    /* 2. Prepare the data event data */
    hogp_data_event_t event;
    event.type = HOGP_DEVT_CURSOR_MOTION;
    event.x = motion->delta_x;
    event.y = motion->delta_y;

    /* 3. Send data if noise threshold exceeded */
    if (abs(motion->delta_x) > MOTION_THRESHOLD || abs(motion->delta_y) > MOTION_THRESHOLD) {
        hogp_send(&event);
    }
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

typedef struct {
    State_t state; /* Defines the command */
    void (*func) (void); /* Defines the function to execute */
} StateMachine_t;

/**
 * Declare fucitions that implement states, variable to hold
 * current state and the state machine itself.
 */
/* State machine function prototypes */
void fn_START(void);
void fn_WORKING(void);
void fn_DEEP_SLEEP(void);
void fn_LOW_POWER_CONSUMPTION(void);
void fn_OFF(void);

StateMachine_t StateMachine[] = {
    {START, fn_START},
    {WORKING, fn_WORKING},
    {DEEP_SLEEP, fn_DEEP_SLEEP},
    {LOW_POWER_CONSUMPTION, fn_LOW_POWER_CONSUMPTION},
    {OFF, fn_OFF}
};

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
        ESP_LOGE("my_project", "Failed to initialize nvs flash, error code: %d ", ret);
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
        ESP_LOGE("my_project", "Failed to initialize HOGP, error code: %d ", res);
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

    /* Create task that will run in the background to listen for movements */
    xTaskCreate(sensor_task, "sensor_task", 4096, NULL, 10, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register motion callback");
    }
    
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

void app_main(void) {
    cur_state = START;
    while(1) {
        if(cur_state < NUM_STATES) {
            (*StateMachine[cur_state].func) ();
        } else {
            /* error */
        }
    }

}
