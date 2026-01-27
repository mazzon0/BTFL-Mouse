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
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "driver/touch_pad.h"

static const char *TAG = "PowerSave";

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
static int64_t last_event_time;
static bool high_performance;

/**
 * Motion reading
 */
/**
 * @brief Read motion data from sensor -- da ilaria ma manca pezzo
 * 
 * @details Updates last motion time!!!!
 * 
 * @code
 * static void pmw3389_read_motion(void) {
    uint8_t motion = pmw3389_read(PMW3389_REG_MOTION);
   
    if (motion & 0x80) {  // Bit 7: motion detected
        // Read 16-bit delta values
        uint8_t delta_x_l = pmw3389_read(PMW3389_REG_DELTA_X_L);
        uint8_t delta_x_h = pmw3389_read(PMW3389_REG_DELTA_X_H);
        uint8_t delta_y_l = pmw3389_read(PMW3389_REG_DELTA_Y_L);
        uint8_t delta_y_h = pmw3389_read(PMW3389_REG_DELTA_Y_H);
       
        int16_t delta_x = (int16_t)((delta_x_h << 8) | delta_x_l);
        int16_t delta_y = (int16_t)((delta_y_h << 8) | delta_y_l);
       
        // Filter small movements (noise)
        if (abs(delta_x) > MOTION_THRESHOLD || abs(delta_y) > MOTION_THRESHOLD) {
            last_motion_time = esp_timer_get_time() / 1000;  // Update timestamp
        }
    }
   }
 * @endcode
 */

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
 * @codevoid config_low_power_consumption(bool high_performance) {
    if (high_performance) { // Active state
        // Set CPU frequency range
        esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = 240,       // Maximum frequence
        .min_freq_mhz = 160,       // Minimum frequence
        .light_sleep_enable = false // Optional: allows automated light sleep
        };
        esp_err_t err = esp_pm_configure(&pm_config); //Apply the frequency limits
        ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "CPU frequence configured (Max:240MHz, Min:160MHz)");
        }

        // Activate Bluetooth (if necessary)
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_IDLE) {
            esp_bt_controller_enable(ESP_BT_MODE_BLE);
            esp_bluedroid_enable(); // Enable Bluetooth software features
        }
        ESP_LOGI(TAG, "Bluetooth activated");
        touch_pad_fsm_start();
    } else { // Inactive state
        // 1. Lower CPU frequency
        esp_pm_config_esp32s3_t pm_config = {
            .max_freq_mhz = 80,       // Maximum frequence
            .min_freq_mhz = 40,       // Minimum frequence
            .light_sleep_enable = false // Optional: allows automated light sleep
            };
            esp_err_t err = esp_pm_configure(&pm_config); // Apply the frequency limits
            ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "CPU frequence configured (Max:80MHz, Min:40MHz)");
            }

        // 2. Disable Bluetooth (if previously initialized)
        // Turning off Bluetooth (Bluedroid)
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
            esp_bluedroid_disable(); // Disable Bluetooth software features
            esp_bluedroid_deinit(); // Free up RAM allocated for Bluetooth
        }
        // Turning off the Bluetooth Controller (Hardware)
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
            esp_bt_controller_disable(); // Phisically turns off the Bluetooth radio module
            esp_bt_controller_deinit(); // Deinitializes the hardware controller driver
        }
        // O basta :
        hogp_shutdown(); // ?
        ESP_LOGI(TAG, "Bluetooth disabled");

        // 3. Disable Touch Sensor
        // If the touch sensor is active, we turn it off to save power in the RTC domain
        touch_pad_fsm_stop(); 
        // In alternativa si puo usare: touch_sensor_disable(); con esp-idf v5.0+ se stai usando driver/touch_sens.h
        ESP_LOGI(TAG, "Touch sensor disabled");
    }
}
 */
void config_low_power_consumption(bool high_performance) {
    if (high_performance) { /* Active state*/
        /* Set CPU frequency range*/
        esp_pm_config_esp32s3_t pm_config = {
        .max_freq_mhz = 240,       /* Maximum frequence */
        .min_freq_mhz = 160,       /* Minimum frequence*/
        .light_sleep_enable = false /* Optional: allows automated light sleep*/
        };
        esp_err_t err = esp_pm_configure(&pm_config); /* Apply the frequency limits*/
        ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "CPU frequence configured (Max:240MHz, Min:160MHz)");
        }

        /* Activate Bluetooth (if necessary)*/
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_IDLE) {
            esp_bt_controller_enable(ESP_BT_MODE_BLE);
            esp_bluedroid_enable(); /* Enable Bluetooth software features*/
        }
        ESP_LOGI(TAG, "Bluetooth activated");
        touch_pad_fsm_start();
    } else { /* Inactive state*/
        /* 1. Lower CPU frequency*/
        esp_pm_config_esp32s3_t pm_config = {
            .max_freq_mhz = 80,       /* Maximum frequence */
            .min_freq_mhz = 40,       /* Minimum frequence*/
            .light_sleep_enable = false /* Optional: allows automated light sleep*/
            };
            esp_err_t err = esp_pm_configure(&pm_config); /* Apply the frequency limits*/
            ESP_ERROR_CHECK(esp_pm_configure(&pm_config));
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "CPU frequence configured (Max:80MHz, Min:40MHz)");
            }

        /* 2. Disable Bluetooth (if previously initialized)*/
        /* Turning off Bluetooth (Bluedroid) ---- immagino si possa cancellare da qua:
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
            esp_bluedroid_disable(); /* Disable Bluetooth software features
            esp_bluedroid_deinit(); /* Free up RAM allocated for Bluetooth
        }
        /* Turning off the Bluetooth Controller (Hardware)
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
            esp_bt_controller_disable(); /* Phisically turns off the Bluetooth radio module
            esp_bt_controller_deinit(); /* Deinitializes the hardware controller driver
        } ----fino a qua ???? */
        hogp_shutdown();
        ESP_LOGI(TAG, "Bluetooth disabled");

        /* 3. Disable Touch Sensor*/
        /* If the touch sensor is active, we turn it off to save power in the RTC domain*/
        touch_pad_fsm_stop(); 
        // In alternativa si puo usare: touch_sensor_disable(); con esp-idf v5.0+ se stai usando driver/touch_sens.h
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
 * The sensor enters its lowest power state and clears its internal "motion detected" bit.
 * GPIO18 is configurd as RTC GPIO, is set as an input, so it can listen for the
 * sensor's signal, and its internal resistors are disabled. The pin is then configured as
 * a wake-up source.
 * A 100ms delay is set to ensure all SPI communcations are closed and to alow the
 * electrical signals on the PCB to stabilize, so that there won't be any "spurious" wakeups.
 * The ESP32 enters Deep Sleep: the CPU powers down and the RAM is wiped. On wake up, a System
 * Reset will be triggered.
 * 
 * @code
 * static void enter_deep_sleep(void) {
    // Configure GPIO18 as RTC GPIO
    rtc_gpio_init(PIN_MOTION);
    rtc_gpio_set_direction(PIN_MOTION, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_dis(PIN_MOTION);
    rtc_gpio_pullup_dis(PIN_MOTION);
   
    // Configure wake-up
    esp_sleep_enable_ext0_wakeup(PIN_MOTION, 1);  // Wake on HIGH
   
    vTaskDelay(pdMS_TO_TICKS(100));
   
    // Enter Deep Sleep (will cause RESET on wake)
    esp_deep_sleep_start();
}
@endcode
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
 * If the time since the last motion detected exceedes 10 seconds, enter Light Sleep mode,
 * if it exceedes 5 minutes, enter Deep Sleep mode.
 * 
 * @code
 * static void check_inactivity(void) {
    if (!sensor_active) return;
   
    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    int64_t inactive_time = current_time - last_motion_time;
   
    if (inactive_time > INACTIVITY_TIMEOUT_MS_LPM) {
        cur_state = LOW_POWER_CONSUMPTION;
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        cur_state = DEEP_SLEEP;
    }
}
 * @endcode
 */
static void check_inactivity(void) {
    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    int64_t inactive_time = current_time - last_event_time;
   
    if (inactive_time > INACTIVITY_TIMEOUT_MS_LPM) {
        /* low power consumption mode */
        cur_state = LOW_POWER_CONSUMPTION;
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        /* deep sleep mode */
        cur_state = DEEP_SLEEP;
    } else {
        config_low_power_consumption(high_performance);
    }
}

void tmx_callback(tmx_gesture_t gesture) {
    // Handle the gesture event
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

/* Variable that holds the current state */
State_t cur_state;

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

    esp_err_t ret;
    // Init the NVS flash (required for bonding)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE("my_project", "Failed to initialize nvs flash, error code: %d ", ret);
    }

    // Init the HOGP component
    hogp_init_info_t hogp_init_info = {
        .device_data = (hogp_device_data_t) {
            .device_name = "BTFL Mouse",
            .appearance = HOGP_APPEARANCE_MOUSE,
        },
        .update_period_ms = 10,
    };

    hogp_result_t res = hogp_setup(&hogp_init_info);
    if (res != HOGP_OK) {
        ESP_LOGE("my_project", "Failed to initialize HOGP, error code: %d ", res);
    }

    /* Init touch driver and task */
    tmx_init();
    xTaskCreate(tmx_task, "tmx_event_task", 4096,(void *) tmx_callback, 5, NULL);

    /* Init optical sensor and task */
    // Configure sensor pins
    pmw3389_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_miso = GPIO_NUM_37,
        .pin_mosi = GPIO_NUM_35,
        .pin_sclk = GPIO_NUM_36,
        .pin_cs = GPIO_NUM_45,
        .pin_motion = GPIO_NUM_18,        // Optional
        .pin_reset = GPIO_NUM_21,         // Optional
        .spi_clock_speed_hz = 2000000,    // 2MHz max
    };

    pmw3389_handle_t sensor = NULL;

    ret = pmw3389_init_and_configure(&config, 3200, &sensor_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "PMW3389 init failed: %s", esp_err_to_name(ret));
    }
    /* Funzione per registrare la callback */
    ret = pmw3389_register_callback(pmw3389_handle_t handle, 
                                        pmw3389_motion_callback_t callback, 
                                        void *user_data);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register motion callback");
    }
    pmw3389_start_motion_tracking_interrupt(sensor, 3200);
    // quindi per questo non serve una xTaskCreate????
    
    last_event_time = esp_timer_get_time()/1000;

    cur_state = WORKING;
}

void fn_WORKING(void) {
    /* altro? non credo. forse serve che il codice 
    di ilaria e di fede mi modifichi il last event time
    in caso di movimento da sensore ottico o touch.
    inoltre sarebbe da controllare non ci siano trasferimenti
    bluetooth mentre va in low_power_consumption*/
    while(1) {
        // Send events to the Bluetooth host
        hogp_data_event_t event;
        event.type = HOGP_DEVT_CURSOR_MOTION;
        event.x = 16;
        event.y = 16;

        for (int i = 0; i < 64; i++) {
            hogp_send(&event);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        check_inactivity();
    }
}

void fn_DEEP_SLEEP(void) {
    sensor_active = false;
    enter_deep_sleep(); 
    /* Following line never executed */
    cur_state = START; 
}

void fn_LOW_POWER_CONSUMPTION(void) {
    config_low_power_consumption(!high_performance);
    cur_state = START;
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
    
    /* hogp_setup(&hogp_init_info);

    vTaskDelay(100000 / portTICK_PERIOD_MS); */
}
