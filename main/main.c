//#include "hogp.h"
#include "power_supply.h"
#include "power_supply.c"
#include "pmw3389.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"

#include "esp_pm.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "driver/touch_pad.h"

#include "esp_pm.h"

#include "freertos/task.h"

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
static volatile bool motion_detected;
static int64_t last_motion_time;
static bool sensor_active;

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
 * @brief Enter Low Power Mode
 * 
 * @param bool high_performance
 * @return void
 * @details The function is called after 5 minutes of inactivity. It lowers the
 * CPU frequency and disables bluetooth and touch features.
 * @codevoid low_power_consumption_state(bool high_performance) {
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
void low_power_consumption_state(bool high_performance) {
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
        /* Turning off Bluetooth (Bluedroid)*/
        if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_ENABLED) {
            esp_bluedroid_disable(); /* Disable Bluetooth software features*/
            esp_bluedroid_deinit(); /* Free up RAM allocated for Bluetooth*/
        }
        /* Turning off the Bluetooth Controller (Hardware)*/
        if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
            esp_bt_controller_disable(); /* Phisically turns off the Bluetooth radio module*/
            esp_bt_controller_deinit(); /* Deinitializes the hardware controller driver*/
        }
        // O basta :
        hogp_shutdown(); // ?
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
    // Put sensor in shutdown
    pmw3389_shutdown();
    pmw3389_clear_motion();
   
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
    /* Put sensor in shutdown */
    pmw3389_shutdown();
    pmw3389_clear_motion();
   
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
        // low power consumption
        cur_state = LOW_POWER_CONSUMPTION;
        low_power_consumption_state();
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        cur_state = DEEP_SLEEP;
        enter_deep_sleep(); 
    }
}
 * @endcode
 */
static void check_inactivity(void) {
    if (!sensor_active) return;
   
    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    int64_t inactive_time = current_time - last_motion_time;
   
    if (inactive_time > INACTIVITY_TIMEOUT_MS_LPM) {
        /* low power consumption */
        cur_state = LOW_POWER_CONSUMPTION;
        low_power_consumption_state();
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        cur_state = DEEP_SLEEP;
        enter_deep_sleep(); 
    }
}

//Main sensor polling task --- SI PUO CANCELLARE?
/* static void sensor_task(void *arg) {
    last_motion_time = esp_timer_get_time() / 1000;
   
    while (1) {
        /* Handle motion interrupt
        if (motion_detected) {
            motion_detected = false;
            if (!sensor_active) {
                cur_state = START;
                pmw3389_wakeup();
            }
        }
       
        /* Read motion if sensor is active
        if (sensor_active) {
            pmw3389_read_motion();
            check_inactivity();
        }
       
        vTaskDelay(pdMS_TO_TICKS(POLLING_RATE_MS));  // ~100Hz polling
    }
} 
*/

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
/* State machine function prototypes*/
void fn_START(void);
void fn_WORKING(void);
void fn_DEEP_SLEEP(void);
void fn_LOW_POWER_CONSUMPTION(void);
void fn_OFF(void);

/* Variable that holds the current state */
State_t cur_state;

StateMachine_t StateMachine[] = {
    {START, fm_START()},
    {WORKING, fm_WORKING()},
    {DEEP_SLEEP, fm_DEEP_SLEEP()},
    {LOW_POWER_CONSUMPTION, fm_LOW_POWER_CONSUMPTION()},
    {OFF, fn_OFF()}
};

/* State functions */
void fn_START(void) {
    /* Init HOGP library and task*/
    hogp_setup(const hogp_init_info_t *const init_info);

    /* Init touch driver and task */
    tmx_init();
    xTaskCreate(tmx_task, "tmx_event_task", 4096,(void *) tmx_callback, 5, NULL);

    /* Init optical sensor and task*/
    pmw3389_init_and_configure(const pmw3389_config_t *config, uint16_t cpi, pmw3389_handle_t *out_handle);
    /* Funzione per registrare la callback */
    esp_err_t pmw3389_register_callback(pmw3389_handle_t handle, 
                                        pmw3389_motion_callback_t callback, 
                                        void *user_data);
    xTaskCreate(pmw3389_start_motion_tracking_interrupt , "pmw3389_event_task", 4096, NULL, 5, NULL);

    motion_detected = false;
    last_motion_time = 0;
    sensor_active = true;

    cur_state = WORKING;
}

void fn_WORKING(void) {
    /*..*/
    check_inactivity(void);
}

void fn_DEEP_SLEEP(void) {
    /*..*/

    cur_state = START; /* Qua non credo questo serva questo stato ? visto che quando
    si sveglia da deep sleep resetta in automatico ? */

void fn_LOW_POWER_CONSUMPTION(void) {
    /*..*/
    check_inactivity(void);
    cur_state = START;
}

void fn_OFF(void) {
    /* Battery code */

    cur_state = START;
}

void app_main(void) {
    hogp_init_info_t hogp_init_info = {
        .n_mice = 1,
        .n_keyboards = 0,
        .n_customs = 0,
        .update_period_ms = 10,
        .appearance = HOGP_APPEARANCE_MOUSE,
        .device_name = "BTFL Mouse"
    };



    
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
