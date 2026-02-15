#include "power_modes.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_pm.h"
#include "driver/rtc_io.h"
#include "hogp.h"
#include "pmw3389.h"
#include "mouse_fsm.h"

#define PIN_MOTION GPIO_NUM_18
#define WAKEUP_LEVEL 1

//Global variables
extern const char *TAG;
extern int64_t last_event_time;
extern pmw3389_handle_t sensor_handle;
extern TaskHandle_t tmx_task_handle;

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

void enter_deep_sleep(void) {      
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
