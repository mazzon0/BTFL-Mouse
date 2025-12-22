/**
 * @file main.c
 * @brief PMW3389 - Minimal main with function calls only
 * 
 * Clean and simple main that uses high-level API functions from pmw3389.c
 * for sensor initialization and motion tracking. All logic is in the driver.
 * 
 * @author Ilaria
 * @date 2025-12-21
 * @version 3.0 - Minimal
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "pmw3389.h"

static const char *TAG = "PMW3389_MAIN";

// ==================== PIN CONFIGURATION ====================
#define PIN_MISO    GPIO_NUM_37
#define PIN_MOSI    GPIO_NUM_35
#define PIN_SCLK    GPIO_NUM_36
#define PIN_CS      GPIO_NUM_45
#define PIN_MOTION  -1            // -1 = Polling mode, GPIO_NUM_18 for interrupt
#define PIN_RESET   GPIO_NUM_21

// ==================== SENSOR SETTINGS ====================
#define SPI_CLOCK_SPEED_HZ  2000000  // 2MHz max for PMW3389
#define DEFAULT_CPI         3200     // Default sensitivity
#define POLL_INTERVAL_MS    20       // Polling interval (50Hz)

/**
 * @brief Main application entry point
 */
void app_main(void) {
    ESP_LOGI(TAG, "=== PMW3389 with Manual CS Driver ===");
    ESP_LOGI(TAG, "");
    
    // Sensor configuration
    pmw3389_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_miso = PIN_MISO,
        .pin_mosi = PIN_MOSI,
        .pin_sclk = PIN_SCLK,
        .pin_cs = PIN_CS,
        .pin_motion = PIN_MOTION,
        .pin_reset = PIN_RESET,
        .spi_clock_speed_hz = SPI_CLOCK_SPEED_HZ,
    };
    
    pmw3389_handle_t sensor = NULL;
    
    // Initialize and configure sensor (all-in-one)
    esp_err_t ret = pmw3389_init_and_configure(&config, DEFAULT_CPI, &sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Setup failed. Check connections and restart.");
        return;
    }
    
    // Start motion tracking (never returns)
    pmw3389_start_motion_tracking(sensor, POLL_INTERVAL_MS);
    

}