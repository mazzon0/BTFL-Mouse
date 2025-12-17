/**
 * @file main.c
 * @brief PMW3389 driver test program - simplified entry point
 * 
 * This is a simplified main application that demonstrates the usage of
 * the PMW3389 optical sensor driver through a single test function call.
 * All testing logic is encapsulated in pmw3389_test_motion().
 * 
 * @author Ilaria
 * @date 2025-12-15
 * @version 2.1 - FIXED: Added pin_reset support
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "pmw3389.h"

// Pin configuration for ESP32-S3
#define PIN_MISO    GPIO_NUM_37
#define PIN_MOSI    GPIO_NUM_35
#define PIN_SCLK    GPIO_NUM_36
#define PIN_CS      GPIO_NUM_45
#define PIN_MOTION  GPIO_NUM_18
#define PIN_RESET   GPIO_NUM_21    

// SPI clock speed (2MHz - raccomandato per PMW3389)
#define SPI_CLOCK_SPEED_HZ  2000000

// Default CPI (Counts Per Inch) setting 
#define DEFAULT_CPI  3200

/**
 * @brief Main application entry point
 * 
 * Configures the PMW3389 sensor hardware parameters and starts
 * the motion test function that handles all initialization,
 * configuration, and continuous motion reading.
 */
void app_main(void) {
    // Configuration sensor hardware settings
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

    // Run motion test (this function runs indefinitely)
    pmw3389_test_motion(&config, DEFAULT_CPI);
}