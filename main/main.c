

// pmw3389---------------------------------
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "pmw3389.h"

static const char *TAG = "MAIN";

// Pin configuartion for ESP32-S3
#define PIN_MISO    GPIO_NUM_37
#define PIN_MOSI    GPIO_NUM_35
#define PIN_SCLK    GPIO_NUM_36
#define PIN_CS      GPIO_NUM_45
#define PIN_MOTION  GPIO_NUM_48

// SPI clock speed (2MHz)
#define SPI_CLOCK_SPEED_HZ  2000000

// Default CPI (Counts Per Inch) setting 
#define DEFAULT_CPI  1600


void app_main(void) {
     //configuration sensor hardware settings
    pmw3389_config_t config = {
        .spi_host = SPI2_HOST,
        .pin_miso = PIN_MISO,
        .pin_mosi = PIN_MOSI,
        .pin_sclk = PIN_SCLK,
        .pin_cs = PIN_CS,
        .pin_motion = PIN_MOTION,
        .spi_clock_speed_hz = SPI_CLOCK_SPEED_HZ,
    };

    
    // Run motion test (this function runs indefinitely)
    pmw3389_test_motion(&config, DEFAULT_CPI);
    
}
