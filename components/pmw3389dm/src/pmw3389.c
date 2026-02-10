/**
 * @file pmw3389.c
 * @brief PMW3389DM-T3QU optical motion sensor driver implementation
 * 
 * This file implements the low-level driver for the PMW3389 optical sensor,
 * providing functions for SPI communication, sensor initialization, motion
 * tracking, and configuration management.
 * 
 * @author Ilaria
 * @date 2025-12-21
 * @version 3.0 - MANUAL CS CONTROL (Arduino-style)
 */

#include "pmw3389.h"
#include "pmw3389_srom.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "PMW3389";

static TaskHandle_t g_motion_task_handle = NULL;
static volatile bool g_motion_interrupt_flag = false;

/**
 * @brief Internal device structure
 */
struct pmw3389_dev_t {
    spi_device_handle_t spi;
    spi_host_device_t spi_host;
    int pin_cs;
    int pin_motion;
    int pin_reset;
    bool initialized;

    pmw3389_motion_callback_t motion_callback;
    void *callback_user_data;
};

/**
 * @brief Circular buffer (per filtro media mobile)
 */
#define FILTER_BUFFER_SIZE 4  // Campioni da mediare (4)

typedef struct {
    int16_t buffer_x[FILTER_BUFFER_SIZE];
    int16_t buffer_y[FILTER_BUFFER_SIZE];
    uint8_t index;
    bool initialized;
} motion_filter_t;

static motion_filter_t g_motion_filter = {0};

/**
 * @brief Delay microseconds
 */
static inline void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

/**
 * @brief Delay milliseconds
 */
static inline void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/**
 * @brief Applica filtro media mobile con circular buffer
 */
static void apply_moving_average_filter(int8_t raw_x, int8_t raw_y, 
                                        int16_t *out_x, int16_t *out_y) {
    // Salva nuovo campione nel buffer circolare
    g_motion_filter.buffer_x[g_motion_filter.index] = raw_x;
    g_motion_filter.buffer_y[g_motion_filter.index] = raw_y;
    
    // Incrementa indice (con wrap-around)
    g_motion_filter.index = (g_motion_filter.index + 1) % FILTER_BUFFER_SIZE;
    
    // Calcola media solo se buffer è inizializzato
    if (!g_motion_filter.initialized) {
        // Prime N letture: riempi il buffer
        *out_x = raw_x;
        *out_y = raw_y;
        
        // Segna come inizializzato dopo N campioni
        static uint8_t init_count = 0;
        if (++init_count >= FILTER_BUFFER_SIZE) {
            g_motion_filter.initialized = true;
        }
        return;
    }
    
    // Calcola media degli ultimi N campioni
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    
    for (int i = 0; i < FILTER_BUFFER_SIZE; i++) {
        sum_x += g_motion_filter.buffer_x[i];
        sum_y += g_motion_filter.buffer_y[i];
    }
    
    *out_x = (int16_t)(sum_x / FILTER_BUFFER_SIZE);
    *out_y = (int16_t)(sum_y / FILTER_BUFFER_SIZE);
}



/**
 * @brief CS Low - Start communication (Arduino style)
 */
static inline void cs_low(pmw3389_handle_t handle) {
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    gpio_set_level(dev->pin_cs, 0);
}

/**
 * @brief CS High - End communication (Arduino style)
 */
static inline void cs_high(pmw3389_handle_t handle) {
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    gpio_set_level(dev->pin_cs, 1);
}

/**
 * @brief SPI transfer single byte (Arduino SPI.transfer() style)
 */
static uint8_t spi_transfer(pmw3389_handle_t handle, uint8_t data) {
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA,
        .length = 8,  // 1 byte
        .tx_data = {data},
    };
    
    spi_device_polling_transmit(dev->spi, &trans);
    return trans.rx_data[0];
}

static void IRAM_ATTR motion_isr_handler(void* arg) {
    g_motion_interrupt_flag = true;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_motion_task_handle != NULL) {
        vTaskNotifyGiveFromISR(g_motion_task_handle, &xHigherPriorityTaskWoken);
        
        if (xHigherPriorityTaskWoken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }
}

esp_err_t pmw3389_init(const pmw3389_config_t *config, pmw3389_handle_t *out_handle) {
    esp_err_t ret;
    
    if (!config || !out_handle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Initializing PMW3389 (Manual CS mode)...");

    struct pmw3389_dev_t *dev = malloc(sizeof(struct pmw3389_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }
    memset(dev, 0, sizeof(struct pmw3389_dev_t));
    
    dev->pin_cs = config->pin_cs;
    dev->pin_motion = config->pin_motion;
    dev->pin_reset = config->pin_reset;
    dev->spi_host = config->spi_host;

    // Configure RESET pin
    if(dev->pin_reset >= 0){
        ESP_LOGI(TAG, "Configuring RESET pin: GPIO%d", dev->pin_reset);
    
        gpio_config_t reset_conf = {
            .pin_bit_mask = (1ULL << dev->pin_reset),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ret = gpio_config(&reset_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure RESET pin: %s", esp_err_to_name(ret));
            free(dev);
            return ret;
        }
        
        // Hardware RESET
        ESP_LOGI(TAG, "Performing hardware reset...");
        gpio_set_level(dev->pin_reset, 0);
        delay_ms(10);
        gpio_set_level(dev->pin_reset, 1);
        delay_ms(50);
        
        ESP_LOGI(TAG, "Hardware reset completed");
    } else {
        ESP_LOGW(TAG, "RESET pin not configured!");
    }

    // Configure CS pin (MANUAL control - like Arduino)
    ESP_LOGI(TAG, "Configuring CS pin: GPIO%d (Manual control)", dev->pin_cs);
    gpio_config_t cs_conf = {
        .pin_bit_mask = (1ULL << dev->pin_cs),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,  // Pull-up on CS
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&cs_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure CS pin: %s", esp_err_to_name(ret));
        free(dev);
        return ret;
    }
    gpio_set_level(dev->pin_cs, 1);  // CS HIGH (inactive)

    // Initialize SPI bus
    spi_bus_config_t buscfg = {
        .miso_io_num = config->pin_miso,
        .mosi_io_num = config->pin_mosi,
        .sclk_io_num = config->pin_sclk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4096,
        .flags = 0,
    };
    
    ret = spi_bus_initialize(config->spi_host, &buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "SPI bus initialization failed: %s", esp_err_to_name(ret));
        free(dev);
        return ret;
    }
    if (ret == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "SPI bus already initialized");
    }

    // Add SPI device with MANUAL CS control
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock_speed_hz,
        .mode = 3,  // SPI_MODE3
        .spics_io_num = -1,  // -1 = Manual CS control (like Arduino!)
        .queue_size = 7,
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    ret = spi_bus_add_device(config->spi_host, &devcfg, &dev->spi);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add SPI device: %s", esp_err_to_name(ret));
        spi_bus_free(config->spi_host);
        free(dev);
        return ret;
    }

    ESP_LOGI(TAG, "SPI configured: Host=%d, CS=GPIO%d (Manual), Clock=%d Hz, Mode=3",
             config->spi_host, config->pin_cs, config->spi_clock_speed_hz);

    // Configure MOTION interrupt pin (if provided)
    if (dev->pin_motion >= 0) {
        ESP_LOGI(TAG, "Configuring MOTION pin: GPIO%d", dev->pin_motion);
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dev->pin_motion),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_NEGEDGE, // ANYEDGE
        };

        gpio_config(&io_conf);
         if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to configure MOTION pin: %s", esp_err_to_name(ret));
            pmw3389_deinit(dev);
            return ret;
    }

        static bool isr_service_installed = false;
        if (!isr_service_installed) {
            ret = gpio_install_isr_service(0);
            if (ret == ESP_OK) {
                isr_service_installed = true;
                ESP_LOGI(TAG, "GPIO ISR service installed");
            } else if (ret == ESP_ERR_INVALID_STATE) {
                ESP_LOGD(TAG, "GPIO ISR service already installed");
            } else {
                ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", 
                         esp_err_to_name(ret));
                pmw3389_deinit(dev);
                return ret;
            }
        }
        
        ret = gpio_isr_handler_add(dev->pin_motion, motion_isr_handler, NULL);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to add ISR handler: %s", esp_err_to_name(ret));
            pmw3389_deinit(dev);
            return ret;
        }
        
        ESP_LOGI(TAG, "Pin MOTION configured with interrupt: GPIO%d", dev->pin_motion);
    }

    ESP_LOGI(TAG, "Waiting for hardware stabilization...");
    delay_ms(50);

    ESP_LOGI(TAG, "Executing sensor reset...");
    
    // CS pulse (Arduino style)
    cs_high(dev);
    delay_ms(1);
    cs_low(dev);
    delay_ms(1);
    cs_high(dev);
    
    ret = pmw3389_write_reg(dev, PMW3389_REG_SHUTDOWN, 0xB6);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Shutdown error");
        pmw3389_deinit(dev);
        return ret;
    }
    delay_ms(300);

    ret = pmw3389_write_reg(dev, PMW3389_REG_POWER_UP_RESET, 0x5A);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power-up reset error");
        pmw3389_deinit(dev);
        return ret;
    }
    delay_ms(50);

    uint8_t product_id = 0;
    ret = pmw3389_read_reg(dev, PMW3389_REG_PRODUCT_ID, &product_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Product ID read error");
        pmw3389_deinit(dev);
        return ret;
    }

    if (product_id != PMW3389_PRODUCT_ID) {
        ESP_LOGE(TAG, "Invalid Product ID: 0x%02X (expected 0x%02X)", 
                 product_id, PMW3389_PRODUCT_ID);
        ESP_LOGE(TAG, "Check SPI connections, power supply, and CS pin");
        pmw3389_deinit(dev);
        return ESP_ERR_NOT_FOUND;
    }

    uint8_t inv_product_id = 0;
    pmw3389_read_reg(dev, PMW3389_REG_INVERSE_PRODUCT_ID, &inv_product_id);

    if (inv_product_id != PMW3389_INVERSE_PRODUCT_ID) {
        ESP_LOGW(TAG, "Unexpected Inverse Product ID: 0x%02X (expected 0x%02X)",
                 inv_product_id, PMW3389_INVERSE_PRODUCT_ID);
    }

    ESP_LOGI(TAG, "PMW3389 detected - Product ID: 0x%02X, Inverse: 0x%02X", 
             product_id, inv_product_id);

    uint8_t revision_id = 0;
    pmw3389_read_reg(dev, PMW3389_REG_REVISION_ID, &revision_id);
    ESP_LOGI(TAG, "Revision ID: 0x%02X", revision_id);

    // Clear motion registers
    uint8_t dummy;
    pmw3389_read_reg(dev, PMW3389_REG_MOTION, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_X_L, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_X_H, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_Y_L, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_Y_H, &dummy);

    dev->initialized = true;
    *out_handle = dev;

    ESP_LOGI(TAG, "Initialization completed successfully");
    return ESP_OK;
}

esp_err_t pmw3389_read_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t *data) {
    if (!handle || !data) {
        return ESP_ERR_INVALID_ARG;
    }

    // MANUAL CS control - Arduino style!
    cs_low(handle);
    spi_transfer(handle, addr & 0x7F);  // Clear bit 7 for read
    delay_us(100);
    *data = spi_transfer(handle, 0x00);
    delay_us(1);
    cs_high(handle);
    delay_us(20);
    
    return ESP_OK;
}

esp_err_t pmw3389_write_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t data) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    // MANUAL CS control - Arduino style!
    cs_low(handle);
    spi_transfer(handle, addr | 0x80);  // Set bit 7 for write
    spi_transfer(handle, data);
    delay_us(20);
    cs_high(handle);
    delay_us(100);
    
    return ESP_OK;
}

/**
 * @brief Upload SROM firmware (Manual CS, burst mode)
 */
static esp_err_t pmw3389_upload_srom(pmw3389_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting SROM firmware upload (%d bytes)...", PMW3389_SROM_LENGTH);

    
    ret = pmw3389_write_reg(handle, PMW3389_REG_CONFIG2, 0x20);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set Config2");
        return ret;
    }
    
    // Step 1: Write 0x1D to SROM_ENABLE
    ret = pmw3389_write_reg(handle, PMW3389_REG_SROM_ENABLE, 0x1D);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable SROM download mode");
        return ret;
    }
    delay_ms(10);
    
    // Step 2: Write 0x18 to SROM_ENABLE
    ret = pmw3389_write_reg(handle, PMW3389_REG_SROM_ENABLE, 0x18);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start SROM download");
        return ret;
    }
    
    // Step 3: Burst upload (Manual CS like Arduino!)
    ESP_LOGI(TAG, "Uploading firmware in burst mode...");
    
    cs_low(handle);
    spi_transfer(handle, PMW3389_REG_SROM_LOAD_BURST | 0x80);
    //delay_us(15);
    
    // Upload all firmware bytes
    for (int i = 0; i < PMW3389_SROM_LENGTH; i++) {
        spi_transfer(handle, pmw3389_srom_data[i]);
        //delay_us(20); //15
        
        // Progress indicator
        if ((i % 512) == 0 && i > 0) {
            ESP_LOGI(TAG, "Progress: %d / %d bytes (%.1f%%)", 
                     i, PMW3389_SROM_LENGTH, (float)i * 100.0f / PMW3389_SROM_LENGTH);
        }
    }
    
    cs_high(handle);
    //delay_us(500); //200
    delay_ms(100);
    
    // Step 4: Verify SROM ID
    uint8_t srom_id;
    ret = pmw3389_read_reg(handle, PMW3389_REG_SROM_ID, &srom_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read SROM ID");
        return ret;
    }
    
    ESP_LOGI(TAG, "SROM upload complete! SROM ID: 0x%02X %s", 
             srom_id, (srom_id == 0xE8) ? "(OK)" : "(ERROR!)");
    
    if (srom_id != 0xE8) {
        ESP_LOGE(TAG, "SROM verification FAILED! Expected 0xE8");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t pmw3389_upload(pmw3389_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Configuring sensor (native mode)...");
    
    // Upload SROM firmware
    //ret = pmw3389_upload_srom(handle);
    ret = ESP_OK;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SROM upload failed");
        return ret;
    }
    
    delay_ms(10);
    
    // Additional configuration
    pmw3389_write_reg(handle, PMW3389_REG_CONFIG2, 0x00);
    
    // Set run downshift time
    pmw3389_write_reg(handle, PMW3389_REG_RUN_DOWNSHIFT, 0x0A); //0x0F

     
    pmw3389_write_reg(handle, PMW3389_REG_CONFIG1, 0x3F);  // 3200 CPI
    
    // Set rest mode rates
    pmw3389_write_reg(handle, PMW3389_REG_REST1_RATE_LOWER, 0x0A); 
    pmw3389_write_reg(handle, PMW3389_REG_REST1_RATE_UPPER, 0x00);
    pmw3389_write_reg(handle, PMW3389_REG_REST1_DOWNSHIFT, 0x00);
    
    pmw3389_write_reg(handle, PMW3389_REG_REST2_RATE_LOWER, 0x64);
    pmw3389_write_reg(handle, PMW3389_REG_REST2_RATE_UPPER, 0x00);
    pmw3389_write_reg(handle, PMW3389_REG_REST2_DOWNSHIFT, 0x00);
    
    pmw3389_write_reg(handle, PMW3389_REG_REST3_RATE_LOWER, 0xC8);
    pmw3389_write_reg(handle, PMW3389_REG_REST3_RATE_UPPER, 0x00);
    
    // Disable angle snap
    pmw3389_write_reg(handle, PMW3389_REG_ANGLE_SNAP, 0x00); 
    
    // Set lift detection threshold
    pmw3389_write_reg(handle, PMW3389_REG_LIFT_CONFIG, 0x02); //0x03
    
    // Set surface quality minimum
    pmw3389_write_reg(handle, PMW3389_REG_MIN_SQ_RUN, 0x00); //0x05
    
    // Clear motion registers
    uint8_t dummy;
    pmw3389_read_reg(handle, PMW3389_REG_MOTION, &dummy);
    pmw3389_read_reg(handle, PMW3389_REG_DELTA_X_L, &dummy);
    pmw3389_read_reg(handle, PMW3389_REG_DELTA_X_H, &dummy);
    pmw3389_read_reg(handle, PMW3389_REG_DELTA_Y_L, &dummy);
    pmw3389_read_reg(handle, PMW3389_REG_DELTA_Y_H, &dummy);
    
    ESP_LOGI(TAG, "Sensor configuration completed");
    return ESP_OK;
}

esp_err_t pmw3389_read_motion(pmw3389_handle_t handle, pmw3389_motion_data_t *motion_data) {
    if (!handle || !motion_data) {
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
   
    ret = pmw3389_write_reg(handle, PMW3389_REG_MOTION, 0x01);
    if (ret != ESP_OK) return ret;
    
    //delay_us(150);
    
    // Leggi Motion register
    uint8_t motion_reg = 0;
    ret = pmw3389_read_reg(handle, PMW3389_REG_MOTION, &motion_reg);
    if (ret != ESP_OK) return ret;
    
    motion_data->motion_detected = (motion_reg & 0x80) != 0;
    motion_data->lift_detected = (motion_reg & 0x08) != 0;
    
    
    uint8_t delta_x_l = 0;
    uint8_t delta_y_l = 0;
    
    ret = pmw3389_read_reg(handle, PMW3389_REG_DELTA_X_L, &delta_x_l);
    if (ret != ESP_OK) return ret;

    delay_us(20);
    
    ret = pmw3389_read_reg(handle, PMW3389_REG_DELTA_Y_L, &delta_y_l);
    if (ret != ESP_OK) return ret;
    
    ret = pmw3389_read_reg(handle, PMW3389_REG_SQUAL, &motion_data->squal);
    if (ret != ESP_OK) return ret;
    
    // Conversione two's complement (8-bit)
    int8_t raw_x = (int8_t)delta_x_l;
    int8_t raw_y = (int8_t)delta_y_l;

    int16_t filtered_x, filtered_y;
    apply_moving_average_filter(raw_x,raw_y, &filtered_x, &filtered_y);

    motion_data->delta_x = (int16_t)filtered_x;
    motion_data->delta_y = (int16_t)filtered_y;
    
    /*  
        int8_t raw_x = (int8_t)delta_x_l;
        int8_t raw_y = (int8_t)delta_y_l;

        // Filtro dead zone: ignora movimenti < 2 counts
        if (abs(raw_x) < 2) raw_x = 0;
        if (abs(raw_y) < 2) raw_y = 0;

        motion_data->delta_x = raw_x;
        motion_data->delta_y = raw_y;*/
    
    return ESP_OK;
}


esp_err_t pmw3389_set_cpi(pmw3389_handle_t handle, uint16_t cpi) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (cpi < 50 || cpi > 16000) {
        ESP_LOGW(TAG, "CPI out of range (50-16000), clamping");
        cpi = (cpi < 50) ? 50 : 16000;
    }
    
    cpi = (cpi / 50) * 50;
    uint8_t cpi_value = (cpi / 50) - 1;
    
    ESP_LOGI(TAG, "Setting CPI to %d (register value: 0x%02X)", cpi, cpi_value);
    
    esp_err_t ret;
    ret = pmw3389_write_reg(handle, PMW3389_REG_CONFIG1, cpi_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = pmw3389_write_reg(handle, PMW3389_REG_CONFIG2, cpi_value);
    if (ret != ESP_OK) {
        return ret;
    }
    
    delay_ms(10);
    
    uint8_t verify_cpi;
    pmw3389_read_reg(handle, PMW3389_REG_CONFIG1, &verify_cpi);
    ESP_LOGI(TAG, "CPI verification: 0x%02X (%d CPI)", verify_cpi, (verify_cpi + 1) * 50);
    
    return ESP_OK;
}

esp_err_t pmw3389_register_callback(pmw3389_handle_t handle, 
                                     pmw3389_motion_callback_t callback, 
                                     void *user_data) {
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }
    
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    dev->motion_callback = callback;
    dev->callback_user_data = user_data;
    
    if (callback) {
        ESP_LOGI(TAG, "Motion callback registered");
    } else {
        ESP_LOGI(TAG, "Motion callback unregistered");
    }
    
    return ESP_OK;
}


esp_err_t pmw3389_deinit(pmw3389_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    pmw3389_write_reg(dev, PMW3389_REG_SHUTDOWN, 0xB6);
    
    if (dev->spi) {
        spi_bus_remove_device(dev->spi);
    }
    
    free(dev);
    
    return ESP_OK;
}

esp_err_t pmw3389_dump_registers(pmw3389_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "=== PMW3389 Register Dump ===");
    
    uint8_t value;
    
    pmw3389_read_reg(handle, PMW3389_REG_PRODUCT_ID, &value);
    ESP_LOGI(TAG, "Product ID:         0x%02X", value);
    
    pmw3389_read_reg(handle, PMW3389_REG_INVERSE_PRODUCT_ID, &value);
    ESP_LOGI(TAG, "Inverse Product ID: 0x%02X", value);
    
    pmw3389_read_reg(handle, PMW3389_REG_REVISION_ID, &value);
    ESP_LOGI(TAG, "Revision ID:        0x%02X", value);
    
    pmw3389_read_reg(handle, PMW3389_REG_SROM_ID, &value);
    ESP_LOGI(TAG, "SROM ID:            0x%02X", value);
    
    pmw3389_read_reg(handle, PMW3389_REG_CONFIG1, &value);
    ESP_LOGI(TAG, "Config1 (CPI):      0x%02X (%d CPI)", value, (value + 1) * 50);
    
    pmw3389_read_reg(handle, PMW3389_REG_CONFIG2, &value);
    ESP_LOGI(TAG, "Config2 (CPI):      0x%02X (%d CPI)", value, (value + 1) * 50);
    
    pmw3389_read_reg(handle, PMW3389_REG_MOTION, &value);
    ESP_LOGI(TAG, "Motion:             0x%02X %s", value, 
             (value & PMW3389_MOTION_BIT) ? "(MOTION DETECTED)" : "(stationary)");
    
    pmw3389_read_reg(handle, PMW3389_REG_SQUAL, &value);
    ESP_LOGI(TAG, "Surface Quality:    0x%02X (%d)", value, value);
    
    pmw3389_read_reg(handle, PMW3389_REG_OBSERVATION, &value);
    ESP_LOGI(TAG, "Observation:        0x%02X", value);
    
    return ESP_OK;
}

esp_err_t pmw3389_test_motion(const pmw3389_config_t *config, uint16_t cpi){
    ESP_LOGI(TAG, "=== PMW3389 Driver Test ===");

    pmw3389_handle_t sensor = NULL;
    esp_err_t ret = pmw3389_init(config, &sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "Sensor initialized successfully!");

    ESP_LOGI(TAG, "Loading SROM firmware...");
    ret = pmw3389_upload(sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor configuration failed");
        pmw3389_deinit(sensor);
        return ret;
    }
    
    ret = pmw3389_set_cpi(sensor, cpi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CPI setting failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "\n=== MOTION READING START ===");
    ESP_LOGI(TAG, "Move the sensor to see ΔX and ΔY values");
    
    uint32_t read_count = 0;
    uint32_t motion_count = 0;
    int32_t total_x = 0;
    int32_t total_y = 0;

    while (1) {
        pmw3389_motion_data_t motion;
        
        ret = pmw3389_read_motion(sensor, &motion);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motion read error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        read_count++;

        if (motion.motion_detected || motion.delta_x != 0 || motion.delta_y != 0) {
            motion_count++;
            total_x += motion.delta_x / 100; //SENSITIVITY
            total_y += motion.delta_y / 100;

            ESP_LOGI(TAG, "ΔX: %6d | ΔY: %6d | SQUAL: %3d | Motion: %s%s",
                     motion.delta_x,
                     motion.delta_y,
                     motion.squal,
                     motion.motion_detected ? "YES" : "NO",
                     motion.lift_detected ? " [LIFT]" : "");
        }

        if (read_count % 50 == 0) {
            ESP_LOGI(TAG, "--- Statistics ---");
            ESP_LOGI(TAG, "Reads: %lu | Motions: %lu (%.1f%%)",
                     read_count, motion_count,
                     (float)motion_count * 100.0f / read_count);
            ESP_LOGI(TAG, "Total displacement - X: %ld | Y: %ld\n",
                     total_x, total_y);
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    pmw3389_deinit(sensor);
    return ESP_OK;
}


esp_err_t pmw3389_init_and_configure(const pmw3389_config_t *config, uint16_t cpi, pmw3389_handle_t *out_handle) {
    if (!config || !out_handle) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }
    
    esp_err_t ret;
    
    // Step 1: Initialize hardware
    ESP_LOGI(TAG, "Initializing PMW3389 sensor...");
    ret = pmw3389_init(config, out_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "✓ Hardware initialized");
    
    // Step 2: Upload firmware
    ESP_LOGI(TAG, "Loading SROM firmware...");
    ret = pmw3389_upload(*out_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Firmware upload failed: %s", esp_err_to_name(ret));
        pmw3389_deinit(*out_handle);
        return ret;
    }
    ESP_LOGI(TAG, "✓ Firmware loaded");
    
    // Step 3: Set CPI
    ESP_LOGI(TAG, "Setting CPI to %d...", cpi);
    ret = pmw3389_set_cpi(*out_handle, cpi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CPI setting failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "✓ CPI configured");
    
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "=== PMW3389 READY ===");
    ESP_LOGI(TAG, "");
    
    return ESP_OK;
}


void pmw3389_start_motion_tracking_interrupt(pmw3389_handle_t handle, uint16_t cpi) {
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle");
        return;
    }
    
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    if (dev->pin_motion < 0) {
        ESP_LOGE(TAG, "Motion pin not configured! Cannot use interrupt mode.");
        ESP_LOGE(TAG, "Set pin_motion to a valid GPIO (e.g., GPIO_NUM_18)");
        return;
    }
    
    ESP_LOGI(TAG, "=== MOTION TRACKING START (INTERRUPT MODE) ===");
    ESP_LOGI(TAG, "CPU will sleep until motion detected on GPIO%d", dev->pin_motion);
    ESP_LOGI(TAG, "Move the sensor to trigger interrupts");
    ESP_LOGI(TAG, "");
    
    // Set the task handle for ISR notifications
    g_motion_task_handle = xTaskGetCurrentTaskHandle();
    
    uint32_t motion_count = 0;
    uint32_t interrupt_count = 0;
    uint32_t false_wake_count = 0;
    int32_t total_x = 0;
    int32_t total_y = 0;
    
    TickType_t last_motion_time = xTaskGetTickCount();
    TickType_t start_time = xTaskGetTickCount();
    
    while (1) {
        // Wait for interrupt notification (timeout 5 seconds)
        uint32_t notification_value = ulTaskNotifyTake(
            pdTRUE,                  // Clear on exit
            pdMS_TO_TICKS(5000)      // 5 second timeout
        );
        
        if (notification_value > 0) {
            // Motion interrupt received!
            interrupt_count++;
            last_motion_time = xTaskGetTickCount();
            
            // Small delay to ensure data is ready
            vTaskDelay(pdMS_TO_TICKS(1));
            
            // Read motion data
            pmw3389_motion_data_t motion;
            esp_err_t ret = pmw3389_read_motion(handle, &motion);
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Motion read error: %s", esp_err_to_name(ret));
                continue;
            }
            
            // Check if there's actual motion
            if (motion.motion_detected || motion.delta_x != 0 || motion.delta_y != 0) {
                motion_count++;
                total_x += motion.delta_x;
                total_y += motion.delta_y;

                  if (dev->motion_callback) {
                    dev->motion_callback(&motion, dev->callback_user_data);
                }
                
                // Display motion event
                ESP_LOGI(TAG, "ΔX: %6d | ΔY: %6d | SQUAL: %3d%s",
                         motion.delta_x,
                         motion.delta_y,
                         motion.squal,
                         motion.lift_detected ? " [LIFT]" : "");
            } else {
                // False wake-up (interrupt but no motion data)
                false_wake_count++;
                ESP_LOGD(TAG, "False wake-up (interrupt without motion data)");
            }
            
            // Clear the interrupt flag
            g_motion_interrupt_flag = false;
            
            // Display statistics every 50 motion events
            if (motion_count > 0 && (motion_count % 50) == 0) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "--- Statistics ---");
                ESP_LOGI(TAG, "Motion events:    %lu", motion_count);
                ESP_LOGI(TAG, "Interrupts:       %lu", interrupt_count);
                ESP_LOGI(TAG, "False wakes:      %lu (%.1f%%)", 
                         false_wake_count, 
                         (float)false_wake_count * 100.0f / interrupt_count);
                ESP_LOGI(TAG, "Total X:          %ld counts", total_x);
                ESP_LOGI(TAG, "Total Y:          %ld counts", total_y);
                
                // Calculate physical distance
                float distance_x_mm = (float)total_x / cpi * 25.4f;
                float distance_y_mm = (float)total_y / cpi * 25.4f;
                float total_distance = sqrtf(distance_x_mm * distance_x_mm + 
                                             distance_y_mm * distance_y_mm);
                
                ESP_LOGI(TAG, "Distance X:       %.1f mm", distance_x_mm);
                ESP_LOGI(TAG, "Distance Y:       %.1f mm", distance_y_mm);
                ESP_LOGI(TAG, "Total distance:   %.1f mm", total_distance);
                ESP_LOGI(TAG, "");
            }
            
        } else {
            // Timeout (no motion for 5 seconds)
            TickType_t current_time = xTaskGetTickCount();
            uint32_t elapsed_sec = (current_time - start_time) * portTICK_PERIOD_MS / 1000;
            uint32_t idle_sec = (current_time - last_motion_time) * portTICK_PERIOD_MS / 1000;
            
            if (motion_count > 0) {
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "=== IDLE (No motion for %lu seconds) ===", idle_sec);
                ESP_LOGI(TAG, "Total runtime:    %lu seconds", elapsed_sec);
                ESP_LOGI(TAG, "Motion events:    %lu", motion_count);
                ESP_LOGI(TAG, "Avg events/min:   %.1f", 
                         elapsed_sec > 0 ? (float)motion_count * 60.0f / elapsed_sec : 0.0f);
                
                // Calculate physical distance
                float distance_x_mm = (float)total_x / cpi * 25.4f;
                float distance_y_mm = (float)total_y / cpi * 25.4f;
                float total_distance = sqrtf(distance_x_mm * distance_x_mm + 
                                             distance_y_mm * distance_y_mm);
                
                ESP_LOGI(TAG, "Total X:          %ld counts (%.1f mm)", total_x, distance_x_mm);
                ESP_LOGI(TAG, "Total Y:          %ld counts (%.1f mm)", total_y, distance_y_mm);
                ESP_LOGI(TAG, "Total distance:   %.1f mm", total_distance);
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "CPU sleeping... (waiting for motion)");
                ESP_LOGI(TAG, "");
            }
        }
    }
    
    // Cleanup (never reached in normal operation)
    g_motion_task_handle = NULL;
}



//-------------------------SPI-----------UPDATE

void pmw3389_test_motion_interrupt(const pmw3389_config_t *config, uint16_t cpi)
{
    esp_err_t ret;
    
    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return;
    }

    pmw3389_handle_t sensor = NULL;
    ret = pmw3389_init(config, &sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Sensor initialized");

    ret = pmw3389_upload(sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor configuration failed");
        pmw3389_deinit(sensor);
        return;
    }

    ESP_LOGI(TAG, "Setting CPI to %d...", cpi);
    ret = pmw3389_set_cpi(sensor, cpi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CPI setting failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "");

    g_motion_task_handle = xTaskGetCurrentTaskHandle();

    uint32_t motion_count = 0;
    uint32_t interrupt_count = 0;
    uint32_t false_wake_count = 0;
    int32_t total_x = 0;
    int32_t total_y = 0;
    
    TickType_t start_time = xTaskGetTickCount();

    while (1) {
        uint32_t notification_value = ulTaskNotifyTake(
            pdTRUE,
            pdMS_TO_TICKS(5000)
        );
        
        if (notification_value > 0) {
            interrupt_count++;
            vTaskDelay(pdMS_TO_TICKS(1));
            
            pmw3389_motion_data_t motion;
            ret = pmw3389_read_motion(sensor, &motion);
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Motion read error: %s", esp_err_to_name(ret));
                continue;
            }
            
            if (motion.motion_detected || motion.delta_x != 0 || motion.delta_y != 0) {
                motion_count++;
                total_x += motion.delta_x;
                total_y += motion.delta_y;
                
                ESP_LOGI(TAG, "ΔX: %6d | ΔY: %6d | SQUAL: %3d%s",
                         motion.delta_x,
                         motion.delta_y,
                         motion.squal,
                         motion.lift_detected ? " LIFT" : "");
            } else {
                false_wake_count++;
                ESP_LOGD(TAG, "False wake-up");
            }
            
            g_motion_interrupt_flag = false;
            
        } else {
            TickType_t current_time = xTaskGetTickCount();
            uint32_t elapsed_sec = (current_time - start_time) * portTICK_PERIOD_MS / 1000;
            
            if (elapsed_sec > 0) {
                float avg_motion_per_min = (float)motion_count * 60.0f / elapsed_sec;
                
                ESP_LOGI(TAG, "Avg motion/min:   %5.1f", avg_motion_per_min);
                ESP_LOGI(TAG, "Total X:          %6ld counts", total_x);
                ESP_LOGI(TAG, "Total Y:          %6ld counts", total_y);
                
                float distance_x_mm = (float)total_x / cpi * 25.4f;
                float distance_y_mm = (float)total_y / cpi * 25.4f;
                float total_distance = sqrtf(distance_x_mm * distance_x_mm + 
                                             distance_y_mm * distance_y_mm);
                
                ESP_LOGI(TAG, "Distance X:       %6.1f mm", distance_x_mm);
                ESP_LOGI(TAG, "Distance Y:       %6.1f mm", distance_y_mm);
                ESP_LOGI(TAG, "Total distance:   %6.1f mm", total_distance);
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, "CPU sleeping...");
                ESP_LOGI(TAG, "");
            }
        }
    }

    g_motion_task_handle = NULL;
    pmw3389_deinit(sensor);
}
