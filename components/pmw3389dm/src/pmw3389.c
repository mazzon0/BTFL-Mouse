/**
 * @file pmw3389.c
 * @brief PMW3389DM-T3QU optical motion sensor driver implementation
 * 
 * This file implements the low-level driver for the PMW3389 optical sensor,
 * providing functions for SPI communication, sensor initialization, motion
 * tracking, and configuration management.
 * 
 * @author Ilaria
 * @date 2025-12-04
 * @version 2.0
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

static TaskHandle_t g_motion_task_handle = NULL;  // Handle task
static volatile bool g_motion_interrupt_flag = false;  // Flag interrupt

/**
 * @brief Internal device structure
 * 
 * Contains all hardware-specific information and state
 * for a single PMW3389 sensor instance.
 */

struct pmw3389_dev_t {
    spi_device_handle_t spi;
    spi_host_device_t spi_host;
    int pin_cs;
    int pin_motion;
    bool initialized;
};

/**
 * @brief Delay for specified microseconds
 * 
 * @param us Microseconds to delay
 */
static inline void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

/**
 * @brief Delay for specified milliseconds
 * 
 * @param ms Milliseconds to delay
 */
static inline void delay_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void IRAM_ATTR motion_isr_handler(void* arg) {
    // flag set
    g_motion_interrupt_flag = true;
    
    //wake-up of the main task
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (g_motion_task_handle != NULL) {
        // notify the task of a movment
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

    ESP_LOGI(TAG, "Initializing PMW3389...");

    struct pmw3389_dev_t *dev = malloc(sizeof(struct pmw3389_dev_t));
    if (!dev) {
        ESP_LOGE(TAG, "Memory allocation failed");
        return ESP_ERR_NO_MEM;
    }
    memset(dev, 0, sizeof(struct pmw3389_dev_t));
    
    dev->pin_cs = config->pin_cs;
    dev->pin_motion = config->pin_motion;
    dev->spi_host = config->spi_host;

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

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = config->spi_clock_speed_hz,
        .mode = 3,
        .spics_io_num = config->pin_cs,
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

    ESP_LOGI(TAG, "SPI configured: Host=%d, CS=GPIO%d, Clock=%d Hz, Mode=3",
             config->spi_host, config->pin_cs, config->spi_clock_speed_hz);

    if (dev->pin_motion >= 0) {
        gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << dev->pin_motion),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_POSEDGE, //interrupt (HIGH)
        };
        gpio_config(&io_conf);

    static bool isr_service_installed = false;
    if (!isr_service_installed) {
        ret = gpio_install_isr_service(0);  // 0 = default priority
        if (ret == ESP_OK) {
            isr_service_installed = true;
            ESP_LOGI(TAG, "GPIO ISR service installed");
        } else if (ret == ESP_ERR_INVALID_STATE) {
            //already install(ok)
            ESP_LOGD(TAG, "GPIO ISR service already installed");
        } else {
            ESP_LOGE(TAG, "Failed to install GPIO ISR service: %s", 
                     esp_err_to_name(ret));
            pmw3389_deinit(dev);
            return ret;
        }
    }
    
    //associate ISR handler with a specific GPIO (motion_isr_handler)
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
        ESP_LOGE(TAG, "Check SPI connections, power supply, and bypass capacitors");
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

    //clear motion registers
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

    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    uint8_t tx_data[2] = {addr & 0x7F, 0x00};
    uint8_t rx_data[2] = {0};

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
        .user = NULL,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register read error 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    *data = rx_data[1];
    delay_us(PMW3389_TSRR);
    
    return ESP_OK;
}

esp_err_t pmw3389_write_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t data) {

    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    uint8_t tx_data[2] = {addr | 0x80, data};

    spi_transaction_t trans = {
        .length = 16,
        .tx_buffer = tx_data,
        .rx_buffer = NULL,
        .user = NULL,
    };

    esp_err_t ret = spi_device_polling_transmit(dev->spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Register write error 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }

    delay_us(PMW3389_TSWW);
    
    return ESP_OK;
}

/**
 * @brief Upload SROM firmware to PMW3389 sensor
 * 
 * @param handle Device handle
 * @return ESP_OK on success
 */
static esp_err_t pmw3389_upload_srom(pmw3389_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }
    
    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    esp_err_t ret;
    
    ESP_LOGI(TAG, "Starting SROM firmware upload...");
    
    // Step 1: Write 0x1D to SROM_ENABLE register
    ret = pmw3389_write_reg(dev, PMW3389_REG_SROM_ENABLE, 0x1D);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SROM enable init failed");
        return ret;
    }
    delay_ms(10);
    
    // Step 2: Write 0x18 to SROM_ENABLE (start download)
    ret = pmw3389_write_reg(dev, PMW3389_REG_SROM_ENABLE, 0x18);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SROM download start failed");
        return ret;
    }
    delay_ms(1);
    
    // Step 3: Write firmware via burst mode
    ESP_LOGI(TAG, "Uploading %d bytes of firmware...", PMW3389_SROM_LENGTH);
    
    // Send burst write address
    uint8_t burst_addr = PMW3389_REG_SROM_LOAD_BURST | 0x80;
    spi_transaction_t trans = {
        .length = 8,
        .tx_buffer = &burst_addr,
        .rx_buffer = NULL
    };
    
    ret = spi_device_polling_transmit(dev->spi, &trans);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Burst command failed");
        return ret;
    }
    delay_us(15);
    
    // Upload firmware bytes
    for (uint16_t i = 0; i < PMW3389_SROM_LENGTH; i++) {
        trans.length = 8;
        trans.tx_buffer = &pmw3389_srom_data[i];
        
        ret = spi_device_polling_transmit(dev->spi, &trans);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SROM upload failed at byte %d", i);
            return ret;
        }
        delay_us(15);
        
        // Progress every 512 bytes
        if ((i & 0x1FF) == 0 && i > 0) {
            ESP_LOGI(TAG, "Progress: %d%%", (i * 100) / PMW3389_SROM_LENGTH);
        }
    }
    
    ESP_LOGI(TAG, "Firmware upload complete (100%%)");
    
    // Step 4: Wait for completion
    delay_ms(10);
    
    // Step 5: Read SROM ID to verify
    uint8_t srom_id;
    ret = pmw3389_read_reg(dev, PMW3389_REG_SROM_ID, &srom_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SROM verification failed");
        return ret;
    }
    
    ESP_LOGI(TAG, "SROM ID after upload: 0x%02X", srom_id);
    
    // Expected SROM ID for PMW3389 with firmware 0xE8
    if (srom_id == 0x04 || srom_id == 0x05 || srom_id == 0x06) {
        ESP_LOGI(TAG, "SROM upload verified successfully!");
    } else {
        ESP_LOGW(TAG, "Unexpected SROM ID: 0x%02X (expected 0x04/0x05/0x06)", srom_id);
    }
    
    delay_ms(10);
    
    return ESP_OK;
}


esp_err_t pmw3389_upload(pmw3389_handle_t handle) {
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    esp_err_t ret;

    ESP_LOGI(TAG, "Uploading SROM firmware...");
    ret = pmw3389_upload_srom(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SROM firmware upload failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "SROM firmware uploaded successfully");

    ESP_LOGI(TAG, "Configuring sensor in native mode");

    uint8_t status;
    ret = pmw3389_read_reg(dev, PMW3389_REG_OBSERVATION, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor status read error");
        return ret;
    }
    
    ESP_LOGD(TAG, "Initial sensor status: 0x%02X", status);

    ESP_LOGD(TAG, "Configuring Rest Mode...");
    ret = pmw3389_write_reg(dev, PMW3389_REG_CONFIG2, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Rest Mode configuration error");
        return ret;
    }
    delay_ms(10);

    uint8_t srom_id;
    ret = pmw3389_read_reg(dev, PMW3389_REG_SROM_ID, &srom_id);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SROM ID read error");
        return ret;
    }
    
    ESP_LOGI(TAG, "SROM ID: 0x%02X (internal firmware)", srom_id);

    ESP_LOGD(TAG, "Disabling Angle Snap...");
    ret = pmw3389_write_reg(dev, PMW3389_REG_ANGLE_SNAP, 0x00);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Angle Snap configuration error");
    }
    delay_ms(1);

    ESP_LOGD(TAG, "Configuring Lift Detection...");
    ret = pmw3389_write_reg(dev, PMW3389_REG_LIFT_CONFIG, 0x02);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Lift Detection configuration error");
    }
    delay_ms(1);

    ESP_LOGD(TAG, "Configuring Surface Quality threshold...");
    ret = pmw3389_write_reg(dev, PMW3389_REG_MIN_SQ_RUN, 0x05);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "SQUAL threshold configuration error");
    }
    delay_ms(1);

    ESP_LOGD(TAG, "Configuring power management...");
    ret = pmw3389_write_reg(dev, PMW3389_REG_RUN_DOWNSHIFT, 0x32);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Downshift configuration error");
    }
    delay_ms(1);

    ret = pmw3389_read_reg(dev, PMW3389_REG_OBSERVATION, &status);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Final status read error");
        return ret;
    }

    if (status & 0x40) {
        ESP_LOGW(TAG, "Warning: SROM error bit active (0x%02X)", status);
        ESP_LOGW(TAG, "Normal without external SROM firmware");
    }

    uint8_t squal;
    ret = pmw3389_read_reg(dev, PMW3389_REG_SQUAL, &squal);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Initial Surface Quality: %d", squal);
    }

    uint8_t dummy;
    pmw3389_read_reg(dev, PMW3389_REG_MOTION, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_X_L, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_X_H, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_Y_L, &dummy);
    pmw3389_read_reg(dev, PMW3389_REG_DELTA_Y_H, &dummy);

    ESP_LOGI(TAG, "Sensor configured successfully in native mode");
    ESP_LOGI(TAG, "Ready for tracking (optimal on quality mouse pads)");
    
    return ESP_OK;
}

esp_err_t pmw3389_read_motion(pmw3389_handle_t handle, pmw3389_motion_data_t *motion_data) {
    if (!handle || !motion_data) {
        return ESP_ERR_INVALID_ARG;
    }

    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    esp_err_t ret;

    uint8_t motion_reg;
    ret = pmw3389_read_reg(dev, PMW3389_REG_MOTION, &motion_reg);
    if (ret != ESP_OK) {
        return ret;
    }

    motion_data->motion_detected = (motion_reg & PMW3389_MOTION_BIT) != 0;

    uint8_t delta_x_l, delta_x_h;
    ret = pmw3389_read_reg(dev, PMW3389_REG_DELTA_X_L, &delta_x_l);
    if (ret != ESP_OK) return ret;
    ret = pmw3389_read_reg(dev, PMW3389_REG_DELTA_X_H, &delta_x_h);
    if (ret != ESP_OK) return ret;

    uint8_t delta_y_l, delta_y_h;
    ret = pmw3389_read_reg(dev, PMW3389_REG_DELTA_Y_L, &delta_y_l);
    if (ret != ESP_OK) return ret;
    ret = pmw3389_read_reg(dev, PMW3389_REG_DELTA_Y_H, &delta_y_h);
    if (ret != ESP_OK) return ret;

    ret = pmw3389_read_reg(dev, PMW3389_REG_SQUAL, &motion_data->squal);
    if (ret != ESP_OK) return ret;

    motion_data->delta_x = (int16_t)((delta_x_h << 8) | delta_x_l);
    motion_data->delta_y = (int16_t)((delta_y_h << 8) | delta_y_l);

    return ESP_OK;
}

esp_err_t pmw3389_set_cpi(pmw3389_handle_t handle, uint16_t cpi) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    if (cpi < 50 || cpi > 16000) {
        ESP_LOGE(TAG, "CPI out of range: %d (valid: 50-16000)", cpi);
        return ESP_ERR_INVALID_ARG;
    }

    if (cpi % 50 != 0) {
        ESP_LOGW(TAG, "CPI not multiple of 50, rounding to %d", (cpi / 50) * 50);
        cpi = (cpi / 50) * 50;
    }

    uint8_t cpi_value = (cpi / 50) - 1;

    esp_err_t ret;
    
    ret = pmw3389_write_reg(handle, PMW3389_REG_CONFIG1, cpi_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config1 write error");
        return ret;
    }
    
    ret = pmw3389_write_reg(handle, PMW3389_REG_CONFIG2, cpi_value);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Config2 write error");
        return ret;
    }

    ESP_LOGI(TAG, "CPI set to: %d (register value: 0x%02X)", cpi, cpi_value);
    
    return ESP_OK;
}

esp_err_t pmw3389_deinit(pmw3389_handle_t handle) {
    if (!handle) {
        return ESP_ERR_INVALID_ARG;
    }

    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    
    ESP_LOGI(TAG, "Deinitializing driver...");

    if (dev->initialized) {
        pmw3389_write_reg(dev, PMW3389_REG_SHUTDOWN, 0xB6);
    }

    if (dev->spi) {
        esp_err_t ret = spi_bus_remove_device(dev->spi);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "SPI device removal error: %s", esp_err_to_name(ret));
        }
    }

    free(dev);
    ESP_LOGI(TAG, "Driver deinitialized");
    
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

    //sensor initialization 
    pmw3389_handle_t sensor = NULL;
    esp_err_t ret = pmw3389_init(config, &sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Sensor initialization failed: %s", esp_err_to_name(ret));
        ESP_LOGE(TAG, "Check SPI connections, power supply (3.3V), and pins");
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

    //set CPI - tracking resolution
    
    ret = pmw3389_set_cpi(sensor, cpi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "CPI setting failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "\n=== MOTION READING START ===");
    ESP_LOGI(TAG, "Move the sensor to see ΔX and ΔY values");
    

    //statics tracking variables
    uint32_t read_count = 0;
    uint32_t motion_count = 0;
    int32_t total_x = 0;
    int32_t total_y = 0;

    while (1) {
        pmw3389_motion_data_t motion;
        
        //read motion data from sensor
        ret = pmw3389_read_motion(sensor, &motion);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Motion read error: %s", esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        read_count++;

        //print only if movement is detected
        if (motion.motion_detected || motion.delta_x != 0 || motion.delta_y != 0) {
            motion_count++;
            total_x += motion.delta_x;
            total_y += motion.delta_y;

            ESP_LOGI(TAG, "ΔX: %6d | ΔY: %6d | SQUAL: %3d | Motion: %s%s",
                     motion.delta_x,
                     motion.delta_y,
                     motion.squal,
                     motion.motion_detected ? "YES" : "NO",
                     motion.lift_detected ? " [LIFT]" : "");
        }

        //statistics every 5s (5oreads)
        if (read_count % 50 == 0) {
            ESP_LOGI(TAG, "--- Statistics ---");
            ESP_LOGI(TAG, "Reads: %lu | Motions: %lu (%.1f%%)",
                     read_count, motion_count,
                     (float)motion_count * 100.0f / read_count);
            ESP_LOGI(TAG, "Total displacement - X: %ld | Y: %ld\n",
                     total_x, total_y);
        }

        // delay between reads - 100ms (10Hz)
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    //cleanup
    pmw3389_deinit(sensor);
    return ESP_OK;
}

/**
 * @brief Test motion with interrupt 
 * 
 * 
 * The task is asleep when there isn't moviment
 */

 
void pmw3389_test_motion_interrupt(const pmw3389_config_t *config, uint16_t cpi)
{
    esp_err_t ret;
    
    if (!config) {
        ESP_LOGE(TAG, "Invalid configuration pointer");
        return;
    }

    // Sensor initialization 
    pmw3389_handle_t sensor = NULL;
    ret = pmw3389_init(config, &sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Sensor initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, " Sensor initialized");

    // Configure sensor
    ret = pmw3389_upload(sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, " Sensor configuration failed");
        pmw3389_deinit(sensor);
        return;
    }

    // Set CPI
    ESP_LOGI(TAG, "Setting CPI to %d...", cpi);
    ret = pmw3389_set_cpi(sensor, cpi);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, " CPI setting failed: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "");

    // Salva handle del task corrente per ISR
    g_motion_task_handle = xTaskGetCurrentTaskHandle();

    // Statistics
    uint32_t motion_count = 0;
    uint32_t interrupt_count = 0;
    uint32_t false_wake_count = 0;
    int32_t total_x = 0;
    int32_t total_y = 0;
    
    TickType_t start_time = xTaskGetTickCount();

    // Main loop with interrupt
    while (1) {
        //wait interrupt
        uint32_t notification_value = ulTaskNotifyTake(
            pdTRUE,              // Clear notification
            pdMS_TO_TICKS(5000)  // Timeout 5 s 
        );
        
        if (notification_value > 0) {
    
            interrupt_count++;
            vTaskDelay(pdMS_TO_TICKS(1));
            
            // read movment data
            pmw3389_motion_data_t motion;
            ret = pmw3389_read_motion(sensor, &motion);
            
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, " Motion read error: %s", esp_err_to_name(ret));
                continue;
            }
            
            // verify movment
            if (motion.motion_detected || motion.delta_x != 0 || motion.delta_y != 0) {
                
                motion_count++;
                total_x += motion.delta_x;
                total_y += motion.delta_y;
                
                ESP_LOGI(TAG, " ΔX: %6d | ΔY: %6d | SQUAL: %3%s",
                         motion.delta_x,
                         motion.delta_y,
                         motion.squal,
                         motion.lift_detected ? " LIFT" : "");
            } else {
                // FALSE WAKE 
                false_wake_count++;
                ESP_LOGD(TAG, " False wake-up (no motion in registers)");
            }
            
            // Reset flag
            g_motion_interrupt_flag = false;
            
        } else {
            // TIMEOUT (5 s without movment)
            // Stampa statistiche
            
            TickType_t current_time = xTaskGetTickCount();
            uint32_t elapsed_sec = (current_time - start_time) * portTICK_PERIOD_MS / 1000;
            
            if (elapsed_sec > 0) {
                float avg_motion_per_min = (float)motion_count * 60.0f / elapsed_sec;
                
                ESP_LOGI(TAG, " Avg motion/min:   %5.1f  ", avg_motion_per_min);
                ESP_LOGI(TAG, " Total X:          %6ld counts  ", total_x);
                ESP_LOGI(TAG, " Total Y:          %6ld counts  ", total_y);
                
                float distance_x_mm = (float)total_x / cpi * 25.4f;
                float distance_y_mm = (float)total_y / cpi * 25.4f;
                float total_distance = sqrtf(distance_x_mm * distance_x_mm + 
                                             distance_y_mm * distance_y_mm);
                
                ESP_LOGI(TAG, " Distance X:       %6.1f mm          ", distance_x_mm);
                ESP_LOGI(TAG, " Distance Y:       %6.1f mm          ", distance_y_mm);
                ESP_LOGI(TAG, " Total distance:   %6.1f mm          ", total_distance);
                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG, " CPU sleeping... (waiting for movement)");
                ESP_LOGI(TAG, "");
            }
        }
    }

    // Cleanup (never reached)
    g_motion_task_handle = NULL;
    pmw3389_deinit(sensor);
}