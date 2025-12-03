/**
 * @file pmw3389.c
 * @brief PMW3389DM-T3QU optical motion sensor driver implementation
 * 
 * This file implements the low-level driver for the PMW3389 optical sensor,
 * providing functions for SPI communication, sensor initialization, motion
 * tracking, and configuration management.
 * 
 * @author Ilaria
 * @date 2025-12-03
 * @version 1.0
 */

#include "pmw3389.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "PMW3389";

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
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&io_conf);
        ESP_LOGI(TAG, "MOTION pin configured: GPIO%d", dev->pin_motion);
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

esp_err_t pmw3389_upload(pmw3389_handle_t handle) {
    if (!handle) {
        ESP_LOGE(TAG, "Invalid handle");
        return ESP_ERR_INVALID_ARG;
    }

    struct pmw3389_dev_t *dev = (struct pmw3389_dev_t *)handle;
    esp_err_t ret;

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
    
    ESP_LOGI(TAG, "==============================");
    
    return ESP_OK;
}