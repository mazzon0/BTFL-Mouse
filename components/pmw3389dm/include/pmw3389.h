/**
 * @file pmw3389.h
 * @brief Driver for PMW3389DM-T3QU optical motion sensor
 * 
 * This header provides the interface for controlling the PMW3389 optical sensor,
 * commonly used in high-precision gaming mice. It includes functions for 
 * initialization, motion reading, CPI configuration, and register access.
 * 
 * @author Ilaria
 * @date 2025-12-21
 * @version 3.0 - MANUAL CS CONTROL 
 */

#ifndef PMW3389_H
#define PMW3389_H

#include <stdint.h>
#include <stdbool.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ==================== REGISTER ADDRESSES ====================
#define PMW3389_REG_PRODUCT_ID          0x00
#define PMW3389_REG_REVISION_ID         0x01
#define PMW3389_REG_MOTION              0x02
#define PMW3389_REG_DELTA_X_L           0x03
#define PMW3389_REG_DELTA_X_H           0x04
#define PMW3389_REG_DELTA_Y_L           0x05
#define PMW3389_REG_DELTA_Y_H           0x06
#define PMW3389_REG_SQUAL               0x07
#define PMW3389_REG_RAW_DATA_SUM        0x08
#define PMW3389_REG_MAXIMUM_RAW_DATA    0x09
#define PMW3389_REG_MINIMUM_RAW_DATA    0x0A
#define PMW3389_REG_SHUTTER_LOWER       0x0B
#define PMW3389_REG_SHUTTER_UPPER       0x0C
#define PMW3389_REG_CONTROL             0x0D
#define PMW3389_REG_CONFIG1             0x0F
#define PMW3389_REG_CONFIG2             0x10
#define PMW3389_REG_ANGLE_TUNE          0x11
#define PMW3389_REG_FRAME_CAPTURE       0x12
#define PMW3389_REG_SROM_ENABLE         0x13
#define PMW3389_REG_RUN_DOWNSHIFT       0x14
#define PMW3389_REG_REST1_RATE_LOWER    0x15
#define PMW3389_REG_REST1_RATE_UPPER    0x16
#define PMW3389_REG_REST1_DOWNSHIFT     0x17
#define PMW3389_REG_REST2_RATE_LOWER    0x18
#define PMW3389_REG_REST2_RATE_UPPER    0x19
#define PMW3389_REG_REST2_DOWNSHIFT     0x1A
#define PMW3389_REG_REST3_RATE_LOWER    0x1B
#define PMW3389_REG_REST3_RATE_UPPER    0x1C
#define PMW3389_REG_OBSERVATION         0x24
#define PMW3389_REG_DATA_OUT_LOWER      0x25
#define PMW3389_REG_DATA_OUT_UPPER      0x26
#define PMW3389_REG_SROM_ID             0x2A
#define PMW3389_REG_MIN_SQ_RUN          0x2B
#define PMW3389_REG_RAW_DATA_THRESHOLD  0x2C
#define PMW3389_REG_CONTROL2            0x2D
#define PMW3389_REG_CONFIG5_L           0x2E
#define PMW3389_REG_CONFIG5_H           0x2F
#define PMW3389_REG_POWER_UP_RESET      0x3A
#define PMW3389_REG_SHUTDOWN            0x3B
#define PMW3389_REG_INVERSE_PRODUCT_ID  0x3F
#define PMW3389_REG_LIFTCUTOFF_CAL3     0x41
#define PMW3389_REG_ANGLE_SNAP          0x42
#define PMW3389_REG_LIFTCUTOFF_CAL1     0x4A
#define PMW3389_REG_MOTION_BURST        0x50
#define PMW3389_REG_SROM_LOAD_BURST     0x62
#define PMW3389_REG_LIFT_CONFIG         0x63
#define PMW3389_REG_RAW_DATA_BURST      0x64
#define PMW3389_REG_LIFTCUTOFF_CAL2     0x65
#define PMW3389_REG_LIFTCUTOFF_CAL_TIMEOUT      0x71
#define PMW3389_REG_LIFTCUTOFF_CAL_MIN_LENGTH   0x72
#define PMW3389_REG_PWM_PERIOD_CNT      0x73
#define PMW3389_REG_PWM_WIDTH_CNT       0x74

// ==================== PRODUCT IDs ====================
#define PMW3389_PRODUCT_ID              0x47
#define PMW3389_INVERSE_PRODUCT_ID      0xB8

// ==================== MOTION REGISTER BITS ====================
#define PMW3389_MOTION_BIT              (1 << 7)
#define PMW3389_MOTION_LIFT             (1 << 3)
#define PMW3389_MOTION_RES              (1 << 2)
#define PMW3389_MOTION_RVALID           (1 << 0)

// ==================== TIMING CONSTANTS (μs) ====================
#define PMW3389_TSWW    180  // Write-to-write delay
#define PMW3389_TSWR    20   // Write-to-read delay
#define PMW3389_TSRW    20   // Read-to-write delay
#define PMW3389_TSRR    20   // Read-to-read delay

/**
 * @brief Structure containing motion data from the sensor
 * 
 * This structure holds the delta movement values, surface quality,
 * and motion/lift detection flags from a single sensor reading.
 */
typedef struct {
    int16_t delta_x;        ///< X-axis displacement in counts
    int16_t delta_y;        ///< Y-axis displacement in counts
    uint8_t squal;          ///< Surface quality (0-255)
    bool motion_detected;   ///< Motion flag from sensor
    bool lift_detected;     ///< Lift detection flag
} pmw3389_motion_data_t;

/**
 * @brief Configuration structure for PMW3389 initialization
 * 
 * This structure contains all hardware configuration parameters
 * needed to initialize the sensor, including SPI settings and GPIO pins.
 */
typedef struct {
    spi_host_device_t spi_host;     ///< SPI host (SPI2_HOST or SPI3_HOST)
    int pin_miso;                    ///< MISO pin number
    int pin_mosi;                    ///< MOSI pin number
    int pin_sclk;                    ///< SCLK pin number
    int pin_cs;                      ///< CS pin number (manually controlled)
    int pin_motion;                  ///< Motion interrupt pin (-1 to disable)
    int pin_reset;                   ///< Reset pin (-1 to disable)
    int spi_clock_speed_hz;          ///< SPI clock speed in Hz (max 2MHz)
} pmw3389_config_t;

/**
 * @brief Opaque handle type for PMW3389 device
 * 
 * This handle represents an initialized PMW3389 sensor instance
 * and must be passed to all driver functions.
 */
typedef void* pmw3389_handle_t;

// ==================== CORE FUNCTIONS ====================

/**
 * @brief Initialize the PMW3389 sensor with manual CS control
 * 
 * This function initializes the SPI bus, configures GPIO pins, performs hardware reset, and verifies 
 * communication with the sensor by reading its product ID.
 * 
 * @param config Pointer to configuration structure containing hardware settings
 * @param out_handle Pointer to store the initialized device handle
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if config or out_handle is NULL
 *     - ESP_ERR_NO_MEM if memory allocation fails
 *     - ESP_ERR_NOT_FOUND if sensor product ID is invalid
 *     - Other ESP_ERR codes for SPI or GPIO errors
 * 
 * @note CS pin is configured as GPIO output for manual control
 * @note SPI device is configured with spics_io_num = -1 for manual CS
 * 
 * @code
 * pmw3389_config_t config = {
 *     .spi_host = SPI2_HOST,
 *     .pin_miso = GPIO_NUM_37,
 *     .pin_mosi = GPIO_NUM_35,
 *     .pin_sclk = GPIO_NUM_36,
 *     .pin_cs = GPIO_NUM_45,
 *     .pin_motion = -1,  // Polling mode
 *     .pin_reset = GPIO_NUM_21,
 *     .spi_clock_speed_hz = 2000000
 * };
 * pmw3389_handle_t sensor;
 * esp_err_t ret = pmw3389_init(&config, &sensor);
 * @endcode
 */
esp_err_t pmw3389_init(const pmw3389_config_t *config, pmw3389_handle_t *out_handle);

/**
 * @brief Read a single register from the sensor
 * 
 * Performs a low-level SPI read operation with manual CS control.
 * CS is pulled low, address byte is sent (bit 7 = 0 for read),
 * data is read, then CS is pulled high.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param addr Register address to read (0x00-0x7F)
 * @param data Pointer to store the read byte value
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or data is NULL
 * 
 * @note Includes proper timing delays as per datasheet
 * 
 * @code
 * uint8_t product_id;
 * esp_err_t ret = pmw3389_read_reg(sensor, PMW3389_REG_PRODUCT_ID, &product_id);
 * @endcode
 */
esp_err_t pmw3389_read_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t *data);

/**
 * @brief Write a single register to the sensor
 * 
 * Performs a low-level SPI write operation with manual CS control.
 * CS is pulled low, address byte is sent (bit 7 = 1 for write),
 * data byte is sent, then CS is pulled high.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param addr Register address to write (0x00-0x7F)
 * @param data Byte value to write to the register
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 * 
 * @note Includes proper timing delays as per datasheet
 * 
 * @code
 * esp_err_t ret = pmw3389_write_reg(sensor, PMW3389_REG_CONFIG1, 0x1F);
 * @endcode
 */
esp_err_t pmw3389_write_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t data);

/**
 * @brief Upload SROM firmware and configure sensor
 * 
 * This function uploads the SROM firmware via burst mode and applies
 * optimal configuration settings including rest modes, lift detection,
 * and angle snap disable. Must be called after pmw3389_init().
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 *     - ESP_FAIL if SROM verification fails
 * 
 * @note SROM data must be available in pmw3389_srom.h
 * @note Upload takes approximately 50-60ms to complete
 * 
 * @code
 * esp_err_t ret = pmw3389_upload(sensor);
 * if (ret == ESP_OK) {
 *     ESP_LOGI(TAG, "Sensor ready for operation");
 * }
 * @endcode
 */
esp_err_t pmw3389_upload(pmw3389_handle_t handle);

/**
 * @brief Read motion data from the sensor
 * 
 * Retrieves the current motion information including X/Y displacement,
 * surface quality, and motion/lift detection status. This function triggers
 * a motion read by writing to the Motion register before reading the delta values.
 * 
 * The function reads 8-bit delta values (low bytes only) provide a range
 * of -128 to +127 counts per read.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param motion_data Pointer to structure to store the motion data
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or motion_data is NULL
 * 
 * @note Delta values are 8-bit signed integers in counts (range: -128 to +127)
 * @note Physical displacement in mm = (delta_counts / CPI) * 25.4
 * @note Function automatically triggers motion register before reading
 * @note For high-speed movements requiring 16-bit range, consider implementing
 *       a separate high-speed read function
 * 
 * @code
 * pmw3389_motion_data_t motion;
 * esp_err_t ret = pmw3389_read_motion(sensor, &motion);
 * if (ret == ESP_OK && motion.motion_detected) {
 *     printf("Movement: X=%d, Y=%d, SQUAL=%d\n", 
 *            motion.delta_x, motion.delta_y, motion.squal);
 * }
 * @endcode
 */
esp_err_t pmw3389_read_motion(pmw3389_handle_t handle, pmw3389_motion_data_t *motion_data);

/**
 * @brief Set the CPI (Counts Per Inch) resolution
 * 
 * Configures the sensor's tracking resolution. Higher CPI values result
 * in more sensitive motion tracking. Value is automatically rounded to
 * nearest multiple of 50.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param cpi Desired CPI value (range: 50-16000, multiples of 50)
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 * 
 * @note Out-of-range values are automatically clamped
 * @note Non-multiples of 50 are rounded down
 * 
 * @code
 * esp_err_t ret = pmw3389_set_cpi(sensor, 3200);
 * @endcode
 */
esp_err_t pmw3389_set_cpi(pmw3389_handle_t handle, uint16_t cpi);

/**
 * @brief Deinitialize the sensor and free resources
 * 
 * Shuts down the sensor, removes the SPI device, and frees all
 * allocated memory. The handle becomes invalid after this call.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 * 
 * @code
 * pmw3389_deinit(sensor);
 * sensor = NULL;
 * @endcode
 */
esp_err_t pmw3389_deinit(pmw3389_handle_t handle);

/**
 * @brief Dump all important sensor registers for debugging
 * 
 * Reads and logs key sensor registers including product ID, configuration,
 * motion status, and surface quality for diagnostic purposes.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 * 
 * @code
 * pmw3389_dump_registers(sensor);
 * @endcode
 */
esp_err_t pmw3389_dump_registers(pmw3389_handle_t handle);

// ==================== TEST FUNCTIONS ====================

/** POLLING ----------------------------
 * @brief Run motion test with polling mode
 * 
 * Comprehensive test function that initializes the sensor, uploads firmware,
 * sets CPI, and enters continuous polling loop displaying motion data and
 * statistics. Runs indefinitely until program termination.
 * 
 * @param config Pointer to sensor configuration structure
 * @param cpi Desired CPI value (50-16000)
 * 
 * @return 
 *     - ESP_OK (never returns in normal operation)
 *     - ESP_ERR codes on initialization failure
 * 
 * @note Function never returns under normal conditions
 * @note Displays motion events and statistics every 50 reads
 * @note Alternative to using pmw3389_init_and_configure() + pmw3389_start_motion_tracking()
 * 
 * @code
 * pmw3389_config_t config = { ... };
 * pmw3389_test_motion(&config, 3200);
 * @endcode
 */
esp_err_t pmw3389_test_motion(const pmw3389_config_t *config, uint16_t cpi);
//-------------------------------------------------------

/**
 * @brief Run motion test with interrupt mode
 * 
 * Test function using hardware interrupt for motion detection. CPU sleeps
 * until motion interrupt occurs, then reads and displays motion data with
 * detailed statistics including distance calculations.
 * 
 * @param config Pointer to sensor configuration (pin_motion must be valid)
 * @param cpi Desired CPI value (50-16000)
 * 
 * @note Requires pin_motion to be configured in config
 * @note Function never returns under normal conditions
 * @note Displays statistics after 5 seconds of inactivity
 * @note Calculates physical distances in mm based on CPI
 * @note This is an OPTIONAL function for interrupt-based applications
 * 
 * @code
 * pmw3389_config_t config = {
 *     .pin_motion = GPIO_NUM_18,
 *     // ... other pins
 * };
 * pmw3389_test_motion_interrupt(&config, 3200);
 * @endcode
 */
void pmw3389_test_motion_interrupt(const pmw3389_config_t *config, uint16_t cpi);

// ==================== HIGH-LEVEL API ====================

/**
 * @brief Initialize and configure PMW3389 sensor (all-in-one)
 * 
 * This function performs complete sensor setup including hardware initialization,
 * firmware upload, and CPI configuration in a single call. Ideal for simple
 * applications that just want to get the sensor working quickly.
 * 
 * @param config Pointer to sensor configuration structure
 * @param cpi Desired CPI value (50-16000)
 * @param out_handle Pointer to store the initialized and configured sensor handle
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if config or out_handle is NULL
 *     - Other ESP_ERR codes for initialization failures
 * 
 * @note This function combines pmw3389_init(), pmw3389_upload(), and pmw3389_set_cpi()
 * @note Recommended for most applications - simplifies setup code
 * 
 * @code
 * pmw3389_config_t config = {
 *     .spi_host = SPI2_HOST,
 *     .pin_cs = GPIO_NUM_45,
 *     // ... other pins
 * };
 * pmw3389_handle_t sensor;
 * esp_err_t ret = pmw3389_init_and_configure(&config, 3200, &sensor);
 * @endcode
 */
esp_err_t pmw3389_init_and_configure(const pmw3389_config_t *config, uint16_t cpi, pmw3389_handle_t *out_handle);


/**
 * @brief Start motion tracking with hardware interrupt 
 * 
 * This function uses hardware interrupts for efficient motion detection.
 * The CPU sleeps until motion is detected, making it much more power-efficient
 * than polling mode. Displays motion data and statistics in real-time.
 * 
 * @param handle Initialized and configured sensor handle
 * @param cpi CPI value (used for distance calculations in statistics)
 * 
 * @return This function never returns under normal conditions
 * 
 * @note Requires pin_motion to be configured during pmw3389_init()
 * @note CPU enters low-power sleep between motion events
 * @note Displays comprehensive statistics including physical distances
 * @note Use after pmw3389_init_and_configure() for complete setup
 * 
 * @code
 * pmw3389_config_t config = {
 *     .pin_motion = GPIO_NUM_18,  // Must be valid GPIO
 *     // ... other pins
 * };
 * pmw3389_handle_t sensor;
 * pmw3389_init_and_configure(&config, 3200, &sensor);
 * pmw3389_start_motion_tracking_interrupt(sensor, 3200);
 * @endcode
 */
void pmw3389_start_motion_tracking_interrupt(pmw3389_handle_t handle, uint16_t cpi);

#ifdef __cplusplus
}
#endif

#endif // PMW3389_H