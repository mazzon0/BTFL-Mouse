/**
 * @file pmw3389.h
 * @brief Driver for PMW3389DM-T3QU optical motion sensor
 * 
 * This header provides the interface for controlling the PMW3389 optical sensor,
 * commonly used in high-precision gaming mice. It includes functions for 
 * initialization, motion reading, CPI configuration, and register access.
 * 
 * @author Ilaria
 * @date 2025-12-04
 * @version 2.0
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

//register adresses
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
#define PMW3389_REG_CONFIG1             0x0E
#define PMW3389_REG_RESOLUTION_L        0x0E
#define PMW3389_REG_RESOLUTION_H        0x0F
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

#define PMW3389_PRODUCT_ID              0xFF
#define PMW3389_INVERSE_PRODUCT_ID      0xB9

#define PMW3389_MOTION_BIT              (1 << 7)
#define PMW3389_MOTION_LIFT             (1 << 3)
#define PMW3389_MOTION_RES              (1 << 2)
#define PMW3389_MOTION_RVALID           (1 << 0)

#define PMW3389_TSWW    180
#define PMW3389_TSWR    20
#define PMW3389_TSRW    20
#define PMW3389_TSRR    20

/**
 * @brief Structure containing motion data from the sensor
 * 
 * This structure holds the delta movement values, surface quality,
 * and motion/lift detection flags from a single sensor reading.
 */
typedef struct {
    int16_t delta_x;
    int16_t delta_y;
    uint8_t squal;
    bool motion_detected;
    bool lift_detected;
} pmw3389_motion_data_t;


/**
 * @brief Configuration structure for PMW3389 initialization
 * 
 * This structure contains all hardware configuration parameters
 * needed to initialize the sensor, including SPI settings and GPIO pins.
 */
typedef struct {
    spi_host_device_t spi_host;
    int pin_miso;
    int pin_mosi;
    int pin_sclk;
    int pin_cs;
    int pin_motion;
    int pin_reset;
    int spi_clock_speed_hz;
} pmw3389_config_t;

/**
 * @brief Opaque handle type for PMW3389 device
 * 
 * This handle represents an initialized PMW3389 sensor instance
 * and must be passed to all driver functions.
 */

typedef struct pmw3389_dev_t* pmw3389_handle_t;


/**
 * @brief Initialize the PMW3389 sensor
 * 
 * This function initializes the SPI bus, configures GPIO pins, performs
 * a hardware reset, and verifies communication with the sensor by reading
 * its product ID.
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
 * @details
 * The initialization sequence includes:
 * - SPI bus initialization with specified pins and clock speed
 * - GPIO configuration for motion interrupt pin (if provided)
 * - Hardware stabilization delay
 * - Sensor reset via shutdown and power-up sequence
 * - Product ID verification (expected: 0x47)
 * - Motion register clearing to prepare for data acquisition
 * 
 * @usage
 * @code
 * pmw3389_config_t config = {
 *     .spi_host = SPI2_HOST,
 *     .pin_miso = GPIO_NUM_13,
 *     .pin_mosi = GPIO_NUM_11,
 *     .pin_sclk = GPIO_NUM_12,
 *     .pin_cs = GPIO_NUM_10,
 *     .pin_motion = GPIO_NUM_9,
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
 * Performs a low-level SPI read operation to retrieve the value
 * of a specific register from the PMW3389 sensor.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param addr Register address to read (0x00-0x7F)
 * @param data Pointer to store the read byte value
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or data is NULL
 *     - Other ESP_ERR codes for SPI communication errors
 * 
 * @details
 * The function automatically handles the required timing delay (TSRR)
 * after the read operation as specified in the PMW3389 datasheet.
 * 
 * @usage
 * @code
 * uint8_t product_id;
 * esp_err_t ret = pmw3389_read_reg(sensor, PMW3389_REG_PRODUCT_ID, &product_id);
 * @endcode
 */
esp_err_t pmw3389_read_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t *data);

/**
 * @brief Write a single register to the sensor
 * 
 * Performs a low-level SPI write operation to set the value
 * of a specific register in the PMW3389 sensor.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param addr Register address to write (0x00-0x7F)
 * @param data Byte value to write to the register
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 *     - Other ESP_ERR codes for SPI communication errors
 * 
 * @details
 * The function automatically handles the required timing delay (TSWW)
 * after the write operation as specified in the PMW3389 datasheet.
 * 
 * @usage
 * @code
 * esp_err_t ret = pmw3389_write_reg(sensor, PMW3389_REG_CONFIG1, 0x1F);
 * @endcode
 */
esp_err_t pmw3389_write_reg(pmw3389_handle_t handle, uint8_t addr, uint8_t data);

/**
 * @brief Upload SROM firmware to PMW3389 sensor
 * 
 * This function uploads the complete SROM (Serial ROM) firmware to the PMW3389
 * optical sensor via SPI burst mode. The upload process follows the manufacturer's
 * specified initialization sequence and verifies successful completion by reading
 * the SROM ID register.
 * 
 * @param[in] handle Device handle obtained from pmw3389_init()
 * 
 * @return ESP_OK on successful firmware upload and verification
 * @return ESP_ERR_INVALID_ARG if handle is NULL
 * @return ESP_FAIL if SPI communication fails during any step
 * 
 * @details
 * The firmware upload sequence consists of the following steps:
 * 1. Initialize SROM mode by writing 0x1D to SROM_ENABLE register
 * 2. Start download mode by writing 0x18 to SROM_ENABLE register
 * 3. Upload firmware data via burst write mode (3070 bytes)
 *    - Each byte requires a 15μs delay between transmissions
 *    - Progress is logged every 512 bytes
 * 4. Wait 10ms for sensor to process the firmware
 * 5. Verify upload by reading SROM_ID register
 *    - Expected values: 0x04, 0x05, or 0x06
 * 
 * @note This function should be called during sensor initialization before
 *       configuring operational parameters
 * @note The entire upload process takes approximately 50-60ms to complete
 * @warning Do not interrupt the upload process once started
 * 
 * @code
 * pmw3389_handle_t sensor;
 * esp_err_t ret = pmw3389_init(&sensor, &config);
 * if (ret == ESP_OK) {
 *     ret = pmw3389_upload_srom(sensor);
 *     if (ret == ESP_OK) {
 *         ESP_LOGI(TAG, "Sensor ready for operation");
 *     }
 * }
 * @endcode
 */
static esp_err_t pmw3389_upload_srom(pmw3389_handle_t handle);

/**
 * @brief Configure the sensor for operation in native mode
 * 
 * This function applies optimal configuration settings for the sensor,
 * including rest mode, lift detection, and surface quality thresholds.
 * It prepares the sensor for motion tracking without external SROM firmware.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL
 *     - Other ESP_ERR codes for register access errors
 * 
 * @details
 * Configuration steps include:
 * - Setting rest mode configuration
 * - Disabling angle snap for raw motion data
 * - Configuring lift detection threshold
 * - Setting surface quality minimum threshold
 * - Configuring power management downshift
 * - Clearing motion registers
 * 
 * @usage
 * @code
 * esp_err_t ret = pmw3389_upload(sensor);
 * if (ret == ESP_OK) {
 *     // Sensor is ready for motion tracking
 * }
 * @endcode
 */
esp_err_t pmw3389_upload(pmw3389_handle_t handle);

/**
 * @brief Read motion data from the sensor
 * 
 * Retrieves the current motion information including X/Y displacement,
 * surface quality, and motion/lift detection status.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param motion_data Pointer to structure to store the motion data
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle or motion_data is NULL
 *     - Other ESP_ERR codes for register read errors
 * 
 * @details
 * The function reads multiple registers in sequence:
 * - Motion register (to check motion flag)
 * - Delta X low and high bytes (16-bit signed displacement)
 * - Delta Y low and high bytes (16-bit signed displacement)
 * - Surface quality register (tracking quality metric)
 * 
 * Delta values are 16-bit signed integers representing movement in counts.
 * The actual physical displacement depends on the configured CPI setting.
 * 
 * @usage
 * @code
 * pmw3389_motion_data_t motion;
 * esp_err_t ret = pmw3389_read_motion(sensor, &motion);
 * if (ret == ESP_OK && motion.motion_detected) {
 *     printf("Movement: X=%d, Y=%d\n", motion.delta_x, motion.delta_y);
 * }
 * @endcode
 */
esp_err_t pmw3389_read_motion(pmw3389_handle_t handle, pmw3389_motion_data_t *motion_data);

/**
 * @brief Set the CPI (Counts Per Inch) resolution
 * 
 * Configures the sensor's tracking resolution. Higher CPI values result
 * in more sensitive motion tracking.
 * 
 * @param handle Device handle obtained from pmw3389_init()
 * @param cpi Desired CPI value (range: 50-16000, must be multiple of 50)
 * 
 * @return 
 *     - ESP_OK on success
 *     - ESP_ERR_INVALID_ARG if handle is NULL or CPI is out of range
 *     - Other ESP_ERR codes for register write errors
 * 
 * @details
 * The PMW3389 supports CPI values from 50 to 16000 in increments of 50.
 * If a non-multiple of 50 is provided, the function will round down to
 * the nearest valid value.
 * 
 * Common CPI values:
 * - 400-800: Low sensitivity, high precision
 * - 1600: Default, balanced setting
 * - 3200-6400: High sensitivity for gaming
 * - 12000+: Very high sensitivity
 * 
 * @usage
 * @code
 * esp_err_t ret = pmw3389_set_cpi(sensor, 1600);
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
 * @details
 * The function performs:
 * - Sensor shutdown command
 * - SPI device removal from bus
 * - Memory deallocation
 * 
 * @usage
 * @code
 * pmw3389_deinit(sensor);
 * sensor = NULL; // Handle is no longer valid
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
 * @details
 * This function is primarily used for debugging and verification.
 * It logs register values to the console via ESP_LOGI.
 * 
 * @usage
 * @code
 * pmw3389_dump_registers(sensor);
 * @endcode
 */
esp_err_t pmw3389_dump_registers(pmw3389_handle_t handle);


/**
 * @brief Run a complete motion test with statistics
 * 
 * This is a comprehensive test function that initializes the sensor,
 * configures it, and enters a continuous loop reading and displaying
 * motion data with periodic statistics.
 * 
 * @param config Pointer to configuration structure with hardware settings
 * @param cpi Desired CPI (Counts Per Inch) resolution (50-16000, multiple of 50)
 * 
 * @return 
 *     - ESP_OK on success (never returns in normal operation)
 *     - ESP_ERR_INVALID_ARG if config is NULL or CPI is out of range
 *     - Other ESP_ERR codes for initialization or configuration errors
 * 
 * @details
 * The function performs:
 * 1. Sensor initialization with provided configuration
 * 2. Sensor configuration upload (native mode)
 * 3. CPI setting
 * 4. Infinite loop reading motion data every 100ms
 * 5. Display motion events (ΔX, ΔY, SQUAL, motion/lift flags)
 * 6. Display statistics every 50 reads (total reads, motion %, cumulative displacement)
 * 
 * This function is designed for testing and demonstration purposes.
 * It runs indefinitely until the program is terminated.
 * 
 * @usage
 * @code
 * pmw3389_config_t config = {
 *       .spi_host = SPI2_HOST,
 *       .pin_miso = PIN_MISO,
 *       .pin_mosi = PIN_MOSI,
 *       .pin_sclk = PIN_SCLK,
 *       .pin_cs = PIN_CS,
 *       .pin_motion = PIN_MOTION,
 *       .spi_clock_speed_hz = SPI_CLOCK_SPEED_HZ,
 *   };
 * pmw3389_test_motion(&config, 3200);
 * @endcode
 */
esp_err_t pmw3389_test_motion(const pmw3389_config_t *config, uint16_t cpi);



/**
 * @brief Motion interrupt test for PMW3389 sensor
 * 
 * This function performs a continuous test of the PMW3389 sensor using 
 * hardware interrupts to detect motion. It monitors and logs detailed 
 * statistics about detected movement, including delta X/Y, traveled 
 * distances, and signal quality.
 * 
 * @param[in] config Pointer to the PMW3389 sensor configuration structure.
 *                   Must not be NULL.
 * @param[in] cpi    CPI (Counts Per Inch) value to set on the sensor.
 *                   Determines the motion detection sensitivity.
 * 
 * @return void      The function does not return (infinite loop) or terminates 
 *                   in case of initialization error.
 * 
 * @details
 * The function performs the following operations:
 * - Initializes the PMW3389 sensor with the provided configuration
 * - Uploads the sensor configuration
 * - Sets the specified CPI value
 * - Enters an infinite loop that:
 *   - Waits for interrupt notifications from the sensor (5 second timeout)
 *   - Reads motion data when an interrupt is detected
 *   - Calculates and accumulates statistics (total movements, distances, false wake-ups)
 *   - Prints detailed statistics after 5 seconds of inactivity
 * 
 * Monitored statistics:
 * - Total number of detected movements
 * - Total number of received interrupts
 * - False wake-up count (interrupts without actual motion)
 * - Cumulative delta X and Y
 * - Traveled distances in mm (X axis, Y axis, and total)
 * - Average movements per minute
 * - Signal quality (SQUAL) and lift detection
 * 
 * @note
 * - The function uses g_motion_task_handle for communication with the ISR
 * - Requires the sensor interrupt to be properly configured
 * - The function never terminates under normal conditions (infinite loop)
 * - In case of initialization error, the function returns immediately
 * 
 * @usage
 * @code
 * pmw3389_config_t config = {
 *     .spi_host = SPI2_HOST,
 *     .cs_io = GPIO_NUM_5,
 *     .motion_io = GPIO_NUM_4
 * };
 * 
 * pmw3389_test_motion_interrupt(&config, 1600); // Test with 1600 CPI
 * @endcode
 */
void pmw3389_test_motion_interrupt(const pmw3389_config_t *config, uint16_t cpi);




#ifdef __cplusplus
}
#endif

#endif // PMW3389_H