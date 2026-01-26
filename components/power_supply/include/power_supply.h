/**
 * @file power_supply.h
 * @brief Entering and exiting sleep.
 * @author Elena
 * @date 2025-12-15
 * @version 1.0
 */

#ifndef __POWERONOFF_H__
#define __POWERONOFF_H__

//#include "sdkconfig.h"
#include <stdio.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "driver/rtc_io.h"
#include "esp_timer.h"

#ifdef __cplucpluc
extern "C"
#endif


//Pin configuration
/**
 * MOTION: GPIO18 (RTC_GPIO for wake-up)
*/
#define PIN_MOTION GPIO_NUM_18 /*RTC_GPIO for wake-up*/
#define WAKEUP_LEVEL 1 /*HIGH LEVEL on motion detected*/

//PMW3389 register addresses
#define PMW3389_REG_MOTION 0X02
#define PMW3389_REG_DELTA_X_L       0x03
#define PMW3389_REG_DELTA_X_H       0x04
#define PMW3389_REG_DELTA_Y_L       0x05
#define PMW3389_REG_DELTA_Y_H       0x06
#define PMW3389_REG_POWER_UP_RESET 0X3A
#define PMW3389_REG_SHUTDOWN 0X3B

#define PMW3389_MOTION_BIT              (1 << 7)
#define PMW3389_MOTION_LIFT             (1 << 3)
#define PMW3389_MOTION_RES              (1 << 2)
#define PMW3389_MOTION_RVALID           (1 << 0)

//Configuration
#define SPI_FREQUENCY 2000000	// 2 MHz
#define INACTIVITY_TIMEOUT_MS_LS 10000	// 10 s
#define INACTIVITY_TIMEOUT_MS_DS 300000 // 5 min
#define MOTION_THRESHOLD 5	// Minimum movement threshold
#define POLLING_RATE_MS 10	// 100Hz polling

//Global variables
//static const char *TAG="PMW3389";
static spi_device_handle_t spi_handle;
static volatile bool motion_detected = false;
static int64_t last_motion_time = 0;
static bool sensor_active = true;

/**
 * @brief Interrupt handler on motion detected
 *
 * @param arg Pointer not used in this function, but in the signature
 * 
 * @return void

 * @details On motion sensor event detected, the main program is paused and
 * the CPU jumts to motion_isr_handler in the RAM and sets motion_detected global variable
 * to true. The CPU then returns where it left off in the main code.
 *
 * @code
 * static void IRAM_ATTR motion_isr_handler(void* arg) {
    motion_detected = true;
   }
 * @endcode
 */
//static void IRAM_ATTR motion_isr_handler(void* arg);

/**
 * @brief Delay for specified microseconds ILARIA
 *
 * @param us Microseconds to delay
 * 
 * @code
 * static inline void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
   }
 * @endcode
 */
static inline void delay_us(uint32_t us);


/**
 * @brief Spi functions 
 */
 /**
  * @brief write/read DA ILARIA
  * 
  * Write byte to PMW3389 register
  * 
  * @code
    static void pmw3389_write(uint8_t reg, uint8_t data) {
        spi_transaction_t trans = {
            .flags = SPI_TRANS_USE_TXDATA,
            .length = 16,  // 2 bytes = 16 bits
            .tx_data = {reg | 0x80, data},  // Bit 7 = 1 for write
        };
   
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &trans));
    ets_delay_us(35);  // tSWW: time between writes
    }
*/
static void pmw3389_write(uint8_t reg, uint8_t data);

/**
 * Read byte from PMW3389 register
 * static uint8_t pmw3389_read(uint8_t reg) {
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};  // Bit 7 = 0 for read
    uint8_t rx_data[2] = {0};
   
    spi_transaction_t trans = {
        .length = 16,  // 2 bytes = 16 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
     ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &trans));
    ets_delay_us(160);  // tSRAD: time after address
   
    return rx_data[1];  // Second byte is the data
}
  * @endcode 
*/
static uint8_t pmw3389_read(uint8_t reg);

/**
 * @brief Initialize PMW3389 sensor
*/
/**
 * @brief Shutdown sensor (10 uA consuption) 
    
 * @code
 * static void pmw3389_shutdown(void) {
    pmw3389_write(PMW3389_REG_SHUTDOWN, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));
    sensor_active = false;
    }
 * @endcode
*/
static void pmw3389_shutdown(void);

/**
 * @brief Clear motion flag by reading all motion registers ILARIA
 * 
 * @code
 * static void pmw3389_clear_motion(void) {
    pmw3389_read(PMW3389_REG_MOTION);
    pmw3389_read(PMW3389_REG_DELTA_X_L);
    pmw3389_read(PMW3389_REG_DELTA_X_H);
    pmw3389_read(PMW3389_REG_DELTA_Y_L);
    pmw3389_read(PMW3389_REG_DELTA_Y_H);
}
 * @endcode
*/
static void pmw3389_clear_motion();

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
static void pmw3389_read_motion(void);

/**
 * Sleep functions
 */
/**
 * @brief Enter Light Sleep mode
 * 
 * @param void
 * 
 * @return void
 * 
 * @details This function prepares the ESP32 and the PMW3389 sensor for Light Sleep.
 * The optical sensor enters its lowest power mode, ao its LED is turned off
 * and its internal processing stops, and any pending motion data or interrupt flags
 * inside the sensor are cleared. The GPIO18 MOTION pin is configured as wake-up
 * source through EXT0 controller, which is part of the RTC domain. The system is then
 * given a 100ms window to ensure all serial communicayions are fully completed and the
 * voltages have stabilized before the CPU feezes. The ESP32 is put to sleep: the CPU is
 * paused and the peripheral clocks are suspended. When the ESP32 wakes up from light sleep,
 * CPU'S execution resumes at the line after esp_light_sleep_start() and the sensor is
 * told to turn its laser back on and start tracking again.
 * 
 * 
 * @code
 * static void enter_light_sleep(void) {
    // Put sensor in shutdown
    pmw3389_shutdown();
    pmw3389_clear_motion();
   
    // Configure wake-up on MOTION pin (GPIO18)
    esp_sleep_enable_ext0_wakeup(PIN_MOTION, 1);  // Wake on HIGH
   
    vTaskDelay(pdMS_TO_TICKS(100));
   
    // Enter Light Sleep
    esp_light_sleep_start();

    // Execution continues from here after motion detected
   
    // Wake up sensor
    pmw3389_wakeup(); //Since the sensor was put into shutdown() before sleep, it is still "dead." 
    //You must call pmw3389_wakeup() here to tell the sensor to turn its laser back on and start 
    //tracking again.
}
 * @endcode
 */
static void enter_light_sleep(void);

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
static void enter_deep_sleep(void);

/**
 * @brief Wakeup sensor
 * 
 * @param void
 * 
 * @return void
 * 
 * @details The function first writes 0x5A to PWM3389 register, triggering
 * a Power-Up Reset. The vTaskDelay function sets a 50ms window during which,
 * while the sensor is rebooting, the CPU can handle other tasks instead of
 * buy waiting.
 * 
 * @code
 * static void pmw3389_wakeup(void) {
    pmw3389_write(PMW3389_REG_POWER_UP_RESET, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(50));
    sensor_active = true;
    last_motion_time = esp_timer_get_time() / 1000;  // Convert to ms
}
    @endcode
 */
static void pmw3389_wakeup(void);

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
   
    if (inactive_time > INACTIVITY_TIMEOUT_MS_LS) {
        enter_light_sleep();
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        enter_deep_sleep(); 
    }
}
 * @endcode
 */
static void check_inactivity(void);

/**
 * SPI initialization
 */
/**
 * @brief Initialize SPI bus
 */

/**
 * GPIO initialization
 */
/**
 * @brief Initialize MOTION pin with interrupt DA ILARIA 
 * 
 * @code
 * static void motion_pin_init(void) {
    // Configure MOTION pin as input
    gpio_config_t motion_cfg = {
        .pin_bit_mask = (1ULL << PIN_MOTION),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_POSEDGE,  // Interrupt on rising edge
    };
    gpio_config(&motion_cfg);
   
    // Install GPIO ISR service
    gpio_install_isr_service(0);
   
    // Add ISR handler for MOTION pin
    gpio_isr_handler_add(PIN_MOTION, motion_isr_handler, NULL);
   }
 * 
 * @endcode
 */
//static void motion_pin_init(void);

/**
 * Main task
 */
/**
 * @brief Main sensor polling task
 * 
 * @param arg Pointer not used in this function, but in the signature
 * 
 * @return void
 * 
 * @details The function updates the last_motion_time and then enters a loop
 * which defines the task that the sensor must perform repeatedly. If a
 * motion is detected and the sensor isn't active, a wakeup will be triggered.
 * If the sensor is active, read motion and then check inactivity time.
 * 
 * @code
 * static void sensor_task(void *arg) {
    last_motion_time = esp_timer_get_time() / 1000;
   
    while (1) {
        // Handle motion interrupt
        if (motion_detected) {
            motion_detected = false;
            if (!sensor_active) {
                pmw3389_wakeup();
            }
        }
       
        // Read motion if sensor is active
        if (sensor_active) {
            pmw3389_read_motion();
            check_inactivity();
        }
       
        vTaskDelay(pdMS_TO_TICKS(POLLING_RATE_MS));  // ~100Hz polling
    }
   }
 * @endcode
 */
static void sensor_task(void* arg);

/* #if CONFIG_EXAMPLE_EXT0_WAKEUP	//se questa opzione è stata abilitata durante la configurazione del progetto, il compilatore processa la riga seguente
void deep_light_sleep_register_ext0_wakeup(void);
#endif */

#endif
