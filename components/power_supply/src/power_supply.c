#include "power_supply.h"
//non so quanto senso abbiano i vari vTaskDelay qua e la

/**
 * This code is used to configure a Wakeup Source, that allows the
 * ESP32 to wakeup from Deep Sleep or Light Sleep using an external signal.
 * Specifically, the esp_sleep_enable_ext0_wakeup method called uses a
 * specialized RTC IO to monitor a pin even when the main CPU is powered off.
 * 
 * Scopo del blocco di codice è di definire quale pin GPIO fisico deve essere utilizzato per      
 * la funzione di risveglio EXT0.
 * La costante intera exr_wakeup_pin_0 contiene il numero del pin GPIO che sarà utilizzato 
 * per la funzione di risveglio da deep sleep.
 * 
*/
#if CONFIG_EXAMPLE_EXT0_WAKEUP    //controlla se l'opzione è abilitata nelle impostazioni del progetto
    {
        #if CONFIG_IDF_TARGET_ESP32
        const int ext_wakeup_pin_0 = 25;
        #endif
    }
    void deep_light_sleep_register_wakeup(void) {
    //Wake up when the voltage on this pin (25) goes high.
    ESP_ERROR_CHECK(esp_sleep_enable_ext0_wakeup(ext_wakeup_pin_0, 1)); //Enables the hardware trigger to wake the CPU from sleep.
    /* Configure pullup/pulldowns via RTCIO to tie wakeup pins to inactive level during deep sleep. */

    //Standard GPIO configuration. Standard GPIO settings are lost during Deep Sleep. To keep the pin behavior consistent while
        //the chip "sleeps", the code uses the following RTC-specific functions
        //It is necessary because since you are waking un on a high signal, you need to ensure the pin stays low when nothing is touching it.
        //Without the pull-down resistor, the pin would "float", causing random electromagnetic noise to trigger accidental wakeups.
    ESP_ERROR_CHECK(rtc_gpio_pullup_dis(ext_wakeup_pin_0)); //Disables the internal pull-up resistor. Prevents the pin from being pulled high internally.
    ESP_ERROR_CHECK(rtc_gpio_pulldown_en(ext_wakeup_pin_0)); //Enables the internal pull-down resistor. Ensures the pin stays at GND level unless an external button/sensor pushes it to 3.3V.
    }
#endif // CONFIG_EXAMPLE_EXT0_WAKEUP

/*static void IRAM_ATTR motion_isr_handler(void* arg) {
    motion_detected = true;
} */

static inline void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

//la diff con ilaria (sono le read e write reg) è che qua ci sono meno cose.. casomai è da cambiare tutte le volte che vengono chiamate 
static void pmw3389_write(uint8_t reg, uint8_t data) {
    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_TXDATA,
        .length = 16,  // 2 bytes = 16 bits
        .tx_data = {reg | 0x80, data},  // Bit 7 = 1 for write
    };
   
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &trans));
    //ets_delay_us(35);  // tSWW: time between writes
    delay_us(35);
}

static uint8_t pmw3389_read(uint8_t reg) {
    uint8_t tx_data[2] = {reg & 0x7F, 0x00};  // Bit 7 = 0 for read
    uint8_t rx_data[2] = {0};
   
    spi_transaction_t trans = {
        .length = 16,  // 2 bytes = 16 bits
        .tx_buffer = tx_data,
        .rx_buffer = rx_data,
    };
     ESP_ERROR_CHECK(spi_device_polling_transmit(spi_handle, &trans));
    //ets_delay_us(160);  // tSRAD: time after address
    delay_us(160);
   
    return rx_data[1];  // Second byte is the data
}

//Shutdown sensor
static void pmw3389_shutdown(void) {
    pmw3389_write(PMW3389_REG_SHUTDOWN, 0xB6);
    vTaskDelay(pdMS_TO_TICKS(10));
    sensor_active = false;
}

//Wakeup sensor
static void pmw3389_wakeup(void) {
    pmw3389_write(PMW3389_REG_POWER_UP_RESET, 0x5A);
    vTaskDelay(pdMS_TO_TICKS(50));
    sensor_active = true;
    last_motion_time = esp_timer_get_time() / 1000;  // Convert to ms
}

//Clean motion flag by reading all motin registers
static void pmw3389_clear_motion(void) {
    pmw3389_read(PMW3389_REG_MOTION);
    pmw3389_read(PMW3389_REG_DELTA_X_L);
    pmw3389_read(PMW3389_REG_DELTA_X_H);
    pmw3389_read(PMW3389_REG_DELTA_Y_L);
    pmw3389_read(PMW3389_REG_DELTA_Y_H);
}

//Read motion data from sensor
static void pmw3389_read_motion(void) {
    uint8_t motion = pmw3389_read(PMW3389_REG_MOTION);
   
    if (motion & 0x80) {  // Bit 7: motion detected
        motion_detected = true;
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

//Enter Light Sleep mode
static void enter_light_sleep(void) {
    // Put sensor in shutdown
    pmw3389_shutdown();
    pmw3389_clear_motion();
   
    // Configure wake-up on MOTION pin (GPIO18)
    esp_sleep_enable_ext0_wakeup(PIN_MOTION, 1);  // Wake on HIGH
   
    vTaskDelay(pdMS_TO_TICKS(100));
   
    // Enter Light Sleep
    esp_light_sleep_start();
   
    // Wake up sensor
    pmw3389_wakeup();
}

//Enter Deep Sleep mode
static void enter_deep_sleep(void) {   
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


//Check inactivity and trigger sleep if needed
static void check_inactivity(void) {
    if (!sensor_active) return;
   
    int64_t current_time = esp_timer_get_time() / 1000;  // Convert to ms
    int64_t inactive_time = current_time - last_motion_time;
   
    if (inactive_time > INACTIVITY_TIMEOUT_MS_LS) {
        sensor_active = false;
        enter_light_sleep();
    } else if(inactive_time > INACTIVITY_TIMEOUT_MS_DS) {
        enter_deep_sleep(); 
    }
}

//Main sensor polling task
static void sensor_task(void *arg) {
    last_motion_time = esp_timer_get_time() / 1000;
   
    while (1) {
        // Handle motion interrupt
        if (motion_detected) {
            motion_detected = false;
            if (!sensor_active) {
                //pmw3389_wakeup();
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
