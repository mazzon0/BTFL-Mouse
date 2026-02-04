#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"

#define WAKEUP_PIN_A GPIO_NUM_4
#define WAKEUP_PIN_B GPIO_NUM_5
#define BATTERY_ADC_CHAN ADC_CHANNEL_0 // GPIO 1 on ESP32-S3

int check_battery(void) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_11, // 0 - 3.1V
    };
    adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHAN, &config);

    int val;
    adc_oneshot_read(adc1_handle, BATTERY_ADC_CHAN, &val);
    
    // Clean up ADC to save power before sleep
    adc_oneshot_del_unit(adc1_handle);
    return val;
}

void start_deep_sleep(void) {
    // Prepare RTC GPIOs
    rtc_gpio_init(WAKEUP_PIN_A);
    rtc_gpio_set_direction(WAKEUP_PIN_A, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en(WAKEUP_PIN_A);
    rtc_gpio_pullup_dis(WAKEUP_PIN_A);

    rtc_gpio_init(WAKEUP_PIN_B);
    rtc_gpio_set_direction(WAKEUP_PIN_B, RTC_GPIO_MODE_INPUT_ONLY);
    rtc_gpio_pulldown_en(WAKEUP_PIN_B);
    rtc_gpio_pullup_dis(WAKEUP_PIN_B);

    // Setup wakeup options
    esp_sleep_enable_timer_wakeup(10 * 1000000); // 10 seconds

    uint64_t mask = (1ULL << WAKEUP_PIN_A) | (1ULL << WAKEUP_PIN_B);
    esp_sleep_enable_ext1_wakeup(mask, ESP_EXT1_WAKEUP_ANY_HIGH);

    // Start deep sleep
    printf("Entering sleep now...\n");
    
    vTaskDelay(pdMS_TO_TICKS(100)); // Ensure serial data is sent before CPU powers down
    esp_deep_sleep_start();
}

void some_fake_work(void) {
    printf("Doing work (5s delay)...\n");
    vTaskDelay(pdMS_TO_TICKS(5000));
}

void app_main(void) {
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();

    switch (wakeup_reason) {
        case ESP_SLEEP_WAKEUP_TIMER:
            printf("Wakeup: Timer. Checking battery...\n");
            int battery_level = check_battery();
            printf("Battery Raw Value: %d\n", battery_level);
            start_deep_sleep();
            break;

        case ESP_SLEEP_WAKEUP_EXT1:
        {
            uint64_t pin_mask = esp_sleep_get_ext1_wakeup_status();
            printf("Wakeup: RTCIO (Mask: %llu)\n", pin_mask);
            
            if (pin_mask & (1ULL << WAKEUP_PIN_A)) printf("Pin A triggered\n");
            if (pin_mask & (1ULL << WAKEUP_PIN_B)) printf("Pin B triggered\n");

            some_fake_work();
            start_deep_sleep();
            break;
        }

        default:
            printf("Normal boot/Reset\n");
            some_fake_work();
            start_deep_sleep();
            break;
    }
}