#include "battery_monitor.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"

#define BATTERY_ADC_CHAN ADC_CHANNEL_0 // GPIO 1 on ESP32-S3
#define NUM_SAMPLES 32

#define POINTS_SIZE 11
const uint16_t VOLTAGE_POINTS[POINTS_SIZE] = {4200, 4050, 3900, 3850, 3800, 3750, 3700, 3650, 3600, 3500, 3300};
const uint8_t  PERCENTAGE_POINTS[POINTS_SIZE] = {100, 90, 80, 70, 60, 50, 40, 30, 20, 10, 0};

static uint16_t battery_average(void) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // 0 - 3.1V
    };
    adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHAN, &config);

    int sum = 0;
    // Multiple samples and averaging
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int val;
        adc_oneshot_read(adc1_handle, BATTERY_ADC_CHAN, &val);
        sum += val;
        esp_sleep_enable_timer_wakeup(500); // 500 us
    }
    uint16_t battery_level = sum / NUM_SAMPLES;
    
    adc_oneshot_del_unit(adc1_handle);

    return battery_level;
}

uint8_t check_battery(void) {
    uint16_t value = battery_average();
    value = value * 14 / 9;

    if (value >= VOLTAGE_POINTS[0]) return 100;
    if (value <= VOLTAGE_POINTS[POINTS_SIZE - 1]) return 0;

    // Linear interpolation between samplesS
    for (int i = 1; i < POINTS_SIZE; i++) {
        if (value > VOLTAGE_POINTS[i]) {
            uint16_t lower_weight = VOLTAGE_POINTS[i - 1] - value;
            uint16_t upper_weight = value - VOLTAGE_POINTS[i];
            return (PERCENTAGE_POINTS[i - 1] * upper_weight + PERCENTAGE_POINTS[i] * lower_weight) /
                (upper_weight + lower_weight);
        }
    }
    
    return 0;
}
