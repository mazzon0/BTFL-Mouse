#include "battery_monitor.h"
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "esp_adc/adc_oneshot.h"

#define BATTERY_ADC_CHAN ADC_CHANNEL_0 // GPIO 1 on ESP32-S3
#define NUM_SAMPLES 8

static uint16_t battery_average(void) {
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = { .unit_id = ADC_UNIT_1 };
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12, // 0 - 3.1V
    };
    adc_oneshot_config_channel(adc1_handle, BATTERY_ADC_CHAN, &config);

    uint16_t sum = 0;
    // Multiple samples and averaging
    for (int i = 0; i < NUM_SAMPLES; i++) {
        int val;
        adc_oneshot_read(adc1_handle, BATTERY_ADC_CHAN, &val);
        sum += val;
        esp_sleep_enable_timer_wakeup(100); // 100 us
    }
    uint16_t battery_level = sum / NUM_SAMPLES;
    
    adc_oneshot_del_unit(adc1_handle);

    return battery_level;
}

uint16_t check_battery(void) {
    uint16_t battery_percentage = battery_average() * 100 / 4200;
    if (battery_percentage > 100) battery_percentage = 100;
    if (battery_percentage < 0) battery_percentage = 0;
    return battery_percentage;
}
