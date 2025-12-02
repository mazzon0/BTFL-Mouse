#include <stdio.h>
#include "tmx.h"

void app_main(void)
{
    tmx_init();

    uint32_t values[12];

    while (1) {
        tmx_read_raw(values, 12);

        for (int i = 0; i < 12; i++) {
            printf("%" PRIu32 ",", values[i]);
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}