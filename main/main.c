#include <stdio.h>
#include "tmx.h"

void app_main(void)
{
    tmx_init();

    static uint32_t values[TMX_M][TMX_N];

    while (1) {
        tmx_read_raw(&values[0][0], TMX_NUM_TOUCH_PADS);

        for (int i = 0; i < TMX_M; i++) {
            for (int j = 0; j < TMX_N; j++) {
                printf("%" PRIu32 ",", values[i][j]);
            }
        }
        printf("\n");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}