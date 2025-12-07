#include <stdio.h>
#include "tmx.h"

void app_main(void)
{
    tmx_init();
    while (1)
    {
        tmx_print();
        vTaskDelay(50 / portTICK_PERIOD_MS); // Add a delay to avoid flooding the output
    }
    
    
}