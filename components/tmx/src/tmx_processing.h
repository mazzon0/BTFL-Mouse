#ifndef TMX_PROCESSING_H
#define TMX_PROCESSING_H   

#include "tmx_driver.h"
#include "tmx_common.h"

#define MAX_NUM_TOUCHES 2
#define MAX_NUM_FRAMES 20

typedef struct {
    int ID;
    uint16_t centroid_x;
    uint16_t centroid_y;
    uint32_t delta_peak;
    uint32_t area;
} tmx_touch_t;

#endif // TMX_PROCESSING_H