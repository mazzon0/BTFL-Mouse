#ifndef TMX_PROCESSING_H
#define TMX_PROCESSING_H   

#include "tmx_driver.h"
#include "tmx_common.h"
#include "tmx_utils.h"

#define MAX_NUM_TOUCHES 2
#define MAX_NUM_FRAMES 20
#define OVERSAMPLING_FACTOR 3
#define TMX_DELTA_THRESHOLD 10000
#define OVERSAMPLED_M (TMX_M * OVERSAMPLING_FACTOR-(OVERSAMPLING_FACTOR-1))
#define OVERSAMPLED_N (TMX_N * OVERSAMPLING_FACTOR-(OVERSAMPLING_FACTOR-1))

typedef struct {
    int ID;
    uint16_t centroid_x;
    uint16_t centroid_y;
    uint32_t delta_peak;
    uint32_t area;
} tmx_touch_t;

esp_err_t tmx_processing_init(void);
void tmx_processing_print(void);

#endif // TMX_PROCESSING_H