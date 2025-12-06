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
    bool is_active;
    int ID;
    float centroid_x;
    float centroid_y;
    uint32_t delta_peak;
    uint32_t area;
} tmx_touch_t;

/**
 * @brief Initialize the TMX processing module.
 */
esp_err_t tmx_processing_init(void);

/**
 * @brief DEBUG Send serial output of processed touch data.
 */
void tmx_processing_print(void);

/**
 * @brief Apply EMWA filtering to raw touch data. Differentiate between fast and slow variations.
 */
void tmx_processing_filtering(void);

/**
 * @brief Oversample the delta signal using bilinear interpolation.
 */
void tmx_processing_oversampling(void);

/**
 * @brief Detect touch blobs in the oversampled delta signal.
 */
void tmx_processing_blob_detection(void);

/**
 * @brief Recursive flood fill algorithm to identify connected touch blobs.
 */
void tmx_processing_flood_fill(int r, int c, tmx_touch_t* touch);

#endif // TMX_PROCESSING_H