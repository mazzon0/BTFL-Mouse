#ifndef TMX_PROCESSING_H
#define TMX_PROCESSING_H   

#include "tmx_driver.h"
#include "tmx_common.h"
#include "esp_timer.h"



#define MAX_NUM_TOUCHES 2
#define MAX_NUM_FRAMES 20
#define OVERSAMPLING_FACTOR 3
#define TMX_DELTA_THRESHOLD 20000
#define REJECTION_AREA_THRESHOLD 1100000
#define MAX_DISTANCE_SQUARED 40000.0f // 200 pixels squared
#define CLICK_MAX_MOVE_SQUARED 1.0f
#define CLICK_MAX_DURATION_MS 100
#define OVERSAMPLED_M (TMX_M * OVERSAMPLING_FACTOR-(OVERSAMPLING_FACTOR-1))
#define OVERSAMPLED_N (TMX_N * OVERSAMPLING_FACTOR-(OVERSAMPLING_FACTOR-1))

typedef enum{
    TRACK_IDLE,
    STATIC_HOLD,
    MOTION_ACTIVE,
    TRACK_RELEASED
} tracker_state_t;

typedef enum{
    IDLE,
    ONE_FINGER,
    TWO_FINGER
} gesture_state_t;

/**
 * @brief Data structure for a detected blob.
 */
typedef struct {
    bool is_active;
    float centroid_x;
    float centroid_y;
    uint32_t delta_peak;
    uint32_t area;
} tmx_touch_t;

/**
 * @brief Data structure for touch tracking.
 */
typedef struct {
    int ID;
    tracker_state_t state;
    uint64_t down_timestamp;
    float start_x, start_y;
    float current_x, current_y;
    float prev_x, prev_y;
    float dx, dy;
    tmx_touch_t last_blob;
    bool is_pressed_reported;
} tmx_tracker_t;

/**
 * @brief Initialize the TMX processing module.
 */
esp_err_t tmx_processing_init(void);

/**
 * @brief DEBUG Send serial output of processed touch data.
 */
void tmx_processing_print(void);

/**
 * @brief Read raw touch data from the TMX driver.
 */
void tmx_processing_raw_read(void);

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

/**
 * @brief Apply finger rejection filtering to the current frame.
 */
void finger_rejection_filtering(void);

/**
 * @brief Associate detected blobs with existing trackersuusing nearest neighbor criteria.
 */
void tmx_processing_associate_blobs(uint64_t current_time_ms);

/**
 * @brief Update the state machine for each touch tracker.
 */
void tmx_processing_tracker_FSM();

/**
 * @brief Detect gestures based on the current state of touch trackers.
 */
tmx_gesture_t tmx_processing_detect_gestures(void);

/**
 * @brief Get the current time in milliseconds.
 */
uint64_t get_current_time_ms(void);




#endif // TMX_PROCESSING_H