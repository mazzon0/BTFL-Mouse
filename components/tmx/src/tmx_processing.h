#ifndef TMX_PROCESSING_H
#define TMX_PROCESSING_H   

#include "tmx_driver.h"
#include "tmx_common.h"
#include "esp_timer.h"



#define MAX_NUM_TOUCHES 2 /** Max number of finger to be tracked */
#define OVERSAMPLING_FACTOR 3 /** Factor of oversambling of the delta matrix */
#define TMX_DELTA_THRESHOLD 20000 /** Threshold to an electrodes to defined as acrive */
#define REJECTION_AREA_THRESHOLD 1100000 /** Area for a blob to be defined as finger rejection */
#define MAX_DISTANCE_SQUARED 40000.0f /** 200 pixels squared */
#define CLICK_MAX_MOVE_SQUARED 1.0f   /** Distance for a blob to be mapped to a tracker */
#define OVERSAMPLED_M (TMX_M * OVERSAMPLING_FACTOR-(OVERSAMPLING_FACTOR-1)) /** Dimension for oversampled matrix */
#define OVERSAMPLED_N (TMX_N * OVERSAMPLING_FACTOR-(OVERSAMPLING_FACTOR-1)) /** Dimension for oversampled matrix */
#define ALPHA_FAST 0.5f /** Smoothing factor for EMWA filtering, filtered data */
#define ALPHA_SLOW 0.005f /** Smoothing factor for EMWA filtering, adaptive baseline */


/**
 * @brief Tracker finite state machine.
 *
 * Represents the lifecycle of a single touch tracker
 * across multiple frames.
 *
 * State transitions:
 *
 *  TRACK_IDLE
 *      -> STATIC_HOLD        (new touch detected)
 *
 *  STATIC_HOLD
 *      -> MOTION_ACTIVE      (movement exceeds threshold)
 *      -> TRACK_RELEASED     (touch removed)
 *
 *  MOTION_ACTIVE
 *      -> TRACK_RELEASED     (touch removed)
 *
 *  TRACK_RELEASED
 *      -> TRACK_IDLE         (cleanup completed)
 */
typedef enum{
    /**
     * @brief No active touch associated to this tracker.
     *
     * The tracker slot is free and can be assigned
     * to a new detected blob.
     */
    TRACK_IDLE,

    /**
     * @brief Touch detected but not significantly moving.
     *
     * The finger is considered stable (within movement threshold).
     * This state is typically used for click detection.
     */
    STATIC_HOLD,

    /**
     * @brief Touch is actively moving.
     *
     * Movement exceeded the click threshold.
     * Used for gestures such as swipe or scroll.
     */
    MOTION_ACTIVE,

    /**
     * @brief Touch has been released.
     *
     * This is a transient state used to:
     *  - Emit release events
     *  - Perform cleanup
     * After handling, the tracker returns to TRACK_IDLE.
     */
    TRACK_RELEASED
} tracker_state_t;


/**
 * @brief High-level gesture detection state machine.
 *
 * Represents the global interaction mode based on
 * the number of active touches.
 *
 * State transitions:
 *
 *  IDLE
 *      -> ONE_FINGER     (1 active tracker)
 *      -> TWO_FINGER     (2 active trackers)
 *
 *  ONE_FINGER
 *      -> IDLE           (0 touches)
 *      -> TWO_FINGER     (2 touches)
 *
 *  TWO_FINGER
 *      -> ONE_FINGER     (1 touch remains)
 *      -> IDLE           (0 touches)
 */
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
} tmx_blob_t;

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
    tmx_blob_t last_blob;
    bool is_pressed_reported;
} tmx_tracker_t;

/**
 * @brief Initialize the TMX processing module.
 * @return esp_err_t tocheck if init success.
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
 * Calculate centroids and Area
 */
void tmx_processing_blob_detection(void);

/**
 * @brief Recursive flood fill algorithm to identify connected touch blobs.
 */
void tmx_processing_flood_fill(int r, int c, tmx_blob_t* blob);

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
 * @return the gesture for the current frame, tmx_gesture_t tupe definition in ```tmx.h```.
 */
tmx_gesture_t tmx_processing_detect_gestures(void);

/**
 * @brief Get the current time in milliseconds.
 * @return the current time.
 */
uint64_t get_current_time_ms(void);




#endif // TMX_PROCESSING_H