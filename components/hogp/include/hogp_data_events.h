#ifndef HOGP_DATA_EVENTS_H
#define HOGP_DATA_EVENTS_H
#include "hogp_user_common.h"

/**
 * @brief Enumeration of data event types.
 * * These events represent user interactions (mouse movement, clicks, scrolls) 
 * that need to be transmitted to the host.
 */
typedef enum {
    HOGP_DEVT_CURSOR_MOTION,          /**< Relative X/Y cursor movement. */
    HOGP_DEVT_SCROLL_MOTION,          /**< Vertical or Horizontal scroll wheel movement. */
    HOGP_DEVT_MOUSE_BUTTON_PRESSED,   /**< A mouse button was pressed down. */
    HOGP_DEVT_MOUSE_BUTTON_RELEASED,  /**< A mouse button was released. */
} hogp_data_event_type_t;

/**
 * @brief Structure representing a data event.
 * * Carries the specific payload (coordinates, button ID) corresponding to the event type.
 */
typedef struct {
    hogp_data_event_type_t type; /**< The type of the data event. */

    union {
        /**
         * @brief Payload for cursor and scroll motion events.
         * * Used by HOGP_DEVT_CURSOR_MOTION and HOGP_DEVT_SCROLL_MOTION.
         */
        struct {
            uint16_t x; /**< Relative X movement or Horizontal scroll value. */
            uint16_t y; /**< Relative Y movement or Vertical scroll value. */
        };

        /**
         * @brief Payload for button events.
         * * Used by HOGP_DEVT_MOUSE_BUTTON_PRESSED and HOGP_DEVT_MOUSE_BUTTON_RELEASED.
         * * 1 = Left, 2 = Right, 3 = Middle, etc.
         */
        uint16_t button;
    };
} hogp_data_event_t;

#endif /* HOGP_DATA_EVENTS_H */
