#ifndef HOGP_DATA_EVENTS_H
#define HOGP_DATA_EVENTS_H
#include "hogp_user_common.h"

/**
 * @brief Enumeration of data event types.
 * These events represent user interactions (mouse movement, clicks, scrolls) 
 * that need to be transmitted to the host.
 */
typedef enum {
    HOGP_DEVT_CURSOR_MOTION,          /**< Relative X/Y cursor movement. */
    HOGP_DEVT_SCROLL_MOTION,          /**< Vertical or Horizontal scroll wheel movement. */
    HOGP_DEVT_MOUSE_BUTTON_PRESSED,   /**< A mouse button was pressed down. */
    HOGP_DEVT_MOUSE_BUTTON_RELEASED,  /**< A mouse button was released. */
    HOGP_DEVT_BATTERY_LEVEL_UPDATE,   /**< The battery level has changed */
} hogp_data_event_type_t;

/**
 * @brief Mouse button bitmasks for HID reports.
 * These values represent the bit positions in the first byte of a standard HID mouse report.
 */
typedef enum {
    HOGP_MOUSE_BLEFT    = 0x01, /**< Left Button (Button 1) */
    HOGP_MOUSE_BRIGHT   = 0x02, /**< Right Button (Button 2) */
    HOGP_MOUSE_BMIDDLE  = 0x04, /**< Middle Button (Button 3) */
    HOGP_MOUSE_B4       = 0x08, /**< Button 4 */
    HOGP_MOUSE_B5       = 0x10, /**< Button 5 */
    HOGP_MOUSE_B6       = 0x20, /**< Button 6 */
    HOGP_MOUSE_B7       = 0x40, /**< Button 7 */
    HOGP_MOUSE_B8       = 0x80  /**< Button 8 */
} hogp_mouse_button_t;

/**
 * @brief Structure representing a data event.
 * Carries the specific payload (coordinates, button ID) corresponding to the event type.
 */
typedef struct {
    hogp_data_event_type_t type; /**< The type of the data event. */

    union {
        /**
         * @brief Payload for cursor and scroll motion events.
         * Used by HOGP_DEVT_CURSOR_MOTION and HOGP_DEVT_SCROLL_MOTION.
         */
        struct {
            uint16_t x; /**< Relative X movement or Horizontal scroll value. */
            uint16_t y; /**< Relative Y movement or Vertical scroll value. */
        };

        /**
         * @brief Payload for button events.
         * Used by HOGP_DEVT_MOUSE_BUTTON_PRESSED and HOGP_DEVT_MOUSE_BUTTON_RELEASED.
         */
        hogp_mouse_button_t button;

        /**
         * @brief Level of the battery (must be between 0 and 100)
         * Used by HOGP_DEVT_BATTERY_LEVEL_UPDATE
         */
        uint8_t battery_level;
    };
} hogp_data_event_t;

#endif /* HOGP_DATA_EVENTS_H */
