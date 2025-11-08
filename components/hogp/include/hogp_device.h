#ifndef HOGP_DEVICE_H
#define HOGP_DEVICE_H
/**
 * @brief This file can be used to manage and initialize data for HID devices.
 * There are 3 structures, one for each HID device type (mouse, keyboard, custom), called hogp_x_t.
 * There are 3 functions to initialize those structures, called hogp_x_init().
 */

/**
 * @brief Data for mouse service.
 */
typedef struct {
    int n;
} hogp_mouse_t;

/**
 * @brief Data for keyboard service.
 */
typedef struct {
    int n;
} hogp_keyboard_t;

/**
 * @brief Data for custom HID service.
 */
typedef struct {
    int n;
} hogp_custom_t;

// TODO add configurations in following functions

/**
 * @brief Init mouse service data given a configuration.
 */
int hogp_mouse_init(hogp_mouse_t *mouse);

/**
 * @brief Init keyboard service data given a configuration.
 */
int hogp_keyboard_init(hogp_keyboard_t *keyboard);

/**
 * @brief Init custom HID service data given a configuration.
 */
int hogp_custom_init(hogp_custom_t *custom);

#endif
