#ifndef TMX_DRIVER_H
#define TMX_DRIVER_H

#include "tmx_common.h"

/**
 * @brief Initialize the touchpad driver.
 * @return esp_err_t to check if the driver initialization success.
 */
esp_err_t tmx_driver_init(void);


/**
 * @brief Read raw touchpad values and write into the provided destination array.
 * @param dest Pointer to an array where the raw touch data will be stored. The array should be large enough to hold TMX_M * TMX_N values.
 * This function reads the raw sensor data from the touchpad and populates the provided array with the current touch values for each electrode. The data can then be used for further processing to detect touch events and gestures.
 */
void tmx_driver_read_raw(uint32_t* dest);


#endif
