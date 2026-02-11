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
 */
void tmx_driver_read_raw(uint32_t* dest);


#endif
