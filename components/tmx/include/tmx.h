#ifndef TMX_H
#define TMX_H

#include "tmx_common.h"

/**
 * @brief Initialize the TMX touchpad driver.
 */
esp_err_t tmx_init(void);

/**
 * @brief Print processed touch data.
 */
esp_err_t tmx_print(void);


#endif
