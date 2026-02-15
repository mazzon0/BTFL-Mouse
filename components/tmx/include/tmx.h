#ifndef TMX_H
#define TMX_H

#include "tmx_common.h"


/**
 * @brief Initialize the TMX touchpad driver.
 */
esp_err_t tmx_init(void);

/**
 * @brief Print processed touch data, usefull for debugging.
 */
esp_err_t tmx_print(void);

/**
 * @brief Process the TMX touchpad data pipeline.
 */
tmx_gesture_t tmx_pipeline_process(void);

/**
 * @brief Callback function type for touch events
 * @param event The touch event data structure containing details about the touch event.
 */

 void tmx_task(void *param);

#endif
