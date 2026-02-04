#ifndef HOGP_H
#define HOGP_H

#include "hogp_user_common.h"
#include "hogp_data_events.h"

/**
 * @brief Initialize the HOGP library and start the Bluetooth stack.
 * Initializes the NimBLE host stack.
 * Sets up the internal HOGP context and queues.
 * Configures GAP and GATT services (HID Service, Device Name).
 * Starts the NimBLE host task and the HOGP FSM task.
 * @param init_info Configuration struct containing device name, appearance, and task period.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_INTERNAL_FAIL if NVS, NimBLE, or GAP/GATT initialization fails.
 * @return HOGP_ERR_NO_MEM if memory allocation for queues fails.
 */
hogp_result_t hogp_setup(const hogp_init_info_t *const init_info);

/**
 * @brief Shut down the HOGP library.
 * Stops internal tasks and frees resources (queues, context).
 * @note This function currently cleans up the context but might not fully stop the FreeRTOS tasks if they are blocked.
 * @return HOGP_OK on success.
 */
hogp_result_t hogp_shutdown(void);

/**
 * @brief Send a user data event (e.g., mouse movement, click) to the host.
 * This function is thread-safe and can be called from ISRs or other tasks.
 * It pushes the event into the internal `data_queue` for processing by the HOGP FSM task.
 * @param event Pointer to the data event structure containing the input type and value.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_QUEUE_FULL if the data queue is full (event dropped).
 */
hogp_result_t hogp_send(const hogp_data_event_t *event);

#endif /* HOGP_H */
