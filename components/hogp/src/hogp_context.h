#ifndef HOGP_CONTEXT_H
#define HOGP_CONTEXT_H
#include "hogp_common.h"

/**
 * @brief  Initialize the HOGP global context.
 * * Sets up the initial state, queues, and device configuration parameters.
 * * It allocates memory for the control and data queues based on the configured sizes.
 * * @param  init_info  Pointer to the initialization structure containing device name, appearance, and task period.
 * * @return HOGP_OK on success.
 * @return HOGP_ERR_INVALID_ARG if `init_info` is NULL.
 * @return HOGP_ERR_NO_MEM if queue creation fails.
 */
hogp_result_t hogp_context_init(const hogp_init_info_t *init_info);

/**
 * @brief  Shut down the HOGP global context.
 * * Frees the memory allocated for the control and data queues.
 * * Resets the global context structure to zero.
 * * @return HOGP_OK on success.
 */
hogp_result_t hogp_context_shutdown(void);

#endif /* HOGP_CONTEXT_H */
