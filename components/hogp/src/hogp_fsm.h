#ifndef HOGP_FSM_H
#define HOGP_FSM_H
#include "hogp_common.h"

/**
 * @brief The main Finite State Machine (FSM) task loop for the HOGP application.
 * * * Handles the high-level logic of the application (Start -> Advertising -> Connected).
 * * Processes control events (connection status, protocol changes) from the `control_queue`.
 * * Processes data events (user input) from the `data_queue` and sends notifications to the host.
 * * Manages sleep/suspend states.
 * * @param params Task parameters (unused, standard FreeRTOS signature).
 */
void hogp_task(void *params);

#endif /* HOGP_FSM_H */
