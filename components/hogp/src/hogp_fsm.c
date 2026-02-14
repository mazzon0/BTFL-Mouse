#include "hogp_fsm.h"
#include "hogp_common.h"
#include "hogp_control_events.h"
#include "hogp_ble.h"
#include "hogp_msg_serializer.h"

// Utility functions
static hogp_result_t state_transition(hogp_context_t *ctx);

// Task

static TaskHandle_t task = NULL;

void hogp_task(void *params) {
    bool running = true;
    hogp_context_t *ctx = hogp_get_context();
    task = xTaskGetCurrentTaskHandle();

    // Timer data
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(ctx->register_period_ms);

    INFO("HOGP task started");

    while (running) {

        hogp_result_t tr = state_transition(ctx);
        if (tr != HOGP_OK && tr != HOGP_ERR_QUEUE_EMPTY) { 
            WARN("state_transition returned %d", tr);
        }

        switch (ctx->state) {

            case HOGP_STATE_START:
                //INFO("STATE START");
                hogp_result_t res = hogp_start_advertising();

                if (res == HOGP_OK) {
                    hogp_control_event_t event;
                    event.type = HOGP_CEVT_ADV_STARTED;
                    if (xQueueSendToBackFromISR(ctx->control_queue, &event, 0) != pdPASS) {
                        ERROR("Failed to add the control event in the queue");
                    }
                }
                break;

            case HOGP_STATE_CLOSED:
                //INFO("STATE CLOSED");
                running = false;
                break;

            case HOGP_STATE_CONNECTED:
                //INFO("STATE CONNECTED");
                {
                    uint8_t messages[MAX_MESSAGES][MAX_MSG_LENGTH];
                    uint8_t sizes[MAX_MESSAGES];
                    hogp_characteristics_t chrs[MAX_MESSAGES];

                    if (!ctx->connection.tx_arrived) {
                        break;  // Waiting for TX
                    }

                    uint8_t num_messages;
                    write_messages(messages, sizes, chrs, &num_messages, ctx);

                    for (uint8_t i = 0; i < num_messages; i++) {                        
                        hogp_result_t res = hogp_notify(messages[i], sizes[i], chrs[i]);
                        if (res == HOGP_OK) ctx->connection.tx_arrived = false;
                    }
                }
                break;
            
            case HOGP_STATE_ADVERTISING:
            case HOGP_STATE_SUSPENDED:
                xQueueReset(ctx->data_queue);
                break;

            default:
                //WARN("Unhandled state %d", ctx->state);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    INFO("HOGP task exiting");
    vTaskDelete(NULL);
}

bool hogp_is_running(void) {
    return eTaskGetState(task) != eDeleted;
}


// Utility functions

/**
 * @brief Processes the control event queue to update the FSM state.
 * Reads from the `control_queue` (non-blocking).
 * Switches based on the current `ctx->state` and the received event.
 * Handles state transitions (e.g., IDLE -> START, CONNECTED -> SUSPENDED).
 * Processes events that don't change state but update context (e.g., MTU update, Protocol change).
 * @param ctx Pointer to the global HOGP context.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_QUEUE_EMPTY if no events are pending.
 */
static hogp_result_t state_transition(hogp_context_t *ctx) {
    hogp_control_event_t e;

    if (xQueueReceive(ctx->control_queue, &e, 0) != pdPASS) {
        //WARN("Failed to read control event from queue");
        return HOGP_ERR_QUEUE_EMPTY;
    }

    INFO("Control event received: %d", e.type);

    switch (ctx->state) {

        case HOGP_STATE_IDLE:
            if (e.type == HOGP_CEVT_BLE_READY) {
                INFO("Transition IDLE → START");
                ctx->state = HOGP_STATE_START;
            }
            else if (e.type == HOGP_CEVT_SHUTDOWN) {
                INFO("Transition IDLE → CLOSED");
                ctx->state = HOGP_STATE_CLOSED;
            }
            break;

        case HOGP_STATE_START:
            if (e.type == HOGP_CEVT_SHUTDOWN) {
                INFO("Transition START → CLOSED");
                ctx->state = HOGP_STATE_CLOSED;
            }
            else if (e.type == HOGP_CEVT_ADV_STARTED) {
                INFO("Transition START → ADVERTISING");
                ctx->state = HOGP_STATE_ADVERTISING;
            }
            break;

        case HOGP_STATE_ADVERTISING:
            if (e.type == HOGP_CEVT_CONNECT) {
                INFO("Transition ADVERTISING → CONNECTED");
                ctx->state = HOGP_STATE_CONNECTED;
                hogp_result_t err = hogp_connect(e.conn_handle);
                if (err != HOGP_OK) {
                    ERROR("Failed to store connection data");
                }
                else if (ctx->connected_cb != NULL) ctx->connected_cb(true);
            }
            else if (e.type == HOGP_CEVT_ADV_COMPLETE) {
                WARN("ADV_COMPLETE → restarting advertising");
                ctx->state = HOGP_STATE_START;
            }
            break;

        case HOGP_STATE_CONNECTED:
            if (e.type == HOGP_CEVT_DISCONNECT) {
                INFO("Transition CONNECTED → START (disconnect)");
                ctx->state = HOGP_STATE_START;
                if (ctx->connected_cb!= NULL) ctx->connected_cb(false);
            }
            else if (e.type == HOGP_CEVT_SUSPEND && e.suspended) {
                INFO("Transition CONNECTED → SUSPENDED");
                ctx->state = HOGP_STATE_SUSPENDED;
                if (ctx->suspended_cb != NULL) ctx->suspended_cb(true);
            }
            else if (e.type == HOGP_CEVT_SHUTDOWN) {
                INFO("Transition CONNECTED → CLOSED");
                ctx->state = HOGP_STATE_CLOSED;
            }
            break;

        case HOGP_STATE_SUSPENDED:
            if (e.type == HOGP_CEVT_DISCONNECT) {
                INFO("Transition SUSPENDED → START");
                ctx->state = HOGP_STATE_START;
                if (ctx->connected_cb!= NULL) ctx->connected_cb(false);
            }
            else if (e.type == HOGP_CEVT_SUSPEND && !e.suspended) {
                INFO("Transition SUSPENDED → CONNECTED");
                ctx->state = HOGP_STATE_CONNECTED;
                if (ctx->suspended_cb != NULL) ctx->suspended_cb(false);
            }
            else if (e.type == HOGP_CEVT_SHUTDOWN) {
                INFO("Transition SUSPENDED → CLOSED");
                ctx->state = HOGP_STATE_CLOSED;
            }
            break;

        default:
            ERROR("Unknown state %d", ctx->state);
    }

    // Other control events
    if (ctx->state == HOGP_STATE_CONNECTED || ctx->state == HOGP_STATE_SUSPENDED) {

        if (e.type == HOGP_CEVT_SET_PROTOCOL) {
            INFO("Protocol changed: %u", e.protocol);
            ctx->connection.protocol = e.protocol;
        }
        else if (e.type == HOGP_CEVT_NOTIFY_TX) {
            INFO("NOTIFY_TX received");
            ctx->connection.tx_arrived = true;
        }
        else if (e.type == HOGP_CEVT_SUBSCRIBE) {
            INFO("Subscribe event: handle=%u flags=%u", e.sub_handle, e.sub);
            hogp_result_t res = hogp_subscribe(e.sub_handle, e.sub);
            return res;
        }
    }

    return HOGP_OK;
}
