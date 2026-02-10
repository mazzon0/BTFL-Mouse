#include "hogp_fsm.h"
#include "hogp_common.h"
#include "hogp_data_events.h"
#include "hogp_control_events.h"
#include "hogp_ble.h"

const uint8_t MAX_MESSAGES = 4;
const uint8_t MAX_MSG_LENGTH = 4;

/**
 * @brief Types of implemented messages
 */
typedef enum {
    MSG_TYPE_MOUSE,
    MSG_TYPE_BATTERY,
} hogp_message_type_t;

/**
 * @brief Data for all implemented messages
 */
typedef struct {
    hogp_message_type_t type;

    union {
        struct {
            uint8_t buttons;
            uint8_t changed_buttons;
            uint8_t dx;
            uint8_t dy;
            uint8_t scroll;
        } mouse_data;

        struct {
            uint8_t level;
        } battery_data;
    };
} hogp_message_t;


// Utility functions
static hogp_result_t state_transition(hogp_context_t *ctx);
static hogp_result_t write_messages(uint8_t messages[MAX_MESSAGES][MAX_MSG_LENGTH], uint8_t *sizes, hogp_characteristics_t *characteristics, uint8_t *num_messages);

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
                    write_messages(messages, sizes, chrs, &num_messages);

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

/**
 * @brief Tries to add an hogp_data_event_t to a hogp_message_t
 * @param message
 * @param event
 * @return HOGP_OK if successfull, HOGP_ERR_INVALID_ARG if some logic constraint doesn't allow the operation
 */
static hogp_result_t add_event_to_message(hogp_message_t *message, hogp_data_event_t *event) {
    hogp_message_type_t msg_type = (event->type == HOGP_DEVT_BATTERY_LEVEL_UPDATE) ? MSG_TYPE_BATTERY : MSG_TYPE_MOUSE;

    if (message->type != msg_type) return HOGP_ERR_INVALID_ARG;

    // Battery message
    if (message->type == MSG_TYPE_BATTERY) {
        // Add to message
        message->battery_data.level = event->battery_level;
        return HOGP_OK;
    }

    // Mouse message
    if (message->type == MSG_TYPE_MOUSE) {
        switch (event->type) {
            case HOGP_DEVT_MOUSE_BUTTON_PRESSED:
                if (event->button & message->mouse_data.changed_buttons) return HOGP_ERR_INVALID_ARG;
                message->mouse_data.buttons |= event->button;
                break;

            case HOGP_DEVT_MOUSE_BUTTON_RELEASED:
                if (event->button & message->mouse_data.changed_buttons) return HOGP_ERR_INVALID_ARG;
                message->mouse_data.buttons &= ~event->button;
                break;

            case HOGP_DEVT_CURSOR_MOTION:
                message->mouse_data.dx += event->x;
                message->mouse_data.dy += event->y;
                break;

            case HOGP_DEVT_SCROLL_MOTION:
                message->mouse_data.scroll += event->y;
                break;

            default:
        }
        return HOGP_OK;
    }

    // This point should never be reached, but in case the unknown event is ignored
    return HOGP_OK;
}

/**
 * @brief Translate the hogp_message_t to a binary message
 * @param message The message to translate
 * @param bin_msg The binary message (output argument)
 * @param size The size of the binary message (output argument)
 * @param chr The characteristic of the binary message (output argument)
 * @param protocol For mouse messages, specifies report or boot protocol
 */
void message_to_binary(hogp_message_t *message, uint8_t *bin_msg, uint8_t *size, hogp_characteristics_t *chr, hogp_protocol_t protocol) {
    switch (message->type) {
        case MSG_TYPE_BATTERY:
            bin_msg[0] = message->battery_data.level;
            *size = 1;
            *chr = BATTERY_LEVEL;
            break;

        case MSG_TYPE_MOUSE:
            if (protocol == HOGP_PROTOCOL_REPORT) {
                bin_msg[0] = message->mouse_data.buttons;
                bin_msg[1] = message->mouse_data.dx;
                bin_msg[2] = message->mouse_data.dy;
                bin_msg[3] = message->mouse_data.scroll;
                *size = 4;
                *chr = MOUSE_REPORT;
            }
            else if (protocol == HOGP_PROTOCOL_BOOT) {
                // TODO
                *chr = MOUSE_BOOT;
            }
            break;
    }
}

/**
 * @brief Writes a list or messages reading elements from the queue
 * Produces at most MAX_MESSAGES messages.
 * @param messages Array of messages (output argument)
 * @param sizes Array of message sizes (output argument)
 * @param characteristics Array of message characteristics (output argument)
 * @param num_messages Number of messages (output argument).
 * @return HOGP_OK is successfull, HOGP_ERR_QUEUE_EMPTY if there are no events to send
 */
static hogp_result_t write_messages(uint8_t messages[MAX_MESSAGES][MAX_MSG_LENGTH], uint8_t *sizes, hogp_characteristics_t *characteristics, uint8_t *num_messages) {
    hogp_context_t *ctx = hogp_get_context();
    hogp_message_t msg = {0};
    *num_messages = 0;

    // Get first data event
    hogp_data_event_t e;
    if (xQueueReceive(ctx->data_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
        return HOGP_ERR_QUEUE_EMPTY;
    }

    msg.type = (e.type == HOGP_DEVT_BATTERY_LEVEL_UPDATE) ? MSG_TYPE_BATTERY : MSG_TYPE_MOUSE;
    add_event_to_message(&msg, &e);

    // Receive all other data events
    while (xQueueReceive(ctx->data_queue, &e, 1 / portTICK_PERIOD_MS) == pdPASS) {
        if (add_event_to_message(&msg, &e) != HOGP_OK) {
            message_to_binary(&msg, messages[*num_messages], &sizes[*num_messages], &characteristics[*num_messages], ctx->connection.protocol);
            (*num_messages)++;

            // Add event to new message
            msg = (hogp_message_t) {0};
            msg.type = (e.type == HOGP_DEVT_BATTERY_LEVEL_UPDATE) ? MSG_TYPE_BATTERY : MSG_TYPE_MOUSE;
            add_event_to_message(&msg, &e);

            // If we are close to the maximum number of messages, save last message and exit
            if (*num_messages == MAX_MESSAGES - 1) {
                message_to_binary(&msg, messages[*num_messages], &sizes[*num_messages], &characteristics[*num_messages], ctx->connection.protocol);
                (*num_messages)++;
                return HOGP_OK;
            }
        }
    }

    message_to_binary(&msg, messages[*num_messages], &sizes[*num_messages], &characteristics[*num_messages], ctx->connection.protocol);
    (*num_messages)++;

    return HOGP_OK;
}
