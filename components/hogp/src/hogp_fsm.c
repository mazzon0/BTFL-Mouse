#include "hogp_fsm.h"
#include "hogp_common.h"
#include "hogp_data_events.h"
#include "hogp_control_events.h"
#include "hogp_ble.h"

// Utility functions
static hogp_result_t state_transition(hogp_context_t *ctx);
static hogp_result_t write_message(uint8_t *message, uint8_t *size, hogp_protocol_t protocol);
static hogp_result_t write_mouse_report(uint8_t *message, uint8_t *size, hogp_data_event_t *event);
static hogp_result_t write_mouse_boot(uint8_t *message, uint8_t *size, hogp_data_event_t *event);


// Task

static TaskHandle_t task = NULL;

void hogp_task(void *params) {
    bool running = true;
    hogp_context_t *ctx = hogp_get_context();
    task = xTaskGetCurrentTaskHandle();

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
                uint8_t message[32];
                uint8_t size;

                hogp_protocol_t protocol = ctx->connection.protocol;
                hogp_characteristics_t chr = (protocol == HOGP_PROTOCOL_REPORT) ? MOUSE_REPORT : MOUSE_BOOT; 
                
                hogp_result_t ret = write_message(message, &size, protocol);
                
                if (ret == HOGP_OK) {
                    INFO("Sending report: %d bytes to %d", size, ctx->connection.conn_handle);
                    hogp_result_t res = hogp_notify(message, size, chr);

                    if (res == HOGP_OK) ctx->connection.tx_arrived = false;
                }
                else if (ret == HOGP_ERR_QUEUE_EMPTY || ret == HOGP_ERR_TX_BUSY || ret == HOGP_ERR_NOT_SUPPORTED || ret == HOGP_ERR_NOT_SUBSCRIBED) { 
                    /* normal condition */ 
                }
                else { 
                    WARN("write_message returned the unexpected value %d", ret); 
                }
                break;
            
            case HOGP_STATE_ADVERTISING:
            case HOGP_STATE_SUSPENDED:
                xQueueReset(ctx->data_queue);
                break;

            default:
                //WARN("Unhandled state %d", ctx->state);
        }

        vTaskDelay(ctx->update_period_ms / portTICK_PERIOD_MS);
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
            }
            else if (e.type == HOGP_CEVT_SUSPEND && e.suspended) {
                INFO("Transition CONNECTED → SUSPENDED");
                ctx->state = HOGP_STATE_SUSPENDED;
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
            }
            else if (e.type == HOGP_CEVT_SUSPEND && !e.suspended) {
                INFO("Transition SUSPENDED → CONNECTED");
                ctx->state = HOGP_STATE_CONNECTED;
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
 * @brief Prepares a HID report message based on the active protocol.
 * Checks if the transport is ready (Flow control: `tx_arrived`).
 * Reads user inputs (mouse/keyboard events) from the `data_queue`.
 * Dispatches the event to the correct formatter (Report Mode vs Boot Mode).
 * @param message Buffer to store the formatted report data.
 * @param size Pointer to store the size of the generated report.
 * @param protocol The current active protocol (Report or Boot).
 * @return HOGP_OK on success.
 * @return HOGP_ERR_TX_BUSY if the BLE stack hasn't acknowledged the previous transmission.
 * @return HOGP_ERR_QUEUE_EMPTY if there is no data to send.
 * @return HOGP_ERR_NOT_SUPPORTED if the event type isn't handled.
 */
static hogp_result_t write_message(uint8_t *message, uint8_t *size, hogp_protocol_t protocol) {
    hogp_context_t *ctx = hogp_get_context();
    hogp_data_event_t e;

    if (!ctx->connection.tx_arrived) {
        return HOGP_ERR_TX_BUSY;
    }

    if (xQueueReceive(ctx->data_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
        //WARN("Data queue is empty");
        return HOGP_ERR_QUEUE_EMPTY;
    }

    INFO("Data event received: %d", e.type);

    // Mouse message
    if (e.type == HOGP_DEVT_CURSOR_MOTION ||
        e.type == HOGP_DEVT_SCROLL_MOTION ||
        e.type == HOGP_DEVT_MOUSE_BUTTON_PRESSED ||
        e.type == HOGP_DEVT_MOUSE_BUTTON_RELEASED)
    {
        if (protocol == HOGP_PROTOCOL_REPORT) {
            return write_mouse_report(message, size, &e);
        }
        else if (protocol == HOGP_PROTOCOL_BOOT) {
            return write_mouse_boot(message, size, &e);
        }
    }

    WARN("Unsupported data event type: %d", e.type);
    return HOGP_ERR_NOT_SUPPORTED;
}

/**
 * @brief Formats a data event into a standard Mouse HID Report.
 * Uses the Report Map defined in `hogp_ble.c`.
 * Mapping: [Buttons, X, Y, Wheel].
 * Checks if the host has subscribed to notifications before formatting.
 * @param message Buffer to write the 4-byte report into.
 * @param size Pointer to write the size (4 bytes).
 * @param event The data event (button press, motion, etc.) to convert.
 * @return HOGP_OK on success.
 * @return HOGP_ERR_NOT_SUBSCRIBED if the host hasn't enabled notifications.
 */
static hogp_result_t write_mouse_report(uint8_t *message, uint8_t *size, hogp_data_event_t *event) {  // TODO compose multiple events together
    hogp_context_t *ctx = hogp_get_context();

    if (event->type == HOGP_DEVT_CURSOR_MOTION || event->type == HOGP_DEVT_SCROLL_MOTION || event->type == HOGP_DEVT_MOUSE_BUTTON_PRESSED || event->type == HOGP_DEVT_MOUSE_BUTTON_RELEASED) {
        if (!ctx->connection.notify_sub.mouse_report) return HOGP_ERR_NOT_SUBSCRIBED;   // if not subscribed, no need to send anything
        *size = 4;
    }

    switch (event->type) {
        case HOGP_DEVT_CURSOR_MOTION:
            message[0] = 0x00;
            message[1] = event->x;
            message[2] = event->y;
            message[3] = 0;
            break;

        case HOGP_DEVT_SCROLL_MOTION:
            message[0] = 0x00;
            message[1] = 0;
            message[2] = 0;
            message[3] = event->y;
            break;

        case HOGP_DEVT_MOUSE_BUTTON_PRESSED:
            ctx->hid_state.buttons |= (1 << event->button);

            message[0] = ctx->hid_state.buttons;
            message[1] = 0;
            message[2] = 0;
            message[3] = 0;
            break;

        case HOGP_DEVT_MOUSE_BUTTON_RELEASED:
            ctx->hid_state.buttons &= ~(1 << event->button);

            message[0] = ctx->hid_state.buttons;
            message[1] = 0;
            message[2] = 0;
            message[3] = 0;
            break;
    }

    return HOGP_OK;
}

/**
 * @brief Formats a data event into a Mouse Boot Protocol Report.
 * @note This is currently not implemented (Stub).
 * @return HOGP_ERR_NOT_SUPPORTED always.
 */
static hogp_result_t write_mouse_boot(uint8_t *message, uint8_t *size, hogp_data_event_t *event) {  // TODO
    return HOGP_ERR_NOT_SUPPORTED;
}
