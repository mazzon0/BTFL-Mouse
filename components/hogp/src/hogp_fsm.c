#include "hogp_fsm.h"
#include "hogp_common.h"
#include "hogp_data_events.h"
#include "hogp_control_events.h"
#include "hogp_ble.h"

// Utility functions
static hogp_error_t state_transition(hogp_context_t *ctx);
static hogp_error_t write_message(uint8_t *message, uint8_t *size, hogp_protocol_t protocol);
static hogp_error_t write_mouse_report(uint8_t *message, uint8_t *size, hogp_data_event_t *event);
static hogp_error_t write_mouse_boot(uint8_t *message, uint8_t *size, hogp_data_event_t *event);


// Task

void hogp_task(void *params) {
    bool running = true;
    hogp_context_t *ctx = hogp_get_context();

    INFO("HOGP task started");

    while (running) {

        int tr = state_transition(ctx);
        if (tr < 0 && tr != -1) {   // TODO check codes
            WARN("state_transition returned %d", tr);
        }

        switch (ctx->state) {

            case HOGP_STATE_START:
                //INFO("STATE START");
                hogp_error_t res = hogp_start_advertising();

                if (res == HOGP_OK) {
                    hogp_control_event_t event;
                    event.type = HOGP_CEVT_ADV_STARTED;
                    if (xQueueSendToBackFromISR(ctx->control_queue, &event, 1 / portTICK_PERIOD_MS) != pdPASS) {
                        ESP_LOGE(HID_TAG, "ERROR: waited too much to add the control event in the queue");
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
                int ret = write_message(message, &size, protocol);
                if (ret == HOGP_OK) {
                    //INFO("Sending report: %d bytes to %d", size, ctx->connection.conn_handle);
                    WARN("sending with conn_handle = %d, placed at %d", ctx->connection.conn_handle, &ctx->connection.conn_handle);
                    hogp_error_t res = hogp_notify(message, size, chr);

                    if (res == HOGP_OK) ctx->connection.tx_arrived = false;
                }
                else if (ret == HOGP_NOTHING_TO_SEND || ret == HOGP_TX_NOT_RECEIVED || ret == HOGP_NOT_SUPPORTED_YET || HOGP_NOT_SUBSCRIBED) { /*INFO("NOT SENDING: %d", ret);*//* normat condition */ }
                else { WARN("write_message returned the unexpected value %d", ret); }
                break;

            default:
                //WARN("Unhandled state %d", ctx->state);
        }

        vTaskDelay(ctx->update_period_ms / portTICK_PERIOD_MS);
    }

    INFO("HOGP task exiting");
    vTaskDelete(NULL);
}





// Utility functions

static hogp_error_t state_transition(hogp_context_t *ctx) {
    hogp_control_event_t e;

    if (xQueueReceive(ctx->control_queue, &e, 0) != pdPASS) {
        //WARN("Failed to read control event from queue");
        return HOGP_NO_CONTROLS;
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
                hogp_error_t err = hogp_connect(e.conn_handle);
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
            hogp_error_t res = hogp_subscribe(e.sub_handle, e.sub);
            return res;
        }
    }

    return HOGP_OK;
}

static hogp_error_t write_message(uint8_t *message, uint8_t *size, hogp_protocol_t protocol) {
    hogp_context_t *ctx = hogp_get_context();
    hogp_data_event_t e;

    if (!ctx->connection.tx_arrived) {
        return HOGP_TX_NOT_RECEIVED;
    }

    if (xQueueReceive(ctx->data_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
        //WARN("Data queue is empty");
        return HOGP_NOTHING_TO_SEND;
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
    return HOGP_NOT_SUPPORTED_YET;
}

static hogp_error_t write_mouse_report(uint8_t *message, uint8_t *size, hogp_data_event_t *event) {  // TODO compose multiple events together
    hogp_context_t *ctx = hogp_get_context();

    if (event->type == HOGP_DEVT_CURSOR_MOTION || event->type == HOGP_DEVT_SCROLL_MOTION || event->type == HOGP_DEVT_MOUSE_BUTTON_PRESSED || event->type == HOGP_DEVT_MOUSE_BUTTON_RELEASED) {
        if (!ctx->connection.notify_sub.mouse_report) return HOGP_NOT_SUBSCRIBED;   // if not subscribed, no need to send anything
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
            message[0] = 0x00;
            message[1] = 0;
            message[2] = 0;
            message[3] = 0;
            break;

        case HOGP_DEVT_MOUSE_BUTTON_RELEASED:
            message[0] = 0x00;
            message[1] = 0;
            message[2] = 0;
            message[3] = 0;
            break;
    }

    return HOGP_OK;
}

static hogp_error_t write_mouse_boot(uint8_t *message, uint8_t *size, hogp_data_event_t *event) {  // TODO
    return HOGP_NOT_SUPPORTED_YET;
}
