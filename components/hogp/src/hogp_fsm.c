#include "hogp_fsm.h"
#include "hogp_common.h"
#include "hogp_data_events.h"
#include "hogp_control_events.h"
#include "hogp_ble.h"

// Utility functions
static int state_transition(hogp_context_t *ctx);
static int write_message(uint8_t *message, uint8_t *size);
static int write_mouse_report(uint8_t *message, uint8_t *size, hogp_data_event_t *event);
static int write_mouse_boot(uint8_t *message, uint8_t *size, hogp_data_event_t *event);


// Task

void hogp_task(void *params) {
    bool running = true;
    hogp_context_t *ctx = hogp_get_context();

    while (running) {
        state_transition(ctx);

        // perform the task associated with the current state
        switch (ctx->state) {
            case HOGP_STATE_START:  // TODO reset queues
                hogp_start_advertising();
                break;

            case HOGP_STATE_CLOSED:
                running = false;
                break;

            case HOGP_STATE_CONNECTED:
                uint8_t message[32];
                uint8_t size;
                int ret = write_message(message, &size);
                if (ret == 0) {
                    hogp_notify(message, size);
                    ctx->connection.tx_arrived = false;
                }
                break;

            default:

        }
        
        vTaskDelay(ctx->update_period_ms / portTICK_PERIOD_MS);   // TODO adapt sleep time to execution time
    }

    vTaskDelete(NULL);
}




// Utility functions

static int state_transition(hogp_context_t *ctx) {
    hogp_control_event_t event;

    if (xQueueIsQueueEmptyFromISR(ctx->control_queue) == pdTRUE) return -1;

    if (xQueueReceive(ctx->control_queue, &event, 1 / portTICK_PERIOD_MS) != pdPASS) {
        // TODO error handling
        return -2;
    }

    switch (ctx->state) {
        case HOGP_STATE_IDLE:
            if(event.type == HOGP_CEVT_BLE_READY) {
                ctx->state = HOGP_STATE_START;
            }
            else if (event.type == HOGP_CEVT_SHUTDOWN) {
                ctx->state = HOGP_STATE_CLOSED;
            }
            break;

        case HOGP_STATE_START:
            if (event.type == HOGP_CEVT_SHUTDOWN) {
                ctx->state = HOGP_STATE_CLOSED;
            }
            else {
                ctx->state = HOGP_STATE_ADVERTISING;
            }
            break;

        case HOGP_STATE_ADVERTISING:
            if (event.type == HOGP_CEVT_CONNECT) {
                ctx->state = HOGP_STATE_CONNECTED;
            }
            else if (event.type == HOGP_CEVT_ADV_COMPLETE) {
                ctx->state = HOGP_STATE_START;
            }
            break;

        case HOGP_STATE_CONNECTED:
            if (event.type == HOGP_CEVT_DISCONNECT) {
                ctx->state = HOGP_STATE_START;
            }
            else if (event.type == HOGP_CEVT_SUSPEND && event.suspended) {
                ctx->state = HOGP_STATE_SUSPENDED;
            }
            else if (event.type == HOGP_CEVT_SHUTDOWN) {
                ctx->state = HOGP_STATE_CLOSED;
            }
            break;

        case HOGP_STATE_SUSPENDED:
            if (event.type == HOGP_CEVT_DISCONNECT) {
                ctx->state = HOGP_STATE_START;
            }
            else if (event.type == HOGP_CEVT_SUSPEND && !event.suspended) {
                ctx->state = HOGP_STATE_CONNECTED;
            }
            else if (event.type == HOGP_CEVT_SHUTDOWN) {
                ctx->state = HOGP_STATE_CLOSED;
            }

        default:

    }

    // Other control events (not changing the state)
    if (ctx->state == HOGP_STATE_CONNECTED || ctx->state == HOGP_STATE_SUSPENDED) {
        if (event.type == HOGP_CEVT_SET_PROTOCOL) {
            ctx->connection.protocol = event.protocol;
        }
        else if (event.type == HOGP_CEVT_NOTIFY_TX) {
            ctx->connection.tx_arrived = true;
        }
        else if (event.type == HOGP_CEVT_MTU) {
            // ignored, the default mtu should be enough (23)
        }
        else if (event.type == HOGP_CEVT_SUBSCRIBE) {
            hogp_subscribe(event.sub_handle, event.sub);
        }
    }

    return 0;
}

static int write_message(uint8_t *message, uint8_t *size) {
    hogp_context_t *ctx = hogp_get_context();
    hogp_data_event_t event;

    if (ctx->connection.tx_arrived) return -1;  // still waiting for a NOTIFY_TX to arrive

    if (xQueueReceive(ctx->data_queue, &event, 1 / portTICK_PERIOD_MS) != pdPASS) {
        // TODO error handling
        return -2;
    }

    // Mouse message
    if (event.type == HOGP_DEVT_CURSOR_MOTION || event.type == HOGP_DEVT_SCROLL_MOTION || event.type == HOGP_DEVT_MOUSE_BUTTON_PRESSED || event.type == HOGP_DEVT_MOUSE_BUTTON_RELEASED) {
        if (ctx->connection.protocol == HOGP_PROTOCOL_REPORT) {
            return write_mouse_report(message, size, &event);
        }
        else if (ctx->connection.protocol == HOGP_PROTOCOL_REPORT) {
            return write_mouse_boot(message, size, &event);
        }
    }

    return -3;
}

static int write_mouse_report(uint8_t *message, uint8_t *size, hogp_data_event_t *event) {  // TODO compose multiple events together
    hogp_context_t *ctx = hogp_get_context();

    if (event->type == HOGP_DEVT_CURSOR_MOTION || event->type == HOGP_DEVT_SCROLL_MOTION || event->type == HOGP_DEVT_MOUSE_BUTTON_PRESSED || event->type == HOGP_DEVT_MOUSE_BUTTON_RELEASED) {
        if (!ctx->connection.notify_sub.mouse_report) return 1;   // if not subscribed, no need to send anything
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

    return 0;
}

static int write_mouse_boot(uint8_t *message, uint8_t *size, hogp_data_event_t *event) {  // TODO
    return 1;
}
