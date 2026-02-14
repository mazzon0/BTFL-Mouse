#include "hogp_msg_serializer.h"
#include "hogp_data_events.h"

#ifdef HOGP_TEST
#include <stdio.h>
#endif

const uint8_t MAX_MESSAGES = 4;
const uint8_t MAX_MSG_LENGTH = 4;

static hogp_message_t init_message(hogp_message_type_t type, hogp_hid_state_t *hid_state) {
    hogp_message_t msg = {0};
    msg.type = type;

    switch (type) {
        case MSG_TYPE_BATTERY:
            msg.battery_data.level = hid_state->battery_level;
            break;

        case MSG_TYPE_MOUSE:
            msg.mouse_data.buttons = hid_state->buttons;
            break;
    }

    return msg;
}

/**
 * @brief Tries to add an hogp_data_event_t to a hogp_message_t and updates the HID state (buttons state and battery level).
 * @param message
 * @param event
 * @return HOGP_OK if successfull, HOGP_ERR_INVALID_ARG if some logic constraint doesn't allow the operation
 */
static hogp_result_t add_event_to_message(hogp_message_t *message, hogp_data_event_t *event, hogp_hid_state_t *hid_state) {
    hogp_message_type_t msg_type = (event->type == HOGP_DEVT_BATTERY_LEVEL_UPDATE) ? MSG_TYPE_BATTERY : MSG_TYPE_MOUSE;

    if (message->type != msg_type) return HOGP_ERR_INVALID_ARG;

    // Battery message
    if (message->type == MSG_TYPE_BATTERY) {
        // Add to message
        message->battery_data.level = event->battery_level;
        hid_state->battery_level = event->battery_level;
        return HOGP_OK;
    }

    // Mouse message
    if (message->type == MSG_TYPE_MOUSE) {
        switch (event->type) {
            case HOGP_DEVT_MOUSE_BUTTON_PRESSED:
                if (event->button & message->mouse_data.changed_buttons) return HOGP_ERR_INVALID_ARG;
                message->mouse_data.buttons |= event->button;
                hid_state->buttons |= event->button;
                message->mouse_data.changed_buttons |=event->button;
                break;

            case HOGP_DEVT_MOUSE_BUTTON_RELEASED:
                if (event->button & message->mouse_data.changed_buttons) return HOGP_ERR_INVALID_ARG;
                message->mouse_data.buttons &= ~event->button;
                hid_state->buttons &= (0xFF & ~event->button);
                message->mouse_data.changed_buttons |=event->button;
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

hogp_result_t write_messages(uint8_t messages[MAX_MESSAGES][MAX_MSG_LENGTH], uint8_t *sizes, hogp_characteristics_t *characteristics, uint8_t *num_messages, hogp_context_t *ctx) {
    *num_messages = 0;

    // Get first data event
    hogp_data_event_t e;
    if (xQueueReceive(ctx->data_queue, &e, 1 / portTICK_PERIOD_MS) != pdPASS) {
        return HOGP_ERR_QUEUE_EMPTY;
    }

    hogp_message_t msg = init_message((e.type == HOGP_DEVT_BATTERY_LEVEL_UPDATE) ? MSG_TYPE_BATTERY : MSG_TYPE_MOUSE, &ctx->hid_state);
    hogp_result_t res = add_event_to_message(&msg, &e, &ctx->hid_state);

    // Receive all other data events
    while (xQueueReceive(ctx->data_queue, &e, 1 / portTICK_PERIOD_MS) == pdPASS) {
        if (add_event_to_message(&msg, &e, &ctx->hid_state) == HOGP_OK) continue;
        
        // If events are incompatible, serializes the message and creates a new one
        message_to_binary(&msg, messages[*num_messages], &sizes[*num_messages], &characteristics[*num_messages], ctx->connection.protocol);
        (*num_messages)++;

        // Add event to new message
        msg = init_message((e.type == HOGP_DEVT_BATTERY_LEVEL_UPDATE) ? MSG_TYPE_BATTERY : MSG_TYPE_MOUSE, &ctx->hid_state);
        add_event_to_message(&msg, &e, &ctx->hid_state);

        // If we are close to the maximum number of messages, save last message and exit
        if (*num_messages == MAX_MESSAGES - 1) {
            message_to_binary(&msg, messages[*num_messages], &sizes[*num_messages], &characteristics[*num_messages], ctx->connection.protocol);
            (*num_messages)++;
            return HOGP_OK;
        }
    }

    message_to_binary(&msg, messages[*num_messages], &sizes[*num_messages], &characteristics[*num_messages], ctx->connection.protocol);
    (*num_messages)++;

    return HOGP_OK;
}
