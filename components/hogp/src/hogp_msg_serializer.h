#ifndef HOGP_MSG_SERIALIZER_H
#define HOGP_MSG_SERIALIZER_H

#include "hogp_common.h"

/**
 * @brief Maximum number of messages that one call of write_messages can serialize
 */
extern const uint8_t MAX_MESSAGES;

/**
 * @brief Maximum length of a message in bytes
 */
extern const uint8_t MAX_MSG_LENGTH;

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

/**
 * @brief Writes a list or messages reading elements from the queue
 * Produces at most MAX_MESSAGES messages.
 * @param messages Array of messages (output argument)
 * @param sizes Array of message sizes (output argument)
 * @param characteristics Array of message characteristics (output argument)
 * @param num_messages Number of messages (output argument).
 * @return HOGP_OK is successfull, HOGP_ERR_QUEUE_EMPTY if there are no events to send
 */
hogp_result_t write_messages(uint8_t messages[MAX_MESSAGES][MAX_MSG_LENGTH], uint8_t *sizes, hogp_characteristics_t *characteristics, uint8_t *num_messages, hogp_context_t *ctx);

#endif
