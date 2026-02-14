#include "../src/hogp_msg_serializer.h"
#include "../include/hogp_data_events.h"
#include <assert.h>
#include <stdio.h>

#define MAX_QUEUE_LENGTH 128
hogp_data_event_t queue[MAX_QUEUE_LENGTH];
uint8_t queue_head = 0;
uint8_t queue_tail = 0;

// implementation that emulates xQueueReceive from FreeRTOS for testing
int xQueueReceive(int queue_handle, hogp_data_event_t *event, int max_time) {
    if (queue_head == queue_tail) return -1;

    *event = queue[queue_head];
    queue_head = (queue_head + 1) % MAX_QUEUE_LENGTH;
    return 0;
}

// implementation that emulates xQueuePush from FreeRTOS for testing
int xQueuePush(int queue_handle, hogp_data_event_t *event) {
    if (queue_head == (queue_tail + 1) % MAX_QUEUE_LENGTH) return -1;

    queue[queue_tail] = *event;
    queue_tail = (queue_tail + 1) % MAX_QUEUE_LENGTH;
    return 0;
}

int main() {
    uint8_t target_msgs[MAX_MESSAGES][MAX_MSG_LENGTH];
    uint8_t messages[MAX_MESSAGES][MAX_MSG_LENGTH];
    uint8_t sizes[MAX_MESSAGES];
    hogp_characteristics_t chrs[MAX_MESSAGES];
    uint8_t num_messages;
    hogp_result_t res;
    hogp_context_t ctx = {0};
    ctx.connection.protocol = HOGP_PROTOCOL_REPORT;
    
    // Test 1: Empty queue
    {
        printf("Test 1 - empty queue: "); fflush(stdout);
        ctx.hid_state.battery_level = 50;
        ctx.hid_state.buttons = 0x00;
        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);
        assert(res == HOGP_ERR_QUEUE_EMPTY);
        printf("Success\n");
    }

    // Test 2: Cursor motion
    {
        printf("Test 2 - cursor motion: "); fflush(stdout);
        // add events to queue
        hogp_data_event_t events[2] = {
            {
                .type = HOGP_DEVT_CURSOR_MOTION,
                .x = 2,
                .y = 6,
            },
            {
                .type = HOGP_DEVT_CURSOR_MOTION,
                .x = 1,
                .y = 4,
            }
        };
        for (int i = 0; i < 2; i++) {
            xQueuePush(0, &events[i]);
        }
        
        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);
        target_msgs[0][0] = 0x00;
        target_msgs[0][1] = 0x03;
        target_msgs[0][2] = 0x0A;
        target_msgs[0][3] = 0x00;

        assert(res == HOGP_OK);
        assert(num_messages == 1);
        assert(sizes[0] == 4);
        assert(chrs[0] == MOUSE_REPORT);
        assert(messages[0][0] == target_msgs[0][0]);
        assert(messages[0][1] == target_msgs[0][1]);
        assert(messages[0][2] == target_msgs[0][2]);
        assert(messages[0][3] == target_msgs[0][3]);
        printf("Success\n");
    }

    // Test 3: Mouse buttons
    {
        printf("Test 3 - mouse buttons: "); fflush(stdout);
        // add events to queue
        hogp_data_event_t events[2] = {
            {
                .type = HOGP_DEVT_MOUSE_BUTTON_PRESSED,
                .button = HOGP_MOUSE_BLEFT,
            },
            {
                .type = HOGP_DEVT_MOUSE_BUTTON_PRESSED,
                .button = HOGP_MOUSE_BMIDDLE,
            }
        };
        for (int i = 0; i < 2; i++) {
            xQueuePush(0, &events[i]);
        }

        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);
        target_msgs[0][0] = 0x05;
        target_msgs[0][1] = 0x00;
        target_msgs[0][2] = 0x00;
        target_msgs[0][3] = 0x00;

        assert(res == HOGP_OK);
        assert(num_messages == 1);
        assert(sizes[0] == 4);
        assert(chrs[0] == MOUSE_REPORT);
        assert(messages[0][0] == target_msgs[0][0]);
        assert(messages[0][1] == target_msgs[0][1]);
        assert(messages[0][2] == target_msgs[0][2]);
        assert(messages[0][3] == target_msgs[0][3]);
        
        // add other events to queue (tested on multiple iterations because buttons needs to store the state)
        hogp_data_event_t other_events[2] = {
            {
                .type = HOGP_DEVT_MOUSE_BUTTON_RELEASED,
                .button = HOGP_MOUSE_BLEFT,
            },
            {
                .type = HOGP_DEVT_MOUSE_BUTTON_PRESSED,
                .button = HOGP_MOUSE_BRIGHT,
            }
        };
        for (int i = 0; i < 2; i++) {
            xQueuePush(0, &other_events[i]);
        }

        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);
        target_msgs[0][0] = 0x06;
        target_msgs[0][1] = 0x00;
        target_msgs[0][2] = 0x00;
        target_msgs[0][3] = 0x00;

        assert(res == HOGP_OK);
        assert(num_messages == 1);
        assert(sizes[0] == 4);
        assert(chrs[0] == MOUSE_REPORT);
        assert(messages[0][0] == target_msgs[0][0]);
        assert(messages[0][1] == target_msgs[0][1]);
        assert(messages[0][2] == target_msgs[0][2]);
        assert(messages[0][3] == target_msgs[0][3]);

        printf("Success\n");
    }

    // Test 4: Scroll
    {
        printf("Test 4 - scroll motion: "); fflush(stdout);
        // add events to queue
        hogp_data_event_t events[2] = {
            {
                .type = HOGP_DEVT_SCROLL_MOTION,
                .y = 8,
            },
            {
                .type = HOGP_DEVT_SCROLL_MOTION,
                .y = 5,
            }
        };
        for (int i = 0; i < 2; i++) {
            xQueuePush(0, &events[i]);
        }
        
        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);
        target_msgs[0][0] = 0x06;
        target_msgs[0][1] = 0x00;
        target_msgs[0][2] = 0x00;
        target_msgs[0][3] = 0x0D;

        assert(res == HOGP_OK);
        assert(num_messages == 1);
        assert(sizes[0] == 4);
        assert(chrs[0] == MOUSE_REPORT);
        assert(messages[0][0] == target_msgs[0][0]);
        assert(messages[0][1] == target_msgs[0][1]);
        assert(messages[0][2] == target_msgs[0][2]);
        assert(messages[0][3] == target_msgs[0][3]);
        printf("Success\n");
    }

    // Test 5: Battery level
    {
        printf("Test 5 - battery level: "); fflush(stdout);
        // add events to queue
        hogp_data_event_t events[2] = {
            {
                .type = HOGP_DEVT_BATTERY_LEVEL_UPDATE,
                .battery_level = 64,
            },
            {
                .type = HOGP_DEVT_BATTERY_LEVEL_UPDATE,
                .battery_level = 62,
            }
        };
        for (int i = 0; i < 2; i++) {
            xQueuePush(0, &events[i]);
        }
        
        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);
        target_msgs[0][0] = 62;

        assert(res == HOGP_OK);
        assert(num_messages == 1);
        assert(sizes[0] == 1);
        assert(chrs[0] == BATTERY_LEVEL);
        assert(messages[0][0] == target_msgs[0][0]);
        printf("Success\n");
    }

    // Test 6: Mixed sequence
    {
        printf("Test 6 - mixed types & state persistence: "); fflush(stdout);
        
        ctx.hid_state.battery_level = 100;
        ctx.hid_state.buttons = 0;
        
        hogp_data_event_t events[] = {
            {.type = HOGP_DEVT_CURSOR_MOTION, .x = 10, .y = 20},            
            {.type = HOGP_DEVT_MOUSE_BUTTON_PRESSED, .button = HOGP_MOUSE_BLEFT}, 
            {.type = HOGP_DEVT_BATTERY_LEVEL_UPDATE, .battery_level = 90},  
            {.type = HOGP_DEVT_CURSOR_MOTION, .x = 5, .y = 5},              
            {.type = HOGP_DEVT_BATTERY_LEVEL_UPDATE, .battery_level = 80}   
        };

        for (int i = 0; i < 5; i++) {
            xQueuePush(0, &events[i]);
        }

        res = write_messages(messages, sizes, chrs, &num_messages, &ctx);

        assert(res == HOGP_OK);
        assert(num_messages == 4);
        assert(chrs[0] == MOUSE_REPORT);
        assert(sizes[0] == 4);
        assert(messages[0][0] == 0x01);
        assert(messages[0][1] == 10);
        assert(messages[0][2] == 20);
        assert(chrs[1] == BATTERY_LEVEL);
        assert(sizes[1] == 1);
        assert(messages[1][0] == 90);
        assert(chrs[2] == MOUSE_REPORT);
        assert(messages[2][0] == 0x01);
        assert(messages[2][1] == 5);
        assert(messages[2][2] == 5);
        assert(chrs[3] == BATTERY_LEVEL);
        assert(messages[3][0] == 80);

        printf("Success\n");
    }

    return 0;
}
