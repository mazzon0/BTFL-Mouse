#ifndef TMX_UTILS_H
#define TMX_UTILS_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Data structure for a circular buffer.
 * *
 * 
 */
typedef struct {
    void* buffer;
    size_t item_size;
    size_t depth;
    int head_index;
    int count;

} CircularBuffer_t;

//--- Circular Buffer Functions ---

/**
 * @brief Initializes the circular buffer structure.
 */
void cb_init(CircularBuffer_t* cb, void* buffer, size_t item_size, size_t depth);

/**
 * @brief Adds an item to the circular buffer.
 */
void cb_push(CircularBuffer_t* cb, const void* item);

/**
 * @brief Reads an element from the buffer at a specified temporal offset.
 */
int cb_read_at_offset(CircularBuffer_t* cb, int offset, void* data_out);




#endif // TMX_UTILS_H