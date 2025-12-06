#include "tmx_utils.h"

//get memory address of item at given index
static void* cb_get_ptr_at(CircularBuffer_t* cb, int index) {
    uint8_t* byte_ptr = (uint8_t*)cb->buffer;
    return (void*)(byte_ptr + (index % cb->depth) * cb->item_size);
}

void cb_init(CircularBuffer_t* cb, void* buffer, size_t item_size, size_t depth) {
    cb->buffer = buffer;
    cb->item_size = item_size;
    cb->depth = depth;
    cb->head_index = 0;
    cb->count = 0;
}

int cb_push(CircularBuffer_t* cb, const void* item){
    void* write_ptr = cb_get_ptr_at(cb, cb->head_index);
    memcpy(write_ptr, item, cb->item_size);

    cb->head_index = (cb->head_index + 1) % cb->depth;
    if (cb->count < cb->depth) {
        cb->count++;
    }
    return 0;  //return success
}

int cb_read_at_offset(CircularBuffer_t* cb, int offset, void* data_out) {
    if (offset < 0 || offset >= cb->count) {
        //offset not valid, past data not yet written
        return -1; 
    }
    
    //Calculate read index
    int read_index = (cb->head_index - offset - 1);
    
    if (read_index < 0) {
        read_index += cb->depth;
    }
    
    //Get memory address
    void* read_ptr = cb_get_ptr_at(cb, read_index);

    memcpy(data_out, read_ptr, cb->item_size);

    return 0;
}

uint64_t get_current_time_ms(void) {
    uint64_t ms = esp_timer_get_time() / 1000ULL;
    return ms;
}