#include "hogp_context.h"
#include "hogp_common.h"
#include "hogp_data_events.h"
#include "hogp_control_events.h"
#include <string.h>

static bool is_init_info_ok(const hogp_init_info_t *init_info);

hogp_result_t hogp_context_init(const hogp_init_info_t *init_info) {
    hogp_context_t *ctx = hogp_get_context();
    
    if (init_info == NULL) {
        ERROR("Failed to initialize context: init_info argument is NULL");
        return HOGP_ERR_INVALID_ARG;
    }

    if (!is_init_info_ok(init_info)) {
        return HOGP_ERR_INVALID_ARG;
    }

    INFO("Initializing HOGP context. Device Name: '%s', Appearance: 0x%04X, Period: %dms", 
         init_info->device_data.device_name, 
         init_info->device_data.appearance,
         init_info->register_period_ms);

    ctx->connection = (hogp_conn_t) {0};
    ctx->connection.mtu = 23;
    ctx->connection.tx_arrived = true;
    ctx->connection.protocol = HOGP_PROTOCOL_REPORT;
    ctx->device.appearance = init_info->device_data.appearance;
    strcpy(ctx->device.device_name, init_info->device_data.device_name);
    ctx->state = HOGP_STATE_IDLE;
    ctx->register_period_ms = init_info->register_period_ms;
    ctx->transmit_period_ms = init_info->transmit_period_ms;
    ctx->hid_state.buttons = 0x00;
    ctx->connected_cb = init_info->connected_cb;
    ctx->suspended_cb = init_info->suspended_cb;

    ctx->control_queue = xQueueCreate(16, sizeof(hogp_control_event_t));
    if (ctx->control_queue == NULL) {
        ERROR("Failed to create control queue (Out of memory)");
        return HOGP_ERR_NO_MEM;
    }

    ctx->data_queue = xQueueCreate(32, sizeof(hogp_data_event_t));
    if (ctx->data_queue == NULL) {
        ERROR("Failed to create data queue (Out of memory)");
        // Clean up the previously created queue
        vQueueDelete(ctx->control_queue);
        ctx->control_queue = NULL;
        return HOGP_ERR_NO_MEM;
    }

    INFO("Context initialized successfully");
    return HOGP_OK;
}

hogp_result_t hogp_context_shutdown(void) {
    hogp_context_t *ctx = hogp_get_context();
    INFO("Shutting down HOGP context");

    if (ctx->control_queue != NULL) {
        vQueueDelete(ctx->control_queue);
        ctx->control_queue = NULL;
    }
    
    if (ctx->data_queue != NULL) {
        vQueueDelete(ctx->data_queue);
        ctx->data_queue = NULL;
    }

    *ctx = (hogp_context_t) {0};
    
    INFO("Context shutdown complete");
    return HOGP_OK;
}

static bool is_init_info_ok(const hogp_init_info_t *init_info) {
    // Check appearance
    if (init_info->device_data.appearance == HOGP_APPEARANCE_CUSTOM || init_info->device_data.appearance == HOGP_APPEARANCE_KEYBOARD) {
        WARN("The host system may require the mouse appearance to receive mouse messages. If you really need this appearance, be aware that this library only supports mouse messages now");
        return true;
    }
    else if (init_info->device_data.appearance != HOGP_APPEARANCE_MOUSE) {
        ERROR("The appearence has an invalid value: %d", init_info->device_data.appearance);
        return false;
    }
    
    // Check name
    if (strlen(init_info->device_data.device_name) > HOGP_DEVICE_NAME_MAX_CHARACTERS) {
        ERROR("Device name has too many characters: %d (max is %d)", strlen(init_info->device_data.device_name), HOGP_DEVICE_NAME_MAX_CHARACTERS);
        return false;
    }

    return true;
}