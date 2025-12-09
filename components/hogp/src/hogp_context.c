#include "hogp_context.h"
#include "hogp_common.h"
#include "hogp_data_events.h"
#include "hogp_control_events.h"
#include <string.h>

int hogp_context_init(const hogp_init_info_t *init_info) {
    int rc = 0;
    hogp_context_t *ctx = hogp_get_context();
    
    // TODO check init_info and also check device name length

    ctx->connection = (hogp_conn_t) {0};
    ctx->connection.mtu = 23;
    ctx->connection.tx_arrived = true;
    ctx->connection.protocol = HOGP_PROTOCOL_REPORT;
    ctx->device.appearance = init_info->device_data.appearance;
    strcpy(ctx->device.device_name, init_info->device_data.device_name);
    ctx->state = HOGP_STATE_IDLE;
    ctx->update_period_ms = init_info->update_period_ms;

    ctx->control_queue = xQueueCreate(16, sizeof(hogp_control_event_t));
    ctx->data_queue = xQueueCreate(32, sizeof(hogp_data_event_t));

    return rc;
}

int hogp_context_shutdown(void) {
    int rc = 0;
    hogp_context_t *ctx = hogp_get_context();

    vQueueDelete(ctx->control_queue);
    vQueueDelete(ctx->data_queue);

    *ctx = (hogp_context_t) {0};
    return rc;
}
