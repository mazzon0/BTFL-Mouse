#ifndef HOGP_H
#define HOGP_H

#include "hogp_user_common.h"
#include "hogp_data_events.h"

int hogp_setup(const hogp_init_info_t *const init_info);

int hogp_shutdown(void);

int hogp_send_data(const hogp_data_event_t *event);

#endif
