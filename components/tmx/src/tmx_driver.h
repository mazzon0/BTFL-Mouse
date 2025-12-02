#ifndef TMX_DRIVER_H
#define TMX_DRIVER_H

#include "tmx_common.h"

esp_err_t tmx_driver_init(void);

void tmx_driver_read_raw(void);

uint32_t* tmx_driver_get_raw_data(void);

#endif
