#include "hogp_common.h"

hogp_context_t *hogp_get_context(void) {
    static hogp_context_t context = {0};
    return &context;
}