#ifndef HOGP_RESULTS_H
#define HOGP_RESULTS_H

/**
 * @brief Return types for HOGP operations.
 * Renamed from hogp_error_t to hogp_result_t to reflect that it can 
 * carry status information beyond just "errors".
 */
typedef enum {
    /* Success */
    HOGP_OK = 0,

    /* Queue & Resource Errors */
    HOGP_ERR_QUEUE_FULL,        /**< Failed to push to a queue (timeout or full) */
    HOGP_ERR_QUEUE_EMPTY,       /**< Attempted to pop from empty queue (replaces NOTHING_TO_SEND, NO_CONTROLS) */
    HOGP_ERR_NO_MEM,            /**< Memory allocation failed (mbuf, etc.) */
    HOGP_ERR_INVALID_ARG,       /**< Invalid argument passed to function */

    /* BLE & Protocol State Errors */
    HOGP_ERR_NOT_CONNECTED,     /**< Operation requires an active connection */
    HOGP_ERR_NOT_SUBSCRIBED,    /**< Host has not subscribed to the necessary characteristic */
    HOGP_ERR_TX_BUSY,           /**< Flow control: Stack is congested, cannot send now (replaces TX_NOT_RECEIVED) */
    HOGP_ERR_TIMEOUT,           /**< Operation timed out */

    /* General Failures */
    HOGP_ERR_NOT_SUPPORTED,     /**< Feature not supported or not implemented yet */
    HOGP_ERR_INTERNAL_FAIL,     /**< Generic internal failure */

} hogp_result_t;

#endif /* HOGP_RESULTS_H */