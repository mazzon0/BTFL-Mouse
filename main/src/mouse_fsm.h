#ifndef MOUSE_FSM_H
#define MOUSE_FSM_H

/**
 * Define states and state machine structure
 */
typedef enum {
    START,
    WORKING,
    DEEP_SLEEP,
    LOW_POWER_CONSUMPTION,
    OFF,
    NUM_STATES
} State_t;

/**
 * State and its associated function
 */
typedef struct {
    State_t state;
    void (*func) (void);
} StateMachine_t;

/* State machine function prototypes */
void fn_START(void);
void fn_WORKING(void);
void fn_DEEP_SLEEP(void);
void fn_LOW_POWER_CONSUMPTION(void);
void fn_OFF(void);

#endif
