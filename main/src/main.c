#include "mouse_fsm.h"

State_t cur_state;
StateMachine_t StateMachine[] = {
    {START, fn_START},
    {WORKING, fn_WORKING},
    {DEEP_SLEEP, fn_DEEP_SLEEP},
    {LOW_POWER_CONSUMPTION, fn_LOW_POWER_CONSUMPTION},
    {OFF, fn_OFF}
};

void app_main(void) {
    cur_state = START;
    while(1) {
        if(cur_state < NUM_STATES) {
            (*StateMachine[cur_state].func) ();
        } else {
            /* error */
        }
    }
}
