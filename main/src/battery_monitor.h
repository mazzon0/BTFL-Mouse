#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <stdint.h>

/**
 * @brief Returns the battery level.
 * By connecting the battery voltage to GPIO_1, returns its level as a number from 0 to 100.
 * The result is proportional to the remaining life time of the battery.
 */
uint8_t check_battery(void);

#endif
