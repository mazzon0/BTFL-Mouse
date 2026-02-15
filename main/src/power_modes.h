#ifndef POWER_MODES_H
#define POWER_MODES_H

#include <stdbool.h>

/**
 * @brief Configure Low Power Mode
 * 
 * @param bool high_performance
 * @return void
 * @details The function is called after 5 minutes of inactivity. It lowers the
 * CPU frequency and disables bluetooth and touch features.
 */
void config_low_power_consumption(bool high_performance);

/**
 * @brief Enter Deep Sleep mode
 * 
 * @param void
 * 
 * @return void
 * 
 * @details This function prepares the ESP32 and the PMW3389 sensor for Deep Sleep.
 * This power-saving mode turns off both the CPU and RAM. When the device is
 * woke up it starts the program execution from the very beginning, so it behaves as a reset.
 * GPIO18 is configurd as RTC GPIO, is set as an input, so it can listen for the
 * sensor's signal, and its internal resistors are disabled - to leave the pin in a
 * high-impedance state (floating), preventing leakage  currents that could drain
 * the battery during deep sleep. The pin is then configured as
 * a wake-up source.
 * A 100ms delay is set to ensure all SPI communcations are closed and to alow the
 * electrical signals on the PCB to stabilize, so that there won't be any "spurious" wakeups.
 * The ESP32 enters Deep Sleep: the CPU powers down and the RAM is wiped. On wake up, a System
 * Reset will be triggered.
 */
void enter_deep_sleep(void);

#endif
