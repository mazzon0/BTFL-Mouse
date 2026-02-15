# Finite State Machine and Power Modes Management

## Key features
- Configure **low power consumption modes**
- Manage **tmx and pmw_3389** events with callback functions
- Define the **State Machine** structure and implement the state functions
- Implement primary **app_main(void)** function

### Low Power Consumption Modes
- While in WORKING state, `config_low_power_consumption(bool high_performance)` function enables the device to work in high_performance mode, at the maximum frequence of 240MHz. After 5 minutes of inactivity, the device will go in the LOW_POWER_CONSUMPTION state and the function will lower CPU frequency to a maximum of 80MHz, disable Bluetooth and disable the Touch Sensor.
- The `check_inactivity()` function is the one that manages the switching between WORKING and LOW_POWER_CONSUMPTION modes, and that sets the current state to DEEP_SLEEP if the inactive_time exceedes 10 minutes. 
- The `enter_deep_sleep()` function prepares the ESP32 and the PMW3389 sensor for Deep Sleep. This power-saving-mode powers down the CPU and wipes the RAM. GPIO18 is configurd as RTC GPIO, is set as an input, so it can listen for the sensor's signal.

### Finite State Machine
The FSM defines five different operating states:
- START - Configures power management, initializes NVS flash, HOGP component, touch driver and optical sensor and creates the respective tasks. Changes the state to WORKING.
- WORKING - Sends events to the Bluetooth host and checks the inactive_time: if it exceedes 5 minutes, state is changed to LOW_POWER_CONSUMPTION, if it exceedes 10 minutes, the current state is set to DEEP_SLEEP. 
- DEEP_SLEEP - GPIO18 is configured as RTC GPIO for wakeup. CPU and RAM are turned off. On wakeup a System Reset will be triggered. State may change to WORKING or to DEEP_SLEEP, based on check_inactivity() function.
- LOW_POWER_CONSUMPTION - Lowers CPU frequency to a maximum of 80MHz, disable Bluetooth and disable the Touch Sensor. From this state, 
- OFF - ????


### app_main(void) Function
Main function first sets current state variable to START. Then enters and infinite loop, executing the current State-Machine-function.
