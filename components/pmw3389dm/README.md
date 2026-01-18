# PMW3389 Optical Sensor Driver for ESP32

A high-performance driver for the PMW3389DM-T3QU optical motion sensor.

## Key Features

- **Manual SPI CS Control** - chip select management for reliable communication
- **Interrupt-Driven Operation** - Hardware interrupt support for power-efficient motion detection
- **Full CPI Range** - Configure sensitivity from 50 to 16,000 CPI (Counts Per Inch)
- **SROM Firmware Upload** - Automated firmware upload via burst mode
- **Motion Tracking** - Real-time X/Y displacement, surface quality, and lift detection
- **Simple API** - High-level functions for quick setup and operation

## Hardware Requirements

- ESP32 (tested with ESP-IDF)
- PMW3389DM-T3QU optical sensor
- SPI connection (MISO, MOSI, SCLK, CS)
- Optional: Motion interrupt pin for efficient operation
- Optional: Reset pin for hardware reset

## Quick Start

```c
#include "pmw3389.h"

// Configure sensor pins
pmw3389_config_t config = {
    .spi_host = SPI2_HOST,
    .pin_miso = GPIO_NUM_37,
    .pin_mosi = GPIO_NUM_35,
    .pin_sclk = GPIO_NUM_36,
    .pin_cs = GPIO_NUM_45,
    .pin_motion = GPIO_NUM_18,        // Optional
    .pin_reset = GPIO_NUM_21,         // Optional
    .spi_clock_speed_hz = 2000000,    // 2MHz max
};

pmw3389_handle_t sensor = NULL;

// Initialize and configure (all-in-one)
pmw3389_init_and_configure(&config, 3200, &sensor);

// Start motion tracking with interrupts
pmw3389_start_motion_tracking_interrupt(sensor, 3200);
```

## API Overview

### High-Level Functions (Recommended)
- `pmw3389_init_and_configure()` - Complete setup in one call
- `pmw3389_start_motion_tracking_interrupt()` - Interrupt-based tracking
- `pmw3389_read_motion()` - Read motion data

### Low-Level Functions
- `pmw3389_init()` - Hardware initialization
- `pmw3389_upload()` - Upload SROM firmware
- `pmw3389_set_cpi()` - Configure sensitivity
- `pmw3389_read_reg()` / `pmw3389_write_reg()` - Direct register access
- `pmw3389_dump_registers()` - Debug diagnostics

## Motion Data Structure

```c
typedef struct {
    int16_t delta_x;        // X-axis displacement (-128 to +127 counts)
    int16_t delta_y;        // Y-axis displacement (-128 to +127 counts)
    uint8_t squal;          // Surface quality (0-255)
    bool motion_detected;   // Motion flag
    bool lift_detected;     // Lift detection flag
} pmw3389_motion_data_t;
```

## Pipeline

1. **Hardware Setup** - Connect PMW3389 to ESP32 via SPI
2. **Initialize Driver** - Configure pins and SPI settings
3. **Upload Firmware** - Load SROM firmware to sensor
4. **Configure CPI** - Set desired sensitivity
5. **Start Tracking** - Begin motion detection (interrupt or polling mode)
6. **Read Motion Data** - Retrieve X/Y movement and surface quality

## Configuration Notes

- **SPI Speed**: Maximum 2MHz for PMW3389
- **CPI Range**: 50-16,000 in increments of 50
- **8-bit Deltas**: Provides -128 to +127 count range per read
- **Manual CS**: CS pin is GPIO-controlled for compatibility

## Files

- `pmw3389.h` / `pmw3389.c` - Main driver implementation
- `pmw3389_srom.h` / `pmw3389_srom.c` - Sensor firmware
- `main.c` - Example usage

## Author

Ilaria
