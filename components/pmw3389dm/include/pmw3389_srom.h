/**
 * @file pmw3389_srom.h
 * @brief PMW3389 SROM Firmware Header
 * 
 * This file contains the declaration of the PMW3389 optical sensor firmware
 * that must be uploaded to the sensor's SROM during initialization.
 * 
 * Firmware: ORCA3_ROW_SROMxE8
 * Source: PMW3389DM-T3QU Official Firmware
 * 
 * @author Ilaria
 * @date 2025-12-12
 */

#ifndef PMW3389_SROM_H
#define PMW3389_SROM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SROM firmware length in bytes
 */
#define PMW3389_SROM_LENGTH 4094

/**
 * @brief SROM firmware data array
 * 
 * This array contains the complete firmware that must be uploaded
 * to the PMW3389 sensor during initialization.
 */
extern const uint8_t pmw3389_srom_data[PMW3389_SROM_LENGTH];

#ifdef __cplusplus
}
#endif

#endif // PMW3389_SROM_H