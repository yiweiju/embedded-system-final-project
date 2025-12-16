#ifndef EEPROM_CONFIG_H
#define EEPROM_CONFIG_H

#include <stdint.h>
#include <stdbool.h>
#include "hx711_tiva.h"

// Forward declaration - matches the typedef in proto.h
typedef struct ProtoState_t ProtoState;

// ============================================================================
// EEPROM Address Layout
// ============================================================================

#define EEPROM_ADDR_CALIBRATION     0x0000  // HX711 calibration data (28 bytes)
#define EEPROM_ADDR_SCHEDULE        0x001C  // Feeding schedule (40 bytes)
#define EEPROM_ADDR_HISTORY         0x0044  // History records (64 bytes, reserved)
#define EEPROM_ADDR_FUTURE          0x0084  // Future expansion

// ============================================================================
// Magic Numbers
// ============================================================================

#define EEPROM_MAGIC_CALIBRATION    0x48583731  // "HX71"
#define EEPROM_MAGIC_SCHEDULE       0x53434844  // "SCHD"

// ============================================================================
// Data Structures
// ============================================================================

// HX711 Calibration Data (28 bytes, 32-bit aligned)
typedef struct {
    uint32_t magic;           // Magic number: 0x48583731 "HX71"
    float food_scale;         // Food sensor scale (counts per gram)
    int32_t food_offset;      // Food sensor offset (tare value)
    float water_scale;        // Water sensor scale (counts per gram)
    int32_t water_offset;     // Water sensor offset (tare value)
    uint32_t crc32;           // CRC32 checksum
    uint32_t reserved;        // Reserved for alignment (must be 0)
} eeprom_calibration_t;

// Feeding Schedule Data (40 bytes, 32-bit aligned)
typedef struct {
    uint32_t magic;           // Magic number: 0x53434844 "SCHD"
    uint32_t sched_len;       // Number of schedule entries (0-8)
    struct {
        uint8_t hh;           // Hour (0-23)
        uint8_t mm;           // Minute (0-59)
        char amount;          // Amount: 'L', 'M', or 'H'
        uint8_t en;           // Enabled: 0 or 1
    } sched[8];               // Up to 8 schedule entries
    uint32_t crc32;           // CRC32 checksum
} eeprom_schedule_t;

// ============================================================================
// API Functions
// ============================================================================

/**
 * Initialize EEPROM module
 * Must be called once during system initialization before any EEPROM operations
 *
 * @return true if initialization successful, false otherwise
 */
bool eeprom_config_init(void);

/**
 * Load HX711 calibration data from EEPROM
 * If data is invalid or corrupted, sensors will keep their current values
 *
 * @param food  Pointer to food sensor structure to load into
 * @param water Pointer to water sensor structure to load into
 * @return true if data loaded successfully, false if data invalid/corrupted
 */
bool eeprom_load_calibration(hx711_t *food, hx711_t *water);

/**
 * Save HX711 calibration data to EEPROM
 *
 * @param food  Pointer to food sensor structure to save
 * @param water Pointer to water sensor structure to save
 * @return true if save successful, false otherwise
 */
bool eeprom_save_calibration(const hx711_t *food, const hx711_t *water);

/**
 * Load feeding schedule from EEPROM
 * If data is invalid or corrupted, schedule will remain unchanged
 *
 * @param st Pointer to ProtoState structure containing schedule arrays
 * @return true if data loaded successfully, false if data invalid/corrupted
 */
bool eeprom_load_schedule(ProtoState *st);

/**
 * Save feeding schedule to EEPROM
 *
 * @param st Pointer to ProtoState structure containing schedule arrays
 * @return true if save successful, false otherwise
 */
bool eeprom_save_schedule(const ProtoState *st);

/**
 * Format EEPROM by erasing all configuration data
 * This will reset all stored data to defaults
 *
 * @return true if format successful, false otherwise
 */
bool eeprom_format(void);

/**
 * Check integrity of all EEPROM configuration data
 *
 * @return true if all data valid, false if any corruption detected
 */
bool eeprom_check_integrity(void);

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * Calculate CRC32 checksum for data buffer
 * Uses standard CRC32 polynomial (0x04C11DB7)
 *
 * @param data Pointer to data buffer
 * @param len  Length of data in bytes
 * @return CRC32 checksum value
 */
uint32_t eeprom_calculate_crc32(const uint8_t *data, uint32_t len);

#endif // EEPROM_CONFIG_H
