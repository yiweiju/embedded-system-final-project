#ifndef HX711_TIVA_H
#define HX711_TIVA_H

#include <stdint.h>
#include "inc/hw_memmap.h"

// Config structure for one HX711 (pins share the same GPIO port)
typedef struct {
    uint32_t port_base;   // GPIO base address for both pins
    uint8_t pin_dout;     // DOUT pin number (0..7)
    uint8_t pin_sck;      // SCK pin number (0..7)
} hx711_cfg_t;

// Instance state for HX711
typedef struct {
    hx711_cfg_t cfg;
    float scale;          // counts per mass unit
    int32_t offset;       // raw offset (tare)
} hx711_t;

// Initialize HX711 instance. No tare is performed here.
void hx711_init(hx711_t *dev, const hx711_cfg_t *cfg);

// Calibration: mass = (raw - offset) / scale
void hx711_set_scale(hx711_t *dev, float scale);
float hx711_get_scale(const hx711_t *dev);
void hx711_set_offset(hx711_t *dev, int32_t offset);

// Polling helpers
int  hx711_data_ready(const hx711_t *dev);   // 1 if DOUT is LOW
// Read raw value with timeout (returns 1 on success, 0 on timeout)
int hx711_read_raw_timeout(hx711_t *dev, int32_t *out, uint32_t timeout_ms);
int32_t hx711_read_raw(hx711_t *dev);        // blocking until ready (legacy, no timeout)
// Get calibrated mass with timeout (returns 1 on success, 0 on timeout)
int hx711_get_mass_timeout(hx711_t *dev, float *out, uint32_t timeout_ms);
float hx711_get_mass(hx711_t *dev);          // blocking until ready (legacy, no timeout)

#endif // HX711_TIVA_H
