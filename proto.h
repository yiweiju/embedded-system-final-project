#ifndef USER_PROTO_H
#define USER_PROTO_H

#include <stdint.h>
#include <stdbool.h>

// Time structure (formerly from rtc_ds3231.h)
typedef struct {
    uint16_t year;  // e.g., 2025
    uint8_t  month; // 1..12
    uint8_t  date;  // 1..31
    uint8_t  weekday; // 1..7
    uint8_t  hour;  // 0..23
    uint8_t  min;   // 0..59
    uint8_t  sec;   // 0..59
} rtc_time_t;

// Public API
void Proto_Init(void);
void Proto_Poll(void);
// 10ms periodic tick (non-blocking actuator scheduling)
void Proto_Tick10ms(void);
// 100ms periodic tick (sensor sampling/filtering)
void Proto_Tick100ms(void);
// 1000ms periodic tick (schedule checking)
void Proto_Tick1000ms(void);

// State accessors (optional for other modules)
typedef struct {
    int bowl_g;
    int water_g;
    int alarm; // 0/bitmask
} StatusSnapshot;

void Proto_GetStatus(StatusSnapshot *out);

// ============================================================================
// Shared Data Structures (Moved here to avoid duplication)
// ============================================================================

// Feeding parameters by angle and step timing (half-step mode)
#define FEED_DEG_L 30u
#define FEED_DEG_M 60u
#define FEED_DEG_H 120u
#define FEED_STEP_DELAY_MS 10u
#define MAX_FEED_STEPS (4096 * 2) // Approximate safe max

// Main State Structure
// Defined here so eeprom_config.c can see the exact layout
typedef struct ProtoState_t {
    int bowl_g;
    int water_g;
    int alarm;
    
    // lastFed/Eaten
    char lastFed_time[6];  // "HH:MM" or "--:--"
    int  lastFed_amount;   // grams
    char lastEaten_time[6];
    int  lastEaten_amount;
    
    // schedule entries
    struct { uint8_t hh, mm; char amount; uint8_t en; } sched[8];
    uint8_t sched_len;
    
    // time sync (ESP32 NTP-based)
    uint32_t unix_base;    // at last sync (timezone already applied)
    uint32_t ms_at_sync;   // ms counter at last sync
    bool time_request_pending;     // waiting for ESP32 time reply
    uint32_t time_request_last_ms; // last time request timestamp (for retry)

    // busy flag for FEED_NOW
    bool busy;

    // feed task state (background stepping)
    uint32_t feed_steps_remaining;
    uint32_t feed_step_delay_ms;
    uint32_t feed_next_step_ms;
    uint32_t feed_deadline_ms;
    int      feed_last_amount_g;

    // schedule tracking
    uint16_t last_sched_minute; // minute of day (0..1439) last checked
    bool sched_init;            // true after first schedule check
} ProtoState;

#endif // USER_PROTO_H
