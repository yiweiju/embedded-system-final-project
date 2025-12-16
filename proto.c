#include "proto.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "uart.h"
#include "hx711_tiva.h"
#include "stepper_uln2003.h"
#include "eeprom_config.h"

// Line buffer for incoming JSON (terminated by \n)
#define RX_LINE_MAX 256
static char rx_line[RX_LINE_MAX];
static uint32_t rx_len = 0;

// GLOBAL STATE
static ProtoState S;

// HX711 devices
static hx711_t g_hx_food, g_hx_water;
static const hx711_cfg_t g_hx_food_cfg = { GPIO_PORTE_BASE, 2, 3 };
static const hx711_cfg_t g_hx_water_cfg = { GPIO_PORTE_BASE, 4, 5 };

// Forward decls
static void handle_at_command(const char *line);
static void send_ok_data(const char *data);
static void send_ok(void);
static void ack_err(uint32_t seq, const char *err_code);
static void cmd_at_status(void);
static void cmd_at_feed(const char *param);
static void cmd_at_log(void);
static void cmd_at_tare(const char *param);
static void cmd_at_settime(const char *param);
static void cmd_at_schedule(const char *param);
static void cmd_at_get_schedule(void);
static void cmd_at_calibrate(const char *param);
static void cmd_at_eeprom_diag(void);
static bool eeprom_init_with_retry(void);

extern uint32_t millis(void);
static void format_HHMM(uint32_t unix_sec, char out[6]);
static uint32_t now_unix(void);
static int level_to_grams(const char *level);
static inline uint32_t deg_to_steps(uint32_t deg);
static bool is_amount_LMH(const char *s);

// Time utility functions (formerly from rtc_ds3231.c)
static bool is_leap_year(uint32_t year);
static uint8_t calculate_weekday(uint32_t year, uint8_t month, uint8_t day);
uint32_t rtc_time_to_unix(const rtc_time_t *t);
bool rtc_unix_to_time(uint32_t unix_sec, rtc_time_t *t);

// ============================================================================
// Initialization & Main Loop
// ============================================================================

void Proto_Init(void)
{
    memset(&S, 0, sizeof(S));
    // Default placeholders
    strcpy(S.lastFed_time, "--:--");
    strcpy(S.lastEaten_time, "--:--");
    S.lastFed_amount = 0;
    S.lastEaten_amount = 0;
    S.unix_base = 0;
    S.ms_at_sync = millis();
    S.busy = false;
    S.feed_step_delay_ms = FEED_STEP_DELAY_MS;
    S.sched_len = 0;
    S.sched_init = false;

    // Request time from ESP32 on boot
    UARTprintf("AT+GETTIME\r\n");
    S.time_request_pending = true;
    S.time_request_last_ms = millis();

    hx711_init(&g_hx_food, &g_hx_food_cfg);
    hx711_init(&g_hx_water, &g_hx_water_cfg);

    // Configure PE1 as output for water pump
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);  // Start with pump OFF

    if (eeprom_init_with_retry()) {
        eeprom_load_calibration(&g_hx_food, &g_hx_water);
        eeprom_load_schedule(&S);
    }
}

void Proto_Poll(void) {
    char ch;
    static bool overflow_skip = false;
    while (UART0_ReadChar(&ch)) {
        if (ch == '\r') continue;
        if (ch == '\n') {
            if (overflow_skip) {
                overflow_skip = false;
                rx_len = 0;
            } else {
                rx_line[rx_len] = '\0';
                if (rx_len > 0) {
                    // Optional guard to drop stray noise before "AT+"
                    char *cmd_start = strstr(rx_line, "AT+");
                    if (cmd_start) handle_at_command(cmd_start);
                }
                rx_len = 0;
            }
        } else {
            if (overflow_skip) continue;
            if (rx_len < RX_LINE_MAX - 1) {
                rx_line[rx_len++] = ch;
            } else {
                overflow_skip = true;
                rx_len = 0;
            }
        }
    }
}

void Proto_GetStatus(StatusSnapshot *out) {
    if (!out) return;
    out->bowl_g = S.bowl_g;
    out->water_g = S.water_g;
    out->alarm = S.alarm;
}

// ============================================================================
// Ticks
// ============================================================================

void Proto_Tick10ms(void) {
    if (S.feed_steps_remaining > MAX_FEED_STEPS) {
        S.feed_steps_remaining = 0;
        S.busy = false;
        stepper_uln2003_all_off();
    }
    if (S.busy && S.feed_steps_remaining > 0) {
        uint32_t nowms = millis();
        // Check timeout/deadline
        if ((int32_t)(nowms - S.feed_deadline_ms) >= 0) S.feed_steps_remaining = 0;
        
        // Time to step?
        if (S.feed_steps_remaining > 0 && (int32_t)(nowms - S.feed_next_step_ms) >= 0) {
            stepper_uln2003_step(+1);
            S.feed_steps_remaining--;
            S.feed_next_step_ms = nowms + S.feed_step_delay_ms;
        }
        
        // Finished?
        if (S.feed_steps_remaining == 0) {
            S.busy = false;
            if (S.unix_base > 0) {
                uint32_t now = now_unix();
                format_HHMM(now, S.lastFed_time);
            }
            S.lastFed_amount = S.feed_last_amount_g;
            stepper_uln2003_all_off();
        }
    }
    if (!S.busy && S.feed_steps_remaining == 0) stepper_uln2003_all_off();
}

void Proto_Tick100ms(void) {
    if (hx711_data_ready(&g_hx_food)) {
        float mass = 0.0f;
        if (hx711_get_mass_timeout(&g_hx_food, &mass, 100)) S.bowl_g = (int)(mass + 0.5f);
    }
    if (hx711_data_ready(&g_hx_water)) {
        float mass = 0.0f;
        if (hx711_get_mass_timeout(&g_hx_water, &mass, 100)) {
            S.water_g = (int)(mass + 0.5f);

            // Water pump control: activate if below 80g
            if (S.water_g < 80) {
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1);  // Pump ON
            } else {
                GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0);  // Pump OFF
            }
        }
    }
}

void Proto_Tick1000ms(void) {
    // Retry time request every 60 seconds if pending
    if (S.time_request_pending) {
        if (millis() - S.time_request_last_ms >= 60000) {
            UARTprintf("AT+GETTIME\r\n");
            S.time_request_last_ms = millis();
        }
    }

    if (S.unix_base == 0) return;

    // Calculate current time (timezone already applied by ESP32)
    uint32_t now = now_unix();
    uint32_t sec_in_day = now % 86400;
    uint16_t current_hh = (uint16_t)(sec_in_day / 3600u);
    uint16_t current_mm = (uint16_t)((sec_in_day % 3600u) / 60u);
    uint16_t current_minute = current_hh * 60u + current_mm;

    if (!S.sched_init) {
        S.last_sched_minute = current_minute;
        S.sched_init = true;
        return;
    }
    if (current_minute == S.last_sched_minute) return;
    S.last_sched_minute = current_minute;

    for (uint8_t i = 0; i < S.sched_len; i++) {
        uint16_t sched_minute = (uint16_t)S.sched[i].hh * 60u + (uint16_t)S.sched[i].mm;
        if (sched_minute == current_minute) {
            if (!S.busy) {
                char level = S.sched[i].amount;
                uint32_t degrees = (level == 'L') ? FEED_DEG_L : (level == 'M') ? FEED_DEG_M : FEED_DEG_H;
                uint32_t steps = deg_to_steps(degrees);
                S.feed_steps_remaining = steps;
                S.feed_step_delay_ms = FEED_STEP_DELAY_MS;
                S.feed_next_step_ms = millis();
                S.feed_deadline_ms = millis() + steps * S.feed_step_delay_ms + 1000u;
                S.feed_last_amount_g = level_to_grams(&level);
                S.busy = true;
                break;
            }
        }
    }
}

// ============================================================================
// Command Handlers
// ============================================================================

static void handle_at_command(const char *line) {
    if (strncmp(line, "AT+", 3) != 0) { ack_err(0, "SYNTAX"); return; }
    const char *cmd = line + 3;
    const char *eq = strchr(cmd, '=');
    
    if (strncmp(cmd, "STATUS", 6) == 0) cmd_at_status();
    else if (strncmp(cmd, "FEED=", 5) == 0 && eq) cmd_at_feed(eq + 1);
    else if (strncmp(cmd, "LOG", 3) == 0) cmd_at_log();
    else if (strncmp(cmd, "TARE=", 5) == 0 && eq) cmd_at_tare(eq + 1);
    else if (strncmp(cmd, "CAL=", 4) == 0 && eq) cmd_at_calibrate(eq + 1);
    else if (strncmp(cmd, "SETTIME=", 8) == 0 && eq) cmd_at_settime(eq + 1);
    else if (strncmp(cmd, "SCHED=", 6) == 0 && eq) cmd_at_schedule(eq + 1);
    else if (strncmp(cmd, "GETSCHED", 8) == 0) cmd_at_get_schedule();
    else if (strncmp(cmd, "EEDIAG", 6) == 0) cmd_at_eeprom_diag();
    else ack_err(0, "UNKNOWN_CMD");
}

static void send_ok_data(const char *data) { UARTprintf("+OK: %s\r\n", data); }
static void send_ok(void) { UARTprintf("+OK\r\n"); }
static void ack_err(uint32_t seq, const char *err) { (void)seq; UARTprintf("+ERR: %s\r\n", err); }

static void cmd_at_status(void) {
    // Get current Unix timestamp
    uint32_t now = now_unix();

    // Convert to date/time components
    rtc_time_t t;
    rtc_unix_to_time(now, &t);

    // Format time string: YYYY-MM-DD HH:MM:SS
    char time_str[20];
    snprintf(time_str, sizeof(time_str), "%04d-%02d-%02d %02d:%02d:%02d",
             t.year, t.month, t.date, t.hour, t.min, t.sec);

    // Build status response with time first
    char buf[128];
    snprintf(buf, sizeof(buf), "TIME=%s,BOWL=%d,WATER=%d,ALARM=%d,BUSY=%d",
             time_str, S.bowl_g, S.water_g, S.alarm, S.busy);
    send_ok_data(buf);
}

static void cmd_at_feed(const char *param) {
    if (S.busy) { ack_err(0, "BUSY"); return; }
    char level = param[0];
    if (!is_amount_LMH(param)) { ack_err(0, "PARAM_ERR"); return; }
    
    uint32_t degrees = (level=='L')?FEED_DEG_L:(level=='M')?FEED_DEG_M:FEED_DEG_H;
    S.feed_steps_remaining = deg_to_steps(degrees);
    S.feed_step_delay_ms = FEED_STEP_DELAY_MS;
    S.feed_next_step_ms = millis();
    S.feed_deadline_ms = millis() + S.feed_steps_remaining * 10 + 1000;
    S.feed_last_amount_g = level_to_grams(&level);
    S.busy = true;
    send_ok();
}

static void cmd_at_log(void) {
    char buf[128];
    snprintf(buf, sizeof(buf), "FED_TIME=%s,FED_AMT=%d,EAT_TIME=%s,EAT_AMT=%d", 
        S.lastFed_time, S.lastFed_amount, S.lastEaten_time, S.lastEaten_amount);
    send_ok_data(buf);
}

static void cmd_at_tare(const char *param) {
    hx711_t *dev = (strncmp(param,"FOOD",4)==0) ? &g_hx_food : (strncmp(param,"WATER",5)==0) ? &g_hx_water : NULL;
    if (!dev) { ack_err(0, "PARAM_ERR"); return; }
    int32_t raw;
    if (!hx711_read_raw_timeout(dev, &raw, 500)) { ack_err(0, "TIMEOUT"); return; }
    hx711_set_offset(dev, raw);
    eeprom_save_calibration(&g_hx_food, &g_hx_water);
    send_ok();
}

static void cmd_at_calibrate(const char *param) {
    // Expected: SENSOR,WEIGHT
    char sensor[10]; const char *comma = strchr(param, ',');
    if (!comma) { ack_err(0, "PARAM_ERR"); return; }
    int len = comma - param;
    if (len >= 10) len=9;
    memcpy(sensor, param, len); sensor[len]=0;
    int weight = atoi(comma+1);
    if (weight <= 0) { ack_err(0, "PARAM_ERR"); return; }
    
    hx711_t *dev = (strcmp(sensor,"FOOD")==0) ? &g_hx_food : (strcmp(sensor,"WATER")==0) ? &g_hx_water : NULL;
    if (!dev) { ack_err(0, "PARAM_ERR"); return; }
    int32_t raw;
    if (!hx711_read_raw_timeout(dev, &raw, 500)) { ack_err(0, "TIMEOUT"); return; }
    float new_scale = (float)(raw - dev->offset) / (float)weight;
    if (new_scale <= 0) { ack_err(0, "CAL_ERR"); return; }
    hx711_set_scale(dev, new_scale);
    eeprom_save_calibration(&g_hx_food, &g_hx_water);
    send_ok();
}

static void cmd_at_settime(const char *param) {
    // Parse: AT+SETTIME=1733472000 (timezone already applied by ESP32)
    uint32_t timestamp = strtoul(param, NULL, 10);

    if (timestamp == 0) {
        ack_err(0, "INVALID_TIMESTAMP");
        return;
    }

    // Update time base
    S.unix_base = timestamp;
    S.ms_at_sync = millis();
    S.time_request_pending = false;  // Cancel any pending requests

    send_ok();
}

static void cmd_at_schedule(const char *param) {
    if (!param || strlen(param) == 0) {
        ack_err(0, "PARAM_ERR");
        return;
    }
    if (strcmp(param, "NONE") == 0) {
        S.sched_len = 0;
        memset(S.sched, 0, sizeof(S.sched));
        eeprom_save_schedule(&S);
        send_ok();
        return;
    }

    // 1. Copy to local buffer because strtok modifies the string
    char buf[256];
    strncpy(buf, param, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    // 2. Parse into temporary array
    struct { uint8_t hh, mm; char amount; uint8_t en; } temp_sched[8];
    uint8_t temp_len = 0;

    // Use strtok to split by semicolon ';'
    char *token = strtok(buf, ";");
    uint8_t iteration_count = 0;
    while (token != NULL && iteration_count < 16) {  // Add safety limit
        iteration_count++;
        if (temp_len >= 8) break; // Max 8 entries

        // Expected new format: "HHMMA" (e.g., "0700M"); fall back to legacy "HH:MM,A,E"
        bool parsed = false;
        uint8_t hh = 0, mm = 0;
        char amt = 0;

        size_t len = strlen(token);
        if (len >= 5) {
            amt = token[len - 1];
            if (amt == 'L' || amt == 'M' || amt == 'H') {
                // Collect exactly 4 digits from the time portion (ignore ':' or other separators)
                char digits[5] = {0};
                uint8_t dcount = 0;
                for (size_t k = 0; k < len - 1 && dcount < 4; k++) {
                    if (token[k] >= '0' && token[k] <= '9') {
                        digits[dcount++] = token[k];
                    }
                }
                if (dcount == 4) {
                    hh = (uint8_t)((digits[0] - '0') * 10 + (digits[1] - '0'));
                    mm = (uint8_t)((digits[2] - '0') * 10 + (digits[3] - '0'));
                    parsed = true;
                }
            }
        }

        // Legacy format support: "HH:MM,A,E"
        if (!parsed) {
            char *colon = strchr(token, ':');
            char *comma1 = colon ? strchr(colon, ',') : NULL;
            if (comma1) {
                hh = (uint8_t)atoi(token);
                mm = (uint8_t)atoi(colon + 1);
                amt = *(comma1 + 1);
                parsed = (amt == 'L' || amt == 'M' || amt == 'H');
            }
        }

        bool time_ok = parsed && (hh <= 23) && (mm <= 59);

        if (time_ok) {
            temp_sched[temp_len].hh = (uint8_t)hh;
            temp_sched[temp_len].mm = (uint8_t)mm;
            temp_sched[temp_len].amount = amt;
            temp_sched[temp_len].en = 1; // Enable flag removed from protocol; always on
            temp_len++;
        }
        token = strtok(NULL, ";");
    }

    // 3. Update Global State
    // FIXED: Use member-wise assignment to avoid compiler error #513 with anonymous structs
    S.sched_len = temp_len;
    for (int i = 0; i < temp_len; i++) {
        S.sched[i].hh = temp_sched[i].hh;
        S.sched[i].mm = temp_sched[i].mm;
        S.sched[i].amount = temp_sched[i].amount;
        S.sched[i].en = temp_sched[i].en;
    }

    // 4. Save to EEPROM for persistence
    eeprom_save_schedule(&S);

    send_ok();
}

static void cmd_at_get_schedule(void) {
    char buf[256]; 
    size_t off=0;
    buf[0] = '\0';

    if (S.sched_len == 0) {
        send_ok_data("NONE");
        return;
    }
    
    for(int i=0; i<S.sched_len; i++) {
        if(i > 0) {
            if (off < sizeof(buf) - 1) buf[off++] = ';';
        }
        int remaining = sizeof(buf) - off;
        if (remaining > 0) {
            off += snprintf(buf+off, remaining, "%02d%02d%c", 
                            S.sched[i].hh, S.sched[i].mm, S.sched[i].amount);
        }
    }
    // Ensure null termination just in case
    if (off < sizeof(buf)) buf[off] = 0;
    else buf[sizeof(buf)-1] = 0;
    
    send_ok_data(buf);
}

static void cmd_at_eeprom_diag(void) {
    bool ok = eeprom_check_integrity();
    send_ok_data(ok ? "PASS" : "FAIL");
}

// Some boards occasionally fail EEPROM init on first boot; retry a few times.
static bool eeprom_init_with_retry(void) {
    for (int i = 0; i < 3; i++) {
        if (eeprom_config_init()) return true;
        uint32_t target = millis() + 10u; // short backoff
        while ((int32_t)(millis() - target) < 0) { /* spin */ }
    }
    return false;
}

// ============================================================================
// Helpers
// ============================================================================

static inline uint32_t deg_to_steps(uint32_t deg)
{
    const uint32_t steps_per_rev = (uint32_t)STEPPER_HALFSTEP_STEPS_PER_REV;
    uint64_t num = (uint64_t)deg * (uint64_t)steps_per_rev + 180u;
    return (uint32_t)(num / 360u);
}

static bool is_amount_LMH(const char *s) {
    if (!s || strlen(s) < 1) return false;
    return (s[0]=='L'||s[0]=='M'||s[0]=='H');
}

static int level_to_grams(const char *level) {
    if (!level) return 0;
    switch (level[0]) {
        case 'L': return 10;
        case 'M': return 25;
        case 'H': return 40;
        default: return 0;
    }
}

static uint32_t now_unix(void) {
    uint32_t elapsed_ms = millis() - S.ms_at_sync;
    return S.unix_base + (elapsed_ms / 1000u);
}

static void format_HHMM(uint32_t unix_sec, char out[6]) {
    uint32_t sec_in_day = unix_sec % 86400u;
    uint32_t hh = (sec_in_day / 3600u) % 24u;
    uint32_t mm = (sec_in_day % 3600u) / 60u;
    snprintf(out, 6, "%02u:%02u", hh, mm);
}

// ============================================================================
// Time Utility Functions (formerly from rtc_ds3231.c)
// ============================================================================

static bool is_leap_year(uint32_t year)
{
    return (year % 4 == 0 && year % 100 != 0) || (year % 400 == 0);
}

static uint8_t calculate_weekday(uint32_t year, uint8_t month, uint8_t day)
{
    // Zeller's congruence for Gregorian calendar
    // Returns 1=Sunday, 2=Monday, ..., 7=Saturday
    if (month < 3) {
        month += 12;
        year--;
    }
    uint32_t K = year % 100;
    uint32_t J = year / 100;
    uint32_t h = (day + (13 * (month + 1)) / 5 + K + K / 4 + J / 4 - 2 * J) % 7;
    // Zeller gives 0=Saturday, 1=Sunday, ..., 6=Friday
    // Convert to 1=Sunday, 2=Monday, ..., 7=Saturday
    uint8_t weekday = (uint8_t)((h + 6) % 7 + 1);
    return weekday;
}

uint32_t rtc_time_to_unix(const rtc_time_t *t)
{
    if (!t) return 0;
    if (t->year < 1970 || t->year > 2099) return 0;
    if (t->month < 1 || t->month > 12) return 0;
    if (t->date < 1 || t->date > 31) return 0;
    if (t->hour > 23 || t->min > 59 || t->sec > 59) return 0;

    // Days in each month (non-leap year)
    const uint8_t days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // Count days from 1970-01-01 to target date
    uint32_t days = 0;

    // Add days for complete years from 1970 to (year-1)
    for (uint32_t y = 1970; y < t->year; y++) {
        days += is_leap_year(y) ? 366 : 365;
    }

    // Add days for complete months in current year
    for (uint8_t m = 1; m < t->month; m++) {
        if (m == 2 && is_leap_year(t->year)) {
            days += 29;
        } else {
            days += days_in_month[m - 1];
        }
    }

    // Add days in current month
    days += t->date - 1;

    // Convert to seconds
    uint32_t unix_time = days * 86400UL;
    unix_time += (uint32_t)t->hour * 3600UL;
    unix_time += (uint32_t)t->min * 60UL;
    unix_time += (uint32_t)t->sec;

    return unix_time;
}

bool rtc_unix_to_time(uint32_t unix_sec, rtc_time_t *t)
{
    if (!t) return false;

    // Constants
    const uint32_t SECONDS_PER_DAY = 86400UL;
    const uint8_t days_in_month[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    // Calculate days since Unix epoch and seconds within day
    uint32_t days = unix_sec / SECONDS_PER_DAY;
    uint32_t sec_in_day = unix_sec % SECONDS_PER_DAY;

    // Find year
    uint32_t year = 1970;
    uint32_t days_counted = 0;

    while (1) {
        uint32_t days_in_year = is_leap_year(year) ? 366 : 365;
        if (days_counted + days_in_year > days) {
            break; // Found the year
        }
        days_counted += days_in_year;
        year++;
    }

    // Days remaining in current year
    uint32_t day_of_year = days - days_counted;

    // Find month and day
    uint8_t month = 1;
    uint8_t feb_days = is_leap_year(year) ? 29 : 28;

    for (month = 1; month <= 12; month++) {
        uint8_t days_this_month = (month == 2) ? feb_days : days_in_month[month - 1];
        if (day_of_year < days_this_month) {
            break;
        }
        day_of_year -= days_this_month;
    }

    uint8_t day = (uint8_t)(day_of_year + 1);

    // Extract time of day
    t->year = (uint16_t)year;
    t->month = month;
    t->date = day;
    t->hour = (uint8_t)(sec_in_day / 3600);
    t->min = (uint8_t)((sec_in_day % 3600) / 60);
    t->sec = (uint8_t)(sec_in_day % 60);

    // Calculate weekday
    t->weekday = calculate_weekday(year, month, day);

    return true;
}
