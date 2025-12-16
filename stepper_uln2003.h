#ifndef STEPPER_ULN2003_H
#define STEPPER_ULN2003_H

#include <stdint.h>

typedef struct {
    uint32_t port_base; // GPIO base for all 4 pins
    uint8_t in1_pin;    // ULN2003 IN1 (A)
    uint8_t in2_pin;    // ULN2003 IN2 (B)
    uint8_t in3_pin;    // ULN2003 IN3 (C)
    uint8_t in4_pin;    // ULN2003 IN4 (D)
} stepper_uln2003_cfg_t;

// Steps per full revolution in half-step mode (typical 28BYJ-48: 4096)
#ifndef STEPPER_HALFSTEP_STEPS_PER_REV
#define STEPPER_HALFSTEP_STEPS_PER_REV 4096
#endif

void stepper_uln2003_init(const stepper_uln2003_cfg_t *cfg);
// direction: +1 forward, -1 reverse. Timing handled by caller (non-blocking).
void stepper_uln2003_step(int direction);
void stepper_uln2003_rotate_steps(uint32_t steps, int direction, uint32_t delay_ms);
void stepper_uln2003_rotate_degrees(float degrees, int direction, uint32_t delay_ms);

// Optional: configure a run indicator LED pin (e.g., PF1 on TM4C LaunchPad).
// When configured, the LED turns on while motor is stepping and turns off after.
void stepper_uln2003_config_run_led(uint32_t port_base, uint8_t pin);

// De-energize all coils (all outputs low)
void stepper_uln2003_all_off(void);

#endif // STEPPER_ULN2003_H
