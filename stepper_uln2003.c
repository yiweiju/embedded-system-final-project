#include "stepper_uln2003.h"
#include <stdint.h>
#include <stdbool.h>

// Use TivaWare driverlib for consistency with other modules
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

extern uint32_t SystemCoreClock;

static stepper_uln2003_cfg_t s_cfg;
static uint8_t s_step_idx = 0; // 0..7 for half-step sequence
static uint32_t s_led_base = 0; static uint8_t s_led_pin = 0; static int s_led_en = 0;

static void delay_us(uint32_t us)
{
    // Fixed precision: use integer arithmetic with rounding (same as hx711_tiva.c)
    uint32_t clk = (SystemCoreClock ? SystemCoreClock : 16000000u);
    uint32_t cycles_per_us = (clk + 2000000u) / 4000000u; // Add half divisor for rounding
    if (cycles_per_us == 0) cycles_per_us = 1;
    volatile uint32_t total = us * cycles_per_us;
    while (total--) { __asm volatile ("nop"); }
}

static void delay_ms(uint32_t ms)
{ while (ms--) delay_us(1000u); }

// Enable GPIO port clock using TivaWare API
static void enable_gpio_port(uint32_t base)
{
    uint32_t periph = 0;
    switch (base) {
        case GPIO_PORTA_BASE: periph = SYSCTL_PERIPH_GPIOA; break;
        case GPIO_PORTB_BASE: periph = SYSCTL_PERIPH_GPIOB; break;
        case GPIO_PORTC_BASE: periph = SYSCTL_PERIPH_GPIOC; break;
        case GPIO_PORTD_BASE: periph = SYSCTL_PERIPH_GPIOD; break;
        case GPIO_PORTE_BASE: periph = SYSCTL_PERIPH_GPIOE; break;
        case GPIO_PORTF_BASE: periph = SYSCTL_PERIPH_GPIOF; break;
        default: periph = 0; break;
    }
    if (periph) {
        SysCtlPeripheralEnable(periph);
        while (!SysCtlPeripheralReady(periph)) {}
    }
}

// Write single pin using TivaWare GPIO API
static inline void pin_out(uint32_t base, uint8_t pin, bool high)
{
    uint8_t mask = (uint8_t)(1u << pin);
    GPIOPinWrite(base, mask, high ? mask : 0);
}

// Optional LED indicator output
static inline void led_out(int on)
{
    if (!s_led_en) return;
    uint8_t mask = (uint8_t)(1u << s_led_pin);
    GPIOPinWrite(s_led_base, mask, on ? mask : 0);
}

void stepper_uln2003_init(const stepper_uln2003_cfg_t *cfg)
{
    s_cfg = *cfg;
    enable_gpio_port(s_cfg.port_base);

    // Configure all 4 pins as digital outputs using TivaWare API
    uint8_t mask = (uint8_t)((1u << s_cfg.in1_pin) | (1u << s_cfg.in2_pin) |
                              (1u << s_cfg.in3_pin) | (1u << s_cfg.in4_pin));

    GPIOPinTypeGPIOOutput(s_cfg.port_base, mask);
    // Set all pins low initially
    GPIOPinWrite(s_cfg.port_base, mask, 0);

    s_step_idx = 0;
}

// Half-step sequence (8 states), order IN1,IN2,IN3,IN4
static const uint8_t seq8[8] = {
    0b1000,
    0b1100,
    0b0100,
    0b0110,
    0b0010,
    0b0011,
    0b0001,
    0b1001,
};

void stepper_uln2003_step(int direction)
{
    if (direction >= 0) {
        s_step_idx = (uint8_t)((s_step_idx + 1) & 7u);
    } else {
        s_step_idx = (uint8_t)((s_step_idx + 7) & 7u);
    }
    uint8_t pat = seq8[s_step_idx];
    pin_out(s_cfg.port_base, s_cfg.in1_pin, (pat >> 3) & 1u);
    pin_out(s_cfg.port_base, s_cfg.in2_pin, (pat >> 2) & 1u);
    pin_out(s_cfg.port_base, s_cfg.in3_pin, (pat >> 1) & 1u);
    pin_out(s_cfg.port_base, s_cfg.in4_pin, (pat >> 0) & 1u);
    // No delay - timing handled by caller (Proto_Tick10ms)
}

void stepper_uln2003_rotate_steps(uint32_t steps, int direction, uint32_t delay_ms_val)
{
    if (steps) led_out(1);
    for (uint32_t i = 0; i < steps; ++i) {
        stepper_uln2003_step(direction);
        if (delay_ms_val) delay_ms(delay_ms_val);  // Blocking delay for rotate_steps only
    }
    // de-energize coils after motion (optional)
    pin_out(s_cfg.port_base, s_cfg.in1_pin, false);
    pin_out(s_cfg.port_base, s_cfg.in2_pin, false);
    pin_out(s_cfg.port_base, s_cfg.in3_pin, false);
    pin_out(s_cfg.port_base, s_cfg.in4_pin, false);
    if (steps) led_out(0);
}

void stepper_uln2003_rotate_degrees(float degrees, int direction, uint32_t delay_ms_val)
{
    if (degrees < 0) degrees = -degrees;
    uint32_t steps = (uint32_t)((degrees / 360.0f) * (float)STEPPER_HALFSTEP_STEPS_PER_REV + 0.5f);
    stepper_uln2003_rotate_steps(steps, direction, delay_ms_val);
}

void stepper_uln2003_config_run_led(uint32_t port_base, uint8_t pin)
{
    s_led_base = port_base;
    s_led_pin = pin;
    s_led_en = 1;

    // Enable LED port and configure pin as digital output using TivaWare API
    enable_gpio_port(s_led_base);
    uint8_t mask = (uint8_t)(1u << s_led_pin);
    GPIOPinTypeGPIOOutput(s_led_base, mask);
    led_out(0);
}

void stepper_uln2003_all_off(void)
{
    uint8_t mask = (uint8_t)((1u << s_cfg.in1_pin) | (1u << s_cfg.in2_pin) |
                              (1u << s_cfg.in3_pin) | (1u << s_cfg.in4_pin));
    GPIOPinWrite(s_cfg.port_base, mask, 0);
}
