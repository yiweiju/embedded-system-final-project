#include "hx711_tiva.h"
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"

extern uint32_t SystemCoreClock; // Provided by system_TM4C123.c
extern uint32_t millis(void);    // Provided by main.c (SysTick counter)

// crude busy-wait delay in microseconds
static void delay_us(uint32_t us)
{
    // Fixed precision: use integer arithmetic with rounding
    // At 50MHz: 50000000 / 4000000 = 12.5, rounds to 13
    uint32_t clk = (SystemCoreClock ? SystemCoreClock : 16000000u);
    uint32_t cycles_per_us = (clk + 2000000u) / 4000000u; // Add half divisor for rounding
    if (cycles_per_us == 0) cycles_per_us = 1;
    volatile uint32_t total = us * cycles_per_us;
    while (total--) { __asm volatile ("nop"); }
}

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

void hx711_init(hx711_t *dev, const hx711_cfg_t *cfg)
{
    dev->cfg = *cfg;
    enable_gpio_port(dev->cfg.port_base);

    uint8_t dout_mask = (uint8_t)(1u << dev->cfg.pin_dout);
    uint8_t sck_mask  = (uint8_t)(1u << dev->cfg.pin_sck);

    // Configure DOUT as input with pull-up; SCK as push-pull output low
    GPIOPinTypeGPIOInput(dev->cfg.port_base, dout_mask);
    GPIOPadConfigSet(dev->cfg.port_base, dout_mask, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    GPIOPinTypeGPIOOutput(dev->cfg.port_base, sck_mask);
    GPIOPinWrite(dev->cfg.port_base, sck_mask, 0);

    dev->scale = 1.0f;
    dev->offset = 0;
}

void hx711_set_scale(hx711_t *dev, float scale)
{ if (scale > 0.0f) dev->scale = scale; }

float hx711_get_scale(const hx711_t *dev)
{ return dev->scale; }

void hx711_set_offset(hx711_t *dev, int32_t offset)
{ dev->offset = offset; }

int hx711_data_ready(const hx711_t *dev)
{
    uint8_t mask = (uint8_t)(1u << dev->cfg.pin_dout);
    return (GPIOPinRead(dev->cfg.port_base, mask) & mask) ? 0 : 1;
}

// Wait for HX711 ready with timeout (returns 1 on success, 0 on timeout)
static int hx711_wait_ready_timeout(hx711_t *dev, uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (!hx711_data_ready(dev)) {
        if ((millis() - start) >= timeout_ms) {
            return 0; // timeout
        }
        delay_us(5);
    }
    return 1; // ready
}

// Legacy function without timeout (for backward compatibility)
static void hx711_wait_ready(hx711_t *dev)
{
    while (!hx711_data_ready(dev)) { delay_us(5); }
}

// Read raw value with timeout protection (returns 1 on success, 0 on timeout)
int hx711_read_raw_timeout(hx711_t *dev, int32_t *out, uint32_t timeout_ms)
{
    if (!out) return 0;

    uint8_t dout_mask = (uint8_t)(1u << dev->cfg.pin_dout);
    uint8_t sck_mask  = (uint8_t)(1u << dev->cfg.pin_sck);

    // Wait for data ready with timeout
    if (!hx711_wait_ready_timeout(dev, timeout_ms)) {
        return 0; // timeout waiting for ready
    }

    uint32_t value = 0u;
    for (int i = 0; i < 24; ++i) {
        GPIOPinWrite(dev->cfg.port_base, sck_mask, sck_mask);
        delay_us(1);
        value = (value << 1) | ((GPIOPinRead(dev->cfg.port_base, dout_mask) & dout_mask) ? 1u : 0u);
        GPIOPinWrite(dev->cfg.port_base, sck_mask, 0);
        delay_us(1);
    }
    // Gain set pulse (128x)
    GPIOPinWrite(dev->cfg.port_base, sck_mask, sck_mask);
    delay_us(1);
    GPIOPinWrite(dev->cfg.port_base, sck_mask, 0);
    delay_us(1);

    // Sign extend from 24-bit to 32-bit
    if (value & 0x800000u) value |= 0xFF000000u;
    *out = (int32_t)value;
    return 1; // success
}

int32_t hx711_read_raw(hx711_t *dev)
{
    uint8_t dout_mask = (uint8_t)(1u << dev->cfg.pin_dout);
    uint8_t sck_mask  = (uint8_t)(1u << dev->cfg.pin_sck);
    hx711_wait_ready(dev);
    uint32_t value = 0u;
    for (int i = 0; i < 24; ++i) {
        GPIOPinWrite(dev->cfg.port_base, sck_mask, sck_mask);
        delay_us(1);
        value = (value << 1) | ((GPIOPinRead(dev->cfg.port_base, dout_mask) & dout_mask) ? 1u : 0u);
        GPIOPinWrite(dev->cfg.port_base, sck_mask, 0);
        delay_us(1);
    }
    // Gain set pulse (128x)
    GPIOPinWrite(dev->cfg.port_base, sck_mask, sck_mask);
    delay_us(1);
    GPIOPinWrite(dev->cfg.port_base, sck_mask, 0);
    delay_us(1);
    if (value & 0x800000u) value |= 0xFF000000u;
    return (int32_t)value;
}

// Get calibrated mass with timeout protection
int hx711_get_mass_timeout(hx711_t *dev, float *out, uint32_t timeout_ms)
{
    if (!out) return 0;

    int32_t raw = 0;
    if (!hx711_read_raw_timeout(dev, &raw, timeout_ms)) {
        return 0; // timeout
    }

    float scale = (dev->scale > 0.0f) ? dev->scale : 1.0f;
    *out = ((float)(raw - dev->offset)) / scale;
    return 1; // success
}

float hx711_get_mass(hx711_t *dev)
{
    int32_t raw = hx711_read_raw(dev);
    float scale = (dev->scale > 0.0f) ? dev->scale : 1.0f;
    return ((float)(raw - dev->offset)) / scale;
}
