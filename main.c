#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "stepper_uln2003.h"


#include "uart.h"
#include "proto.h"

static volatile uint32_t g_ms_ticks = 0;
static volatile uint8_t g_flag_10ms = 0;
static volatile uint8_t g_flag_100ms = 0;
static volatile uint8_t g_flag_1000ms = 0;

void SysTickIntHandler(void)
{
    g_ms_ticks++;
    if ((g_ms_ticks % 10u) == 0u)   g_flag_10ms = 1;
    if ((g_ms_ticks % 100u) == 0u)  g_flag_100ms = 1;
    if ((g_ms_ticks % 1000u) == 0u) g_flag_1000ms = 1;
}

uint32_t millis(void)
{
    return g_ms_ticks;
}

int main(void)
{
    // System clock 50 MHz using PLL (16MHz crystal)
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    // Init UART0 console at 115200 (PC or ESP32)
    UART0_ConsoleInit(115200);

    // Systick @1kHz
    SysTickPeriodSet(SysCtlClockGet() / 1000);
    SysTickIntRegister(SysTickIntHandler);
    SysTickIntEnable();
    SysTickEnable();
    IntMasterEnable();

    Proto_Init();

    // Initialize stepper outputs and run a brief self-test to verify wiring
    // ULN2003 IN1..IN4 <- PB4..PB7
    stepper_uln2003_init(&(stepper_uln2003_cfg_t){ GPIO_PORTB_BASE, 4, 5, 6, 7 });
    // Optional: PF1 as run indicator LED
    // stepper_uln2003_config_run_led(GPIO_PORTF_BASE, 1);
    // Self-test: 16 half-steps forward then 16 back, 30ms per step
    stepper_uln2003_rotate_steps(16, +1, 30);
    stepper_uln2003_rotate_steps(16, -1, 30);

    // Main loop: poll UART for JSON lines and dispatch
    while (1) {
        Proto_Poll();
        if (g_flag_10ms) { g_flag_10ms = 0; Proto_Tick10ms(); }
        if (g_flag_100ms) { g_flag_100ms = 0; Proto_Tick100ms(); }
        if (g_flag_1000ms) { g_flag_1000ms = 0; Proto_Tick1000ms(); }
    }
}
