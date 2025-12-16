// UART配置 (更新于2025-12-06):
// - 硬件模块：UART1（从UART0迁移）
// - 物理引脚：PC4 (U1RX), PC5 (U1TX)
// - 用途：    ESP32通信（AT命令协议）
// - 波特率：  115200 8N1
// - 时钟源：  PIOSC 16MHz（温度稳定）
// - RX缓冲：  512字节中断驱动环形缓冲区
//
// 注意：公共API保留"UART0"命名以保持向后兼容性

#include "uart.h"

#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include <stdarg.h>
#include <stdio.h>

// RX ring buffer
#define UART0_RX_BUF_SZ 2048u
static volatile unsigned int rx_head = 0; // index [0..UART0_RX_BUF_SZ-1]
static volatile unsigned int rx_tail = 0;
static volatile char rx_buf[UART0_RX_BUF_SZ];

// ISR Context: Only modifies rx_head
static inline void rx_push_char(char c)
{
    unsigned int next = (rx_head + 1u) % UART0_RX_BUF_SZ;
    
    // FIX: Race condition solved by "Drop New" policy.
    // If buffer is full, we do NOT touch rx_tail (which main loop owns).
    // We simply discard the incoming character.
    if (next != rx_tail) {
        rx_buf[rx_head] = c;
        rx_head = next;
    } else {
        // Buffer overflow: Byte dropped. 
        // Optional: Set a flag here if you want to detect overflow.
    }
}

// Main Loop Context: Only modifies rx_tail
static bool rx_pop_char(char *c)
{
    if (rx_tail == rx_head) return false;
    
    *c = rx_buf[rx_tail];
    // Atomic read of rx_tail is safe.
    // We are the only writer to rx_tail, so no interrupt disable needed.
    rx_tail = (rx_tail + 1u) % UART0_RX_BUF_SZ;
    
    return true;
}

static void UART1IntHandler(void)
{
    uint32_t status = UARTIntStatus(UART1_BASE, true);
    UARTIntClear(UART1_BASE, status);
    if (status & (UART_INT_RX | UART_INT_RT)) {
        while (UARTCharsAvail(UART1_BASE)) {
            int32_t ch = UARTCharGetNonBlocking(UART1_BASE);
            if (ch >= 0) rx_push_char((char)ch);
            else break;
        }
    }
}

void UART0_ConsoleInit(uint32_t baud)
{
    // Enable GPIOC and UART1
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC)){}
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART1)){}

    // Configure PC4/PC5 for U1RX/U1TX
    GPIOPinConfigure(GPIO_PC4_U1RX);
    GPIOPinConfigure(GPIO_PC5_U1TX);
    GPIOPinTypeUART(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    // Use PIOSC (16MHz) as UART clock for stability regardless of system clock
    UARTClockSourceSet(UART1_BASE, UART_CLOCK_PIOSC);

    // Directly configure UART1 for 8N1 using PIOSC 16MHz
    UARTDisable(UART1_BASE);
    UARTConfigSetExpClk(UART1_BASE, 16000000, baud,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTEnable(UART1_BASE);

    // Enable FIFO with mid thresholds
    UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX4_8, UART_FIFO_RX4_8);
    UARTFIFOEnable(UART1_BASE);

    // Enable RX interrupts and register handler
    IntDisable(INT_UART1);
    UARTIntDisable(UART1_BASE, 0xFFFFFFFF);
    UARTIntRegister(UART1_BASE, UART1IntHandler);
    UARTIntEnable(UART1_BASE, UART_INT_RX | UART_INT_RT);
    IntEnable(INT_UART1);
}

// Minimal printf over UART1
static void uart0_write(const char *s)
{
    while (*s) {
        if (*s == '\n') {
            UARTCharPut(UART1_BASE, '\r');
        }
        UARTCharPut(UART1_BASE, *s++);
    }
}

void UARTprintf(const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    uart0_write(buf);
}

bool UART0_ReadChar(char *ch)
{
    if (!ch) return false;
    return rx_pop_char(ch);
}
