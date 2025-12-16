#ifndef USER_UART_H
#define USER_UART_H

#include <stdint.h>
#include <stdbool.h>

// Initialize UART0 as console for JSON line I/O (PC via ICDI)
void UART0_ConsoleInit(uint32_t baud);

// Non-blocking read of one character from UART RX ring buffer
// Returns true and writes to *ch when a character is available.
bool UART0_ReadChar(char *ch);

// Minimal printf over UART0
void UARTprintf(const char *fmt, ...);

#endif // USER_UART_H
