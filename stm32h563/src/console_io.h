#ifndef CONSOLE_IO_H
#define CONSOLE_IO_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* Console byte I/O multiplexed over USART2 (PD5/PD6) and the USB-CDC VCP.
 * Output (printf via _write) goes to both sinks; input is merged from both
 * into one RX ring.  USART2 RX is fed from its IRQ; USB-CDC RX from the class
 * Receive callback (both call console_io_rx_push). */

void console_io_init(void);

/* Write to both sinks (USART2 blocking, USB-CDC best-effort/ring-buffered). */
void console_io_write(const uint8_t *buf, size_t len);

/* Push one received byte into the RX ring (called from ISR / CDC callback). */
void console_io_rx_push(uint8_t byte);

/* Pop one byte from the RX ring; returns -1 if empty. */
int  console_io_getc(void);
bool console_io_rx_available(void);

#endif /* CONSOLE_IO_H */
