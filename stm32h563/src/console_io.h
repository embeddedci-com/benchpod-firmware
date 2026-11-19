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

/* Write log text to both sinks with a CR before every bare LF, all under the console lock (no
   other task's output lands inside it).  printf's _write and log_printf (bp_log.h) use it. */
void console_io_write_text(const char *ptr, size_t len);

/* The prompt and line editor (console_prompt.h), under the console lock.  console_io_prompt
   prints "> " and puts the prompt up; background output written while it is up starts on its own
   line and ends with the prompt and the typed text printed again. */
void   console_io_prompt(void);
/* A late reply of several lines: true before its first line, false after its last (one prompt). */
void   console_io_hold_prompt(bool on);
void   console_io_edit_char(char c);
void   console_io_edit_backspace(void);
/* Enter: ends the line, takes the prompt down, copies the typed line to out.  Its length. */
size_t console_io_edit_enter(char *out, size_t cap);

/* Push one received byte into the RX ring (called from ISR / CDC callback). */
void console_io_rx_push(uint8_t byte);

/* Pop one byte from the RX ring; returns -1 if empty. */
int  console_io_getc(void);
bool console_io_rx_available(void);

#endif /* CONSOLE_IO_H */
