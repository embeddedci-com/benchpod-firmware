#ifndef CONSOLE_H
#define CONSOLE_H

#include <stddef.h>
#include <stdint.h>

/* Output sink for command results — lets the same command set serve the local
   USB/USART console and the TCP command server. */
typedef void (*console_out_t)(void *ctx, const char *s);

/* Execute one command line, writing all output through `out`.  `line` is
   modified in place (tokenised).  Transport-agnostic; serialized via hw_lock. */
void console_exec(char *line, console_out_t out, void *ctx);

/* Local interactive console over the USB-CDC VCP / USART2.  console_poll() (net
   task's sibling console task) does line editing only and submits complete lines
   to the hw worker, which runs them via console_run_line(). */
void console_init(void);
void console_poll(void);        /* non-blocking; call in a loop (console task)   */
void console_run_line(const char *line);  /* execute one line (hw worker task)   */

/* Count of console RX bytes dropped because the ring was full (status report). */
uint32_t console_rx_dropped(void);

/* Count of console TX bytes dropped because the UART TX ring was full. */
uint32_t console_tx_dropped(void);

#endif /* CONSOLE_H */
