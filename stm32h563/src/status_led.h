#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <stdint.h>
#include <stdbool.h>

/*
 * status_led — what the pod's RGB status LED shows.
 *
 * The LED hangs off the iCE40's RGB driver pads; the STM32 sets it with SET_LED (0x42, a 3-bit
 * mask, one bit per pad). The gateware only dims it. Everything below is decided here, on the
 * hw worker, and only a change of colour goes out over SPI (plus a slow refresh, since loading
 * gateware resets the mask).
 *
 * What it shows, first match wins:
 *
 *   boot failed           solid red      iCE40 or PSRAM self-test failed, or safe mode
 *   target power fault    red, blinking  an eFuse reports FLT
 *   booting               off            until the iCE40/PSRAM bring-up ends
 *   boot OK               3 green blinks once bring-up ends with a working iCE40 and PSRAM
 *   update                magenta        firmware or gateware staging / verify (OTA)
 *   busy                  blue           a capture, upload or other PSRAM-bus job in flight
 *   command               cyan flash     a command that drives the pod (not polled reads)
 *   target power on       green          an eFuse is on, nothing else going on
 *   idle                  off
 *
 * A pod whose iCE40 never configured cannot light the LED at all: dark after boot means that.
 *
 * Pure logic, host-tested (test_status_led.c); status_led_hw.c feeds it the pod's state.
 */

/* SET_LED mask bit per colour. The bits are the iCE40 pads (bit0 = RGB0 pad 39, bit1 = RGB1 pad
   40, bit2 = RGB2 pad 41), which colour sits on which pad is the LED's pinout (LED1,
   S4-3528RGBTA-A), checked on a rev3 pod with `led 1|2|4`. The gateware's names for the bits
   (green/yellow/red) are from the v1 board. */
#define STATUS_LED_BLUE     0x01u
#define STATUS_LED_GREEN    0x02u
#define STATUS_LED_RED      0x04u
#define STATUS_LED_CYAN     (STATUS_LED_GREEN | STATUS_LED_BLUE)
#define STATUS_LED_MAGENTA  (STATUS_LED_RED | STATUS_LED_BLUE)
#define STATUS_LED_OFF      0x00u

#define STATUS_LED_BOOT_BLINKS     3u
#define STATUS_LED_BOOT_BLINK_MS   200u    /* on, then off, per blink */
#define STATUS_LED_FAULT_BLINK_MS  250u
#define STATUS_LED_CMD_FLASH_MS    150u
#define STATUS_LED_REFRESH_MS      1000u   /* resend an unchanged mask this often */

/* The pod's state, sampled once per poll. */
typedef struct {
    bool boot_done;     /* the iCE40/PSRAM bring-up has finished (or was skipped) */
    bool hw_ok;         /* iCE40 configured, PSRAM datapath OK, not in safe mode */
    bool power_fault;   /* an eFuse reports FLT */
    bool power_on;      /* an eFuse is on */
    bool updating;      /* firmware or gateware staging / verify */
    bool busy;          /* a capture, upload or other PSRAM-bus job */
} status_led_in_t;

/* Reset all state (boot). */
void status_led_reset(void);

/* A command that drives the pod ran: flash for STATUS_LED_CMD_FLASH_MS from now_ms. */
void status_led_command(uint32_t now_ms);

/* The colour for `in` at now_ms (one of the STATUS_LED_* masks). Call it every poll: the boot
   blinks start at the first call that sees boot_done. */
uint8_t status_led_color(const status_led_in_t *in, uint32_t now_ms);

/* The mask to send at now_ms, or -1 when the LED already shows it and the refresh is not due.
   Call once per poll with status_led_color's result. */
int status_led_next_write(uint8_t color, uint32_t now_ms);
/* Forget what the LED shows, so the next status_led_next_write sends. */
void status_led_reset_write(void);

/* Hardware glue (status_led_hw.c, hw worker only): sample the pod, write SET_LED on a change. */
void status_led_service(void);
/* The iCE40/PSRAM bring-up has finished (boot_deferred_hw_init returned). */
void status_led_boot_finished(void);
/* A command was dispatched; `drives` is false for read-only and polled commands. */
void status_led_note_command(bool drives);
/* Show `mask` for hold_ms (the `led` console command), then go back to the status. */
void status_led_override(uint8_t mask, uint32_t hold_ms);

#endif /* STATUS_LED_H */
