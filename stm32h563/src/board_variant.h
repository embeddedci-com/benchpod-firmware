/*
 * board_variant.h — does this BenchPod have the analog front end, and where?
 *
 * The digital-only board (digital-v3, digital-v3r2) is the analog board without its analog
 * section: no ADC, DAC, ±14 V supplies or relays. It carries the same revision strap as the
 * v3 analog board, so board_rev cannot tell them apart.
 *
 * One-piece analog boards: the analog section is detected by its two I2C expanders on the main
 * bus, U55 (DAC mux, 0x24) and U58 (analog switching, 0x26). Both sit on the main +3V3 rail, so
 * they answer from power-up, and they exist only on analog boards. Analog v3r2 has no U55 (its
 * DAC8554 has a channel per output, dac_quad.h): U58 alone answers there, which is enough. An I2C
 * ACK is definitive, needs no gateware and switches nothing. The ADC and DAC themselves cannot be
 * detected: the DAC has no data output, and the MCP33131 has no ID register.
 *
 * Digital v3r3 (module_link.h) splits the pod into a digital core board and a plug-in analog
 * module on a 24-pin FFC. It still has the v3 strap, so it is recognised by its module-link pins
 * instead. There the analog section is the module: MOD_PRESENT# low and U58 answering on the
 * module bus (I2C2). The module has the analog v3r2 DAC8554 and the same U58 bits, and no U55.
 * The main bus is not probed for expanders on digital v3r3 (it has none).
 *
 * Either way detection runs once at boot: the pod has to be restarted after a module or add-on
 * is fitted. Accessors are plain RAM reads afterwards.
 */
#ifndef BOARD_VARIANT_H
#define BOARD_VARIANT_H

#include <stdbool.h>
#include <stdint.h>

#define BOARD_VARIANT_PROBE_TRIES 3u

/* The decision for the one-piece boards, with the probe injected (host tests). True when either
   expander ACKs on any of BOARD_VARIANT_PROBE_TRIES attempts: one missed ACK must not turn an
   analog board into a digital one. *u55 / *u58 report which expander answered (may be NULL). */
bool board_variant_detect(bool (*probe)(uint8_t addr), bool *u55, bool *u58);

typedef enum {
    BOARD_ANALOG_NONE = 0,   /* digital-only */
    BOARD_ANALOG_ONBOARD,    /* one-piece analog board: expanders on the main bus (I2C1) */
    BOARD_ANALOG_MODULE,     /* digital v3r3 with the analog module: U58 on the module bus (I2C2) */
} board_analog_t;

typedef struct {
    bool u55, u58;           /* main-bus expanders that answered (one-piece boards) */
    bool module_u58;         /* U58 answered on the module bus */
} board_variant_seen_t;

/* The whole decision (host tests).  link_board: the module-link pins of digital v3r3 were seen;
   present: MOD_PRESENT# reads low.  probe_main / probe_module ACK-test an address on each bus.
   On a link board only the module bus is probed, and only when a module is present; elsewhere
   only the main bus.  `seen` may be NULL. */
board_analog_t board_variant_decide(bool link_board, bool present,
                                    bool (*probe_main)(uint8_t addr),
                                    bool (*probe_module)(uint8_t addr),
                                    board_variant_seen_t *seen);

/* Probe and latch the result, and point the analog expander driver at the right bus.  Call once
   after i2c_bus_init() and module_link_init(). */
void board_variant_init(void);

/* The board has the analog front end (on board or as a module). True until board_variant_init()
   runs, so nothing is hidden on an analog board by calling it early. */
bool board_has_analog(void);

/* The analog front end is the plug-in module of digital v3r3. */
bool board_analog_is_module(void);

/* "analog" or "digital" (the module counts as analog). */
const char *board_variant_str(void);

/* The board has the ESP32-C3 Wi-Fi module.  Every board before digital v3r3 does; digital v3r3
   (module_link_board()) has none.  True until module_link_init() runs. */
bool board_has_wifi_module(void);

#endif /* BOARD_VARIANT_H */
