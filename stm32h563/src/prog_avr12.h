#ifndef PROG_AVR12_H
#define PROG_AVR12_H

/* prog_avr12.h — the 12-bit frame shared by AVR TPI and XMEGA PDI (prog_tpi.c, prog_pdi.c).
 *
 * Frame: start bit (0), 8 data bits LSB first, even parity, 2 stop bits (1). The sender changes
 * data while the clock is low; the receiver samples on the rising edge. The pod holds the clock
 * HIGH between frames.
 *
 * Direction change: the pod releases data (data_drive false) before the chip answers and takes
 * it back (data_drive true) with the line high before the next frame. The chip's guard time is
 * set to 2 bits by the protocol's entry code, so the answer's start bit comes within a few clocks;
 * avr12_rx clocks a small search window and looks for the start bit in it.
 *
 * Nothing is flushed by avr12_tx: a caller batches frames and flushes with avr12_commit or
 * through avr12_rx (which flushes the frames queued before it). Static inline: no code unless used.
 */

#include "prog_flash.h"

/* TPI `flags` (prog_tpi.c), set by the host from its device table:
   bits 0-2  words per flash write: 0 or 1 = one word (ATtiny4/5/9/10, 102/104), 2 = ATtiny20,
             4 = ATtiny40 (the chip latches that many words and starts one write);
   bit 3     page erase (NVM command 0x18, ATtiny102/104): PROG_ERASE_PAGE erases each page it
             writes. Without it PROG_ERASE_PAGE is one section erase of the whole code section
             before the first page (the other chips cannot erase less). */
#define TPI_F_WORDS_MASK  0x07u
#define TPI_F_PAGE_ERASE  0x08u

#define AVR12_GUARD  8u                 /* idle clocks searched for the start bit */
#define AVR12_RXCLK  (AVR12_GUARD + 12u)

static inline unsigned avr12_parity(uint8_t b) {
    b ^= (uint8_t)(b >> 4); b ^= (uint8_t)(b >> 2); b ^= (uint8_t)(b >> 1);
    return b & 1u;
}

static inline void avr12_bit(const prog_wire_ops_t *w, int v) {
    w->set(w->io, 0, v);   /* data changes on the falling edge */
    w->set(w->io, 1, v);   /* the chip samples on the rising edge */
}

/* Idle bits: data high, the clock running. */
static inline void avr12_idle(const prog_wire_ops_t *w, unsigned n) {
    w->set(w->io, 1, 1);
    w->data_drive(w->io, true);
    while (n--) avr12_bit(w, 1);
}

static inline void avr12_tx(const prog_wire_ops_t *w, uint8_t b) {
    w->set(w->io, 1, 1);
    w->data_drive(w->io, true);
    avr12_bit(w, 0);
    for (unsigned i = 0; i < 8; i++) avr12_bit(w, (b >> i) & 1);
    avr12_bit(w, (int)avr12_parity(b));
    avr12_bit(w, 1);
    avr12_bit(w, 1);
}

/* Send what is queued. 0 or PROG_E_PROTOCOL. */
static inline int avr12_commit(const prog_wire_ops_t *w) {
    uint8_t dummy[4];
    return w->flush(w->io, dummy, sizeof dummy) < 0 ? PROG_E_PROTOCOL : 0;
}

/* Read one frame the chip sends: 0 and *out; PROG_E_NO_ANSWER when no start bit shows up in the
   window; PROG_E_PROTOCOL for bad parity, stop bits or a failed flush. Leaves data released. */
static inline int avr12_rx(const prog_wire_ops_t *w, uint8_t *out) {
    uint8_t s[AVR12_RXCLK];
    w->set(w->io, 1, 1);
    w->data_drive(w->io, false);
    for (unsigned i = 0; i < AVR12_RXCLK; i++) {
        w->set(w->io, 0, 1);
        w->set(w->io, 1, 1);
        w->sample(w->io);
    }
    int n = w->flush(w->io, s, sizeof s);
    if (n != (int)AVR12_RXCLK) return PROG_E_PROTOCOL;
    unsigned i = 0;
    while (i <= AVR12_GUARD && s[i]) i++;
    if (i > AVR12_GUARD) return PROG_E_NO_ANSWER;
    uint8_t v = 0;
    for (unsigned k = 0; k < 8; k++) v |= (uint8_t)((s[i + 1 + k] & 1u) << k);
    if ((s[i + 9] & 1u) != avr12_parity(v) || !s[i + 10] || !s[i + 11]) return PROG_E_PROTOCOL;
    *out = v;
    return 0;
}

#endif /* PROG_AVR12_H */
