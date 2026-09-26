#ifndef CAN_BUS_H
#define CAN_BUS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ============================================================================
 * CAN — TCAN1044 transceiver (U10) on FDCAN1 (PD1 TX / PD0 RX), classic CAN.
 *
 * The pod has a SINGLE CAN node, so `normal` mode only works with an external
 * node on the bus to ACK frames.  For self-contained bring-up / CI on one pod,
 * use a loopback mode: the node ACKs its own frames and every `can_tx` shows up
 * on `can_rx_pop`.
 *   internal  — TX->RX looped inside the FDCAN core; transceiver NOT involved
 *               (works with nothing wired to CAN+/CAN-).  Validates firmware.
 *   external  — TX drives the real TCAN1044 pins, looped back to RX.  Validates
 *               the transceiver + PCB.  Bus must be otherwise idle.
 *   normal    — real bus operation (needs a second node).
 *   listen    — bus-monitoring: RX only, never drives the bus / never ACKs.
 *
 * Kernel clock = HSE (25 MHz); bit timing is derived per configured bitrate.
 * ========================================================================== */

typedef enum {
    CAN_MODE_NORMAL = 0,
    CAN_MODE_LOOPBACK_INTERNAL,
    CAN_MODE_LOOPBACK_EXTERNAL,
    CAN_MODE_LISTEN,
} can_mode_t;

/* One classic CAN frame (data length 0..8). */
typedef struct {
    uint32_t id;        /* 11-bit (std) or 29-bit (ext) identifier            */
    bool     ext;       /* true = 29-bit extended identifier                  */
    bool     rtr;       /* true = remote-transmission-request (no data)       */
    uint8_t  dlc;       /* data length in bytes (0..8)                        */
    uint8_t  data[8];
    uint32_t ts;        /* device uptime in ms when RX'd (HAL_GetTick, ISR-   */
                        /* stamped, jitter-free) — 0 on a frame you build     */
} can_frame_t;

/* Max autonomous-responder rules (see can_responder_add). */
#define CAN_RESPONDER_MAX  8u

typedef struct {
    bool     enabled;       /* FDCAN started                                  */
    can_mode_t mode;
    uint32_t bitrate;       /* configured nominal bitrate (bit/s)             */
    bool     term;          /* 120 Ω termination engaged (PA4)                */
    uint8_t  tec;           /* transmit error counter                         */
    uint8_t  rec;           /* receive error counter                          */
    bool     bus_off;
    bool     error_passive;
    uint32_t rx_pending;    /* frames waiting in the software RX ring          */
    uint32_t rx_overflow;   /* frames dropped because the ring was full        */
    uint32_t responder_rules;  /* active autonomous-responder rules            */
    uint32_t responder_hits;   /* auto-replies the firmware has sent           */
    uint32_t bus_off_recoveries; /* bus-off recoveries started since boot      */
} can_status_t;

/* Set the termination GPIO to a safe default (off) and mark CAN down.  Called
   once at boot; does NOT bring up FDCAN (that happens lazily on can_configure). */
void can_bus_init(void);

/* Bring up FDCAN1 with the given bitrate + mode and (re)apply termination.
   Safe to call repeatedly — tears down any previous config first.
   Returns 0, -1 on a bad bitrate (no exact bit timing), -2 on HAL failure. */
int  can_configure(uint32_t bitrate, can_mode_t mode, bool fd, bool term);

/* Stop FDCAN1 and release the peripheral.  Termination is left as-is. */
void can_disable(void);

/* Queue one classic frame for transmission (0..8 data bytes).
   Returns 0, -1 if CAN is not enabled, -2 if bus-off, -3 if the TX FIFO is full
   or the frame is invalid.  -2 is not sticky: the core is put back on the bus
   automatically (see bus-off recovery below), so a later call succeeds once
   the bus is healthy again. */
int  can_tx(const can_frame_t *f);

/* Pop up to `max` received frames into `out` (oldest first).  Non-blocking.
   Returns the number of frames copied (0 if none / not enabled). */
int  can_rx_pop(can_frame_t *out, int max);

/* Populate `out` with the current link state.  Returns 0 always. */
int  can_get_status(can_status_t *out);

/* Switch the 120 Ω bus termination in (on=true) / out.  Works even when FDCAN
   is disabled — it is a plain GPIO (PA4 -> U24).  Returns 0. */
int  can_set_term(bool on);
bool can_get_term(void);

/* ---- Autonomous responder (ECU simulation) ------------------------------
 * The firmware watches every received frame and, when one matches a rule's
 * (match_id, match_ext), immediately transmits `reply` from the RX interrupt —
 * no host round-trip, so it answers a DUT's request in microseconds (vs. tens
 * of ms for a host poll loop).  Mirrors the I2C sensor emulation model.
 *
 * In `normal` mode the pod never receives its own transmissions, so a reply is
 * sent once per matching request.  In loopback modes the reply also echoes to
 * RX — harmless as long as the reply's id doesn't itself match a rule (which
 * would loop).  Matching survives can_configure().
 *
 * Add one rule.  Returns the slot index (>=0), or -1 if the table is full. */
int  can_responder_add(uint32_t match_id, bool match_ext, const can_frame_t *reply);
void can_responder_clear(void);       /* remove all rules */
int  can_responder_count(void);       /* active rule count */

/* ---- Bus-off recovery -----------------------------------------------------
 * On bus-off (TEC > 255) the FDCAN core sets PSR.BO and CCCR.INIT and stops.
 * It stays that way until software clears CCCR.INIT (RM0481 FDCAN "Bus-off
 * recovery"); the core then waits for 129 x 11 recessive bits and rejoins the
 * bus, which also clears PSR.BO.  The firmware does that from the bus-off
 * interrupt, and again from the task-side paths (can_tx, can_rx_pop,
 * can_get_status) in case the interrupt was missed.  Each start is counted
 * (can_status_t.bus_off_recoveries).
 *
 * Pure register logic, host-tested: given CCCR (read-modify-written) and a
 * PSR snapshot, start a recovery if the core is bus-off and still in INIT.
 * Returns true when a recovery was started.  A core that already left INIT
 * (recovery in progress) is left alone, so repeated calls count once. */
#define CAN_REG_PSR_BO     (1u << 7)
#define CAN_REG_CCCR_INIT  (1u << 0)

static inline bool can_busoff_recover_regs(volatile uint32_t *cccr, uint32_t psr)
{
    if ((psr & CAN_REG_PSR_BO) == 0u) return false;
    if ((*cccr & CAN_REG_CCCR_INIT) == 0u) return false;
    *cccr &= ~CAN_REG_CCCR_INIT;
    return true;
}

/* Parse a mode name ("normal"|"internal"|"external"|"listen") into can_mode_t.
   Returns 0, -1 if unknown. */
int  can_mode_from_name(const char *name, can_mode_t *out);
const char *can_mode_name(can_mode_t mode);

#endif /* CAN_BUS_H */
