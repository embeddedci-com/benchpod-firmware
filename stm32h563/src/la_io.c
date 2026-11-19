/* la_io.c: the LA I/O bank, its voltage (the bank mux on PG3, status on PG4), 5 V I/O and pin
   21 (digital v3r4), and the LA clamps' settle.  See la_io.h. */
#include "la_io.h"
#include "board_caps.h"
#include "vref_out.h"
#include "la_pull.h"   /* la_pull_all_off: v2 pull-ups go to +3V3, not to the bank */
#include "stm32h5xx_hal.h"
#include "board_pins.h"
#include "pico_compat.h"   /* sleep_ms */
#include "bp_log.h"

/* The two voltages, always set together except in 5 V mode (la_io.h). */
static struct {
    int bank_mv;   /* the bank (and the FPGA's I/O): LA_VCCIO_UNSET, _1V8 or _3V3 */
    int io_mv;     /* the DUT's I/O voltage: the bank, or LA_VCCIO_5V0 when pin 21 is on 5 V */
} s_la = { LA_VCCIO_UNSET, LA_VCCIO_UNSET };

void la_vccio_init(void) {
    /* The GPIOG clock is enabled by signal_engine_init(), which calls this. */
    GPIO_InitTypeDef gp = {0};

    /* PG4 = ST: TPS2116 open-drain status output (external pull-up R152 to 3V3);
       read-only input. */
    gp.Mode = GPIO_MODE_INPUT;
    gp.Pull = GPIO_NOPULL;
    gp.Pin  = LA_VCCIO_ST_PIN;
    HAL_GPIO_Init(LA_VCCIO_ST_PORT, &gp);

    /* PG3 = select (PR1): leave as a Hi-Z input at boot, so R151's 100 k pull-up
       to +3V3 decides the power-on state.  On v3 that is VIN1 = +3.3 V, the safe
       default for a 3.3 V DUT; on v2 (MODE grounded) PR1 high is the mux's
       SHUTDOWN state and the bank rail is simply unpowered.  Either way the
       software state stays UNSET, so LA-bank ops are refused until
       la_vccio_set_mv() is called — the host must choose the DUT voltage
       deliberately. */
    gp.Mode = GPIO_MODE_INPUT;
    gp.Pull = GPIO_NOPULL;
    gp.Pin  = LA_VCCIO_SW_PIN;
    HAL_GPIO_Init(LA_VCCIO_SW_PORT, &gp);

    s_la.bank_mv = LA_VCCIO_UNSET;
    s_la.io_mv   = LA_VCCIO_UNSET;
}

/* The LA clamps' settle (digital v3r4).  A task delay, not a busy wait: this runs on the hw worker,
   the one task that touches the LA bank, pin 21 and the pulls (hw_worker.h), so no other command,
   pull write or output change can run until it returns; the net and USB tasks do run.  A tick
   delay can end up to one tick (1 ms) early, so it asks for one more.  Before the scheduler
   starts, sleep_ms is a busy wait. */
static void la_settle_clamps(void) { sleep_ms(LA_VCCIO_SETTLE_CLAMPS_MS + 1u); }

/* 5 V I/O (digital v3r4, board_caps()->la_5v): the bank (and the FPGA) stays at 3.3 V, the LA
   clamps keep 5 V off the FPGA pins, and pin 21, the rail every pull-up pulls to, goes to 5 V.
   A pod high above 3.3 V comes only through a pull-up.  Order: the bank to 3.3 V first, then pin
   21 to 5 V; back: pin 21 to the bank first, then the bank.  5 V is checked on the pin 21 sense:
   if it did not come up the previous setting is restored and -3 returned. */
int la_io_set_5v(void) {
    const int prev = s_la.io_mv;
    if (prev == LA_VCCIO_5V0 && vref_out_measure_mv() >= VREF_5V_MIN_MV) return 0;   /* as it is */
    int rc = la_vccio_set_mv(LA_VCCIO_3V3);   /* also waits for the clamps */
    if (rc != 0) return rc;
    (void)vref_out_set_source(VREF_SRC_5V);
    la_settle_clamps();                        /* the sense filter (6.7 ms, calculated) and the pull-ups */
    int mv = vref_out_measure_mv();
    if (mv < VREF_5V_MIN_MV) {
        LOG_W("[la] 5 V did not come up on pin 21 (%d mV); back to %d mV\n", mv, prev);
        (void)vref_out_set_source(VREF_SRC_BANK);
        if (prev == LA_VCCIO_1V8 || prev == LA_VCCIO_3V3) (void)la_vccio_set_mv(prev);
        else la_vccio_init();
        return -3;
    }
    s_la.io_mv = LA_VCCIO_5V0;
    return 0;
}

int la_vccio_set_mv(int mv) {
    if (mv == LA_VCCIO_5V0 && board_caps()->la_5v) return la_io_set_5v();
    if (mv != LA_VCCIO_1V8 && mv != LA_VCCIO_3V3) return -1;

    /* 1.8 V is a rev3 feature.  On v2 the TPS2116's MODE pin is tied to GND, so
       the part has no manual 1.8 V selection at all: MODE low + PR1 low passes
       the HIGHER of the two inputs (always +3V3) and MODE low + PR1 high is
       SHUTDOWN — the bank goes high-Z, not to 1.8 V.  The old firmware called
       that state "1.8 V" and it never was; refuse it rather than silently
       cutting the DUT's I/O rail. */
    if (mv == LA_VCCIO_1V8 && !board_caps()->la_1v8) return -2;

    /* On v3 the pulls go to LA_VCCIO or GND (LA1-6 up and LA7/8 down before digital v3r4,
       every channel both ways on v3r4), so the engaged pulls simply follow the bank to 1.8 V
       and stay on.  Only the v2 pull-ups
       went to +3V3, where a closed one would hold the DUT line ~1.5 V above its own
       rail; v2 is refused above, and this keeps the invariant if that ever changes. */
    if (mv == LA_VCCIO_1V8 && !board_caps()->la_1v8) la_pull_all_off();

    /* Drive the select level BEFORE switching PG3 to a push-pull output, so the
       pin never momentarily drives the wrong rail.
       v3 (U8 MODE tied to +3V3 = manual mode, datasheet truth table):
           PR1 high -> VIN1 = +3V3,  PR1 low -> VIN2 = +1V8.
       v2 (MODE tied to GND): PR1 low passes the higher input (+3V3) and PR1
       high is shutdown, so the only reachable level is +3V3 with PR1 LOW — the
       polarity is genuinely inverted between the two boards. */
    /* Leaving 5 V (digital v3r4): pin 21 back on the bank before the bank moves. */
    if (vref_out_source() == VREF_SRC_5V) (void)vref_out_set_source(VREF_SRC_BANK);

    GPIO_PinState sel;
    if (board_caps()->la_1v8)
        sel = (mv == LA_VCCIO_3V3) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    else
        sel = GPIO_PIN_RESET;   /* v2: only +3V3 is reachable, and PR1 must be low */
    HAL_GPIO_WritePin(LA_VCCIO_SW_PORT, LA_VCCIO_SW_PIN, sel);

    GPIO_InitTypeDef gp = {0};
    gp.Mode  = GPIO_MODE_OUTPUT_PP;
    gp.Pull  = GPIO_NOPULL;
    gp.Speed = GPIO_SPEED_FREQ_LOW;
    gp.Pin   = LA_VCCIO_SW_PIN;
    HAL_GPIO_Init(LA_VCCIO_SW_PORT, &gp);

    s_la.bank_mv = mv;
    s_la.io_mv   = mv;
    /* Let the VCCIO rail settle before it is used.  With the LA clamps (digital v3r4) their
       reference follows the bank through a filter: about 30 ms before the pod drives.  Older
       boards keep the 2 ms busy wait they always had. */
    if (board_caps()->la_clamps) la_settle_clamps();
    else HAL_Delay(2);
    return 0;
}

int la_vccio_get_mv(void) { return s_la.bank_mv; }
int la_io_get_mv(void) { return s_la.io_mv; }

int la_vccio_status_pin(void) {
    return (HAL_GPIO_ReadPin(LA_VCCIO_ST_PORT, LA_VCCIO_ST_PIN) == GPIO_PIN_SET) ? 1 : 0;
}

int la_vccio_readback_mv(void) {
    if (!board_caps()->la_1v8) return 0;   /* v2 ST is ambiguous (low = 1V8 OR shutdown) */
    return la_vccio_status_pin() ? LA_VCCIO_3V3 : LA_VCCIO_1V8;
}
