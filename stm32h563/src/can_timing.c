/*
 * can_timing.c — FDCAN bit-timing solver and CAN FD length helpers (see
 * can_timing.h).  No HAL: host-tested.
 */
#include "can_timing.h"

/* FDCAN register limits (RM0481 NBTP / DBTP). */
#define NOM_PRESC_MAX  512u
#define NOM_SEG1_MAX   256u
#define NOM_SEG2_MAX   128u
#define DAT_PRESC_MAX  32u
#define DAT_SEG1_MAX   32u
#define DAT_SEG2_MAX   16u
#define DAT_SJW_MAX    16u
#define TDCO_MAX       127u

/* The classic solver, kept exactly as it was before CAN FD so classic rates
   keep their timing. */
static int solve_classic(uint32_t fclk, uint32_t bitrate, can_timing_t *out)
{
    for (uint32_t total = 25; total >= 8; total--) {
        uint32_t denom = bitrate * total;
        if (denom == 0 || (fclk % denom) != 0) continue;
        uint32_t p = fclk / denom;
        if (p < 1 || p > NOM_PRESC_MAX) continue;
        uint32_t t1 = (total * 7u) / 8u;      /* 0.875 * total, floored */
        if (t1 < 2) t1 = 2;
        if (t1 > total - 2) t1 = total - 2;
        uint32_t s1 = t1 - 1;                  /* seg1 excludes sync */
        uint32_t s2 = total - 1 - s1;
        if (s1 < 2 || s2 < 2 || s2 > NOM_SEG2_MAX || s1 > NOM_SEG1_MAX) continue;
        out->presc = p;
        out->seg1  = s1;
        out->seg2  = s2;
        out->sjw   = (s2 < 4) ? s2 : 4;
        out->tdco  = 0;
        return 0;
    }
    return -1;
}

/* Split `total` tq at `sp_permille` (sync + seg1 = sample point), seg2 >= 2. */
static bool split(uint32_t total, uint32_t sp_permille,
                  uint32_t seg1_max, uint32_t seg2_max,
                  uint32_t *seg1, uint32_t *seg2)
{
    if (total < 5) return false;
    uint32_t t1 = (total * sp_permille + 500u) / 1000u;   /* sync + seg1, rounded */
    if (t1 > total - 2) t1 = total - 2;
    if (t1 < 3) t1 = 3;
    uint32_t s1 = t1 - 1;
    uint32_t s2 = total - t1;
    if (s1 < 1 || s1 > seg1_max || s2 < 2 || s2 > seg2_max) return false;
    *seg1 = s1;
    *seg2 = s2;
    return true;
}

int can_timing_nominal(uint32_t fclk, uint32_t bitrate, bool fd, can_timing_t *out)
{
    if (!out || bitrate == 0 || fclk == 0) return -1;
    if (!fd) return solve_classic(fclk, bitrate, out);

    for (uint32_t p = 1; p <= NOM_PRESC_MAX; p++) {
        if (fclk % p) continue;
        uint32_t tq_hz = fclk / p;
        if (tq_hz % bitrate) continue;
        uint32_t total = tq_hz / bitrate;
        if (total > 1u + NOM_SEG1_MAX + NOM_SEG2_MAX) continue;
        uint32_t s1, s2;
        if (!split(total, 800u, NOM_SEG1_MAX, NOM_SEG2_MAX, &s1, &s2)) continue;
        out->presc = p;
        out->seg1  = s1;
        out->seg2  = s2;
        out->sjw   = s2;
        out->tdco  = 0;
        return 0;
    }
    return -1;
}

int can_timing_data(uint32_t fclk, uint32_t bitrate, can_timing_t *out)
{
    if (!out || bitrate == 0 || fclk == 0) return -1;
    if (bitrate > CAN_FD_DATA_MAX_BPS) return -2;
    uint32_t sp = (bitrate <= 2000000u) ? 800u : 750u;

    for (uint32_t p = 1; p <= DAT_PRESC_MAX; p++) {
        if (fclk % p) continue;
        uint32_t tq_hz = fclk / p;
        if (tq_hz % bitrate) continue;
        uint32_t total = tq_hz / bitrate;
        if (total > 1u + DAT_SEG1_MAX + DAT_SEG2_MAX) continue;
        uint32_t s1, s2;
        if (!split(total, sp, DAT_SEG1_MAX, DAT_SEG2_MAX, &s1, &s2)) continue;
        out->presc = p;
        out->seg1  = s1;
        out->seg2  = s2;
        out->sjw   = (s2 < DAT_SJW_MAX) ? s2 : DAT_SJW_MAX;
        /* TDC: the secondary sample point sits at the measured TX->RX loop
           delay plus this offset; put the offset at the data sample point.
           The FDCAN only supports TDC with a data prescaler of 1 or 2. */
        uint32_t tdco = p * (1u + s1);
        out->tdco = (p <= 2u && tdco <= TDCO_MAX) ? tdco : 0u;
        return 0;
    }
    return -1;
}

uint32_t can_timing_sample_permille(const can_timing_t *t)
{
    uint32_t total = 1u + t->seg1 + t->seg2;
    return ((1u + t->seg1) * 1000u + total / 2u) / total;
}

uint8_t can_dlc_to_len(uint32_t dlc)
{
    static const uint8_t t[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return t[dlc & 0xFu];
}

int can_len_to_dlc(uint32_t len, bool fd)
{
    if (len <= 8u) return (int)len;
    if (!fd || len > CAN_FD_MAX_LEN) return -1;
    for (uint32_t c = 9; c < 16; c++) {
        if (can_dlc_to_len(c) >= len) return (int)c;
    }
    return -1;
}
