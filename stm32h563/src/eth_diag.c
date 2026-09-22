#include "eth_diag.h"

#include <stdio.h>

const char *eth_diag_phy_mode(uint16_t physcsr) {
    switch ((physcsr & ETH_DIAG_SCSR_SPEED_MASK) >> ETH_DIAG_SCSR_SPEED_SHIFT) {
    case 1:  return "10M half";
    case 5:  return "10M full";
    case 2:  return "100M half";
    case 6:  return "100M full";
    default: return "none";
    }
}

const char *eth_diag_mac_mode(uint32_t maccr) {
    bool fast = (maccr & ETH_DIAG_MACCR_FES) != 0, full = (maccr & ETH_DIAG_MACCR_DM) != 0;
    return fast ? (full ? "100M full" : "100M half") : (full ? "10M full" : "10M half");
}

/* The PHY resolved a mode (link up) that the MAC is not programmed for: frames get mangled. */
static bool mode_mismatch(const eth_diag_t *d) {
    unsigned phy = (d->physcsr & ETH_DIAG_SCSR_SPEED_MASK) >> ETH_DIAG_SCSR_SPEED_SHIFT;
    unsigned mac = ((d->maccr & ETH_DIAG_MACCR_FES) ? 2u : 1u) | ((d->maccr & ETH_DIAG_MACCR_DM) ? 4u : 0u);
    bool resolved = phy == 1 || phy == 2 || phy == 5 || phy == 6;
    return (d->bsr & ETH_DIAG_BSR_LINK) && resolved && phy != mac;
}

int eth_diag_format(const eth_diag_t *d, char *buf, size_t cap) {
    if (!d->phy_ok)
        return snprintf(buf, cap, "phy unreadable (MDIO) | mac %s | rx good %lu crc %lu align %lu "
                        "missed %lu allocfail %lu%s | tx good %lu col %lu/%lu",
                        eth_diag_mac_mode(d->maccr), (unsigned long)d->rx_good,
                        (unsigned long)d->rx_crc, (unsigned long)d->rx_align,
                        (unsigned long)d->rx_missed, (unsigned long)d->rx_alloc_fail,
                        d->rx_alloc_stuck ? " (pool empty now)" : "", (unsigned long)d->tx_good,
                        (unsigned long)d->tx_col_single, (unsigned long)d->tx_col_multi);
    return snprintf(buf, cap,
                    "phy link %s aneg %s%s mode %s partner 0x%04x symerr %u | mac %s%s | lwip link %s | "
                    "rx frames %lu (bcast %lu) unicast %lu crc %lu align %lu missed %lu%s allocfail %lu%s | tx good %lu col %lu/%lu",
                    (d->bsr & ETH_DIAG_BSR_LINK) ? "up" : "DOWN",
                    (d->bsr & ETH_DIAG_BSR_ANEG_DONE) ? "done" : "NOT DONE",
                    (d->bsr & ETH_DIAG_BSR_RFAULT) ? " REMOTE-FAULT" : "",
                    eth_diag_phy_mode(d->physcsr), d->anlpar, d->symbol_errors,
                    eth_diag_mac_mode(d->maccr), mode_mismatch(d) ? " (MISMATCH with phy)" : "",
                    d->netif_link ? "up" : "down",
                    (unsigned long)d->rx_frames, (unsigned long)d->rx_bcast,
                    (unsigned long)d->rx_good, (unsigned long)d->rx_crc, (unsigned long)d->rx_align,
                    (unsigned long)d->rx_missed, d->rx_missed_ovf ? "+ovf" : "",
                    (unsigned long)d->rx_alloc_fail, d->rx_alloc_stuck ? " (pool empty now)" : "",
                    (unsigned long)d->tx_good, (unsigned long)d->tx_col_single,
                    (unsigned long)d->tx_col_multi);
}

int eth_diag_error_delta(const eth_diag_t *prev, const eth_diag_t *cur, char *buf, size_t cap) {
    struct { const char *name; uint32_t delta; } c[] = {
        {"crc",       cur->rx_crc - prev->rx_crc},
        {"align",     cur->rx_align - prev->rx_align},
        {"missed",    cur->rx_missed - prev->rx_missed},
        {"allocfail", cur->rx_alloc_fail - prev->rx_alloc_fail},
        {"col",       (cur->tx_col_single + cur->tx_col_multi) - (prev->tx_col_single + prev->tx_col_multi)},
        {"symerr",    (uint16_t)(cur->symbol_errors - prev->symbol_errors)},
    };
    size_t pos = 0;
    if (cap) buf[0] = '\0';
    for (size_t i = 0; i < sizeof(c) / sizeof(c[0]); i++) {
        if (!c[i].delta || pos >= cap) continue;
        int n = snprintf(buf + pos, cap - pos, "%s%s +%lu", pos ? " " : "", c[i].name, (unsigned long)c[i].delta);
        if (n > 0) pos += (size_t)n;
    }
    return (int)pos;
}

int eth_diag_json(const eth_diag_t *d, char *buf, size_t cap) {
    return snprintf(buf, cap,
                    "{\"phy_ok\":%s,\"link\":%s,\"aneg_done\":%s,\"remote_fault\":%s,\"phy_mode\":\"%s\","
                    "\"mac_mode\":\"%s\",\"partner\":%u,\"symbol_errors\":%u,\"lwip_link\":%s,"
                    "\"rx_frames\":%lu,\"rx_bcast\":%lu,\"rx_good\":%lu,\"rx_crc\":%lu,\"rx_align\":%lu,\"rx_missed\":%lu,\"rx_missed_overflow\":%lu,"
                    "\"rx_alloc_fail\":%lu,\"rx_pool_empty\":%s,\"tx_good\":%lu,\"tx_collisions\":%lu,"
                    "\"mode_mismatch\":%s,\"bsr\":%u,\"physcsr\":%u,\"maccr\":%lu}",
                    d->phy_ok ? "true" : "false",
                    (d->bsr & ETH_DIAG_BSR_LINK) ? "true" : "false",
                    (d->bsr & ETH_DIAG_BSR_ANEG_DONE) ? "true" : "false",
                    (d->bsr & ETH_DIAG_BSR_RFAULT) ? "true" : "false",
                    eth_diag_phy_mode(d->physcsr), eth_diag_mac_mode(d->maccr), d->anlpar,
                    d->symbol_errors, d->netif_link ? "true" : "false",
                    (unsigned long)d->rx_frames, (unsigned long)d->rx_bcast, (unsigned long)d->rx_good, (unsigned long)d->rx_crc, (unsigned long)d->rx_align,
                    (unsigned long)d->rx_missed, (unsigned long)d->rx_missed_ovf,
                    (unsigned long)d->rx_alloc_fail, d->rx_alloc_stuck ? "true" : "false",
                    (unsigned long)d->tx_good,
                    (unsigned long)(d->tx_col_single + d->tx_col_multi),
                    mode_mismatch(d) ? "true" : "false", d->bsr, d->physcsr, (unsigned long)d->maccr);
}
