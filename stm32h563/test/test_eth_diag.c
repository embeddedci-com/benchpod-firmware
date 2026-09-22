/* Host test for eth_diag.c: PHY/MAC mode decoding, the mismatch flag, formatting and error deltas. */
#include "eth_diag.h"

#include <stdio.h>
#include <string.h>

static int failures;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d  %s\n", __FILE__, __LINE__, #c); failures++; } } while (0)
#define SCSR(code) ((uint16_t)((code) << ETH_DIAG_SCSR_SPEED_SHIFT))
#define MAC_100F (ETH_DIAG_MACCR_FES | ETH_DIAG_MACCR_DM)

static eth_diag_t healthy(void) {
    eth_diag_t d;
    memset(&d, 0, sizeof(d));
    d.phy_ok = true;
    d.bsr = ETH_DIAG_BSR_LINK | ETH_DIAG_BSR_ANEG_DONE;
    d.physcsr = SCSR(6) | ETH_DIAG_SCSR_AUTODONE;
    d.anlpar = 0x05e1;
    d.maccr = MAC_100F;
    d.netif_link = true;
    return d;
}

int main(void) {
    char buf[512];

    CHECK(strcmp(eth_diag_phy_mode(SCSR(6)), "100M full") == 0);
    CHECK(strcmp(eth_diag_phy_mode(SCSR(2)), "100M half") == 0);
    CHECK(strcmp(eth_diag_phy_mode(SCSR(5)), "10M full") == 0);
    CHECK(strcmp(eth_diag_phy_mode(SCSR(1)), "10M half") == 0);
    CHECK(strcmp(eth_diag_phy_mode(0), "none") == 0);
    CHECK(strcmp(eth_diag_mac_mode(MAC_100F), "100M full") == 0);
    CHECK(strcmp(eth_diag_mac_mode(ETH_DIAG_MACCR_FES), "100M half") == 0);
    CHECK(strcmp(eth_diag_mac_mode(0), "10M half") == 0);

    eth_diag_t d = healthy();
    eth_diag_format(&d, buf, sizeof(buf));
    CHECK(strstr(buf, "phy link up aneg done mode 100M full partner 0x05e1") != NULL);
    CHECK(strstr(buf, "MISMATCH") == NULL);

    /* PHY says 100M full, MAC still at 100M half: flagged in text and JSON. */
    d.maccr = ETH_DIAG_MACCR_FES;
    eth_diag_format(&d, buf, sizeof(buf));
    CHECK(strstr(buf, "mac 100M half (MISMATCH with phy)") != NULL);
    eth_diag_json(&d, buf, sizeof(buf));
    CHECK(strstr(buf, "\"mode_mismatch\":true") != NULL);
    /* No mismatch claimed while the link is down (the PHY's mode is stale then). */
    d.bsr = 0;
    eth_diag_format(&d, buf, sizeof(buf));
    CHECK(strstr(buf, "phy link DOWN aneg NOT DONE") != NULL && strstr(buf, "MISMATCH") == NULL);

    d = healthy();
    d.phy_ok = false;
    eth_diag_format(&d, buf, sizeof(buf));
    CHECK(strncmp(buf, "phy unreadable", 14) == 0);

    /* Error deltas: only growing error counters, including across a wrap. */
    eth_diag_t a = healthy(), b = healthy();
    b.rx_good = 1000;
    CHECK(eth_diag_error_delta(&a, &b, buf, sizeof(buf)) == 0 && buf[0] == '\0');
    b.rx_crc = 3;
    b.rx_missed = 12;
    b.tx_col_single = 2;
    a.symbol_errors = 0xFFFE;
    b.symbol_errors = 1;
    CHECK(eth_diag_error_delta(&a, &b, buf, sizeof(buf)) > 0);
    CHECK(strcmp(buf, "crc +3 missed +12 col +2 symerr +3") == 0);
    a.rx_alloc_fail = 0xFFFFFFFFu;
    b.rx_alloc_fail = 1;
    eth_diag_error_delta(&a, &b, buf, sizeof(buf));
    CHECK(strstr(buf, "allocfail +2") != NULL);

    /* The JSON is one object and the line fits the 384-byte line buffers at worst. */
    d = healthy();
    d.rx_frames = d.rx_bcast = d.rx_good = d.rx_crc = d.rx_align = d.tx_good = d.tx_col_single = d.tx_col_multi = 0xFFFFFFFFu;
    d.rx_missed = d.rx_missed_ovf = d.rx_alloc_fail = 0xFFFFFFFFu;
    d.rx_alloc_stuck = true;
    d.bsr |= ETH_DIAG_BSR_RFAULT;
    d.maccr = 0;
    int n = eth_diag_format(&d, buf, sizeof(buf));
    printf("  worst-case eth diag line: %d bytes\n", n);
    CHECK(n < 384);   /* the net task's and the console's line buffers */
    n = eth_diag_json(&d, buf, sizeof(buf));
    printf("  worst-case eth diag json: %d bytes\n", n);
    CHECK(n < 512 && buf[0] == '{' && buf[n - 1] == '}');

    if (failures) { printf("test_eth_diag: %d FAILURE(S)\n", failures); return 1; }
    printf("test_eth_diag: all passed\n");
    return 0;
}
