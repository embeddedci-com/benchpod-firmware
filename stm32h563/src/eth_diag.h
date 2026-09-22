#ifndef ETH_DIAG_H
#define ETH_DIAG_H

/*
 * eth_diag — a snapshot of the wired link for debugging: what the LAN8742 negotiated, what the MAC
 * was programmed to, and the error/drop counters on both. Filled on the net task
 * (ethernetif_diag_refresh); decoding and formatting are pure C, host-tested in test/test_eth_diag.c.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* LAN8742 register bits (datasheet section 4.2). */
#define ETH_DIAG_BSR_LINK        (1u << 2)    /* BSR: link up (latched low) */
#define ETH_DIAG_BSR_RFAULT      (1u << 4)    /* BSR: remote fault */
#define ETH_DIAG_BSR_ANEG_DONE   (1u << 5)    /* BSR: autonegotiation complete */
#define ETH_DIAG_SCSR_AUTODONE   (1u << 12)   /* PHYSCSR: autonegotiation done */
#define ETH_DIAG_SCSR_SPEED_SHIFT 2u          /* PHYSCSR [4:2]: 1=10H 5=10F 2=100H 6=100F */
#define ETH_DIAG_SCSR_SPEED_MASK (7u << ETH_DIAG_SCSR_SPEED_SHIFT)
/* STM32H5 ETH_MACCR bits. */
#define ETH_DIAG_MACCR_DM        (1u << 13)   /* full duplex */
#define ETH_DIAG_MACCR_FES       (1u << 14)   /* 100 Mbit/s */

typedef struct {
    bool     phy_ok;          /* the MDIO reads succeeded */
    uint16_t bsr;             /* PHY basic status */
    uint16_t physcsr;         /* PHY special control/status: the resolved speed/duplex */
    uint16_t anlpar;          /* link partner ability */
    uint16_t symbol_errors;   /* PHY symbol error counter (wraps) */
    uint32_t maccr;           /* MAC configuration: the speed/duplex the MAC actually uses */
    bool     netif_link;      /* lwIP's view of the link */
    uint32_t rx_frames;       /* driver: every frame handed to lwIP (unicast, broadcast, multicast) */
    uint32_t rx_bcast;        /* driver: of those, broadcast (ARP, DHCP offers on most routers) */
    uint32_t rx_good;         /* MMC: good unicast frames received */
    uint32_t rx_crc;          /* MMC: frames received with a CRC error */
    uint32_t rx_align;        /* MMC: frames received with an alignment error */
    uint32_t tx_good;         /* MMC: good frames sent */
    uint32_t tx_col_single;   /* MMC: sent after one collision (only possible at half duplex) */
    uint32_t tx_col_multi;    /* MMC: sent after several collisions */
    uint32_t rx_missed;       /* DMA: frames dropped because no RX descriptor was free */
    uint32_t rx_missed_ovf;   /* DMA: times that counter overflowed */
    uint32_t rx_alloc_fail;   /* driver: RX buffer pool empty (receive stalls until one frees) */
    bool     rx_alloc_stuck;  /* the pool is empty right now */
} eth_diag_t;

/* "100M full", "10M half", ... from PHYSCSR; "none" without a resolved mode. */
const char *eth_diag_phy_mode(uint16_t physcsr);
/* The same for the MAC configuration register. */
const char *eth_diag_mac_mode(uint32_t maccr);

/* One human line: link state, both modes, partner ability and every counter. */
int eth_diag_format(const eth_diag_t *d, char *buf, size_t cap);

/* The error counters that grew since `prev` ("crc +3 missed +12 ..."), or 0 when none did.
   Counters are free-running and may wrap; the unsigned difference handles that. */
int eth_diag_error_delta(const eth_diag_t *prev, const eth_diag_t *cur, char *buf, size_t cap);

/* The snapshot as a JSON object (no surrounding whitespace). */
int eth_diag_json(const eth_diag_t *d, char *buf, size_t cap);

#endif /* ETH_DIAG_H */
