/*
 * lwipopts.h — NO_SYS=1 (bare raw API) config for the bench-pod, adapted from
 * ST's NUCLEO-H563ZI LwIP_TCP_Echo_Server reference.
 *
 * Difference from the reference: the LWIP_RAM_HEAP_POINTER relocation is dropped
 * — the lwIP MEM heap is a normal .bss array.  On STM32H5 the ETH DMA can reach
 * all SRAM, so no special placement is needed, and this avoids depending on a
 * hardcoded absolute SRAM address matching our linker layout.
 */
#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

#define SYS_LIGHTWEIGHT_PROT    0
#define NO_SYS                  1

/* ---------- Memory ---------- */
#define MEM_ALIGNMENT           4
/* The lwIP `mem` heap ALSO backs mbedTLS (altcp_tls with ALTCP_MBEDTLS_PLATFORM_ALLOC=1
   routes every mbedTLS calloc through mem_malloc, and rejects any single alloc
   > MEM_SIZE).  A TLS session needs the 16 KB SSL IN buffer + 4 KB OUT buffer +
   context + handshake temporaries (cert chain, bignums) concurrently, so 16 KB
   is far too small — altcp_tls_new() failed with "out of mem".  64 KB holds a
   session with headroom for in-flight pbufs. */
#define MEM_SIZE                (64 * 1024)

#define MEMP_NUM_TCP_PCB        10
#define MEMP_NUM_TCP_SEG        TCP_SND_QUEUELEN
/* lwIP's default MEMP_NUM_SYS_TIMEOUT counts only the core stack timeouts
   (TCP/ARP/DHCP/DNS/IGMP) and NOT the mDNS responder, which schedules its own
   (probe/announce/delayed multicast reply).  With DHCP + mDNS both active the
   default pool is exhausted -> "MEMP_SYS_TIMEOUT is empty" assert.  Size it
   explicitly with headroom.
   12 proved marginal: on an Ethernet LINK BOUNCE lwIP re-arms a burst all at
   once — DHCP restart (coarse+fine+retries) + IGMP re-join of 224.0.0.251 +
   ACD re-probe + mDNS re-probe/announce — which transiently exceeds 12 and
   trips the (non-fatal) assert, silently dropping one timeout.  20 covers the
   worst-case link-up peak (~224 B RAM). */
#define MEMP_NUM_SYS_TIMEOUT    20

/* ---------- Pbuf ---------- */
#define PBUF_POOL_BUFSIZE       1536
#define LWIP_SUPPORT_CUSTOM_PBUF 1

/* ---------- Network interface ---------- */
#define LWIP_NETIF_LINK_CALLBACK 1

/* ---------- TCP ---------- */
#define LWIP_TCP                1
#define TCP_TTL                 255
#define TCP_MSS                 (1500 - 40)
/* Receive/send windows: 8 segments (~11.7 KB). Cloud throughput is bandwidth-delay-product
   limited (RTT to the Cloudflare edge ~42 ms, so throughput ≈ window / RTT ≈ 160-208 KB/s),
   but raising the windows destabilizes given the tight RAM: a bigger TCP_WND overflows the
   16 KB cloud RX accumulator (BP_CLOUD_RX_ACCUM) on a server->device burst, and a bigger
   TCP_SND_BUF scales MEMP_NUM_TCP_SEG and pressures the 64 KB MEM_SIZE (shared with the
   ~25 KB TLS session), starving download-path allocations -> reconnects. Both HW-observed.
   So the windows stay at 8 segments (the proven-stable value); real throughput gains need a
   larger MEM_SIZE (no bss headroom today), a 2nd bulk WS, or device-pull HTTP. See
   [[cloud-bulk-ws-and-speedtest]]. The 4-segment (5840 B) window was even worse (a deep
   DAC-replay upload crawled); 8 is the stable sweet spot. */
#define TCP_SND_BUF             (8 * TCP_MSS)
#define TCP_WND                 (8 * TCP_MSS)

/* Keepalive (enabled per LAN connection in srv_accept): a client that sleeps, loses its cable or
   crashes while idle held its slot forever, with any UART proxy or SWD session on it, and five
   of those locked the LAN out until reboot.  30 s idle, then 4 probes 5 s apart: ~50 s. */
#define LWIP_TCP_KEEPALIVE      1
#define TCP_KEEPIDLE_DEFAULT    30000
#define TCP_KEEPINTVL_DEFAULT   5000
#define TCP_KEEPCNT_DEFAULT     4

/* The Ethernet driver receives into a small zero-copy pool shared with TCP's out-of-order queue
   and IP reassembly (lwIP defaults: unbounded / 10 pbufs for up to 15 s).  Junk fragments or a
   lossy window could hold the pool and stall receive for seconds.  Cap both. */
#define TCP_OOSEQ_MAX_PBUFS     4
#define IP_REASS_MAX_PBUFS      4
#define MEMP_NUM_REASSDATA      2   /* <= IP_REASS_MAX_PBUFS / 2 (lwIP init.c check) */

/* ---------- ICMP / DHCP / UDP / DNS ---------- */
#define LWIP_ICMP               1
#define LWIP_DHCP               1
/* Skip DHCP Address-Conflict-Detection (ARP probe of the offered address before
   binding). Defaults ON with LWIP_DHCP, but on a managed network the DHCP server
   owns address uniqueness, and the ACD probe only adds ~1-2 s of latency and can
   spuriously DECLINE a valid lease on a marginal/lossy link (e.g. Wi-Fi) or when
   the re-kick restarts dhcp mid-probe. Off = faster, more robust acquisition.
   This also stops dhcp_start() re-adding the ACD struct to lwIP's list on every
   re-kick. Applies to both the wired and Wi-Fi netifs. */
#define LWIP_DHCP_DOES_ACD_CHECK  0
#define LWIP_UDP                1
#define UDP_TTL                 255

/* ---------- mDNS / DNS-SD responder (benchpod.local auto-discovery) ----------
   Advertises "_benchpod._tcp" so a client can browse the LAN and enumerate
   every pod at once. The responder needs IPv4 multicast (it joins 224.0.0.251,
   so LWIP_IGMP=1 + NETIF_FLAG_IGMP on each netif) and one netif client-data
   slot per interface. Source: lib/stm32_mw_lwip/src/apps/mdns. */
#define LWIP_IGMP                  1   /* also makes IP_OPTIONS_SEND derive to 1 so IGMP reports carry a router-alert option */
#define LWIP_MDNS_RESPONDER        1
#define LWIP_NUM_NETIF_CLIENT_DATA 1
#define MDNS_MAX_SERVICES          1
#define MEMP_NUM_UDP_PCB           5   /* DHCP + cloud DNS + 1 global mDNS pcb */

/* DNS: needed so the cloud client can resolve the server hostname (the RP2350
   offloaded this to the ESP-AT modem; here lwIP does it).  DHCP supplies the
   resolver address. */
#define LWIP_DNS                1
#define DNS_TABLE_SIZE          4
#define DNS_MAX_NAME_LENGTH     256

/* ---------- altcp + TLS (outbound WSS cloud client) ---------- */
/* altcp gives a pluggable transport layer; altcp_tls_mbedtls wraps it with
   mbedTLS so cloud_client.c can open a wss:// connection.  See
   config/mbedtls_bench_config.h and src/mbedtls_port.c. */
#define LWIP_ALTCP              1
#define LWIP_ALTCP_TLS          1
#define LWIP_ALTCP_TLS_MBEDTLS  1

#define LWIP_STATS              0

/* Checksums are selected PER-NETIF (LWIP_CHECKSUM_CTRL_PER_NETIF): the STM32H5
   ETH MAC inserts IP/UDP/TCP checksums in hardware (ethernetif.c TxConfig), but
   the Wi-Fi path (ESP32-C3 over SPI) has NO checksum hardware. So the software
   checksum routines are compiled in (GEN_*=1) and then disabled on the eth netif
   / enabled on the wifi netif at netif_add time (net_server.c, esp_netif.c).
   Without this the wifi netif emits a ZERO IP checksum and every AP silently
   drops it — the DHCP DISCOVER is never answered (confirmed on-air). RX checks
   stay off on both (CHECK_*=0): eth is hardware-verified, wifi is trusted. */
#define LWIP_CHECKSUM_CTRL_PER_NETIF 1
#define CHECKSUM_GEN_IP        1
#define CHECKSUM_GEN_UDP       1
#define CHECKSUM_GEN_TCP       1
#define CHECKSUM_GEN_ICMP      1
#define CHECKSUM_CHECK_IP      0
#define CHECKSUM_CHECK_UDP     0
#define CHECKSUM_CHECK_TCP     0
#define CHECKSUM_CHECK_ICMP    0

/* ---------- Sequential / socket layers (unused: raw API only) ---------- */
#define LWIP_NETCONN            0
#define LWIP_SOCKET             0

/* ---------- Routing ---------- */
/* Source-based route choice (net_server.c, logic in net_route.c): a reply leaves
   the interface that owns its source address, and same-subnet traffic with no
   bound source prefers eth. Without it lwIP picks the first netif on the subnet,
   which is Wi-Fi (netif_add prepends), so eth-arrived TCP replied over Wi-Fi. */
struct netif;
struct ip4_addr;
struct netif *net_route_src_hook(const struct ip4_addr *src, const struct ip4_addr *dest);
#define LWIP_HOOK_IP4_ROUTE_SRC(src, dest) net_route_src_hook((src), (dest))

#endif /* __LWIPOPTS_H__ */
