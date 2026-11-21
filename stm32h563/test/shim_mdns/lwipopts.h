/* Host-test lwIP options for test_mdns_nsec: a NO_SYS IPv4 + UDP + IGMP stack with lwIP's real
   mDNS responder, driven through a fake netif (the test feeds ip4_input and captures output).
   The mDNS settings match config/lwipopts.h. arch/cc.h comes from shim_altcp. */
#ifndef TEST_MDNS_LWIPOPTS_H
#define TEST_MDNS_LWIPOPTS_H
#define NO_SYS                     1
#define SYS_LIGHTWEIGHT_PROT       0
#define LWIP_IPV4                  1
#define LWIP_IPV6                  0
#define LWIP_TCP                   0
#define LWIP_RAW                   0
#define LWIP_ARP                   0
#define LWIP_ETHERNET              0
#define LWIP_DHCP                  0
#define LWIP_ICMP                  0
#define LWIP_UDP                   1
#define UDP_TTL                    255
#define LWIP_IGMP                  1
#define LWIP_MDNS_RESPONDER        1
#define LWIP_NUM_NETIF_CLIENT_DATA 1
#define MDNS_MAX_SERVICES          2
#define MEMP_NUM_SYS_TIMEOUT       20
#define LWIP_NETCONN               0
#define LWIP_SOCKET                0
#define MEM_LIBC_MALLOC            1
#define MEMP_MEM_MALLOC            1
#define MEM_ALIGNMENT              4
#define CHECKSUM_CHECK_IP          0
#define CHECKSUM_CHECK_UDP         0
#define LWIP_STATS                 0
#define LWIP_RAND()                ((u32_t)rand())
#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS 1   /* the host libc has htons etc. */
#include <stdlib.h>
#include <arpa/inet.h>
#endif
