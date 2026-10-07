/* Host-test lwIP options for test_altcp_bp_proxy: just enough of lwIP's headers to compile
   src/altcp_bp_proxy.c against the real altcp structs (no stack, no OS). */
#ifndef TEST_LWIPOPTS_H
#define TEST_LWIPOPTS_H
#define NO_SYS                1
#define SYS_LIGHTWEIGHT_PROT  0
#define LWIP_IPV4             1
#define LWIP_IPV6             0
#define LWIP_TCP              1
#define LWIP_ALTCP            1
#define LWIP_ALTCP_TLS        0
#define LWIP_TCP_KEEPALIVE    0
#define LWIP_NETCONN          0
#define LWIP_SOCKET           0
#define MEM_LIBC_MALLOC       1
#define LWIP_DONT_PROVIDE_BYTEORDER_FUNCTIONS 1   /* the host libc has htons etc. */
#endif
