#ifndef ALTCP_TLS_BP_H
#define ALTCP_TLS_BP_H
/* BenchPod additions to lwIP's altcp_tls_mbedtls (see altcp_tls_mbedtls_bp.c). */
#include "lwip/altcp.h"

/* Offer plaintext the recv callback refused (its buffer was full) to it again, once it has room.
   lwIP context only (the net task). */
void altcp_tls_bp_kick(struct altcp_pcb *conn);

struct altcp_tls_config;
/* Give a listening TLS pcb another configuration: connections accepted from now on use it, open
   ones keep theirs. Lets the LAN TLS listener take a new certificate without closing and binding
   :8443 again, which lwIP refuses while a connection still has that local port. lwIP context. */
void altcp_tls_bp_set_listen_config(struct altcp_pcb *listen_conn, struct altcp_tls_config *conf);

#endif
