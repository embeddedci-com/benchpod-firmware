#ifndef ALTCP_TLS_BP_H
#define ALTCP_TLS_BP_H
/* BenchPod additions to lwIP's altcp_tls_mbedtls (see altcp_tls_mbedtls_bp.c). */
#include "lwip/altcp.h"

/* Offer plaintext the recv callback refused (its buffer was full) to it again, once it has room.
   lwIP context only (the net task). */
void altcp_tls_bp_kick(struct altcp_pcb *conn);

#endif
