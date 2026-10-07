/*
 * altcp_bp_proxy.h: HTTP CONNECT layer for the cloud link (see altcp_bp_proxy.c).
 */
#ifndef ALTCP_BP_PROXY_H
#define ALTCP_BP_PROXY_H

#include "lwip/opt.h"

#if LWIP_ALTCP

#include "lwip/ip_addr.h"
#include "lwip/altcp.h"

struct altcp_bp_proxy_config {
  ip_addr_t   proxy_addr;    /* the proxy */
  u16_t       proxy_port;
  const char *target_host;   /* the server's host name for the CONNECT line */
  const char *auth_b64;      /* base64("user:password") or NULL */
  u16_t       last_status;   /* the proxy's reply code of the last attempt (200 = tunnel up) */
};

/* A proxy layer over a new TCP pcb. Connect it (or a TLS pcb wrapped around it) to the target
   port; the address passed to connect is ignored, the CONNECT line uses target_host. */
struct altcp_pcb *altcp_bp_proxy_new_tcp(struct altcp_bp_proxy_config *config, u8_t ip_type);

#endif /* LWIP_ALTCP */
#endif /* ALTCP_BP_PROXY_H */
