#ifndef MBEDTLS_PORT_H
#define MBEDTLS_PORT_H

/* Route mbedTLS heap allocation to the FreeRTOS allocator.  Call once, before
   any TLS configuration/handshake (see net_init()).  Entropy is wired up
   separately via mbedtls_hardware_poll() (MBEDTLS_ENTROPY_HARDWARE_ALT). */
void mbedtls_port_init(void);

#endif /* MBEDTLS_PORT_H */
