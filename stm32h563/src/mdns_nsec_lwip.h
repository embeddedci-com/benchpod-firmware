#ifndef MDNS_NSEC_LWIP_H
#define MDNS_NSEC_LWIP_H

/* Hook mdns_nsec_reply() (mdns_nsec.h) into lwIP's mDNS responder so AAAA questions for the
   pod's name get an NSEC answer. Call once, right after mdns_resp_init(). Net task. */
void mdns_nsec_install(void);

#endif
