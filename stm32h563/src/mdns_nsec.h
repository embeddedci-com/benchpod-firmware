#ifndef MDNS_NSEC_H
#define MDNS_NSEC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ---- mDNS negative answers (RFC 6762 section 6.1), host-testable ------------
 *
 * The pod is IPv4-only. lwIP's mDNS responder answers the A question for
 * <host>.local but stays silent on AAAA, so a resolver that asks for both
 * (macOS getaddrinfo with AF_UNSPEC) waits about 5 s for an AAAA answer that
 * never comes. RFC 6762 section 6.1: a responder that owns a name but has no
 * record of the asked type answers with an NSEC record for the name listing
 * the types it does have. Here that is A only.
 *
 * mdns_nsec_reply() parses one mDNS message (no lwIP) and, when it is a query
 * with an AAAA question for <host>.local, builds the reply carrying that NSEC.
 * mdns_nsec_lwip.c hooks it in front of lwIP's mDNS receive callback.
 *
 *   legacy: the query came from a port other than 5353 (RFC 6762 section 6.7,
 *           a one-shot unicast resolver): the reply echoes the query id and the
 *           question, uses a TTL of 10 s and no cache-flush bit, and goes back
 *           to the sender by unicast. Otherwise the reply has id 0, no question
 *           section, and is sent to the mDNS multicast group.
 *
 * Returns the reply length, or 0 when nothing should be sent (not a query,
 * malformed, no AAAA question for host, or cap too small). host is the bare
 * label ("benchpod-a1b2c3"); ".local" is implied. Names compare ignoring case.
 * ---------------------------------------------------------------------------*/

#define MDNS_NSEC_TTL         120u  /* same as lwIP's host record TTL */
#define MDNS_NSEC_LEGACY_TTL   10u  /* RFC 6762 section 6.7 */
/* Reply length for a host label of n bytes: 41 + 2n, or 53 + 3n for legacy (242 at n = 63). */
#define MDNS_NSEC_REPLY_MAX   256u

size_t mdns_nsec_reply(const uint8_t *msg, size_t len, const char *host, bool legacy,
                       uint8_t *out, size_t cap);

#endif
