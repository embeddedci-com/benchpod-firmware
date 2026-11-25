#include "lan_tls_resume.h"
#include "mbedtls/ssl_cache.h"
#include "mbedtls/ssl_ticket.h"

/* The session carries a client certificate (what the pairing check needs after the resume). */
static int has_peer_cert(const mbedtls_ssl_session *s)
{
#if defined(MBEDTLS_X509_CRT_PARSE_C) && defined(MBEDTLS_SSL_KEEP_PEER_CERTIFICATE)
    return s->MBEDTLS_PRIVATE(peer_cert) != NULL;
#else
    (void)s;
    return 0;
#endif
}

/* The caller frees `session` on any non-zero return (ssl_parse_session_ticket_ext,
   ssl_handle_id_based_session_resumption), so a refusal needs no cleanup here. */
int lan_tls_ticket_parse(void *p_ticket, mbedtls_ssl_session *session, unsigned char *buf, size_t len)
{
    int ret = mbedtls_ssl_ticket_parse(p_ticket, session, buf, len);
    if (ret == 0 && !has_peer_cert(session)) return MBEDTLS_ERR_SSL_SESSION_TICKET_EXPIRED;
    return ret;
}

int lan_tls_cache_get(void *data, const unsigned char *session_id, size_t session_id_len,
                      mbedtls_ssl_session *session)
{
    int ret = mbedtls_ssl_cache_get(data, session_id, session_id_len, session);
    if (ret == 0 && !has_peer_cert(session)) return 1;   /* 1: not found */
    return ret;
}
