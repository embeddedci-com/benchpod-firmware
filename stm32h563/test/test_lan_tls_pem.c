/* Host test for lan_tls_pem_split (src/lan_tls_pem.c): an uploaded bundle splits into its one
   private key and its certificates, in any order, and bad bundles are refused with a reason. */
#include "lan_tls_pem.h"

#include <stdio.h>
#include <string.h>

static int fails = 0;
#define CHECK(cond, ...) do { if (!(cond)) { \
        printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); fails++; } } while (0)

#define CERT1 "-----BEGIN CERTIFICATE-----\nAAAA\nBBBB\n-----END CERTIFICATE-----\n"
#define CERT2 "-----BEGIN CERTIFICATE-----\nCCCC\n-----END CERTIFICATE-----\n"
#define ECKEY "-----BEGIN EC PRIVATE KEY-----\nKKKK\n-----END EC PRIVATE KEY-----\n"
#define P8KEY "-----BEGIN PRIVATE KEY-----\nPPPP\n-----END PRIVATE KEY-----\n"

static char key[512], crt[512];
static size_t kl, cl;

static const char *split(const char *in)
{
    memset(key, 0x55, sizeof(key));
    memset(crt, 0x55, sizeof(crt));
    kl = cl = 0;
    return lan_tls_pem_split(in, strlen(in), key, sizeof(key), &kl, crt, sizeof(crt), &cl);
}

int main(void)
{
    const char *why = split(CERT1 CERT2 ECKEY);
    CHECK(!why, "cert+chain+key refused: %s", why);
    CHECK(strcmp(key, ECKEY) == 0, "key: '%s'", key);
    CHECK(strcmp(crt, CERT1 CERT2) == 0, "certs: '%s'", crt);
    CHECK(kl == strlen(ECKEY) + 1 && cl == strlen(CERT1 CERT2) + 1, "lengths %zu %zu include the NUL", kl, cl);

    why = split(P8KEY "\r\n" "-----BEGIN CERTIFICATE-----\r\nAAAA\r\n-----END CERTIFICATE-----\r\n");
    CHECK(!why, "key first, CRLF refused: %s", why);
    CHECK(strcmp(key, P8KEY) == 0, "PKCS#8 key: '%s'", key);
    CHECK(strcmp(crt, "-----BEGIN CERTIFICATE-----\nAAAA\n-----END CERTIFICATE-----\n") == 0, "CRLF cert: '%s'", crt);

    why = split("junk before\n" CERT1 "-----BEGIN EC PARAMETERS-----\nXX\n-----END EC PARAMETERS-----\n" ECKEY);
    CHECK(!why && strcmp(key, ECKEY) == 0 && strcmp(crt, CERT1) == 0, "other blocks not ignored: %s", why);

    why = split(CERT1);
    CHECK(why && strstr(why, "no private key"), "missing key: %s", why);
    why = split(ECKEY);
    CHECK(why && strstr(why, "no certificate"), "missing cert: %s", why);
    why = split(CERT1 ECKEY P8KEY);
    CHECK(why && strstr(why, "more than one"), "two keys: %s", why);
    why = split(CERT1 "-----BEGIN ENCRYPTED PRIVATE KEY-----\nEEEE\n-----END ENCRYPTED PRIVATE KEY-----\n");
    CHECK(why && strstr(why, "encrypted"), "PKCS#8 encrypted: %s", why);
    why = split(CERT1 "-----BEGIN RSA PRIVATE KEY-----\nProc-Type: 4,ENCRYPTED\nDEK-Info: AES\n\nEEEE\n-----END RSA PRIVATE KEY-----\n");
    CHECK(why && strstr(why, "encrypted"), "legacy encrypted: %s", why);
    why = split(CERT1 "-----BEGIN EC PRIVATE KEY-----\nKKKK\n");
    CHECK(why && strstr(why, "END"), "unterminated: %s", why);

    /* Too large for the buffers: refused, never overflowed. */
    char big[2048] = CERT1;
    while (strlen(big) + sizeof(CERT2) + sizeof(ECKEY) < sizeof(big)) strcat(big, CERT2);
    strcat(big, ECKEY);
    why = split(big);
    CHECK(why && strstr(why, "too large"), "oversize certs: %s", why);
    CHECK((unsigned char)crt[sizeof(crt) - 1] == 0x55 || crt[sizeof(crt) - 1] == '\n' || crt[sizeof(crt) - 1] == 0,
          "wrote past the end");

    if (fails) { printf("test_lan_tls_pem: %d FAILED\n", fails); return 1; }
    printf("test_lan_tls_pem: all passed\n");
    return 0;
}
