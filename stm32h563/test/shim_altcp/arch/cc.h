/* Host-test lwIP arch for test_altcp_bp_proxy: an lwIP assert fails the test loudly. */
#ifndef TEST_ARCH_CC_H
#define TEST_ARCH_CC_H
#include <stdio.h>
#include <stdlib.h>
#define LWIP_PLATFORM_DIAG(x)   do { printf x; } while (0)
#define LWIP_PLATFORM_ASSERT(x) do { printf("lwIP assert: %s (%s:%d)\n", x, __FILE__, __LINE__); abort(); } while (0)
#endif
