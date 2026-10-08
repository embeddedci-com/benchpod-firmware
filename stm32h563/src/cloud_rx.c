#include "cloud_rx.h"

#include <string.h>

bool cloud_rx_append(uint8_t *buf, size_t cap, size_t *len, const uint8_t *data, size_t n) {
    if (n > cap - *len) return false;
    memcpy(buf + *len, data, n);
    *len += n;
    return true;
}

void cloud_rx_consume(uint8_t *buf, size_t *len, size_t n) {
    if (n >= *len) { *len = 0; return; }
    memmove(buf, buf + n, *len - n);
    *len -= n;
}

bool cloud_rx_refuse(size_t cap, size_t len, size_t incoming) {
    return incoming > cap - len && len > 0;
}
