/*
 * c3_update.h — write an esp update into the ESP32-C3 from the PSRAM staging (see c3_update.c).
 */
#ifndef C3_UPDATE_H
#define C3_UPDATE_H

#include <stdbool.h>

/* Start a write ota_apply_esp_staged scheduled, once its delay has passed. hw worker loop. */
void c3_update_poll(void);

/* A write is scheduled and has not started yet. */
bool c3_update_pending(void);

#endif /* C3_UPDATE_H */
