#include "hw_lock.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* Recursive mutex.  Since the hw-worker task rewrite, ALL instrument-hardware
   access (iCE40 SPI, I2C, XSPI, CAN, target power) happens on the single worker
   task — the net task owns only lwIP + the ESP co-processor and never touches the
   instrument buses.  So this lock is effectively uncontended; it is kept (and
   made recursive) purely as defensive insurance and so the existing leaf-level
   hw_lock()/hw_unlock() pairs in signal_engine.c / target_power.c can nest freely
   under a command dispatch that may also hold it, without risking the
   self-deadlock a plain mutex would cause. */
static SemaphoreHandle_t s_mtx;

void hw_lock_init(void) { s_mtx = xSemaphoreCreateRecursiveMutex(); }
void hw_lock(void)      { if (s_mtx) xSemaphoreTakeRecursive(s_mtx, portMAX_DELAY); }
void hw_unlock(void)    { if (s_mtx) xSemaphoreGiveRecursive(s_mtx); }
