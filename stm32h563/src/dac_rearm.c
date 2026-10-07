#include "dac_rearm.h"

int dac_rearm_quiesce(const dac_rearm_ops_t *ops, bool force, bool cotrig_pending) {
    uint8_t st = 0;
    if (!force && !(ops->status_read(&st) == 0 && (st & DAC_REARM_STATUS_DAC_RUN)))
        return 0;                                   /* nothing running: no stop, no extra bus time */
    ops->stop();
    int rc = -1;
    for (unsigned i = 0; i < DAC_REARM_MAX_POLLS; i++) {
        if (ops->status_read(&st) == 0 && !(st & DAC_REARM_STATUS_DAC_RUN)) { rc = 0; break; }
    }
    if (cotrig_pending) (void)ops->arm_cotrig();    /* STOP_DAC cancelled it */
    return rc;
}
