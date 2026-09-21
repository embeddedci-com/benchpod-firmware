#ifndef HW_LOCK_H
#define HW_LOCK_H

/* Serializes access to the shared instrument hardware (iCE40 SPI, I2C) between
   the local console task and the TCP command server (net task).  Create the
   mutex once before either task runs. */
void hw_lock_init(void);
void hw_lock(void);
void hw_unlock(void);

#endif /* HW_LOCK_H */
