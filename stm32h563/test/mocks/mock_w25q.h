/* RAM model of the W25Q for host tests: NOR semantics (erase sets 0xFF, program can only clear
   bits; programming a bit that is already 0 back to 1 is counted as a bug), a JEDEC ID, and a
   power cut after N flash operations (each page program and each erase is one), delivered by
   longjmp to mock_w25q_power_jmp. */
#ifndef MOCK_W25Q_H
#define MOCK_W25Q_H

#include <stdint.h>
#include <setjmp.h>

#define MOCK_W25Q_BYTES (8u * 1024u * 1024u)

extern uint8_t mock_w25q[MOCK_W25Q_BYTES];
extern uint8_t mock_w25q_id[3];
extern int     mock_w25q_ops;              /* erases + page programs so far */
extern int     mock_w25q_bad_programs;     /* programs that tried to set a 0 bit to 1 */
extern int     mock_w25q_cut_after;        /* power cut after this many more ops (0 = off) */
extern int     mock_w25q_fail_read;        /* make every read fail */
extern int     mock_w25q_open_depth;
extern int     mock_w25q_sessions;        /* w25q_session_open calls (quiesce + open) */
extern jmp_buf mock_w25q_power_jmp;

void mock_w25q_reset(void);               /* erased, W25Q64 ID, counters cleared */

#endif /* MOCK_W25Q_H */
