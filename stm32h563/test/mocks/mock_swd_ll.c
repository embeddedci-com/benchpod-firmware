#include "mock_swd_ll.h"
#include "swd_ll.h"
#include <string.h>

uint8_t  mock_swd_ll_ack;
uint32_t mock_swd_ll_dp_value;
uint32_t mock_swd_ll_ap_seq[64];
int      mock_swd_ll_ap_seq_len;
int      mock_swd_ll_wait_before_ok;
uint32_t mock_swd_ll_writes[256];
int      mock_swd_ll_writes_n;
int      mock_swd_ll_xfer_calls;
int      mock_swd_ll_nreset_calls;
bool     mock_swd_ll_last_nreset;

static uint32_t s_posted;
static bool     s_have_posted;
static int      s_ap_idx;

void mock_swd_ll_reset(void) {
    mock_swd_ll_ack = SWD_ACK_OK;
    mock_swd_ll_dp_value = 0;
    memset(mock_swd_ll_ap_seq, 0, sizeof(mock_swd_ll_ap_seq));
    mock_swd_ll_ap_seq_len = 0;
    mock_swd_ll_wait_before_ok = 0;
    memset(mock_swd_ll_writes, 0, sizeof(mock_swd_ll_writes));
    mock_swd_ll_writes_n = 0;
    mock_swd_ll_xfer_calls = 0;
    mock_swd_ll_nreset_calls = 0;
    mock_swd_ll_last_nreset = false;
    s_posted = 0; s_have_posted = false; s_ap_idx = 0;
}

/* config / sequence calls are recorded only where a test needs them */
void swd_ll_reset(void) {}
void swd_ll_configure(uint8_t turnaround, bool data_phase) { (void)turnaround; (void)data_phase; }
void swd_ll_set_idle(uint8_t idle_cycles) { (void)idle_cycles; }
void swd_ll_seq_out(const uint8_t *data, uint32_t nbits) { (void)data; (void)nbits; }
void swd_ll_seq_in(uint8_t *data, uint32_t nbits) { (void)data; (void)nbits; }

void swd_ll_nreset(bool asserted) {
    mock_swd_ll_nreset_calls++;
    mock_swd_ll_last_nreset = asserted;
}

uint8_t swd_ll_transfer(uint8_t request, uint32_t *data) {
    mock_swd_ll_xfer_calls++;

    if (mock_swd_ll_wait_before_ok > 0) {
        mock_swd_ll_wait_before_ok--;
        return SWD_ACK_WAIT;
    }
    if (mock_swd_ll_ack != SWD_ACK_OK)
        return mock_swd_ll_ack;

    if (request & SWD_REQ_RnW) {
        bool is_rdbuff = (request & (SWD_REQ_A2 | SWD_REQ_A3)) == (SWD_REQ_A2 | SWD_REQ_A3)
                         && !(request & SWD_REQ_APnDP);
        if (request & SWD_REQ_APnDP) {
            /* posted AP read: return previous, latch next */
            uint32_t ret = s_have_posted ? s_posted : 0;
            s_posted = (s_ap_idx < mock_swd_ll_ap_seq_len)
                       ? mock_swd_ll_ap_seq[s_ap_idx++] : 0;
            s_have_posted = true;
            if (data) *data = ret;
        } else if (is_rdbuff) {
            if (data) *data = s_have_posted ? s_posted : 0;
        } else {
            if (data) *data = mock_swd_ll_dp_value;
        }
    } else {
        if (mock_swd_ll_writes_n < (int)(sizeof(mock_swd_ll_writes) / sizeof(uint32_t)))
            mock_swd_ll_writes[mock_swd_ll_writes_n++] = data ? *data : 0;
    }
    return SWD_ACK_OK;
}
