/* sd_wasm.h: the ABI of sd_wasm.c (README.md).  In the WASM module the functions are exported
 * under the names in brackets; on the host (golden_host.c) they are plain C functions. */
#pragma once
#include <stdint.h>

void       *sdw_alloc(uint32_t n);                                   /* [sd_alloc] */
void        sdw_free(void *p);                                       /* [sd_free] */
const char *sdw_out_ptr(void);                                       /* [out_ptr] */
uint32_t    sdw_abi_version(void);                                   /* [abi_version] */
uint32_t    sdw_validate(const char *def, uint32_t def_len);         /* [validate] */
uint32_t    sdw_preview(const char *def, uint32_t def_len, const char *vals, uint32_t vals_len,
                        const uint8_t *live, uint32_t live_len, double t_ms, double input);   /* [preview] */
uint32_t    sdw_uart_answer(const char *def, uint32_t def_len, const char *vals, uint32_t vals_len,
                            const uint8_t *req, uint32_t req_len, double t_ms);              /* [uart_answer] */
uint32_t    sdw_stimulus_validate(const char *json, uint32_t len);   /* [stimulus_validate] */
uint32_t    sdw_stimulus_sample(const char *json, uint32_t len, double from_s, double to_s,
                                double step_s);                      /* [stimulus_sample] */
