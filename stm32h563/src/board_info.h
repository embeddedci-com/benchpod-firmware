#pragma once

/* board_info.h — board / ADC metadata reported to the cloud.
 *
 * The retired Raspberry Pi agent used a Rigol scope whose volts/timebase came
 * straight off the instrument. The networked pod instead returns raw ADC counts,
 * and the cloud server (embeddedci-server/api/benchpod_capture.go) scales them to
 * volts. So the pod advertises its ADC resolution, full-scale span and channel
 * count in the `capabilities` frame and in `status`, and the server scales per
 * device (cap.adc_bits / cap.adc_fullscale_mv / cap.adc_channels / cap.board).
 *
 * These are board-nominal, uncalibrated defaults; a calibrated pod can refine
 * ADC_FULLSCALE_MV from its calibration record.
 */

#define BOARD_NAME "stm32h563"

/* v2 analog front-end: MCP33131D-10, a 16-bit serial SAR ADC sampled through the
 * iCE40 FPGA, sharing a 4.096 V precision reference with the 16-bit DAC (see
 * docs/v1-dac-adc-loopback.md — "16-bit DAC + ADC with a shared 4.096 V precision
 * reference").  So counts 0..65535 span ~4096 mV.  Single capture channel (the
 * SMA input). */
#define ADC_BITS         16
#define ADC_CHANNELS     1
#define ADC_FULLSCALE_MV 4096

/* DAC capability reported to the cloud (server persists cap.dac_*; the webapp
 * gates the signal-generator UI on it). v2 has the FPGA AC waveform DAC sequencer
 * wired, so AC generate + arbitrary replay work, and it can hold a DC level.
 * The DAC hardware is 16-bit (shared 4.096 V reference with the ADC), but the
 * current JSON generate/load/replay interface drives it with 8-bit codes; this
 * advertises that effective interface depth (Phase 6 raises it to 16). */
#define DAC_AC           1      /* 1 = FPGA AC waveform DAC (generate/replay) */
#define DAC_REPLAY       1      /* 1 = arbitrary replay via the FPGA DAC8551 sequencer */
#define DAC_DC           1      /* 1 = settable DC level (dac_set) */
#define DAC_BITS         8      /* resolution of the parametric generate interface */
/* Arbitrary load/replay is 16-bit on v2: handle_load/handle_replay/load_bin stream
 * 16-bit samples into adc_buf16 and the FPGA DAC8551 sequencer clocks 16-bit words. */
#define DAC_REPLAY_BITS  16     /* resolution of arbitrary load/replay samples */
#define DAC_CHANNELS     1
#define DAC_FULLSCALE_MV 4096
