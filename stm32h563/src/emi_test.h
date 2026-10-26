#ifndef EMI_TEST_H
#define EMI_TEST_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/*
 * emi_test — the EMC pre-compliance build (`make EMI_TEST=1`, never with RELEASE=1).
 *
 * From boot the pod runs the worst-case activity of the lab's operating mode
 * (benchpod-compliance emc/test-setup.md, "Operating mode") by itself, so a spectrum analyser can
 * watch it without a laptop driving the pod:
 *
 *   - LA voltage 3.3 V, target power on at both eFuses (J5 internal 5 V, J6 from J4)
 *   - the DAC playing one output: a 1 kHz sine on 0-3.3 V by default, or DC on 0-5 V, 12 mA on
 *     the 4-20 mA output, or off. One at a time: the analog v3r1 board has one DAC behind muxes,
 *     and the analog paths keep that meaning on the quad-DAC boards too
 *   - back-to-back deep captures, ADC + LA and LA only in turn, each read back over the PSRAM bus
 *     and formatted like a client's, then thrown away
 *   - optionally a 115200 baud NMEA stream out of one LA pin (`emi uart <la>`), so the pod's own
 *     LA drivers toggle; off by default, since it drives a pin of the DUT
 *
 * The network side runs as usual (cloud, LAN, USB network); Ethernet traffic comes from the
 * laptop (the measurement script pings the pod).
 *
 * The loop drives the pod with its own JSON commands on the pseudo-connection CH_EMI_CONN,
 * whose replies go to emi_test_reply() instead of a socket: every check and gate a client meets
 * applies here too. A heavy command from a client meanwhile gets "busy"; `emi stop` hands the
 * pod back. All of it runs on the hw worker (console commands do too).
 *
 * The USB console prints a monitor line every EMI_TEST_REPORT_MS and `emi` shows the state.
 */

/* Wait this long after boot before starting, so the network and cloud link come up first. */
#define EMI_TEST_START_DELAY_MS  10000u
/* A monitor line on the USB console this often. */
#define EMI_TEST_REPORT_MS       5000u

typedef enum {
    EMI_DAC_SINE = 0,   /* 1 kHz sine, 0.15-3.15 V on the 0-3.3 V output (to the BNC) */
    EMI_DAC_DC,         /* 3.0 V DC on the 0-5 V output (J11 to J8: 12 mA in the loop input) */
    EMI_DAC_CURRENT,    /* 12 mA held on the 4-20 mA output (J9) */
    EMI_DAC_OFF,
} emi_dac_mode_t;

/* Hw worker loop, every pass: start after the boot delay, then keep the activity running. */
void emi_test_poll(void);

/* Replies to the loop's commands (at_send_data for CH_EMI_CONN): counts errors, keeps the last. */
void emi_test_reply(const uint8_t *buf, size_t len);
/* What a bulk read-back may still send to CH_EMI_CONN in this worker pass (at_send_avail): a
   budget per pass, so a deep read-back is spread over passes like a real socket's. */
size_t emi_test_avail(void);

/* Console `emi start|stop`: start the activity now / stop it and switch outputs and power off. */
void emi_test_start(void);
void emi_test_stop(void);
bool emi_test_running(void);
/* Console `emi dac <mode>`: change the DAC activity (applied at once when running). */
void emi_test_set_dac(emi_dac_mode_t mode);
/* Console `emi uart <1-14>|off`: NMEA out of an LA pin while running; 0 = off. */
void emi_test_set_uart(int la);
/* "sine", "dc", "current", "off"; -1 for anything else. */
int  emi_test_dac_from_name(const char *name);

/* One-line state for the console (`emi`) and the monitor line. */
size_t emi_test_describe(char *buf, size_t cap);
/* The same as a JSON object for `status` ("emi_test"):
   {"running":true,"dac":"sine","uart_la":0,"uptime_s":12,"captures":40,"errors":0} */
size_t emi_test_json(char *buf, size_t cap);

#endif
