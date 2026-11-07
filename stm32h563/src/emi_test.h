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
 *   - the DAC: on a quad-DAC board (DAC8554, analog v3r2 and the v4 analog module) all at once by
 *     default (`all`): the 1 kHz sine on 0-3.3 V, 3.0 V DC on 0-5 V and 12 mA on the 4-20 mA
 *     output; the +-12 V output stays at 0 V. The single-DAC boards (analog v3r1) have one DAC
 *     behind muxes, so one output at a time there, the sine by default. Either can pick one
 *     output (`emi dac sine|dc|current`) or none
 *   - back-to-back deep captures, ADC + LA and LA only in turn, each read back over the PSRAM bus
 *     and formatted like a client's, then thrown away
 *   - optionally a 115200 baud NMEA stream out of one LA pin (`emi uart <la>`), so the pod's own
 *     LA drivers toggle; off by default, since it drives a pin of the DUT
 *
 *   - a stream of raw Ethernet frames out of the Ethernet port (default 20 Mbit/s, full
 *     1514-byte frames, EtherType 0x88B6, to a locally administered MAC nobody owns), so the
 *     wired link carries traffic with no laptop script. Layer 2 only: it needs a link, not an IP
 *     address, DHCP or a peer. Through a Wi-Fi bridge it keeps the bridge's radio transmitting
 *     too. Each frame starts with "BPEMI", a sequence number and the pod's uptime, so a capture
 *     (`tcpdump ether proto 0x88b6`) can count losses. `emi net <mbit>|off` sets the rate
 *
 * The rest of the network runs as usual (cloud, LAN, USB network).
 *
 * The loop drives the pod with its own JSON commands on the pseudo-connection CH_EMI_CONN,
 * whose replies go to emi_test_reply() instead of a socket: every check and gate a client meets
 * applies here too. A heavy command from a client meanwhile gets "busy"; `emi stop` hands the
 * pod back. All of it runs on the hw worker (console commands do too).
 *
 * The USB console prints a monitor line every EMI_TEST_REPORT_MS and `emi` shows the state. The
 * same line goes out of the Ethernet port as a broadcast frame on the stream's EtherType, starting
 * with "BPMON" (plus the run number, reset cause, uptime and the stream's sequence number), from
 * boot on and whether the loop runs or not: a laptop on the cable reads it in Wireshark or with
 * tools/emi/emi_watch.py, with no IP address.
 *
 * Every run is also kept in the run log in the W25Q (emi_log.h): a RUN_START at boot, an EVENT on
 * every LED or link change, a CHECKPOINT every 10 s. `emi log` prints it, the JSON command
 * emi_log reads it out (tools/emi/emi_log_dump.py). Log writes go between two captures (the W25Q
 * shares the PSRAM bus); the DAC outputs keep running through them.
 */

/* Wait this long after boot before starting, so the network and cloud link come up first. */
#define EMI_TEST_START_DELAY_MS  10000u
/* A monitor line on the USB console this often. */
#define EMI_TEST_REPORT_MS       5000u
/* The Ethernet stream: default and top rate, frame size (a full 1500-byte MTU plus the 14-byte
   header), EtherType (IEEE 802 local experimental 2; 0x88B5 is the Wi-Fi bridge's control
   channel) and destination (locally administered unicast, 02 'B' 'P' 'E' 'M' 'I': switches flood
   it like any unknown address, an access point keeps it off the air). The default is 20 Mbit/s,
   twice the "at least 10 % of line rate" CISPR 32 asks of a LAN port on a 100 Mbit/s link, since
   the lab runs the build unattended. */
#define EMI_NET_DEFAULT_MBIT     20u
#define EMI_NET_MAX_MBIT         50u
#define EMI_NET_FRAME            1514u
#define EMI_NET_ETHERTYPE        0x88B6u
#define EMI_NET_DST_MAC          { 0x02, 0x42, 0x50, 0x45, 0x4D, 0x49 }
/* An Ethernet problem (no link, or a frame the pod could not send) counts as a failure like a
   failed capture (red now, blue afterwards, status_led.h), so a lab watching only the LED sees the
   LAN port fail during an immunity test. A link that has not come up yet after a start is given this long
   (autonegotiation) before it counts. */
#define EMI_NET_LINK_GRACE_MS    10000u

typedef enum {
    EMI_DAC_SINE = 0,   /* 1 kHz sine, 0.15-3.15 V on the 0-3.3 V output (to the BNC) */
    EMI_DAC_DC,         /* 3.0 V DC on the 0-5 V output (J11 to J8: 12 mA in the loop input) */
    EMI_DAC_CURRENT,    /* 12 mA held on the 4-20 mA output (J9) */
    EMI_DAC_OFF,
    EMI_DAC_ALL,        /* quad DAC only: the sine, the DC and the 12 mA together */
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
/* Console `emi dac <mode>`: change the DAC activity (applied at once when running). Returns
   false for `all` on a board without the quad DAC, and for anything but `off` on a digital-only
   board (no DAC or ADC: the loop there runs the LA captures only). */
bool emi_test_set_dac(emi_dac_mode_t mode);
/* Console `emi uart <1-14>|off`: NMEA out of an LA pin while running; 0 = off. */
void emi_test_set_uart(int la);
/* Console `emi net <mbit>|off`: the Ethernet stream's rate, 0 = off; false above
   EMI_NET_MAX_MBIT. */
bool emi_test_set_net_rate(unsigned mbit);

struct netif;
/* Net task, every net_poll pass: send the stream's frames that are due, on `eth` only.
   lwIP is NO_SYS and owned by the net task, so the stream lives there, not on the hw worker. */
void emi_test_net_poll(struct netif *eth);

/* "sine", "dc", "current", "off", "all"; -1 for anything else. */
int  emi_test_dac_from_name(const char *name);

/* What the status LED shows for the loop (a status_led_emi_t, status_led.h). */
uint8_t emi_test_led_state(void);

/* The run log (emi_log.h). This boot's run number, 0 when the log is not available. */
uint32_t emi_test_run(void);
/* Console `emi log [n|all]`: print the last n records (0 = all) on the device log, a few lines
   per worker pass; false when there is no log. */
bool   emi_test_log_print(uint32_t n);
#if defined(EMI_LOG_TEST_SECTORS)
void   emi_test_log_fill(uint32_t n);   /* test builds only: `emi log fill <n>` */
#endif
/* "run log: run 12, 345 records in 7040 slots (...), ..." for `emi log`. */
size_t emi_test_log_describe(char *buf, size_t cap);

/* One-line state for the console (`emi`) and the monitor line. */
size_t emi_test_describe(char *buf, size_t cap);
/* The same as a JSON object, in the emi_log reply ("emi_test"):
   {"running":true,"dac":"sine","uart_la":0,"core_mhz":250,"uptime_s":12,"captures":40,"errors":0,
    "net_mbit":10,"net_sent":12000,"net_drops":0,"net_link":true,"net_link_losses":0,
    "led":"green","failed_since_power_on":false,"reset_cause":"power-on"} */
size_t emi_test_json(char *buf, size_t cap);
/* The short form for `status` ("emi_test"), whose LAN reply has to fit one connection's 2 KB TX
   ring whole (conn_tx): {"running":true,"led":"green","failed_since_power_on":false,
   "captures":40,"errors":0,"net_link":true,"run":3} */
size_t emi_test_json_short(char *buf, size_t cap);

#endif
