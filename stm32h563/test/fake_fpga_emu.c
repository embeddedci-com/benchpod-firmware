/*
 * fake_fpga_emu.c — see fake_fpga_emu.h.
 */
#include "fake_fpga_emu.h"
#include "sensor_port.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

fake_fpga_t g_fpga;

void fake_reset(void) {
    memset(&g_fpga, 0, sizeof(g_fpga));
    g_fpga.now_us = 1000000;
    g_fpga.image = 0;
    g_fpga.dr_width = 50;
    g_fpga.current_shares_dac = true;
}

void fake_advance_us(uint64_t us) { g_fpga.now_us += us; }

/* ---- gateware ---- */

int fpga_emu_i2c_config(uint8_t t, uint8_t addr7, uint8_t sda_w, uint8_t scl_w, uint8_t flags, uint8_t page,
                        uint8_t busy_mask, uint8_t drdy) {
    if (g_fpga.image != 2 || t > 3) return -1;
    fake_target_t *x = &g_fpga.t[t];
    x->addr7 = addr7; x->sda = sda_w; x->scl = scl_w; x->flags = flags; x->page = page; x->mask = busy_mask; x->drdy = drdy;
    x->configs++;
    return 0;
}
int fpga_emu_i2c_disable(uint8_t t) { if (t > 3) return -1; g_fpga.t[t].flags &= (uint8_t)~EMU_I2C_ENABLE; return 0; }
/* A DUT write that lands while the pod reads the log and loads the image (the race a driver
   hits when it writes its setup right after a soft reset): done just before the next load. */
static struct { bool armed; uint8_t addr7, reg, val; } s_race;
void fake_race_write_before_load(uint8_t addr7, uint8_t reg, uint8_t val) {
    s_race.armed = true; s_race.addr7 = addr7; s_race.reg = reg; s_race.val = val;
}
int fpga_emu_load_regs(uint8_t t, uint8_t start, const uint8_t *d, size_t n) {
    if (t > 4 || start + n > 256 || !n) return -1;
    if (s_race.armed) { s_race.armed = false; (void)fake_i2c_write(s_race.addr7, s_race.reg, &s_race.val, 1); }
    memcpy(&g_fpga.t[t].regs[start], d, n);
    g_fpga.t[t].loads++;
    g_fpga.t[t].load_bytes += (unsigned)n;
    return 0;
}
int fpga_emu_read_regs(uint8_t t, uint8_t start, uint8_t *d, size_t n) {
    if (t > 4 || start + n > 256 || !n) return -1;
    memcpy(d, &g_fpga.t[t].regs[start], n);
    return 0;
}
int fpga_emu_load_attrs(uint8_t t, uint8_t start, const uint8_t *a, size_t n) {
    if (t > 4 || start + n > 256) return -1;
    memcpy(&g_fpga.t[t].attrs[start], a, n);
    return 0;
}
int fpga_emu_read_attrs(uint8_t t, uint8_t start, uint8_t *a, size_t n) {
    if (t > 4 || start + n > 256) return -1;
    memcpy(a, &g_fpga.t[t].attrs[start], n);
    return 0;
}
int fpga_emu_status(uint8_t t, uint8_t *flags, uint8_t *wr) {
    if (t > 4) return -1;
    fake_target_t *x = &g_fpga.t[t];
    if (flags) *flags = (uint8_t)(((x->flags & 1) ? EMU_ST_ENABLED : 0) | (g_fpga.now_us < x->busy_until_us ? EMU_ST_BUSY : 0));
    if (wr) *wr = x->wr_count;
    return 0;
}
int fpga_emu_read_log(uint8_t t, uint8_t first, uint8_t n, uint8_t *pairs) {
    if (t > 4 || n == 0 || n > 32) return -1;
    for (unsigned k = 0; k < n; k++) {
        unsigned e = (first + k) % 32u;
        pairs[2 * k] = g_fpga.t[t].log[e][0];
        pairs[2 * k + 1] = g_fpga.t[t].log[e][1];
    }
    return 0;
}
int fpga_emu_times(uint8_t t, uint8_t i, uint16_t us) { if (t > 3 || i > 3) return -1; g_fpga.t[t].times[i] = us; return 0; }
int fpga_emu_drdy(uint8_t t, bool a) { if (t > 3) return -1; if (a) g_fpga.t[t].drdy_asserts++; else g_fpga.t[t].drdy_deasserts++; return 0; }
int fpga_emu_pinmap(const uint8_t codes[14]) { memcpy(g_fpga.pinmap, codes, 14); return 0; }
int fpga_emu_config(uint8_t w, bool flush) { g_fpga.dr_width = w; if (flush) { g_fpga.fifo_len = 0; g_fpga.fifo_flushes++; } return 0; }
int fpga_emu_fifo_write(const uint8_t *d, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (g_fpga.fifo_len >= 2048) { g_fpga.fifo_ovf = true; continue; }
        g_fpga.fifo[g_fpga.fifo_len++] = d[i];
    }
    return 0;
}
int fpga_emu_fifo_status(uint16_t *level, bool *ovf, bool *udf) {
    if (level) *level = (uint16_t)g_fpga.fifo_len;
    if (ovf) *ovf = g_fpga.fifo_ovf;
    if (udf) *udf = g_fpga.fifo_udf;
    g_fpga.fifo_ovf = g_fpga.fifo_udf = false;
    return 0;
}
int fpga_emu_spi_config(uint8_t cs, uint8_t sck, uint8_t mosi, uint8_t flags, uint8_t rb, uint8_t ib, uint8_t mask) {
    if (g_fpga.image != 2) return -1;
    uint8_t c[7] = { cs, sck, mosi, flags, rb, ib, mask };
    memcpy(g_fpga.spi_cfg, c, 7);
    return 0;
}
int fpga_emu_uart2_config(uint8_t rx, uint8_t tx, uint32_t baud, bool en) {
    if (g_fpga.image != 2) return -1;
    g_fpga.u2_rx_w = rx; g_fpga.u2_tx_w = tx; g_fpga.u2_baud = baud; g_fpga.u2_armed = en; g_fpga.u2_configs++;
    return 0;
}
int fpga_emu_uart2_write(const uint8_t *d, size_t n) {
    if (n > 256 || g_fpga.u2_nwire + n > sizeof(g_fpga.u2_wire)) return -1;
    memcpy(g_fpga.u2_wire + g_fpga.u2_nwire, d, n);   /* sent at once: the FIFO is empty again */
    g_fpga.u2_nwire += n;
    return 0;
}
int fpga_emu_uart2_read(uint8_t *d, size_t n) {
    if (n > g_fpga.u2_nrx) return -1;
    memcpy(d, g_fpga.u2_rx, n);
    memmove(g_fpga.u2_rx, g_fpga.u2_rx + n, g_fpga.u2_nrx - n);
    g_fpga.u2_nrx -= (unsigned)n;
    return 0;
}
int fpga_emu_uart2_status(uint16_t *avail, uint8_t *flags) {
    if (avail) *avail = (uint16_t)g_fpga.u2_nrx;
    if (flags) *flags = (uint8_t)(EMU_U2_TX_EMPTY | (g_fpga.u2_armed ? EMU_U2_ARMED : 0));
    return 0;
}

/* ---- the DUT side ---- */

int fake_target_of(uint8_t addr7) {
    for (unsigned t = 0; t < 4; t++)
        if ((g_fpga.t[t].flags & EMU_I2C_ENABLE) && g_fpga.t[t].addr7 == addr7) return (int)t;
    return -1;
}

static void start_conv(fake_target_t *x, uint8_t attr) {
    x->busy_until_us = g_fpga.now_us + x->times[(attr >> 6) & 3u];
}

static void data_write(fake_target_t *x, uint8_t a, uint8_t v) {
    if (!(x->attrs[a] & EMU_ATTR_RO)) x->regs[a] = v;
    x->log[x->wr_count % 32u][0] = a;
    x->log[x->wr_count % 32u][1] = v;
    x->wr_count++;
    if (x->attrs[a] & EMU_ATTR_TRIG) start_conv(x, x->attrs[a]);
}

/* The pointer byte(s) as the gateware maps them: word registers (2p), the command map
   (image[h] ^ l) or the 16-bit window (the low byte; outside the window: not served). */
static bool set_pointer(fake_target_t *x, int reg) {
    unsigned p;
    x->off_window = false;
    if (x->flags & EMU_I2C_CMAP) p = (unsigned)x->regs[(reg >> 8) & 0xFF] ^ ((unsigned)reg & 0xFFu);
    else if (x->flags & EMU_I2C_ADDR16) { p = (unsigned)reg & 0xFFu; x->off_window = ((reg >> 8) & 0xFF) != x->page; }
    else if (x->flags & EMU_I2C_WORD) p = 2u * ((unsigned)reg & 0x7Fu);
    else p = (unsigned)reg & 0xFFu;
    x->pointer = (uint8_t)p;
    if (!x->off_window && (x->attrs[x->pointer] & EMU_ATTR_CMD)) start_conv(x, x->attrs[x->pointer]);
    return !x->off_window;
}

bool fake_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, size_t n) {
    return fake_i2c_write16(addr7, reg, data, n);
}

bool fake_i2c_write16(uint8_t addr7, uint16_t reg, const uint8_t *data, size_t n) {
    int t = fake_target_of(addr7);
    if (t < 0) return false;
    fake_target_t *x = &g_fpga.t[t];
    if (x->flags & (EMU_I2C_MUTE | EMU_I2C_BUS_STUCK)) return false;
    if (!set_pointer(x, reg)) return true;   /* outside the window: acknowledged, ignored */
    for (size_t i = 0; i < n; i++) data_write(x, x->pointer++, data[i]);
    return true;
}

static uint8_t read_byte(fake_target_t *x, bool *advance) {
    uint8_t a = x->pointer, at = x->attrs[a];
    *advance = true;
    if (at & EMU_ATTR_FIFO) {
        *advance = false;
        if (!g_fpga.fifo_len) { g_fpga.fifo_udf = true; return 0; }
        uint8_t b = g_fpga.fifo[0];
        memmove(g_fpga.fifo, g_fpga.fifo + 1, --g_fpga.fifo_len);
        return b;
    }
    if (x->off_window) return 0;
    uint8_t v = x->regs[a];
    if ((at & EMU_ATTR_BUSY) && g_fpga.now_us < x->busy_until_us) v |= x->mask;
    return v;
}

bool fake_i2c_read(uint8_t addr7, int reg, uint8_t *out, size_t n) {
    int t = fake_target_of(addr7);
    if (t < 0) return false;
    fake_target_t *x = &g_fpga.t[t];
    if (x->flags & (EMU_I2C_MUTE | EMU_I2C_BUS_STUCK)) return false;
    if (reg >= 0) (void)set_pointer(x, reg);
    if (((x->flags >> 2) & 3u) == 1u && g_fpga.now_us < x->busy_until_us) return false;   /* nack while busy */
    for (size_t i = 0; i < n; i++) {
        bool adv;
        out[i] = read_byte(x, &adv);
        if (adv) x->pointer++;
    }
    return true;
}

void fake_spi_write(uint8_t reg, const uint8_t *data, size_t n) {
    fake_target_t *x = &g_fpga.t[4];
    for (size_t i = 0; i < n; i++) data_write(x, (uint8_t)(reg + i), data[i]);
}

void fake_spi_read(uint8_t reg, uint8_t *out, size_t n) {
    fake_target_t *x = &g_fpga.t[4];
    x->pointer = reg;
    for (size_t i = 0; i < n; i++) {
        bool adv;
        adv = true;
        out[i] = (g_fpga.spi_cfg[3] & EMU_SPI_MUTE) ? 0xFF : read_byte(x, &adv);
        if (adv) x->pointer++;
    }
}

void fake_uart_dut_send(const char *s, size_t n) {
    for (size_t i = 0; i < n && g_fpga.u2_nrx < 256; i++) g_fpga.u2_rx[g_fpga.u2_nrx++] = (uint8_t)s[i];
}

/* ---- the pod hooks ---- */

uint64_t sensor_port_now_us(void) { return g_fpga.now_us; }

typedef struct { size_t n; long long pad; } hdr_t;
static size_t s_heap_used;
void *sensor_port_alloc(size_t n) {
    if (g_fpga.heap_limit && s_heap_used + n > g_fpga.heap_limit) return NULL;
    hdr_t *h = malloc(sizeof(hdr_t) + n);
    if (!h) return NULL;
    h->n = n;
    s_heap_used += n;
    g_fpga.allocs++;
    return h + 1;
}
void sensor_port_free(void *p) {
    if (!p) return;
    hdr_t *h = (hdr_t *)p - 1;
    s_heap_used -= h->n;
    g_fpga.allocs--;
    free(h);
}
size_t sensor_port_heap_free(void) { return g_fpga.heap_limit ? g_fpga.heap_limit - s_heap_used : 100000; }
int  sensor_port_image(void) { return g_fpga.image; }
int  sensor_port_load_emulation(char *err, size_t cap) {
    if (g_fpga.load_rc) { snprintf(err, cap, "the emulation image is not installed"); return -1; }
    g_fpga.image = 2;
    g_fpga.image_loads++;
    for (unsigned t = 0; t < 5; t++) { memset(g_fpga.t[t].regs, 0, 256); g_fpga.t[t].flags = 0; }
    g_fpga.dac_running = false;
    sm_on_gateware_reconfigured();
    return 0;
}
bool sensor_port_dac_running(void) { return g_fpga.dac_running; }
int  sensor_port_analog_set(sm_out_t out, double v, double max_v, char *err, size_t cap) {
    (void)max_v; (void)err; (void)cap;
    g_fpga.analog_out = out; g_fpga.analog_value = v; g_fpga.analog_sets++;
    return 0;
}
void sensor_port_analog_release(sm_out_t out) { (void)out; g_fpga.analog_releases++; }
bool sensor_port_current_shares_dac(void) { return g_fpga.current_shares_dac; }
bool sensor_port_la_voltage_set(void) { return true; }
uint16_t sensor_port_pull_mask(void) { return 0; }
void sensor_port_caps_changed(void) {}
