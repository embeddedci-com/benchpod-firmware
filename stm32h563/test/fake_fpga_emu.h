/*
 * fake_fpga_emu.h — a behavioural model of the emulation gateware (fpga_emu.h, ice40/PROTOCOL.md)
 * plus the pod hooks (sensor_port.h), for test_sensor_mgr.c. The DUT side is driven by the test:
 * fake_i2c_write / fake_i2c_read act like a bus master, fake_uart_dut_send feeds UART2 RX.
 */
#ifndef FAKE_FPGA_EMU_H
#define FAKE_FPGA_EMU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fpga_emu.h"
#include "sensor_mgr.h"

typedef struct {
    uint8_t  regs[256], attrs[256];
    uint8_t  addr7, sda, scl, flags, page, mask, drdy;
    uint16_t times[4];
    uint8_t  wr_count;
    uint8_t  log[32][2];
    uint64_t busy_until_us;
    unsigned drdy_asserts, drdy_deasserts, configs, loads, load_bytes;
    uint8_t  pointer;
    bool     off_window;      /* 16-bit window: the pointer's high byte is not the page */
} fake_target_t;

typedef struct {
    fake_target_t t[5];
    uint8_t  spi_cfg[7];
    uint8_t  pinmap[14];
    uint8_t  dr_width;
    unsigned fifo_flushes;
    uint8_t  fifo[2048];
    unsigned fifo_len;
    bool     fifo_ovf, fifo_udf;
    /* uart2 */
    bool     u2_armed;
    uint8_t  u2_rx_w, u2_tx_w;
    uint32_t u2_baud;
    unsigned u2_configs;
    uint8_t  u2_wire[16384];
    size_t   u2_nwire;
    uint8_t  u2_rx[256];
    unsigned u2_nrx;
    /* port */
    uint64_t now_us;
    int      image;
    unsigned image_loads;
    int      load_rc;
    bool     dac_running;
    bool     current_shares_dac;
    sm_out_t analog_out;
    double   analog_value;
    unsigned analog_sets, analog_releases;
    long     allocs;          /* live heap blocks */
    size_t   heap_limit;      /* 0 = no limit */
} fake_fpga_t;

extern fake_fpga_t g_fpga;

void fake_reset(void);
void fake_advance_us(uint64_t us);
/* A DUT write transaction on an I2C address: the register pointer then `n` data bytes.
   Returns false when no enabled target acknowledges (mute, not configured). */
bool fake_i2c_write(uint8_t addr7, uint8_t reg, const uint8_t *data, size_t n);
/* The same with a 16-bit pointer (16-bit window or command map: high byte first on the bus). */
bool fake_i2c_write16(uint8_t addr7, uint16_t reg, const uint8_t *data, size_t n);
/* One DUT register write done right before the pod's next register load (a race). */
void fake_race_write_before_load(uint8_t addr7, uint8_t reg, uint8_t val);
/* A DUT read: set the pointer to reg (unless reg < 0: continue) and read n bytes. */
bool fake_i2c_read(uint8_t addr7, int reg, uint8_t *out, size_t n);
/* The SPI target: a write of n bytes at reg (the address byte as the DUT sends it). */
void fake_spi_write(uint8_t reg, const uint8_t *data, size_t n);
void fake_spi_read(uint8_t reg, uint8_t *out, size_t n);
void fake_uart_dut_send(const char *s, size_t n);
int  fake_target_of(uint8_t addr7);

#endif /* FAKE_FPGA_EMU_H */
