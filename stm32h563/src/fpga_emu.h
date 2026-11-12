/*
 * fpga_emu.h — the sensor blocks of the emulation gateware image (v54+, FPGA_FEATURES bit2):
 * four I2C targets, one SPI target, the shared sensor FIFO and UART2 (TX + RX).
 *
 * Thin SPI wrappers around the opcodes in ice40/PROTOCOL.md (0x60-0x64, 0x6A-0x6F, 0x75,
 * 0x77-0x7C), defined in signal_engine.c. The sensor manager (sensor_mgr.c) is the only caller;
 * the host tests replace this whole interface with a fake FPGA (test/fake_fpga_emu.c).
 *
 * Targets: 0..3 are the I2C targets, FPGA_EMU_SPI_TARGET (4) is the SPI target (register image,
 * attributes, status and write log). LA channels here are WIRE indices 0..13 (LA1 = 0), as the
 * gateware takes them. Every call returns 0, or <0 on an SPI failure / bad argument.
 */
#ifndef FPGA_EMU_H
#define FPGA_EMU_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FPGA_EMU_I2C_TARGETS   4u
#define FPGA_EMU_SPI_TARGET    4u
#define FPGA_EMU_TARGETS       5u
#define FPGA_EMU_LOG_ENTRIES   32u     /* write-log ring per target */
#define FPGA_EMU_FIFO_BYTES    2047u   /* the shared sensor FIFO: 2047 bytes usable */
#define FPGA_EMU_UART2_FIFO    256u    /* UART2 TX and RX FIFO depth */
#define FPGA_EMU_TIMES         4u      /* conversion times per I2C target */

/* I2C_CONFIG flags (byte 4). */
#define EMU_I2C_ENABLE      0x01u
#define EMU_I2C_ADDR16      0x02u
#define EMU_I2C_BUSY_FLAG   (0u << 2)
#define EMU_I2C_BUSY_NACK   (1u << 2)
#define EMU_I2C_BUSY_STRETCH (2u << 2)
#define EMU_I2C_MUTE        0x10u      /* fault: the address is not acknowledged */
#define EMU_I2C_BUS_STUCK   0x20u      /* fault: SDA held low until a config clears it */
/* I2C_CONFIG drdy byte. */
#define EMU_DRDY_OFF        0u
#define EMU_DRDY_PULSE      1u
#define EMU_DRDY_LEVEL      2u
#define EMU_DRDY_ACTIVE_HIGH 0x04u
#define EMU_DRDY_ON_CONV    0x08u      /* assert at the end of each conversion */

/* I2C_LOAD_ATTRS attribute byte. */
#define EMU_ATTR_TRIG   0x01u
#define EMU_ATTR_CMD    0x02u
#define EMU_ATTR_BUSY   0x04u
#define EMU_ATTR_FIFO   0x08u
#define EMU_ATTR_CLR    0x10u
#define EMU_ATTR_RO     0x20u
#define EMU_ATTR_TIME(i) ((uint8_t)(((i) & 3u) << 6))

/* SPIT_CONFIG flags. */
#define EMU_SPI_ENABLE  0x01u
#define EMU_SPI_CPHA    0x02u
#define EMU_SPI_CPOL    0x04u
#define EMU_SPI_MUTE    0x08u
#define EMU_SPI_DUMMY(n) ((uint8_t)(((n) & 3u) << 4))

/* EMU_PINMAP driver codes. */
#define EMU_PIN_NONE      0u
#define EMU_PIN_I2C       1u
#define EMU_PIN_DRDY(t)   ((uint8_t)(2u + (t)))   /* data ready of I2C target 0..3 */
#define EMU_PIN_SPI_MISO  6u
#define EMU_PIN_UART2_TX  7u

/* I2C_STATUS flags (I2C targets). */
#define EMU_ST_ENABLED  0x80u
#define EMU_ST_BUSY     0x40u
#define EMU_ST_DRDY     0x20u

/* UART2_STATUS flags. */
#define EMU_U2_RX_AVAIL_HI 0x01u
#define EMU_U2_TX_FULL     0x08u
#define EMU_U2_TX_EMPTY    0x10u
#define EMU_U2_ARMED       0x20u
#define EMU_U2_RX_OVF      0x40u
#define EMU_U2_RX_FRAME    0x80u

int fpga_emu_i2c_config(uint8_t target, uint8_t addr7, uint8_t sda_w, uint8_t scl_w, uint8_t flags,
                        uint8_t page, uint8_t busy_mask, uint8_t drdy);
int fpga_emu_i2c_disable(uint8_t target);
int fpga_emu_load_regs(uint8_t target, uint8_t start, const uint8_t *data, size_t len);
/* len bytes of the image (the gateware's repeated first byte is dropped here). */
int fpga_emu_read_regs(uint8_t target, uint8_t start, uint8_t *data, size_t len);
int fpga_emu_load_attrs(uint8_t target, uint8_t start, const uint8_t *attrs, size_t len);
int fpga_emu_read_attrs(uint8_t target, uint8_t start, uint8_t *attrs, size_t len);
int fpga_emu_status(uint8_t target, uint8_t *flags, uint8_t *wr_count);
/* n (<= 32) write-log entries from entry index `first` (mod 32): pairs[2k] = address inside the
   window, pairs[2k+1] = the value. */
int fpga_emu_read_log(uint8_t target, uint8_t first, uint8_t n, uint8_t *pairs);
int fpga_emu_times(uint8_t target, uint8_t index, uint16_t time_us);
int fpga_emu_drdy(uint8_t target, bool assert_it);
/* codes[w] = the driver of LA wire w (EMU_PIN_*). */
int fpga_emu_pinmap(const uint8_t codes[14]);
int fpga_emu_config(uint8_t dr_width_us, bool flush_fifo);
int fpga_emu_fifo_write(const uint8_t *data, size_t len);
/* level = bytes waiting (0..2048); the sticky overflow/underflow flags are cleared by the read. */
int fpga_emu_fifo_status(uint16_t *level, bool *overflow, bool *underflow);
int fpga_emu_spi_config(uint8_t cs_w, uint8_t sck_w, uint8_t mosi_w, uint8_t flags, uint8_t read_bit,
                        uint8_t inc_bit, uint8_t addr_mask);
/* rx_w / tx_w: wire indices (0xFF = none). baud: the bit rate, the gateware divider is computed
   here. enable false disarms it. */
int fpga_emu_uart2_config(uint8_t rx_w, uint8_t tx_w, uint32_t baud, bool enable);
int fpga_emu_uart2_write(const uint8_t *data, size_t len);
int fpga_emu_uart2_read(uint8_t *data, size_t len);
int fpga_emu_uart2_status(uint16_t *rx_avail, uint8_t *flags);

#endif /* FPGA_EMU_H */
