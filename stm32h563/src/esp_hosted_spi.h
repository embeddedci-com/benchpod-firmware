#ifndef ESP_HOSTED_SPI_H
#define ESP_HOSTED_SPI_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#include "esp_hosted_frame.h"   /* esp_hosted_rx_t, ESP_*_IF */
#include "esp_hosted_pump.h"    /* esp_hosted_pump_stats_t */

/* ---- ESP-Hosted SPI transport (STM32 master ↔ ESP32-C3 slave) --------------
 *
 * Drives the on-board ESP32-C3 (U4) over SPI4 using the esp-hosted framing
 * (esp_hosted_frame.*). The STM32 is the SPI master; the ESP32 raises HANDSHAKE
 * when it is ready for a transaction and DATA_READY when it has a frame to send.
 * Polled cooperatively from the net task (NO_SYS=1) via esp_hosted_spi_poll().
 *
 * Received frames are demuxed by interface type: ESP_STA_IF (802.3 data) → the
 * data callback (esp_netif), ESP_SERIAL_IF (RPC) → the serial callback
 * (esp_wifi_ctrl); ESP_PRIV_IF boot/events are handled internally.
 *
 * ⚠ BRING-UP: the constants below MUST match the slave's menuconfig, and the
 *   signal polarities/clock are bench-verify items (no hardware tested yet).
 * ---------------------------------------------------------------------------*/

/* Fixed full-duplex SPI transfer size — both ends agree on this (esp-hosted
   ESP_TRANSPORT_SPI_MAX_BUF_SIZE). */
#define ESP_HOSTED_SPI_BUF_SIZE   1600

/* SPI mode (CPOL/CPHA). Upstream slave default is mode 3. MUST match the slave's
   CONFIG_ESP_SPI_MODE. */
#define ESP_HOSTED_SPI_MODE       3

/* Slave-driven status line active levels (esp-hosted ESP32-C3 default = high). */
#define ESP_HOSTED_HS_ACTIVE_HIGH 1   /* HANDSHAKE: slave ready for a transfer  */
#define ESP_HOSTED_DR_ACTIVE_HIGH 1   /* DATA_READY: slave has a frame to send  */

/* Callback for a received frame of a given interface (payload points into an
   internal buffer valid only for the duration of the call — copy if retained). */
typedef void (*esp_hosted_frame_cb_t)(const esp_hosted_rx_t *rx);

/* Configure SPI4 + the control/status GPIOs and hold the ESP32 in reset.
   Does NOT release reset — call esp_hosted_spi_start() once Wi-Fi is configured
   so an unprovisioned unit leaves the co-processor powered down. Returns 0/-1. */
int  esp_hosted_spi_init(void);

/* Release the ESP32 from reset (normal boot) and begin the link bring-up; the
   slave's boot ESP_PRIV_EVENT_INIT is consumed in poll(), after which
   esp_hosted_spi_ready() returns true. Idempotent. */
void esp_hosted_spi_start(void);

/* Hold the ESP32 in reset (EN low) and mark the link down. */
void esp_hosted_spi_stop(void);

/* True once the slave has announced itself (boot event seen). */
bool esp_hosted_spi_ready(void);

/* True once the SPI4 GPDMA channels inited OK (Wi-Fi transactions use DMA). */
bool esp_hosted_spi_dma_active(void);

/* Advance the transport: while the slave is ready for a transaction (HANDSHAKE)
   and there is something to move (queued TX, or DATA_READY), run a full-duplex
   transfer and demux the received frame — up to ESP_HOSTED_PUMP_MAX_BATCH times
   per call, so a backlog drains faster than one frame per net-loop tick without
   starving lwIP/Ethernet. Call every net-loop pass. */
void esp_hosted_spi_poll(void);

/* Transport counters since the last esp_hosted_spi_start() — batch-length
   histogram, settle hit/miss, transfer errors. Reported by `wifi_status`. */
const esp_hosted_pump_stats_t *esp_hosted_spi_stats(void);

/* Enqueue a frame for transmission on the given interface. Copies the payload.
   Returns 0 on success, -1 if the TX queue is full or the frame is too large. */
int  esp_hosted_spi_send(uint8_t if_type, uint8_t if_num,
                         const uint8_t *payload, uint16_t len);

/* Register the demux callbacks (data = ESP_STA_IF, serial = ESP_SERIAL_IF). */
void esp_hosted_spi_set_data_cb(esp_hosted_frame_cb_t cb);
void esp_hosted_spi_set_serial_cb(esp_hosted_frame_cb_t cb);

#endif /* ESP_HOSTED_SPI_H */
