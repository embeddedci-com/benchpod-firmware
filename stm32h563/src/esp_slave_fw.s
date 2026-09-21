/*
 * esp_slave_fw.s — embed the ESP32-C3 esp-hosted *slave* merged firmware image
 * as a read-only blob in STM32 flash, so the pod can program the on-board C3
 * (U4) over its ROM serial bootloader (esp_rom_flash.c), the same way the iCE40
 * bitstream is embedded for flash-ice40.
 *
 * The image is a single esptool `merge_bin` output flashed at offset 0x0
 * (bootloader @0x0 + partition-table @0x8000 + ota_data @0xd000 + app @0x10000).
 * Rebuild it with esp32-hosted-slave/build.sh; the .incbin path is relative to
 * the Makefile's working directory (stm32h563/).
 *
 * Exposes:  const uint8_t  esp_slave_fw[];       // image bytes
 *           const uint32_t esp_slave_fw_len;     // image length
 */
	/* Dedicated section (NOT .rodata*) so the linker collects it into the
	 * FLASH_BLOBS region — see .blobs in config/STM32H563ZITX_FLASH.ld. */
	.section .blob_esp_slave_fw, "a", %progbits
	.balign 4
	.global esp_slave_fw
esp_slave_fw:
	.incbin "../esp32-hosted-slave/prebuilt/esp32c3-hosted-slave-merged.bin"
1:
	.balign 4
	.global esp_slave_fw_len
esp_slave_fw_len:
	.word 1b - esp_slave_fw
