// ============================================================================
// image_config.vh: which runtime image this build is (v54).
//
// One top (top_v2.v) builds three images, picked by a -D flag:
//
//   (none)            loop image:      the in-fabric DAC control loop (dac_loop)
//   -DUSE_DEEP_REPLAY deep image:      deep DAC replay from PSRAM (dac_psram_reader + arbiter)
//   -DUSE_EMULATION   emulation image: the emulated sensors (I2C targets, SPI target, UART2)
//
// Everything else (ADC capture, LA capture incl. change capture, the UART proxy, SWD, GPIO,
// the SPI master, the shallow DAC generate) is in all three.  Since v54 the sensor blocks are
// only in the emulation image; the firmware swaps to it when a sensor starts.  Two derived
// macros keep the ifdefs short:
//
//   HAS_DAC_LOOP   the loop image (neither flag)
//   HAS_EMULATION  the emulation image
// ============================================================================
`ifndef IMAGE_CONFIG_VH
`define IMAGE_CONFIG_VH
// Define at most one of the two flags (the deep one wins if both are set).
`ifdef USE_DEEP_REPLAY
`elsif USE_EMULATION
  `define HAS_EMULATION
`else
  `define HAS_DAC_LOOP
`endif
`endif
