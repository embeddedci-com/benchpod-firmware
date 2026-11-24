/* test_gw_caps.c: gw_caps_for() (the gateware gates) and gw_caps() (published from a second slot when the running
   version or feature byte changes). */
#include "gw_caps.h"
#include "signal_engine.h"
#include "la_rate.h"

#include <stdio.h>
#include <string.h>

static int checks, fails;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("FAIL %s:%d: ", __FILE__, __LINE__); \
                                                   printf(__VA_ARGS__); printf("\n"); } } while (0)

/* fakes: the running gateware */
static uint8_t g_ver, g_feat;
uint8_t signal_engine_fpga_version(void) { return g_ver; }
uint8_t signal_engine_fpga_features_cached(void) { return g_feat; }

/* Every gate is exactly `version >= its *_MIN_GW`, for every version. */
static void test_versions(void)
{
    for (unsigned v = 0; v <= 255; v++) {
        const gw_caps_t c = gw_caps_for((uint8_t)v, 0);
#define GATE(field, min) CHECK(c.field == (v >= (min)), "v%u " #field, v)
        GATE(dac_deep_replay, DAC_DEEP_REPLAY_MIN_GW);
        GATE(concurrent_psram, CONCURRENT_PSRAM_MIN_GW);
        GATE(dac_capture_stop, DAC_CAPTURE_STOP_MIN_GW);
        GATE(capture_bases, CAPTURE_BASES_MIN_GW);
        GATE(dac_control_loop, DAC_CONTROL_LOOP_MIN_GW);
        GATE(dac_cotrig, DAC_COTRIG_MIN_GW);
        GATE(dac_loop_source, DAC_LOOP_SOURCE_MIN_GW);
        GATE(dac_loop_inmap, DAC_LOOP_INMAP_MIN_GW);
        GATE(exact_cap_divider, EXACT_CAP_DIVIDER_MIN_GW);
        GATE(gpio_get, GPIO_GET_MIN_GW);
        GATE(capture_trigger, CAPTURE_TRIGGER_MIN_GW);
        GATE(capture_opcode_only, CAPTURE_OPCODE_ONLY_MIN_GW);
        GATE(reload_wire, RELOAD_WIRE_MIN_GW);
        GATE(spi_master, SPI_MASTER_MIN_GW);
        GATE(swd_queue, SWD_QUEUE_MIN_GW);
        GATE(dac_ctrl, DAC_CTRL_MIN_GW);
        GATE(la_change, LA_CHANGE_MIN_GW);
        GATE(spi_fast_link, SPI_FAST_LINK_MIN_GW);
        GATE(dual_la_full_rate, DUAL_LA_FULL_RATE_MIN_GW);
        GATE(emulation, EMULATION_MIN_GW);
        GATE(spi_modes12, SPI_MODES12_MIN_GW);
        GATE(i2c_master, I2C_MASTER_MIN_GW);
        GATE(uart_frame, UART_FRAME_MIN_GW);
        GATE(uart_half_duplex, UART_HALFDUPLEX_MIN_GW);
        GATE(uart_de, UART_DE_MIN_GW);
        GATE(uart_config2, UART_CONFIG2_MIN_GW);
#undef GATE
        CHECK(!c.img_control_loop && !c.img_deep_replay && !c.img_emulation, "v%u no image bits", v);
    }
    /* the released gateware numbers the docs quote */
    CHECK(gw_caps_for(53, 0).dual_la_full_rate && !gw_caps_for(52, 0).dual_la_full_rate, "v53");
    CHECK(gw_caps_for(58, 0).uart_de && !gw_caps_for(57, 0).uart_de, "v58");
    CHECK(gw_caps_for(60, 0).uart_config2 && !gw_caps_for(59, 0).uart_config2, "v60");
}

static void test_images(void)
{
    for (unsigned f = 0; f < 8; f++) {
        const gw_caps_t c = gw_caps_for(59, (uint8_t)f);
        CHECK(c.img_control_loop == ((f & FPGA_FEATURE_CONTROL_LOOP) != 0), "features %u loop", f);
        CHECK(c.img_deep_replay == ((f & FPGA_FEATURE_DEEP_REPLAY) != 0), "features %u deep", f);
        CHECK(c.img_emulation == ((f & FPGA_FEATURE_EMULATION) != 0), "features %u emulation", f);
    }
    /* bus masters: every image up to v64, then only the images that set the bit (v65 split) */
    for (unsigned v = 0; v <= 255; v++) {
        CHECK(gw_caps_for((uint8_t)v, FPGA_FEATURE_EMULATION).bus_masters == (v < BUS_MASTERS_SPLIT_MIN_GW),
              "v%u emulation image bus masters", v);
        CHECK(gw_caps_for((uint8_t)v, FPGA_FEATURE_CONTROL_LOOP | FPGA_FEATURE_BUS_MASTERS).bus_masters,
              "v%u loop image bus masters", v);
    }
    CHECK(BUS_MASTERS_SPLIT_MIN_GW == 65, "the split is v65");
    CHECK(gw_caps_for(64, FPGA_FEATURE_EMULATION).bus_masters, "v64 emulation still has them");
    CHECK(!gw_caps_for(65, FPGA_FEATURE_EMULATION).bus_masters, "v65 emulation has none");
    CHECK(gw_caps_for(65, 0x2A).bus_masters && gw_caps_for(65, 0x29).bus_masters, "v65 deep and loop");
}

/* gw_caps() follows the running gateware: a new version or feature byte refills it. */
static void test_running(void)
{
    g_ver = 0; g_feat = 0;
    CHECK(!gw_caps()->dac_deep_replay && !gw_caps()->uart_de, "no gateware");
    g_ver = 59;
    CHECK(gw_caps()->uart_de && gw_caps()->dac_ctrl, "v59 after a version change");
    g_ver = 45;
    CHECK(gw_caps()->spi_master && !gw_caps()->i2c_master, "v45 after a downgrade");
    g_feat = FPGA_FEATURE_DEEP_REPLAY;
    CHECK(gw_caps()->img_deep_replay && !gw_caps()->img_control_loop, "an image swap");
    g_ver = 255; g_feat = 255;
    CHECK(gw_caps()->uart_de && gw_caps()->img_emulation, "v255, every feature bit");
    CHECK(gw_caps() == gw_caps(), "one struct");
}

/* A refresh builds the new caps in the other slot and publishes it with one pointer store: the
   struct a reader already holds keeps every old field, and the new one is complete. */
static void test_swap(void)
{
    g_ver = 45; g_feat = 0;
    const gw_caps_t *old = gw_caps();
    const gw_caps_t want_old = gw_caps_for(45, 0);
    CHECK(memcmp(old, &want_old, sizeof want_old) == 0, "slot filled from v45");
    CHECK(gw_caps() == old, "no refresh while nothing changed");

    g_ver = 59; g_feat = FPGA_FEATURE_EMULATION;
    const gw_caps_t *cur = gw_caps();
    const gw_caps_t want_new = gw_caps_for(59, FPGA_FEATURE_EMULATION);
    CHECK(cur != old, "a refresh publishes the other slot");
    CHECK(memcmp(cur, &want_new, sizeof want_new) == 0, "new slot complete");
    CHECK(memcmp(old, &want_old, sizeof want_old) == 0, "old slot untouched by the refresh");
    CHECK(gw_caps() == cur, "the new slot stays published");

    g_feat = 0;   /* an image swap alone refreshes too, back into the first slot */
    const gw_caps_t *third = gw_caps();
    const gw_caps_t want3 = gw_caps_for(59, 0);
    CHECK(third != cur && third == old, "slots alternate");
    CHECK(memcmp(third, &want3, sizeof want3) == 0, "third fill complete");
    CHECK(memcmp(cur, &want_new, sizeof want_new) == 0, "previous slot untouched");
}

int main(void)
{
    test_versions();
    test_images();
    test_running();
    test_swap();
    printf("test_gw_caps: %d checks, %d failed\n", checks, fails);
    return fails ? 1 : 0;
}
