// ============================================================================
// tb_adc_timing.v — timing + throughput sign-off for the PRODUCTION adc_mcp33131
// running its FSM on the 24 MHz control clock, against the analog-accurate
// mcp33131_model (real tCONV/tEN/tDO + explicit pad/cable round-trip).
//
// The stock sim/tb_adc_mcp33131.v changes SDO with ZERO delay and therefore can't
// see an SDO-settle failure; this one can.  It runs the engine with its real SDO
// SB_IO (the yosys cells model: the DDR input register samples on the falling clk
// edge) and MEASURES the setup and hold margins at every sampling instant: setup =
// sample time - the time the bit became valid at the FPGA pin, hold = the time the
// next change reaches the pin - sample time.
//
// SCLK = clk/2 = 12 MHz.  v63 samples each bit in the middle of SCLK high, 1.5 clk
// (62.5 ns) after the edge that drove the SCLK fall that shifted it out, and the MSB
// 2.5 clk (104 ns) after the edge that dropped CNVST.  Until v62 both were 1 clk
// (41.67 ns).  What has to stay positive is
//
//   data bits:  1.5 * T_clk - (SCLK_OUT + tDO + SDO_IN)
//   MSB:        2.5 * T_clk - (CNVST_OUT + tEN + SDO_IN)
//
// SCLK_OUT / CNVST_OUT / SDO_IN lump the FPGA output and input delays, the module FFC
// with its series resistors and the ESD array.  tDO max is 9.5 ns at DVIO 3.3 V (the
// MCP33131 datasheet DS20005947B, Table 1-2); 16 ns is the 1.7 V figure.  The rows:
//   * shipping pads (6 + 6 ns) at 3.3 V and at the 1.7 V tDO;
//   * "R 33+100": a round trip that left the v62 sampling point ~2 ns (the 2026-11
//     EMC pre-scan's worst case with the board's resistors), and "R 100+100", which
//     v62 could not read (negative margin there); both must read clean now;
//   * zero delay (hold corner): SDO changes as early as it can;
//   * a NEGATIVE CONTROL whose round trip exceeds 1.5 clk.  It MUST FAIL: it proves
//     the harness detects a settle violation, so the green rows mean something.
// The "v62" column is the margin the old sampling point (1 clk) would have had.
//
// Run:  make adctimingtest
// ============================================================================
`timescale 1ns/1ps

module adc_timing_harness #(
    parameter real    CLK_NS       = 20.833, // clk HALF-period (20.833 -> 24 MHz)
    parameter real    TDO_NS       = 9.5,    // SDO valid after SCLK low (3.3 V DVIO)
    parameter real    SCLK_OUT_NS  = 6.0,    // FPGA FF -> ADC SCLK pin (pad + cable)
    parameter real    SDO_IN_NS    = 6.0,    // ADC SDO pin -> FPGA input register
    parameter integer NSAMP        = 12,
    parameter [15:0]  DIVIDER      = 16'd2   // request max rate (engine floors it)
)(
    output reg done,
    output reg pass
);
    reg clk = 0;
    always #(CLK_NS) clk = ~clk;

    reg         rst = 1;
    wire        cnvst, sclk, sdi, sdo_from_adc;
    wire [15:0] sample;
    wire        sample_stb;

    // physical round-trip: SCLK / CNVST late into the ADC, SDO late back to the FPGA
    wire sclk_at_adc, cnvst_at_adc, sdo_at_fpga;
    assign #(SCLK_OUT_NS) sclk_at_adc  = sclk;
    assign #(SCLK_OUT_NS) cnvst_at_adc = cnvst;
    assign #(SDO_IN_NS)   sdo_at_fpga  = sdo_from_adc;

    reg [15:0] pat = 16'hA53C;
    wire [15:0] analog_code = pat;
    wire conv_active, sdi_ok, bit_evt, chg_evt, bit_evt_f, chg_evt_f;
    assign #(SDO_IN_NS) bit_evt_f = bit_evt;
    assign #(SDO_IN_NS) chg_evt_f = chg_evt;

    // Production engine: SCLK = clk/2 = 12 MHz, its own SDO SB_IO (cells model).
    adc_mcp33131 dut (
        .clk(clk), .rst(rst), .en(1'b1), .divider(DIVIDER), .restart(1'b0),
        .adc_cnvst(cnvst), .adc_sclk(sclk), .adc_sdi(sdi), .adc_sdo(sdo_at_fpga),
        .sample(sample), .sample_stb(sample_stb)
    );

    mcp33131_model #(.TDO_NS(TDO_NS)) adc (
        .cnvst(cnvst_at_adc), .sclk(sclk_at_adc), .sdi(sdi), .analog_code(analog_code),
        .sdo(sdo_from_adc), .bit_evt(bit_evt), .chg_evt(chg_evt),
        .conv_active(conv_active), .sdi_ok(sdi_ok)
    );

    reg [15:0] expected;
    always @(posedge cnvst_at_adc) expected <= analog_code;

    // ---- setup / hold at the sampling instants ----
    // A used sample is the falling clk edge while the engine is in S_READ with sclk_ph = 1
    // (the SB_IO's D_IN_1, shifted in on the next rising edge); cnt == 1 there is the MSB.
    real t_valid = 0.0, t_chg = 0.0, t_samp = 0.0, su;
    real su_msb = 1.0e9, su_bit = 1.0e9, hold = 1.0e9;
    reg  samp_pending = 1'b0;
    integer violations = 0;
    always @(bit_evt_f) t_valid = $realtime;
    always @(chg_evt_f) begin
        t_chg = $realtime;
        if (samp_pending && ($realtime - t_samp) < hold) hold = $realtime - t_samp;
        samp_pending = 1'b0;
    end
    always @(negedge clk) if (!rst && dut.st == 2'd2 && dut.sclk_ph) begin
        if (t_chg > t_valid || sdo_at_fpga === 1'bx) begin
            violations = violations + 1;            // sampled while SDO was changing
            su = -1.0;
        end else su = $realtime - t_valid;
        if (dut.cnt == 5'd1) begin if (su < su_msb) su_msb = su; end
        else                 begin if (su < su_bit) su_bit = su; end
        t_samp = $realtime; samp_pending = 1'b1;
    end

    // data check + steady-state sample-period measurement (ns between strobes)
    integer got = 0, bad = 0;
    real    t_prev = 0.0, meas_period_ns = 0.0;
    always @(posedge clk) begin
        if (sample_stb) begin
            if (sample === expected) ; else bad = bad + 1;
            if (got >= 3) meas_period_ns = $realtime - t_prev;  // ignore warm-up
            t_prev = $realtime;
            got = got + 1;
            pat = pat + 16'h1111;
        end
    end

    initial begin
        pass = 1'b0; done = 1'b0;
        repeat (4) @(posedge clk); rst = 0;
        wait (got >= NSAMP);
        pass = (bad == 0) && (violations == 0) && sdi_ok && (hold > 0.0);
        done = 1'b1;
    end
endmodule

module tb_adc_timing;
    wire dP,pP, dW,pW, dR,pR, dR2,pR2, dZ,pZ, dF,pF;
    // Shipping pads, 3.3 V tDO
    adc_timing_harness                                                         hP  (.done(dP),  .pass(pP));
    // 1.7 V tDO corner
    adc_timing_harness #(.TDO_NS(16.0))                                        hW  (.done(dW),  .pass(pW));
    // round trip that left the v62 point ~2 ns (33 + 100 ohm worst case, 2026-11 EMC pre-scan)
    adc_timing_harness #(.SCLK_OUT_NS(15.0), .SDO_IN_NS(15.0))                 hR  (.done(dR),  .pass(pR));
    // more series resistance (100 + 100 ohm): beyond the v62 point
    adc_timing_harness #(.SCLK_OUT_NS(18.0), .SDO_IN_NS(18.0))                 hR2 (.done(dR2), .pass(pR2));
    // hold corner: no pad delay, SDO changes the instant SCLK falls
    adc_timing_harness #(.SCLK_OUT_NS(0.0), .SDO_IN_NS(0.0), .TDO_NS(0.0))    hZ  (.done(dZ),  .pass(pZ));
    // NEGATIVE CONTROL: round trip 28 + 9.5 + 28 = 65.5 ns > 1.5 clk (62.5 ns).  MUST FAIL.
    adc_timing_harness #(.SCLK_OUT_NS(28.0), .SDO_IN_NS(28.0))                 hF  (.done(dF),  .pass(pF));

    task show(input [8*24:1] name, input p, input real out_ns, input real tdo, input real in_ns,
              input real su_bit, input real su_msb, input real hold, input real per_ns);
        real tclk, rt, v62;
        begin
            tclk = 41.667;
            rt   = out_ns + tdo + in_ns;
            v62  = tclk - rt;
            $display("  %-0s round trip %5.1f ns | v62 margin %6.1f | v63 setup bit %6.1f MSB %6.1f hold %6.1f ns | %4.0f kSPS -> %s",
                     name, rt, v62, su_bit, su_msb, hold, (per_ns > 0.0) ? 1.0e6/per_ns : 0.0, p ? "PASS" : "FAIL");
        end
    endtask

    integer fails = 0;
    initial begin
        wait (dP & dW & dR & dR2 & dZ & dF);
        #1;
        $display("");
        $display("=== adc_mcp33131 @ 24 MHz clk (12 MHz SCLK), SDO sampled mid-SCLK-high (v63) ===");
        $display("  (setup/hold measured at the SB_IO falling-edge sample; -1 = sampled while SDO changed)");
        show("3.3V tDO 9.5 (ship)   ", pP,  6.0,  9.5,  6.0,  hP.su_bit,  hP.su_msb,  hP.hold,  hP.meas_period_ns);
        show("1.7V tDO 16 (corner)  ", pW,  6.0,  16.0, 6.0,  hW.su_bit,  hW.su_msb,  hW.hold,  hW.meas_period_ns);
        show("R 33+100 (15+15 ns)   ", pR,  15.0, 9.5,  15.0, hR.su_bit,  hR.su_msb,  hR.hold,  hR.meas_period_ns);
        show("R 100+100 (18+18 ns)  ", pR2, 18.0, 9.5,  18.0, hR2.su_bit, hR2.su_msb, hR2.hold, hR2.meas_period_ns);
        show("zero delay (hold)     ", pZ,  0.0,  0.0,  0.0,  hZ.su_bit,  hZ.su_msb,  hZ.hold,  hZ.meas_period_ns);
        show("28+28 ns (must FAIL)  ", pF,  28.0, 9.5,  28.0, hF.su_bit,  hF.su_msb,  hF.hold,  hF.meas_period_ns);
        $display("");
        $display("  Shipping config: max sample rate ~= %0.0f kSPS (back to back; the firmware floors the period at 60 clk = 400 kSPS)",
                 1.0e6/hP.meas_period_ns);
        $display("");

        if (!pP)  begin $display("UNEXPECTED: shipping 3.3V config FAILED");       fails = fails + 1; end
        if (!pW)  begin $display("UNEXPECTED: 1.7V tDO corner FAILED");            fails = fails + 1; end
        if (!pR)  begin $display("UNEXPECTED: 33+100 ohm round trip FAILED");      fails = fails + 1; end
        if (!pR2) begin $display("UNEXPECTED: 100+100 ohm round trip FAILED");     fails = fails + 1; end
        if (!pZ)  begin $display("UNEXPECTED: zero-delay hold corner FAILED");     fails = fails + 1; end
        // the shipping row must keep at least 30 ns of setup on every bit
        if (hP.su_bit < 30.0 || hP.su_msb < 30.0) begin
            $display("UNEXPECTED: shipping setup margin below 30 ns (bit %0.1f, MSB %0.1f)", hP.su_bit, hP.su_msb);
            fails = fails + 1;
        end
        if (pF) begin $display("UNEXPECTED: 65.5 ns round trip PASSED: the harness cannot detect a settle violation, the oracle is broken"); fails = fails + 1; end

        if (fails == 0)
            $display("PASS tb_adc_timing: reads clean with positive setup/hold from zero delay to a 45.5 ns round trip (v62 failed above 41.7); a 65.5 ns round trip is detected");
        else
            $display("FAIL tb_adc_timing: %0d unexpected result(s)", fails);
        $finish;
    end

    initial begin #800000 $display("FAIL tb_adc_timing: timeout"); $finish; end
endmodule
