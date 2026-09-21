// ============================================================================
// tb_adc_timing.v — timing + throughput sign-off for the PRODUCTION adc_mcp33131
// running its FSM on the 24 MHz control clock, against the analog-accurate
// mcp33131_model (real tCONV/tEN/tDO + explicit pad/PCB round-trip).
//
// The stock sim/tb_adc_mcp33131.v changes SDO with ZERO delay and therefore can't
// see the SDO-settle failure that once forced the engine off the 48 MHz capture
// clock; this one can.  It proves the shipping configuration reads the injected
// analog code correctly and reports the sample rate we actually achieve.
//
// The engine's SCLK is hard-wired to clk/2 — there is NO divider parameter any
// more (the capture datapath was collapsed onto the single 24 MHz `clk`, see
// docs/adc-clock-analysis.md and the "24 MHz ADC-engine workaround" note).  At the
// 24 MHz control clock that is a 12 MHz SCLK, and the engine samples SDO one clk
// period after the previous SCLK falling edge, so the settle window is exactly ONE
// clk period (41.67 ns).  What must stay positive is the pad/PCB round-trip budget:
//
//   margin = T_clk - (SCLK_OUT + tDO + SDO_IN) = 41.67 - (6 + tDO + 6)
//          = +20.2 ns  at 3.3 V DVIO (tDO 9.5 ns)
//          = +13.7 ns  at the 1.7 V corner (tDO 16 ns)
//
// Both comfortably positive — which is exactly why 12 MHz SCLK reads rock-solid,
// where the old 24 MHz-SCLK (48 MHz FSM) attempt had a -0.7 ns margin, sampled
// mid-transition and read back the 0x5555 coupling pattern on hardware.
//
// Run:  make adctimingtest
// ============================================================================
`timescale 1ns/1ps

module adc_timing_harness #(
    parameter real    CLK_NS      = 20.833, // clk HALF-period (20.833 -> 24 MHz)
    parameter real    TDO_NS      = 9.5,    // SDO valid after SCLK low (3.3 V DVIO)
    parameter real    SCLK_OUT_NS = 6.0,    // FPGA FF -> ADC SCLK pin (pad + PCB)
    parameter real    SDO_IN_NS   = 6.0,    // ADC SDO pin -> FPGA sample FF
    parameter integer NSAMP       = 12,
    parameter [15:0]  DIVIDER     = 16'd2   // request max rate (engine floors it)
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

    // physical round-trip: SCLK late into the ADC, SDO late back to the FPGA
    wire sclk_at_adc, sdo_at_fpga;
    assign #(SCLK_OUT_NS) sclk_at_adc = sclk;
    assign #(SDO_IN_NS)   sdo_at_fpga = sdo_from_adc;

    reg [15:0] pat = 16'hA53C;
    wire [15:0] analog_code = pat;
    wire conv_active, sdi_ok;

    // Production engine: SCLK = clk/2 = 12 MHz, hard-wired (no SCLK_DIV param).
    adc_mcp33131 dut (
        .clk(clk), .rst(rst), .en(1'b1), .divider(DIVIDER),
        .adc_cnvst(cnvst), .adc_sclk(sclk), .adc_sdi(sdi), .adc_sdo(sdo_at_fpga),
        .sample(sample), .sample_stb(sample_stb)
    );

    mcp33131_model #(.TDO_NS(TDO_NS)) adc (
        .cnvst(cnvst), .sclk(sclk_at_adc), .sdi(sdi), .analog_code(analog_code),
        .sdo(sdo_from_adc), .conv_active(conv_active), .sdi_ok(sdi_ok)
    );

    reg [15:0] expected;
    always @(posedge cnvst) expected <= analog_code;

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
        pass = (bad == 0) && sdi_ok;
        done = 1'b1;
    end
endmodule

module tb_adc_timing;
    wire dP,pP, dW,pW, dF,pF;
    // Shipping config: 24 MHz FSM, SCLK = clk/2 = 12 MHz, 3.3 V tDO, real pads
    adc_timing_harness                   hP (.done(dP), .pass(pP));
    // Worst-case tDO (1.7 V corner) — must still pass
    adc_timing_harness #(.TDO_NS(16.0))  hW (.done(dW), .pass(pW));
    // NEGATIVE CONTROL — the historical failing edge: run the SAME engine on a
    // 48 MHz clk (CLK_NS 10.417) => 24 MHz SCLK, settle window ~20.8 ns, margin
    // ~-0.7 ns.  The engine samples SDO inside the model's tDO x-window and reads
    // the 0x5555-family garbage that bit us on hardware.  This MUST FAIL — it
    // proves the harness can actually DETECT a settle violation; without it the
    // test would be a rubber stamp that only ever goes green.
    adc_timing_harness #(.CLK_NS(10.417)) hF (.done(dF), .pass(pF));

    task show(input [8*26:1] name, input p, input real clk_half_ns, input real tdo, input real per_ns);
        real tclk, sclk_mhz, settle, margin;
        begin
            tclk     = 2.0*clk_half_ns;       // clk full period
            sclk_mhz = 1000.0/(2.0*tclk);     // SCLK = clk/2
            settle   = tclk;                  // settle window = 1 clk period
            margin   = settle - (12.0 + tdo); // - (pad-out 6 + tDO + pad-in 6)
            $display("  %-0s SCLK=%5.2f MHz  settle=%5.1f ns  margin=%6.2f ns  period=%6.0f ns (~%4.0f kSPS)  -> %s",
                     name, sclk_mhz, settle, margin, per_ns,
                     (per_ns>0.0)?1.0e6/per_ns:0.0, p?"PASS":"FAIL");
        end
    endtask

    integer fails = 0;
    initial begin
        wait (dP & dW & dF);
        #1;
        $display("");
        $display("=== adc_mcp33131 @ 24 MHz FSM (12 MHz SCLK) — production timing + throughput ===");
        $display("  (settle = 1 clk period - [pad-out 6 + tDO + pad-in 6])");
        show("3.3V tDO=9.5 (SHIP) ", pP, 20.833, 9.5,  hP.meas_period_ns);
        show("1.7V tDO=16 (corner)", pW, 20.833, 16.0, hW.meas_period_ns);
        show("48MHz FSM (must FAIL)", pF, 10.417, 9.5,  hF.meas_period_ns);
        $display("");
        $display("  Shipping config: max sample rate ~= %0.0f kSPS  (Nyquist BW ~= %0.0f kHz)",
                 1.0e6/hP.meas_period_ns, 0.5e6/hP.meas_period_ns);
        $display("");

        if (!pP) begin $display("UNEXPECTED: shipping 3.3V config FAILED");                fails=fails+1; end
        if (!pW) begin $display("UNEXPECTED: 1.7V worst-case tDO FAILED");                 fails=fails+1; end
        if ( pF) begin $display("UNEXPECTED: 48 MHz FSM (24 MHz SCLK) PASSED — the harness cannot detect a settle violation, the oracle is broken"); fails=fails+1; end

        if (fails == 0)
            $display("PASS tb_adc_timing: 12 MHz SCLK reads correctly at nominal AND 1.7V-corner tDO; 24 MHz SCLK is the failing edge (harness detects it)");
        else
            $display("FAIL tb_adc_timing: %0d unexpected result(s)", fails);
        $finish;
    end

    initial begin #800000 $display("FAIL tb_adc_timing: timeout"); $finish; end
endmodule
