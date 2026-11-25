// ============================================================================
// mcp33131_model.v — analog-accurate behavioural model of the MCP33131D-10
// (16-bit, 1 MSPS SAR ADC, 3-wire serial interface) for gateware timing sims.
//
// Unlike the trivial shift-register in tb_adc_mcp33131.v (which changes SDO with
// ZERO delay and therefore passes at ANY clock), this model reproduces the real
// analog timing that forced the capture engine down to a 24 MHz control clock:
//
//   • CNVST rising  -> start conversion; the analog input `analog_code` is LATCHED
//                      at that edge (SAR sample-and-hold).  Conversion takes tCONV.
//   • CNVST falling -> read phase.  After tEN the MSB (D15) is presented on SDO,
//                      BEFORE the first SCLK edge (datasheet Fig 7-2).
//   • SDO changes on the SCLK *falling* edge (datasheet Table 1-2 "tDO — Output
//     Valid from SCLK Low").  For a window of tDO after each falling edge SDO is
//     INDETERMINATE — the model drives 1'bx there.  A capture engine that samples
//     inside that window reads x and the testbench flags a FAIL.  This is exactly
//     the "sampled mid-transition -> 0x5555" failure seen on hardware at 24 MHz
//     SCLK.  The engine samples in the middle of SCLK high (v63: the falling clk edge
//     after the SCLK rise, 1.5 clk after the SCLK fall left the FPGA; v62 and earlier
//     sampled 1 clk after, on the SCLK rise).
//   • `bit_evt` toggles when a new bit is valid on SDO (the MSB tEN after CNVST falls,
//     later bits tDO after an SCLK fall) and `chg_evt` when SDO starts to change (CNVST
//     fall, SCLK fall).  Delay them like SDO and the testbench measures the setup and
//     hold margins at the engine's sampling instant.
//
// Datasheet numbers (DS20005947B, Table 1-2, DVIO >= 3.3 V unless noted):
//   tCONV(tCNV) max  710 ns      tEN max 10 ns       tDO max  9.5 ns @3.3V
//   tACQ min         290 ns      tDIS max 15 ns      tDO max 16   ns @1.7V
//   tCYC min           1 us (=1 MSPS)   fSCLK max 100 MHz (tSCLK min 10 ns @3.3V)
//   tQUIET min        10 ns (last SCLK edge -> next CNVST rising)
//
// The engine drives adc_sclk/adc_cnvst; wire the physical round-trip delay in the
// testbench so the model sees SCLK late and the engine sees SDO late:
//
//   engine.adc_sclk  --#(SCLK_OUT_NS)-->  model.sclk
//   model.sdo        --#(SDO_IN_NS)  -->  engine.adc_sdo
//
// so the effective settle window at the engine's sample FF is
//   1.5 * T_clk  -  (SCLK_OUT_NS + tDO + SDO_IN_NS)          (v63; 1 * T_clk until v62)
// which is what actually has to stay positive.  Those pad/cable round-trip terms
// (series resistors on the module FFC, ESD array capacitance) — not tDO alone — are
// what eats the margin.
// ============================================================================
`timescale 1ns/1ps
module mcp33131_model #(
    parameter real TCONV_NS = 710.0,   // conversion time (max, <=85C)
    parameter real TEN_NS   = 10.0,    // SDO enable (MSB valid) after CNVST low
    parameter real TDO_NS   = 9.5,     // SDO valid after SCLK falling (DVIO 3.3V)
    parameter real TDIS_NS  = 15.0     // SDO -> Hi-Z after CNVST high / last bit
)(
    input  wire        cnvst,          // conversion-start + active-low chip-select
    input  wire        sclk,           // serial clock (already pad-delayed by TB)
    input  wire        sdi,            // must be held HIGH for normal 3-wire op
    input  wire [15:0] analog_code,    // the code the ADC will "convert" this cycle
    output reg         sdo,            // serial data out (drive back through SDO_IN)
    output reg         bit_evt,        // toggles: a new bit is valid on sdo
    output reg         chg_evt,        // toggles: sdo starts to change (invalid until bit_evt)

    // observability for the testbench
    output reg         conv_active,    // high during tCONV
    output reg         sdi_ok          // 0 if SDI was ever seen low during a cycle
);
    reg [15:0] latched;                // SAR sample-and-hold, captured at CNVST rise
    reg [15:0] result;                 // conversion result, valid after tCONV
    integer    idx;                    // bit index being shifted out (15..0)
    reg        reading;                // in the read phase (CNVST low, post-tEN)

    initial begin
        sdo = 1'b1;                    // idle high (mimics the FPGA pull-up on SDO)
        conv_active = 1'b0; reading = 1'b0; sdi_ok = 1'b1; idx = 0;
        bit_evt = 1'b0; chg_evt = 1'b0;
        latched = 16'h0000; result = 16'h0000;
    end

    // SDI must stay high for the whole cycle (datasheet Fig 7-2 Note 1). Latch a
    // violation so the TB can prove the "SDI tied low" mis-wiring is caught too.
    always @(*) if (sdi !== 1'b1) sdi_ok = 1'b0;

    // ---- conversion: CNVST rising edge starts it, result ready after tCONV ----
    always @(posedge cnvst) begin
        latched     <= analog_code;    // sample-and-hold locks the input here
        reading     <= 1'b0;
        sdo         <= 1'b1;           // SDO releases (idle high) while converting
        conv_active <= 1'b1;
        #(TCONV_NS);
        result      <= latched;        // conversion complete
        conv_active <= 1'b0;
    end

    // ---- read phase: CNVST falling edge -> present MSB after tEN --------------
    always @(negedge cnvst) begin
        chg_evt = ~chg_evt;
        sdo     = 1'bx;                // leaving Hi-Z/idle: indeterminate until tEN
        #(TEN_NS);
        idx     = 15;
        sdo     = result[15];          // MSB valid before the first SCLK edge
        bit_evt = ~bit_evt;
        reading = 1'b1;
    end

    // ---- each SCLK FALLING edge advances to the next bit, tDO later -----------
    // Between the falling edge and tDO, SDO is indeterminate (drive x). Any engine
    // that samples in that window captures x -> the read is corrupted.
    always @(negedge sclk) begin
        if (reading && !cnvst) begin
            sdo = 1'bx;                // indeterminate during the tDO transition
            chg_evt = ~chg_evt;
            #(TDO_NS);
            if (idx > 0) idx = idx - 1;
            sdo = result[idx];         // next bit settled and held until next fall
            bit_evt = ~bit_evt;
        end
    end

    // ---- SDO returns to idle/Hi-Z when CNVST goes back high -------------------
    always @(posedge cnvst) begin
        #(TDIS_NS);
        if (cnvst) begin reading = 1'b0; sdo = 1'b1; end
    end
endmodule
