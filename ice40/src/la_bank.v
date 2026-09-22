// ============================================================================
// la_bank.v — Logic-Analyzer GPIO bank for the bench-pod iCE40 gateware.
//
// Owns the N bidirectional LA channels (LA1..LAN, wired in *.pcf; N=12 on v1,
// N=14 on v2) and muxes
// five drivers onto them with fixed priority  swd > stepper > i2c > uart > static :
//
//   static   per-channel {out,oe} latches set by the GPIO_SET command
//            (default oe=0 → high-Z).  This is the plain "drive a pin
//            high/low/high-Z" path.
//   i2c      while an emulated I2C sensor is armed, i2c_target owns the SDA
//            channel as an OPEN-DRAIN output: it only ever pulls SDA low
//            (out=0, oe=i2c_sda_drive_low) or releases it (oe=0, line pulled
//            high externally) — it NEVER drives SDA high.  SCL is read-only
//            (sampled via la_in); the bus needs external pull-ups.
//   uart     while a UART proxy is armed, uart_engine owns the TX channel as a
//            normal PUSH-PULL output (idle high); the RX channel is read-only
//            (sampled via la_in).
//   stepper  while a step train runs, stepper_engine owns ONE channel and
//            drives the pulse train onto it (oe forced high).
//   swd      while a SWD session is armed, swd_engine owns up to three
//            channels (SWCLK / SWDIO / nRESET); SWDIO is bidirectional so its
//            oe is driven by the engine (released during turnaround).
//
// Each channel is a single SB_IO in PIN_TYPE 6'b1010_01 = combinational
// tristate output (OUTPUT_ENABLE) + simple input (D_IN_0), so every channel
// can be driven, released to high-Z, or read back (needed for SWDIO sampling).
//
// Channels here are 0-based (0..N-1).  The firmware exposes them to the host as
// 1-based LA channel indices (LA1..LAN) and maps 1→0 on the wire.
// ============================================================================

module la_bank #(
    parameter N = 12
)(
    input  wire              clk,
    input  wire              rst,

    // ---- static control (GPIO_SET) ----
    input  wire              set_stb,    // 1-cycle strobe
    input  wire [3:0]        set_ch,     // 0..N-1
    input  wire [1:0]        set_mode,   // 0=low, 1=high, 2=high-Z

    // ---- i2c override (open-drain SDA) ----
    input  wire              i2c_active,        // emulated sensor armed
    input  wire [3:0]        i2c_sda_ch,
    input  wire              i2c_sda_drive_low, // 1 → pull SDA low; 0 → release

    // ---- uart override (push-pull TX) ----
    input  wire              uart_active,       // UART proxy armed
    input  wire [3:0]        uart_tx_ch,
    input  wire              uart_tx_val,       // TX line level (idle high)

    // ---- stepper override ----
    input  wire              step_active,
    input  wire [3:0]        step_ch,
    input  wire              step_val,

    // ---- swd override ----
    input  wire              swd_active,
    input  wire [3:0]        swd_clk_ch,
    input  wire              swd_clk_val,
    input  wire [3:0]        swd_dio_ch,
    input  wire              swd_dio_val,
    input  wire              swd_dio_oe,
    input  wire              swd_nrst_present,
    input  wire [3:0]        swd_nrst_ch,
    input  wire              swd_nrst_val,
    input  wire              swd_nrst_oe,

    // ---- physical pins + readback ----
    inout  wire [N-1:0]      la,
    output wire [N-1:0]      la_in
);

    // ---- static per-channel latches (GPIO_SET target) ----
    // Power-up initialisers (loaded from the bitstream at configuration) force
    // every channel to high-Z (output-enable low) the instant the FPGA boots —
    // before the reset sequence even runs — so the LA pins NEVER drive a DUT
    // until the host explicitly commands GPIO_SET / a stepper / SWD / I2C.  The
    // SB_IO instances below also disable internal pull-ups, so high-Z is a true
    // float.  The reset branch keeps them high-Z across any later reset.
    reg [N-1:0] s_out = {N{1'b0}};
    reg [N-1:0] s_oe  = {N{1'b0}};   // all high-Z (OE low) at power-up

    always @(posedge clk) begin
        if (rst) begin
            s_out <= {N{1'b0}};
            s_oe  <= {N{1'b0}};   // all high-Z at boot
        end else if (set_stb && set_ch < N) begin
            case (set_mode)
                2'd0: begin s_out[set_ch] <= 1'b0; s_oe[set_ch] <= 1'b1; end // low
                2'd1: begin s_out[set_ch] <= 1'b1; s_oe[set_ch] <= 1'b1; end // high
                default: begin               s_oe[set_ch] <= 1'b0; end       // high-Z
            endcase
        end
    end

    // ---- effective output/oe after mux (swd > stepper > i2c > uart > static) ----
    // Each of the five overrides can target ANY channel, chosen at runtime by a
    // 4-bit channel index.  The original form did a per-channel indexed WRITE
    // (`eff_out[uart_tx_ch] = ...`) inside a priority if-chain, which synthesises to
    // a 4-bit magnitude-compare of every driver's channel against every one of the
    // N bit positions (≈6 comparators × N bits).  Instead decode each driver's
    // target channel ONCE into an N-bit one-hot mask and drive the per-channel
    // priority mux from the masks — same priority, same `ch < N` guard, fewer LUTs
    // (this bank is almost pure combinational logic, so the LUTs it saves are LCs).
    // Formally equivalent to the old if-chain (200k-vector cross-check + logic proof).
    function [N-1:0] onehot;   // (en && ch<N) ? (1<<ch) : 0
        input en; input [3:0] ch;
        onehot = (en && ch < N) ? ({{(N-1){1'b0}}, 1'b1} << ch) : {N{1'b0}};
    endfunction
    // uart: push-pull TX (idle high).  i2c: open-drain SDA (never drives high).
    wire [N-1:0] m_uart = onehot(uart_active, uart_tx_ch);
    wire [N-1:0] m_i2c  = onehot(i2c_active,  i2c_sda_ch);
    wire [N-1:0] m_step = onehot(step_active, step_ch);
    wire [N-1:0] m_swdc = onehot(swd_active,  swd_clk_ch);
    wire [N-1:0] m_swdd = onehot(swd_active,  swd_dio_ch);
    wire [N-1:0] m_swdn = onehot(swd_active && swd_nrst_present, swd_nrst_ch);

    wire [N-1:0] eff_out, eff_oe;
    genvar c;
    generate for (c = 0; c < N; c = c + 1) begin : g_mux
        // priority (high→low): swd_nrst > swd_dio > swd_clk > step > i2c > uart > static
        assign eff_out[c] = m_swdn[c] ? swd_nrst_val :
                            m_swdd[c] ? swd_dio_val  :
                            m_swdc[c] ? swd_clk_val  :
                            m_step[c] ? step_val     :
                            m_i2c [c] ? 1'b0         :
                            m_uart[c] ? uart_tx_val  : s_out[c];
        assign eff_oe [c] = m_swdn[c] ? swd_nrst_oe  :
                            m_swdd[c] ? swd_dio_oe   :
                            m_swdc[c] ? 1'b1         :
                            m_step[c] ? 1'b1         :
                            m_i2c [c] ? i2c_sda_drive_low :
                            m_uart[c] ? 1'b1         : s_oe[c];
    end endgenerate

    // ---- bidirectional IO buffers ----
    genvar i;
    generate
        for (i = 0; i < N; i = i + 1) begin : g_io
            SB_IO #(
                .PIN_TYPE(6'b1010_01),   // tristate output + simple input
                .PULLUP  (1'b0)
            ) io_i (
                .PACKAGE_PIN  (la[i]),
                .OUTPUT_ENABLE(eff_oe[i]),
                .D_OUT_0      (eff_out[i]),
                .D_IN_0       (la_in[i])
            );
        end
    endgenerate

endmodule
