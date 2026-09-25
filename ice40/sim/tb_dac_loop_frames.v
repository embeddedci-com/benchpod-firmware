// ============================================================================
// tb_dac_loop_frames.v — every DAC frame the closed loop sends is a value the loop's
// output `v` actually held: never a TORN frame mixing two of them.
//
// dac8551_engine takes a stream sample as two pops, the low byte (S_PLO) and then the
// high byte (S_PHI) two clk48 cycles later.  Before v38, dac_loop served both bytes
// straight from `v`, so a tick that updated `v` between the two pops sent
// {new high byte, old low byte}: e.g. 0x00FF -> 0x0100 went out as 0x01FF.  The loop
// drives real power, so one such frame is a real output glitch.
//
// Real dac_loop + real dac8551_engine (stream mode) on one clk48, frames decoded off the
// SPI pins exactly like tb_dac8551.  The loop is made to move `v` several times per DAC
// frame (sweep source over an all-distinct ramp curve, k = 1.0, the shortest tick and DAC
// divider), so ticks land on every phase of the frame.  Every decoded frame must be a value
// `v` held at some point; the run must also produce many frames and many distinct values,
// so a stuck loop cannot pass.
// Run: make loopframetest
// ============================================================================
`timescale 1ns/1ps
module tb_dac_loop_frames;
    localparam ADDR_W = 12, SHIFT = 5;

    reg clk48 = 0; always #10.4166 clk48 = ~clk48;
    reg rst48 = 1, arm = 0, start = 0;

    wire [ADDR_W-1:0] lut_raddr;
    reg  [7:0]  lut_rdata;
    wire [7:0]  strm_data; wire strm_valid, strm_pop;
    wire [15:0] v_out, in_used, idx_used;
    wire        tripped;
    wire [11:0] wave_addr;
    wire        sync, sclk, din, running;

    dac_loop #(.ADDR_W(ADDR_W), .SHIFT(SHIFT)) loop (
        .clk48(clk48), .rst48(rst48), .arm(arm),
        .adc_sample(16'd0),
        .k_q15(16'd32767), .vmin(16'd0), .vmax(16'hFFFF), .tick_div(16'd0),
        .src_sel(2'd2), .in_fixed(16'd0), .sweep_step(16'd7 << SHIFT),   // index +7 per tick
        .in_zero(16'd0), .in_gain(16'sd0), .in_trip(11'd0),
        .map_en(1'b0), .trip_en(1'b0),
        .lut_raddr(lut_raddr), .lut_rdata(lut_rdata),
        .strm_data(strm_data), .strm_valid(strm_valid), .strm_pop(strm_pop),
        .v_out(v_out), .in_used(in_used), .idx_used(idx_used), .tripped(tripped)
    );
    dac8551_engine #(.ADDR_W(12)) dac (
        .clk(clk48), .rst(rst48), .start(start), .stop(1'b0),
        .period_samples(13'd0), .divider(16'd0),
        .wave_addr(wave_addr), .wave_data(8'd0),
        .psram_mode(1'b1), .strm_data(strm_data), .strm_valid(strm_valid), .strm_pop(strm_pop),
        .dac_sync(sync), .dac_sclk(sclk), .dac_din(din), .running(running)
    );

    // ---- curve: every entry distinct, and both bytes move from one entry to the next ----
    reg [7:0] lut [0:(1<<ADDR_W)-1];
    always @(posedge clk48) lut_rdata <= lut[lut_raddr];
    integer i;
    reg [15:0] val;
    initial for (i = 0; i < (1<<(ADDR_W-1)); i = i + 1) begin
        val = 16'd1000 + i[15:0] * 16'd29;
        lut[{i[ADDR_W-2:0],1'b0}] = val[7:0];
        lut[{i[ADDR_W-2:0],1'b1}] = val[15:8];
    end

    // ---- every value v has held ----
    reg seen [0:65535];
    integer nvals = 0;
    initial for (i = 0; i < 65536; i = i + 1) seen[i] = 1'b0;
    always @(posedge clk48) if (!seen[v_out]) begin seen[v_out] = 1'b1; nvals = nvals + 1; end

    // ---- frame decoder (as tb_dac8551): DIN taken on SCLK falling with SYNC low ----
    integer errors = 0, bits = 0, nfr = 0, torn = 0;
    reg     ps = 1'b1, pc = 1'b0, pd = 1'b0;
    reg [23:0] sh = 24'd0;
    always @(posedge clk48) if (!rst48) begin
        if (!sclk && pc && !ps) begin
            sh = {sh[22:0], pd};
            bits = bits + 1;
            if (bits == 24) begin
                nfr = nfr + 1;
                if (!seen[sh[15:0]]) begin
                    torn = torn + 1;
                    if (torn <= 5)
                        $display("FAIL frame %0d = %04h: never a value of v (torn lo/hi)", nfr, sh[15:0]);
                end
            end
        end
        if (!sync && ps) bits = 0;
        ps = sync; pc = sclk; pd = din;
    end

    initial begin
        repeat (5) @(posedge clk48); rst48 = 0;
        @(negedge clk48) arm = 1;
        @(negedge clk48) start = 1; @(negedge clk48) start = 0;
        repeat (60000) @(posedge clk48);
        if (torn != 0) begin
            $display("FAIL: %0d of %0d DAC frames were torn", torn, nfr); errors = errors + 1;
        end
        if (nfr < 500 || nvals < 1000) begin
            $display("FAIL: only %0d frames / %0d distinct v values: the loop did not exercise the DAC", nfr, nvals);
            errors = errors + 1;
        end
        if (errors == 0)
            $display("PASS tb_dac_loop_frames: %0d DAC frames over %0d distinct v values, none torn", nfr, nvals);
        $finish;
    end
endmodule
