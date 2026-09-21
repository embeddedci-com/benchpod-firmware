// ============================================================================
// tb_dac_psram_reader.v — self-checking test of the QPI streaming READ master.
//
// Models an APS6404L answering the Fast-Quad-Read (0xEB) command: on each SCLK
// rising edge with CS low it reconstructs cmd (2 nib) + 24-bit addr (6 nib), skips
// WAIT_CYCLES dummy clocks, then DRIVES data nibbles from mem[] (high nibble first)
// so the reader samples them.  The reader's QPI FSM is on `clk` (24 MHz) and its
// prefetch FIFO is DUAL-CLOCK — this test pops out of the clk48 read port at a
// DAC-like rate and checks the bytes match PSRAM contents in order, INCLUDING the
// loop wrap at len_bytes (so a whole waveform replays repeatedly).
//
// Run:  make dacpsramrdtest
// ============================================================================
`timescale 1ns/1ps
module tb_dac_psram_reader;
    localparam [8:0]  CHUNK = 9'd8;
    localparam [3:0]  WAIT  = 4'd6;
    localparam [23:0] BASE  = 24'h000100;   // nonzero base -> exercises the address nibbles
    localparam [23:0] LEN   = 24'd50;        // waveform length in bytes (25 samples)

    // clk48 primary; clk = clk48/2, phase-locked (exactly like top_v2).
    reg clk48 = 0; always #10 clk48 = ~clk48;
    reg clk = 0;   always @(posedge clk48) clk <= ~clk;
    reg rst = 1, rst48 = 1, run = 0;

    // reader <-> tri bus
    wire [3:0] rd_io_o; wire rd_io_oe; wire ps_cs, ps_sclk;
    tri  [3:0] psram_io;
    wire [7:0] rd_data; wire rd_valid; reg rd_pop = 0;

    dac_psram_reader #(.CHUNK_BYTES(CHUNK), .WAIT_CYCLES(WAIT), .FIFO_AW(5)) dut (
        .clk(clk), .rst(rst), .clk48(clk48), .rst48(rst48), .run(run),
        .base_addr(BASE), .len_bytes(LEN),
        .bus_gnt(1'b1), .bus_req(), .bus_busy(),
        .data(rd_data), .data_valid(rd_valid), .data_pop(rd_pop),
        .io_o(rd_io_o), .io_oe(rd_io_oe), .io_i(psram_io),
        .cs(ps_cs), .sclk(ps_sclk), .active()
    );
    assign psram_io = rd_io_oe ? rd_io_o : 4'bzzzz;   // reader drives cmd/addr

    // ---- APS6404L read model ----
    reg  [7:0] mem [0:1023];
    integer    mi;
    initial for (mi=0;mi<1024;mi=mi+1) mem[mi] = mi[7:0] ^ 8'h5A;   // deterministic pattern

    integer    mnib;
    reg [23:0] maddr;
    reg [3:0]  mdrive; reg mdriving;
    reg [7:0]  mbyte;
    localparam DATA0 = 2 + 6 + 6;   // first data nibble index = cmd+addr+WAIT (WAIT=6)

    assign psram_io = mdriving ? mdrive : 4'bzzzz;    // model drives data

    always @(negedge ps_cs) begin mnib = 0; mdriving = 1'b0; end
    always @(posedge ps_sclk) if (!ps_cs) begin
        if (mnib < 2) mdriving <= 1'b0;
        else if (mnib < 8) begin
            case (mnib)
                2: maddr[23:20] = psram_io;
                3: maddr[19:16] = psram_io;
                4: maddr[15:12] = psram_io;
                5: maddr[11:8]  = psram_io;
                6: maddr[7:4]   = psram_io;
                7: maddr[3:0]   = psram_io;
            endcase
            mdriving <= 1'b0;
        end else if (mnib < DATA0) mdriving <= 1'b0;
        else begin
            if (((mnib - DATA0) & 1) == 0) begin mbyte = mem[maddr[9:0]]; mdrive <= mbyte[7:4]; end
            else begin mdrive <= mbyte[3:0]; maddr = maddr + 24'd1; end
            mdriving <= 1'b1;
        end
        mnib = mnib + 1;
    end
    always @(posedge ps_cs) mdriving <= 1'b0;

    // ---- pop bytes on the clk48 read port at a DAC-like slow rate ----
    integer i, errors;
    reg [7:0] got;
    task pop_one(output [7:0] b); begin
        @(posedge clk48); while (!rd_valid) @(posedge clk48);
        b = rd_data; rd_pop <= 1'b1; @(posedge clk48); rd_pop <= 1'b0;
        repeat (16) @(posedge clk48);      // idle gap ~ DAC update period
    end endtask

    localparam NPOP = 120;              // > 2 full loops (LEN=50) to exercise the wrap
    initial begin
        errors = 0;
        repeat (4) @(posedge clk48); rst = 0; rst48 = 0;
        @(negedge clk); run = 1'b1;   // negedge: avoid the posedge sampling race (run is registered in real HW)
        for (i = 0; i < NPOP; i = i + 1) begin
            pop_one(got);
            if (got !== mem[(BASE[9:0] + (i % LEN)) & 10'h3FF]) begin
                $display("FAIL: byte[%0d] got %02h want %02h (pos %0d)",
                         i, got, mem[(BASE[9:0] + (i % LEN)) & 10'h3FF], i % LEN);
                errors = errors + 1;
            end
        end
        if (errors == 0)
            $display("PASS tb_dac_psram_reader: %0d bytes streamed in order across the loop wrap (len=%0d)", NPOP, LEN);
        else
            $display("FAIL tb_dac_psram_reader: %0d error(s)", errors);
        $finish;
    end

    initial begin #40000000 $display("FAIL tb_dac_psram_reader: timeout"); $finish; end
endmodule
