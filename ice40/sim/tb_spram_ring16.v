// tb_spram_ring16.v — spram_ring16 (v45 wrap-bit pointers) against the v44 ring with its
// separate `count` register (sim/spram_ring16_ref.v), cycle for cycle, at AW=3 (8 words) so
// random producer bursts and consumer stalls drive it through full, empty and wrap-around
// thousands of times.  Every output must match every cycle.  Then (v46) the overflow clear: flood
// the ring until it drops a sample, pulse clr_ovf, and check the flag stays clear while clean
// traffic drains it (until v45 it was sticky until rst).
// Run:  make ringtest
`timescale 1ns/1ps
module tb_spram_ring16;
    reg clk = 0; always #5 clk = ~clk;
    reg rst = 1;
    reg [7:0] in_data = 0; reg in_stb = 0; reg out_full = 0; reg clr = 0; reg cmp = 1;
    wire in_full_a, out_stb_a, empty_a, ovf_a; wire [7:0] out_a;
    wire in_full_b, out_stb_b, empty_b, ovf_b; wire [7:0] out_b;
    spram_ring16     #(.AW(3)) a (.clk(clk), .rst(rst), .clr_ovf(clr), .in_data(in_data), .in_stb(in_stb), .in_full(in_full_a),
        .out_data(out_a), .out_stb(out_stb_a), .out_full(out_full), .empty(empty_a), .overflow(ovf_a));
    spram_ring16_ref #(.AW(3)) b (.clk(clk), .rst(rst), .in_data(in_data), .in_stb(in_stb), .in_full(in_full_b),
        .out_data(out_b), .out_stb(out_stb_b), .out_full(out_full), .empty(empty_b), .overflow(ovf_b));
    integer errors = 0, i, seed = 7, fulls = 0, empties = 0, burst = 0;
    always @(negedge clk) if (!rst && cmp) begin
        if ({in_full_a, out_stb_a, empty_a, ovf_a} !== {in_full_b, out_stb_b, empty_b, ovf_b} ||
            (out_stb_a && out_a !== out_b)) begin
            errors = errors + 1;
            if (errors < 8) $display("FAIL t=%0t new f%b s%b e%b o%b d%02x  ref f%b s%b e%b o%b d%02x", $time,
                in_full_a, out_stb_a, empty_a, ovf_a, out_a, in_full_b, out_stb_b, empty_b, ovf_b, out_b);
        end
        if (!a.cnt_nfull) fulls = fulls + 1;
        if (empty_a) empties = empties + 1;
    end
    initial begin
        #20 rst = 0;
        for (i = 0; i < 300000; i = i + 1) begin
            @(posedge clk); #1;
            if (burst == 0 && $urandom(seed) % 50 == 0) burst = $urandom(seed) % 64;
            in_stb   = (burst > 0) && ($urandom(seed) % 3 != 0);
            if (in_stb) begin in_data = in_data + 1; burst = burst - 1; end
            out_full = ($urandom(seed) % 100) < (i % 20000 < 10000 ? 90 : 20);   // stall phases
        end
        // ---- v46: overflow, clear, clean traffic ----
        // (whole 2-byte samples, as the LA producer writes: first complete a half-written word)
        cmp = 0; out_full = 1; in_stb = 0;
        @(posedge clk); #1; if (a.lo_v) begin in_stb = 1; in_data = in_data + 1; @(posedge clk); #1; in_stb = 0; end
        for (i = 0; i < 64; i = i + 1) begin @(posedge clk); #1; in_stb = 1; in_data = in_data + 1; end
        @(posedge clk); #1; in_stb = 0;
        if (ovf_a !== 1'b1) begin $display("FAIL tb_spram_ring16: a flooded ring did not flag overflow"); errors = errors + 1; end
        @(posedge clk); #1; clr = 1; @(posedge clk); #1; clr = 0;
        if (ovf_a !== 1'b0) begin $display("FAIL tb_spram_ring16: clr_ovf left overflow set"); errors = errors + 1; end
        out_full = 0;
        for (i = 0; i < 400; i = i + 1) begin                 // drain, then clean paced traffic
            @(posedge clk); #1;
            in_stb = (i > 100) && (i % 6 < 2); if (in_stb) in_data = in_data + 1;
            if (ovf_a !== 1'b0) begin
                if (errors < 8) $display("FAIL tb_spram_ring16: overflow set again by clean traffic at %0d", i);
                errors = errors + 1;
            end
        end
        if (errors == 0 && fulls > 0 && empties > 0)
            $display("PASS tb_spram_ring16: == v44 ring on 300k cycles (%0d cycles full, %0d empty)", fulls, empties);
        else $display("FAIL tb_spram_ring16: %0d mismatches, %0d full cycles, %0d empty cycles", errors, fulls, empties);
        $finish;
    end
endmodule
