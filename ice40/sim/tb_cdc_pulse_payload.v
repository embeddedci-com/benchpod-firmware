// ============================================================================
// tb_cdc_pulse_payload.v — every crossed pulse must arrive exactly once, and the consumer
// that loads dst_data ON dst_pulse must see THAT pulse's value (never the previous one —
// the v33 DAC restart bug).
//
// Two instances: clk -> clk48 (the synchronous 1:2 ratio top_v2 uses) and an asynchronous
// ~18.9 MHz source.  Each sends N pulses with a fresh random value, spaced per the module's
// source-side contract.  The checker is a flop on dst_clk that loads dst_data when dst_pulse
// is high — sampling at the posedge reads exactly what such a flop would load.
//
// Run:  make cdctest
// ============================================================================
`timescale 1ns/1ps
module tb_cdc_pulse_payload;
    reg clk48 = 1'b0; always #10 clk48 = ~clk48;
    reg clk   = 1'b0; always @(posedge clk48) clk <= ~clk;     // clk48/2, like top_v2
    reg aclk  = 1'b0; always #26.5 aclk = ~aclk;               // unrelated source clock

    localparam W = 16, N = 300;
    integer errors = 0, seed = 7;

    reg          a_pulse = 1'b0, b_pulse = 1'b0;
    reg  [W-1:0] a_data  = 16'h0000, b_data = 16'h0000;
    wire         a_dst_pulse, b_dst_pulse;
    wire [W-1:0] a_dst_data, b_dst_data;

    cdc_pulse_payload #(.W(W), .INIT(16'hBEEF)) dut_a (
        .src_clk(clk), .src_pulse(a_pulse), .src_data(a_data),
        .dst_clk(clk48), .dst_pulse(a_dst_pulse), .dst_data(a_dst_data));
    cdc_pulse_payload #(.W(W), .INIT(16'hBEEF)) dut_b (
        .src_clk(aclk), .src_pulse(b_pulse), .src_data(b_data),
        .dst_clk(clk48), .dst_pulse(b_dst_pulse), .dst_data(b_dst_data));

    reg [W-1:0] a_exp [0:N-1];
    reg [W-1:0] b_exp [0:N-1];
    integer a_src = 0, a_dst = 0, b_src = 0, b_dst = 0;

    always @(posedge clk48) begin
        if (a_dst_pulse) begin
            if (a_dst >= a_src) begin
                $display("FAIL sync: extra pulse %0d (only %0d sent)", a_dst, a_src); errors = errors + 1;
            end else if (a_dst_data !== a_exp[a_dst]) begin
                $display("FAIL sync: pulse %0d carried %04h, want %04h", a_dst, a_dst_data, a_exp[a_dst]);
                errors = errors + 1;
            end
            a_dst = a_dst + 1;
        end
        if (b_dst_pulse) begin
            if (b_dst >= b_src) begin
                $display("FAIL async: extra pulse %0d (only %0d sent)", b_dst, b_src); errors = errors + 1;
            end else if (b_dst_data !== b_exp[b_dst]) begin
                $display("FAIL async: pulse %0d carried %04h, want %04h", b_dst, b_dst_data, b_exp[b_dst]);
                errors = errors + 1;
            end
            b_dst = b_dst + 1;
        end
    end

    // Source drivers change on the source clock's negedge (no sampling race).  The value of
    // each pulse is held until the next one; a gap of >= 3 src cycles keeps it for >= 4 dst
    // cycles after the sampling edge in both instances (the module's contract).
    task drive_a;
        integer k, gap; begin
            for (k = 0; k < N; k = k + 1) begin
                gap = 3 + (($random(seed) & 32'h7FFFFFFF) % 6);
                repeat (gap) @(negedge clk);
                a_data = $random(seed); a_exp[k] = a_data; a_src = k + 1;
                a_pulse = 1'b1; @(negedge clk); a_pulse = 1'b0;
            end
        end
    endtask
    task drive_b;
        integer k, gap; begin
            for (k = 0; k < N; k = k + 1) begin
                gap = 3 + (($random(seed) & 32'h7FFFFFFF) % 4);
                repeat (gap) @(negedge aclk);
                b_data = $random(seed); b_exp[k] = b_data; b_src = k + 1;
                b_pulse = 1'b1; @(negedge aclk); b_pulse = 1'b0;
            end
        end
    endtask

    initial begin
        repeat (3) @(posedge clk48);
        if (a_dst_data !== 16'hBEEF || b_dst_data !== 16'hBEEF) begin
            $display("FAIL: dst_data did not start at INIT (%04h, %04h)", a_dst_data, b_dst_data); errors = errors + 1;
        end
        fork drive_a; drive_b; join
        repeat (20) @(posedge clk48);
        if (a_dst != N) begin $display("FAIL sync: %0d of %0d pulses arrived", a_dst, N); errors = errors + 1; end
        if (b_dst != N) begin $display("FAIL async: %0d of %0d pulses arrived", b_dst, N); errors = errors + 1; end
        if (errors == 0)
            $display("PASS tb_cdc_pulse_payload: %0d sync + %0d async pulses, each consumed with its own value", a_dst, b_dst);
        else
            $display("FAIL tb_cdc_pulse_payload: %0d error(s)", errors);
        $finish;
    end
    initial begin #20000000 $display("FAIL tb_cdc_pulse_payload: timeout"); $finish; end
endmodule
