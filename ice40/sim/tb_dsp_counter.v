// tb_dsp_counter.v — dsp_counter (an SB_MAC16 accumulator) must behave exactly like the
// fabric counter it replaces, cycle for cycle: q and the free carry-out flag, counting up and
// down, with load and enable driven randomly (load wins), across the 16-bit half boundary and
// the 32-bit wrap.  Uses yosys' SB_MAC16 simulation model.   Run: make dsptest
`timescale 1ns/1ps
module tb_dsp_counter;
    reg clk = 0; always #10 clk = ~clk;
    reg        load = 0, en = 0;
    reg [31:0] load_val = 0;
    wire [31:0] q_dn, q_up;
    wire        nz, ones;
    dsp_counter #(.UP(0)) dn (.clk(clk), .load(load), .load_val(load_val), .en(en), .q(q_dn), .flag(nz));
    dsp_counter #(.UP(1)) up (.clk(clk), .load(load), .load_val(load_val), .en(en), .q(q_up), .flag(ones));

    reg [31:0] ref_dn = 0, ref_up = 0;   // the fabric counters being replaced
    always @(posedge clk) begin
        if (load)    begin ref_dn <= load_val; ref_up <= load_val; end
        else if (en) begin ref_dn <= ref_dn - 1; ref_up <= ref_up + 1; end
    end

    integer errors = 0, i, checks = 0;
    // values that exercise the 16-bit carry/borrow between the halves and the 32-bit wrap
    reg [31:0] seeds [0:9];
    initial begin
        seeds[0] = 32'd0;          seeds[1] = 32'd1;          seeds[2] = 32'd2;
        seeds[3] = 32'h0000FFFF;   seeds[4] = 32'h00010000;   seeds[5] = 32'h00010001;
        seeds[6] = 32'hFFFFFFFF;   seeds[7] = 32'hFFFFFFFE;   seeds[8] = 32'h7FFFFFFF;
        seeds[9] = 32'h00FFFFFF;
    end
    always @(negedge clk) if (i > 2) begin
        checks = checks + 1;
        if (q_dn !== ref_dn || nz !== (ref_dn != 0)) begin
            if (errors < 5) $display("FAIL down: q %h flag %b, want %h %b", q_dn, nz, ref_dn, ref_dn != 0);
            errors = errors + 1;
        end
        if (q_up !== ref_up || ones !== (ref_up == 32'hFFFFFFFF)) begin
            if (errors < 5) $display("FAIL up: q %h flag %b, want %h %b", q_up, ones, ref_up, ref_up == 32'hFFFFFFFF);
            errors = errors + 1;
        end
    end
    initial begin
        i = 0;
        @(negedge clk); load = 1; load_val = 32'd5; @(negedge clk); load = 0;
        for (i = 0; i < 4000; i = i + 1) begin
            @(negedge clk);
            load     = ($random % 23) == 0;
            load_val = seeds[($random & 32'h7fffffff) % 10];
            en       = ($random % 4) != 0;
        end
        if (errors == 0) $display("PASS tb_dsp_counter: SB_MAC16 up/down counter == fabric counter over %0d cycles (q + carry-out flag)", checks);
        else             $display("FAIL tb_dsp_counter: %0d mismatches", errors);
        $finish;
    end
endmodule
