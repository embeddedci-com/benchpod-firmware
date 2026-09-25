// tb_dsp_counter.v — dsp_counter (an SB_MAC16 accumulator) must behave exactly like the
// fabric counter it replaces, cycle for cycle: q and the free carry-out flag, counting up and
// down, with load and enable driven randomly (load wins), across the 16-bit half boundary and
// the 32-bit wrap.  dsp_counter2 (two 16-bit counters in one DSP) likewise, both halves in both
// directions, with each half's load and enable independent of the other's, so a carry leaking
// between the halves shows up.  Uses yosys' SB_MAC16 simulation model.   Run: make dsptest
`timescale 1ns/1ps
module tb_dsp_counter;
    reg clk = 0; always #10 clk = ~clk;
    reg        load = 0, en = 0;
    reg [31:0] load_val = 0;
    wire [31:0] q_dn, q_up;
    wire        nz, ones;
    dsp_counter #(.UP(0)) dn (.clk(clk), .load(load), .load_val(load_val), .en(en), .q(q_dn), .flag(nz));
    dsp_counter #(.UP(1)) up (.clk(clk), .load(load), .load_val(load_val), .en(en), .q(q_up), .flag(ones));

    // dsp_counter2: top down + bottom up, and top up + bottom down, halves driven independently
    reg        lt = 0, et = 0, lb = 0, eb = 0;
    reg [15:0] vt = 0, vb = 0;
    wire [15:0] t_dn, b_up, t_up, b_dn;
    wire        t_nz, t_ones;
    dsp_counter2 #(.UP_T(0), .UP_B(1)) d2a (.clk(clk), .load_t(lt), .val_t(vt), .en_t(et), .qt(t_dn),
        .flag(t_nz), .load_b(lb), .val_b(vb), .en_b(eb), .qb(b_up));
    dsp_counter2 #(.UP_T(1), .UP_B(0)) d2b (.clk(clk), .load_t(lt), .val_t(vt), .en_t(et), .qt(t_up),
        .flag(t_ones), .load_b(lb), .val_b(vb), .en_b(eb), .qb(b_dn));
    reg [15:0] r_tdn = 0, r_bup = 0, r_tup = 0, r_bdn = 0;
    always @(posedge clk) begin
        if (lt)      begin r_tdn <= vt; r_tup <= vt; end
        else if (et) begin r_tdn <= r_tdn - 1; r_tup <= r_tup + 1; end
        if (lb)      begin r_bup <= vb; r_bdn <= vb; end
        else if (eb) begin r_bup <= r_bup + 1; r_bdn <= r_bdn - 1; end
    end

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
        if (t_dn !== r_tdn || t_nz !== (r_tdn != 0) || b_up !== r_bup ||
            t_up !== r_tup || t_ones !== (r_tup == 16'hFFFF) || b_dn !== r_bdn) begin
            if (errors < 5) $display("FAIL dual: top dn %h/%b up %h/%b bot up %h dn %h, want %h/%b %h/%b %h %h",
                t_dn, t_nz, t_up, t_ones, b_up, b_dn, r_tdn, r_tdn != 0, r_tup, r_tup == 16'hFFFF, r_bup, r_bdn);
            errors = errors + 1;
        end
    end
    reg [15:0] seeds16 [0:5];
    initial begin
        seeds16[0] = 16'd0; seeds16[1] = 16'd1; seeds16[2] = 16'd2;
        seeds16[3] = 16'hFFFF; seeds16[4] = 16'hFFFE; seeds16[5] = 16'h8000;
    end
    initial begin
        i = 0;
        @(negedge clk); load = 1; load_val = 32'd5; lt = 1; lb = 1; @(negedge clk); load = 0; lt = 0; lb = 0;
        for (i = 0; i < 4000; i = i + 1) begin
            @(negedge clk);
            load     = ($random % 23) == 0;
            load_val = seeds[($random & 32'h7fffffff) % 10];
            en       = ($random % 4) != 0;
            lt = ($random % 19) == 0; vt = seeds16[($random & 32'h7fffffff) % 6]; et = ($random % 3) != 0;
            lb = ($random % 17) == 0; vb = seeds16[($random & 32'h7fffffff) % 6]; eb = ($random % 3) != 0;
        end
        if (errors == 0) $display("PASS tb_dsp_counter: SB_MAC16 up/down counters (32-bit and dual 16-bit) == fabric counters over %0d cycles (q + carry-out flag)", checks);
        else             $display("FAIL tb_dsp_counter: %0d mismatches", errors);
        $finish;
    end
endmodule
