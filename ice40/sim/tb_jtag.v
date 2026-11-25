// tb_jtag.v: swd_engine's JTAG master (v61, jtag_seq.v), unit level.
//
// A behavioural chain of two TAPs, like the NUCLEO-F446RE's: TDI -> boundary-scan TAP (IR 5,
// IDCODE 0x06413041) -> Cortex-M4 TAP (IR 4, IDCODE 0x4BA00477) -> TDO.  Each TAP is a full
// IEEE 1149.1 state machine (rising TCK), shifts on the rising edge and changes TDO on the falling
// edge; after Test-Logic-Reset its IR selects IDCODE, the all-ones IR BYPASS.  The bench feeds
// DAP_JTAG_Sequence sequences into SWD_FEED and checks the reply bytes, at half periods 2 (6 MHz),
// 3 and 13.  A monitor flags TDI or TMS changing while TCK is high.  Also checked: a 64-cycle
// sequence (count 0), a queue that runs dry mid-sequence (TCK held low), SPI_STATUS mode/busy, and
// that SWD bit-bang bytes are not decoded while JTAG is armed.
`timescale 1ns/1ps
module tb_jtag;
    reg clk = 0; always #20.833 clk = ~clk;   // 24 MHz
    reg rst = 1;

    reg        spi_arm_stb = 0, disarm_stb = 0, feed_begin = 0, feed_stb = 0;
    reg  [7:0] feed_byte = 0;
    reg  [6:0] half = 7'd2;
    reg  [3:0] flags = 4'b1000;
    reg  [8:0] rd_addr = 0;
    wire [7:0] rd_data;
    wire [15:0] reply_count;
    wire [7:0] q_done, q_flags;
    wire armed, spi_mode, spi_busy;
    wire [3:0] clk_ch, dio_ch, in_ch, cs_ch;
    wire clk_val, clk_oe, dio_val, dio_oe, cs_val, cs_on;

    wire tck = clk_val, tdi = dio_val, tms = cs_val;
    wire tdo;

    swd_engine #(.REPLY_AW(9)) dut (
        .clk(clk), .rst(rst),
        .arm_stb(1'b0), .arm_clk_ch(4'd0), .arm_dio_ch(4'd0),
        .spi_arm_stb(spi_arm_stb), .spi_sck_ch(4'd3), .spi_mosi_ch(4'd5), .spi_miso_ch(4'd6),
        .spi_cs_ch(4'd4), .spi_half(half), .spi_flags(flags), .spi_cs_stb(1'b0), .spi_cs_assert(1'b0),
        .disarm_stb(disarm_stb),
        .q_cfg_stb(1'b0), .q_cfg_half(6'd0), .q_cfg_idle(5'd0), .feed_q(1'b0),
        .feed_begin(feed_begin), .feed_stb(feed_stb), .feed_byte(feed_byte),
        .dio_in(tdo), .scl_in(1'b1),
        .rd_addr(rd_addr), .rd_data(rd_data), .reply_count(reply_count),
        .armed(armed), .spi_mode(spi_mode), .spi_busy(spi_busy), .q_done(q_done), .q_flags(q_flags),
        .clk_ch(clk_ch), .clk_val(clk_val), .clk_oe(clk_oe),
        .dio_ch(dio_ch), .dio_val(dio_val), .dio_oe(dio_oe),
        .in_ch(in_ch), .cs_ch(cs_ch), .cs_val(cs_val), .cs_on(cs_on)
    );

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin $display("FAIL: %0s", msg); errors = errors + 1; end endtask

    // ---- the chain: TDI -> bs (IR 5) -> cpu (IR 4) -> TDO ----
    wire bs_tdo;
    reg  trst_n = 1;
    tap_model #(.IRLEN(5), .IDCODE(32'h06413041), .IDCODE_IR(5'h01)) bs  (.tck(tck), .tms(tms), .tdi(tdi),    .trst_n(trst_n), .tdo(bs_tdo));
    tap_model #(.IRLEN(4), .IDCODE(32'h4BA00477), .IDCODE_IR(4'hE))  cpu (.tck(tck), .tms(tms), .tdi(bs_tdo), .trst_n(trst_n), .tdo(tdo));

    // TDI / TMS must only move while TCK is low
    reg tdi_q = 1, tms_q = 1;
    integer bad_edges = 0;
    always @(posedge clk) begin
        if (armed && flags[3] && tck && (tdi !== tdi_q || tms !== tms_q)) bad_edges = bad_edges + 1;
        tdi_q <= tdi; tms_q <= tms;
    end
    integer tck_rises = 0;
    always @(posedge tck) tck_rises = tck_rises + 1;

    // ---- feed helpers ----
    reg [7:0] fb [0:511];
    integer   fn;
    integer   gap = 0;                     // clk between feed bytes (a slow link)
    task feed_go;
        integer i, g;
        begin
            @(posedge clk); feed_begin <= 1; @(posedge clk); feed_begin <= 0;
            for (i = 0; i < fn; i = i + 1) begin
                @(posedge clk); feed_byte <= fb[i]; feed_stb <= 1;
                @(posedge clk); feed_stb <= 0;
                for (g = 0; g < gap; g = g + 1) @(posedge clk);
            end
            repeat (4) @(posedge clk);
            while (spi_busy) @(posedge clk);
            repeat (2) @(posedge clk);
        end
    endtask
    // one sequence: n cycles (1..64), TMS level, capture, TDI bits (LSB first)
    task seq(input integer n, input t, input c, input [63:0] bits);
        integer b;
        begin
            fb[fn] = {c, t, n[5:0]}; fn = fn + 1;
            for (b = 0; b < (n + 7) / 8; b = b + 1) begin fb[fn] = bits[b*8 +: 8]; fn = fn + 1; end
        end
    endtask
    task tms_seq(input integer n, input [63:0] bits);   // TMS pattern, one cycle each, TDI high
        integer b;
        for (b = 0; b < n; b = b + 1) seq(1, bits[b], 1'b0, 64'h1);
    endtask
    task rdb(input integer i, output [7:0] o);
        begin rd_addr = i; @(posedge clk); @(posedge clk); o = rd_data; end
    endtask
    task rdbk(input integer i, input integer k, output [7:0] o);   // a last byte of k bits
        begin rdb(i, o); o = o >> (8 - k); end
    endtask
    task reply64(input integer at, output [63:0] o);
        integer b; reg [7:0] t;
        begin o = 0; for (b = 0; b < 8; b = b + 1) begin rdb(at + b, t); o[b*8 +: 8] = t; end end
    endtask

    task arm(input [6:0] h, input [3:0] f);
        begin
            half = h; flags = f;
            @(posedge clk); spi_arm_stb <= 1; @(posedge clk); spi_arm_stb <= 0;
            repeat (2) @(posedge clk);
        end
    endtask

    // ---- one full pass at a half period ----
    reg [63:0] v;
    reg [7:0]  r;
    task pass(input [6:0] h);
        integer rises0;
        begin
            arm(h, 4'b1000);
            if (!spi_mode) fail("SPI_STATUS mode bit clear while JTAG is armed");
            if (!cs_on || !dio_oe || !clk_oe) fail("JTAG slots not driven");
            if (tck !== 1'b0 || tms !== 1'b1 || tdi !== 1'b1) fail("idle levels after arm");

            // 1. Test-Logic-Reset, Run-Test/Idle, Shift-DR; read both IDCODEs (63 bits + 1 on exit)
            fn = 0;
            tms_seq(5, 64'b11111);                   // TLR
            tms_seq(4, 64'b0010);                    // RTI, Select-DR, Capture-DR, Shift-DR
            seq(63, 1'b0, 1'b1, 64'h0);              // 63 bits in Shift-DR
            seq(1,  1'b1, 1'b1, 64'h0);              // the 64th, to Exit1-DR
            tms_seq(2, 64'b01);                      // Update-DR, RTI
            feed_go;
            if (reply_count != 9) begin fail("IDCODE scan: reply count"); $display("  got %0d", reply_count); end
            reply64(0, v);
            rdbk(7, 7, r); v[62:56] = r[6:0];
            rdbk(8, 1, r);
            v[63] = r[0];
            if (v[31:0] !== 32'h4BA00477) begin fail("cpu IDCODE"); $display("  got %h", v[31:0]); end
            if (v[63:32] !== 32'h06413041) begin fail("bs IDCODE"); $display("  got %h", v[63:32]); end

            // 2. both TAPs to BYPASS (9 IR bits of 1), then an 8-bit DR pattern comes out 2 bits late
            fn = 0;
            tms_seq(4, 64'b0011);                    // from RTI: Select-DR, Select-IR, Capture-IR, Shift-IR
            seq(8, 1'b0, 1'b1, 64'hFF);
            seq(1, 1'b1, 1'b1, 64'h1);               // the 9th IR bit, to Exit1-IR
            tms_seq(4, 64'b0011);                    // Update-IR, Select-DR, Capture-DR, Shift-DR
            seq(10, 1'b0, 1'b1, 64'h0A5);            // 0xA5 then 2 zero bits
            seq(1, 1'b1, 1'b0, 64'h0);
            tms_seq(2, 64'b01);
            feed_go;
            if (reply_count != 4) begin fail("bypass: reply count"); $display("  got %0d", reply_count); end
            rdb(0, r);
            // IR capture reads 01 in the low bits of each IR (1149.1): cpu IR first (bits 3:0 = xx01)
            if (r[1:0] !== 2'b01) begin fail("IR capture pattern"); $display("  got %b", r); end
            rdb(2, v[7:0]); rdbk(3, 2, v[15:8]);
            if (v[9:0] !== {8'hA5, 2'b00}) begin fail("bypass data"); $display("  got %b", v[9:0]); end

            // 3. a 64-cycle sequence (count 0) in TLR with TMS high, nothing captured
            fn = 0;
            rises0 = tck_rises;
            seq(64, 1'b1, 1'b0, 64'hFFFF_FFFF_FFFF_FFFF);
            feed_go;
            if (tck_rises - rises0 != 64) begin fail("64-cycle sequence"); $display("  rises %0d", tck_rises - rises0); end
            if (reply_count != 0) fail("non-capturing sequence wrote a reply");

            // 4. the same IDCODE scan with a slow link: the queue runs dry inside sequences
            gap = 3 * h * 8;
            fn = 0;
            tms_seq(5, 64'b11111);
            tms_seq(4, 64'b0010);
            seq(32, 1'b0, 1'b1, 64'h0);
            seq(31, 1'b0, 1'b1, 64'h0);
            seq(1,  1'b1, 1'b1, 64'h0);
            tms_seq(2, 64'b01);
            feed_go;
            gap = 0;
            reply64(0, v);
            rdbk(7, 7, r); v[62:56] = r[6:0];
            rdbk(8, 1, r);
            v[63] = r[0];
            if (v !== 64'h06413041_4BA00477) begin fail("slow-link IDCODE scan"); $display("  got %h", v); end
            if (tck !== 1'b0) fail("TCK not low at rest");
        end
    endtask

    initial begin
        repeat (4) @(posedge clk); rst <= 0; repeat (2) @(posedge clk);
        pass(7'd2);
        pass(7'd3);
        pass(7'd13);
        if (bad_edges != 0) begin fail("TDI/TMS moved while TCK was high"); $display("  %0d times", bad_edges); end

        // SWD bit-bang bytes ('f' = SWCLK high) are queue bytes in JTAG mode, never decoded:
        // a lone info byte with no TDI byte must not move TCK.
        fn = 1; fb[0] = "f";
        @(posedge clk); feed_begin <= 1; @(posedge clk); feed_begin <= 0;
        @(posedge clk); feed_byte <= "f"; feed_stb <= 1; @(posedge clk); feed_stb <= 0;
        repeat (40) @(posedge clk);
        if (tck !== 1'b0) fail("bit-bang byte decoded in JTAG mode");

        // re-arm as SPI mode 0: TMS slot becomes CS (high, released), TCK slot SCK
        arm(7'd2, 4'b0000);
        if (!cs_on || cs_val !== 1'b1) fail("SPI after JTAG: CS");
        disarm_stb <= 1; @(posedge clk); disarm_stb <= 0; @(posedge clk);
        if (armed || cs_on) fail("disarm");

        if (errors == 0) $display("PASS: tb_jtag");
        else             $display("FAILED: %0d errors", errors);
        $finish;
    end

    initial begin #20_000_000; $display("FAIL: timeout"); $finish; end
endmodule

// One IEEE 1149.1 TAP: IR of IRLEN bits (Capture-IR loads ...01), IDCODE and BYPASS DRs.
module tap_model #(parameter IRLEN = 4, parameter [31:0] IDCODE = 32'h1, parameter [IRLEN-1:0] IDCODE_IR = 1) (
    input  wire tck, input wire tms, input wire tdi, input wire trst_n,
    output reg  tdo
);
    localparam TLR = 4'd0, RTI = 4'd1, SDR = 4'd2, CDR = 4'd3, SHDR = 4'd4, E1DR = 4'd5, PDR = 4'd6,
               E2DR = 4'd7, UDR = 4'd8, SIR = 4'd9, CIR = 4'd10, SHIR = 4'd11, E1IR = 4'd12,
               PIR = 4'd13, E2IR = 4'd14, UIR = 4'd15;
    reg [3:0] st = TLR;
    reg [IRLEN-1:0] ir = IDCODE_IR, irsh = 0;
    reg [31:0] dr = 0;
    wire byp = (ir != IDCODE_IR);
    initial tdo = 1'b0;

    always @(posedge tck or negedge trst_n) begin
        if (!trst_n) begin st <= TLR; ir <= IDCODE_IR; end
        else begin
            // shift in the current state, then move on
            if (st == SHDR) dr   <= byp ? {31'd0, tdi} : {tdi, dr[31:1]};
            if (st == SHIR) irsh <= {tdi, irsh[IRLEN-1:1]};
            case (st)
                TLR:  st <= tms ? TLR  : RTI;
                RTI:  st <= tms ? SDR  : RTI;
                SDR:  st <= tms ? SIR  : CDR;
                CDR:  begin st <= tms ? E1DR : SHDR; dr <= byp ? 32'd0 : IDCODE; end
                SHDR: st <= tms ? E1DR : SHDR;
                E1DR: st <= tms ? UDR  : PDR;
                PDR:  st <= tms ? E2DR : PDR;
                E2DR: st <= tms ? UDR  : SHDR;
                UDR:  st <= tms ? SDR  : RTI;
                SIR:  st <= tms ? TLR  : CIR;
                CIR:  begin st <= tms ? E1IR : SHIR; irsh <= {{(IRLEN-1){1'b0}}, 1'b1}; end
                SHIR: st <= tms ? E1IR : SHIR;
                E1IR: st <= tms ? UIR  : PIR;
                PIR:  st <= tms ? E2IR : PIR;
                E2IR: st <= tms ? UIR  : SHIR;
                UIR:  begin st <= tms ? SDR : RTI; ir <= irsh; end
            endcase
            if (st == TLR && tms) ir <= IDCODE_IR;
        end
    end
    // TDO changes on the falling edge, from the register being shifted
    always @(negedge tck)
        tdo <= (st == SHDR) ? dr[0] : (st == SHIR) ? irsh[0] : 1'b0;
endmodule
