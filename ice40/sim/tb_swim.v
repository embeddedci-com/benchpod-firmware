// tb_swim.v: swd_engine's SWIM master (v62, swim_engine.v), unit level.
//
// A behavioural STM8 SWIM target on one open-drain wire (wired AND of the pod's pull-down, the
// chip's pull-down and a pull-up with a configurable rise time).  The chip model:
//   * watches for the entry sequence (a 16 us low, then 4 pulses at 1 kHz and 4 at 2 kHz, each
//     within 3 percent) and answers with a sync pulse of 128 of ITS clocks (clk_ns: two or three
//     SWIM clock rates are run),
//   * receives command frames (header 0, 3 bits, even parity) and data frames (header 0, 8 bits,
//     parity), decides each bit from the measured low time against 10 SWIM clocks (5 at high
//     speed), answers with ACK / NACK bits (a 1 / a 0),
//   * SRST, ROTF (N bytes from a 24-bit address sent to the pod, each ACKed by the pod, a NACK
//     makes it send the byte again) and WOTF (N bytes written to a 64 KB memory); a write of 0x10
//     to 0x007F80 (SWIM_CSR HS) switches the chip to high speed when the command is over.
// Knobs: inj_nack (NACK the next n frames), inj_par (send the next n data frames with bad parity),
// no_ack, no_sync, stuck (holds the line low), rise_ns, data_gap.
//
// The bench feeds queue tokens into SWD_FEED and checks the reply bytes, REPORT records and
// SWD_QSTATUS flags.  A monitor checks that the pod never drives the line high and never drives
// the clk / CS slots.
`timescale 1ns/1ps
module tb_swim;
    reg clk = 0; always #20.833 clk = ~clk;   // 24 MHz
    reg rst = 1;

    reg        spi_arm_stb = 0, disarm_stb = 0, feed_begin = 0, feed_stb = 0;
    reg  [7:0] feed_byte = 0;
    reg  [4:0] flags = 5'b10000;
    reg  [8:0] rd_addr = 0;
    wire [7:0] rd_data;
    wire [15:0] reply_count;
    wire [7:0] q_done, q_flags;
    wire armed, spi_mode, spi_busy;
    wire [3:0] clk_ch, dio_ch, in_ch, cs_ch;
    wire clk_val, clk_oe, dio_val, dio_oe, cs_val, cs_on;
    wire line;

    swd_engine #(.REPLY_AW(9)) dut (
        .clk(clk), .rst(rst),
        .arm_stb(1'b0), .arm_clk_ch(4'd0), .arm_dio_ch(4'd0),
        .spi_arm_stb(spi_arm_stb), .spi_sck_ch(4'd5), .spi_mosi_ch(4'd5), .spi_miso_ch(4'd5),
        .spi_cs_ch(4'd5), .spi_half(7'd2), .spi_flags(flags), .spi_cs_stb(1'b0), .spi_cs_assert(1'b0),
        .disarm_stb(disarm_stb),
        .q_cfg_stb(1'b0), .q_cfg_half(6'd0), .q_cfg_idle(5'd0), .feed_q(1'b0),
        .feed_begin(feed_begin), .feed_stb(feed_stb), .feed_byte(feed_byte),
        .dio_in(line), .scl_in(1'b1),
        .rd_addr(rd_addr), .rd_data(rd_data), .reply_count(reply_count),
        .armed(armed), .spi_mode(spi_mode), .spi_busy(spi_busy), .q_done(q_done), .q_flags(q_flags),
        .clk_ch(clk_ch), .clk_val(clk_val), .clk_oe(clk_oe),
        .dio_ch(dio_ch), .dio_val(dio_val), .dio_oe(dio_oe),
        .in_ch(in_ch), .cs_ch(cs_ch), .cs_val(cs_val), .cs_on(cs_on)
    );

    integer errors = 0;
    task fail(input [8*80-1:0] msg); begin $display("FAIL: %0s", msg); errors = errors + 1; end endtask
    task check(input integer got, input integer exp, input [8*60-1:0] msg);
        if (got !== exp) begin $display("FAIL: %0s: got %0d (0x%0h), want %0d (0x%0h)", msg, got, got, exp, exp); errors = errors + 1; end
    endtask

    // ================== the wire ==================
    real    clk_ns  = 125.0;                 // the chip's SWIM clock period
    real    rise_ns = 20.0;                  // pull-up rise time
    reg     tgt_low = 0, stuck = 0;
    wire    host_low = (dio_oe === 1'b1) && (dio_val === 1'b0);
    wire    pulled   = host_low | tgt_low | stuck;
    reg     line_r = 1;
    assign  line = line_r;
    always @(posedge pulled) line_r = 0;
    always @(negedge pulled) begin #(rise_ns); if (!pulled) line_r = 1; end

    // the pod: open drain only, the other slots idle
    integer bad_drive = 0;
    always @(posedge clk) if (armed && flags[4] && ((dio_oe === 1'b1 && dio_val !== 1'b0) || clk_oe || cs_on)) bad_drive = bad_drive + 1;

    // ================== the chip ==================
    reg        no_sync = 0, no_ack = 0, hs = 0, hs_next = 0;
    integer    inj_nack = 0, inj_par = 0, data_gap = 40;
    integer    srst_count = 0, bad_entries = 0, host_nacks = 0, nacks_sent = 0, entries = 0;
    reg [7:0]  mem [0:65535];

    task slot(input b, input last);          // one device-to-host bit; `last`: return after the low part
        real lo, per;
        begin
            lo  = (b ? 2.0 : (hs ? 8.0 : 20.0)) * clk_ns;
            per = (hs ? 10.0 : 22.0) * clk_ns;
            tgt_low = 1; #(lo); tgt_low = 0;
            if (!last) #(per - lo);
        end
    endtask
    task rbit(output b);                     // one host-to-device bit, by its measured low time
        real t0, d;
        begin
            @(negedge line); t0 = $realtime; @(posedge line);
            d = ($realtime - t0) / clk_ns;
            b = (d < (hs ? 5.0 : 10.0));
        end
    endtask
    task rx_frame(input integer n, output [9:0] v);
        integer i; reg b;
        begin v = 0; for (i = 0; i < n; i = i + 1) begin rbit(b); v = {v[8:0], b}; end end
    endtask
    task respond(input good, output acked);  // the ACK / NACK for a host frame
        begin
            acked = 0;
            #(4.0 * clk_ns);
            if (no_ack) ;
            else if (inj_nack > 0) begin inj_nack = inj_nack - 1; slot(1'b0, 1'b1); nacks_sent = nacks_sent + 1; end
            else if (!good) begin slot(1'b0, 1'b1); nacks_sent = nacks_sent + 1; end
            else begin slot(1'b1, 1'b1); acked = 1; end
        end
    endtask
    task get_cmd(output [2:0] c);
        reg [9:0] v; reg ok;
        begin
            ok = 0;
            while (!ok) begin rx_frame(5, v); respond(!v[4] && !(^v[3:0]), ok); end
            c = v[3:1];
        end
    endtask
    task get_byte(output [7:0] d);
        reg [9:0] v; reg ok;
        begin
            ok = 0;
            while (!ok) begin rx_frame(10, v); respond(!v[9] && !(^v[8:0]), ok); end
            d = v[8:1];
        end
    endtask
    task send_byte(input [7:0] d);
        reg [9:0] f; reg a, done; integer i;
        begin
            done = 0;
            while (!done) begin
                f = {1'b1, d, ^d};
                if (inj_par > 0) begin f[0] = ~f[0]; inj_par = inj_par - 1; end
                for (i = 9; i >= 0; i = i - 1) slot(f[i], i == 0);
                rbit(a);
                if (a) done = 1; else begin host_nacks = host_nacks + 1; #(data_gap * clk_ns); end
            end
        end
    endtask

    reg [2:0]  c;
    reg [7:0]  n, a2, a1, a0, d;
    reg [23:0] addr;
    integer    k, cnt;
    reg        ok;
    real       t0, th0, tl0, hold, th, tl, ex;
    reg skip_entry = 0;
    always begin : tgt_main
        // ---- entry: 16 us low, 4 pulses at 1 kHz, 4 at 2 kHz, then sync ----
        ok = skip_entry;
        while (!ok) begin
            @(negedge line); t0 = $realtime; @(posedge line); hold = $realtime - t0;
            if (hold > 12000.0 && hold < 21000.0) begin
                ok = 1;
                for (k = 0; k < 8; k = k + 1) if (ok) begin
                    th0 = $realtime; @(negedge line); th = $realtime - th0;
                    tl0 = $realtime; @(posedge line); tl = $realtime - tl0;
                    ex = (k < 4) ? 500000.0 : 250000.0;
                    if (th < ex * 0.97 || th > ex * 1.03 || tl < ex * 0.97 || tl > ex * 1.03) begin
                        ok = 0; bad_entries = bad_entries + 1;
                    end
                end
            end
        end
        if (!skip_entry) begin
            entries = entries + 1;
            #30000.0;
            if (!no_sync) begin tgt_low = 1; #(128.0 * clk_ns); tgt_low = 0; end
        end
        skip_entry = 0;
        // ---- SWIM mode ----
        forever begin
            get_cmd(c);
            case (c)
            3'd0: srst_count = srst_count + 1;
            3'd1, 3'd2: begin
                get_byte(n); get_byte(a2); get_byte(a1); get_byte(a0);
                addr = {a2, a1, a0}; cnt = (n == 0) ? 256 : n; hs_next = hs;
                if (c == 3'd1) begin
                    for (k = 0; k < cnt; k = k + 1) begin #(data_gap * clk_ns); send_byte(mem[(addr + k) & 24'hFFFF]); end
                end else begin
                    for (k = 0; k < cnt; k = k + 1) begin
                        get_byte(d); mem[(addr + k) & 24'hFFFF] = d;
                        if (((addr + k) & 24'hFFFFFF) == 24'h007F80 && d[4]) hs_next = 1;
                    end
                end
                hs = hs_next;
            end
            default: ;
            endcase
        end
    end
    task tgt_restart;                        // the chip is reset: it waits for an entry sequence again
        begin skip_entry = 0; disable tgt_main; tgt_low = 0; hs = 0; end
    endtask
    task tgt_resync;                         // the chip stays in SWIM mode, but forgets a half received frame
        begin skip_entry = 1; disable tgt_main; tgt_low = 0; end
    endtask

    // ================== feed helpers ==================
    reg [7:0] fb [0:511];
    integer   fn;
    integer   gap = 0;                     // clk between feed bytes (a slow link)
    task tk(input [7:0] b); begin fb[fn] = b; fn = fn + 1; end endtask
    task t_entry;  tk(8'h01);                    endtask
    task t_cmd(input [2:0] cmd); tk({2'b00, cmd, 3'd2}); endtask
    task t_txb(input [7:0] b); begin tk(8'h03); tk(b); end endtask
    task t_rxb;    tk(8'h04);                    endtask
    task t_cfg(input hsp, input [3:0] retr, input [7:0] tmo, input [7:0] rise);
        begin tk(8'h05); tk({hsp, 3'b000, retr}); tk(tmo); tk(rise); end
    endtask
    task t_report; tk(8'h06);                    endtask
    task t_rotf(input [7:0] nn, input [23:0] ad); begin tk(8'h07); tk(nn); tk(ad[23:16]); tk(ad[15:8]); tk(ad[7:0]); end endtask
    task t_wotf(input [7:0] nn, input [23:0] ad); begin tk(8'h0F); tk(nn); tk(ad[23:16]); tk(ad[15:8]); tk(ad[7:0]); end endtask

    task feed_go;
        integer i, g, w;
        begin
            @(posedge clk); feed_begin <= 1; @(posedge clk); feed_begin <= 0;
            for (i = 0; i < fn; i = i + 1) begin
                @(posedge clk); feed_byte <= fb[i]; feed_stb <= 1;
                @(posedge clk); feed_stb <= 0;
                for (g = 0; g < gap; g = g + 1) @(posedge clk);
            end
            repeat (4) @(posedge clk);
            w = 0;
            while (spi_busy && w < 3000000) begin @(posedge clk); w = w + 1; end
            if (w >= 3000000) fail("queue did not finish");
            repeat (4) @(posedge clk);
        end
    endtask
    task rdb(input integer i, output [7:0] o);
        begin rd_addr = i; @(posedge clk); @(posedge clk); o = rd_data; end
    endtask
    function [7:0] rb(input integer i); begin rb = 0; end endfunction

    task rearm; begin tgt_resync; arm; end endtask
    task arm;
        begin
            @(posedge clk); spi_arm_stb <= 1; @(posedge clk); spi_arm_stb <= 0;
            repeat (3) @(posedge clk);
            fn = 0;
        end
    endtask

    reg [7:0] r, r1, r2, r3;
    integer   lv;
    // the REPORT record at reply index i: ack, nack, flags
    task rep_at(input integer i, input integer ack, input integer nack, input integer flg, input [8*40-1:0] msg);
        begin
            rdb(i, r1); rdb(i + 1, r2); rdb(i + 2, r3);
            if (r1 !== ack[7:0] || r2 !== nack[7:0] || r3 !== flg[7:0]) begin
                $display("FAIL: %0s: report ack %0d nack %0d flags %0h, want %0d %0d %0h", msg, r1, r2, r3, ack, nack, flg);
                errors = errors + 1;
            end
        end
    endtask
    task sync_at(input integer i, input integer lo, input integer hi, input [8*40-1:0] msg);   // L within lo..hi
        begin
            rdb(i, r1); rdb(i + 1, r2);
            lv = {r2[3:0], r1};
            if (lv < lo || lv > hi) begin $display("FAIL: %0s: sync length %0d, want %0d..%0d", msg, lv, lo, hi); errors = errors + 1; end
        end
    endtask
    task data_at(input integer i, input [23:0] ad, input integer nn, input [8*40-1:0] msg);   // reply bytes = chip memory
        integer j;
        begin
            for (j = 0; j < nn; j = j + 1) begin
                rdb(i + j, r);
                if (r !== mem[(ad + j) & 24'hFFFF]) begin $display("FAIL: %0s: byte %0d got %0h want %0h", msg, j, r, mem[(ad + j) & 24'hFFFF]); errors = errors + 1; end
            end
        end
    endtask
    task mem_is(input [23:0] ad, input [7:0] v0, input [7:0] v1, input [7:0] v2, input [7:0] v3, input [8*40-1:0] msg);
        begin
            if (mem[ad[15:0]] !== v0 || mem[ad[15:0] + 1] !== v1 || mem[ad[15:0] + 2] !== v2 || mem[ad[15:0] + 3] !== v3) begin
                $display("FAIL: %0s: chip memory %h %h %h %h", msg, mem[ad[15:0]], mem[ad[15:0] + 1], mem[ad[15:0] + 2], mem[ad[15:0] + 3]);
                errors = errors + 1;
            end
        end
    endtask

    integer i;
    initial begin
        for (i = 0; i < 65536; i = i + 1) mem[i] = (i * 7 + 3) & 8'hFF;
        #200 rst = 0;
        repeat (10) @(posedge clk);

        // ---- 1. entry at 8 MHz, SRST, ROTF, WOTF: the plain round trips ----
        clk_ns = 125.0; tgt_restart; arm;
        t_entry; t_cmd(3'd0); t_report;
        feed_go;
        sync_at(0, 381, 388, "sync at 8 MHz");
        rep_at(2, 1, 0, 8'h00, "after SRST");
        check(reply_count, 5, "reply count entry+SRST+report");
        check(srst_count, 1, "SRST seen by the chip");
        check(entries, 1, "entry sequences seen");
        check(bad_entries, 0, "malformed entry pulses");
        check(q_flags, 8'h00, "q_flags idle after entry");
        if (!spi_mode) fail("SPI_STATUS mode bit clear while SWIM is armed");

        fn = 0; t_rotf(8'd5, 24'h001234); t_report; feed_go;
        data_at(0, 24'h001234, 5, "ROTF 5 bytes");
        rep_at(5, 11, 0, 8'h00, "after ROTF");
        check(reply_count, 8, "reply count ROTF");

        fn = 0; t_wotf(8'd4, 24'h002000); tk(8'hAA); tk(8'h55); tk(8'h01); tk(8'hFE); t_report; feed_go;
        mem_is(24'h002000, 8'hAA, 8'h55, 8'h01, 8'hFE, "WOTF 4 bytes");
        rep_at(0, 20, 0, 8'h00, "after WOTF");
        check(q_flags, 8'h00, "q_flags idle after WOTF");

        // N = 0 means 256, one byte back to back with the chip's gap at its minimum, a slow feed link
        fn = 0; t_rotf(8'd2, 24'h00F000); t_report; data_gap = 22; gap = 2000; feed_go; gap = 0;
        data_at(0, 24'h00F000, 2, "ROTF, chip answering back to back, slow feed");
        rep_at(2, 27, 0, 8'h00, "after slow ROTF");
        data_gap = 40;

        // ---- 2. raw frames: TXB / RXB, a CMD the chip does not know is still ACKed ----
        fn = 0; t_cmd(3'd1); t_txb(8'd1); t_txb(8'h00); t_txb(8'h30); t_txb(8'h00); t_rxb; t_report; feed_go;
        data_at(0, 24'h003000, 1, "raw ROTF with RXB");
        rep_at(1, 33, 0, 8'h00, "after raw ROTF");

        // ---- 3. NACK and retry ----
        fn = 0; inj_nack = 2; t_wotf(8'd1, 24'h002100); tk(8'h5A); t_report; feed_go;
        mem_is(24'h002100, 8'h5A, mem[16'h2101], mem[16'h2102], mem[16'h2103], "WOTF after two NACKs");
        rep_at(0, 39, 2, 8'h00, "two NACKs retried");
        check(q_flags, 8'h08, "q_flags after retries (2 NACKs counted)");

        // a bad parity frame from the chip: NACK from the pod, the chip sends the byte again
        fn = 0; inj_par = 1; t_rotf(8'd2, 24'h004000); t_report; feed_go;
        data_at(0, 24'h004000, 2, "ROTF with a bad-parity frame");
        rep_at(2, 46, 3, 8'h20, "parity NACK");
        check(host_nacks, 1, "chip saw the pod's NACK");

        // retries used up: stops, the failure appends the report, the rest of the queue is not run
        fn = 0; inj_nack = 10; t_wotf(8'd1, 24'h002200); tk(8'h77); t_report; t_cmd(3'd0); feed_go;
        check(q_flags & 8'hFE, 8'h3E, "q_flags NACK retries used up");   // nack-failed, NACK count 7, stopped
        check(reply_count, 3, "failure reply is the report");
        rep_at(0, 46, 7, 8'hB0, "NACK retries used up");
        check(srst_count, 1, "the SRST behind the failure was not run");
        inj_nack = 0;
        // a retry count of 1 from CFG: two NACKs are too many
        rearm; t_cfg(1'b0, 4'd1, 8'd32, 8'd64); inj_nack = 2; t_cmd(3'd0); feed_go;
        check(q_flags & 8'h22, 8'h22, "CFG retries 1: stopped, NACK failure");
        inj_nack = 0;

        // ---- 4. the chip never answers ----
        rearm; no_ack = 1; t_cmd(3'd0); t_report; feed_go;
        check(q_flags & 8'hC3, 8'h82, "no ACK: timeout and stopped");
        rep_at(0, 0, 0, 8'h81, "no ACK: report");   // stopped, flags bit 0
        no_ack = 0;
        // the same with a short timeout (CFG): 2 x 16 ticks
        rearm; no_ack = 1; t_cfg(1'b0, 4'd3, 8'd2, 8'd64); t_rxb; feed_go;
        check(q_flags & 8'h83, 8'h82, "no frame: timeout and stopped");
        no_ack = 0;

        // ---- 5. a line held low (a shorted line, or a chip that is stuck) ----
        rearm; stuck = 1; t_cmd(3'd0); feed_go;
        check(q_flags & 8'h83, 8'h82, "line stuck low: timeout and stopped");
        rep_at(0, 0, 0, 8'h88, "stuck low: flags bit 3");
        stuck = 0;

        // ---- 6. slow rising line: the engine waits for it, the bits stay intact ----
        // 700 ns is 5.6 SWIM clocks: the pod's high part after a 0 bit (2 clocks) would be gone before the line
        // rose.  (The chip's own bits need a faster line, see 200 ns below: its high part is only 2 clocks.)
        rearm; rise_ns = 700.0; t_wotf(8'd3, 24'h002300); tk(8'h11); tk(8'h22); tk(8'h33); t_cmd(3'd0); t_report; feed_go;
        mem_is(24'h002300, 8'h11, 8'h22, 8'h33, mem[16'h2303], "slow rise WOTF");
        rep_at(0, 9, 0, 8'h00, "slow rise 700 ns");
        rearm; rise_ns = 200.0; t_rotf(8'd3, 24'h002300); t_report; feed_go;
        data_at(0, 24'h002300, 3, "ROTF on a 200 ns line");
        rep_at(3, 8, 0, 8'h00, "slow rise 200 ns");
        // without the wait (CFG rise = 0) the 700 ns line corrupts the frame: no ACK, the queue stops
        rearm; rise_ns = 700.0; t_cfg(1'b0, 4'd0, 8'd8, 8'd0); t_cmd(3'd0); feed_go;
        if (q_flags[1] !== 1'b1) fail("slow rise without the wait: expected the queue to stop");
        rise_ns = 20.0;

        // ---- 7. high speed: SWIM_CSR HS, then the pod follows ----
        rearm; t_wotf(8'd1, 24'h007F80); tk(8'h10); feed_go;
        if (hs !== 1'b1) fail("chip is not in high speed");
        fn = 0; t_cfg(1'b1, 4'd3, 8'd32, 8'd64); t_rotf(8'd6, 24'h001000); t_wotf(8'd2, 24'h002400); tk(8'hC3); tk(8'h3C); t_report; feed_go;
        data_at(0, 24'h001000, 6, "high speed ROTF");
        mem_is(24'h002400, 8'hC3, 8'h3C, mem[16'h2402], mem[16'h2403], "high speed WOTF");
        rep_at(6, 24, 0, 8'h40, "high speed report");
        hs = 0;

        // ---- 8. other SWIM clock rates: 6.4 MHz (L 480) and 9 MHz (L 341) ----
        clk_ns = 156.25; tgt_restart; arm; t_entry; t_rotf(8'd3, 24'h005000); t_report; feed_go;
        sync_at(0, 477, 484, "sync at 6.4 MHz");
        data_at(2, 24'h005000, 3, "ROTF at 6.4 MHz");
        rep_at(5, 8, 0, 8'h00, "6.4 MHz report");
        fn = 0; t_wotf(8'd2, 24'h005100); tk(8'h99); tk(8'h66); feed_go;
        mem_is(24'h005100, 8'h99, 8'h66, mem[16'h5102], mem[16'h5103], "WOTF at 6.4 MHz");
        clk_ns = 111.11; tgt_restart; arm; t_entry; t_rotf(8'd3, 24'h005200); t_wotf(8'd1, 24'h005300); tk(8'hA5); feed_go;
        sync_at(0, 338, 344, "sync at 9 MHz");
        data_at(2, 24'h005200, 3, "ROTF at 9 MHz");
        mem_is(24'h005300, 8'hA5, mem[16'h5301], mem[16'h5302], mem[16'h5303], "WOTF at 9 MHz");
        check(bad_entries, 0, "malformed entry pulses over the run");

        // ---- 9. entry failures: no sync pulse, a sync pulse out of range ----
        clk_ns = 125.0; no_sync = 1; tgt_restart; arm; t_entry; t_cmd(3'd0); feed_go;
        sync_at(0, 0, 0, "no sync pulse: L = 0");
        check(reply_count, 2, "no sync pulse: 2 reply bytes");
        check(q_flags & 8'h83, 8'h82, "no sync pulse: timeout and stopped");
        no_sync = 0;
        clk_ns = 40.0; tgt_restart; arm; t_entry; feed_go;               // 128 x 40 ns = 5.1 us = 123 clk
        sync_at(0, 120, 126, "short sync reported");
        check(q_flags & 8'h83, 8'h82, "short sync: stopped");
        clk_ns = 125.0;

        check(bad_drive, 0, "the pod drove the open-drain line high or drove another slot");
        if (errors == 0) $display("PASS: tb_swim");
        else $display("FAILED: %0d errors", errors);
        $finish;
    end
    initial begin #800_000_000; $display("FAIL: timeout"); $finish; end
endmodule
