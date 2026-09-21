// ============================================================================
// tb_top_capture.v — top-level integration test of the UNIFIED capture datapath.
//
// The per-module benches (dualwrtest) feed psram_dual_writer directly, and
// topsmoketest only checks PING/VERSION/STATUS — so nothing exercised the real
// end-to-end path: SPI OP_CAPTURE -> cmd_dispatch -> arm ADC + LA producers ->
// spram_ring16 x2 -> psram_dual_writer -> the two PSRAM regions.  This bench
// closes that gap: it enables the ADC ramp self-test (so the ADC bytes are a
// known ramp with no analog model), drives the LA bank to a known 12-bit word,
// arms OP_CAPTURE over SPI, and acts as the PSRAM write-slave — decoding the QPI
// 0x38 writes and checking that the ADC stream landed in the ADC region and the
// LA stream in the LA region.  This is the exact scenario `cap-selftest` runs on
// the bench.
//
// Run:  make topcapturetest
// ============================================================================
`timescale 1ns/1ps
module tb_top_capture;
    // PSRAM region bases — keep in sync with tools/gen_protocol.py (sys_config.vh).
    localparam [23:0] LA_BASE  = 24'h000000;   // v18: LA at the FRONT (was 0x400000)
    localparam [23:0] ADC_BASE = 24'h400000;   // v18: ADC at 4 MB (was 0x000000)

    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;

    reg  sck = 1'b0, mosi = 1'b0, csn = 1'b1;
    reg  bus_own = 1'b1;            // start owned; released for the capture
    reg  adc_sdo = 1'b1;
    wire miso, busy;
    wire adc_cnvst, adc_sclk, adc_sdi, dac_sync, dac_sclk, dac_din;
    wire led_g, led_b, led_r;
    wire psram_sclk, psram_cs, psram_io0, psram_io1, psram_io2, psram_io3;

    // Drive the LA bank to a known 12-bit word (LA1..LA12 = 0xA5A).  A reg so the v35 trigger
    // passes can move pins; la_bank never drives in this bench.
    localparam [11:0] LA_WORD = 12'hA5A;
    reg  [11:0] la_drv = LA_WORD;
    wire [13:0] la;
    assign la = {2'b00, la_drv};

    top dut (
        .clk48(clk48),
        .sck(sck), .mosi(mosi), .miso(miso), .csn(csn), .busy(busy),
        .bus_own(bus_own),
        .adc_cnvst(adc_cnvst), .adc_sclk(adc_sclk), .adc_sdi(adc_sdi), .adc_sdo(adc_sdo),
        .dac_sync(dac_sync), .dac_sclk(dac_sclk), .dac_din(dac_din),
        .psram_sclk(psram_sclk), .psram_cs(psram_cs),
        .psram_io0(psram_io0), .psram_io1(psram_io1),
        .psram_io2(psram_io2), .psram_io3(psram_io3),
        .la(la),
        .led_g(led_g), .led_b(led_b), .led_r(led_r)
    );

    localparam SPI_HALF = 200;
    integer errors = 0;

    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b]; #(SPI_HALF);
                sck = 1'b1; rx[b] = miso; #(SPI_HALF);
                sck = 1'b0;
            end
        end
    endtask

    // opcode + one reply byte
    task cmd1(input [7:0] op, output [7:0] reply);
        reg [7:0] junk;
        begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(op, junk);
            spi_byte(8'h00, reply);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // opcode + N argument bytes, no reply
    task cmd_args1(input [7:0] op, input [7:0] a0);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(op, junk); spi_byte(a0, junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // v16: OP_CAPTURE is a 10-byte payload — adc_cnt(3) + adc_div(2) + la_cnt(3) +
    // la_div(2) — both counts full 24-bit for deep ADC + deep LA.
    task cmd_capture(input [23:0] adc_cnt, input [15:0] adc_div,
                     input [23:0] la_cnt,  input [15:0] la_div);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h31, junk);                 // OP_CAPTURE
            spi_byte(adc_cnt[7:0], junk); spi_byte(adc_cnt[15:8], junk); spi_byte(adc_cnt[23:16], junk);
            spi_byte(adc_div[7:0], junk); spi_byte(adc_div[15:8], junk);
            spi_byte(la_cnt[7:0],  junk); spi_byte(la_cnt[15:8],  junk); spi_byte(la_cnt[23:16], junk);
            spi_byte(la_div[7:0],  junk); spi_byte(la_div[15:8],  junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // OP_START_DAC (0x11): period(2) + divider(2) — shallow BRAM replay arm.
    task cmd_start_dac(input [15:0] period, input [15:0] divider);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h11, junk);
            spi_byte(period[7:0],  junk); spi_byte(period[15:8],  junk);
            spi_byte(divider[7:0], junk); spi_byte(divider[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // OP_START_DAC_PSRAM (0x13): base(3) + count(3) + divider(2) — deep PSRAM replay arm.
    task cmd_start_dac_psram(input [23:0] base, input [23:0] count, input [15:0] divider);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h13, junk);
            spi_byte(base[7:0],   junk); spi_byte(base[15:8],  junk); spi_byte(base[23:16],  junk);
            spi_byte(count[7:0],  junk); spi_byte(count[15:8], junk); spi_byte(count[23:16], junk);
            spi_byte(divider[7:0],junk); spi_byte(divider[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // A 0-argument instant opcode (OP_STOP_DAC 0x12, OP_DAC_ARM_ON_CAPTURE 0x19).
    task cmd_op0(input [7:0] op);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(op, junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // OP_LOAD_WAVE (0x10): len(2) + bytes into the waveform BRAM from address 0.  Loads n 16-bit
    // samples (LE byte pairs, as the firmware does) with the value base + index, so every frame
    // the DAC plays names the sample it came from.
    task cmd_load_wave16(input integer n, input [15:0] base);
        reg [7:0] junk; reg [15:0] v, len; integer k; begin
            len = 2 * n;
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h10, junk);
            spi_byte(len[7:0], junk); spi_byte(len[15:8], junk);
            for (k = 0; k < n; k = k + 1) begin
                v = base + k;
                spi_byte(v[7:0], junk); spi_byte(v[15:8], junk);
            end
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // ---- DAC8551 pin decoder: DIN is taken on SCLK falling edges while SYNC is low, and the
    //      24th edge completes a {ctrl(8), data(16)} frame.  Frames are recorded while dac_rec.
    //      The engine raises SYNC on the SAME clk48 edge as the 24th SCLK fall, so "SYNC low" is
    //      sampled at the SCLK rise half a bit earlier (DIN only changes on rises). ----
    localparam DAC_PA = 40, DAC_PB = 4, DAC_NF = 3*DAC_PB + 2;
    integer    dac_bits = 0, dac_nfr = 0, dac_rec = 0, dac_in_frame = 0;
    reg [23:0] dac_sh = 24'd0;
    reg [15:0] dac_fr [0:63];
    always @(negedge dac_sync) dac_bits = 0;
    always @(posedge dac_sclk) dac_in_frame = (dac_sync === 1'b0);
    always @(negedge dac_sclk) if (dac_in_frame) begin
        dac_sh   = {dac_sh[22:0], dac_din};
        dac_bits = dac_bits + 1;
        if (dac_bits == 24 && dac_rec) begin
            if (dac_nfr < 64) dac_fr[dac_nfr] = dac_sh[15:0];
            dac_nfr = dac_nfr + 1;
        end
    end

    // OP_START_MEASURE (0x30): count(2) [= DAC period AND ADC count] + dac_div(2) + cap_div(2).
    task cmd_measure(input [15:0] count, input [15:0] dac_div, input [15:0] cap_div);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h30, junk);
            spi_byte(count[7:0],   junk); spi_byte(count[15:8],   junk);
            spi_byte(dac_div[7:0], junk); spi_byte(dac_div[15:8], junk);
            spi_byte(cap_div[7:0], junk); spi_byte(cap_div[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_DAC_LOOP_SRC (0x1A): src(1) + fixed(2) + step(2).
    task cmd_loop_src(input [7:0] src, input [15:0] fixed, input [15:0] step);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h1A, junk); spi_byte(src, junk);
            spi_byte(fixed[7:0], junk); spi_byte(fixed[15:8], junk);
            spi_byte(step[7:0],  junk); spi_byte(step[15:8],  junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_START_DAC_LOOP (0x15): k(2) + vmin(2) + vmax(2) + tick_div(2).
    task cmd_start_loop(input [15:0] k, input [15:0] vmin, input [15:0] vmax, input [15:0] tick);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h15, junk);
            spi_byte(k[7:0],    junk); spi_byte(k[15:8],    junk);
            spi_byte(vmin[7:0], junk); spi_byte(vmin[15:8], junk);
            spi_byte(vmax[7:0], junk); spi_byte(vmax[15:8], junk);
            spi_byte(tick[7:0], junk); spi_byte(tick[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // Record DAC frames from now until DAC_NF have been decoded (or a timeout).
    task record_dac_frames;
        integer t; begin
            dac_nfr = 0; dac_rec = 1; t = 0;
            while (dac_nfr < DAC_NF && t < 4000) begin #100; t = t + 1; end
            dac_rec = 0;
        end
    endtask
    // The first DAC_NF recorded frames must be base + (k % per), or all `base` when per == 0.
    task check_dac_frames(input [8*10-1:0] what, input integer per, input [15:0] base);
        integer k, bad; reg [15:0] w; begin
            bad = 0;
            if (dac_nfr < DAC_NF) begin
                $display("FAIL %0s: only %0d DAC frames (want %0d)", what, dac_nfr, DAC_NF); errors = errors + 1;
            end else for (k = 0; k < DAC_NF; k = k + 1) begin
                w = (per == 0) ? base : base + (k % per);
                if (dac_fr[k] !== w) begin
                    if (bad < 4) $display("FAIL %0s: DAC frame %0d = %04h, want %04h", what, k, dac_fr[k], w);
                    bad = bad + 1; errors = errors + 1;
                end
            end
        end
    endtask

    // ---- PSRAM QPI write-slave: decode 0x38 + 24-bit addr + data into per-region
    //      byte arrays.  Only observes (writes need no read data). ----
    wire [3:0] ps_io = {psram_io3, psram_io2, psram_io1, psram_io0};
    reg [7:0] adc_mem [0:255];
    reg [7:0] la_mem  [0:255];
    integer   adc_n, la_n;
    integer   nib_i, in_la;
    reg [7:0]  d_cmd, d_byte;
    reg [23:0] d_addr, wr_ptr;
    initial begin nib_i=0; adc_n=0; la_n=0; in_la=0; end

    always @(negedge psram_cs) nib_i = 0;
    always @(posedge psram_sclk) begin
        if (psram_cs === 1'b0) begin
            case (nib_i)
                0: d_cmd[7:4]    = ps_io;
                1: d_cmd[3:0]    = ps_io;
                2: d_addr[23:20] = ps_io;
                3: d_addr[19:16] = ps_io;
                4: d_addr[15:12] = ps_io;
                5: d_addr[11:8]  = ps_io;
                6: d_addr[7:4]   = ps_io;
                7: begin
                    d_addr[3:0] = ps_io;
                    if (d_cmd !== 8'h38) begin $display("FAIL: cmd=%02h want 38", d_cmd); errors=errors+1; end
                    if (d_addr >= ADC_BASE) begin in_la = 0; wr_ptr = d_addr - ADC_BASE; end
                    else                    begin in_la = 1; wr_ptr = d_addr - LA_BASE;  end
                end
                default: begin
                    if (((nib_i-8) & 1) == 0) d_byte[7:4] = ps_io;
                    else begin
                        d_byte[3:0] = ps_io;
                        if (in_la) begin if (wr_ptr[7:0] < 256) la_mem[wr_ptr[7:0]]  = d_byte; la_n=la_n+1; end
                        else       begin if (wr_ptr[7:0] < 256) adc_mem[wr_ptr[7:0]] = d_byte; adc_n=adc_n+1; end
                        wr_ptr = wr_ptr + 24'd1;
                    end
                end
            endcase
            nib_i = nib_i + 1;
        end
    end

    // OP_LA_CAPTURE (0x69): count(3) + divider(2).
    task cmd_la_capture(input [23:0] cnt, input [15:0] div);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h69, junk);
            spi_byte(cnt[7:0], junk); spi_byte(cnt[15:8], junk); spi_byte(cnt[23:16], junk);
            spi_byte(div[7:0], junk); spi_byte(div[15:8], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_SET_TRIGGER (0x33, v35): channel + mode + flags (0).
    task cmd_set_trigger(input [7:0] ch, input [7:0] mode);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h33, junk); spi_byte(ch, junk); spi_byte(mode, junk); spi_byte(8'h00, junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask
    // OP_SET_DAC_STOP_AFTER (0x14): cycles(4).
    task cmd_stop_after(input [31:0] cycles);
        reg [7:0] junk; begin
            csn = 1'b0; #(SPI_HALF);
            spi_byte(8'h14, junk);
            spi_byte(cycles[7:0], junk);   spi_byte(cycles[15:8], junk);
            spi_byte(cycles[23:16], junk); spi_byte(cycles[31:24], junk);
            #(SPI_HALF); csn = 1'b1; #(4*SPI_HALF);
        end
    endtask

    // poll STATUS until CAP_DONE (bit2) or timeout
    task wait_done(input integer max_polls);
        reg [7:0] st; integer k; begin
            st = 0; k = 0;
            while (!(st[2]) && k < max_polls) begin
                cmd1(8'h03, st); k = k + 1; #(2000);
            end
            if (!st[2]) begin $display("FAIL: CAP_DONE never set (STATUS=0x%02x)", st); errors=errors+1; end
            else if (st[5]) begin $display("FAIL: capture OVERFLOW (STATUS=0x%02x)", st); errors=errors+1; end
        end
    endtask

    // ---- sample-period monitor (v32): the LA and ADC producers must sample EXACTLY every
    //      `divider` clocks of the 24 MHz clk, as firmware reports (rate = 24 MHz / div).
    //      Gateware <= v31 took divider + 1 in both.  Counts clk cycles between LA low-byte
    //      strobes and between ADC sample strobes while per_on is set; the arm resets it. ----
    integer per_on = 0, la_want = 0, adc_want = 0, cyc = 0;
    integer la_prev = 0, la_lo = 0, la_bad = 0, la_hi = 0;
    integer adc_prev = 0, adc_seen = 0, adc_ok = 0, adc_bad = 0, adc_armed = 0, adc_wr = 0;
    // v35 trigger monitor: the cycle of the arm, of t0, of the first LA/ADC byte after the arm, of
    // the (co-triggered) DAC engine start and of the stop-after trip; and any producer write while
    // the capture is still waiting for its trigger (must never happen).
    integer arm_cyc = -1, t0_cyc = -1, t0_n = 0, wait_seen = 0, la0_cyc = -1, adc0_cyc = -1;
    integer dacst_cyc = -1, auto_cyc = -1, chg_cyc = 0, wait_wr = 0;
    always @(posedge dut.clk) begin
        cyc = cyc + 1;
        if (dut.arm) begin arm_cyc = cyc; la0_cyc = -1; adc0_cyc = -1; end
        if (dut.t0) begin t0_cyc = cyc; t0_n = t0_n + 1; end
        if (dut.trig_wait) wait_seen = 1;
        if (dut.trig_wait && (dut.la_wr_stb || dut.adc_wr_stb)) wait_wr = wait_wr + 1;
        if (dut.la_wr_stb  && la0_cyc  < 0) la0_cyc  = cyc;
        if (dut.adc_wr_stb && adc0_cyc < 0) adc0_cyc = cyc;
        if (dut.dac_start_eff) dacst_cyc = cyc;
        if (dut.dac_autostop)  auto_cyc  = cyc;
        if (dut.la_cap_start) begin la_lo = 0; la_hi = 0; end
        if (dut.cap_start)    begin adc_seen = 0; adc_armed = 1; end
        if (per_on && dut.la_wr_stb) begin
            if (!la_hi) begin
                if (la_lo > 0 && cyc - la_prev != la_want) begin
                    if (la_bad < 4) $display("FAIL: LA sample %0d came %0d clocks after the last (want %0d)",
                                             la_lo, cyc - la_prev, la_want);
                    la_bad = la_bad + 1;
                end
                la_prev = cyc; la_lo = la_lo + 1;
            end
            la_hi = !la_hi;
        end
        if (per_on && dut.adc_wr_stb) adc_wr = adc_wr + 1;       // ADC bytes produced (2/sample)
        if (per_on && adc_armed && dut.adc_sample_stb) begin
            // only after the arm (the engine runs on the OLD divider during the SPI command),
            // and skip the first two intervals after it: the free-running engine picks the
            // new divider up at its next conversion start
            if (adc_seen >= 2) begin
                if (cyc - adc_prev != adc_want) begin
                    if (adc_bad < 4) $display("FAIL: ADC sample came %0d clocks after the last (want %0d)",
                                              cyc - adc_prev, adc_want);
                    adc_bad = adc_bad + 1;
                end else adc_ok = adc_ok + 1;
            end
            adc_prev = cyc; adc_seen = adc_seen + 1;
        end
    end

    // v35: a triggered capture that has not fired yet — TRIGGER_STATUS says waiting, STATUS says
    // busy and not done, and nothing has been produced or written.
    task check_waiting(input [8*24-1:0] what);
        reg [7:0] st; begin
            cmd1(8'h34, st);
            if (st !== 8'h01) begin $display("FAIL trig %0s: TRIGGER_STATUS=%02h while waiting (want 01)", what, st); errors=errors+1; end
            cmd1(8'h03, st);
            if (!st[1] || st[2]) begin $display("FAIL trig %0s: STATUS=%02h while waiting (want busy, not done)", what, st); errors=errors+1; end
            if (t0_n != 0 || la_n != 0 || adc_n != 0 || la0_cyc >= 0 || adc0_cyc >= 0) begin
                $display("FAIL trig %0s: capture started before the trigger (t0 %0d, LA %0d / ADC %0d bytes)",
                         what, t0_n, la_n, adc_n); errors=errors+1; end
        end
    endtask
    // v35: after a trigger fired and the capture completed.
    task check_fired(input [8*24-1:0] what);
        reg [7:0] st; begin
            cmd1(8'h34, st);
            if (st !== 8'h02) begin $display("FAIL trig %0s: TRIGGER_STATUS=%02h after the capture (want 02)", what, st); errors=errors+1; end
            if (t0_n != 1) begin $display("FAIL trig %0s: t0 fired %0d times (want 1)", what, t0_n); errors=errors+1; end
            if (wait_wr != 0) begin $display("FAIL trig %0s: %0d producer write(s) while waiting", what, wait_wr); errors=errors+1; end
        end
    endtask
    // v35: the first n LA samples in the LA region are all `word`.
    task check_la_word(input [8*24-1:0] what, input integer n, input [11:0] word);
        integer k, bad; reg [15:0] g; begin
            bad = 0;
            if (la_n < n*2) begin $display("FAIL trig %0s: LA region got %0d bytes, want >= %0d", what, la_n, n*2); errors=errors+1; end
            else for (k = 0; k < n; k = k + 1) begin
                g = {la_mem[k*2+1], la_mem[k*2]};
                if ((g & 16'h0FFF) !== {4'b0, word}) begin
                    if (bad < 3) $display("FAIL trig %0s: LA[%0d]=%03h want %03h", what, k, g & 16'h0FFF, word);
                    bad = bad + 1; errors = errors + 1;
                end
            end
        end
    endtask

    localparam NA = 8, NL = 8, NP = 64;
    localparam STOP_N = 300;                 // stop-after cycles for the trigger co-trigger pass
    reg [7:0] r; integer i;
    reg [15:0] got; reg [15:0] want;
    integer base_la6, base_la2, base_dac, base_auto;
    initial begin
        #5000;
        cmd1(8'h01, r); if (r !== 8'hA5) begin $display("FAIL: PING=%02h", r); errors=errors+1; end
        cmd_args1(8'h23, 8'h01);          // CAPTURE_TEST: ADC ramp (+0x0101/sample)
        bus_own = 1'b0;                   // hand the shared bus to the iCE40 for the capture
        #1000;
        cmd_capture(NA, 16'd40, NL, 16'd6);   // 8 ADC @div40, 8 LA @div6, one trigger
        wait_done(4000);
        #4000;                            // let the final chunk drain
        bus_own = 1'b1;

        // ---- ADC region: ramp bytes (LE 16-bit, +0x0101/sample). Anchor to the
        //      first captured value (the free-running ramp counter's phase). ----
        if (adc_n < NA*2) begin $display("FAIL: ADC region got %0d bytes, want >= %0d", adc_n, NA*2); errors=errors+1; end
        else begin
            want = {adc_mem[1], adc_mem[0]};            // first sample = ramp anchor
            for (i = 0; i < NA; i = i + 1) begin
                got = {adc_mem[i*2+1], adc_mem[i*2+0]};
                if (got !== want) begin $display("FAIL: ADC[%0d]=%04h want %04h", i, got, want); errors=errors+1; end
                want = want + 16'h0101;
            end
        end

        // ---- LA region: every sample = LA_WORD (0x0A5A, low 12 bits) ----
        if (la_n < NL*2) begin $display("FAIL: LA region got %0d bytes, want >= %0d", la_n, NL*2); errors=errors+1; end
        else begin
            for (i = 0; i < NL; i = i + 1) begin
                got = {la_mem[i*2+1], la_mem[i*2+0]};
                if ((got & 16'h0FFF) !== {4'b0, LA_WORD}) begin
                    $display("FAIL: LA[%0d]=%04h want %03h", i, got, LA_WORD); errors=errors+1; end
            end
        end

        $display("  [both]   ADC region: %0d bytes, first %04h; LA region: %0d bytes, first %04h",
                 adc_n, {adc_mem[1],adc_mem[0]}, la_n, {la_mem[1],la_mem[0]});

        // ---- ADC-only (la_cnt=0): CAP_DONE must still assert (the unified opcode
        //      pulses both arms, but run_la must be gated off when la_cnt=0), and
        //      only the ADC region gets written. ----
        adc_n = 0; la_n = 0;
        bus_own = 1'b0; #1000;
        cmd_capture(NA, 16'd40, 16'd0, 16'd6);
        wait_done(4000); #4000; bus_own = 1'b1;
        if (adc_n < NA*2) begin $display("FAIL: ADC-only wrote %0d ADC bytes", adc_n); errors=errors+1; end
        if (la_n != 0)    begin $display("FAIL: ADC-only wrote %0d LA bytes (want 0)", la_n); errors=errors+1; end
        $display("  [adc]    ADC region: %0d bytes; LA region: %0d bytes", adc_n, la_n);

        // ---- LA-only (adc_cnt=0): CAP_DONE must assert; only the LA region written.
        adc_n = 0; la_n = 0;
        bus_own = 1'b0; #1000;
        cmd_capture(16'd0, 16'd40, NL, 16'd6);
        wait_done(4000); #4000; bus_own = 1'b1;
        if (la_n < NL*2)  begin $display("FAIL: LA-only wrote %0d LA bytes", la_n); errors=errors+1; end
        if (adc_n != 0)   begin $display("FAIL: LA-only wrote %0d ADC bytes (want 0)", adc_n); errors=errors+1; end
        $display("  [la]     ADC region: %0d bytes; LA region: %0d bytes", adc_n, la_n);

        // ---- SAMPLE PERIOD (v32): LA at divider 2 (the 12 MS/s peak, no idle gap for the
        //      ring to read in — it must buffer the whole burst) and ADC at divider 100; both
        //      periods must be exactly the divider, and every LA sample must arrive intact. ----
        adc_n = 0; la_n = 0; adc_ok = 0; adc_bad = 0; la_bad = 0;
        la_want = 2; adc_want = 100; adc_armed = 0; per_on = 1;
        bus_own = 1'b0; #1000;
        cmd_capture(NA, 16'd100, NP, 16'd2);
        wait_done(4000); #4000; bus_own = 1'b1;
        per_on = 0;
        if (la_lo != NP) begin $display("FAIL: period pass saw %0d LA samples (want %0d)", la_lo, NP); errors=errors+1; end
        if (la_bad != 0) begin $display("FAIL: %0d LA sample(s) off the divider-2 period", la_bad); errors=errors+1; end
        if (adc_bad != 0 || adc_ok < 3) begin
            $display("FAIL: ADC period: %0d bad / %0d good interval(s) at divider 100", adc_bad, adc_ok); errors=errors+1; end
        if (la_n < NP*2) begin $display("FAIL: period pass LA region got %0d bytes, want >= %0d", la_n, NP*2); errors=errors+1; end
        else begin
            for (i = 0; i < NP; i = i + 1) begin
                got = {la_mem[i*2+1], la_mem[i*2+0]};
                if ((got & 16'h0FFF) !== {4'b0, LA_WORD}) begin
                    $display("FAIL: period pass LA[%0d]=%04h want %03h", i, got, LA_WORD); errors=errors+1; end
            end
        end
        $display("  [period] LA %0d samples @div2 (%0d off-period), ADC %0d intervals @div100 (%0d off)",
                 la_lo, la_bad, adc_ok, adc_bad);

        // ---- SECOND RUN WITH DIFFERENT PARAMETERS (v34): re-arm straight away with new counts AND
        //      new dividers.  Every count/divider is a register the arm latches; a stale one (the
        //      v33 DAC bug class) shows up here as the previous run's count or period. ----
        adc_n = 0; la_n = 0; adc_ok = 0; adc_bad = 0; la_bad = 0; adc_wr = 0;
        la_want = 6; adc_want = 72; adc_armed = 0; per_on = 1;
        bus_own = 1'b0; #1000;
        cmd_capture(24'd12, 16'd72, 24'd20, 16'd6);
        wait_done(4000); #4000; bus_own = 1'b1;
        per_on = 0;
        if (la_lo != 20) begin $display("FAIL re-arm: %0d LA samples (want 20; the previous run took %0d)", la_lo, NP); errors=errors+1; end
        if (la_bad != 0) begin $display("FAIL re-arm: %0d LA sample(s) off the divider-6 period", la_bad); errors=errors+1; end
        if (adc_bad != 0 || adc_ok < 3) begin
            $display("FAIL re-arm: ADC period: %0d bad / %0d good interval(s) at divider 72", adc_bad, adc_ok); errors=errors+1; end
        if (adc_wr != 24) begin $display("FAIL re-arm: ADC produced %0d bytes (want 24 = 12 samples)", adc_wr); errors=errors+1; end
        $display("  [re-arm] LA %0d samples @div6 (%0d off), ADC %0d bytes @div72 (%0d intervals, %0d off)",
                 la_lo, la_bad, adc_wr, adc_ok, adc_bad);

        // ---- CO-TRIGGER (v27, OP_DAC_ARM_ON_CAPTURE 0x19): the DAC start is DEFERRED to the
        //      capture arm, so the engine must NOT run after START_DAC and must start exactly when
        //      OP_CAPTURE fires (DAC sample 0 == capture t0). ----
        cmd_op0(8'h12);                        // STOP_DAC — clean slate
        #1000;
        cmd_op0(8'h19);                        // OP_DAC_ARM_ON_CAPTURE — stage the co-trigger
        cmd_start_dac(16'd8, 16'd4);           // shallow replay arm — its engine start is HELD
        #2000;                                 // give any (suppressed) CDC start time to appear
        if (dut.dac_pend !== 1'b1) begin
            $display("FAIL cotrig: dac_pend not set after ARM_ON_CAPTURE + START_DAC"); errors=errors+1; end
        if (dut.dac_running !== 1'b0) begin
            $display("FAIL cotrig: DAC ran before the capture (dac_running=1, start not deferred)"); errors=errors+1; end

        adc_n = 0; la_n = 0;
        bus_own = 1'b0; #1000;
        cmd_capture(NA, 16'd40, NL, 16'd6);    // arm -> the deferred DAC start fires on t0
        #1500;                                 // engine start propagates through the clk48 CDC
        if (dut.dac_running !== 1'b1) begin
            $display("FAIL cotrig: DAC did not start at the capture arm (dac_running=0)"); errors=errors+1; end
        if (dut.dac_pend !== 1'b0) begin
            $display("FAIL cotrig: dac_pend not cleared by the arm"); errors=errors+1; end
        wait_done(4000); #4000; bus_own = 1'b1;
        cmd_op0(8'h12); #1000;                 // stop the DAC

        // sanity: a NORMAL start (no co-trigger) must run immediately.
        cmd_start_dac(16'd8, 16'd4);
        #2000;
        if (dut.dac_running !== 1'b1) begin
            $display("FAIL cotrig: normal START_DAC did not run immediately (co-trigger leaked)"); errors=errors+1; end
        cmd_op0(8'h12); #1000;
        $display("  [cotrig] DAC start held after START_DAC, fired at the capture arm, cleared after");

        // ---- DEEP (PSRAM) co-trigger control-plane: the deep replay mode is set at STAGE (so its
        //      reader can prefill — dac_autostop_lat must be CLEARED by the raw start), the engine
        //      start is deferred, and STOP_DAC cancels the staged start.  (bus_own stays 1 so the
        //      deep reader is held in reset — this checks the control plane, not the datapath; the
        //      reader/engine byte order across the loop wrap is covered by tb_dac_psram_replay.) ----
        cmd_op0(8'h12); #1000;                 // clean slate
        cmd_op0(8'h19);                        // ARM_ON_CAPTURE
        cmd_start_dac_psram(24'h400000, 24'd8, 16'd4);   // deep replay arm — engine start held
        #2000;
        if (dut.dac_pend !== 1'b1) begin
            $display("FAIL cotrig-deep: dac_pend not set after ARM_ON_CAPTURE + START_DAC_PSRAM"); errors=errors+1; end
        if (dut.dac_running !== 1'b0) begin
            $display("FAIL cotrig-deep: deep DAC ran before the capture (start not deferred)"); errors=errors+1; end
        if (dut.dac_psram_mode !== 1'b1) begin
            $display("FAIL cotrig-deep: deep replay mode not set at stage"); errors=errors+1; end
        if (dut.dac_autostop_lat !== 1'b0) begin
            $display("FAIL cotrig-deep: reader gated off at stage (lat set) — FIFO can't prefill for t0"); errors=errors+1; end
        cmd_op0(8'h12); #1000;                 // STOP_DAC cancels the staged start + leaves deep mode
        if (dut.dac_pend !== 1'b0) begin
            $display("FAIL cotrig-deep: STOP_DAC did not cancel the staged start"); errors=errors+1; end
        if (dut.dac_psram_mode !== 1'b0) begin
            $display("FAIL cotrig-deep: STOP_DAC did not leave deep replay mode"); errors=errors+1; end
        $display("  [cotrig] deep replay defers its start + primes the reader at stage; STOP cancels");

        // ---- DAC RESTART ON A SHORTER WAVEFORM (v33): the first pass must loop at the NEW period.
        //      START_DAC's period/divider cross into clk48 copies latched by the synchronized start
        //      pulse; the engine loaded its loop counter from that SAME pulse, so it got the
        //      PREVIOUS copy and its first pass ran the old length — past the end of the new
        //      waveform into stale BRAM.  On the bench a 5 kHz square (181 samples) after a 200 Hz
        //      one (2034) played ~2 ms of the old square's flat levels, which broke
        //      test_adc_sample_rate_matches_the_host_clock.  Play DAC_PA samples, stop (mid-frame,
        //      like the capture-tied auto-stop), load a DAC_PB-sample waveform, and require the first
        //      DAC_NF frames to be exactly the new samples in loop order. ----
        cmd_op0(8'h12); #1000;
        cmd_load_wave16(DAC_PA, 16'hA000);
        cmd_start_dac(DAC_PA, 16'd200);
        #20000;                                // several frames of the long waveform
        cmd_op0(8'h12); #1000;
        cmd_load_wave16(DAC_PB, 16'hB000);
        dac_nfr = 0; dac_rec = 1;
        cmd_start_dac(DAC_PB, 16'd4);
        i = 0;
        while (dac_nfr < DAC_NF && i < 4000) begin #100; i = i + 1; end
        dac_rec = 0;
        cmd_op0(8'h12); #1000;
        if (dac_nfr < DAC_NF) begin
            $display("FAIL dac-restart: only %0d DAC frames after the restart (want %0d)", dac_nfr, DAC_NF); errors=errors+1; end
        else begin
            r = 0;
            for (i = 0; i < DAC_NF; i = i + 1) begin
                want = 16'hB000 + (i % DAC_PB);
                if (dac_fr[i] !== want) begin
                    if (r < 4) $display("FAIL dac-restart: frame %0d = %04h, want %04h (first pass ran the old period?)",
                                        i, dac_fr[i], want);
                    r = r + 1; errors = errors + 1;
                end
            end
        end
        $display("  [dac]    restart %0d->%0d samples: first %0d frames %04h %04h %04h %04h %04h ...",
                 DAC_PA, DAC_PB, DAC_NF, dac_fr[0], dac_fr[1], dac_fr[2], dac_fr[3], dac_fr[4]);

        // ---- MEASURE after a longer DAC run (v34): OP_START_MEASURE sets the DAC period (= count)
        //      and arms an ADC capture on the same cycle.  Its DAC start takes the same clk48
        //      crossing, so its first pass must use the NEW period too.  (The BRAM still holds the
        //      DAC_PB-sample B waveform in front of the old A waveform's tail.) ----
        cmd_start_dac(DAC_PA, 16'd200);
        #20000;
        cmd_op0(8'h12); #1000;
        adc_n = 0; la_n = 0;
        bus_own = 1'b0; #1000;
        cmd_measure(DAC_PB, 16'd4, 16'd72);
        record_dac_frames;
        wait_done(4000); #4000; bus_own = 1'b1;
        cmd_op0(8'h12); #1000;
        check_dac_frames("measure", DAC_PB, 16'hB000);
        if (adc_n < DAC_PB*2) begin $display("FAIL measure: ADC region got %0d bytes, want >= %0d", adc_n, DAC_PB*2); errors=errors+1; end
        $display("  [measure] after a %0d-sample DAC: frames %04h %04h %04h %04h %04h ..., ADC %0d bytes",
                 DAC_PA, dac_fr[0], dac_fr[1], dac_fr[2], dac_fr[3], dac_fr[4], adc_n);

        // ---- CONTROL LOOP re-armed with a different window (v34, loop image): the loop holds v at
        //      vmin while disarmed, so an arm with vmin = vmax = V must drive exactly V from its
        //      first frame — and a re-arm with a new V must never show the previous one. ----
        cmd_load_wave16(DAC_PB, 16'h4000);            // curve entry 0 (the index for input 0) defined
        cmd_loop_src(8'd1, 16'd0, 16'd0);             // fixed input 0, no sweep
        cmd_start_loop(16'h7FFF, 16'h1234, 16'h1234, 16'd16);
        record_dac_frames;
        cmd_op0(8'h12); #1000;
        check_dac_frames("loop arm", 0, 16'h1234);
        cmd_start_loop(16'h7FFF, 16'h5678, 16'h5678, 16'd16);
        record_dac_frames;
        cmd_op0(8'h12); #1000;
        check_dac_frames("loop rearm", 0, 16'h5678);
        $display("  [loop]   arm -> %04h, re-arm -> first frames %04h %04h %04h",
                 16'h1234, dac_fr[0], dac_fr[1], dac_fr[2]);

        // ==== CAPTURE TRIGGER (v35, OP_SET_TRIGGER 0x33 / OP_TRIGGER_STATUS 0x34) ====
        // Every relationship is measured on an UNTRIGGERED run first (from the arm) and must hold
        // identically on the triggered run measured from t0.  A pin change reaches t0 three clk
        // edges later: two synchroniser flops, then the cycle the condition is tested.

        // -- untriggered baseline: LA-only @div6 (trigger off since reset) --
        la_drv = 12'h000;
        adc_n = 0; la_n = 0; wait_seen = 0; t0_n = 0;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd16, 16'd6);
        wait_done(4000); #4000; bus_own = 1'b1;
        base_la6 = la0_cyc - arm_cyc;
        if (wait_seen)             begin $display("FAIL trig: an untriggered capture waited"); errors=errors+1; end
        if (t0_n != 1 || t0_cyc != arm_cyc) begin $display("FAIL trig: untriggered t0 (%0d x, cycle %0d) is not the arm (%0d)", t0_n, t0_cyc, arm_cyc); errors=errors+1; end
        cmd1(8'h34, r); if (r !== 8'h00) begin $display("FAIL trig: TRIGGER_STATUS=%02h after an untriggered capture", r); errors=errors+1; end
        check_la_word("untriggered", 16, 12'h000);

        // -- RISING on LA4 (ch 3).  The pin is already HIGH at the arm, and other pins move while it
        //    is low: neither may fire.  Count 16 @div6, and STATUS/TRIGGER_STATUS polls in between
        //    (they overwrite the dispatcher's payload, which the held producers must not need). --
        la_drv = 12'h008;
        cmd_set_trigger(8'd3, 8'd1);
        adc_n = 0; la_n = 0; t0_n = 0; wait_wr = 0; la_bad = 0; la_want = 6; adc_armed = 0; per_on = 1;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd16, 16'd6);
        #20000;  check_waiting("rising, pin high at arm");
        la_drv = 12'h000; #20000; la_drv = 12'hFF7; #20000;
        check_waiting("rising, other pins");
        chg_cyc = cyc; la_drv = 12'hA5A;               // LA4 rises
        wait_done(4000); #4000; bus_own = 1'b1; per_on = 0;
        check_fired("rising");
        if (t0_cyc - chg_cyc != 3)       begin $display("FAIL trig rising: t0 %0d clk after the pin (want 3)", t0_cyc - chg_cyc); errors=errors+1; end
        if (la0_cyc - t0_cyc != base_la6) begin $display("FAIL trig rising: first LA byte %0d clk after t0 (untriggered: %0d after the arm)", la0_cyc - t0_cyc, base_la6); errors=errors+1; end
        if (la_lo != 16 || la_bad != 0)  begin $display("FAIL trig rising: %0d LA samples, %0d off the divider-6 period", la_lo, la_bad); errors=errors+1; end
        check_la_word("rising", 16, 12'hA5A);
        $display("  [trig]   rising LA4: waited through a high pin + other edges; t0 = pin+3, first LA byte t0+%0d (arm+%0d untriggered)",
                 la0_cyc - t0_cyc, base_la6);

        // -- untriggered baseline for the unified capture + co-trigger + stop-after --
        cmd_set_trigger(8'd7, 8'd0);                   // off
        cmd_op0(8'h12); #1000;
        cmd_op0(8'h19); cmd_start_dac(16'd8, 16'd4); cmd_stop_after(STOP_N);
        dacst_cyc = -1; auto_cyc = -1; adc_n = 0; la_n = 0; t0_n = 0;
        bus_own = 1'b0; #1000;
        cmd_capture(NA, 16'd100, 24'd32, 16'd6);
        wait_done(4000); #4000; bus_own = 1'b1;
        cmd_op0(8'h12); #1000;
        base_dac = dacst_cyc - arm_cyc; base_auto = auto_cyc - arm_cyc;
        if (base_dac != 0 || base_auto != STOP_N) begin
            $display("FAIL trig: untriggered co-trigger at arm+%0d (want 0), stop-after at arm+%0d (want %0d)", base_dac, base_auto, STOP_N); errors=errors+1; end

        // -- FALLING on LA8 (ch 7), unified CAPTURE (ADC ramp + LA) with the DAC co-trigger and
        //    SET_DAC_STOP_AFTER: the DAC must not start, and the stop-after must not count, until t0. --
        la_drv = 12'hFFF;
        cmd_set_trigger(8'd7, 8'd2);
        cmd_op0(8'h19); cmd_start_dac(16'd8, 16'd4); cmd_stop_after(STOP_N);
        dacst_cyc = -1; auto_cyc = -1; adc_n = 0; la_n = 0; t0_n = 0; wait_wr = 0;
        bus_own = 1'b0; #1000;
        cmd_capture(NA, 16'd100, 24'd32, 16'd6);
        #20000;  check_waiting("falling, co-trigger");
        if (dut.dac_running !== 1'b0 || dut.dac_pend !== 1'b1 || dacst_cyc >= 0) begin
            $display("FAIL trig falling: the co-triggered DAC started before the trigger"); errors=errors+1; end
        if (auto_cyc >= 0 || dut.cap_stop_cnt != STOP_N) begin
            $display("FAIL trig falling: stop-after counted while waiting (count %0d)", dut.cap_stop_cnt); errors=errors+1; end
        chg_cyc = cyc; la_drv = 12'h000;               // LA8 falls
        wait_done(4000); #4000; bus_own = 1'b1;
        cmd_op0(8'h12); cmd_stop_after(32'd0); #1000;
        check_fired("falling");
        if (t0_cyc - chg_cyc != 3) begin $display("FAIL trig falling: t0 %0d clk after the pin (want 3)", t0_cyc - chg_cyc); errors=errors+1; end
        if (dacst_cyc - t0_cyc != base_dac) begin $display("FAIL trig falling: co-trigger DAC start at t0+%0d (untriggered arm+%0d)", dacst_cyc - t0_cyc, base_dac); errors=errors+1; end
        if (auto_cyc - t0_cyc != base_auto) begin $display("FAIL trig falling: stop-after tripped at t0+%0d (untriggered arm+%0d)", auto_cyc - t0_cyc, base_auto); errors=errors+1; end
        if (adc0_cyc <= t0_cyc) begin $display("FAIL trig falling: first ADC byte (cycle %0d) not after t0 (%0d)", adc0_cyc, t0_cyc); errors=errors+1; end
        if (adc_n < NA*2) begin $display("FAIL trig falling: ADC region got %0d bytes, want >= %0d", adc_n, NA*2); errors=errors+1; end
        else begin
            want = {adc_mem[1], adc_mem[0]};
            for (i = 0; i < NA; i = i + 1) begin
                got = {adc_mem[i*2+1], adc_mem[i*2+0]};
                if (got !== want) begin $display("FAIL trig falling: ADC[%0d]=%04h want %04h", i, got, want); errors=errors+1; end
                want = want + 16'h0101;
            end
        end
        check_la_word("falling", 32, 12'h000);
        $display("  [trig]   falling LA8 + co-trigger: DAC start t0+%0d, stop-after t0+%0d, first ADC byte t0+%0d",
                 dacst_cyc - t0_cyc, auto_cyc - t0_cyc, adc0_cyc - t0_cyc);

        // -- HIGH on LA1 (ch 0), already high at the arm: fires on the first waiting cycle --
        la_drv = 12'h001;
        cmd_set_trigger(8'd0, 8'd3);
        adc_n = 0; la_n = 0; t0_n = 0; wait_wr = 0;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd8, 16'd6);
        wait_done(4000); #4000; bus_own = 1'b1;
        check_fired("high");
        if (t0_cyc != arm_cyc + 1) begin $display("FAIL trig high: t0 at arm+%0d (want arm+1 for a pin already high)", t0_cyc - arm_cyc); errors=errors+1; end
        if (la0_cyc - t0_cyc != base_la6) begin $display("FAIL trig high: first LA byte t0+%0d (want %0d)", la0_cyc - t0_cyc, base_la6); errors=errors+1; end
        check_la_word("high", 8, 12'h001);

        // -- LOW on LA12 (ch 11): waits while high, fires when it drops --
        la_drv = 12'hFFF;
        cmd_set_trigger(8'd11, 8'd4);
        adc_n = 0; la_n = 0; t0_n = 0; wait_wr = 0;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd8, 16'd6);
        #20000;  check_waiting("low");
        chg_cyc = cyc; la_drv = 12'h7FF;
        wait_done(4000); #4000; bus_own = 1'b1;
        check_fired("low");
        if (t0_cyc - chg_cyc != 3) begin $display("FAIL trig low: t0 %0d clk after the pin (want 3)", t0_cyc - chg_cyc); errors=errors+1; end
        check_la_word("low", 8, 12'h7FF);
        $display("  [trig]   high LA1 (already high): t0 = arm+%0d; low LA12 fired", t0_cyc - arm_cyc);

        // -- ABORT while waiting: a unified CAPTURE re-armed with both counts 0 ends it, the old
        //    trigger condition afterwards starts nothing, and a staged co-trigger is NOT fired by
        //    the abort (t0 is only ever the trigger cycle while a trigger is set) --
        la_drv = 12'h000;
        cmd_set_trigger(8'd5, 8'd1);
        cmd_op0(8'h19); cmd_start_dac(16'd8, 16'd4);   // stage a co-triggered DAC start
        adc_n = 0; la_n = 0; t0_n = 0; wait_wr = 0; dacst_cyc = -1;
        bus_own = 1'b0; #1000;
        cmd_capture(NA, 16'd100, 24'd16, 16'd6);
        #20000;  check_waiting("abort (capture)");
        cmd_capture(24'd0, 16'd100, 24'd0, 16'd6);
        wait_done(4000); #4000;
        cmd1(8'h34, r); if (r !== 8'h00) begin $display("FAIL trig abort: TRIGGER_STATUS=%02h after the abort (want 00)", r); errors=errors+1; end
        if (dacst_cyc >= 0 || dut.dac_running !== 1'b0 || dut.dac_pend !== 1'b1) begin
            $display("FAIL trig abort: the abort fired the staged co-trigger (dac_running=%b dac_pend=%b)", dut.dac_running, dut.dac_pend); errors=errors+1; end
        cmd_op0(8'h12); #1000;                          // STOP_DAC cancels the staged start
        if (dut.dac_pend !== 1'b0) begin $display("FAIL trig abort: STOP_DAC did not cancel the staged co-trigger"); errors=errors+1; end
        la_drv = 12'h020; #20000;                        // the aborted run's rising edge on LA6
        if (la_n != 0 || adc_n != 0 || la0_cyc >= 0 || adc0_cyc >= 0) begin
            $display("FAIL trig abort: the aborted capture produced data (LA %0d / ADC %0d bytes)", la_n, adc_n); errors=errors+1; end
        bus_own = 1'b1;
        // ... and an LA-only capture aborted with LA_CAPTURE count 0
        la_drv = 12'h000; t0_n = 0;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd16, 16'd6);
        #20000;  check_waiting("abort (la_capture)");
        cmd_la_capture(24'd0, 16'd6);
        wait_done(4000); #4000;
        la_drv = 12'h020; #20000;
        cmd1(8'h34, r);
        if (r !== 8'h00 || la_n != 0 || la0_cyc >= 0) begin
            $display("FAIL trig abort: LA-only abort left TRIGGER_STATUS=%02h, %0d LA bytes", r, la_n); errors=errors+1; end
        bus_own = 1'b1;
        $display("  [trig]   abort while waiting (CAPTURE 0/0 and LA_CAPTURE 0): done, nothing produced");

        // -- RE-ARM with different parameters: FALLING on LA10 (ch 9), 24 samples @div2 --
        la_drv = 12'hFFF;
        cmd_set_trigger(8'd9, 8'd2);
        adc_n = 0; la_n = 0; t0_n = 0; wait_wr = 0; la_bad = 0; la_want = 2; adc_armed = 0; per_on = 1;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd24, 16'd2);
        #20000;  check_waiting("re-arm");
        chg_cyc = cyc; la_drv = 12'h1FF;               // LA10 falls
        wait_done(4000); #4000; bus_own = 1'b1; per_on = 0;
        check_fired("re-arm");
        base_la2 = la0_cyc - t0_cyc;
        if (t0_cyc - chg_cyc != 3)      begin $display("FAIL trig re-arm: t0 %0d clk after the pin (want 3)", t0_cyc - chg_cyc); errors=errors+1; end
        if (la_lo != 24 || la_bad != 0) begin $display("FAIL trig re-arm: %0d LA samples (want 24), %0d off the divider-2 period", la_lo, la_bad); errors=errors+1; end
        check_la_word("re-arm", 24, 12'h1FF);

        // -- trigger OFF again: an untriggered capture after triggered ones is the v34 capture --
        la_drv = 12'hA5A;
        cmd_set_trigger(8'd9, 8'd0);
        adc_n = 0; la_n = 0; t0_n = 0; wait_seen = 0; la_bad = 0; la_want = 2; adc_armed = 0; per_on = 1;
        bus_own = 1'b0; #1000;
        cmd_la_capture(24'd24, 16'd2);
        wait_done(4000); #4000; bus_own = 1'b1; per_on = 0;
        cmd1(8'h34, r);
        if (r !== 8'h00 || wait_seen || t0_n != 1 || t0_cyc != arm_cyc) begin
            $display("FAIL trig off: TRIGGER_STATUS=%02h waited=%0d t0 %0d x at arm+%0d", r, wait_seen, t0_n, t0_cyc - arm_cyc); errors=errors+1; end
        if (la0_cyc - arm_cyc != base_la2) begin $display("FAIL trig off: first LA byte arm+%0d (triggered div2 run: t0+%0d)", la0_cyc - arm_cyc, base_la2); errors=errors+1; end
        if (la_lo != 24 || la_bad != 0)   begin $display("FAIL trig off: %0d LA samples, %0d off the divider-2 period", la_lo, la_bad); errors=errors+1; end
        check_la_word("off", 24, 12'hA5A);
        $display("  [trig]   re-arm falling LA10 @div2 then trigger off: first LA byte t0+%0d == arm+%0d; sim time %0t",
                 base_la2, la0_cyc - arm_cyc, $time);

        if (errors == 0)
            $display("PASS tb_top_capture: OP_CAPTURE both/ADC-only/LA-only all write the right region(s)");
        else
            $display("FAIL tb_top_capture: %0d error(s)", errors);
        $finish;
    end

    initial begin #20000000 $display("FAIL tb_top_capture: timeout"); $finish; end
endmodule
