// ============================================================================
// tb_la_change.v — the v50 LA change capture, sampler to bytes:
// la48_sampler -> la_change_capture -> spram_ring48 -> a byte sink with random backpressure,
// plus la_byte_pack -> spram_ring48 for the raw format.
//
// Each run drives the 14 LA pins on clk48 negedges, decodes the bytes the ring sends out
// (format in la_change_capture.v) and checks:
//   * the decoded state at every tick equals the sample pairs the encoder took (pa, pb), so
//     the encoding, the pairing into words, the ring and the byte order are exact;
//   * the decoded states equal the pin levels at every clk48 edge, at one fixed latency, so
//     the sampler's pairs are in order and no tick is lost or doubled;
//   * the stream ends at count*256 - 1 with END (or earlier with `overflow`, SMALL_RING).
//
// Run:  make lachangetest   (both ring sizes)
// ============================================================================
`timescale 1ns/1ps
module tb_la_change;
`ifdef SMALL_RING
    localparam AW = 4;                       // 16 words: overflows under a max-rate burst
`else
    localparam AW = 14;
`endif
    reg clk48 = 1'b0;
    always #10 clk48 = ~clk48;
    reg clkdiv = 1'b0;
    always @(posedge clk48) clkdiv <= ~clkdiv;
    wire clk = clkdiv;

    reg  rst = 1'b1;
    reg  [13:0] la = 14'd0;
    wire [13:0] pa, pb;
    la48_sampler #(.N(14)) smp (.clk48(clk48), .clkdiv(clkdiv), .la_in(la), .pa(pa), .pb(pb));

    reg         start = 1'b0, hold = 1'b0;
    reg  [23:0] count = 24'd0;
    reg  [23:0] limit = 24'hFFFFFF;          // region bytes (huge unless a run tests the end)
    wire [47:0] cwd;
    wire        cstb, cbusy, cdone, covf, rdy;
    la_change_capture #(.N(14)) enc (
        .clk(clk), .rst(rst), .pa(pa), .pb(pb), .start(start), .hold(hold), .count(count), .limit(limit),
        .wd(cwd), .wstb(cstb), .wrdy(rdy), .busy(cbusy), .done(cdone), .overflow(covf));

    reg  [7:0]  rb = 8'd0;
    reg         rb_stb = 1'b0, rb_flush = 1'b0;
    wire [47:0] pwd;
    wire        pstb, ppart;
    wire [2:0]  plen;
    la_byte_pack pk (.clk(clk), .rst(rst), .in_data(rb), .in_stb(rb_stb), .flush(rb_flush),
                     .wd(pwd), .wstb(pstb), .wlen(plen), .partial(ppart));

    reg         raw = 1'b0;
    reg         out_full = 1'b0;
    wire [7:0]  odata;
    wire        ostb, rempty, rovf;
    spram_ring48 #(.AW(AW)) ring (
        .clk(clk), .rst(rst), .clr_ovf(start),
        .in_word(raw ? pwd : cwd), .in_stb(raw ? pstb : cstb), .in_len(raw ? plen : 3'd0), .in_rdy(rdy),
        .out_data(odata), .out_stb(ostb), .out_full(out_full), .empty(rempty), .overflow(rovf));

    // ---- sink: random backpressure (~70% / ~25% duty) ----
    integer seed = 7, slow = 0;
    always @(posedge clk) out_full <= slow ? (($random(seed) & 3) != 0) : (($unsigned($random(seed)) % 10) >= 7);
    localparam MAXB = 1 << 20;
    reg [7:0] bytes [0:MAXB-1];
    integer   nb = 0;
    always @(posedge clk) if (ostb) begin bytes[nb] = odata; nb = nb + 1; end

    // ---- references ----
    localparam MAXT = 1 << 18;
    reg [13:0] ref_s [0:MAXT-1];             // pairs the encoder took, in tick order
    integer    nref = 0;
    always @(posedge clk)
        if (enc.step && !(enc.need_word && !rdy)) begin
            ref_s[nref] = enc.ra; ref_s[nref+1] = enc.rb; nref = nref + 2;
        end
    reg [13:0] pin [0:MAXT-1];               // the pins at every clk48 edge
    integer    npin = 0;
    always @(posedge clk48) begin pin[npin] = la; npin = npin + 1; end
    integer    pin0;                         // npin at the arm

    integer errors = 0;
    reg [13:0] dec [0:MAXT-1];
    integer    ndec;                         // decoded ticks
    integer    end_seen;
    integer    max_d;                        // largest delta seen (keep-alive check)

    // Decode the byte stream into per-tick states.
    task decode;
        integer i, t, d; reg [23:0] e; reg [13:0] st; begin
            ndec = 0; end_seen = 0; t = -1; st = 0; max_d = 0;
            for (i = 0; i + 2 < nb && !end_seen; i = i + 3) begin
                e = {bytes[i+2], bytes[i+1], bytes[i]};
                d = e[23:14];
                if (i == 0) begin
                    if (d != 0) begin $display("FAIL first event delta %0d, want 0", d); errors = errors + 1; end
                    t = 0; st = e[13:0];
                end else if (d == 0) begin
                    end_seen = 1;
                end else begin
                    if (d > max_d) max_d = d;
                    while (ndec < t + d) begin dec[ndec] = st; ndec = ndec + 1; end
                    t = t + d; st = e[13:0];
                end
            end
            if (t >= 0) begin dec[ndec] = st; ndec = ndec + 1; end
        end
    endtask

    task check(input [8*24-1:0] what, input integer want_ticks, input want_ovf);
        integer i, bad, o, best; begin
            decode;
            if (!end_seen) begin $display("FAIL %0s: no END event", what); errors = errors + 1; end
            if (covf !== want_ovf) begin $display("FAIL %0s: overflow %b, want %b", what, covf, want_ovf); errors = errors + 1; end
            if (!want_ovf && ndec != want_ticks) begin
                $display("FAIL %0s: decoded %0d ticks, want %0d", what, ndec, want_ticks); errors = errors + 1;
            end
            if (want_ovf && ndec >= want_ticks) begin
                $display("FAIL %0s: decoded %0d ticks, an overflow must cut it short of %0d", what, ndec, want_ticks); errors = errors + 1;
            end
            if (ndec > nref) begin $display("FAIL %0s: decoded %0d ticks, the encoder took %0d", what, ndec, nref); errors = errors + 1; end
            // 1) the encoder's pairs
            bad = 0;
            for (i = 0; i < ndec && i < nref; i = i + 1)
                if (dec[i] !== ref_s[i]) begin
                    if (bad < 4) $display("FAIL %0s: tick %0d decoded %04h, encoder took %04h", what, i, dec[i], ref_s[i]);
                    bad = bad + 1;
                end
            errors = errors + bad;
            // 2) the pins, at one latency
            // (the first pair was sampled a few clk48 edges BEFORE the arm edge)
            best = -99;
            for (o = -8; o < 16 && best == -99; o = o + 1) begin
                bad = 0;
                for (i = 0; i < ndec && bad == 0; i = i + 1)
                    if (pin0 + o + i < 0 || pin0 + o + i >= npin || dec[i] !== pin[pin0 + o + i]) bad = 1;
                if (bad == 0) best = o;
            end
            if (best == -99) begin $display("FAIL %0s: decoded ticks match the pins at no fixed latency", what); errors = errors + 1; end
            $display("  %0s: %0d bytes, %0d events, %0d ticks, overflow %b, pin latency %0d",
                     what, nb, nb / 3, ndec, covf, best);
        end
    endtask

    task arm(input [23:0] c);
        begin
            @(negedge clk); nb = 0; nref = 0; pin0 = npin; count = c; start = 1'b1;
            @(negedge clk); start = 1'b0;
        end
    endtask
    task wait_done;
        begin
            while (!cdone) @(posedge clk);
            while (!rempty) @(posedge clk);
            repeat (4) @(posedge clk);
        end
    endtask

    // pin patterns, applied on clk48 negedges while `pat` runs
    integer pat = 0, k = 0;
    always @(negedge clk48) begin
        k = k + 1;
        case (pat)
            1: begin                                   // max rate on LA1, slower on two more
                la[0] <= ~la[0];
                if (k % 3 == 0) la[1] <= ~la[1];
                if (k % 7 == 0) la[5] <= ~la[5];
            end
            2: case (k)                                // sparse; the 950..3500 gap needs keep-alives
                   900, 950, 3500, 3501, 4100, 5200: la <= la ^ (14'd1 << (k % 14));
                   default: ;
               endcase
            3: if (k % 5 == 0) la <= la + 14'd1;       // counting bus
            4: if (k % 12 == 0) la[2] <= ~la[2];      // steady 4 M edges/s
            default: ;
        endcase
    end

    integer i;
    initial begin
        repeat (8) @(posedge clk); rst = 1'b0; repeat (4) @(posedge clk);

        // A: max rate burst (1 event per tick on LA1) for 4 x 256 ticks
        pat = 1; arm(24'd4); wait_done; pat = 0;
`ifdef SMALL_RING
        check("burst, small ring", 1024, 1'b1);

        // E: SUSTAINED.  A steady 4 M edges/s (12 MB/s) through the same 16-word ring with the
        // sink at ~70% duty (~17 MB/s, the PSRAM writer's rate): 64 x 256 ticks is ~1365 events,
        // ~40 ring-fulls, so only a drain that keeps up gets through without an overflow.
        pat = 4; arm(24'd64); wait_done; pat = 0;
        check("sustained, small ring", 64 * 256, 1'b0);
        if (nb / 3 < 1300) begin $display("FAIL sustained: only %0d events", nb / 3); errors = errors + 1; end
`else
        check("burst", 1024, 1'b0);

        // B: sparse changes, gaps > 1023 ticks
        pat = 0; k = 0; arm(24'd24);
        pat = 2;
        wait_done; pat = 0;
        check("sparse", 24 * 256, 1'b0);
        if (max_d < 1000) begin $display("FAIL sparse: largest delta %0d, the run never needed a keep-alive", max_d); errors = errors + 1; end
        else $display("  sparse: largest delta %0d (keep-alive)", max_d);

        // F: REGION END.  A 600-byte region under a steady 4 M edges/s: the stream must stop with
        // overflow, END inside the region, and everything before it exact.
        limit = 24'd600; pat = 4; arm(24'd16); wait_done; pat = 0; limit = 24'hFFFFFF;
        check("region end", 16 * 256, 1'b1);
        if (nb > 600) begin $display("FAIL region end: %0d bytes written, region is 600", nb); errors = errors + 1; end

        // C: trigger hold for 200 clk, counting bus
        pat = 3; hold = 1'b1; arm(24'd3);
        repeat (200) @(posedge clk); @(negedge clk); hold = 1'b0;
        wait_done; pat = 0;
        if (nref != 3 * 256) begin $display("FAIL hold: encoder took %0d ticks, want %0d", nref, 3 * 256); errors = errors + 1; end
        // pins are checked from the release, not the arm
        pin0 = pin0 + 2 * 200;
        check("trigger hold", 3 * 256, 1'b0);

        // D: raw bytes through la_byte_pack, a partial last word
        raw = 1'b1; slow = 1; nb = 0;
        @(negedge clk);
        for (i = 0; i < 100; i = i + 1) begin
            rb = i[7:0] ^ 8'h5A; rb_stb = 1'b1; @(negedge clk);
            rb_stb = 1'b0; @(negedge clk);
        end
        rb_flush = 1'b1; @(negedge clk); rb_flush = 1'b0;
        while (!rempty) @(posedge clk);
        repeat (4) @(posedge clk);
        if (nb != 100) begin $display("FAIL raw: %0d bytes out, want 100 (the tail word trimmed)", nb); errors = errors + 1; end
        for (i = 0; i < 100; i = i + 1)
            if (bytes[i] !== (i[7:0] ^ 8'h5A)) begin
                if (errors < 8) $display("FAIL raw: byte %0d = %02h, want %02h", i, bytes[i], i[7:0] ^ 8'h5A);
                errors = errors + 1;
            end
        if (rovf) begin $display("FAIL raw: ring overflow"); errors = errors + 1; end
        $display("  raw: %0d bytes in order", nb);
        raw = 1'b0; slow = 0;
`endif
        if (errors == 0) $display("PASS tb_la_change");
        else             $display("FAIL tb_la_change: %0d errors", errors);
        $finish;
    end
    initial begin #50000000 $display("FAIL tb_la_change: timeout"); $finish; end
endmodule
