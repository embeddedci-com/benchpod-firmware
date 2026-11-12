// ============================================================================
// i2c_emu.v: up to 4 emulated I2C targets in ONE time-multiplexed engine (v54, emulation image).
//
// Sensor-agnostic, like the v24..v53 i2c_target it replaces: each target serves bytes from (and
// captures DUT writes into) a 256-byte register image that the STM32 fills over SPI; all sensor
// knowledge stays in the firmware.  What is new is that four targets share one copy of the logic.
//
// HOW IT IS TIME-MULTIPLEXED
//   A slot counter runs 0,1,2,3,4,0,...  Slots 0..3 are the four targets, slot 4 serves the SPI
//   side (cmd_dispatch).  Every target's whole state (its configuration and its bus state machine)
//   is a 112-bit CONTEXT word kept in seven 256x16 block RAMs, one address per target.  For the
//   target of a slot the engine reads the context (stage A), computes the next state from it and
//   from the sampled bus (stage B, one clock), and writes it back (end of stage B).  So each
//   target is sampled every 5 clocks, 4.8 MHz at the 24 MHz clk: >= 2.8 samples of the shortest
//   SCL high time at 400 kHz (0.6 us).  1 MHz (Fm+) works only with SCL high >= 0.42 us.
//   The configuration and the bus state share the context words: an SPI configuration write (in
//   slot 4) writes the configuration bits only, an engine write the state bits only (the block
//   RAM's bit write mask), so neither ever overwrites the other.
//
//   Register images: one 1024x16 memory, 256 words per target, each word = {attribute, data}.
//   In stage B the engine reads the word at the target's NEXT pointer; stage C copies it into a
//   one-word-per-target CACHE (one more block RAM) that the next stage B of that target reads.
//   The cache is rewritten every round, so a register the STM32 reloads is served within 5 clk.
//   Slot 4 uses the same ports for the STM32's loads and read-back, so they never collide.
//
//   Wide per-target state that only changes at events (counters, times) is in the context too, and
//   time is a shared free-running microsecond counter (us_now): a conversion stores its END time
//   and ends when us_now reaches it (every us_now value lasts 24 clk, so every slot sees it).
//
// PER-REGISTER ATTRIBUTES (the high byte of each image word, loaded with I2C_LOAD_ATTRS)
//   bit0 TRIG  a DUT data write to this register starts a conversion
//   bit1 CMD   the DUT setting the pointer to this register starts a conversion (a command, e.g.
//              Sensirion: the command byte is the pointer, its answer the registers from there)
//   bit2 BUSY  while a conversion runs, reads of this register OR in the target's busy mask
//   bit3 FIFO  reads pop the shared FIFO instead (and the pointer does not advance past it)
//   bit4 CLR   reading this register clears a data-ready LEVEL
//   bit5 RO    DUT writes to this register are not stored (they are still counted and recorded)
//   bit7:6     TIME: which of the target's four conversion times (I2C_TIMES) a TRIG/CMD starts
//
// PER-TARGET CONFIGURATION (I2C_CONFIG)
//   enable, 7-bit address, SDA and SCL LA channels (any; targets may share a bus), 8- or 16-bit
//   register addressing (16-bit: the image is the 256-byte window whose high address byte is
//   `page`; outside it reads 0x00 and writes are ignored; the pointer wraps inside the window),
//   while_busy = flag (reads go on, BUSY registers OR in `mask`) / nack (a read address is not
//   acknowledged while converting) / stretch (the read address is acknowledged, then SCL is held
//   low until the conversion ends: bounded by the conversion time, at most 65.5 ms), the faults
//   mute (the address is not acknowledged) and bus_stuck (SDA held low until the firmware clears
//   it), and the data-ready output (off / pulse / level, polarity, on conversion end).
//
// I2C electricals as before: SDA and SCL are only ever pulled low or released (open drain, the bus
// needs pull-ups); SDA changes only while SCL is low.  `pull` is the OR of every target's pull-low
// requests per LA channel, for la_bank.  The SCL-silence watchdog (a transaction with no SCL edge
// for 7..8 ms releases the bus) is kept, and does not run while the target stretches SCL itself.
// ============================================================================
module i2c_emu #(
    parameter CLK_MHZ = 24,
    parameter N       = 14
)(
    input  wire          clk,
    input  wire          rst,
    input  wire [N-1:0]  la_lvl,        // LA levels through a 2-flop synchroniser
    output wire [N-1:0]  pull,          // per LA channel: some target pulls it low
    output reg  [3:0]    drdy_pin,      // per target: the data-ready pin level (polarity applied)
    output reg  [15:0]   us_now,        // free-running microsecond counter (shared with the SPI target)

    // ---- SPI side (cmd_dispatch).  Strobes; their payload wires hold >= 16 clk. ----
    input  wire [1:0]    sel_t,         // the target the SPI-side commands address
    input  wire          cfg_stb,       // write sel_t's configuration ...
    input  wire          cfg_en_only,   // ... only its enable bit (I2C_DISABLE)
    input  wire [40:0]   cfg_data,      // see CFG below
    input  wire          reg_we,        // load one image byte of sel_t at reg_addr
    input  wire          reg_attr,      // 1 = the attribute byte (load and read-back), 0 = data
    input  wire [7:0]    reg_addr,      // load address (sampled with reg_we) and read-back address
    input  wire [7:0]    reg_wdata,
    output reg  [7:0]    reg_rdata,     // image byte of sel_t at reg_addr, refreshed every 5 clk
    input  wire          tt_we,         // set conversion time tt_idx of sel_t
    input  wire [1:0]    tt_idx,
    input  wire [15:0]   tt_val,        // microseconds
    input  wire          fire_stb,      // firmware data-ready command for sel_t ...
    input  wire          fire_set,      // ... 1 = assert (a pulse in pulse mode), 0 = deassert
    input  wire [7:0]    dr_width,      // data-ready pulse width in us (all targets)
    output reg  [31:0]   status,        // sel_t, refreshed every 5 clk: {flags, wr_count, last addr, last value}

    // ---- shared FIFO (reads of a FIFO register pop it) ----
    output reg           fifo_pop,
    input  wire [7:0]    fifo_head,
    input  wire          fifo_empty
);
    // ------------------------------------------------------------------------------------------
    // timebase: us_now advances every CLK_MHZ clk
    // ------------------------------------------------------------------------------------------
    reg [4:0] us_div;
    always @(posedge clk) begin
        if (rst) begin us_div <= 5'd0; us_now <= 16'd0; end
        else if (us_div == CLK_MHZ - 1) begin us_div <= 5'd0; us_now <= us_now + 16'd1; end
        else us_div <= us_div + 5'd1;
    end

    // ------------------------------------------------------------------------------------------
    // slots and pipeline stages
    // ------------------------------------------------------------------------------------------
    reg [2:0] slot = 3'd0, sB = 3'd4, sC = 3'd4;   // stage A, B, C slot
    always @(posedge clk) begin
        if (rst) begin slot <= 3'd0; sB <= 3'd4; sC <= 3'd4; end
        else begin slot <= (slot == 3'd4) ? 3'd0 : slot + 3'd1; sB <= slot; sC <= sB; end
    end
    wire svcA = slot[2], svcB = sB[2], svcC = sC[2];   // slot 4 = the SPI side
    wire [1:0] tB = sB[1:0], tC = sC[1:0];

    // ------------------------------------------------------------------------------------------
    // context layout (112 bits): configuration [111:71], state [70:0]
    // ------------------------------------------------------------------------------------------
    localparam CW = 112, CFG_LO = 71;
    wire [CW-1:0] ctx;                 // stage B: the context of target tB (or sel_t in slot 4)
    // configuration (cfg_data is laid out the same way, MSB first)
    wire       c_en    = ctx[111];
    wire [6:0] c_addr  = ctx[110:104];
    wire       c_a16   = ctx[103];
    wire [1:0] c_wb    = ctx[102:101];  // while_busy: 0 flag, 1 nack, 2 stretch
    wire       c_mute  = ctx[100];
    wire       c_stuck = ctx[99];
    wire [7:0] c_page  = ctx[98:91];
    wire [7:0] c_mask  = ctx[90:83];
    wire [3:0] c_sda   = ctx[82:79];
    wire [3:0] c_scl   = ctx[78:75];
    wire [1:0] c_dmode = ctx[74:73];   // data ready: 0 off, 1 pulse, 2 level
    wire       c_dpol  = ctx[72];      // 1 = active high
    wire       c_dconv = ctx[71];      // assert at the end of each conversion
    // state
    wire [2:0] s_st    = ctx[70:68];
    wire [2:0] s_bc    = ctx[67:65];
    wire [7:0] s_sr    = ctx[64:57];
    wire       s_rw    = ctx[56];
    wire       s_ak    = ctx[55];
    wire       s_rf    = ctx[54];
    wire [1:0] s_pst   = ctx[53:52];   // pointer bytes: 0 none, 1 high byte seen (16-bit), 2 loaded
    wire [7:0] s_ptr   = ctx[51:44];
    wire       s_inw   = ctx[43];      // the pointer is inside the image window
    wire       s_sclp  = ctx[42];
    wire       s_sdap  = ctx[41];
    wire       s_sdal  = ctx[40];      // pulling SDA low
    wire       s_scll  = ctx[39];      // pulling SCL low (stretch)
    wire       s_cact  = ctx[38];      // a conversion is running
    wire [15:0] s_cend = ctx[37:22];   // ... until us_now reaches this
    wire [2:0] s_wd    = ctx[21:19];   // SCL-silence watchdog, in us_now[10] toggles
    wire       s_wdb   = ctx[18];
    wire       s_cchk  = ctx[17];      // the pointer just moved: check its CMD attribute
    wire       s_drl   = ctx[16];      // data ready asserted
    wire [7:0] s_dend  = ctx[15:8];    // pulse ends when us_now[7:0] reaches this
    wire [7:0] s_wrc   = ctx[7:0];     // DUT write counter

    localparam ST_IDLE = 3'd0, ST_ADDR = 3'd1, ST_ACK = 3'd2, ST_WRITE = 3'd3,
               ST_STRETCH = 3'd4, ST_READ = 3'd5, ST_RDACK = 3'd6, ST_IGNORE = 3'd7;

    // ------------------------------------------------------------------------------------------
    // SPI-side requests, served in slot 4 (each strobe's payload holds >= 16 clk, a slot 4
    // comes every 5)
    // ------------------------------------------------------------------------------------------
    reg       cfg_pend = 1'b0, cfg_eo = 1'b0;
    reg       rw_pend = 1'b0;
    reg [7:0] rw_addr;
    reg       tt_pend = 1'b0;
    reg [3:0] fire_on = 4'd0, fire_off = 4'd0;   // per target, consumed in its stage B

    // ------------------------------------------------------------------------------------------
    // conversion start: a TRIG/CMD sets pend[t] (with its time index); slot 4 reads that time from
    // the table and the target's next stage B starts the conversion with it.  Pending counts as busy.
    // ------------------------------------------------------------------------------------------
    reg [3:0] pend = 4'd0;
    reg [1:0] tidx [0:3];
    reg [1:0] pend_t;                  // stage A of slot 4: the lowest pending target
    wire [1:0] tidx_sel = tidx[pend_t];
    always @(*) begin
        casez (pend)
            4'b???1: pend_t = 2'd0;
            4'b??10: pend_t = 2'd1;
            4'b?100: pend_t = 2'd2;
            default: pend_t = 2'd3;
        endcase
    end
    reg        tv_v = 1'b0;            // tv holds the time of target tv_t
    reg [1:0]  tv_t, pt_A;
    reg        pv_A;
    reg [15:0] tv;
    always @(posedge clk) begin
        if (svcA) begin pt_A <= pend_t; pv_A <= |pend; end
    end

    // ------------------------------------------------------------------------------------------
    // block RAMs
    // ------------------------------------------------------------------------------------------
    // context: 7 x 256x16, address = target
    wire [1:0]   ctx_ra = svcA ? sel_t : slot[1:0];
    wire [CW-1:0] ctx_wd;              // stage B: next state (state bits) / cfg_data (configuration bits)
    wire         ctx_we = svcB ? cfg_pend : 1'b1;
    wire [1:0]   ctx_wa = svcB ? sel_t : tB;
    // write mask: 1 = keep.  Engine: keep the configuration.  SPI: keep the state (and, for
    // I2C_DISABLE, everything but the enable bit).
    wire [CW-1:0] ctx_mask = svcB ? (cfg_eo ? {1'b0, {(CW-1){1'b1}}} : {{(CW-CFG_LO){1'b0}}, {CFG_LO{1'b1}}})
                                  : {{(CW-CFG_LO){1'b1}}, {CFG_LO{1'b0}}};
    genvar gi;
    generate for (gi = 0; gi < 7; gi = gi + 1) begin : g_ctx
        SB_RAM40_4K #(.READ_MODE(0), .WRITE_MODE(0)) ram (
            .RDATA(ctx[gi*16 +: 16]), .RADDR({9'd0, ctx_ra}), .RCLK(clk), .RCLKE(1'b1), .RE(1'b1),
            .WADDR({9'd0, ctx_wa}), .WCLK(clk), .WCLKE(1'b1), .WE(ctx_we),
            .WDATA(ctx_wd[gi*16 +: 16]), .MASK(ctx_mask[gi*16 +: 16]));
    end endgenerate

    // register images: 1024 x {attr, data}
    reg  [15:0] img [0:1023];
    integer ii;
    initial for (ii = 0; ii < 1024; ii = ii + 1) img[ii] = 16'd0;   // block RAMs configure to 0
    reg  [15:0] img_rd;                // stage C
    reg  [9:0]  img_ra;                // stage B (comb)
    reg         img_we_d, img_we_a;    // stage B (comb): write the data / attribute byte
    reg  [9:0]  img_wa;
    reg  [7:0]  img_wd;
    always @(posedge clk) begin
        if (img_we_d) img[img_wa][7:0]  <= img_wd;
        if (img_we_a) img[img_wa][15:8] <= img_wd;
        img_rd <= img[img_ra];
    end

    // cache (addresses 0..3) + conversion time table (16 + {target, index})
    reg  [15:0] cch [0:255];
    initial for (ii = 0; ii < 256; ii = ii + 1) cch[ii] = 16'd0;
    reg  [15:0] cch_rd;                // stage B: cache word of tB, or a time in slot 4
    always @(posedge clk) begin
        cch_rd <= cch[svcA ? {4'b0001, pend_t, tidx_sel} : {6'd0, slot[1:0]}];
        if (!svcC)        cch[{6'd0, tC}] <= img_rd;
        else if (tt_pend) cch[{4'b0001, sel_t, tt_idx}] <= tt_val;
    end

    // last write per target (address, value), and the SPI target's at address 4 (unused for now)
    reg  [15:0] lw [0:255];
    initial for (ii = 0; ii < 256; ii = ii + 1) lw[ii] = 16'd0;
    reg  [15:0] lw_rd;
    reg         lw_we;  reg [15:0] lw_wd;
    always @(posedge clk) begin
        if (lw_we) lw[{6'd0, tB}] <= lw_wd;
        lw_rd <= lw[{6'd0, sel_t}];
    end

    // ------------------------------------------------------------------------------------------
    // stage B: the bus state machine of target tB
    // ------------------------------------------------------------------------------------------
    wire [7:0] a_data = cch_rd[7:0];
    wire [7:0] a_attr = s_inw ? cch_rd[15:8] : 8'd0;   // no attributes outside the window
    wire a_trig = a_attr[0], a_cmd = a_attr[1], a_busy = a_attr[2], a_fifo = a_attr[3],
         a_clr  = a_attr[4], a_ro  = a_attr[5];
    wire [1:0] a_time = a_attr[7:6];

    wire scl_n = la_lvl[c_scl];
    wire sda_n = la_lvl[c_sda];
    wire rise  = ~s_sclp &  scl_n;
    wire fall  =  s_sclp & ~scl_n;
    // START / STOP: SDA moves while SCL is high in both samples (SCL falling between the two samples
    // is a data change, not a condition)
    wire start_c = s_sclp & scl_n &  s_sdap & ~sda_n;
    wire stop_c  = s_sclp & scl_n & ~s_sdap &  sda_n;
    wire [7:0] byte_in = {s_sr[6:0], sda_n};

    wire busy    = s_cact | pend[tB];
    wire busy_sel = s_cact | pend[sel_t];   // slot 4: ctx is sel_t's
    wire conv_end = s_cact & (us_now == s_cend);
    // the byte a read sends: 0 outside the window, the FIFO head, or the image with the busy overlay
    wire [7:0] rd_byte = !s_inw ? 8'd0 : a_fifo ? (fifo_empty ? 8'd0 : fifo_head)
                                       : (a_data | ((busy & a_busy) ? c_mask : 8'd0));

    reg [2:0] n_st, n_bc;  reg [7:0] n_sr, n_ptr;  reg n_rw, n_ak, n_rf, n_inw, n_sdal, n_scll;
    reg [1:0] n_pst;  reg n_cact;  reg [15:0] n_cend;  reg [2:0] n_wd;  reg n_wdb, n_cchk, n_drl;
    reg [7:0] n_dend, n_wrc;
    reg       n_sclp, n_sdap;
    reg       trig;  reg [1:0] trig_time;   // start a conversion (via pend)
    reg       load_rd;                      // a read byte is loaded this beat (pop / clear-on-read)
    reg       tv_take;

    always @(*) begin
        n_st = s_st;  n_bc = s_bc;  n_sr = s_sr;  n_rw = s_rw;  n_ak = s_ak;  n_rf = s_rf;
        n_pst = s_pst;  n_ptr = s_ptr;  n_inw = s_inw;  n_sdal = s_sdal;  n_scll = s_scll;
        n_cact = s_cact;  n_cend = s_cend;  n_wd = s_wd;  n_wdb = s_wdb;  n_cchk = 1'b0;
        n_drl = s_drl;  n_dend = s_dend;  n_wrc = s_wrc;
        trig = 1'b0;  trig_time = a_time;  load_rd = 1'b0;  tv_take = 1'b0;
        img_we_d = 1'b0;  img_we_a = 1'b0;  lw_we = 1'b0;
        img_wa = {tB, s_ptr};  img_wd = byte_in;  lw_wd = {s_ptr, byte_in};

        // ---- conversions and data ready (independent of the bus) ----
        if (tv_v && tv_t == tB && pend[tB]) begin
            n_cact = 1'b1;  n_cend = us_now + tv;  tv_take = 1'b1;
        end else if (conv_end) begin
            n_cact = 1'b0;
            if (c_dconv && c_dmode != 2'd0) begin n_drl = 1'b1; n_dend = us_now[7:0] + dr_width; end
        end
        if (fire_on[tB])  begin n_drl = 1'b1; n_dend = us_now[7:0] + dr_width; end
        if (fire_off[tB]) n_drl = 1'b0;
        if (s_drl && c_dmode == 2'd1 && us_now[7:0] == s_dend) n_drl = 1'b0;   // pulse over
        // the pointer moved onto a command register
        if (s_cchk && a_cmd) begin trig = 1'b1; trig_time = a_time; end

        // ---- the bus ----
        if (!c_en) begin
            n_st = ST_IDLE;  n_sdal = 1'b0;  n_scll = 1'b0;  n_pst = 2'd0;
        end else if (start_c) begin
            n_st = ST_ADDR;  n_bc = 3'd0;  n_sdal = 1'b0;  n_scll = 1'b0;  n_pst = 2'd0;
        end else if (stop_c) begin
            n_st = ST_IDLE;  n_sdal = 1'b0;  n_scll = 1'b0;  n_pst = 2'd0;
        end else begin
            case (s_st)
                ST_ADDR: if (rise) begin
                    n_sr = byte_in;  n_bc = s_bc + 3'd1;
                    if (s_bc == 3'd7) begin
                        n_rw = sda_n;  n_ak = 1'b0;
                        if (s_sr[6:0] != c_addr || c_mute || (sda_n && busy && c_wb == 2'd1))
                            n_st = ST_IGNORE;          // not us, muted, or a read NACKed while busy
                        else
                            n_st = ST_ACK;
                    end
                end
                ST_ACK: if (fall) begin
                    if (!s_ak) begin
                        n_sdal = 1'b1;  n_ak = 1'b1;   // drive the ACK
                    end else begin
                        n_ak = 1'b0;
                        if (s_rw) begin
                            if (busy && c_wb == 2'd2) begin
                                n_sdal = 1'b0;  n_scll = 1'b1;  n_st = ST_STRETCH;
                            end else begin
                                n_sr = {rd_byte[6:0], 1'b0};  n_sdal = ~rd_byte[7];
                                n_bc = 3'd1;  n_rf = 1'b0;  n_st = ST_READ;  load_rd = 1'b1;
                            end
                        end else begin
                            n_sdal = 1'b0;  n_st = ST_WRITE;
                        end
                    end
                end
                // SCL held low: wait for the conversion, then set up the MSB, then release SCL
                ST_STRETCH: begin
                    if (!s_ak) begin
                        if (!busy) begin
                            n_sr = {rd_byte[6:0], 1'b0};  n_sdal = ~rd_byte[7];
                            n_bc = 3'd1;  n_rf = 1'b0;  n_ak = 1'b1;  load_rd = 1'b1;
                        end
                    end else begin
                        n_scll = 1'b0;  n_ak = 1'b0;  n_st = ST_READ;
                    end
                end
                ST_WRITE: if (rise) begin
                    n_sr = byte_in;  n_bc = s_bc + 3'd1;
                    if (s_bc == 3'd7) begin
                        n_ak = 1'b0;  n_st = ST_ACK;
                        if (s_pst == 2'd0 && c_a16) begin
                            n_pst = 2'd1;  n_inw = (byte_in == c_page);
                        end else if (s_pst != 2'd2) begin
                            n_ptr = byte_in;  n_pst = 2'd2;  n_cchk = 1'b1;
                            if (!c_a16) n_inw = 1'b1;
                        end else begin
                            // a data byte: store it (inside the window, not read-only), count it
                            img_we_d = s_inw & ~a_ro;
                            lw_we    = s_inw;
                            if (s_inw) n_wrc = s_wrc + 8'd1;
                            if (a_trig) begin trig = 1'b1; trig_time = a_time; end
                            if (!a_fifo) n_ptr = s_ptr + 8'd1;
                        end
                    end
                end
                ST_READ: if (fall) begin
                    n_bc = s_bc + 3'd1;
                    if (s_rf) begin
                        n_sr = {rd_byte[6:0], 1'b0};  n_sdal = ~rd_byte[7];  n_rf = 1'b0;  load_rd = 1'b1;
                    end else begin
                        n_sdal = ~s_sr[7];  n_sr = {s_sr[6:0], 1'b0};
                        if (s_bc == 3'd7) begin n_ak = 1'b0; n_st = ST_RDACK; end
                    end
                end
                ST_RDACK: begin
                    if (fall && !s_ak) begin n_sdal = 1'b0; n_ak = 1'b1; end
                    if (rise && s_ak) begin
                        n_ak = 1'b0;
                        if (!sda_n) begin            // ACK: the master wants the next byte
                            if (!a_fifo) n_ptr = s_ptr + 8'd1;
                            n_rf = 1'b1;  n_st = ST_READ;
                        end else n_st = ST_IGNORE;   // NACK: done
                    end
                end
                default: n_sdal = 1'b0;              // IDLE / IGNORE: wait for START / STOP
            endcase
        end
        if (load_rd && a_clr) n_drl = 1'b0;

        // ---- SCL-silence watchdog (not while idle, ignored, or stretching SCL ourselves) ----
        n_wdb = us_now[10];
        if (rise || fall || !c_en || s_st == ST_IDLE || s_st == ST_IGNORE || s_st == ST_STRETCH)
            n_wd = 3'd0;
        else if (us_now[10] != s_wdb && s_wd != 3'd7)
            n_wd = s_wd + 3'd1;
        if (s_wd == 3'd7) begin
            n_st = ST_IDLE;  n_sdal = 1'b0;  n_scll = 1'b0;  n_pst = 2'd0;  n_wd = 3'd0;
        end

        n_sclp = scl_n;  n_sdap = sda_n;

        // ---- slot 4: the SPI side's image access ----
        img_ra = {tB, n_ptr};
        if (svcB) begin
            img_we_d = rw_pend & ~reg_attr;  img_we_a = rw_pend & reg_attr;
            img_wa   = {sel_t, rw_addr};     img_wd   = reg_wdata;
            img_ra   = {sel_t, reg_addr};
            lw_we    = 1'b0;  trig = 1'b0;   tv_take  = 1'b0;   load_rd = 1'b0;
        end
    end

    assign ctx_wd = {cfg_data,
                     n_st, n_bc, n_sr, n_rw, n_ak, n_rf, n_pst, n_ptr, n_inw, n_sclp, n_sdap,
                     n_sdal, n_scll, n_cact, n_cend, n_wd, n_wdb, n_cchk, n_drl, n_dend, n_wrc};

    // ------------------------------------------------------------------------------------------
    // registered side effects of stage B
    // ------------------------------------------------------------------------------------------
    reg [N-1:0] pv [0:3];              // each target's pull-low requests, per LA channel
    function [N-1:0] onehot(input [3:0] ch);
        onehot = ({{(N-1){1'b0}}, 1'b1} << ch);
    endfunction
    wire [N-1:0] pull_next = (onehot(c_sda) & {N{c_en & (n_sdal | c_stuck)}}) |
                             (onehot(c_scl) & {N{c_en & n_scll}});
    assign pull = pv[0] | pv[1] | pv[2] | pv[3];

    integer k;
    always @(posedge clk) begin
        fifo_pop <= 1'b0;
        if (rst) begin
            cfg_pend <= 1'b0;  rw_pend <= 1'b0;  tt_pend <= 1'b0;
            fire_on <= 4'd0;  fire_off <= 4'd0;  pend <= 4'd0;  tv_v <= 1'b0;
            for (k = 0; k < 4; k = k + 1) pv[k] <= {N{1'b0}};
            drdy_pin <= 4'd0;
        end else begin
            // SPI-side requests: set by the strobes, cleared once slot 4 served them
            if (cfg_stb) begin cfg_pend <= 1'b1; cfg_eo <= cfg_en_only; end
            else if (svcB) cfg_pend <= 1'b0;
            if (reg_we) begin rw_pend <= 1'b1; rw_addr <= reg_addr; end
            else if (svcB) rw_pend <= 1'b0;
            if (tt_we) tt_pend <= 1'b1;
            else if (svcC) tt_pend <= 1'b0;
            if (fire_stb) begin
                fire_on[sel_t]  <= fire_set;
                fire_off[sel_t] <= ~fire_set;
            end

            // slot 4: latch the pending conversion's time; the read-back byte; the status snapshot
            if (svcB) begin
                tv <= cch_rd;  tv_t <= pt_A;  tv_v <= pv_A;
                status <= {c_en, busy_sel, s_drl, s_st[2:0], s_inw, 1'b0, s_wrc, lw_rd};
            end
            if (svcC) reg_rdata <= reg_attr ? img_rd[15:8] : img_rd[7:0];

            // a target's stage B
            if (!svcB) begin
                pv[tB] <= pull_next;
                drdy_pin[tB] <= n_drl ~^ c_dpol;   // active high: the level; active low: inverted
                if (!fire_stb || sel_t != tB) begin fire_on[tB] <= 1'b0; fire_off[tB] <= 1'b0; end
                if (tv_take) tv_v <= 1'b0;
                if (trig) begin pend[tB] <= 1'b1; tidx[tB] <= trig_time; end
                else if (tv_take) pend[tB] <= 1'b0;
                if (load_rd && a_fifo && !fifo_empty) fifo_pop <= 1'b1;
            end
        end
    end
endmodule
