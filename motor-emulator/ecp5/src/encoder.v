// encoder.v — rotor-position encoder emulation from the motor model's angle (one clock, no CDC).
//
// Mechanical angle. The model integrates the electrical angle theta (32 bits per electrical
// revolution). The encoder counts electrical revolutions modulo the pole pairs (erev, from the
// wrap of theta between ticks) and divides once per tick:
//   mech  = floor((erev x 2^16 + theta[31:16]) / POLES)        (16 bits per mechanical revolution)
//   angle = (DIR ? -mech : mech) + OFS                          (ENC_ANGLE; SPI and ABZ use it)
// so POLES x mech tracks theta exactly, with no drift however long it runs. theta must move less
// than 1/4 electrical revolution per tick (1/4 rev per 5 us is 3M eRPM).
//
// ABZ. The generator keeps its own angle g (16 bits) and chases the angle in steps of 2 LSB:
// each tick it takes the distance angle - g (modular, so the short way round for free) and
// spreads those steps evenly over the next PWM period (Bresenham against the period, at most one
// step per clk: 72M LSB/s, 66k RPM mechanical). So the outputs lag the angle by one tick and the
// edges come at an even rate. Each step adds +-2 x CPR to r = g x CPR mod 2^16; the carry or
// borrow steps the count, so count = floor(g x CPR / 2^16) exactly (CPR <= 32764). Quadrature
// states 0-3 = A/B 00, 10, 11, 01 (A leads B counting up); Z is high for the one state at count 0
// (A = B = 0), the AS5047P and MA730 default index. CPR is used as a multiple of 4 (minimum 4).
// Writing ENC_CPR restarts the generator at g = 0, count 0: it then walks to the angle at up to
// one PWM period of steps per tick.
//
// SPI slave, oversampled at 36 MHz. The next bit goes on MISO 2-3 clk after the DUT's sampling
// edge, and the first bit is on MISO while CS is still high, so the limit is one SCK period
// >= ~110 ns (SCK <= 8 MHz). Profiles (ENC_CTRL[3:2]):
//   0 AS5047P, 1 AS5048A: mode 1, 16-bit frames, even parity, the response in the next frame;
//     error flags (parity, invalid command, framing) with EF sticky until the error register
//     (0x0001) is read; a write is a command frame then a data frame (MISO: the old content,
//     then the new content in the following frame).
//   2 MA730: mode 0 or 3; every frame returns the 16-bit angle (partial reads fine); 010aaaaa
//     xxxxxxxx reads register a, 100aaaaa vvvvvvvv writes it; the value comes back on MISO[15:8]
//     in the next frame.
// Register file (64 x 16, block RAM; BenchPod reads and writes it through ENC_RF_ADDR/RF_DATA):
// 0x00-0x1F the MA730's registers, 0x20 + a the AS504x registers at 0x0000-0x001F, 0x3C-0x3F the
// AS504x registers at 0x3FFC-0x3FFF (DIAAGC / MAG / diagnostics: what BenchPod puts there is what
// the DUT reads, e.g. a magnet fault). The angle registers are live. Writes by the DUT are stored
// and read back but change nothing (zero position, direction, resolution): BenchPod sets
// ENC_OFS, DIR and ENC_CPR to match. No OTP programming.
`default_nettype none
module encoder (
    input  wire        clk,
    input  wire        rst,
    input  wire        tick,            // PWM period start (the model's tick)
    input  wire        reset_state,     // MODEL_RESET: theta went to 0
    input  wire [31:0] theta,
    input  wire [9:0]  period,          // clocks per tick
    // BenchPod writes (registered in top, held until the next write):
    // 0 CTRL, 1 CPR, 2 POLES, 3 OFS, 4 RF_ADDR, 5 RF_DATA
    input  wire        pw_en,
    input  wire [2:0]  pw_addr,
    input  wire [15:0] pw_data,
    // DUT side (synchronised here)
    input  wire        cs_in, sck_in, mosi_in,
    // fault injection: [0] angle frozen (a stuck sensor: SPI and ABZ hold), [1] ABZ outputs dead
    // (low), [2] MISO stuck low
    input  wire [2:0]  fault,
    output reg         enc_a, enc_b, enc_z,
    output wire        enc_miso,        // straight from the shift register's flop
    // readback
    output reg  [4:0]  ctrl,            // [0] ABZ on, [1] SPI on, [3:2] profile, [4] DIR
    output reg  [15:0] cpr,
    output reg  [5:0]  poles,
    output reg  [15:0] ofs,
    output reg  [5:0]  rf_addr,
    output reg  [15:0] rf_link_q,       // rf[rf_addr]
    output reg  [15:0] angle,
    output reg  [15:0] count,           // ABZ count (0 .. CPR - 1)
    output wire [15:0] status           // [15:8] SPI frames, [2:0] AS504x error flags
);
    wire [1:0]  prof    = ctrl[3:2];
    wire        prof_ma = (prof == 2'd2);
    wire [5:0]  ppe     = (poles == 6'd0) ? 6'd1 : poles;

    // ------------------------------------------------------------------ register file (BRAM)
    reg  [15:0] rf [0:63];
    integer k;
    initial begin
        for (k = 0; k < 64; k = k + 1) rf[k] = 16'h0000;
        rf[6'h04] = 16'h00C0; rf[6'h05] = 16'h00FF; rf[6'h06] = 16'h001C;   // MA730 factory values
        rf[6'h38] = 16'h0001;                                              // AS5047P SETTINGS1
        rf[6'h3C] = 16'h0180;                                              // AS5047P DIAAGC: LF, AGC 0x80
    end
    wire        l_we = pw_en && (pw_addr == 3'd5);
    reg         s_we;
    reg  [5:0]  s_idx;
    reg  [15:0] s_wd;
    reg  [15:0] rf_q;
    reg  [1:0]  fs;                     // SPI frame processing: 0 idle, 1 write, 2 read, 3 respond
    wire        rf_we = l_we | (s_we && fs == 2'd1);
    wire [5:0]  rf_wa = l_we ? rf_addr : s_idx;
    wire [15:0] rf_wd = l_we ? pw_data : s_wd;
    wire [5:0]  rf_ra = (fs == 2'd2) ? s_idx : rf_addr;
    always @(posedge clk) begin
        if (rf_we) rf[rf_wa] <= rf_wd;
        rf_q <= rf[rf_ra];
    end

    // ------------------------------------------------------------------ angle, once per tick
    reg  [3:0]  es;                     // 0 idle, 1 divide, 2 angle, 3-8 ABZ target
    reg  [1:0]  th_prev;
    reg  [5:0]  erev;
    reg  [21:0] dnum;                   // dividend, the quotient shifts in from the right
    reg  [5:0]  drem;
    reg  [4:0]  dcnt;
    wire [6:0]  drs = {drem, dnum[21]};
    wire        dge = (drs >= {1'b0, ppe});
    reg  [5:0]  erev_n;
    always @(*) begin
        erev_n = (erev >= ppe) ? 6'd0 : erev;
        if (th_prev == 2'b11 && theta[31:30] == 2'b00)      erev_n = (erev_n == ppe - 6'd1) ? 6'd0 : erev_n + 6'd1;
        else if (th_prev == 2'b00 && theta[31:30] == 2'b11) erev_n = (erev_n == 6'd0) ? ppe - 6'd1 : erev_n - 6'd1;
    end
    reg  [15:0] g;                      // the generator's angle
    reg  [15:0] gap;
    reg         up;
    reg  [9:0]  dd;
    reg  signed [10:0] dmt;
    reg         ld;
    wire [14:0] steps = gap[15] ? ~gap[15:1] : gap[15:1];      // |distance| / 2 (one's complement:
                                                                   // the next tick takes up the slack)

    always @(posedge clk) begin
        ld <= 1'b0;
        if (rst || reset_state) begin
            es <= 4'd0; erev <= 6'd0; th_prev <= 2'b00;
            if (rst) angle <= 16'd0;
        end else case (es)
            4'd0: if (tick) begin
                erev <= erev_n; th_prev <= theta[31:30];
                dnum <= {erev_n, theta[31:16]}; drem <= 6'd0; dcnt <= 5'd21; es <= 4'd1;
            end
            4'd1: begin                                      // restoring divide, 22 steps
                drem <= dge ? drs[5:0] - ppe : drs[5:0];
                dnum <= {dnum[20:0], dge};
                dcnt <= dcnt - 5'd1;
                if (dcnt == 5'd0) es <= 4'd2;
            end
            4'd2: begin if (!fault[0]) angle <= (ctrl[4] ? ~dnum[15:0] : dnum[15:0]) + ofs + {15'd0, ctrl[4]}; es <= 4'd3; end
            4'd3: begin gap <= angle - g; es <= 4'd5; end
            4'd5: begin up <= ~gap[15]; dd <= (steps > {5'd0, period}) ? period : steps[9:0]; es <= 4'd6; end
            default: begin dmt <= $signed({1'b0, dd}) - $signed({1'b0, period}); ld <= 1'b1; es <= 4'd0; end
        endcase
    end

    // ------------------------------------------------------------------ ABZ edge generator
    reg  signed [10:0] e, dmt_r;
    reg  [9:0]  dd_r;
    reg         up_r;
    reg  [15:0] r;                      // g x CPR mod 2^16
    wire [15:0] cpr2   = {cpr[14:0], 1'b0};                       // the count per 2-LSB step, x 2^16
    // r +- 2 x CPR in one adder; [16] is the carry going up, NOT the borrow going down
    wire [16:0] r_next = {1'b0, r} + {1'b0, cpr2 ^ {16{~up_r}}} + {16'd0, ~up_r};
    // g wraps exactly when the count wraps (count = floor(g x CPR / 2^16)): the wrap of g sets the
    // count to 0 or CPR - 1, every other carry steps it
    wire [16:0] g_next = {1'b0, g} + (up_r ? 17'h00002 : 17'h1FFFE);
    wire        g_wrap = g_next[16];                            // 65534 -> 0 or 0 -> 65534
    always @(posedge clk) begin
        if (rst || (pw_en && pw_addr == 3'd1)) begin
            g <= 16'd0; r <= 16'd0; count <= 16'd0;
            e <= -11'sd1; dd_r <= 10'd0; dmt_r <= -11'sd1; up_r <= 1'b1;
        end else if (ld) begin
            e <= -$signed({2'b00, period[9:1]}); dd_r <= dd; dmt_r <= dmt; up_r <= up;
        end else if (!e[10]) begin
            e <= e + dmt_r;
            g <= g_next[15:0];
            r <= r_next[15:0];
            if (g_wrap)              count <= up_r ? 16'd0 : {cpr[15:2] - 14'd1, 2'b11};
            else if (r_next[16] == up_r) count <= count + (up_r ? 16'd1 : 16'hFFFF);
        end else
            e <= e + $signed({1'b0, dd_r});
        enc_a <= ctrl[0] & ~fault[1] & (count[1] ^ count[0]);
        enc_b <= ctrl[0] & ~fault[1] & count[1];
        enc_z <= ctrl[0] & ~fault[1] & (count == 16'd0);
    end

    // ------------------------------------------------------------------ SPI slave
    reg  [2:0]  cs_s, sck_s;
    reg  [1:0]  mosi_s;
    always @(posedge clk) begin
        cs_s   <= {cs_s[1:0], cs_in};
        sck_s  <= {sck_s[1:0], sck_in};
        mosi_s <= {mosi_s[0], mosi_in};
    end
    wire cs_lo   = ~cs_s[1];
    wire cs_rise = cs_s[1] & ~cs_s[2];
    // the DUT's sampling edge: falling in mode 1 (AS504x), rising in modes 0 and 3 (MA730)
    wire samp = cs_lo & (prof_ma ? (sck_s[1] & ~sck_s[2]) : (~sck_s[1] & sck_s[2]));

    reg  [15:0] isr, osr, resp;
    reg  [4:0]  bcnt;
    reg         fdone;
    reg         resp_ang;               // MA730: the next frame returns the angle
    reg  [2:0]  err;                    // AS504x: [2] parity, [1] invalid command, [0] framing
    reg  [7:0]  frames;
    reg         wr_pend, s_rd, s_live, s_errfl, s_inv, s_set1;
    reg  [5:0]  wr_idx;
    wire [15:0] load_word = (prof_ma && resp_ang) ? {angle[15:2], 2'b00} : resp;

    // AS504x command decode, straight from the receive register (stable until the next frame)
    wire [13:0] fa     = isr[13:0];
    wire        a_47   = (prof == 2'd0);
    wire        a_hi   = (fa[13:2] == 12'hFFF);                    // 0x3FFC-0x3FFF
    wire        a_wok  = (fa == 14'h0003) || (fa == 14'h0016) || (fa == 14'h0017) ||
                         (a_47 && (fa == 14'h0018 || fa == 14'h0019));
    wire        a_ok   = a_wok || (fa == 14'h0000) || (fa == 14'h0001) || (a_hi && (a_47 || fa[1:0] != 2'b00));
    wire        a_live = (fa == 14'h3FFF) || (a_47 && fa == 14'h3FFE);
    wire [5:0]  a_idx  = a_hi ? {4'b1111, fa[1:0]} : {1'b1, fa[4:0]};
    wire [13:0] r_dat  = s_inv ? 14'd0 : s_live ? angle[15:2] : s_errfl ? {11'd0, err} : rf_q[13:0];
    wire        r_ef   = |err;

    always @(posedge clk) begin
        fdone <= 1'b0;
        if (rst) begin
            bcnt <= 5'd0; resp <= 16'h0000; resp_ang <= 1'b1; err <= 3'd0; frames <= 8'd0;
            fs <= 2'd0; wr_pend <= 1'b0; osr <= 16'h0000; s_we <= 1'b0;
        end else begin
            // ---- shifting
            if (!cs_lo) begin
                bcnt <= 5'd0;
                osr  <= ctrl[1] ? load_word : 16'h0000;   // the MSB is on MISO before CS falls
                if (cs_rise && !prof_ma && bcnt != 5'd0 && bcnt != 5'd16) begin
                    err <= err | 3'b001; wr_pend <= 1'b0;
                end
            end else if (samp) begin
                isr <= {isr[14:0], mosi_s[1]};
                osr <= {osr[14:0], 1'b0};
                if (bcnt != 5'd17) bcnt <= bcnt + 5'd1;
                if (bcnt == 5'd15) fdone <= 1'b1;
                if (bcnt == 5'd0 && prof_ma) resp_ang <= 1'b1;   // a register reply has gone out
            end

            // ---- frame processing
            case (fs)
                2'd0: if (fdone) begin
                    frames <= frames + 8'd1;
                    s_we <= 1'b0; s_rd <= 1'b0; s_live <= 1'b0; s_errfl <= 1'b0; s_inv <= 1'b0;
                    fs <= 2'd2;
                    if (prof_ma) begin
                        s_idx <= {1'b0, isr[12:8]};
                        s_wd  <= {8'h00, isr[7:0]};
                        if (isr[15:13] == 3'b100) begin s_we <= 1'b1; s_rd <= 1'b1; fs <= 2'd1; end
                        else if (isr[15:13] == 3'b010) s_rd <= 1'b1;
                    end else if (wr_pend) begin              // AS504x data frame
                        wr_pend <= 1'b0; s_idx <= wr_idx;
                        s_wd <= s_set1 ? {8'h00, isr[7:2], 2'b01} : {2'b00, isr[13:0]};
                        if (^isr || isr[14]) begin err <= err | 3'b100; s_inv <= 1'b1; end
                        else begin s_we <= 1'b1; fs <= 2'd1; end
                    end else if (^isr) begin                 // parity error: no command
                        err <= err | 3'b100; s_inv <= 1'b1;
                    end else if (!a_ok || (!isr[14] && !a_wok)) begin
                        err <= err | 3'b010; s_inv <= 1'b1;
                    end else begin
                        s_idx <= a_idx; s_live <= a_live; s_errfl <= (fa == 14'h0001);
                        if (!isr[14]) begin wr_pend <= 1'b1; wr_idx <= a_idx; s_set1 <= (fa == 14'h0018); end
                    end
                end
                2'd1: if (!l_we) fs <= 2'd2;                 // the write (a link write goes first)
                2'd2: fs <= 2'd3;                            // read address on the RAM
                default: begin                               // rf_q holds the register
                    fs <= 2'd0;
                    if (prof_ma) begin
                        if (s_rd) begin resp <= {rf_q[7:0], 8'h00}; resp_ang <= 1'b0; end
                    end else begin
                        resp <= {^{r_ef, r_dat}, r_ef, r_dat};
                        if (s_errfl) err <= 3'd0;
                    end
                end
            endcase
        end
    end
    assign enc_miso = osr[15] & ~fault[2];
    assign status = {frames, 5'd0, err};

    // ------------------------------------------------------------------ BenchPod side
    reg rd_link;                        // rf_q holds the link's read
    always @(posedge clk) begin
        rd_link <= (fs != 2'd2);
        if (rd_link) rf_link_q <= rf_q;
        if (rst) begin
            ctrl <= 5'd0; cpr <= 16'd4000; poles <= 6'd1; ofs <= 16'd0; rf_addr <= 6'd0;
        end else if (pw_en) case (pw_addr)
            3'd0: ctrl <= pw_data[4:0];
            3'd1: cpr <= (pw_data[14:2] == 13'd0) ? 16'd4 : {1'b0, pw_data[14:2], 2'b00};   // 4 .. 32764
            3'd2: poles <= pw_data[5:0];
            3'd3: ofs <= pw_data;
            3'd4: rf_addr <= pw_data[5:0];
            default: ;
        endcase
    end
endmodule
`default_nettype wire
