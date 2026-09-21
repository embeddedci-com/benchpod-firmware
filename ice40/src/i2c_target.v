// ============================================================================
// i2c_target.v — generic I2C target (slave) for emulating I2C sensors.
//
// Sensor-agnostic by design: it presents a configurable 7-bit address and
// serves bytes from / captures bytes into an external 256-byte register file
// (i2c_regfile.v).  All sensor-specific behaviour (chip IDs, calibration,
// measurement math) lives on the RP2350B, which fills the register image over
// SPI.  This mirrors the software reference in rp2350-hil-adapter
// (bmp280_simulator.cpp::on_i2c_slave_event): register-pointer model with
// auto-increment, first write byte = pointer, subsequent writes stored.
//
// Lightweight "conversion handshake" (also generic, config-driven): when the
// DUT writes the configured trigger register, a down-counter runs for the
// configured time; while it runs, reads of the configured "busy" register
// OR-in a configured mask so the DUT observes a realistic measuring/busy bit
// (e.g. BMP280 forced-mode: trig 0xF4, busy reg 0xF3, mask 0x08).  The RP still
// owns the actual data bytes.
//
// I2C electricals (open-drain):
//   - SCL is INPUT only (we are a target; the master clocks).  No clock
//     stretching.
//   - SDA is driven LOW only via `sda_drive_low` (la_bank turns this into
//     oe=1/out=0); a logic '1' is the released (high-Z, pulled-up) state.  We
//     NEVER drive SDA high.  We only ever change SDA while SCL is low, so our
//     own SDA edges can never be mistaken for START/STOP.
//
// SCL/SDA are async to the 24 MHz clock and are double-flopped (same CDC idiom
// as spi_slave.v).  At ≤400 kHz the bus is heavily oversampled.
// ============================================================================

module i2c_target #(
    parameter CLK_MHZ = 24    // system clock in MHz → microsecond prescaler
)(
    input  wire        clk,
    input  wire        rst,

    // ---- configuration (latched on cfg_stb; cleared on disable_stb) ----
    input  wire        cfg_stb,
    input  wire [6:0]  cfg_addr7,
    input  wire [3:0]  cfg_sda_ch,
    input  wire [3:0]  cfg_scl_ch,
    input  wire        cfg_enable,     // flags bit0 at config time
    input  wire [7:0]  cfg_trig_reg,
    input  wire [7:0]  cfg_busy_reg,
    input  wire [7:0]  cfg_busy_mask,
    input  wire [15:0] cfg_conv_us,    // conversion time in microseconds (0 = none)
    input  wire        disable_stb,

    // latched channel assignment (top uses these to route the bus + la_bank)
    output reg  [3:0]  sda_ch,
    output reg  [3:0]  scl_ch,

    // ---- I2C bus (from la_bank readback / to la_bank open-drain driver) ----
    input  wire        scl_in,
    input  wire        sda_in,
    output reg         sda_drive_low,  // 1 → pull SDA low; 0 → release (high-Z)

    // ---- register file port ----
    output reg         reg_we,
    output reg  [7:0]  reg_waddr,
    output reg  [7:0]  reg_wdata,
    output wire [7:0]  reg_raddr,      // = register pointer
    input  wire [7:0]  reg_rdata,

    // ---- status (to cmd_dispatch) ----
    output reg         armed,
    output reg  [15:0] xfer_count,
    output reg  [15:0] wr_count,
    output reg  [7:0]  last_wr_addr,
    output reg  [7:0]  last_wr_val
);

    // ---- latched configuration ----
    reg [6:0]  addr7;
    reg [7:0]  trig_reg;
    reg [7:0]  busy_reg;
    reg [7:0]  busy_mask;
    reg [15:0] conv_us;

    // ---- SCL/SDA synchronisers ----
    reg scl0, scl1, scl2;
    reg sda0, sda1, sda2;
    always @(posedge clk) begin
        if (rst) begin
            scl0 <= 1'b1; scl1 <= 1'b1; scl2 <= 1'b1;
            sda0 <= 1'b1; sda1 <= 1'b1; sda2 <= 1'b1;
        end else begin
            scl0 <= scl_in; scl1 <= scl0; scl2 <= scl1;
            sda0 <= sda_in; sda1 <= sda0; sda2 <= sda1;
        end
    end

    wire scl_rise = scl1 & ~scl2;
    wire scl_fall = ~scl1 & scl2;
    wire scl_high = scl1;
    wire sda_now  = sda1;
    wire sda_prev = sda2;

    // START = SDA↓ while SCL high; STOP = SDA↑ while SCL high.
    wire start_cond = scl_high &  sda_prev & ~sda_now;
    wire stop_cond  = scl_high & ~sda_prev &  sda_now;

    // ---- conversion / busy timer (generic, config-driven) ----
    reg        conv_trig;          // 1-cycle pulse from FSM on trig_reg write
    reg [15:0] conv_left;          // microseconds remaining
    reg [4:0]  conv_presc;         // ÷CLK_MHZ → 1 µs ticks
    wire       conv_busy = (conv_left != 16'd0);
    localparam [4:0] PRESC_MAX = CLK_MHZ[4:0] - 5'd1;

    // ---- bus-recovery watchdog ----
    // If the DUT master dies mid-transaction while we are holding SDA low (an ACK
    // or a read data bit), SDA would stay wedged low until a STOP or firmware
    // disable — hanging the DUT's bus.  Guard against it: count clocks since the
    // last SCL edge while a transaction is ACTIVE (not S_IDLE/S_IGNORE); if none
    // arrives for ~25 ms (SCL is quiet far longer than any real I2C bit even at
    // 100 kHz), release SDA and return to IDLE so the bus self-recovers.  It never
    // fires during real traffic (SCL toggles constantly) nor between transactions
    // (the idle states reset the counter).
    // Power-of-two silence threshold: fire when the counter's TOP bit sets, so both
    // the "fired" and "still counting" tests are a single-bit check instead of a
    // full-width magnitude compare against an arbitrary constant.  That drops a
    // ~20-bit comparator (twice) and one counter FF — and it matters for fmax too:
    // scl_wdog is the source register of the design's worst timing path.  2^18 clk
    // @24 MHz ≈ 10.9 ms of SCL silence — still far longer than any real I2C bit
    // (even a 10 kHz bit is 100 us), so it never fires during traffic and simtest
    // (<< this) is unaffected.  Width scales with CLK_MHZ.
    localparam integer WDOG_BITS = $clog2(CLK_MHZ * 20000); // 19 @24 MHz; fire ≈10.9 ms
    reg  [WDOG_BITS-1:0] scl_wdog;
    wire                 wdog_fire = scl_wdog[WDOG_BITS-1];

    always @(posedge clk) begin
        if (rst) begin
            conv_left  <= 16'd0;
            conv_presc <= 5'd0;
        end else if (conv_trig) begin
            conv_left  <= conv_us;
            conv_presc <= 5'd0;
        end else if (conv_left != 16'd0) begin
            if (conv_presc == PRESC_MAX) begin
                conv_presc <= 5'd0;
                conv_left  <= conv_left - 16'd1;
            end else begin
                conv_presc <= conv_presc + 5'd1;
            end
        end
    end

    // ---- register pointer & data path ----
    reg [7:0] reg_ptr;
    assign reg_raddr = reg_ptr;

    // Byte to transmit for a read of the current pointer, with the generic
    // busy-bit overlay applied when a conversion is in flight.
    wire [7:0] rd_byte =
        (conv_busy && (reg_ptr == busy_reg)) ? (reg_rdata | busy_mask)
                                             : reg_rdata;

    // ---- FSM ----
    localparam S_IDLE      = 3'd0;
    localparam S_ADDR      = 3'd1;
    localparam S_ADDR_ACK  = 3'd2;  // slave drives ACK for matched address
    localparam S_WRITE     = 3'd3;  // receive a data byte from the DUT
    localparam S_WRITE_ACK = 3'd4;  // slave drives ACK for received byte
    localparam S_READ      = 3'd5;  // transmit a data byte to the DUT
    localparam S_READ_ACK  = 3'd6;  // sample master ACK/NACK after a sent byte
    localparam S_IGNORE    = 3'd7;  // not addressed / NACKed — wait for (re)START/STOP

    (* fsm_encoding = "none" *) reg [2:0] state;
    reg [2:0] bit_cnt;
    reg [7:0] shift_in;
    reg [7:0] tx_shift;
    reg       rw;            // 1 = master reads from us
    reg       ptr_loaded;    // reg_ptr has been set this transaction
    reg       ack_step;      // sub-phase within an ACK (assert → release/sample)
    reg       rd_first;      // next read fall must load+drive the MSB

    wire [7:0] addr_full = {shift_in[6:0], sda_now};  // full byte at 8th rise

    always @(posedge clk) begin
        // 1-cycle defaults
        reg_we    <= 1'b0;
        conv_trig <= 1'b0;

        if (rst) begin
            state         <= S_IDLE;
            bit_cnt       <= 3'd0;
            shift_in      <= 8'h00;
            tx_shift      <= 8'h00;
            rw            <= 1'b0;
            ptr_loaded    <= 1'b0;
            ack_step      <= 1'b0;
            rd_first      <= 1'b0;
            sda_drive_low <= 1'b0;
            reg_ptr       <= 8'h00;
            reg_waddr     <= 8'h00;
            reg_wdata     <= 8'h00;
            armed         <= 1'b0;
            xfer_count    <= 16'd0;
            wr_count      <= 16'd0;
            last_wr_addr  <= 8'h00;
            last_wr_val   <= 8'h00;
            sda_ch        <= 4'd0;
            scl_ch        <= 4'd0;
            addr7         <= 7'h00;
            trig_reg      <= 8'h00;
            busy_reg      <= 8'h00;
            busy_mask     <= 8'h00;
            conv_us       <= 16'd0;
            scl_wdog      <= {WDOG_BITS{1'b0}};
        end else begin
            // ---- SCL-silence watchdog counter: reset on any SCL edge or whenever
            //      no transaction is active; otherwise count up and saturate. ----
            if (scl_rise || scl_fall ||
                state == S_IDLE || state == S_IGNORE || !armed)
                scl_wdog <= {WDOG_BITS{1'b0}};
            else if (!wdog_fire)
                scl_wdog <= scl_wdog + 1'b1;

            // ---- config / disable ----
            if (cfg_stb) begin
                addr7     <= cfg_addr7;
                sda_ch    <= cfg_sda_ch;
                scl_ch    <= cfg_scl_ch;
                trig_reg  <= cfg_trig_reg;
                busy_reg  <= cfg_busy_reg;
                busy_mask <= cfg_busy_mask;
                conv_us   <= cfg_conv_us;
                armed     <= cfg_enable;
                // fresh session
                state         <= S_IDLE;
                ptr_loaded    <= 1'b0;
                sda_drive_low <= 1'b0;
            end
            if (disable_stb) begin
                armed         <= 1'b0;
                state         <= S_IDLE;
                sda_drive_low <= 1'b0;
            end

            if (!armed) begin
                sda_drive_low <= 1'b0;
                state         <= S_IDLE;
            end else if (start_cond) begin
                // (re)START — begin a new address phase, keep reg_ptr.
                state         <= S_ADDR;
                bit_cnt       <= 3'd0;
                shift_in      <= 8'h00;
                ack_step      <= 1'b0;
                rd_first      <= 1'b0;
                sda_drive_low <= 1'b0;
            end else if (stop_cond) begin
                state         <= S_IDLE;
                ptr_loaded    <= 1'b0;
                sda_drive_low <= 1'b0;
                xfer_count    <= xfer_count + 16'd1;
            end else begin
                case (state)
                    // ---- address byte ----
                    S_ADDR: begin
                        if (scl_rise) begin
                            shift_in <= {shift_in[6:0], sda_now};
                            if (bit_cnt == 3'd7) begin
                                if (shift_in[6:0] == addr7) begin
                                    rw       <= sda_now;   // bit0 = R/W
                                    ack_step <= 1'b0;
                                    state    <= S_ADDR_ACK;
                                end else begin
                                    state <= S_IGNORE;     // not us
                                end
                                bit_cnt <= 3'd0;
                            end else begin
                                bit_cnt <= bit_cnt + 3'd1;
                            end
                        end
                    end

                    // ---- ACK the matched address; then branch read/write ----
                    S_ADDR_ACK: begin
                        if (scl_fall) begin
                            if (!ack_step) begin
                                sda_drive_low <= 1'b1;     // assert ACK (low)
                                ack_step      <= 1'b1;
                            end else begin
                                ack_step <= 1'b0;
                                if (rw) begin
                                    // this fall sets up data bit7 (MSB)
                                    tx_shift      <= {rd_byte[6:0], 1'b0};
                                    sda_drive_low <= ~rd_byte[7];
                                    bit_cnt       <= 3'd1; // MSB driven
                                    rd_first      <= 1'b0;
                                    state         <= S_READ;
                                end else begin
                                    sda_drive_low <= 1'b0; // release for RX
                                    bit_cnt       <= 3'd0;
                                    shift_in      <= 8'h00;
                                    state         <= S_WRITE;
                                end
                            end
                        end
                    end

                    // ---- receive a data byte (write transaction) ----
                    S_WRITE: begin
                        if (scl_rise) begin
                            shift_in <= {shift_in[6:0], sda_now};
                            if (bit_cnt == 3'd7) begin
                                if (!ptr_loaded) begin
                                    reg_ptr    <= addr_full;
                                    ptr_loaded <= 1'b1;
                                end else begin
                                    reg_we       <= 1'b1;
                                    reg_waddr    <= reg_ptr;
                                    reg_wdata    <= addr_full;
                                    last_wr_addr <= reg_ptr;
                                    last_wr_val  <= addr_full;
                                    wr_count     <= wr_count + 16'd1;
                                    if (reg_ptr == trig_reg) conv_trig <= 1'b1;
                                    reg_ptr      <= reg_ptr + 8'd1;
                                end
                                ack_step <= 1'b0;
                                state    <= S_WRITE_ACK;
                                bit_cnt  <= 3'd0;
                            end else begin
                                bit_cnt <= bit_cnt + 3'd1;
                            end
                        end
                    end

                    S_WRITE_ACK: begin
                        if (scl_fall) begin
                            if (!ack_step) begin
                                sda_drive_low <= 1'b1; // assert ACK
                                ack_step      <= 1'b1;
                            end else begin
                                sda_drive_low <= 1'b0; // release
                                ack_step      <= 1'b0;
                                bit_cnt       <= 3'd0;
                                shift_in      <= 8'h00;
                                state         <= S_WRITE;
                            end
                        end
                    end

                    // ---- transmit a data byte (read transaction) ----
                    S_READ: begin
                        if (scl_fall) begin
                            if (rd_first) begin
                                tx_shift      <= {rd_byte[6:0], 1'b0};
                                sda_drive_low <= ~rd_byte[7];
                                bit_cnt       <= 3'd1;
                                rd_first      <= 1'b0;
                            end else begin
                                sda_drive_low <= ~tx_shift[7];
                                tx_shift      <= {tx_shift[6:0], 1'b0};
                                if (bit_cnt == 3'd7) begin
                                    ack_step <= 1'b0;
                                    state    <= S_READ_ACK;
                                end
                                bit_cnt <= bit_cnt + 3'd1;
                            end
                        end
                    end

                    S_READ_ACK: begin
                        // fall #9: release SDA so the master can drive ACK/NACK
                        if (scl_fall && !ack_step) begin
                            sda_drive_low <= 1'b0;
                            ack_step      <= 1'b1;
                        end
                        // rise #9: sample the master's ACK(0)/NACK(1)
                        if (scl_rise && ack_step) begin
                            ack_step <= 1'b0;
                            if (~sda_now) begin
                                // ACK → master wants the next byte
                                reg_ptr  <= reg_ptr + 8'd1;
                                rd_first <= 1'b1;   // next fall loads+drives MSB
                                state    <= S_READ;
                            end else begin
                                // NACK → master is done reading
                                state <= S_IGNORE;
                            end
                        end
                    end

                    // S_IDLE / S_IGNORE: wait for (re)START or STOP (handled above)
                    default: begin
                        sda_drive_low <= 1'b0;
                    end
                endcase
            end

            // ---- watchdog recovery (final priority) ----
            // Placed after the FSM so its non-blocking writes win: on a stuck bus
            // (SCL silent for ~WDOG_BITS worth of clocks while mid-transaction)
            // release SDA and reset to IDLE.  A real START/STOP this same cycle is
            // handled above and the counter would already be reset by the SCL edge,
            // so this can't steal a live transaction.
            if (wdog_fire) begin
                state         <= S_IDLE;
                ptr_loaded    <= 1'b0;
                sda_drive_low <= 1'b0;
            end
        end
    end

endmodule
