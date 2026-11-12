// ============================================================================
// spi_target.v: one emulated SPI sensor (v54, emulation image).
//
// The register model of the I2C targets (i2c_emu.v) on an SPI bus: a 256-word image whose low byte
// is the data and high byte the attributes (bit3 FIFO: reads pop the shared FIFO and the pointer
// stays; bit5 RO: DUT writes are not stored), loaded by the STM32 as target 4 of I2C_LOAD_REGS /
// I2C_LOAD_ATTRS.  A transfer (CS low) is: an address byte, `dummy` bytes, then data bytes that
// the DUT writes or reads.  The address byte holds the address (`addr_mask` selects its bits), a
// read flag (bit `rb_pos` == `rb_val` means read) and optionally an auto-increment flag (bit
// `ib_pos` == `ib_val`; without one the address always increments).  Common layouts: BMP280/BME280
// (bit7 = 1 read, mask 0x7F), MPU-6000 (the same), LIS3DH (bit7 read, bit6 increment, mask 0x3F).
//
// SPI modes 0..3 (CPHA, CPOL).  CS, SCK and MOSI are LA wires sampled at clk through the 2-flop
// synchroniser; MISO goes out through the pin map (code 6), driven only while CS is low and the
// target is enabled and not muted (the mute fault releases it).  The first data byte after the
// address is read from the image within 3 clk of the address byte's last sampling edge and goes
// out on the next shifting edge, so SCK must be at most about 2 MHz (a half period >= 6 clk).
//
// The image's one read port serves the transfer while CS is low and the STM32's read-back
// (I2C_READ_REGS target 4) while it is high; its one write port takes a DUT write first and the STM32's load one clk later.
// Every stored DUT byte also goes to ring 4 of i2c_emu's write log (ev).
// ============================================================================
module spi_target #(
    parameter N = 14
)(
    input  wire          clk,
    input  wire          rst,
    input  wire [N-1:0]  la_lvl,
    output reg           miso,
    output wire          miso_oe,

    input  wire          cfg_stb,
    input  wire [55:0]   cfg,         // SPIT_CONFIG: {addr_mask, inc_bit, read_bit, flags, mosi, sck, cs}
    input  wire          reg_we,      // STM32 load of one image byte
    input  wire          reg_attr,
    input  wire [7:0]    reg_addr,
    input  wire [7:0]    reg_wdata,
    output reg  [7:0]    reg_rdata,   // read-back of reg_addr, refreshed every clk while CS is high
    output wire [31:0]   status,      // {flags, wr_count, 16'd0}
    output reg           ev,          // a DUT byte was stored: log {ev_data} at entry ev_idx (pulse)
    output reg  [4:0]    ev_idx,
    output wire [15:0]   ev_data,

    output reg           fifo_pop,
    output reg           fifo_unf,
    input  wire [7:0]    fifo_head,
    input  wire          fifo_empty
);
    // ---- configuration ----
    reg [3:0] c_cs, c_sck, c_mosi;
    reg       c_en, c_cpha, c_cpol, c_mute;
    reg [1:0] c_dummy;
    reg [2:0] c_rbp, c_ibp;
    reg       c_rbv, c_ibv, c_ibe;
    reg [7:0] c_mask;
    always @(posedge clk) begin
        if (rst) c_en <= 1'b0;
        else if (cfg_stb) begin
            c_cs  <= cfg[3:0];  c_sck <= cfg[11:8];  c_mosi <= cfg[19:16];
            c_en  <= cfg[24];   c_cpha <= cfg[25];   c_cpol <= cfg[26];  c_mute <= cfg[27];
            c_dummy <= cfg[29:28];
            c_rbp <= cfg[34:32];  c_rbv <= cfg[39];
            c_ibp <= cfg[42:40];  c_ibe <= cfg[46];  c_ibv <= cfg[47];
            c_mask <= cfg[55:48];
        end
    end

    // ---- the bus ----
    // the three inputs, picked out of the synchronised levels and registered (one more clk of
    // latency, and the 14:1 multiplexers stay off the state machine's clk path)
    reg  cs_n = 1'b1, sck = 1'b0, mosi = 1'b0;
    always @(posedge clk) begin cs_n <= la_lvl[c_cs]; sck <= la_lvl[c_sck]; mosi <= la_lvl[c_mosi]; end
    reg  sck_p, cs_p;
    wire act   = c_en & ~cs_n;
    wire edge_ = act & (sck != sck_p);
    wire lead  = edge_ & (sck != c_cpol);     // leaves the idle level
    wire trail = edge_ & (sck == c_cpol);
    wire samp  = c_cpha ? trail : lead;
    wire shft  = c_cpha ? lead  : trail;
    wire cs_fall = c_en & cs_p & ~cs_n;

    localparam PH_ADDR = 2'd0, PH_DUMMY = 2'd1, PH_DATA = 2'd2;
    reg [1:0] ph;
    reg [2:0] bc, ob;                         // bits sampled / shifted in this byte
    reg [7:0] sr, tx;
    reg       rd, inc;
    reg [7:0] ptr;
    reg [1:0] dc;
    reg [7:0] wrc, last_a, last_v;
    wire [7:0] byte_in = {sr[6:0], mosi};

    // ---- the image (256 x {attr, data}) ----
    (* no_rw_check *) reg [15:0] img [0:255];
    integer ii;
    initial for (ii = 0; ii < 256; ii = ii + 1) img[ii] = 16'd0;
    // The read port reads the word at ptr while CS is low and the STM32's read-back address while
    // it is high (a read-back during a transfer returns the byte it last read).
    reg [15:0] img_rd;
    wire [15:0] cur = img_rd;                 // the word at ptr, 1 clk old
    reg        ld_pend = 1'b0;  reg [7:0] ld_addr;
    wire       a_fifo = cur[11], a_ro = cur[13];

    wire dut_wr = samp & (bc == 3'd7) & (ph == PH_DATA) & ~rd;
    wire load   = shft & (ob == 3'd0);        // a new byte goes out
    wire ld_rd  = load & (ph == PH_DATA) & rd;
    wire [7:0] nb = ~ld_rd ? 8'd0 : a_fifo ? (fifo_empty ? 8'd0 : fifo_head) : cur[7:0];

    always @(posedge clk) begin
        img_rd <= img[cs_n ? reg_addr : ptr];
        if (cs_p && cs_n) reg_rdata <= reg_attr ? img_rd[15:8] : img_rd[7:0];
        // write port: the DUT's byte first, then a pending STM32 load
        if (dut_wr && !a_ro) img[ptr][7:0] <= byte_in;
        else if (ld_pend) begin
            if (reg_attr) img[ld_addr][15:8] <= reg_wdata;
            else          img[ld_addr][7:0]  <= reg_wdata;
        end
    end

    always @(posedge clk) begin
        fifo_pop <= 1'b0;  fifo_unf <= 1'b0;  ev <= 1'b0;
        if (rst) begin
            ld_pend <= 1'b0;  ph <= PH_ADDR;  bc <= 3'd0;  ob <= 3'd0;  miso <= 1'b0;
            wrc <= 8'd0;  last_a <= 8'd0;  last_v <= 8'd0;  sck_p <= 1'b0;  cs_p <= 1'b1;
        end else begin
            if (reg_we) begin ld_pend <= 1'b1; ld_addr <= reg_addr; end
            else if (!dut_wr) ld_pend <= 1'b0;
            sck_p <= sck;  cs_p <= cs_n;

            if (!act) begin
                ph <= PH_ADDR;  bc <= 3'd0;
            end
            if (cs_fall) begin
                ph <= PH_ADDR;  bc <= 3'd0;  tx <= 8'd0;  miso <= 1'b0;
                ob <= c_cpha ? 3'd0 : 3'd1;   // CPHA 0: the first bit (0) is out with CS
            end else if (act) begin
                if (samp) begin
                    sr <= byte_in;  bc <= bc + 3'd1;
                    if (bc == 3'd7) case (ph)
                        PH_ADDR: begin
                            rd  <= (byte_in[c_rbp] == c_rbv);
                            inc <= ~c_ibe | (byte_in[c_ibp] == c_ibv);
                            ptr <= byte_in & c_mask;
                            dc  <= c_dummy;
                            ph  <= (c_dummy != 2'd0) ? PH_DUMMY : PH_DATA;
                        end
                        PH_DUMMY: begin
                            dc <= dc - 2'd1;
                            if (dc == 2'd1) ph <= PH_DATA;
                        end
                        default: if (!rd) begin
                            wrc <= wrc + 8'd1;  last_a <= ptr;  last_v <= byte_in;
                            ev  <= 1'b1;        ev_idx <= wrc[4:0];
                            if (!a_fifo) ptr <= ptr + {7'd0, inc};
                        end
                    endcase
                end
                if (shft) begin
                    ob <= ob + 3'd1;
                    if (load) begin
                        miso <= nb[7];  tx <= {nb[6:0], 1'b0};
                        if (ld_rd) begin
                            if (a_fifo) begin fifo_pop <= ~fifo_empty; fifo_unf <= fifo_empty; end
                            else ptr <= ptr + {7'd0, inc};
                        end
                    end else begin
                        miso <= tx[7];  tx <= {tx[6:0], 1'b0};
                    end
                end
            end
        end
    end

    assign miso_oe = act & ~c_mute;
    assign status  = {c_en, act, 6'd0, wrc, 16'd0};
    assign ev_data = {last_a, last_v};
endmodule
