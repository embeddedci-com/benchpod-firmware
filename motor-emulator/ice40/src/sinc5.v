// ============================================================================
// sinc5.v — five sinc3 decimators (OSR 64) sharing one comb pipeline.
//
// Per channel only the three integrators run at the modulator rate (bit_en, 18 MHz): 19-bit
// Hogenauer CIC, modular arithmetic, never reset-sensitive. The comb stages run once per 64 bits, so
// one comb unit serves all five channels: channel k decimates when the shared bit counter reaches
// 8k (the five sampling instants are 8 bits = 444 ns apart), and its comb delay elements live in a
// block RAM (2 x SB_RAM40_4K as 32-bit words, address {k, stage}).
//
// Output per channel: signed 16-bit, 0 = 50 % ones density, +32767 = all ones (saturated), as in
// the single-channel version. 281.25 kSPS per channel at 18 MHz; `valid` strobes for each channel
// (valid_ch says which), `set_done` after channel 4. The first three outputs of each channel after
// power-up are settling.
// ============================================================================
`default_nettype none

module sinc5 (
    input  wire        clk,
    input  wire        rst,
    input  wire        bit_en,
    input  wire [4:0]  din,
    output reg  signed [15:0] d0, d1, d2, d3, d4,
    output reg         valid,
    output reg  [2:0]  valid_ch,
    output reg         set_done
);
    localparam W = 19;

    // ---------------------------------------------------------------- integrators
    reg [W-1:0] i1 [0:4];
    reg [W-1:0] i2 [0:4];
    reg [W-1:0] i3 [0:4];
    reg [5:0]   dec;
    integer k;
    always @(posedge clk) begin
        if (rst) begin
            dec <= 6'd0;
            for (k = 0; k < 5; k = k + 1) begin i1[k] <= 0; i2[k] <= 0; i3[k] <= 0; end
        end else if (bit_en) begin
            dec <= dec + 6'd1;
            for (k = 0; k < 5; k = k + 1) begin
                i1[k] <= i1[k] + {{(W-1){1'b0}}, din[k]};
                i2[k] <= i2[k] + i1[k];
                i3[k] <= i3[k] + i2[k];
            end
        end
    end

    // ---------------------------------------------------------------- comb state RAM
    reg  [7:0]  raddr, waddr;
    reg  [31:0] wdata;
    reg         we;
    reg  [31:0] rdata;
    reg  [31:0] mem [0:255];
    integer m;
    initial for (m = 0; m < 256; m = m + 1) mem[m] = 32'd0;
    always @(posedge clk) begin
        if (we) mem[waddr] <= wdata;
        rdata <= mem[raddr];
    end

    // ---------------------------------------------------------------- shared comb pipeline
    reg [2:0]   ch;
    reg [W-1:0] x, c1, c2, c3;
    reg [4:0]   cs;                                  // stage strobes
    wire        start = bit_en && (dec[2:0] == 3'd0) && (dec[5:3] < 3'd5);
    wire signed [W:0] s = $signed({1'b0, c3}) - $signed(20'd131072);

    always @(posedge clk) begin
        valid <= 1'b0; set_done <= 1'b0; we <= 1'b0;
        if (rst) begin
            cs <= 5'd0; ch <= 3'd0;
            d0 <= 0; d1 <= 0; d2 <= 0; d3 <= 0; d4 <= 0;
        end else begin
            cs <= {cs[3:0], 1'b0};
            if (start) begin
                ch <= dec[5:3];
                case (dec[5:3])
                    3'd0: x <= i3[0]; 3'd1: x <= i3[1]; 3'd2: x <= i3[2]; 3'd3: x <= i3[3];
                    default: x <= i3[4];
                endcase
                raddr <= {2'b00, dec[5:3], 3'd0};         // c1_d of this channel
                cs[0] <= 1'b1;
            end
            if (cs[0]) raddr <= {2'b00, ch, 3'd1};        // c2_d next
            if (cs[1]) begin                             // rdata = c1_d
                c1 <= x - rdata[W-1:0];
                we <= 1'b1; waddr <= {2'b00, ch, 3'd0}; wdata <= {13'd0, x};
                raddr <= {2'b00, ch, 3'd2};
            end
            if (cs[2]) begin                             // rdata = c2_d
                c2 <= c1 - rdata[W-1:0];
                we <= 1'b1; waddr <= {2'b00, ch, 3'd1}; wdata <= {13'd0, c1};
            end
            if (cs[3]) begin                             // rdata = c3_d
                c3 <= c2 - rdata[W-1:0];
                we <= 1'b1; waddr <= {2'b00, ch, 3'd2}; wdata <= {13'd0, c2};
            end
            if (cs[4]) begin
                case (ch)
                    3'd0: d0 <= (s >= 20'sd131068) ? 16'sd32767 : s[17:2];
                    3'd1: d1 <= (s >= 20'sd131068) ? 16'sd32767 : s[17:2];
                    3'd2: d2 <= (s >= 20'sd131068) ? 16'sd32767 : s[17:2];
                    3'd3: d3 <= (s >= 20'sd131068) ? 16'sd32767 : s[17:2];
                    default: d4 <= (s >= 20'sd131068) ? 16'sd32767 : s[17:2];
                endcase
                valid <= 1'b1; valid_ch <= ch;
                if (ch == 3'd4) set_done <= 1'b1;
            end
        end
    end
endmodule
`default_nettype wire
