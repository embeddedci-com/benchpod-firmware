// tb_dac8551_loop.v — guards the dac8551_engine waveform-loop terminal (the
// `sleft` down-counter that replaced `sidx + 1 >= period_samples`).
//
// Rather than sniff the serial frames (racy at frame boundaries), it watches the
// clk-synchronous waveform-BRAM address stream.  Each sample i reads addr 2*i
// then 2*i+1, so for a 3-sample loop the de-duplicated address sequence must be
//   0,1, 2,3, 4,5, 0,1, ...      (wrap back to sample 0 after EXACTLY 3 samples)
// An off-by-one terminal would wrap early (…4,5 -> never) or over-run to 6,7.
`timescale 1ns/1ps
module tb_dac8551_loop;
    reg clk = 0;
    always #10 clk = ~clk;

    reg         rst = 1, start = 0, stop = 0;
    reg  [12:0] period  = 13'd3;     // 3-sample loop
    reg  [15:0] divider = 16'd2;
    wire [11:0] waddr;
    reg  [7:0]  wdata;
    wire        sync, sclk, din, running;

    reg [7:0] wave [0:7];            // contents irrelevant; addresses are checked
    always @(posedge clk) wdata <= wave[waddr];

    dac8551_engine #(.ADDR_W(12)) dut (
        .clk(clk), .rst(rst), .start(start), .stop(stop),
        .period_samples(period), .divider(divider),
        .wave_addr(waddr), .wave_data(wdata),
        .psram_mode(1'b0), .strm_data(8'h00), .strm_valid(1'b0), .strm_pop(),
        .dac_sync(sync), .dac_sclk(sclk), .dac_din(din), .running(running)
    );

    // De-duplicated waveform-address stream.
    integer    an = 0;
    reg [11:0] astream [0:31];
    reg [11:0] alast = 12'hFFF;
    always @(posedge clk) if (!rst && running && waddr != alast) begin
        astream[an] = waddr; an = an + 1; alast = waddr;
    end

    integer errors = 0, k;
    reg [11:0] want [0:7];
    initial begin
        want[0]=0; want[1]=1; want[2]=2; want[3]=3;
        want[4]=4; want[5]=5; want[6]=0; want[7]=1;   // <- wrap to sample 0
    end

    initial begin
        repeat (4) @(posedge clk); rst = 0;
        @(posedge clk); start = 1; @(posedge clk); start = 0;
        wait (an >= 8);
        for (k = 0; k < 8; k = k + 1)
            if (astream[k] !== want[k]) begin
                $display("FAIL tb_dac8551_loop: addr[%0d]=%0d want %0d",
                         k, astream[k], want[k]); errors = errors + 1;
            end
        if (errors == 0)
            $display("PASS tb_dac8551_loop: 3-sample loop wraps 0,1,2,3,4,5,0,1 (sidx 0..2,0)");
        else
            $display("FAIL tb_dac8551_loop: %0d error(s)", errors);
        $finish;
    end

    initial begin #500000 $display("FAIL tb_dac8551_loop: timeout"); $finish; end
endmodule
