// tb_dispatch_args.v — self-checking testbench for the cmd_dispatch payload
// byte-counting / terminal logic.
//
// This is the regression guard for the `arg_rem` down-counter that replaced the
// per-state `arg_count + 1 >= arg_len` 16-bit add+compare carry chain (the path
// that used to cap fmax).  Equivalence to the old behaviour means: each command
// must still consume / produce EXACTLY arg_len payload bytes and terminate on
// the right one.  Covered here:
//   * inbound streaming, group A  — LOAD_WAVE / I2C_LOAD_REGS write exactly N
//     bytes to the right addresses (incl. the N==1 boundary)
//   * fixed-length collect        — I2C_CONFIG (9B) / UART_CONFIG (6B) fire their
//     config strobe exactly once, on the final byte, with correct decoded fields
//   * outbound streaming, group B — READ_CAPTURE advances exactly N times then
//     stops (no over-run when extra padding bytes are clocked in)
`timescale 1ns/1ps
module tb_dispatch_args;
    localparam ADDR_W = 12;

    reg clk = 0;
    always #5 clk = ~clk;

    reg        rst = 1;
    reg  [7:0] rx_byte = 8'h00;
    reg        rx_valid = 0;
    reg        cs_active = 0;
    wire [7:0] tx_byte;

    // waveform BRAM write port (LOAD_WAVE)
    wire              wave_we;
    wire [ADDR_W-1:0] wave_waddr;
    wire [7:0]        wave_wdata;

    // capture BRAM read port (READ_CAPTURE) — behavioural ramp w/ 1-cycle latency

    // I2C sensor config decode (I2C_CONFIG)
    wire        i2c_cfg_stb;
    wire [6:0]  i2c_cfg_addr7;
    wire [3:0]  i2c_cfg_sda_ch, i2c_cfg_scl_ch;
    wire        i2c_cfg_enable;
    wire [7:0]  i2c_cfg_trig_reg, i2c_cfg_busy_reg, i2c_cfg_busy_mask;
    wire [15:0] i2c_cfg_conv_us;

    // I2C register-file write port (I2C_LOAD_REGS)
    wire        i2c_reg_we;
    wire [7:0]  i2c_reg_waddr, i2c_reg_wdata;

    // UART TX FIFO write port (UART_WRITE)
    wire        uart_tx_we;
    wire [7:0]  uart_tx_wdata;

    // stepper control (GPIO_STEP) — 16-bit steps/delay after Tier-2 narrowing
    wire        step_start;
    wire [3:0]  step_channel;
    wire [15:0] step_steps;
    wire [15:0] step_delay;

    // unified capture arming (OP_CAPTURE 0x31): arms the ADC + LA producers together
    wire [23:0]      cap_count;          // 24-bit: deep ADC capture up to 2,097,152 samples (matches cmd_dispatch)
    wire [15:0]      cap_divider;
    wire             la_cap_start;
    wire [23:0]      la_cap_count;      // 24-bit: deep LA up to the full 8 MB PSRAM
    wire [15:0]      la_cap_divider;

    // UART config decode (UART_CONFIG)
    wire        uart_cfg_stb;
    wire [3:0]  uart_cfg_rx_ch, uart_cfg_tx_ch;
    wire [23:0] uart_cfg_div;

    // ---- P3 hardening: previously-untested decode + response paths ----
    wire            dac_start, cap_start;          // START_DAC / START_CAPTURE arming
    wire [ADDR_W:0] dac_period;
    wire [15:0]     dac_divider;
    wire            gpio_set_stb;                  // GPIO_SET
    wire [3:0]      gpio_set_ch;
    wire [1:0]      gpio_set_mode;
    wire            swd_arm_stb, swd_nrst_present;  // SWD_ARM
    wire [3:0]      swd_clk_ch, swd_dio_ch, swd_nrst_ch;
    // STATUS flag inputs (driven to a known pattern to check the response byte)
    reg dac_running = 0, cap_busy = 0, cap_done = 0, step_busy = 0, swd_armed = 0;
    reg cap_overflow = 0;   // STATUS bit5
    reg loop_tripped = 0;   // STATUS bit6 (v30 control-loop over-range trip)
    wire        uart_cfg_enable;
    // v35: capture trigger config (SET_TRIGGER), its status (TRIGGER_STATUS) and GPIO_GET levels
    wire [3:0]  trig_ch;
    wire        trig_en, trig_edge, trig_pol;
    reg         trig_wait = 0, trig_fired = 0;
    reg  [13:0] la_levels = 14'h0000;

    cmd_dispatch #(.GATEWARE_VERSION(8'd6), .ADDR_W(ADDR_W)) dut (
        .clk(clk), .rst(rst),
        .rx_byte(rx_byte), .rx_valid(rx_valid), .cs_active(cs_active), .tx_byte(tx_byte),
        .wave_we(wave_we), .wave_waddr(wave_waddr), .wave_wdata(wave_wdata),
        .i2c_cfg_stb(i2c_cfg_stb), .i2c_cfg_addr7(i2c_cfg_addr7),
        .i2c_cfg_sda_ch(i2c_cfg_sda_ch), .i2c_cfg_scl_ch(i2c_cfg_scl_ch),
        .i2c_cfg_enable(i2c_cfg_enable), .i2c_cfg_trig_reg(i2c_cfg_trig_reg),
        .i2c_cfg_busy_reg(i2c_cfg_busy_reg), .i2c_cfg_busy_mask(i2c_cfg_busy_mask),
        .i2c_cfg_conv_us(i2c_cfg_conv_us),
        .i2c_reg_we(i2c_reg_we), .i2c_reg_waddr(i2c_reg_waddr), .i2c_reg_wdata(i2c_reg_wdata),
        .uart_cfg_stb(uart_cfg_stb), .uart_cfg_rx_ch(uart_cfg_rx_ch),
        .uart_cfg_tx_ch(uart_cfg_tx_ch), .uart_cfg_div(uart_cfg_div),
        .uart_cfg_enable(uart_cfg_enable),
        .uart_tx_we(uart_tx_we), .uart_tx_wdata(uart_tx_wdata),
        .step_start(step_start), .step_channel(step_channel),
        .step_steps(step_steps), .step_delay(step_delay),
        .cap_count(cap_count), .cap_divider(cap_divider),
        .la_cap_start(la_cap_start), .la_cap_count(la_cap_count), .la_cap_divider(la_cap_divider),
        .dac_start(dac_start), .dac_period(dac_period), .dac_divider(dac_divider),
        .cap_start(cap_start),
        .gpio_set_stb(gpio_set_stb), .gpio_set_ch(gpio_set_ch), .gpio_set_mode(gpio_set_mode),
        .swd_arm_stb(swd_arm_stb), .swd_clk_ch(swd_clk_ch), .swd_dio_ch(swd_dio_ch),
        .swd_nrst_ch(swd_nrst_ch), .swd_nrst_present(swd_nrst_present),
        .dac_running(dac_running), .cap_busy(cap_busy), .cap_done(cap_done),
        .cap_overflow(cap_overflow), .loop_tripped(loop_tripped),
        .trig_ch(trig_ch), .trig_en(trig_en), .trig_edge(trig_edge), .trig_pol(trig_pol),
        .trig_wait(trig_wait), .trig_fired(trig_fired), .la_levels(la_levels),
        .step_busy(step_busy), .swd_armed(swd_armed)
        // remaining engine ports intentionally unconnected (not exercised here)
    );

    integer errors = 0;

    // ---- monitors (reset per subtest) -------------------------------------
    integer          wr_n;                  // LOAD_WAVE write count
    reg [ADDR_W-1:0] wr_a [0:63];
    reg [7:0]        wr_d [0:63];
    always @(posedge clk) if (!rst && wave_we) begin
        wr_a[wr_n] = wave_waddr; wr_d[wr_n] = wave_wdata; wr_n = wr_n + 1;
    end

    integer    rg_n;                        // I2C_LOAD_REGS write count
    reg [7:0]  rg_a [0:63];
    reg [7:0]  rg_d [0:63];
    always @(posedge clk) if (!rst && i2c_reg_we) begin
        rg_a[rg_n] = i2c_reg_waddr; rg_d[rg_n] = i2c_reg_wdata; rg_n = rg_n + 1;
    end

    integer i2c_cfg_n;  always @(posedge clk) if (i2c_cfg_stb)  i2c_cfg_n  = i2c_cfg_n  + 1;
    integer uart_cfg_n; always @(posedge clk) if (uart_cfg_stb) uart_cfg_n = uart_cfg_n + 1;

    integer    tw_n;                        // UART_WRITE FIFO push count
    reg [7:0]  tw_d [0:63];
    always @(posedge clk) if (!rst && uart_tx_we) begin
        tw_d[tw_n] = uart_tx_wdata; tw_n = tw_n + 1;
    end

    integer step_n; always @(posedge clk) if (step_start) step_n = step_n + 1;
    integer gset_n; always @(posedge clk) if (gpio_set_stb) gset_n = gset_n + 1;
    integer sarm_n; always @(posedge clk) if (swd_arm_stb)  sarm_n = sarm_n + 1;
    integer dac_n;  always @(posedge clk) if (dac_start)    dac_n  = dac_n  + 1;
    integer caps_n; always @(posedge clk) if (cap_start)    caps_n = caps_n + 1;
    integer lacap_n; always @(posedge clk) if (la_cap_start) lacap_n = lacap_n + 1;


    // ---- drivers ----------------------------------------------------------
    task feed(input [7:0] b);               // one byte, sampled on one posedge
        begin @(negedge clk); rx_byte = b; rx_valid = 1; @(negedge clk); rx_valid = 0; end
    endtask
    task cs_hi; begin @(negedge clk); cs_active = 1; end endtask
    task cs_lo; begin                        // CSn↑ -> FSM back to IDLE
        @(negedge clk); cs_active = 0; @(negedge clk); @(negedge clk); end
    endtask

    // ---- subtests ---------------------------------------------------------
    // group A: LOAD_WAVE of n bytes must write exactly n cells (addr 0..n-1).
    task test_load_wave(input integer n);
        integer k; begin
            wr_n = 0; cs_hi;
            feed(8'h10); feed(n[7:0]); feed(n[15:8]);
            for (k = 0; k < n; k = k + 1) feed(8'hA0 + k[7:0]);
            cs_lo;
            if (wr_n !== n) begin
                $display("FAIL load_wave n=%0d: wrote %0d cells", n, wr_n); errors = errors + 1;
            end else for (k = 0; k < n; k = k + 1)
                if (wr_a[k] !== k[ADDR_W-1:0] || wr_d[k] !== (8'hA0 + k[7:0])) begin
                    $display("FAIL load_wave n=%0d idx %0d: addr=%0d data=%02h",
                             n, k, wr_a[k], wr_d[k]); errors = errors + 1;
                end
        end
    endtask

    // group A: I2C_LOAD_REGS [start][len][data..] writes n cells at start+0..n-1.
    task test_load_regs(input [7:0] start, input integer n);
        integer k; begin
            rg_n = 0; cs_hi;
            feed(8'h62); feed(start); feed(n[7:0]); feed(n[15:8]);
            for (k = 0; k < n; k = k + 1) feed(8'hD0 + k[7:0]);
            cs_lo;
            if (rg_n !== n) begin
                $display("FAIL load_regs n=%0d: wrote %0d cells", n, rg_n); errors = errors + 1;
            end else for (k = 0; k < n; k = k + 1)
                if (rg_a[k] !== (start + k[7:0]) || rg_d[k] !== (8'hD0 + k[7:0])) begin
                    $display("FAIL load_regs idx %0d: addr=%02h data=%02h",
                             k, rg_a[k], rg_d[k]); errors = errors + 1;
                end
        end
    endtask

    // group A: UART_WRITE [len][data..] pushes exactly n bytes into the TX FIFO.
    // (Exercises the OP_UART_WRITE arm of the S_READ_LEN1 command dispatch.)
    task test_uart_write(input integer n);
        integer k; begin
            tw_n = 0; cs_hi;
            feed(8'h72); feed(n[7:0]); feed(n[15:8]);
            for (k = 0; k < n; k = k + 1) feed(8'hE0 + k[7:0]);
            cs_lo;
            if (tw_n !== n) begin
                $display("FAIL uart_write n=%0d: pushed %0d bytes", n, tw_n); errors = errors + 1;
            end else for (k = 0; k < n; k = k + 1)
                if (tw_d[k] !== (8'hE0 + k[7:0])) begin
                    $display("FAIL uart_write idx %0d: data=%02h", k, tw_d[k]); errors = errors + 1;
                end
        end
    endtask

    // zero-length payload must dispatch straight to DONE — no FIFO pushes.
    task test_uart_write_zero;
        begin
            tw_n = 0; cs_hi;
            feed(8'h72); feed(8'h00); feed(8'h00);   // len = 0
            feed(8'h00);                              // stray byte: must be ignored
            cs_lo;
            if (tw_n !== 0) begin
                $display("FAIL uart_write_zero: pushed %0d bytes (want 0)", tw_n);
                errors = errors + 1;
            end
        end
    endtask

    // (group B READ_CAPTURE over-run test removed in v17 — the opcode is gone; the
    //  same length-prefixed read-back over-run guard is still covered by SWD_READ.)

    // collect: GPIO_STEP (5 payload bytes after Tier-2 narrowing) fires step_start
    // once and decodes [channel][steps(2,LE)][delay_us(2,LE)] into 16-bit fields.
    task test_gpio_step;
        begin
            step_n = 0; cs_hi;
            feed(8'h41);   // OP_GPIO_STEP
            feed(8'h05);   // channel
            feed(8'h34);   // steps_lo
            feed(8'h12);   // steps_hi   -> steps = 0x1234
            feed(8'h67);   // delay_lo
            feed(8'h05);   // delay_hi   -> delay = 0x0567 (final byte = rx_byte)
            cs_lo;
            if (step_n !== 1)              begin $display("FAIL gpio_step: start fired %0d times (want 1)", step_n); errors=errors+1; end
            if (step_channel !== 4'h5)     begin $display("FAIL gpio_step: channel=%0d",  step_channel); errors=errors+1; end
            if (step_steps   !== 16'h1234) begin $display("FAIL gpio_step: steps=%04h",    step_steps);   errors=errors+1; end
            if (step_delay   !== 16'h0567) begin $display("FAIL gpio_step: delay=%04h",    step_delay);   errors=errors+1; end
        end
    endtask

    // collect: OP_CAPTURE (10 payload bytes) arms the ADC + raw-LA producers on the
    // SAME cycle (unified capture) and decodes both count/divider pairs into the
    // separate ADC (cap_*) and LA (la_cap_*) config.  BOTH counts are 24-bit (v16
    // deep ADC + deep LA), so this feeds an ADC count > 16 bits and an LA count that
    // spans the full multi-MB region.
    task test_capture;
        begin
            caps_n = 0; lacap_n = 0; cs_hi;
            feed(8'h31);                            // OP_CAPTURE
            feed(8'h00); feed(8'h80); feed(8'h1F);  // adc_count = 0x1F8000 (2,064,384) — DEEP: bit set above bit 15
            feed(8'h18); feed(8'h00);               // adc_div   = 0x0018
            feed(8'h00); feed(8'h40); feed(8'h10);  // la_count  = 0x104000 (1,065,024) — DEEP 24-bit LA
            feed(8'h64); feed(8'h00);               // la_div    = 0x0064 (final byte = rx_byte)
            cs_lo;
            if (caps_n !== 1)                  begin $display("FAIL capture: adc arm fired %0d (want 1)", caps_n); errors=errors+1; end
            if (lacap_n !== 1)                 begin $display("FAIL capture: la arm fired %0d (want 1)", lacap_n); errors=errors+1; end
            // 0x1F8000 > 65535 (16-bit max): a re-narrowed cap_count would drop the high byte — guards the 24-bit widening (v16 deep ADC).
            if (cap_count      !== 24'h1F8000) begin $display("FAIL capture: adc_count=%h (want 1F8000 — 24-bit deep count)", cap_count); errors=errors+1; end
            if (cap_divider    !== 16'h0018)   begin $display("FAIL capture: adc_div=%h",     cap_divider);    errors=errors+1; end
            if (la_cap_count   !== 24'h104000) begin $display("FAIL capture: la_count=%h (want 104000 — 24-bit deep LA)", la_cap_count); errors=errors+1; end
            if (la_cap_divider !== 16'h0064)   begin $display("FAIL capture: la_div=%h",       la_cap_divider); errors=errors+1; end
            caps_n = 0;   // leave the shared cap_start counter clean for later tests
        end
    endtask

    // collect: OP_LA_CAPTURE (0x69, 5 payload bytes) arms the deep-LA producer with a
    // 24-bit sample count (so one capture can span the full 8 MB PSRAM).
    task test_la_capture;
        begin
            lacap_n = 0; cs_hi;
            feed(8'h69);   // OP_LA_CAPTURE
            feed(8'h00); feed(8'h80); feed(8'h3F);   // count = 0x3F8000 (4,161,536 = full LA region)
            feed(8'h02); feed(8'h00);                // divider = 0x0002 (final byte = rx_byte)
            cs_lo;
            if (lacap_n !== 1)                 begin $display("FAIL la_capture: la arm fired %0d (want 1)", lacap_n); errors=errors+1; end
            if (la_cap_count   !== 24'h3F8000) begin $display("FAIL la_capture: la_count=%h",  la_cap_count);   errors=errors+1; end
            if (la_cap_divider !== 16'h0002)  begin $display("FAIL la_capture: la_div=%h",     la_cap_divider); errors=errors+1; end
        end
    endtask

    // collect: GPIO_SET (2 payload bytes) fires gpio_set_stb once + decodes.
    task test_gpio_set;
        begin
            gset_n = 0; cs_hi;
            feed(8'h40); feed(8'h09); feed(8'h02);   // channel 9, mode 2 (hi-Z)
            cs_lo;
            if (gset_n !== 1)            begin $display("FAIL gpio_set: stb %0d (want 1)", gset_n); errors=errors+1; end
            if (gpio_set_ch !== 4'h9)    begin $display("FAIL gpio_set: ch=%0d", gpio_set_ch); errors=errors+1; end
            if (gpio_set_mode !== 2'd2)  begin $display("FAIL gpio_set: mode=%0d", gpio_set_mode); errors=errors+1; end
        end
    endtask

    // collect: SWD_ARM (3 payload bytes) fires swd_arm_stb once + decodes.
    task test_swd_arm;
        begin
            sarm_n = 0; cs_hi;
            feed(8'h50); feed(8'h03); feed(8'h04); feed(8'h05);   // clk=3, dio=4, nrst=5
            cs_lo;
            if (sarm_n !== 1)               begin $display("FAIL swd_arm: stb %0d (want 1)", sarm_n); errors=errors+1; end
            if (swd_clk_ch !== 4'h3)        begin $display("FAIL swd_arm: clk=%0d", swd_clk_ch); errors=errors+1; end
            if (swd_dio_ch !== 4'h4)        begin $display("FAIL swd_arm: dio=%0d", swd_dio_ch); errors=errors+1; end
            if (swd_nrst_ch !== 4'h5)       begin $display("FAIL swd_arm: nrst=%0d", swd_nrst_ch); errors=errors+1; end
            if (swd_nrst_present !== 1'b1)  begin $display("FAIL swd_arm: nrst_present=%b (want 1, nrst!=0xFF)", swd_nrst_present); errors=errors+1; end
        end
    endtask

    // S_ARG3: START_DAC (4 bytes) and START_CAPTURE decode their count+divider.
    task test_start_dac;
        begin
            dac_n = 0; cs_hi;
            feed(8'h11); feed(8'h34); feed(8'h12); feed(8'h18); feed(8'h00);  // period 0x1234, div 0x0018
            cs_lo;
            if (dac_n !== 1)              begin $display("FAIL start_dac: start %0d (want 1)", dac_n); errors=errors+1; end
            if (dac_period !== 13'h1234)  begin $display("FAIL start_dac: period=%h (want 1234)", dac_period); errors=errors+1; end
            if (dac_divider !== 16'h0018) begin $display("FAIL start_dac: div=%h", dac_divider); errors=errors+1; end
        end
    endtask
    task test_start_capture;
        begin
            caps_n = 0; cs_hi;
            feed(8'h20); feed(8'h00); feed(8'h01); feed(8'h28); feed(8'h00);  // count 0x0100, div 0x0028
            cs_lo;
            if (caps_n !== 1)              begin $display("FAIL start_cap: start %0d (want 1)", caps_n); errors=errors+1; end
            if (cap_count !== 24'h000100)  begin $display("FAIL start_cap: count=%h", cap_count); errors=errors+1; end
            if (cap_divider !== 16'h0028)  begin $display("FAIL start_cap: div=%h", cap_divider); errors=errors+1; end
        end
    endtask

    // S_IDLE instant responses: PING -> 0xA5, VERSION -> gateware version,
    // STATUS -> packed flag byte.  tx_byte is registered, so sample after the byte.
    task expect_resp(input [7:0] op, input [7:0] want, input [127:0] name);
        begin
            cs_hi; feed(op);
            @(negedge clk);
            if (tx_byte !== want) begin $display("FAIL %0s: tx=%02h want %02h", name, tx_byte, want); errors=errors+1; end
            cs_lo;
        end
    endtask

    // collect: I2C_CONFIG (9 payload bytes) fires i2c_cfg_stb once + decodes.
    task test_i2c_config;
        begin
            i2c_cfg_n = 0; cs_hi;
            feed(8'h60);                        // OP_I2C_CONFIG
            feed(8'h55);  // addr7
            feed(8'h03);  // sda_ch
            feed(8'h04);  // scl_ch
            feed(8'h01);  // flags  -> enable
            feed(8'hAA);  // trig_reg
            feed(8'hBB);  // busy_reg
            feed(8'hCC);  // busy_mask
            feed(8'h12);  // conv_lo
            feed(8'h34);  // conv_hi (final byte = rx_byte)
            cs_lo;
            if (i2c_cfg_n !== 1)                begin $display("FAIL i2c_cfg: stb fired %0d times (want 1)", i2c_cfg_n); errors=errors+1; end
            if (i2c_cfg_addr7     !== 7'h55)    begin $display("FAIL i2c_cfg: addr7=%02h",     i2c_cfg_addr7);     errors=errors+1; end
            if (i2c_cfg_sda_ch    !== 4'h3)     begin $display("FAIL i2c_cfg: sda=%0d",        i2c_cfg_sda_ch);    errors=errors+1; end
            if (i2c_cfg_scl_ch    !== 4'h4)     begin $display("FAIL i2c_cfg: scl=%0d",        i2c_cfg_scl_ch);    errors=errors+1; end
            if (i2c_cfg_enable    !== 1'b1)     begin $display("FAIL i2c_cfg: enable=%b",      i2c_cfg_enable);    errors=errors+1; end
            if (i2c_cfg_trig_reg  !== 8'hAA)    begin $display("FAIL i2c_cfg: trig=%02h",      i2c_cfg_trig_reg);  errors=errors+1; end
            if (i2c_cfg_busy_reg  !== 8'hBB)    begin $display("FAIL i2c_cfg: busy_reg=%02h",  i2c_cfg_busy_reg);  errors=errors+1; end
            if (i2c_cfg_busy_mask !== 8'hCC)    begin $display("FAIL i2c_cfg: busy_mask=%02h", i2c_cfg_busy_mask); errors=errors+1; end
            if (i2c_cfg_conv_us   !== 16'h3412) begin $display("FAIL i2c_cfg: conv_us=%04h",   i2c_cfg_conv_us);   errors=errors+1; end
        end
    endtask

    // collect: UART_CONFIG (6 payload bytes) fires uart_cfg_stb once + decodes.
    task test_uart_config;
        begin
            uart_cfg_n = 0; cs_hi;
            feed(8'h70);                        // OP_UART_CONFIG
            feed(8'h02);  // rx_ch
            feed(8'h05);  // tx_ch
            feed(8'h10);  // div_lo
            feed(8'h20);  // div_mid
            feed(8'h03);  // div_hi
            feed(8'h01);  // flags -> enable (final byte = rx_byte)
            cs_lo;
            if (uart_cfg_n !== 1)                  begin $display("FAIL uart_cfg: stb fired %0d times (want 1)", uart_cfg_n); errors=errors+1; end
            if (uart_cfg_rx_ch !== 4'h2)           begin $display("FAIL uart_cfg: rx_ch=%0d", uart_cfg_rx_ch); errors=errors+1; end
            if (uart_cfg_tx_ch !== 4'h5)           begin $display("FAIL uart_cfg: tx_ch=%0d", uart_cfg_tx_ch); errors=errors+1; end
            if (uart_cfg_div   !== 24'h032010)     begin $display("FAIL uart_cfg: div=%06h",  uart_cfg_div);   errors=errors+1; end
            if (uart_cfg_enable !== 1'b1)          begin $display("FAIL uart_cfg: enable=%b", uart_cfg_enable); errors=errors+1; end
        end
    endtask

    // collect: SET_TRIGGER (0x33, 3 payload bytes) decodes [channel][mode][flags] into the
    // persistent trigger config — and only on the final (flags) byte.
    reg [6:0] saved_trig = 7'd0;
    task test_set_trigger(input [7:0] ch, input [7:0] mode,
                          input exp_en, input exp_edge, input exp_pol);
        reg [6:0] early; begin
            cs_hi;
            feed(8'h33); feed(ch); feed(mode);
            @(negedge clk);
            early = {trig_ch, trig_en, trig_edge, trig_pol};
            feed(8'h00);                              // flags (reserved) = final byte
            feed(8'h5A);                              // stray byte after the payload: ignored
            cs_lo;
            if (trig_ch !== ch[3:0] || trig_en !== exp_en || trig_edge !== exp_edge || trig_pol !== exp_pol) begin
                $display("FAIL set_trigger ch=%0d mode=%0d: ch=%0d en=%b edge=%b pol=%b (want %0d %b %b %b)",
                         ch, mode, trig_ch, trig_en, trig_edge, trig_pol, ch[3:0], exp_en, exp_edge, exp_pol);
                errors = errors + 1;
            end
            if (early !== saved_trig) begin
                $display("FAIL set_trigger ch=%0d mode=%0d: config changed before the final payload byte", ch, mode);
                errors = errors + 1;
            end
            saved_trig = {trig_ch, trig_en, trig_edge, trig_pol};
        end
    endtask

    // GPIO_GET (0x43): 2-byte LE reply of la_levels, both bytes from ONE snapshot taken at the opcode.
    task test_gpio_get(input [13:0] lv);
        begin
            la_levels = lv;
            cs_hi; feed(8'h43);
            @(negedge clk);
            if (tx_byte !== lv[7:0]) begin $display("FAIL gpio_get %04h: lo=%02h", lv, tx_byte); errors=errors+1; end
            la_levels = ~lv;                          // pins move between the two reply bytes
            feed(8'h00);
            @(negedge clk);
            if (tx_byte !== {2'b00, lv[13:8]}) begin $display("FAIL gpio_get %04h: hi=%02h (torn or not zero-extended)", lv, tx_byte); errors=errors+1; end
            feed(8'h00);
            cs_lo;
        end
    endtask

    initial begin
        repeat (4) @(negedge clk); rst = 0; @(negedge clk);
        if (trig_en !== 1'b0) begin $display("FAIL: trigger enabled out of reset"); errors=errors+1; end

        test_load_wave(1);     // boundary: single-byte payload
        test_load_wave(3);
        test_load_wave(8);
        test_load_regs(8'h10, 3);
        test_uart_write(5);    // length-prefixed inbound stream
        test_uart_write_zero;  // zero-length -> straight to DONE
        test_gpio_step;
        test_gpio_set;
        test_swd_arm;
        test_start_dac;
        test_start_capture;
        test_capture;
        test_la_capture;
        test_i2c_config;
        test_uart_config;

        // instant responses
        expect_resp(8'h01, 8'hA5, "ping");
        expect_resp(8'h02, 8'd6,  "version");   // GATEWARE_VERSION param = 6
        dac_running=1; cap_busy=0; cap_done=1; step_busy=0; swd_armed=1; cap_overflow=0;  // -> 0x15
        expect_resp(8'h03, 8'h15, "status");
        cap_overflow=1;                                                   // bit5 -> 0x35
        expect_resp(8'h03, 8'h35, "status+overflow");
        loop_tripped=1;                                                   // bit6 -> 0x75
        expect_resp(8'h03, 8'h75, "status+loop trip");
        cap_overflow=0; loop_tripped=0;

        // v35: SET_TRIGGER mode table, TRIGGER_STATUS bits, GPIO_GET
        //                ch     mode  en edge pol
        test_set_trigger(8'd3,  8'd1, 1, 1, 1);   // rising
        test_set_trigger(8'd11, 8'd2, 1, 1, 0);   // falling
        test_set_trigger(8'd0,  8'd3, 1, 0, 1);   // high
        test_set_trigger(8'd7,  8'd4, 1, 0, 0);   // low
        test_set_trigger(8'd9,  8'd5, 0, 0, 0);   // out-of-range mode = off
        test_set_trigger(8'd5,  8'd1, 1, 1, 1);
        test_set_trigger(8'd5,  8'd0, 0, 0, 0);   // off
        trig_wait=0; trig_fired=0; expect_resp(8'h34, 8'h00, "trig_status idle");
        trig_wait=1; trig_fired=0; expect_resp(8'h34, 8'h01, "trig_status wait");
        trig_wait=0; trig_fired=1; expect_resp(8'h34, 8'h02, "trig_status fired");
        trig_wait=0; trig_fired=0;
        test_gpio_get(14'h2A5C);
        test_gpio_get(14'h13C3);
        test_gpio_get(14'h3000);

        if (errors == 0)
            $display("PASS tb_dispatch_args: arg_rem terminal counting matches across all paths");
        else
            $display("FAIL tb_dispatch_args: %0d error(s)", errors);
        $finish;
    end

    initial begin #500000 $display("FAIL tb_dispatch_args: timeout"); $finish; end
endmodule
