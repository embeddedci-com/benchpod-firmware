// emu_defs.vh — constants shared by the motor-emulator gateware and its benches.
//
// One clock domain: `clk` = 36 MHz from the PLL (12 MHz oscillator x 3). Everything, including
// the SPI link (oversampled) and the modulator clock (clk / 2 = 18 MHz, exactly 50% duty), runs
// on it, so there is no clock-domain crossing in the design (see benchpod-firmware/AGENTS.md).

`ifndef EMU_DEFS_VH
`define EMU_DEFS_VH

`define EMU_CLK_HZ        36000000
`define EMU_ID            16'h4D45      // "ME": read back at register 0x00
`define EMU_VERSION       16'h0004      // gateware version, register 0x01

// Register map (16-bit registers, see PROTOCOL.md)
`define R_ID              8'h00
`define R_VERSION         8'h01
`define R_BOARD           8'h02         // [1:0] board id, [8] probe done, [9] probe ok, W: bit 15 re-probe
`define R_STATUS          8'h03
`define R_CONTROL         8'h04
`define R_ARM             8'h05         // W: any value -> 100 us ARM_REQ_N pulse
`define R_WD_CLEAR        8'h06         // W: any value -> clear the SYNC watchdog trip
`define R_SCRATCH         8'h07
`define R_PWM_PERIOD      8'h08         // clocks per PWM period (default 180 = 200 kHz)
`define R_DEADTIME        8'h09         // clocks of gap per edge, on top of the MP6539's own
`define R_DUTY_A          8'h0A         // Q0.16 fraction of the period, phase node high
`define R_DUTY_B          8'h0B
`define R_DUTY_C          8'h0C
`define R_MIN_ON          8'h0D         // shortest high-side pulse in clocks (shorter -> dropped)
`define R_SINC_A          8'h10         // signed 16-bit, sinc3 OSR 64 (A, B, C phases, D battery, E bus)
`define R_SINC_B          8'h11
`define R_SINC_C          8'h12
`define R_SINC_D          8'h13
`define R_SINC_E          8'h14
`define R_SINC_COUNT      8'h15         // increments on every new A..E sample set
`define R_BRAKE_DUTY      8'h18         // [7:0] brake PWM duty /256 at about 20 kHz
`define R_HALL            8'h1C         // [2:0] hall levels seen by the DUT (manual mode)

// motor model (motor_model.v; formulas in its header and PROTOCOL.md)
`define R_MODE            8'h20         // [1:0] 0 direct duties, 1 fixed speed, 2 dynamic; [2] CM loop; [3] DT comp; [4] halls from model
`define R_W_SET           8'h21
`define R_WSHIFT          8'h22
`define R_KE              8'h23
`define R_KT              8'h24
`define R_LOAD            8'h25
`define R_DAMP            8'h26
`define R_INVJ            8'h27
`define R_JSHIFT          8'h28
`define R_CM_KP           8'h29
`define R_CM_KI           8'h2A
`define R_CM_MARGIN       8'h2B         // common-mode shaping margin from each rail, Q16 of the bus (2 % = 1311)
`define R_DT_FRAC         8'h2C
`define R_DT_IDB          8'h2D
`define R_HALL_OFS        8'h2E
`define R_MODEL_RESET     8'h2F         // W: theta = 0, speed = W_SET, CM integrator = 0
`define R_THETA           8'h30         // R: top 16 bits of the electrical angle
`define R_SPEED           8'h31         // R: w16
`define R_EMF             8'h32         // R: back-EMF amplitude E
`define R_I0              8'h33
`define R_VCM             8'h34
`define R_IQ              8'h35
`define R_MDUTY_A         8'h36         // R: model duties
`define R_MDUTY_B         8'h37
`define R_MDUTY_C         8'h38
`define R_MODEL_COUNT     8'h39
`define R_I_OFS_A         8'h3A         // current offsets (sinc codes), subtracted before the model
`define R_I_OFS_B         8'h3B
`define R_I_OFS_C         8'h3C
`define R_VBUS_OFS        8'h3D
`define R_ADV             8'h3E         // back-EMF angle advance in half PWM periods (0-7, default 3)
`define R_RSUB            8'h3F         // resistance subtracted from the emulated phase: u += R_SUB x i >>> 15

// R_CONTROL bits
`define C_PWM_EN          0
`define C_HSWAP_EN        1
`define C_BRAKE_EN        2
`define C_SYNC_MASTER     3             // board 0 only: emit the 1 ms SYNC pulse
`define C_SYNC_REQUIRED   4             // stop the bridge when SYNC pulses stop (stack mode)

// R_STATUS bits
`define S_FAULT_N         0             // FAULT line level (1 = no fault)
`define S_LATCH_Q         1             // board fault latch armed
`define S_BACKSTOP        2             // 57 V backstop comparator (1 = braking)
`define S_SYNC_LEVEL      3
`define S_WD_TRIP         4             // SYNC held low > 100 us, or pulses missing in stack mode
`define S_PWM_ACTIVE      5             // bridge PWM actually running (all gates met)
`define S_SYNC_SEEN       6             // a SYNC pulse arrived in the last 3 ms
`define S_PLL_LOCK        7

`endif
