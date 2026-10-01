// yosys tri-state buffer primitive, for gate-level runs of the SIM build (behavioural pads)
module \$_TBUF_ (input A, input E, output Y);
    assign Y = E ? A : 1'bz;
endmodule
