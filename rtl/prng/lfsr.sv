`timescale 1ns / 1ps

// Linear Feedback Shift Register
//
// benefits for this application:
// 1. uniform output (each bit is a 1 just over 50% of the time)
// 2. not truly random (for a cycle length >= 3, any 3 outputs are guaranteed
//    unique)
// 3. simple hardware
module lfsr#(
    parameter int unsigned register_width = 64,
    parameter int unsigned window_width = register_width / 2,
    parameter int unsigned window_start = register_width / 4,
    // Taken from XAPP02 to get a maximum length 64-bit lFSR
    parameter bit [register_width-1:0] polynomial = 64'hD800_0000_0000_0000,
    // in theory, this LFSR value shouldn't matter so long as it's not all 0's
    // and not all 1's
    parameter bit [register_width-1:0] start_value = 64'h1234_5678_9ABC_DEF0,
    parameter bit reset_polarity = 1)(
    input logic clock,
    input logic reset,
    output logic [window_width-1:0] random
    );

    logic [register_width-1:0] lfsr_value;

    assign random = lfsr_value[window_start+:window_width];

    logic shift_bit;

    always_comb begin : find_shift_bit
        shift_bit = 1;
        for (int unsigned i = 0; i < register_width; i++) begin
            if (polynomial[i]) begin
                shift_bit = shift_bit ~^ lfsr_value[i];
            end
        end
    end : find_shift_bit

    always_ff @(posedge clock) begin 
        if (reset == reset_polarity || lfsr_value == '1 || lfsr_value == '0) begin
            lfsr_value <= start_value;
        end else begin
            // matches the direction from XAPP02
            lfsr_value[1+:register_width-1] = lfsr_value;
            lfsr_value[0] = shift_bit;
        end
    end

endmodule : lfsr