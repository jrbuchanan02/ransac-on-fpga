`timescale 1ns / 1ps

// 0-cycle latency adder designed to be in a chain of adders for a pipelined 
// addition module
module adder_in_chain#(
    parameter int unsigned width = 16)(
    input logic [width-1:0] lhs,
    input logic [width-1:0] rhs,
    input logic icarry,
    output logic [width-1:0] res,
    output logic ocarry
    );

    always_comb begin
        {ocarry, res} <= lhs + rhs + icarry;
    end

endmodule : adder_in_chain