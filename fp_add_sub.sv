`timescale 1ns/1ps

// fp_add_sub: adds or subtracts two ransac::fixed_t values
// in some arbitrary pipeline.
//
// Note: fp_add_sub does not saturate values

`include "fixed_point.svh"

module fp_add_sub#(
        parameter type external_pipeline = logic,
        parameter bit any_latency = 1
    )(
        input logic clock,
        input ransac_fixed::fixed_t lhs,
        input ransac_fixed::fixed_t rhs,
        input logic subtract,
        input external_pipeline pipeline_i,

        output ransac_fixed::fixed_t res,
        output external_pipeline pipeline_o
    );

    generate
        
        if (any_latency) begin

            always_ff @(posedge clock) begin
                res <= lhs + (subtract ? -rhs : rhs);
                pipeline_o <= pipeline_i;
            end

        end else begin
            assign res = lhs + (subtract ? -rhs : rhs);
            assign pipeline_o = pipeline_i;
        end
        
    endgenerate

endmodule : fp_add_sub