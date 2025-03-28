
`timescale 1ns/1ps

// fast_fp_fused_multiply_add
//
// does one of:
//
// R = A*B + C
// R = A*B - C
// R = -A*B + C
// R = -A*B - C
//
// with a throughput of one result per cycle and a variable latency (defaults to number of bytes in fixed_t)

`include "fixed_point.svh"

module fast_fp_fused_multiply_add#(
        parameter type external_pipeline = logic,
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 8,
        parameter bit addition_has_latency = 1
    )(
        input logic clock,
        input ransac_fixed::fma_opcode_t opcode,
        input ransac_fixed::fixed_t a,
        input ransac_fixed::fixed_t b,
        input ransac_fixed::fixed_t c,
        input external_pipeline pipeline_i,

        output ransac_fixed::fixed_t r,
        output external_pipeline pipeline_o
    );

    typedef struct {
        ransac_fixed::product_t product;
        ransac_fixed::fixed_t c;
        external_pipeline external;
    } multiply_stage_t;

    multiply_stage_t multiply_stages[multiply_latency-1:0];

    ransac_fixed::product_t adjusted_product;
    ransac_fixed::fixed_t multiply_result;

    assign adjusted_product = multiply_stages[multiply_latency-1].product >>> ransac_fixed::fraction_bits;
    assign multiply_result = adjusted_product[ransac_fixed::value_bits()-1:0];

    fp_add_sub#(
        .external_pipeline(external_pipeline),
        .any_latency(addition_has_latency)
    ) add_part(
        .clock(clock),
        .lhs(multiply_result),
        .rhs(multiply_stages[multiply_latency-1].c),
        .subtract(0),
        .pipeline_i(multiply_stages[multiply_latency-1].external),

        .res(r),
        .pipeline_o(pipeline_o)
    );

    always_ff @(posedge clock) begin
        multiply_stages[0].external <= pipeline_i;

        case (opcode)

        ransac_fixed::FMA_OPCODE_POS_A_POS_C: begin
            multiply_stages[0].product <= a * b;
            multiply_stages[0].c <= c;
        end

        ransac_fixed::FMA_OPCODE_POS_A_NEG_C: begin
            multiply_stages[0].product <= a * b;
            multiply_stages[0].c <= -c;
        end

        ransac_fixed::FMA_OPCODE_NEG_A_POS_C: begin
            multiply_stages[0].product <= -a * b;
            multiply_stages[0].c <= c;
        end


        ransac_fixed::FMA_OPCODE_NEG_A_NEG_C: begin
            multiply_stages[0].product <= -a * b;
            multiply_stages[0].c <= -c;
        end

        endcase

        for (int i = 0; i < multiply_latency - 1; i++) begin
            multiply_stages[i+1] <= multiply_stages[i];
        end
    end

endmodule : fast_fp_fused_multiply_add