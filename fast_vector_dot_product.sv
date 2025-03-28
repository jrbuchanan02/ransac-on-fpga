`timescale 1ns/1ps

// fast_vector_dot_product
//
// calculates the dot product between two vectors with a configurable latency
// but a throughput of one result per cycle

module fast_vector_dot_product#(
        parameter type external_pipeline = logic,
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 8,
        parameter bit addition_has_latency = 1
    )(
        input logic clock,

        input ransac_fixed::vector3f_t lhs,
        input ransac_fixed::vector3f_t rhs,
        // initial offset for the FMA
        // this is an optimization for the fast_point_distance_to_plane module
        input ransac_fixed::fixed_t off,
        input external_pipeline pipeline_i,

        output ransac_fixed::fixed_t dot_product,
        output external_pipeline pipeline_o
    );

    typedef struct {
        ransac_fixed::vector3f_t lhs;
        ransac_fixed::vector3f_t rhs;
        external_pipeline external;
    } pipeline_stage_t;

    // 0 -> x
    // 1 -> y
    // 2 -> z
    // 3 -> output from z
    pipeline_stage_t to_stage[2:0];
    ransac_fixed::fixed_t dot[2:0];
    
    assign to_stage[0].lhs = lhs;
    assign to_stage[0].rhs = rhs;
    assign dot[0] = off;
    assign to_stage[0].external = pipeline_i;

    assign dot_product = dot[2];
    assign pipeline_o = to_stage[2].external;

    genvar i;
    generate
        for(i = 1; i < 3; i++) begin
            fast_fp_fused_multiply_add#(
                .multiply_latency(multiply_latency),
                .addition_has_latency(addition_has_latency),
                .external_pipeline(pipeline_stage_t)
            ) fma(
                .clock(clock),
                .a(i == 1 ? to_stage[i - 1].lhs.x : i == 2 ? to_stage[i - 1].lhs.y : to_stage[i - 1].lhs.z),
                .b(i == 1 ? to_stage[i - 1].rhs.x : i == 2 ? to_stage[i - 1].rhs.y : to_stage[i - 1].rhs.z),
                .c(dot[i - 1]),
                .opcode(ransac_fixed::FMA_OPCODE_POS_A_POS_C),
                .r(dot[i]),
                .pipeline_i(to_stage[i - 1]),
                .pipeline_o(to_stage[i])
            );
        end
    endgenerate

endmodule : fast_vector_dot_product