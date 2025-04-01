`timescale 1ns/1ps

// fast_vector_dot_product
//
// calculates the dot product between two vectors with a configurable latency
// but a throughput of one result per cycle

module fast_vector_dot_product#(
        parameter type external_pipeline = logic,
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 8
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
        ransac_fixed::fixed_t off;
        external_pipeline external;
    } pipeline_stage_t;

    pipeline_stage_t in_to_x;
    pipeline_stage_t x_to_y;
    pipeline_stage_t y_to_z;
    pipeline_stage_t z_to_out;

    ransac_fixed::fixed_t x;
    ransac_fixed::fixed_t y;
    ransac_fixed::fixed_t z;

    always_comb begin : assign_in_to_x
        in_to_x.lhs <= lhs;
        in_to_x.rhs <= rhs;
        in_to_x.off <= off;
        in_to_x.external <= pipeline_i;
    end : assign_in_to_x

    fast_fp_fused_multiply_add#(
        .external_pipeline(pipeline_stage_t),
        .multiply_latency(multiply_latency)
    ) x_part (
        .clock(clock),
        .opcode(ransac_fixed::FMA_OPCODE_POS_A_POS_C),
        .a(in_to_x.lhs.x),
        .b(in_to_x.rhs.x),
        .c(in_to_x.off),
        .pipeline_i(in_to_x),
        .r(x),
        .pipeline_o(x_to_y)
    );

    fast_fp_fused_multiply_add#(
        .external_pipeline(pipeline_stage_t),
        .multiply_latency(multiply_latency)
    ) y_part (
        .clock(clock),
        .opcode(ransac_fixed::FMA_OPCODE_POS_A_POS_C),
        .a(x_to_y.lhs.y),
        .b(x_to_y.rhs.y),
        .c(x),
        .pipeline_i(x_to_y),
        .r(y),
        .pipeline_o(y_to_z)
    );

    fast_fp_fused_multiply_add#(
        .external_pipeline(pipeline_stage_t),
        .multiply_latency(multiply_latency)
    ) z_part (
        .clock(clock),
        .opcode(ransac_fixed::FMA_OPCODE_POS_A_POS_C),
        .a(y_to_z.lhs.z),
        .b(y_to_z.rhs.z),
        .c(y),
        .pipeline_i(y_to_z),
        .r(z),
        .pipeline_o(z_to_out)
    );

    assign dot_product = z;
    assign pipeline_o = z_to_out.external;

endmodule : fast_vector_dot_product