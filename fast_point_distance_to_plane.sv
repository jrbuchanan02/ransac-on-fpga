`timescale 1ns/1ps

// fast_point_distance_to_plane
// 
// given point p and plane q
// tells you how far away p is from q
// latency: variable
// throughput: 1 result / cycle

`include "fixed_point.svh"

module fast_point_distance_to_plane#(
        parameter type external_pipeline = logic,
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 8,
        parameter bit addition_has_latency = 1
    )(
        input logic clock,

        input ransac_fixed::point_t point,
        input ransac_fixed::plane_t plane,
        input external_pipeline pipeline_i,

        output ransac_fixed::fixed_t distance,
        output external_pipeline pipeline_o
    );

    ransac_fixed::fixed_t distance_but_possibly_negative;

    fast_vector_dot_product#(
        .external_pipeline(external_pipeline),
        .multiply_latency(multiply_latency),
        .addition_has_latency(addition_has_latency)
    ) dot_product(
        .clock(clock),
        .lhs(point),
        .rhs(plane.normal),
        .off(-plane.d),
        .pipeline_i(pipeline_i),
        .pipeline_o(pipeline_o),
        .dot_product(distance_but_possibly_negative)
    );

    assign distance = (distance_but_possibly_negative < 0) ? -distance_but_possibly_negative : distance_but_possibly_negative;

endmodule : fast_point_distance_to_plane