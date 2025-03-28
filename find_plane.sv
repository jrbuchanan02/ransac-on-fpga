`timescale 1ns/1ps

// find_plane
//
// given 3 points, finds a plane between them, automatically normalizing the plane's normal

`include "fixed_point.svh"

module find_plane#(
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 16,
        parameter int unsigned rsqrt_iterations = 100,
        parameter bit reset_polarity = 1
    )(
        input logic clock,
        input logic reset,

        input ransac_fixed::point_t a,
        input ransac_fixed::point_t b,
        input ransac_fixed::point_t c,
        input logic input_valid,

        output logic input_ready,
        output logic output_valid,
        output ransac_fixed::plane_t p
    );

    struct packed {
        ransac_fixed::point_t a;
        ransac_fixed::point_t b;
        ransac_fixed::point_t c;
    } stored_vars;

    ransac_fixed::vector3f_t v1;    // a - b
    ransac_fixed::vector3f_t v2;    // a - c

    ransac_fixed::vector3f_t n; // v1 cross v2 (not normalized but in the correct direction)
    
    ransac_fixed::fixed_t m; // squared magnitude of n

    ransac_fixed::fixed_t rsqrt_m;   // 1 / sqrt m

    // resulting_plane.normal = n * rsqrt_m
    // resulting_plane.d = a dot resulting_plane.normal
    ransac_fixed::plane_t resulting_plane;






endmodule : find_plane