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

    logic cross_product_input_valid;
    logic cross_product_input_ready;
    logic cross_product_output_valid;
    ransac_fixed::vector3f_t cross_product_lhs;
    ransac_fixed::vector3f_t cross_product_rhs;
    ransac_fixed::vector3f_t cross_product_res;

    logic squared_magnitude_input_valid;
    logic squared_magnitude_input_ready;
    logic squared_magnitude_output_valid;
    ransac_fixed::vector3f_t squared_magnitude_vector;
    ransac_fixed::fixed_t squared_magnitude_result;

    logic rsqrt_input_valid;
    logic [$clog2(rsqrt_iterations)-1:0] rsqrt_iteration_number;
    logic rsqrt_input_ready;
    logic rsqrt_output_valid;
    ransac_fixed::fixed_t rsqrt_number;
    ransac_fixed::fixed_t rsqrt_old_guess;
    ransac_fixed::fixed_t rsqrt_new_guess;

    logic fma_input_valid;
    logic fma_input_ready;
    logic fma_output_valid;
    ransac_fixed::fixed_t fma_a;
    ransac_fixed::fixed_t fma_b;
    ransac_fixed::fixed_t fma_c;
    ransac_fixed::fma_opcode_t fma_opcode;
    ransac_fixed::fixed_t fma_r;

    logic dot_product_input_valid;
    logic dot_product_input_ready;
    logic dot_product_output_valid;
    ransac_fixed::vector3f_t dot_product_lhs;
    ransac_fixed::vector3f_t dot_product_rhs;
    ransac_fixed::fixed_t dot_product_res;


    assign cross_product_lhs = v1;
    assign cross_product_rhs = v2;

    assign squared_magnitude_vector = n;

    assign fma_opcode = ransac_fixed::FMA_OPCODE_POS_A_POS_C;

    enum {
        IDLE,
        FIND_V1_V2,
        CROSS_PRODUCT_INIT_STALL,
        FIND_NONNORMAL_N_WAIT,
        FIND_NONNORMAL_N_DETECT_SQUARED_MAGNITUDE_OVERFLOW,
        FIND_N_DOT_N_STALL,
        FIND_N_DOT_N_WAIT,
        RSQRT_N_DOT_N_ITERATE_INIT,
        RSQRT_N_DOT_N_ITERATE_WAIT,
        RSQRT_N_DOT_N_ITERATE_NEXT,
        RSQRT_N_DOT_N_ITERATE,
        RSQRT_NORMALIZE_N_X_WAIT,
        RSQRT_NORMALIZE_N_Y_WAIT,
        RSQRT_NORMALIZE_N_Z_WAIT,
        FIND_ORIGIN_DIST_INIT,
        FIND_ORIGIN_DIST_WAIT
    } state;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            state <= IDLE;
            input_ready <= 1;
            output_valid <= 0;
            cross_product_input_valid <= 0;
            squared_magnitude_input_valid <= 0;
            rsqrt_input_valid <= 0;
            fma_input_valid <= 0;
        end else begin
            case (state)

            IDLE: begin
                stored_vars.a <= a;
                stored_vars.b <= b;
                stored_vars.c <= c;

                if (input_valid) begin
                    input_ready <= 0;
                    output_valid <= 0;
                    state <= FIND_V1_V2;
                end else begin
                    input_ready <= 1;
                    state <= IDLE;
                end
            end

            FIND_V1_V2: begin
                v1.x <= stored_vars.a.x - stored_vars.b.x;
                v1.y <= stored_vars.a.y - stored_vars.b.y;
                v1.z <= stored_vars.a.z - stored_vars.b.z;

                v2.x <= stored_vars.a.x - stored_vars.c.x;
                v2.y <= stored_vars.a.y - stored_vars.c.y;
                v2.z <= stored_vars.a.z - stored_vars.c.z;

                state <= CROSS_PRODUCT_INIT_STALL;

                cross_product_input_valid <= 1;
            end
            
            CROSS_PRODUCT_INIT_STALL: begin
                cross_product_input_valid <= 1;
                state <= FIND_NONNORMAL_N_WAIT;
            end

            FIND_NONNORMAL_N_WAIT: begin
                cross_product_input_valid <= 0;
                n <= cross_product_res;
                if (cross_product_output_valid) begin
                    state <= FIND_NONNORMAL_N_DETECT_SQUARED_MAGNITUDE_OVERFLOW;
                end else begin
                    squared_magnitude_input_valid <= 0;
                end
            end
            
            FIND_NONNORMAL_N_DETECT_SQUARED_MAGNITUDE_OVERFLOW: begin
                if ( n.x[ransac_fixed::value_bits() - 2] != n.x[ransac_fixed::value_bits() - 1] ||
                     n.y[ransac_fixed::value_bits() - 2] != n.y[ransac_fixed::value_bits() - 1] ||
                     n.z[ransac_fixed::value_bits() - 2] != n.z[ransac_fixed::value_bits() - 1] ||
                     n.x[ransac_fixed::value_bits() - 3] != n.x[ransac_fixed::value_bits() - 1] ||
                     n.y[ransac_fixed::value_bits() - 3] != n.y[ransac_fixed::value_bits() - 1] ||
                     n.z[ransac_fixed::value_bits() - 3] != n.z[ransac_fixed::value_bits() - 1]) begin
                    
                    n.x <= n.x / 8;
                    n.y <= n.y / 8;
                    n.z <= n.z / 8;  
                end 
                state <= FIND_N_DOT_N_STALL;
                squared_magnitude_input_valid <= 1;
            end
            
            FIND_N_DOT_N_STALL: begin
                state <= FIND_N_DOT_N_WAIT;
                squared_magnitude_input_valid <= 1;
            end

            FIND_N_DOT_N_WAIT: begin
                squared_magnitude_input_valid <= 0;
                m <= squared_magnitude_result;
                if (squared_magnitude_output_valid) begin
                    state <= RSQRT_N_DOT_N_ITERATE_INIT;
                end
            end

            RSQRT_N_DOT_N_ITERATE_INIT: begin
                rsqrt_input_valid <= 1;
                rsqrt_old_guess <= ransac_fixed::rsqrt_initial_guess(m);
                rsqrt_number <= m;
                rsqrt_iteration_number <= 0;
                state <= RSQRT_N_DOT_N_ITERATE_WAIT;
            end

            RSQRT_N_DOT_N_ITERATE_WAIT: begin
                rsqrt_input_valid <= 0;
                rsqrt_old_guess <= rsqrt_new_guess;

                if (rsqrt_output_valid) begin
                    rsqrt_iteration_number <= rsqrt_iteration_number + 1;
                    rsqrt_m <= rsqrt_new_guess;
                    state <= RSQRT_N_DOT_N_ITERATE;
                end
            end

            RSQRT_N_DOT_N_ITERATE_NEXT: begin
                rsqrt_input_valid <= 1;
                rsqrt_old_guess <= rsqrt_new_guess;
                state <= RSQRT_N_DOT_N_ITERATE_WAIT;
            end

            RSQRT_N_DOT_N_ITERATE: begin
                if (rsqrt_iteration_number == rsqrt_iterations) begin
                    // set up normalization multiply with X component
                    fma_a <= rsqrt_m;
                    fma_b <= n.x;
                    fma_c <= 0;
                    fma_input_valid <= 1;
                    state <= RSQRT_NORMALIZE_N_X_WAIT;
                end else begin
                    rsqrt_input_valid <= 1;
                    state <= RSQRT_N_DOT_N_ITERATE_NEXT;
                end
            end

            RSQRT_NORMALIZE_N_X_WAIT: begin
                fma_a <= rsqrt_m;
                fma_b <= n.y;
                fma_c <= 0;

                if (fma_output_valid) begin
                    p.normal.x <= fma_r;
                    fma_input_valid <= 1;
                    state <= RSQRT_NORMALIZE_N_Y_WAIT;
                end else begin
                    fma_input_valid <= 0;
                end
            end

            RSQRT_NORMALIZE_N_Y_WAIT: begin
                fma_a <= rsqrt_m;
                fma_b <= n.z;
                fma_c <= 0;

                if (fma_output_valid) begin
                    p.normal.y <= fma_r;
                    fma_input_valid <= 1;
                    state <= RSQRT_NORMALIZE_N_Z_WAIT;
                end else begin
                    fma_input_valid <= 0;
                end
            end

            RSQRT_NORMALIZE_N_Z_WAIT: begin
                
                if (fma_output_valid) begin
                    p.normal.z <= fma_r;
                    state <= FIND_ORIGIN_DIST_INIT;
                end
            end

            FIND_ORIGIN_DIST_INIT: begin
                dot_product_input_valid <= 1;
                dot_product_lhs <= p.normal;
                dot_product_rhs <= stored_vars.a;
                state <= FIND_N_DOT_N_WAIT;
            end

            FIND_ORIGIN_DIST_WAIT: begin
                dot_product_input_valid <= 0;
                p.d <= dot_product_res;
                if (dot_product_output_valid) begin
                    state <= IDLE;
                    input_ready <= 1;
                    output_valid <= 1;
                end
            end

            endcase
        end
    end

    vector_cross_product#(
        .multiply_latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) cross_product(
        .clock(clock),
        .reset(reset),
        .input_valid(cross_product_input_valid),
        .input_ready(cross_product_input_ready),
        .output_valid(cross_product_output_valid),
        .lhs(cross_product_lhs),
        .rhs(cross_product_rhs),
        .res(cross_product_res)
    );

    slow_vector_squared_magnitude#(
        .multiply_latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) squared_magnitude(
        .clock(clock),
        .reset(reset),
        .input_valid(squared_magnitude_input_valid),
        .input_ready(squared_magnitude_input_ready),
        .output_valid(squared_magnitude_output_valid),
        .vector(squared_magnitude_vector),
        .squared_magnitude(squared_magnitude_result)
    );

    newtons_method_rsqrt#(
        .multiply_latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) rsqrt(
        .clock(clock),
        .reset(reset),
        .input_valid(rsqrt_input_valid),
        .input_ready(rsqrt_input_ready),
        .output_valid(rsqrt_output_valid),
        .number(rsqrt_number),
        .old_guess(rsqrt_old_guess),
        .new_guess(rsqrt_new_guess)
    );

    slow_fp_fused_multiply_add#(
        .latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) fma(
        .clock(clock),
        .reset(reset),
        .opcode(fma_opcode),
        .input_valid(fma_input_valid),
        .input_ready(fma_input_ready),
        .output_valid(fma_output_valid),
        .a(fma_a),
        .b(fma_b),
        .c(fma_c),
        .r(fma_r)
    );

    slow_vector_dot_product#(
        .multiply_latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) dot_product(
        .clock(clock),
        .reset(reset),
        .input_ready(dot_product_input_ready),
        .input_valid(dot_product_input_valid),
        .output_valid(dot_product_output_valid),
        .lhs(dot_product_lhs),
        .rhs(dot_product_rhs),
        .dot_product(dot_product_res)
    );

endmodule : find_plane