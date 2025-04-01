`timescale 1ns/1ps

// slow_vector_dot_product
//
// calculates the dot product of two vectors
//
// note: since this operation is done once per plane,
// it usees slow_fp_fused_multiply_add internally, this
// operation requires 3 fused-multiply-add operations
// internally.

`include "fixed_point.svh"

module slow_vector_dot_product#(
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 8,
        parameter bit reset_polarity = 1
    )(
        input logic clock,
        input logic reset,

        input logic input_valid,
        input ransac_fixed::vector3f_t lhs,
        input ransac_fixed::vector3f_t rhs,

        output logic input_ready,
        output logic output_valid,
        output ransac_fixed::fixed_t dot_product
    );


    struct packed {
        ransac_fixed::vector3f_t lhs;
        ransac_fixed::vector3f_t rhs;
    } stored_vars;

    enum {
        IDLE,
        FMA_X,  // set up vars for x component
        FMA_WAIT_X, // wait on x component, set up vars for y component
        FMA_INIT_Y, // wait one more cycle on y component
        FMA_WAIT_Y, // wait on y component, set up vars for z compoennt
        FMA_INIT_Z, // wait one more cycle on z component
        FMA_WAIT_Z  // wait on z component
    } state;

    ransac_fixed::fixed_t a;    // lhs component
    ransac_fixed::fixed_t b;    // rhs component
    ransac_fixed::fixed_t c;    // rest of squared 

    logic fma_input_valid;
    logic fma_input_ready;
    logic fma_output_valid;
    ransac_fixed::fixed_t fma_r;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            state <= IDLE;
            output_valid <= 0;
            input_ready <= 1;
            fma_input_valid <= 0;
        end else begin

            case (state)

            IDLE: begin

                stored_vars.lhs <= lhs;
                stored_vars.rhs <= rhs;
                fma_input_valid <= 0;
                
                if (input_valid) begin
                    input_ready <= 0;
                    output_valid <= 0;
                    state <= FMA_X;
                end else begin
                    input_ready <= 1;
                end
            end

            FMA_X: begin
                a <= stored_vars.lhs.x;
                b <= stored_vars.rhs.x;
                c <= 0;
                fma_input_valid <= 1;
                state <= FMA_WAIT_X;
            end

            FMA_WAIT_X: begin
                a <= stored_vars.lhs.y;
                b <= stored_vars.rhs.y;
                c <= fma_r;
                if (fma_output_valid) begin
                    fma_input_valid <= 1;
                    state <= FMA_INIT_Y;
                end else begin
                    fma_input_valid <= 0;
                end
            end
            
            FMA_INIT_Y: begin
                a <= stored_vars.lhs.y;
                b <= stored_vars.rhs.y;
                c <= fma_r;
                fma_input_valid <= 1;
                state <= FMA_WAIT_Y;
            end

            FMA_WAIT_Y: begin
                a <= stored_vars.lhs.z;
                b <= stored_vars.rhs.z;
                c <= fma_r;
                if (fma_output_valid) begin
                    fma_input_valid <= 1;
                    state <= FMA_INIT_Z;
                end else begin
                    fma_input_valid <= 0;
                end
            end

            FMA_INIT_Z: begin
                a <= stored_vars.lhs.z;
                b <= stored_vars.rhs.z;
                c <= fma_r;
                fma_input_valid <= 1;
                state <= FMA_WAIT_Z;
            end

            FMA_WAIT_Z: begin
                fma_input_valid <= 0;
                if (fma_output_valid) begin
                    dot_product <= fma_r;
                    state <= IDLE;
                    output_valid <= 1;
                end
            end

            endcase
        end
    end

    slow_fp_fused_multiply_add#(
        .latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) fma(
        .clock(clock),
        .reset(reset),
        .input_valid(fma_input_valid),
        .input_ready(fma_input_ready),
        .a(a),
        .b(b),
        .c(c),
        .opcode(ransac_fixed::FMA_OPCODE_POS_A_POS_C),
        .output_valid(fma_output_valid),
        .r(fma_r)
    );

endmodule : slow_vector_dot_product


module slow_vector_squared_magnitude#(
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 8,
        parameter bit reset_polarity = 1
    )(
        input logic clock,
        input logic reset,

        input logic input_valid,
        input ransac_fixed::vector3f_t vector,

        output logic input_ready,
        output logic output_valid,
        output ransac_fixed::fixed_t squared_magnitude
    );

    slow_vector_dot_product#(
        .multiply_latency(multiply_latency),
        .reset_polarity(reset_polarity)
    ) internal (
        .clock(clock),
        .reset(reset),
        .input_valid(input_valid),
        .lhs(vector),
        .rhs(vector),
        .input_ready(input_ready),
        .output_valid(output_valid),
        .dot_product(squared_magnitude)
    );

endmodule : slow_vector_squared_magnitude