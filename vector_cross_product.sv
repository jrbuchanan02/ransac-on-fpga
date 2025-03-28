`timescale 1ns/1ps

`include "fixed_point.svh"

// calculates the cross product between two vectors.
//
// note that since this module is used in finding the equation for a plane,
// this function uses the slow_fp_fused_multiply_add for its FMA operations
//
// However, all 3 dimensions are calculated in parallel, so the latency is 
// two fused-multiply-add operations and not one fma
module vector_cross_product#(
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
        output ransac_fixed::vector3f_t res
    );

    ransac_fixed::vector3f_t temp;

    enum {
        IDLE,
        INIT_FMA_FIRST_COMPONENT,
        WAIT_FMA_FIRST_COMPONENT,
        INIT_FMA_SECOND_COMPONENT,
        WAIT_FMA_SECOND_COMPONENT
    } state;

    struct packed {
        ransac_fixed::vector3f_t lhs;
        ransac_fixed::vector3f_t rhs;
    } stored_vars;

    logic inputs_valid[2:0];
    ransac_fixed::fixed_t a[2:0];
    ransac_fixed::fixed_t b[2:0];
    ransac_fixed::fixed_t c[2:0];

    ransac_fixed::fixed_t r[2:0];

    ransac_fixed::fma_opcode_t opcode[2:0];
    

    logic inputs_ready[2:0];
    logic outputs_valid[2:0];

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            state <= IDLE;
            output_valid <= 0;
            input_ready <= 1;
            for (int i = 0; i < 2; i++) begin
                inputs_valid[i] <= 0;
            end
        end else begin

            case (state)

            IDLE: begin
                stored_vars.lhs <= lhs;
                stored_vars.rhs <= rhs;

                for (int i = 0; i < 3; i++) begin
                    inputs_valid[i] <= 0;
                end

                if (input_valid) begin
                    input_ready <= 0;
                    output_valid <= 0;
                    state <= INIT_FMA_FIRST_COMPONENT;
                end else begin
                    input_ready <= 1;
                    output_valid <= 1;
                end
            end


            INIT_FMA_FIRST_COMPONENT: begin
                for (int i = 0; i < 2; i++) begin
                    c[i] <= 0;
                    inputs_valid[i] <= 1;
                end
                a[0] <= stored_vars.lhs.y;
                a[1] <= stored_vars.lhs.z;
                a[2] <= stored_vars.lhs.x;

                b[0] <= stored_vars.rhs.z;
                b[1] <= stored_vars.rhs.x;
                b[2] <= stored_vars.rhs.y;

                opcode[0] <= ransac_fixed::FMA_OPCODE_POS_A_NEG_C;
                opcode[1] <= ransac_fixed::FMA_OPCODE_POS_A_NEG_C;
                opcode[2] <= ransac_fixed::FMA_OPCODE_POS_A_NEG_C;

                state <= WAIT_FMA_FIRST_COMPONENT;
            end

            WAIT_FMA_FIRST_COMPONENT: begin
                for (int i = 0; i < 3; i++) begin
                    inputs_valid[i] <= 0;
                end

                if (outputs_valid[0] == 1 && outputs_valid[1] == 1 && outputs_valid[2] == 1) begin
                    state <= INIT_FMA_SECOND_COMPONENT;
                end
            end

            INIT_FMA_SECOND_COMPONENT: begin

                for (int i = 0; i < 3; i++) begin
                    c[i] <= r[i];
                    inputs_valid[i] <= 1;
                end

                opcode[0] <= ransac_fixed::FMA_OPCODE_NEG_A_POS_C;
                opcode[1] <= ransac_fixed::FMA_OPCODE_NEG_A_POS_C;
                opcode[2] <= ransac_fixed::FMA_OPCODE_NEG_A_POS_C; 

                a[0] <= stored_vars.lhs.z;
                a[1] <= stored_vars.lhs.x;
                a[2] <= stored_vars.lhs.y;

                b[0] <= stored_vars.rhs.y;
                b[1] <= stored_vars.rhs.z;
                b[2] <= stored_vars.rhs.x;


                state <= WAIT_FMA_SECOND_COMPONENT;
            end

            WAIT_FMA_SECOND_COMPONENT: begin
                for (int i = 0; i < 3; i++) begin
                    inputs_valid[i] <= 0;
                end

                if (outputs_valid[0] == 1 && outputs_valid[1] == 1 && outputs_valid[2] == 1) begin
                    state <= IDLE;
                    res.x <= r[0];
                    res.y <= r[1];
                    res.z <= r[2];
                    output_valid <= 1;
                    input_ready <= 1;
                end

            end

            endcase
        end
    end

    genvar i;
    generate
        for (i = 0; i < 3; i++) begin
            slow_fp_fused_multiply_add#(
                .latency(multiply_latency),
                .reset_polarity(reset_polarity)
            ) component(
                .clock(clock),
                .reset(reset),
                .input_valid(inputs_valid[i]),
                .input_ready(inputs_ready[i]),
                .output_valid(outputs_valid[i]),
                .a(a[i]),
                .b(b[i]),
                .c(c[i]),
                .r(r[i]),
                .opcode(opcode[i])
            );
        end
    endgenerate

endmodule : vector_cross_product