`timescale 1ns/1ps

// slow_fp_fused_multiply_add:
//
// Does one of 
//
// R = A*B + C
// R = A*B - C
// R = -A*B + C
// R = -A*B - C
//
// with a throughput that matches the latency (i.e., latency cycles per result)
// maybe works with vivado?

module slow_fp_fused_multiply_add#(
        parameter int unsigned latency = ransac_fixed::value_bits() / 8,
        parameter bit reset_polarity = 1
    )(
        input logic clock,
        input logic reset,

        input logic input_valid,
        input ransac_fixed::fixed_t a,
        input ransac_fixed::fixed_t b,
        input ransac_fixed::fixed_t c,
        input ransac_fixed::fma_opcode_t opcode,

        output logic input_ready,
        output logic output_valid,
        output ransac_fixed::fixed_t r
    );


    enum {
        IDLE,
        MULTIPLY_INIT,
        MULTIPLY_WAIT,
        ADD
    } state;

    ransac_fixed::product_t product;

    ransac_fixed::product_t adjusted_product;
    ransac_fixed::fixed_t a_times_b;

    struct packed {
        ransac_fixed::fixed_t a;
        ransac_fixed::fixed_t b;
        ransac_fixed::fixed_t c;
    } stored_vars;

    logic [$clog2(latency+1)-1:0] cycle_count;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            state <= IDLE;
            output_valid <= 0;
            input_ready <= 1;
        end else begin
            case (state)

            IDLE: begin

                stored_vars.b <= b;

                case (opcode)

                ransac_fixed::FMA_OPCODE_POS_A_POS_C: begin
                    stored_vars.a <= a;
                    stored_vars.c <= c;
                end

                ransac_fixed::FMA_OPCODE_POS_A_NEG_C: begin
                    stored_vars.a <= a;
                    stored_vars.c <= -c;
                end

                ransac_fixed::FMA_OPCODE_NEG_A_POS_C: begin
                    stored_vars.a <= -a;
                    stored_vars.c <= c;
                end

                ransac_fixed::FMA_OPCODE_NEG_A_NEG_C: begin
                    stored_vars.a <= -a;
                    stored_vars.c <= -c;
                end

                endcase

                if (input_valid) begin
                    input_ready <= 0;
                    output_valid <= 0;
                    state <= MULTIPLY_INIT;
                end else begin
                    input_ready <= 1;
                end
            end

            MULTIPLY_INIT: begin
                product <= stored_vars.a * stored_vars.b;
                cycle_count <= 0;
                state <= MULTIPLY_WAIT;
            end

            MULTIPLY_WAIT: begin
                cycle_count <= cycle_count + 1;
                if (cycle_count == latency-1) begin
                    state <= ADD;
                end
            end

            ADD: begin
                r <= a_times_b + stored_vars.c;
                output_valid <= 1;
                input_ready <= 1;
                state <= IDLE;
            end

            endcase
        end
    end

    assign adjusted_product = product >>> ransac_fixed::fraction_bits;
    assign a_times_b = adjusted_product[ransac_fixed::value_bits()-1:0];

endmodule : slow_fp_fused_multiply_add