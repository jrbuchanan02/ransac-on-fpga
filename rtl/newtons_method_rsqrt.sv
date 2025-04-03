`timescale 1ns/1ps

// newtons_method_rsqrt
//
// runs a single iteration of newton's method to find 1 / sqrt(x) given a guess g
//
// output = 1/2 * g * (1 + g * g * x)
//
//
// or (g + x * g * g * g) >>> 1 (fixed_t is signed)
//
// fma 0: a = x, b = g, c = 0           r = xg
// fma 1: a = fma0.r, b = g, c = 1      r = xg^2 + 1
// fma 2: a = fma1.r, b = g, c = 0      r = xg^3 + g
//
// this method can be iterated by repeatedly passing the result into the guess and repeating the newton's method
// iteration

`include "fixed_point.svh"

module newtons_method_rsqrt#(
        // faster multiplier by default since we expect many iterations of Newton's
        // method
        parameter int unsigned multiply_latency = ransac_fixed::value_bits() / 16,
        parameter bit reset_polarity = 1
    )(
        input logic clock,
        input logic reset,

        input logic input_valid,
        input ransac_fixed::fixed_t number,
        input ransac_fixed::fixed_t old_guess,

        output logic input_ready,
        output logic output_valid,
        output ransac_fixed::fixed_t new_guess
    );
    

    enum {
        IDLE,
        FMA_INIT,   // a = -x, b = g, c = 0
        FMA_WAIT0,  // a = fma.r, b = g, c = 1 (in fixed_t!); wait until previous fma finishes before starting next one
        FMA_STALL,  // wait one cycle between WAIT0 and WAIT1 (because for some reason output_valid on the FMA is too slow?)
        FMA_WAIT1,  // a = fma.r, b = g, c = 0; wait until previous fma finishes before starting next one
        FMA_STALL2, // again wait one cycle between wait1 and wait2
        FMA_WAIT2   // wait on fma, new_guess = fma.r >>> 1
    } state;

    struct packed {
        ransac_fixed::fixed_t number;
        ransac_fixed::fixed_t guess;
    } stored_vars;

    logic fma_input_valid;
    logic fma_input_ready;
    logic fma_output_valid;
    ransac_fixed::fixed_t fma_a;
    ransac_fixed::fixed_t fma_b;
    ransac_fixed::fixed_t fma_c;
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
                stored_vars.number <= number;
                stored_vars.guess <= old_guess;

                fma_input_valid <= 0;

                if (input_valid) begin
                    input_ready <= 0;
                    output_valid <= 0;
                    state <= FMA_INIT;
                end else begin
                    input_ready <= 1;
                end
            end

            FMA_INIT: begin
                fma_a <= -stored_vars.number;
                fma_b <= stored_vars.guess;
                fma_c <= 0;
                fma_input_valid <= 1;
                state <= FMA_WAIT0;
            end

            FMA_WAIT0: begin
                fma_a <= -stored_vars.number;
                fma_b <= stored_vars.guess;
                fma_c <= 3 * ransac_fixed::one();

                if (fma_output_valid) begin
                    fma_input_valid <= 1;
                    state <= FMA_STALL;
                end else begin
                    fma_input_valid <= 0;
                end
            end

            FMA_STALL: begin
                fma_a <= fma_r;
                fma_b <= stored_vars.guess;
                fma_c <= 3 * ransac_fixed::one();
                fma_input_valid <= 1;
                state <= FMA_WAIT1;
            end

            FMA_WAIT1: begin
                fma_a <= fma_r;
                fma_b <= stored_vars.guess;
                fma_c <= 0;

                if (fma_output_valid) begin
                    fma_input_valid <= 1;
                    state <= FMA_STALL2;
                end else begin
                    fma_input_valid <= 0;
                end
            end
            
            FMA_STALL2: begin
                fma_a <= fma_r;
                fma_b <= stored_vars.guess;
                fma_c <= 0;
                fma_input_valid <= 1;
                state <= FMA_WAIT2;
            end

            FMA_WAIT2: begin
                fma_input_valid <= 0;
                
                if (fma_output_valid) begin
                    state <= IDLE;
                    new_guess <= fma_r >>> 1;
                    output_valid <= 1;
                    input_ready <= 1;
                end
            end

            endcase
        end
        
        // simulation only error checking
        
        if (new_guess !== 'bx && new_guess < 0) begin
            $error("Negative guess for 1 / sqrt(x)!");
            $finish;
        end
        
        if (old_guess !== 'bx && old_guess < 0) begin
            $error("Negative initial guess for 1 / sqrt(x)");
            $finish;
        end
        
        if (number !== 'bx && number <= 0) begin
            $error("Attempt to take 1 / sqrt of a negative number");
            $finish;
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
        .output_valid(fma_output_valid),
        .opcode(ransac_fixed::FMA_OPCODE_POS_A_POS_C),
        .a(fma_a),
        .b(fma_b),
        .c(fma_c),
        .r(fma_r)
    );

endmodule : newtons_method_rsqrt