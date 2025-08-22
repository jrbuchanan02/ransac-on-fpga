`timescale 1ns / 1ps

// simple 1 bit / cycle pipelined unsigned division / remainder calculation.
module pipelined_divmod#(
    parameter int unsigned word_bits = 32,
    parameter type metadata_type = logic,
    parameter metadata_type metadata_on_reset = 1'b0,
    parameter bit reset_polarity = 1)(
    input logic clock,
    input logic reset,
    input logic ivalid,
    output logic iready,
    input logic [word_bits-1:0] divisor,
    input logic [word_bits-1:0] dividend,
    input metadata_type imeta,
    output logic ovalid,
    input logic oready,
    output logic [word_bits-1:0] quotient,
    output logic [word_bits-1:0] remainder,
    output metadata_type ometa
    );

    struct packed {
        logic [2 * word_bits-1:0] d;
        logic [2 * word_bits-1:0] r;
        logic [1 * word_bits-1:0] q;
        logic valid;
        metadata_type metadata;
    } stage[word_bits:-1];

    logic pipeline_should_advance;

    always_comb begin 
        if (!stage[word_bits].valid) begin
            pipeline_should_advance <= 1;
        end else if (ovalid && oready) begin
            pipeline_should_advance <= 1;
        end else begin
            pipeline_should_advance <= 0;
        end

        stage[-1].d = divisor;
        stage[-1].r = dividend;
        stage[-1].q = '0;
        stage[-1].valid = ivalid;
        stage[-1].metadata = imeta;



        iready = pipeline_should_advance;
        // i.e., ovalid reacts immediately to reset, not on the next cycle.
        ovalid = stage[word_bits].valid && (reset != reset_polarity);
        quotient = stage[word_bits].q;
        remainder = stage[word_bits].r;
        ometa = stage[word_bits].metadata;
    end

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin

            for (int unsigned i = 0; i <= word_bits; i++) begin
                stage[i].d <= '0;
                stage[i].r <= '0;
                stage[i].q <= '0;
                stage[i].valid <= '0;
            end
        end else begin
            if (pipeline_should_advance) begin
                for (int unsigned i = 0; i <= word_bits; i++) begin
                    stage[i].metadata <= stage[i - 1].metadata;
                    if (stage[i - 1].d > stage[i - 1].r) begin
                        stage[i].r <= stage[i - 1].r;
                        stage[i].d <= stage[i - 1].d >> 1;
                        stage[i].q <= {stage[i - 1].q[word_bits-1-:word_bits-1], 1'b0};
                    end else begin
                        stage[i].r <= stage[i - 1].r - stage[i - 1].d;
                        stage[i].d <= stage[i - 1].d >> 1;
                        stage[i].q <= {stage[i - 1].q[word_bits-1-:word_bits-1], 1'b1};
                    end
                end
            end
        end
    end

endmodule : pipelined_divmod