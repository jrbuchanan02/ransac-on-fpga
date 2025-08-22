`timescale 1ns / 1ps

module pipelined_adder#(
    parameter int unsigned width = 32,
    parameter int unsigned latency = 2,
    parameter type metadata_type = logic,
    parameter metadata_type default_metadata = '0,
    parameter bit reset_polarity = 1
    )(
    input logic clock,
    input logic reset,
    input logic ivalid,
    output logic iready,
    input metadata_type imeta,
    input logic [width-1:0] lhs,
    input logic [width-1:0] rhs,
    input logic icarry,
    output logic ovalid,
    input logic oready,
    output logic ometa,
    output logic [width-1:0] res,
    output logic ocarry
    );

    struct packed {
        logic [width-1:0] lhs;
        logic [width-1:0] rhs;
        logic carry;
        logic [width:0] res;
        logic valid;
        metadata_type metadata;
    } stage[latency-1:-1];

    localparam int unsigned bits_per_cycle = width / latency;

    logic [latency-1:0] [bits_per_cycle:0] partial_sum;

    logic pipeline_should_advance;

    always_comb begin
        if (!stage[latency-1].valid) begin
            pipeline_should_advance <= 1;
        end else if (ovalid && oready) begin
            pipeline_should_advance <= 1;
        end else begin
            pipeline_should_advance <= 0;
        end

        stage[-1].lhs = lhs;
        stage[-1].rhs = rhs;
        stage[-1].carry = icarry;
        stage[-1].res = '0;
        stage[-1].valid = ivalid;
        stage[-1].metadata = imeta;

        iready = pipeline_should_advance;
        ovalid = stage[latency-1].valid && (reset != reset_polarity);
        res = stage[latency-1].res[width-1:0];
        ocarry = stage[latency-1].res[width];
        ometa = stage[latency-1].metadata;

        for (int unsigned i = 0; i < latency; i++) begin
            partial_sum[i] = 
                stage[i - 1].lhs[i * bits_per_cycle + bits_per_cycle - 1-:bits_per_cycle] + 
                stage[i - 1].rhs[i * bits_per_cycle + bits_per_cycle - 1-:bits_per_cycle] + 
                stage[i - 1].carry;
        end
    end

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < latency; i++) begin
                stage[i] <= '{
                    lhs : '0,
                    rhs : '0,
                    carry : '0,
                    res : '0,
                    valid : '0,
                    metadata : default_metadata
                };
            end
        end else begin
            if (pipeline_should_advance) begin
                for (int unsigned i = 0; i < latency; i++) begin
                    stage[i].lhs <= stage[i - 1].lhs;
                    stage[i].rhs <= stage[i - 1].rhs;
                    stage[i].carry <= partial_sum[i][bits_per_cycle];
                    stage[i].res <= stage[i - 1].res | (partial_sum[i][bits_per_cycle-1:0] << (i * bits_per_cycle));
                    stage[i].valid <= stage[i - 1].valid;
                    stage[i].metadata <= stage[i - 1].metadata;
                    
                end
            end
        end
    end
endmodule : pipelined_adder