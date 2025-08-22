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

    // "width" is one bit longer so that the carry out is just one bit more
    // significant than res in the full_res variable for a pipeline stage.
    localparam int unsigned effective_width = width + 1;
    localparam int unsigned bits_per_cycle = effective_width / latency + (effective_width % latency != 0);

    typedef struct packed {
        struct packed {
            logic [bits_per_cycle-1:0] lhs;
            logic [bits_per_cycle-1:0] rhs;
            logic icarry;
            logic ocarry;
            logic [bits_per_cycle-1:0] res;
        } adder_vars;
        struct packed {
            logic [width-1:0] lhs;
            logic [width-1:0] rhs;
            logic [bits_per_cycle:0] found_res;
            logic [latency * bits_per_cycle -1:0] full_res;
            logic ocarry;
        } data;
        metadata_type meta;
        logic valid;
    } stage_s;

    stage_s [latency-1:-1] stage;
    logic pipeline_should_advance;

    always_comb begin
        if (!stage[latency-1].valid) begin
            pipeline_should_advance <= 1;
        end else if (ovalid && oready) begin
            pipeline_should_advance <= 1;
        end else begin
            pipeline_should_advance <= 0;
        end

        stage[-1].adder_vars.lhs = lhs[bits_per_cycle-1-:bits_per_cycle];
        stage[-1].adder_vars.rhs = rhs[bits_per_cycle-1-:bits_per_cycle];
        stage[-1].adder_vars.icarry = icarry;
        stage[-1].data.lhs = lhs;
        stage[-1].data.rhs = rhs;
        stage[-1].data.found_res = '0;
        stage[-1].data.full_res = '0;
        stage[-1].data.ocarry = '0;
        stage[-1].meta = imeta;
        stage[-1].valid = ivalid;

        iready = pipeline_should_advance;
        ovalid = stage[latency-1].valid && (reset != reset_polarity);
        res = stage[latency-1].data.full_res[width-1:0];
        ocarry = stage[latency-1].data.full_res[width];
        ometa = stage[latency-1].meta;

        for (int unsigned i = 0; i < latency; i++) begin
            stage[i].data.found_res = { stage[i].adder_vars.ocarry, stage[i].adder_vars.res };
            stage[i].full_res[i * bits_per_cycle + bits_per_cycle-1-:bits_per_cycle] = stage[i].data.found_res;
            if (i > 0) begin
                stage[i].full_res[(i - 1) * bits_per_cycle + bits_per_cycle-1-:(i - 1) * bits_per_cycle] = stage[i - 1].data.found_res[(i - 1) * bits_per_cycle + bits_per_cycle - 1-:(i - 1) * bits_per_cycle];
            end
        end
    end

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < latency; i++) begin
                stage[i].adder_vars.lhs <= '0;
                stage[i].adder_vars.rhs <= '0;
                stage[i].adder_vars.icarry <= '0;
                stage[i].data.lhs <= '0;
                stage[i].data.rhs <= '0;
            end
        end else begin
            if (pipeline_should_advance) begin
                for (int unsigned i = 0; i < latency; i++) begin
                    stage[i] <= stage[i - 1];
                end
            end
        end
    end

    genvar i;
    generate
        for (i = 0; i < latency; i++) begin
            adder_in_chain#(
                .width(bits_per_cycle)
            ) adder(
                .lhs(stage[i].adder_vars.lhs),
                .rhs(stage[i].adder_vars.rhs),
                .icarry(stage[i].adder_vars.icarry),
                .res(stage[i].adder_vars.res),
                .ocarry(stage[i].adder_vars.ocarry),
            );
        end
    endgenerate
endmodule : pipelined_adder