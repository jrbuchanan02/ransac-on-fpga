`timescale 1ns / 1ps

module random_in_range#(
    parameter int unsigned lfsr_register_width = 64,
    parameter int unsigned lfsr_window_width = 32,
    parameter int unsigned lfsr_window_start = 32,
    parameter logic [lfsr_register_width - 1:0] lfsr_polynomial = 64'hD800_0000_0000_0000,
    parameter logic [lfsr_register_width - 1:0] lfsr_seed = { $random(), $random() },
    parameter bit reset_polarity = 1)(
    input logic clock,
    input logic reset,

    input logic ivalid,
    output logic iready,
    input logic [lfsr_window_width-1:0] base,
    input logic [lfsr_window_width-1:0] max_offset,

    output logic [lfsr_window_width-1:0] random,
    output logic [lfsr_window_width-1:0] random_base_param,
    output logic [lfsr_window_width-1:0] random_offset_param,
    output logic ovalid,
    input logic oready
    );

    logic [lfsr_window_width-1:0] lfsr_random;

    typedef struct packed {
        logic [lfsr_window_width-1:0] base;
        logic [lfsr_window_width-1:0] offset;
    } metadata_s;

    logic [lfsr_window_width-1:0] random_in_bounds;

    metadata_s imeta;
    metadata_s ometa;

    always_comb begin 
        imeta.base = base;
        imeta.offset = max_offset;
        random = random_in_bounds + ometa.base;
        random_base_param = ometa.base;
        random_offset_param = ometa.offset;
    end

    pipelined_divmod#(
        .word_bits(lfsr_window_width),
        .metadata_type(metadata_s),
        .metadata_on_reset('{base:'0, offset:'0}),
        .reset_polarity(reset_polarity)
    ) divmod(
        .clock(clock),
        .reset(reset),
        .ivalid(ivalid),
        .iready(iready),
        .divisor(max_offset),
        .dividend(lfsr_random),
        .imeta(imeta),
        .ovalid(ovalid),
        .oready(oready),
        .quotient( ),
        .remainder(random_in_bounds),
        .ometa(ometa)
    );

    lfsr#(
        .register_width(lfsr_register_width),
        .window_width(lfsr_window_width),
        .window_start(lfsr_window_start),
        .polynomial(lfsr_polynomial),
        .start_value(lfsr_seed),
        .reset_polarity(reset_polarity)
    ) prng_source(
        .clock(clock),
        .reset(reset),
        .random(lfsr_random)
    );

endmodule : random_in_range