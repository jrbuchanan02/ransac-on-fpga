`timescale 1ns / 1ps

module fp_fma#(
        parameter bit reset_polarity = 1'b1,
        parameter int unsigned ibits = 12,
        parameter int unsigned fbits = 20,
        parameter int unsigned id_bits = 8,
        parameter int unsigned latency = 8,
        parameter int unsigned add_latency = 0
    )(
        input logic clock,
        input logic reset,

        input logic [ibits+fbits-1:0] a,
        input logic [ibits+fbits-1:0] b,
        input logic [2*(ibits+fbits)-1:0] c,

        // 2 * width of a, b, or c to prevent overflow on multiplication
        // 1 more bit to prevent overflow on addition
        // +1-1 (instead of just nothing) to keep congruence with the -1 in the other types
        output logic [2*(ibits+fbits)+1-1:0] r,

        input logic [id_bits-1:0] iid,

        output logic [id_bits-1:0] oid,

        input logic ivalid,
        output logic iready,

        output logic ovalid,
        input logic oacknowledge
    );

    typedef logic [ibits+fbits-1:0] single_t;
    typedef logic [2*(ibits+fbits)-1:0] double_t;
    typedef logic [2*(ibits+fbits)+1-1:0] result_t;

    localparam bits_in_single = ibits+fbits;
    localparam bits_in_double = 2 * bits_in_single;
    localparam bits_in_result = bits_in_double + 1;

    typedef struct packed {
        double_t ab;
        double_t c;

        logic [id_bits-1:0] id;

        logic valid;

    } stage_s;


    stage_s stages[latency-1:0];


    // some combinatorial things

    logic pipeline_should_advance;
    // pipeline should advance when the stage we're outputting is invalid
    // (either from reset or from stall stage) or when the stage is valid
    // and the input is acknowledged
    always_comb begin
        if (!stages[latency-1].valid) begin
            pipeline_should_advance = 1;
        end else if (oacknowledge) begin
            pipeline_should_advance = 1;
        end else begin
            pipeline_should_advance = 0;
        end
    end

    // things for if there is an add latency

    generate

    if (add_latency == 0) begin : no_add_latency
        always @(posedge clock) begin : update_outputs
            ovalid <= stages[latency-1].valid;
            r <= $signed(stages[latency-1].ab) + $signed(stages[latency-1].c);
            oid <= stages[latency-1].id;
        end : update_outputs
    end : no_add_latency
    else begin : manage_add_latency
        typedef struct packed {
            result_t r;
            logic valid;
            logic [id_bits-1:0] id;
        } add_stage_s;

        add_stage_s add_stages[add_latency-1:0];

        always @(posedge clock) begin : advance_add_pipeline
            ovalid <= add_stages[add_latency-1].valid;
            r <= add_stages[add_latency-1].r;
            oid <= add_stages[add_latency-1].id;

            if (pipeline_should_advance) begin
                add_stages[0].valid <= stages[latency-1].valid;
                add_stages[0].r <= $signed(stages[latency-1].ab) + $signed(stages[latency-1].c);
                add_stages[0].id <= stages[latency-1].id;
                for (int unsigned i = 1; i < latency; i++) begin
                    add_stages[i] <= add_stages[i - 1];
                end
            end
        end : advance_add_pipeline
    end : manage_add_latency

    endgenerate

    always @(posedge clock) begin
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < latency; i++) begin
                stages[i].valid <= 0;
            end
            iready <= 1;
        end
        else begin : main_logic
            if (pipeline_should_advance) begin : advance_pipeline
                // advance the pipeline stages
                
                // update iready
                iready <= 1;

                // special case for stage 0
                stages[0].ab <= $signed(a) * $signed(b);
                stages[0].c <= c;
                stages[0].id <= iid;
                stages[0].valid <= ivalid;

                for (int unsigned i = 1; i < latency; i++) begin
                    stages[i] <= stages[i - 1];
                end

            end : advance_pipeline
            else begin
                iready <= 0;
            end
        end : main_logic
    end

endmodule : fp_fma