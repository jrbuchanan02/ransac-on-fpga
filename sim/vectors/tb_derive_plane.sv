`timescale 1ns / 1ps

`include "vectors/vector_pkg.svh"
`include "simulated_point_trios.svh"

module tb_derive_plane;

    logic clock;

    logic dut_clock;
    logic dut_reset;
    logic dut_ivalid;
    logic dut_iready;
    vector::point_t dut_a;
    vector::point_t dut_b;
    vector::point_t dut_c;
    logic dut_ovalid;
    logic dut_oacknowledge;
    vector::vector3s_s dut_n;
    vector::single_t dut_d;
    vector::derive_plane_status_e dut_status;

    initial begin : manage_dut_clock
        clock = 1;
        forever begin
            clock = ~clock;
            # 5;
        end
    end : manage_dut_clock
    assign dut_clock = clock;

    initial begin : manage_dut_reset
        dut_reset = 1;
        @(posedge dut_clock);
        @(negedge dut_clock);
        @(posedge dut_clock) dut_reset = 0;
    end : manage_dut_reset

    logic signed [31:0] error_scratch_var;

    // run through the test.
    initial begin : run_test
        dut_ivalid = 0;
        dut_a = '0;
        dut_b = '0;
        dut_c = '0;
        dut_oacknowledge = 1;
        @(negedge dut_reset);
        for (int unsigned i = 0; i < simulated_point_trios::group_size;) begin : trios
            @(negedge dut_clock) begin
                dut_ivalid = 0;

                // finished trio, check results.
                if (dut_ovalid) begin
                    if (dut_status != vector::DERIVE_PLANE_STATUS_SUCCESS) begin
                        $error("Failed to give result when one was expected. Point A: %p; Point B: %p; Point C: %p\n", dut_a, dut_b, dut_c);
                        $finish(2);
                    end
                    // compare actual vs expected.
                    for (int j = 0; j < 3; j++) begin
                        error_scratch_var = dut_n.c[j] - simulated_point_trios::n_trios[i][j];
                        if (error_scratch_var < 0) begin
                            error_scratch_var = -error_scratch_var;
                        end

                        if (error_scratch_var > simulated_point_trios::tolerance) begin
                            $error("Too much error: component %d has a value of %x which is different enough from ground truth value %x to exceed threshold of +/- %x", j, dut_n.c[j], simulated_point_trios::n_trios[i][j], simulated_point_trios::tolerance);
                            $finish(2);
                        end
                    end

                    error_scratch_var = dut_d - simulated_point_trios::d_trios[i];
                    if (error_scratch_var < 0) begin
                        error_scratch_var = -error_scratch_var;
                    end

                    if (error_scratch_var > simulated_point_trios::tolerance) begin
                        $error("Too much error: plane distance has a value of %x which is different enough from ground truth value %x to exceed threshold of +/- %%x", dut_d, simulated_point_trios::d_trios[i], simulated_point_trios::tolerance);
                        $finish(2);
                    end
                    // if still here, set up the next calculation.
                    i++;
                end else if (dut_iready) begin
                    // submit calculation.
                    for (int unsigned j = 0; j < 3; j++) begin
                        dut_a.c[j] = simulated_point_trios::a_trios[i][j];
                        dut_b.c[j] = simulated_point_trios::b_trios[i][j];
                        dut_c.c[j] = simulated_point_trios::c_trios[i][j];
                        dut_ivalid = 1;
                    end
                end
            end
        end : trios
        for (int unsigned i = 0; i < simulated_point_trios::group_size;) begin : pairs
            @(negedge dut_clock) begin
                dut_ivalid = 0;

                // finished 'trio' of two unique points, check results.
                if (dut_ovalid) begin
                    if (dut_status != vector::DERIVE_PLANE_STATUS_LESS_THAN_THREE_UNIQUE_POINTS) begin
                        $error("Incorrectly gave result when an error was expected. Point A: %p; Point B: %p; Point C: %p\n", dut_a, dut_b, dut_c);
                        $finish(2);
                    end
                    // if still here, set up the next calculation.
                    i++;
                end else if (dut_iready) begin
                    // submit calculation.
                    for (int unsigned j = 0; j < 3; j++) begin
                        dut_a.c[j] = simulated_point_trios::a_pairs[i][j];
                        dut_b.c[j] = simulated_point_trios::b_pairs[i][j];
                        dut_c.c[j] = simulated_point_trios::c_pairs[i][j];
                        dut_ivalid = 1;
                    end
                end
            end
        end : pairs
        for (int unsigned i = 0; i < simulated_point_trios::group_size;) begin : singles
            @(negedge dut_clock) begin
                dut_ivalid = 0;

                // finished 'trio' of two unique points, check results.
                if (dut_ovalid) begin
                    if (dut_status != vector::DERIVE_PLANE_STATUS_LESS_THAN_THREE_UNIQUE_POINTS) begin
                        $error("Incorrectly gave result when an error was expected. Point A: %p; Point B: %p; Point C: %p\n", dut_a, dut_b, dut_c);
                        $finish(2);
                    end
                    // if still here, set up the next calculation.
                    i++;
                end else if (dut_iready) begin
                    // submit calculation.
                    for (int unsigned j = 0; j < 3; j++) begin
                        dut_a.c[j] = simulated_point_trios::singles[i][j];
                        dut_b.c[j] = simulated_point_trios::singles[i][j];
                        dut_c.c[j] = simulated_point_trios::singles[i][j];
                        dut_ivalid = 1;
                    end
                end
            end
        end : singles
        
        $display("Finished.");
        $finish(0);
    end : run_test

    derive_plane#(
        .latency_fma(vector::fma_latency_singles),
        .reset_polarity(1)
    ) dut(
        .clock(dut_clock),
        .reset(dut_reset),
        .ivalid(dut_ivalid),
        .iready(dut_iready),
        .a(dut_a),
        .b(dut_b),
        .c(dut_c),
        .ovalid(dut_ovalid),
        .oacknowledge(dut_oacknowledge),
        .n(dut_n),
        .d(dut_d),
        .status(dut_status)
    );
endmodule : tb_derive_plane