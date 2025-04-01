`timescale 1ns/1ps

// ransac_logic: given some memory of points, outputs a plane to describe the ground plane
// according to a random sample consensus

// a note on how this logic works:
//
// 1. points loaded in may be out of bounds, in this instance a response must exist but the
//    logic here (should) be smart enough to ignore the nonexistent point received.
module ransac_logic#(
        parameter int unsigned point_pipeline_multiply_latency = ransac_fixed::value_bits() / 8,
        parameter bit point_pipeline_addition_has_latency = 1,
        parameter int unsigned plane_calculate_multiply_latency = ransac_fixed::value_bits() / 16,
        parameter int unsigned rsqrt_iterations = 100,
        parameter bit reset_polarity = 1,
        parameter int unsigned max_point_count = 1 << 17,
        // these seed defaults are chosen since they have an equal number of 1's and 0's
        // 3 LFSR's are used instead of 1 to minimize the output correlation between the
        // random numbers generated
        parameter logic [63:0] lfsr_0_seed = 64'hA5A5_5A5A_A5A5_5A5A,
        parameter logic [63:0] lfsr_1_seed = 64'h8787_C33C_1EE1_2D2D,
        parameter logic [63:0] lfsr_2_seed = 64'h3333_CCCC_AAAA_5555
        )(
        input logic clock,
        input logic rng_clock,
        input logic reset,

        input ransac_fixed::point_t point_in,
        output logic point_addr_valid,
        input logic point_data_valid,
        output logic [$clog2(max_point_count)-1:0] point_addr,
        input logic [$clog2(max_point_count)-1:0] point_count,
        input logic [31:0] iterations,
        input logic calculation_start,
        input ransac_fixed::fixed_t threshold,
        output logic calculation_can_start,
        output logic calculation_done,
        output ransac_fixed::plane_t plane
    );


    typedef logic [$clog2(max_point_count)-1:0] point_addr_t;
    typedef point_addr_t point_size_t;

    point_addr_t ransac_point_addrs[2:0];

    localparam logic [63:0] lfsr_seeds[2:0] = '{
        lfsr_0_seed,
        lfsr_1_seed,
        lfsr_2_seed
    };
    
    point_size_t best_plane_inlier_count;
    point_size_t curr_plane_inlier_count;
    ransac_fixed::plane_t best_plane;
    ransac_fixed::plane_t curr_plane;

    typedef struct packed {
        logic point_exists; // if 1, then the address < point_count
        logic entry_is_stall;   // if 1, then this point doesn't exist because
                                // the point cloud memory hasn't responded yet.
    } pipeline_t;

    enum {
        IDLE,   // no calculation, calculation_can_start = 1
                // keep plane and calculation_done
        CAPTURE_PLANE_ADDRS,    // determine addresses of the points (modulo by point_count)
        LOAD_PLANE_POINT_A_INIT,    // set up addr
        LOAD_PLANE_POINT_A_WAIT,    // wait on data
        LOAD_PLANE_POINT_B_INIT,    // set up addr
        LOAD_PLANE_POINT_B_WAIT,    // wait on data
        LOAD_PLANE_POINT_C_INIT,    // set up addr
        LOAD_PLANE_POINT_C_WAIT,    // wait on data, the cycle we're done, start finding the plane
        FIND_CURR_PLANE_WAIT,       // wait on the plane calculation
        FEED_NEW_POINT_ADDR,    // set up the point address
        FEED_NEW_POINT_READ,    // read in the new point
        FEED_NEW_POINT_WAIT,    // after the last point which exists, wait on all the points
                                // remaining in the pipeline
        FEED_NEW_POINT_LOOP     // keep the better of the best_plane and curr_plane, then
                                // advance to the next iteration
    } state;

    ransac_fixed::point_t submitted_test_point;
    ransac_fixed::plane_t submitted_test_plane;
    ransac_fixed::fixed_t submitted_new_distance;
    pipeline_t submitted_pipeline_i;
    pipeline_t submitted_pipeline_o;

    ransac_fixed::point_t find_plane_a;
    ransac_fixed::point_t find_plane_b;
    ransac_fixed::point_t find_plane_c;
    logic find_plane_input_valid;
    logic find_plane_input_ready;
    logic find_plane_output_valid;
    ransac_fixed::plane_t find_plane_p;

    logic [63:0] raw_rng[2:0];

    logic [31:0] iteration_number;
    point_size_t point_number;

    struct packed {
        logic [31:0] iterations;
        point_size_t point_count;
        ransac_fixed::fixed_t threshold;
    } stored_vars;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            calculation_can_start <= 1;
            state <= IDLE;
            point_addr_valid <= 0;
            find_plane_input_valid <= 0;
            submitted_pipeline_i.point_exists <= 0;
        end else begin
            case (state)
            IDLE: begin
                point_addr_valid <= 0;
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                stored_vars.iterations <= iterations;
                stored_vars.point_count <= point_count;
                stored_vars.threshold <= threshold;
                if (calculation_start) begin
                    calculation_done <= 0;
                    calculation_can_start <= 0;
                    state <= CAPTURE_PLANE_ADDRS;
                    iteration_number <= 0;
                end else begin
                    calculation_can_start <= 1;
                    best_plane_inlier_count <= 0;
                    curr_plane_inlier_count <= 0;
                end
            end
            CAPTURE_PLANE_ADDRS: begin
                point_addr_valid <= 0;
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                for (int i = 0; i < 3; i++) begin
                    ransac_point_addrs[i] <= raw_rng[i][$clog2(max_point_count)-1:0];
                end
                submitted_pipeline_i.point_exists <= 0;
            end
            LOAD_PLANE_POINT_A_INIT: begin
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                point_addr_valid <= 1;
                point_addr <= ransac_point_addrs[0] % stored_vars.point_count;
                state <= LOAD_PLANE_POINT_A_WAIT;
            end
            LOAD_PLANE_POINT_A_WAIT: begin
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                find_plane_a <= point_in;
                if (point_data_valid) begin
                    point_addr_valid <= 0;
                    state <= LOAD_PLANE_POINT_B_INIT;
                end else begin
                    point_addr_valid <= 1;
                    point_addr <= ransac_point_addrs[0] % stored_vars.point_count;
                    state <= LOAD_PLANE_POINT_A_WAIT;
                end
            end
            LOAD_PLANE_POINT_B_INIT: begin
                point_addr_valid <= 1;
                point_addr <= ransac_point_addrs[1] % stored_vars.point_count;
                state <= LOAD_PLANE_POINT_B_WAIT;
            end
            LOAD_PLANE_POINT_B_WAIT: begin
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                find_plane_b <= point_in;
                if (point_data_valid) begin
                    point_addr_valid <= 0;
                    state <= LOAD_PLANE_POINT_C_INIT;
                end else begin
                    point_addr_valid <= 1;
                    point_addr <= ransac_point_addrs[1] % stored_vars.point_count;
                    state <= LOAD_PLANE_POINT_B_WAIT;
                end
            end
            LOAD_PLANE_POINT_C_INIT: begin
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                point_addr_valid <= 1;
                point_addr <= ransac_point_addrs[2] % stored_vars.point_count;
                state <= LOAD_PLANE_POINT_C_WAIT;
            end
            LOAD_PLANE_POINT_C_WAIT: begin
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                find_plane_c <= point_in;
                if (point_data_valid) begin
                    point_addr_valid <= 0;
                    find_plane_input_valid <= 1;
                    state <= FIND_CURR_PLANE_WAIT;
                end
            end
            FIND_CURR_PLANE_WAIT: begin
                curr_plane <= find_plane_p;
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 0;
                curr_plane_inlier_count <= 0;
                if (find_plane_output_valid) begin
                    state <= FEED_NEW_POINT_ADDR;
                    iteration_number <= iteration_number + 1;
                    point_number <= 0;
                end
            end
            FEED_NEW_POINT_ADDR: begin
                point_addr <= point_number;
                point_addr_valid <= 1;
                find_plane_input_valid <= 0;
                submitted_pipeline_i.point_exists <= 1;
                submitted_pipeline_i.entry_is_stall <= 1;
                state <= FEED_NEW_POINT_READ;

                if (submitted_pipeline_o.point_exists && !submitted_pipeline_o.entry_is_stall) begin
                    if (submitted_new_distance <= stored_vars.threshold) begin
                        curr_plane_inlier_count <= curr_plane_inlier_count + 1;
                    end
                end
            end
            FEED_NEW_POINT_READ: begin
                submitted_test_plane <= curr_plane;
                submitted_test_point <= point_in;
                submitted_pipeline_i.point_exists <= point_number < stored_vars.point_count;
                if (point_data_valid) begin
                    point_addr_valid <= 0;
                    submitted_pipeline_i.entry_is_stall <= 0;
                    point_number <= point_number + 1;
                    if (point_number <= stored_vars.point_count) begin
                        state <= FEED_NEW_POINT_ADDR;
                    end begin
                        state <= FEED_NEW_POINT_WAIT;
                    end
                end else begin
                    point_addr_valid <= 1;
                    submitted_pipeline_i.entry_is_stall <= 1;
                end

                if (submitted_pipeline_o.point_exists && !submitted_pipeline_o.entry_is_stall) begin
                    if (submitted_new_distance <= stored_vars.threshold) begin
                        curr_plane_inlier_count <= curr_plane_inlier_count + 1;
                    end
                end
            end
            FEED_NEW_POINT_WAIT: begin
                submitted_test_plane <= curr_plane;
                submitted_test_point <= point_in;
                submitted_pipeline_i.point_exists <= 0;
                submitted_pipeline_i.entry_is_stall <= 0;

                if (submitted_pipeline_o.point_exists && !submitted_pipeline_o.entry_is_stall) begin
                    if (submitted_new_distance <= stored_vars.threshold) begin
                        curr_plane_inlier_count <= curr_plane_inlier_count + 1;
                    end
                end

                // this check is ok without any extra data in the pipeline structure so long as
                // reading all the points takes longer than getting the first point
                if (!submitted_pipeline_o.point_exists) begin
                    state <= FEED_NEW_POINT_LOOP;
                end
            end
            FEED_NEW_POINT_LOOP: begin
                // keep the better plane

                if (curr_plane_inlier_count > best_plane_inlier_count) begin
                    best_plane_inlier_count <= curr_plane_inlier_count;
                    best_plane <= best_plane;
                end

                iteration_number <= iteration_number + 1;
                point_number <= 0;

                // set up another iteration
                if (iteration_number < stored_vars.iterations) begin
                    state <= CAPTURE_PLANE_ADDRS;
                end else begin
                    // if no more iterations, then we're done
                    
                    // remember that the assignment to best_plane was non-blocking
                    if (curr_plane_inlier_count > best_plane_inlier_count) begin
                        plane <= curr_plane; 
                    end else begin
                        plane <= best_plane;
                    end

                    calculation_done <= 1;
                    calculation_can_start <= 1;
                    state <= IDLE;
                end
            end
            endcase
        end
    end

    fast_point_distance_to_plane#(
        .external_pipeline(pipeline_t),
        .multiply_latency(point_pipeline_multiply_latency),
        .addition_has_latency(point_pipeline_addition_has_latency)
    ) distance_to_plane_test(
        .clock(clock),
        .point(submitted_test_point),
        .plane(submitted_test_plane),
        .distance(submitted_new_distance),
        .pipeline_i(submitted_pipeline_i),
        .pipeline_o(submitted_pipeline_o)
    );

    find_plane#(
        .multiply_latency(plane_calculate_multiply_latency),
        .rsqrt_iterations(rsqrt_iterations),
        .reset_polarity(1)
    ) find_plane_calc(
        .clock(clock),
        .reset(reset),
        .a(find_plane_a),
        .b(find_plane_b),
        .c(find_plane_c),
        .input_valid(find_plane_input_valid),
        .input_ready(find_plane_input_ready),
        .output_valid(find_plane_output_valid),
        .p(find_plane_p)
    );

    genvar i;
    generate
        for(i = 0; i < 3; i++) begin
            lfsr#(
                .seed(lfsr_seeds[i])
            ) rng(
                .rng_clock(rng_clock),
                .rng_reset(reset),
                .read_clock(clock),
                .random_value(raw_rng[i])
            );
        end
    endgenerate

endmodule : ransac_logic