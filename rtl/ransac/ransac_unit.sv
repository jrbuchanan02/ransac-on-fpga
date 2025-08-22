`timescale 1ns / 1ps

/**
 * Module which actually performs the entire RANSAC algorithm. 
 *
 * This module has two+ AXI ports: a control port and one or more memory ports.
 *
 * The control port is used for controlling the module (directing it to a start
 * address for the cloud, commanding it to start calculations, and being able to
 * check when the unit has finished processing.
 * 
 * The ransac unit has a configurable numbre of plane checking units and a 
 * configurable number of check_inlier units per plane checking unit. It also
 * has a configurable latency for all types of FMA unit in the module and a 
 * configurable quantity of LFSR units (the more LFSR units, the more points can
 * be requested per cycle).
 *
 * The control port only accepts control_data_width-aligned accesses.
 * Control port memory layout. Each entry is in order and has a width equal to 
 * the smallest multiple of control_data_width to contain all bits in the field.
 * The strobe fields are always one multiple of control_data_width in size.
 *
 * most read / write fields are read-only while calculations are active, writing
 * to these fields while calculations are running will be silently discarded.
 *
 * - calculation start strobe (write only, any write starts calculations if the
 *                             unit's current state so allows)
 * - calculation abort strobe (write only, any write aborts calculations if the
 *                             unit's current state so allows)
 * - calculation state (read only, bit 0 -> is 1 when active, bit 1 is 1 when 
 *                      processing an abort, bit 2 is 1 when finalizing a 
 *                      calculation normally. All other bits read 0)
 * - cloud base (read / write, writes blocked when active) -> the base address of
 *              the point cloud in memory. (memory_adddr_width bits wide)
 * - cloud size (read / write, writes blocked when active) -> the number of points
 *              found in the cloud. (clog2(maximum_point_count) bits wide)
 * - requested iteration count (read / write, writes blocked when active) -> number
 *              of requested iterations for the calculation. (clog2(maximum_iteration_count) bits wide)
 * - processed iteration count (read only) -> number of iterations run since the 
 *              most recent reset event or calculation start.
 * - inlier threshold (read / write, writes blocked when active) -> normalized 
 *              distance between a point in the cloud and a candidate ground plane
 *              stored as vector::single_t (vector::bits_in_single bits wide)
 * - ground plane inlier count (read only) -> quantity of inliers in the ground
 *              plane. (1 bit longer than cloud size to allow the sensical reading
 *              for when all points are inliers to some plane)
 * - ground plane normal vector x component (read only) -> x component of the 
 *              ground plane's normal vector (vector:;bits_in_single bits wide).
 *              note that this vector is not normalized.
 * - ground plane normal vector y component (read only) -> y component of the 
 *              ground plane's normal vector (vector::bits_in_single bits wide).
 * - ground plane normal vector z component (read only) -> z component of the
 *              ground plane's normal vector (vector::bits_in_single bits wide).
 * - ground plane distance to origin (read only) -> given plane normal N, some 
 *              point on the plane P, and this value D, the equation N dot P = D
 *              holds true. Separately, D = distance from the origin to the
 *              point on the plane closest to the origin.
 */
module ransac_unit#(
    parameter int unsigned memory_addr_width = 32,
    parameter int unsigned memory_data_width = vector::bits_in_single,
    parameter int unsigned memory_maximum_latency = 1 << 10,
    parameter int unsigned maximum_point_count = 1 << 20,
    parameter int unsigned maximum_iteration_count = 1 << 20,
    parameter logic [31:0] memory_rid_base = 32'h0000_0000,
    parameter int unsigned control_addr_width = 32,
    parameter int unsigned control_data_width = 32,
    parameter int unsigned plane_check_unit_count = 2,
    parameter int unsigned check_inlier_units_per_plane_check_unit = 2,
    parameter int unsigned small_fma_latency = vector::fma_latency_singles,
    parameter int unsigned large_fma_latency = vector::fma_latency_doubles,
    parameter int unsigned plane_fma_latency = vector::fma_latency_doubles,
    parameter int unsigned lfsr_register_width = 64,
    parameter int unsigned lfsr_window_width = 32,
    parameter int unsigned lfsr_window_start = 16,
    parameter logic [lfsr_register_width-1:0] lfsr_seed = {
        // seed chosen arbitrarily from the values which, given the default
        // polynomial, do not place the LFSR in an infinite loop.
        //
        // $random() was considered for the defualt value of this parameter but
        // Vivado synthesis would make $random return 0 for this parameter (which
        // would place the LFSR in an infinite loop)
        64'h0123_4567_89AB_CDEF
    },
    parameter logic [lfsr_register_width-1:0] lfsr_polynomial = 64'hD800_0000_0000_0000,
    parameter bit [control_addr_width-1:0] control_addr_base = 32'h4000_0000,
    parameter bit reset_polarity = 1)(
    input logic clock,
    input logic reset,

    // control port (responds via AXI Lite)
    input logic [control_addr_width-1:0] control_awaddr,
    input logic control_awvalid,
    output logic control_awready,

    input logic [control_addr_width-1:0] control_araddr,
    input logic control_arvalid,
    output logic control_arready,

    input logic [control_data_width-1:0] control_wdata,
    input logic [control_data_width/8-1:0] control_wstrb,
    input logic control_wvalid,
    output logic control_wready,

    output logic [control_data_width-1:0] control_rdata,
    output logic [1:0] control_rresp,
    output logic control_rvalid,
    input logic control_rready,

    output logic [1:0] control_bresp,
    output logic control_bvalid,
    input logic control_bready,

    // memory port

    // Note: we do not use the write port but it still has to exist.
    output logic [memory_addr_width-1:0] memory_awaddr,
    output logic [31:0] memory_awid,
    output logic memory_awvalid,
    output logic [2:0] memory_awprot,
    input logic memory_awready,

    // Note: we will heavily use the read port
    output logic [memory_addr_width-1:0] memory_araddr,
    output logic [31:0] memory_arid,
    output logic memory_arvalid,
    output logic [2:0] memory_arprot,
    input logic memory_arready,

    // Note: we will not use the write port but it still has to exist
    output logic [memory_data_width-1:0] memory_wdata,
    output logic [memory_data_width/8-1:0] memory_wstrb,
    output logic [31:0] memory_wid,
    output logic memory_wvalid,
    output logic memory_wlast,
    input logic memory_wready,

    input logic [memory_data_width-1:0] memory_rdata,
    input logic [1:0] memory_rresp,
    input logic [31:0] memory_rid,
    input logic memory_rvalid,
    output logic memory_rready,

    input logic [1:0] memory_bresp,
    input logic [31:0] memory_bid,
    input logic memory_bvalid,
    output logic memory_bready

    );

    struct packed {
        logic ivalid;
        logic iready;
        logic [$clog2(maximum_point_count)-1:0] cloud_length;
        logic [2:0][$clog2(maximum_point_count)-1:0] plane_point_start_offsets;
        vector::single_t threshold;
        logic ovalid;
        logic oacknowledge;
        logic [$clog2(maximum_point_count):0] inlier_count;
        ransac::plane_checking_unit_status_e status;
        vector::vector3s_s plane_n;
        vector::single_t plane_d;
        logic [memory_addr_width-1:0] point_addr;
        logic point_addr_valid;
        logic point_addr_ready;
        logic [memory_data_width-1:0] point_data;
        logic [1:0] point_resp;
        logic point_data_valid;
        logic point_data_ready;
    } plane_checking_unit_port[plane_check_unit_count-1:0];

    struct packed {
        logic ivalid;
        logic iready;
        logic [lfsr_window_width-1:0] base;
        logic [lfsr_window_width-1:0] max_offset;
        
        logic [lfsr_window_width-1:0] random;
        // base -> 0 (plane_checking unit has no base address control so we
        // do that ourselves)
        logic [lfsr_window_width-1:0] random_base_param;
        // offset -> cloud length
        logic [lfsr_window_width-1:0] random_offset_param;
        logic ovalid;
        logic oready;
    } random_in_range_port;

    // returns the number of (control_data_width)-bit multiples required to 
    // store the memory variables. Storing the data in multiples of words allows
    // the control port to only allow word-aligned accesses (note that AXI 
    // requires the data width to be a power of 2, meaning that the control
    // data width can be assumed to be a power of 2).
    function automatic int unsigned word_aligned_width(input int unsigned bit_width);
    begin
        if (bit_width % control_data_width != 0) begin
            word_aligned_width = control_data_width * ((bit_width / control_data_width) + 1);
        end else begin
            word_aligned_width = control_data_width * (bit_width / control_data_width);
        end
    end
    endfunction : word_aligned_width

    localparam int unsigned possible_max_iteration_count = maximum_iteration_count + plane_check_unit_count;

    localparam int unsigned bits_for_memory_var_cloud_length = word_aligned_width($clog2(maximum_point_count));
    localparam int unsigned bits_for_memory_var_cloud_base = word_aligned_width(memory_addr_width);
    localparam int unsigned bits_for_memory_var_single_t = word_aligned_width(vector::single_fbits + vector::single_ibits);
    localparam int unsigned bits_for_memory_var_inlier_count = word_aligned_width($clog2(maximum_point_count+1));
    localparam int unsigned bits_for_memory_var_requested_iterations = word_aligned_width($clog2(maximum_iteration_count));
    localparam int unsigned bits_for_memory_var_processed_iterations = word_aligned_width($clog2(possible_max_iteration_count));

    // memory_vars as in "variables accessible from memory"
    typedef struct packed {
        // length of the cloud
        logic [bits_for_memory_var_cloud_length-1:0] cloud_length;
        // base address of the cloud in memory
        logic [bits_for_memory_var_cloud_base-1:0] cloud_base;
        // threshold to use for the calculation
        logic [bits_for_memory_var_single_t-1:0] inlier_threshold;
        // normal vector for the plane
        logic [2:0][bits_for_memory_var_single_t-1:0] derived_plane_normal;
        // distance from closest point on the plane to the origin to the origin
        logic [bits_for_memory_var_single_t-1:0] derived_plane_distance;
        // number of inliers to the plane in the cloud
        logic [bits_for_memory_var_inlier_count-1:0] derived_plane_inlier_count;
        // requested number of iterations (unit might run plane_check_unit count
        // more iterations than this number)
        logic [bits_for_memory_var_requested_iterations-1:0] requested_iterations;
        logic [bits_for_memory_var_processed_iterations-1:0] processed_iterations;
    } memory_vars_s;

    memory_vars_s memory_vars;

    // like in plane checking unit, multiple state machines are required to 
    // properly manage internal variables.
    //
    // - we need a control memory port state machine to manage control variables
    //   this state machine also needs a safeguard against editing control 
    //   variables while calculations are running (so its write logic needs to 
    //   support reading the calculation state).
    // - we need a main calculation logic state machine to manage interacting 
    //   with the plane checking unit calculation parameters and to tabulate 
    //   the results from these units
    // - we need what is essentially an AXI interconnect to control access to
    //   the memory port.
    // - we also need to keep track of the last 3 (unique) randomly generated
    //   point cloud offsets in the range of the current values of the point 
    //   cloud.
    //
    
    // logic managing the plane checking unit memory port and its multiplexing 
    // into RAM
    // (perhaps have some form of cache for the point cloud?)
    //
    // the actual AXI multiplexing part can be implemented relatively easily 
    // and even allowing for interleaved transactions (since AXI supports 
    // multiple transactions in flight and even supports completing transactions
    // out of order). We can simply use the read ID to determine which plane
    // checking unit the data is intended for This ID-based system works as AXI
    // intends memory to work, our logic is then simply about ensuring that the
    // different ports use the address lines one at a time. We can accomplish
    // this method by using a scoreboard to track the difference in number of 
    // transactions between units.

    struct packed {
        // minimum -> always 0
        // so:
        // - when unit is serviced and another unit has a score of 0, increment
        //   this score. Otherwise, decrement all other scores
        // - on reset, all scores are 0.
        // - prefer servicing the unit with the lowest score.
        logic [plane_check_unit_count-1:0] [31:0] scoreboard;
        logic [plane_check_unit_count-1:0] units_pending_read;
        logic [$clog2(plane_check_unit_count)-1:0] smallest_score_pending_read;
        logic [$clog2(plane_check_unit_count)-1:0] servicing_addr_for_unit;
        logic [$clog2(plane_check_unit_count)-1:0] servicing_data_for_unit;
        logic [$clog2(plane_check_unit_count)-1:0] transactions_in_flight;

        logic decrement_other_scores_on_service;
    } cloud_read_control;

    enum {
        CLOUD_READ_ADDR_IDLE,
        CLOUD_READ_ADDR_WAIT
    } cloud_read_addr_state;


    always_comb begin
        memory_awprot <= 3'b000;    // data, secure, unprivileged
        memory_arprot <= 3'b000;    // data, secure, unprivileged
        memory_wlast <= '0; // never writing.

        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            cloud_read_control.units_pending_read[i] <= plane_checking_unit_port[i].point_addr_valid;
        end

        cloud_read_control.smallest_score_pending_read = '0;
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            if (cloud_read_control.units_pending_read[i] && 
                cloud_read_control.scoreboard[i] < cloud_read_control.scoreboard[cloud_read_control.smallest_score_pending_read]) begin
                
                cloud_read_control.smallest_score_pending_read = i;
            end
        end

        cloud_read_control.decrement_other_scores_on_service = '1;
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            if (cloud_read_control.scoreboard[i] == 0) begin
                cloud_read_control.decrement_other_scores_on_service = '0;
            end
        end

        // always ready for read data (could perhaps cause issues if data not
        // always valid?)
        memory_rready <= '1;

        // handle read data valid signals
        if (memory_rvalid && memory_rready) begin
            // determine the matching plane checking unit based on the ID.
            for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                if (memory_rid == memory_rid_base + i) begin
                    plane_checking_unit_port[i].point_data_valid <= '1;
                end else begin
                    plane_checking_unit_port[i].point_data_valid <= 0;
                end
            end
        end else begin
            for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                plane_checking_unit_port[i].point_data_valid <= '0;
            end
        end
    end

    always @(posedge clock) begin
        if (reset == reset_polarity) begin
            cloud_read_addr_state <= CLOUD_READ_ADDR_IDLE;
            for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                cloud_read_control.scoreboard[i] = 0;
            end
            cloud_read_control.transactions_in_flight = '0;
        end else begin
            for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                plane_checking_unit_port[i].point_addr_ready = 0;
                plane_checking_unit_port[i].point_data = memory_rdata;
                plane_checking_unit_port[i].point_resp = memory_rresp;
            end
            case (cloud_read_addr_state)
            CLOUD_READ_ADDR_IDLE: begin

                if (cloud_read_control.units_pending_read != '0) begin
                    cloud_read_control.servicing_addr_for_unit = cloud_read_control.smallest_score_pending_read;
                    memory_araddr <= plane_checking_unit_port[cloud_read_control.servicing_addr_for_unit].point_addr + memory_vars.cloud_base;
                    memory_arvalid <= '1;
                    memory_arid <= memory_rid_base + cloud_read_control.servicing_addr_for_unit;
                    cloud_read_addr_state <= CLOUD_READ_ADDR_WAIT;
                end else begin
                    memory_arvalid <= '0;
                end
            end

            CLOUD_READ_ADDR_WAIT: begin

                if (memory_arready && memory_arvalid) begin
                    plane_checking_unit_port[cloud_read_control.servicing_addr_for_unit].point_addr_ready = '1;
                    memory_arvalid <= '0;
                    cloud_read_addr_state <= CLOUD_READ_ADDR_IDLE;
                    
                    if (cloud_read_control.decrement_other_scores_on_service) begin
                        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                            cloud_read_control.scoreboard[i] = cloud_read_control.scoreboard[i] - 1;
                        end
                    end
                    cloud_read_control.scoreboard[cloud_read_control.servicing_addr_for_unit] = cloud_read_control.scoreboard[cloud_read_control.servicing_addr_for_unit] + 1; 
                end
            end

            endcase

        end
    end

    struct packed {
        // TODO: perhaps change all_points_valid into a mask per unit for its
        // valid offsets?
        logic all_points_valid;
        logic [3 * plane_check_unit_count - 1:0][lfsr_window_width-1:0] offset;
        logic [3 * plane_check_unit_count - 1:0] offset_in_bounds;
        logic [plane_check_unit_count-1:0] point_trio_unique;
        logic [lfsr_window_width-1:0] most_recent_random;
    } plane_point_offset_control;

    always_comb begin : manage_plane_point_offset_signals
        random_in_range_port.base = 0;
        random_in_range_port.max_offset = memory_vars.cloud_length;
        random_in_range_port.ivalid = 1;
        random_in_range_port.oready = 1;

        plane_point_offset_control.most_recent_random = random_in_range_port.random;

        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            plane_point_offset_control.point_trio_unique[i] = 1;
            for (int unsigned j = 0; j < 3; j++) begin
                // modulo here should be ok since the loop will have to be
                // unrolled (hopefully exposing that the only case where the
                // modulo matters is the last one where we check if offset[i][2]
                // equals offset[i][0])
                if (plane_point_offset_control.offset[3 * i + j] == plane_point_offset_control.offset[3 * i + ((j + 1) % 3)]) begin
                    plane_point_offset_control.point_trio_unique[i] = 0;
                end

                if (plane_point_offset_control.offset[3 * i + j] < memory_vars.cloud_length) begin
                    plane_point_offset_control.offset_in_bounds[3 * i + j] = '1;
                end else begin
                    // length out of bounds, no longer valid.
                    plane_point_offset_control.offset_in_bounds[3 * i + j] = '0;
                end
            end
        end

        // assign offsets to the plane checking unit port
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            for (int unsigned j = 0; j < 3; j++) begin
                plane_checking_unit_port[i].plane_point_start_offsets[j] <= plane_point_offset_control.offset[3 * i + j];
            end
        end

        if (plane_point_offset_control.offset_in_bounds == '1 && plane_point_offset_control.point_trio_unique == '1) begin
            plane_point_offset_control.all_points_valid = 1;
        end else begin
            plane_point_offset_control.all_points_valid = 0;
        end
    end : manage_plane_point_offset_signals

    always_ff @(posedge clock) begin : plane_point_offset_advance_logic
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                for (int unsigned j = 0; j < 3; j++) begin
                    // will always invalidate all random offsets for both reasons
                    // they could be invalidated (all values are equal and 
                    // never unsigned less than the cloud length)
                    plane_point_offset_control.offset[3 * i + j] <= '1;
                end
            end
        end else begin
            // allow a max offset param < cloud length since those random numbers
            // will still be valid
            if (random_in_range_port.ovalid && random_in_range_port.oready && random_in_range_port.random_offset_param <= memory_vars.cloud_length) begin
                for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                    
                    for (int unsigned j = 0; j < 3; j++) begin
                        if (i == 0 && j == 0) begin
                            plane_point_offset_control.offset[3 * i] <= plane_point_offset_control.most_recent_random;
                        end
                        else begin 
                            plane_point_offset_control.offset[3 * i + j] <= plane_point_offset_control.offset[3 * i + j - 1];
                        end
                    end
                end
            end
        end
    end : plane_point_offset_advance_logic

    // logic for managing the main calculation goes here. (i.e., managing the 
    // signals to begin / abort calculations, clear caches, and monitor the
    // plane checking units)

    enum {
        // time between calculating and starting a new calculation
        RANSAC_STATE_IDLE,
        // an abort has been requested but not all units have finished their
        // iteration.
        RANSAC_STATE_ABORT_WAIT,
        // actively running calculations.
        RANSAC_STATE_ACTIVE,
        // running the last calculations
        RANSAC_STATE_FINAL_GROUP_WAIT
    } ransac_state;

    struct packed {
        // the plane with the most inliers from this most recent calculation
        struct packed {
            // the number of inliers
            logic [$clog2(maximum_point_count):0] inlier_count;
            // the definition of the plane with the most inliers.
            vector::vector3s_s plane_n;
            vector::single_t plane_d;
        } best_plane;
        struct packed {
            // units which have the iready handshake signal high (i.e., units
            // which, if so requested, would start calculations on the next
            // cycle)
            logic [plane_check_unit_count-1:0] iready_state;
            // units which have valid output
            logic [plane_check_unit_count-1:0] ovalid_state;
            // mask of all conditions required to allow starting a plane checking
            // unit.
            logic [plane_check_unit_count-1:0] allowed_start;
            logic [plane_check_unit_count-1:0] error_raised;
            // status codes of each unit.
            // for now, if an error occurs, just stop?
            ransac::plane_checking_unit_status_e [plane_check_unit_count-1:0] status;
            // plane normal (valid iff status is success)
            vector::vector3s_s [plane_check_unit_count-1:0] n;
            // plane distance (valid iff status is success)
            vector::single_t [plane_check_unit_count-1:0] d;
            // plane inlier count (valid iff status is success)
            logic [plane_check_unit_count-1:0][$clog2(maximum_point_count):0] inliers;
        } unit_state;

        // summary of the finished iteration(s) on this cycle
        struct packed {
            // best inlier count of the iterations with ovalid high on this cycle
            logic [$clog2(maximum_point_count):0] best_inlier_count;
            // plane_n value of the unit with ovalid high and the highest inlier count on this cycle
            vector::vector3s_s best_n;
            // plane_d value of the unit with ovalid high and the highest inlier count on this cycle
            vector::single_t best_d;
            // 1 if ANY unit has eligible for providing the values above.
            logic any_finished;
        } finished_iteration_summary;

        struct packed {
            logic start_command_received;
            logic abort_command_received;
        } request_from_control;

        struct packed {
            logic ransac_process_active;
            logic ransac_process_stopping;
            logic ransac_process_about_to_finish;
        } response_to_control;

        logic [$clog2(possible_max_iteration_count)-1:0] pending_iterations;
        logic [$clog2(possible_max_iteration_count)-1:0] next_pending_iterations;

    } ransac_control;

    always_comb begin : handle_ransac_control_combinatorial
        // wire together various fields.
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            ransac_control.unit_state.iready_state[i] <= plane_checking_unit_port[i].iready;
            ransac_control.unit_state.ovalid_state[i] <= plane_checking_unit_port[i].ovalid;
            ransac_control.unit_state.status[i] <= plane_checking_unit_port[i].status;
            ransac_control.unit_state.n[i] <= plane_checking_unit_port[i].plane_n;
            ransac_control.unit_state.d[i] <= plane_checking_unit_port[i].plane_d;
            ransac_control.unit_state.inliers[i] <= plane_checking_unit_port[i].inlier_count;
        end

        // determine which units ended their calculatiion in an error
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            ransac_control.unit_state.error_raised[i] <= (ransac_control.unit_state.status[i] != ransac::PLANE_CHECKING_UNIT_STATUS_SUCCESS) & ransac_control.unit_state.ovalid_state[i];
        end

        // determine the next pending iterations count (current + started - errored)
        ransac_control.next_pending_iterations <= ransac_control.pending_iterations + $countones(ransac_control.unit_state.allowed_start) - $countones(ransac_control.unit_state.error_raised);


        // broadcast various memory variables
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            plane_checking_unit_port[i].cloud_length <= memory_vars.cloud_length[$clog2(maximum_point_count)-1:0];
            plane_checking_unit_port[i].threshold <= memory_vars.inlier_threshold[vector::bits_in_single-1:0];
        end

        // determine if the units are allowed to start.
        ransac_control.unit_state.allowed_start <= {plane_check_unit_count{plane_point_offset_control.all_points_valid}} & ransac_control.unit_state.iready_state;


        ransac_control.finished_iteration_summary.best_inlier_count = '0;
        ransac_control.finished_iteration_summary.best_n = '0;
        ransac_control.finished_iteration_summary.best_d = '0;
        ransac_control.finished_iteration_summary.any_finished = '0;
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            if (ransac_control.unit_state.ovalid_state[i] == '1 && ransac_control.unit_state.status[i] == ransac::PLANE_CHECKING_UNIT_STATUS_SUCCESS) begin
                ransac_control.finished_iteration_summary.any_finished = '1;
                if (ransac_control.unit_state.inliers[i] > ransac_control.finished_iteration_summary.best_inlier_count || ransac_control.finished_iteration_summary.best_inlier_count == '0) begin
                    ransac_control.finished_iteration_summary.best_inlier_count = ransac_control.unit_state.inliers[i];
                    ransac_control.finished_iteration_summary.best_n = ransac_control.unit_state.n[i];
                    ransac_control.finished_iteration_summary.best_d = ransac_control.unit_state.d[i];
                end
            end
        end

        // might change later, but all outputs should be cleared on the next cycle
        for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
            plane_checking_unit_port[i].oacknowledge <= '1;
        end

        // update the processed iterations memory variable (always read only)
        memory_vars.processed_iterations <= ransac_control.pending_iterations;

    end : handle_ransac_control_combinatorial

    always_ff @(posedge clock) begin : ransac_control_state_machine
        if (reset == reset_polarity) begin : handle_reset
            // all ivalid -> 0
            for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                plane_checking_unit_port[i].ivalid <= '0;
            end

            // state -> IDLE
            ransac_state <= RANSAC_STATE_IDLE;
            ransac_control.response_to_control.ransac_process_active <= '0;
            ransac_control.response_to_control.ransac_process_stopping <= '0;

            ransac_control.best_plane.inlier_count <= '0;
            ransac_control.best_plane.plane_n <= '0;
            ransac_control.best_plane.plane_d <= '0;

            ransac_control.pending_iterations <= '0;
            ransac_control.response_to_control.ransac_process_about_to_finish <= '0;

        end : handle_reset
        else begin : state_machine_logic
            case (ransac_state)
            RANSAC_STATE_IDLE: begin
                // not stopping a calculation since there is no calculation to
                // stop.
                ransac_control.response_to_control.ransac_process_stopping <= '0;
                // not about to finish since there is no calculation to finish.
                ransac_control.response_to_control.ransac_process_about_to_finish <= '0;

                if (ransac_control.request_from_control.start_command_received) begin
                    ransac_state <= RANSAC_STATE_ACTIVE;
                    ransac_control.response_to_control.ransac_process_active <= '1;
                    // all ivalid -> 0 since no calculations should start on this cycle
                    for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                        plane_checking_unit_port[i].ivalid <= ransac_control.unit_state.allowed_start[i];
                    end
                    ransac_control.pending_iterations <= $countones(ransac_control.unit_state.allowed_start);
                    // clear the previous results.
                    ransac_control.best_plane.inlier_count <= '0;
                    ransac_control.best_plane.plane_n <= '0;
                    ransac_control.best_plane.plane_d <= '0;
                end else begin
                    ransac_state <= RANSAC_STATE_IDLE;
                    ransac_control.response_to_control.ransac_process_active <= '0;
                    // start all calculations which are allowed to do so.
                    for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                        plane_checking_unit_port[i].ivalid <= '0;
                    end
                end
            end
            RANSAC_STATE_ACTIVE: begin
                // even if request to abort on this cycle, the calculation is 
                // still running.
                ransac_control.response_to_control.ransac_process_active <= '1;
                
                // if any units finished calculations on this cycle, remember
                // the better of the current record and the potential new record.
                if (ransac_control.finished_iteration_summary.any_finished && (ransac_control.finished_iteration_summary.best_inlier_count > ransac_control.best_plane.inlier_count || ransac_control.best_plane.inlier_count == '0)) begin
                    ransac_control.best_plane.inlier_count <= ransac_control.finished_iteration_summary.best_inlier_count;
                    ransac_control.best_plane.plane_n <= ransac_control.finished_iteration_summary.best_n;
                    ransac_control.best_plane.plane_d <= ransac_control.finished_iteration_summary.best_d;
                end

                // process the abort request first
                if (ransac_control.request_from_control.abort_command_received) begin
                    // indicate that the abort was received, don't start any new
                    // units, etc.
                    ransac_control.response_to_control.ransac_process_stopping <= '1;
                    for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                        plane_checking_unit_port[i].ivalid <= '0;
                    end
                    ransac_state <= RANSAC_STATE_ABORT_WAIT;
                    // not about to finish since processing an abort is considered
                    // different from finishing a calculation.
                    ransac_control.response_to_control.ransac_process_about_to_finish <=  '0;
                    ransac_control.pending_iterations <= ransac_control.pending_iterations;
                end else begin
                    // not aborting the calculations (but may have finished)

                    // start the next group of units
                    for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                        plane_checking_unit_port[i].ivalid <= ransac_control.unit_state.allowed_start[i];
                    end
                    ransac_control.pending_iterations <= ransac_control.next_pending_iterations;

                    // compare against pending iterations to guarantee that 
                    // AT LEAST the requested number of iterations run
                    // (i.e., if all iterations on this step fail, we will
                    // run the bare minimum). As opposed to these iterations
                    // failing and the unit running fewer iterations than 
                    // requested.
                    if (ransac_control.pending_iterations >= memory_vars.requested_iterations) begin
                        // finished
                        ransac_state <= RANSAC_STATE_FINAL_GROUP_WAIT;
                        ransac_control.response_to_control.ransac_process_about_to_finish <= '1;
                    end else begin
                        // not finished.
                        ransac_state <= RANSAC_STATE_ACTIVE;
                        ransac_control.response_to_control.ransac_process_about_to_finish <= '0;
                    end
                end
            end
            RANSAC_STATE_ABORT_WAIT: begin
                // since the calculations have been aborted, prevent the 
                // units from starting new calculations
                for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                    plane_checking_unit_port[i].ivalid <= '0;
                end

                // all units finished.
                if (ransac_control.unit_state.iready_state == '1) begin
                    ransac_control.response_to_control.ransac_process_active <= '0;
                    ransac_control.response_to_control.ransac_process_stopping <= '0;
                    ransac_state <= RANSAC_STATE_IDLE;
                end else begin
                    ransac_control.response_to_control.ransac_process_active <= '1;
                    ransac_control.response_to_control.ransac_process_stopping <= '1;
                    ransac_state <= RANSAC_STATE_ABORT_WAIT;
                end

                ransac_control.response_to_control.ransac_process_about_to_finish <= '0;
            end

            RANSAC_STATE_FINAL_GROUP_WAIT: begin
                // since the calculations have been aborted, prevent the 
                // units from starting new calculations
                for (int unsigned i = 0; i < plane_check_unit_count; i++) begin
                    plane_checking_unit_port[i].ivalid <= '0;
                end

                // all units finished.
                if (ransac_control.unit_state.iready_state == '1) begin
                    ransac_control.response_to_control.ransac_process_active <= '0;
                    ransac_control.response_to_control.ransac_process_about_to_finish <= '0;
                    ransac_state <= RANSAC_STATE_IDLE;
                end else begin
                    ransac_control.response_to_control.ransac_process_active <= '1;
                    ransac_control.response_to_control.ransac_process_about_to_finish <= '1;
                    ransac_state <= RANSAC_STATE_FINAL_GROUP_WAIT;
                end

                ransac_control.response_to_control.ransac_process_stopping <= '0;
                ransac_control.pending_iterations <= ransac_control.next_pending_iterations;
            end
            endcase
        end : state_machine_logic
    end : ransac_control_state_machine

    // logic for managing the control AXI port goes here. (i.e., converting
    // reads / writes of special commands into signals to the other two logic
    // systems and generally controlling an AXI port)

    enum {
        // not processing AXI read commands
        CONTROL_AXI_READ_IDLE,
        // responding to an AXI read command
        CONTROL_AXI_READ_RESP
    } control_axi_read_state;

    enum {
        // not processing AXI write commands
        CONTROL_AXI_WRITE_IDLE,
        // taking in AXI write data (or waiting on it)
        CONTROL_AXI_WRITE_DATA,
        // responding to an AXI write transaction (driving B channel)
        CONTROL_AXI_WRITE_RESP
    } control_axi_write_state;
    
    // all of this data about control port fields is overly complicated since 
    // I at first didn't want to hard-code all of the variable sizes and offsets
    // (and, indeed, I don't hard-code the offsets), but due to Vivado's 
    // function call depth limit, I had to remove various functions which 
    // made the process a bit more automated (these weren't cases of infinite
    // recursiion, but to be fair, the recursion did go quite deep).

    // which fields are accessible through the control port
    typedef enum logic [3:0] {
        CONTROL_PORT_CALCULATION_START,
        CONTROL_PORT_CALCULATION_ABORT,
        CONTROL_PORT_CALCULATION_STATE,
        CONTROL_PORT_CLOUD_BASE,
        CONTROL_PORT_CLOUD_SIZE,
        CONTROL_PORT_REQUESTED_ITERATIONS,
        CONTROL_PORT_PROCESSED_ITERATIONS,
        CONTROL_PORT_INLIER_THRESHOLD,
        CONTROL_PORT_GROUND_PLANE_INLIER_COUNT,
        CONTROL_PORT_GROUND_PLANE_NX,
        CONTROL_PORT_GROUND_PLANE_NY,
        CONTROL_PORT_GROUND_PLANE_NZ,
        CONTROL_PORT_GROUND_PLANE_D,
        CONTROL_PORT_NO_FIELD
    } control_port_field_e;

    struct packed {
        logic [control_addr_width-1:0] field_offset;
        control_port_field_e field;
        logic read_allowed;
    } control_axi_read_vars;

    struct packed {
        logic [control_addr_width-1:0] field_offset;
        control_port_field_e field;
        logic write_allowed;
    } control_axi_write_vars;

    // size of a field given its id
    localparam int unsigned control_field_size[0:13] = '{
        1,
        1,
        1,
        $bits(memory_vars.cloud_base) / control_data_width,
        $bits(memory_vars.cloud_length) / control_data_width,
        $bits(memory_vars.requested_iterations) / control_data_width,
        $bits(memory_vars.processed_iterations) / control_data_width,
        word_aligned_width(vector::bits_in_single) / control_data_width,
        $bits(memory_vars.derived_plane_inlier_count) / control_data_width,
        word_aligned_width(vector::bits_in_single) / control_data_width,
        word_aligned_width(vector::bits_in_single) / control_data_width,
        word_aligned_width(vector::bits_in_single) / control_data_width,
        word_aligned_width(vector::bits_in_single) / control_data_width,
        0
    }; 

    localparam control_port_field_e[0:13] previous_control_field = '{
        CONTROL_PORT_CALCULATION_START,
        CONTROL_PORT_CALCULATION_START,
        CONTROL_PORT_CALCULATION_ABORT,
        CONTROL_PORT_CALCULATION_STATE,
        CONTROL_PORT_CLOUD_BASE,
        CONTROL_PORT_CLOUD_SIZE,
        CONTROL_PORT_REQUESTED_ITERATIONS,
        CONTROL_PORT_PROCESSED_ITERATIONS,
        CONTROL_PORT_INLIER_THRESHOLD,
        CONTROL_PORT_GROUND_PLANE_INLIER_COUNT,
        CONTROL_PORT_GROUND_PLANE_NX,
        CONTROL_PORT_GROUND_PLANE_NY,
        CONTROL_PORT_GROUND_PLANE_NZ,
        CONTROL_PORT_NO_FIELD
    };

    // keep this up to date with the previous control field function.
    localparam control_port_field_e highest_offset_field = CONTROL_PORT_GROUND_PLANE_D;
    // keep this up to date with the previous control field function
    localparam control_port_field_e lowest_offset_field = CONTROL_PORT_CALCULATION_START;

    localparam logic [0:13][control_addr_width-1:0] control_field_offset = '{
        0,  // start
        1,  // abort
        2,  // state
        3,  // base
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE],    // size
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE], // req iterations
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS], // proc iterations
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS] + 
            control_field_size[CONTROL_PORT_PROCESSED_ITERATIONS], // inlier threshold
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS] + 
            control_field_size[CONTROL_PORT_PROCESSED_ITERATIONS] +
            1 * control_field_size[CONTROL_PORT_INLIER_THRESHOLD],    // inlier count
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS] + 
            control_field_size[CONTROL_PORT_PROCESSED_ITERATIONS] +
            control_field_size[CONTROL_PORT_GROUND_PLANE_INLIER_COUNT] + 
            1 * control_field_size[CONTROL_PORT_INLIER_THRESHOLD],     // normal x
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS] + 
            control_field_size[CONTROL_PORT_PROCESSED_ITERATIONS] +
            control_field_size[CONTROL_PORT_GROUND_PLANE_INLIER_COUNT] + 
            2 * control_field_size[CONTROL_PORT_INLIER_THRESHOLD],     // normal y
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS] + 
            control_field_size[CONTROL_PORT_PROCESSED_ITERATIONS] +
            control_field_size[CONTROL_PORT_GROUND_PLANE_INLIER_COUNT] + 
            3 * control_field_size[CONTROL_PORT_INLIER_THRESHOLD],     // normal z
        3 + control_field_size[CONTROL_PORT_CLOUD_BASE] + control_field_size[CONTROL_PORT_CLOUD_SIZE] + 
            control_field_size[CONTROL_PORT_REQUESTED_ITERATIONS] + 
            control_field_size[CONTROL_PORT_PROCESSED_ITERATIONS] +
            control_field_size[CONTROL_PORT_GROUND_PLANE_INLIER_COUNT] + 
            4 * control_field_size[CONTROL_PORT_INLIER_THRESHOLD],     // distance
        '1    // no field
    };

    // // given a field, determine its offset
    // // the field which lists itself as the previous control field is considered 
    // // to be the first in memory. Other field positions determined by recursively
    // // calling this function on the previous field.
    // function static logic [control_addr_width-1:0] control_field_offset(input control_port_field_e field);
    // begin
    //     if (field == lowest_offset_field) begin
    //         control_field_offset = '0;
    //     end else if (previous_control_field[field] == lowest_offset_field) begin
    //         control_field_offset = control_field_size[lowest_offset_field];
    //     end else begin
    //         control_field_offset = control_field_offset(previous_control_field[field]) + control_field_size[previous_control_field[field]];
    //     end
    // end
    // endfunction : control_field_offset

    // using the above functions, determine if an address is in the range of 
    // addresses given to the provided field.
    // 
    // this condition occurs when address - base_address >= control_field_offset(field)
    // but less than control_field_offset + control_field_size
    function automatic bit address_refers_to_field(input logic [control_addr_width-1:0] addr, input control_port_field_e field);
        bit above_minimum;
        bit below_maximum;
    begin
        if (addr >= control_field_offset[field] + control_addr_base) begin
            above_minimum = '1;
        end else begin
            above_minimum = '0;
        end

        if (addr < control_field_offset[field] + control_addr_base + control_field_size[field]) begin
            below_maximum = '1;
        end else begin
            below_maximum = '0;
        end

        address_refers_to_field = above_minimum & below_maximum;
    end
    endfunction : address_refers_to_field

    function automatic control_port_field_e partial_address_lookup(input logic [control_addr_width-1:0] addr, input control_port_field_e starting_field);
    begin
        // if the first field to look at matches, then lookup is complete
        if (address_refers_to_field(addr, starting_field)) begin
            partial_address_lookup = starting_field;
        // otherwise, if there is no previous field, then no field matches
        end else if (starting_field == lowest_offset_field || starting_field == CONTROL_PORT_NO_FIELD) begin
            partial_address_lookup = CONTROL_PORT_NO_FIELD;
        end else begin
        // otherwise, check the previous field to see if it matches.
            partial_address_lookup = partial_address_lookup(addr, previous_control_field[starting_field]);
        end
    end
    endfunction : partial_address_lookup

    function automatic control_port_field_e address_to_field(input logic [control_addr_width-1:0] addr);
    begin
        address_to_field = partial_address_lookup(addr, highest_offset_field);
    end
    endfunction : address_to_field

    function automatic logic [control_addr_width-1:0] offset_into_field(input logic [control_addr_width-1:0] addr, input control_port_field_e matching_field);
    begin
        offset_into_field = addr - control_addr_base - control_field_offset[matching_field];
    end
    endfunction : offset_into_field

    function automatic bit field_read_allowed(input control_port_field_e field);
    begin
        case (field)
        CONTROL_PORT_CALCULATION_START,
        CONTROL_PORT_CALCULATION_ABORT,
        CONTROL_PORT_NO_FIELD: field_read_allowed = '0;
        default: field_read_allowed = '1;
        endcase
    end
    endfunction : field_read_allowed

    // if, at this present moment, the write to a field is allowed.
    function automatic bit field_write_allowed(input control_port_field_e field);
    begin
        case (field)
        // read only fields (and an out of bounds access)
        CONTROL_PORT_CALCULATION_STATE,
        CONTROL_PORT_PROCESSED_ITERATIONS,
        CONTROL_PORT_GROUND_PLANE_INLIER_COUNT,
        CONTROL_PORT_GROUND_PLANE_NX,
        CONTROL_PORT_GROUND_PLANE_NY,
        CONTROL_PORT_GROUND_PLANE_NZ,
        CONTROL_PORT_GROUND_PLANE_D,
        CONTROL_PORT_NO_FIELD: field_write_allowed = '0;
        CONTROL_PORT_CLOUD_BASE,
        CONTROL_PORT_CLOUD_SIZE,
        CONTROL_PORT_REQUESTED_ITERATIONS,
        CONTROL_PORT_INLIER_THRESHOLD: field_write_allowed = !ransac_control.response_to_control.ransac_process_active;
        // write unconditionally allowed
        default: field_write_allowed = '1;
        endcase
    end
    endfunction : field_write_allowed

    function automatic logic [control_data_width-1:0] read_control_data(input control_port_field_e field, input logic [control_addr_width-1:0] field_offset);
    begin
        case (field)
        CONTROL_PORT_CALCULATION_STATE: begin
            read_control_data[control_data_width-1:3] = '0;
            read_control_data[2] = ransac_control.response_to_control.ransac_process_about_to_finish;
            read_control_data[1] = ransac_control.response_to_control.ransac_process_stopping;
            read_control_data[0] = ransac_control.response_to_control.ransac_process_active;
        end
        CONTROL_PORT_CLOUD_BASE: begin
            read_control_data = memory_vars.cloud_base[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_CLOUD_SIZE: begin
            read_control_data = memory_vars.cloud_length[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_REQUESTED_ITERATIONS: begin
            read_control_data = memory_vars.requested_iterations[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_PROCESSED_ITERATIONS: begin
            read_control_data = memory_vars.processed_iterations[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_INLIER_THRESHOLD: begin
            read_control_data = memory_vars.inlier_threshold[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_GROUND_PLANE_INLIER_COUNT: begin
            read_control_data = memory_vars.derived_plane_inlier_count[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_GROUND_PLANE_NX: begin
            read_control_data = memory_vars.derived_plane_normal[0][control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_GROUND_PLANE_NY: begin
            read_control_data = memory_vars.derived_plane_normal[1][control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_GROUND_PLANE_NZ: begin
            read_control_data = memory_vars.derived_plane_normal[2][control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        CONTROL_PORT_GROUND_PLANE_D: begin
            read_control_data = memory_vars.derived_plane_distance[control_data_width - 1 + control_data_width * field_offset-:control_data_width];
        end
        default: read_control_data = '0;
        endcase
    end
    endfunction : read_control_data

    always_comb begin : control_axi_decode
        control_axi_read_vars.field = address_to_field(control_araddr);
        control_axi_read_vars.field_offset = offset_into_field(control_araddr, control_axi_read_vars.field);
        control_axi_read_vars.read_allowed = field_read_allowed(control_axi_read_vars.field);
        
    end : control_axi_decode

    // handle the control AXI read port
    always_ff @(posedge clock) begin : control_axi_read
        if (reset == reset_polarity) begin
            // responsible only for writing own state and respond signals
            control_arready <= '1;
            control_rresp <= 2'b00;
            control_rvalid <= '0;
            control_rdata <= '0;
            control_axi_read_state <= CONTROL_AXI_READ_IDLE;
            
        end else begin
            case (control_axi_read_state)
            CONTROL_AXI_READ_IDLE: begin
                if (control_arready && control_arvalid) begin
                    control_arready <= '0;
                    control_rvalid <= '1;
                    if (control_axi_read_vars.read_allowed) begin
                        control_rresp <= 2'b00;
                    end else begin
                        control_rresp <= 2'b10;
                    end
                    control_rdata <= read_control_data(control_axi_read_vars.field, control_axi_read_vars.field_offset);
                    control_axi_read_state <= CONTROL_AXI_READ_RESP;
                end
            end
            CONTROL_AXI_READ_RESP: begin
                if (control_rready && control_rvalid) begin
                    control_rvalid <= '0;
                    control_arready <= '1;
                    control_axi_read_state <= CONTROL_AXI_READ_IDLE;
                end
            end
            endcase
        end
    end : control_axi_read

    // handle the control AXI write port

    always_ff @(posedge clock) begin : control_axi_write
        if (reset == reset_polarity) begin
            control_awready <= '1;
            control_wready <= '0;
            control_bresp <= '0;
            control_bvalid <= '0;
            control_axi_write_state <= CONTROL_AXI_WRITE_IDLE;

            // reset the writeable memory registers.
            memory_vars.cloud_length <= '0;
            memory_vars.cloud_base <= '0;
            memory_vars.inlier_threshold <= '0;
            memory_vars.requested_iterations <= '0;

            control_axi_write_vars.field <= CONTROL_PORT_NO_FIELD;
            control_axi_write_vars.field_offset <= '0;
            control_axi_write_vars.write_allowed <= '0;

            ransac_control.request_from_control.start_command_received = '0;
            ransac_control.request_from_control.abort_command_received = '0;
        end else begin
            ransac_control.request_from_control.start_command_received = '0;
            ransac_control.request_from_control.abort_command_received = '0;

            case (control_axi_write_state)
            CONTROL_AXI_WRITE_IDLE: begin
                control_axi_write_vars.field <= address_to_field(control_awaddr);
                control_axi_write_vars.field_offset <= offset_into_field(control_awaddr, control_axi_write_vars.field);
                control_axi_write_vars.write_allowed <= field_write_allowed(control_axi_write_vars.field);
                if (control_awready && control_awvalid) begin
                    control_awready <= '0;
                    control_wready <= '1;
                    control_bresp <= '0;
                    control_bvalid <= '0;
                    control_axi_write_state <= CONTROL_AXI_WRITE_DATA;
                end
            end
            CONTROL_AXI_WRITE_DATA: begin
                if (control_wready && control_wvalid) begin
                    control_awready <= '0;
                    control_wready <= '0;
                    control_bvalid <= '1;
                    control_axi_write_state <= CONTROL_AXI_WRITE_RESP;
                    if (control_axi_write_vars.write_allowed) begin
                        control_bresp <= 2'b00;
                        case (control_axi_write_vars.field)
                        CONTROL_PORT_CALCULATION_START: begin
                            if (control_wstrb != '0) begin
                                ransac_control.request_from_control.start_command_received = '1;
                            end
                        end 
                        CONTROL_PORT_CALCULATION_ABORT: begin
                            if (control_wstrb != '0) begin
                                ransac_control.request_from_control.abort_command_received = '1;
                            end
                        end
                        CONTROL_PORT_CLOUD_BASE: begin
                            for (int unsigned i = 0; i < control_data_width / 8; i++) begin
                                if (control_wstrb[i]) memory_vars.cloud_base[control_data_width * control_axi_write_vars.field_offset + 8 * i + 7-:8] <= control_wdata[8*i+7-:8];
                                // memory_vars.cloud_base[control_data_width - 1 + control_data_width * control_axi_write_vars.field_offset-:control_data_width] <= control_wdata;
                            end
                        end
                        CONTROL_PORT_CLOUD_SIZE: begin
                            for (int unsigned i = 0; i < control_data_width / 8; i++) begin
                                if (control_wstrb[i]) memory_vars.cloud_length[control_data_width * control_axi_write_vars.field_offset + 8 * i + 7-:8] <= control_wdata[8*i+7-:8];
                                // memory_vars.cloud_length[control_data_width - 1 + control_data_width * control_axi_write_vars.field_offset-:control_data_width] <= control_wdata;
                            end
                        end
                        CONTROL_PORT_REQUESTED_ITERATIONS: begin
                            for (int unsigned i = 0; i < control_data_width / 8; i++) begin
                                if (control_wstrb[i]) memory_vars.requested_iterations[control_data_width * control_axi_write_vars.field_offset + 8 * i + 7-:8] <= control_wdata[8*i+7-:8];
                                // memory_vars.requested_iterations[control_data_width - 1 + control_data_width * control_axi_write_vars.field_offset-:control_data_width] <= control_wdata;
                            end
                        end
                        CONTROL_PORT_INLIER_THRESHOLD: begin
                            for (int unsigned i = 0; i < control_data_width / 8; i++) begin
                                if (control_wstrb[i]) memory_vars.inlier_threshold[control_data_width * control_axi_write_vars.field_offset + 8 * i + 7-:8] <= control_wdata[8*i+7-:8];
                                // memory_vars.inlier_threshold[control_data_width - 1 + control_data_width * control_axi_write_vars.field_offset-:control_data_width] <= control_wdata;
                            end
                        end
                        default: ; // intentionally blank.
                        endcase
                    end else begin
                        control_bresp <= 2'b10;
                    end
                end
            end
            CONTROL_AXI_WRITE_RESP: begin
                if (control_bready && control_bvalid) begin
                    control_awready <= '1;
                    control_wready <= '0;
                    control_bvalid <= '0;
                    control_bresp <= 2'b00;
                    control_axi_write_state <= CONTROL_AXI_WRITE_IDLE;
                end
            end
            endcase
        end
    end : control_axi_write

    // drive unused memory write channel
    always_comb begin
        memory_awaddr = '0;
        memory_awvalid = '0;
        memory_wvalid = '0;
        memory_wstrb = '0;
        memory_wdata = '0;
        memory_bready = '0;
    end

    genvar i;

    generate
        for (i = 0; i < plane_check_unit_count; i++) begin
            plane_checking_unit#(
                .maximum_points(maximum_point_count),
                .check_inlier_instance_count(check_inlier_units_per_plane_check_unit),
                .addr_width(memory_addr_width),
                .data_width(memory_data_width),
                .maximum_read_latency(memory_maximum_latency),
                .small_fma_latency(small_fma_latency),
                .large_fma_latency(large_fma_latency),
                .plane_fma_latency(plane_fma_latency),
                .reset_polarity(reset_polarity)
            ) pcu(
                .clock(clock),
                .reset(reset),
                .ivalid(plane_checking_unit_port[i].ivalid),
                .iready(plane_checking_unit_port[i].iready),
                .cloud_length(plane_checking_unit_port[i].cloud_length),
                .plane_point_start_offsets(plane_checking_unit_port[i].plane_point_start_offsets),
                .threshold(plane_checking_unit_port[i].threshold),
                .ovalid(plane_checking_unit_port[i].ovalid),
                .oacknowledge(plane_checking_unit_port[i].oacknowledge),
                .inlier_count(plane_checking_unit_port[i].inlier_count),
                .status(plane_checking_unit_port[i].status),
                .point_addr(plane_checking_unit_port[i].point_addr),
                .plane_n(plane_checking_unit_port[i].plane_n),
                .plane_d(plane_checking_unit_port[i].plane_d),
                .point_addr_valid(plane_checking_unit_port[i].point_addr_valid),
                .point_addr_ready(plane_checking_unit_port[i].point_addr_ready),
                .point_data(plane_checking_unit_port[i].point_data),
                .point_resp(plane_checking_unit_port[i].point_resp),
                .point_data_valid(plane_checking_unit_port[i].point_data_valid),
                .point_data_ready(plane_checking_unit_port[i].point_data_ready)
            );
        end

    endgenerate

    random_in_range#(
        .lfsr_register_width(lfsr_register_width),
        .lfsr_window_width(lfsr_window_width),
        .lfsr_window_start(lfsr_window_start),
        .lfsr_polynomial(lfsr_polynomial),
        .lfsr_seed(lfsr_seed),
        .reset_polarity(reset_polarity)
    ) rng_unit(
        .clock(clock),
        .reset(reset),
        .ivalid(random_in_range_port.ivalid),
        .iready(random_in_range_port.iready),
        .base(random_in_range_port.base),
        .max_offset(random_in_range_port.max_offset),
        .random(random_in_range_port.random),
        .random_base_param(random_in_range_port.random_base_param),
        .random_offset_param(random_in_range_port.random_offset_param),
        .ovalid(random_in_range_port.ovalid),
        .oready(random_in_range_port.oready)
    );
endmodule : ransac_unit