`timescale 1ns / 1ps

`include "ransac/ransac_pkg.svh"
`include "vectors/vector_pkg.svh"

module plane_checking_unit#(
    // maximum points in the cloud, as supported by hardware
    // address must fit at least (3 * bytes_in_single) x this.
    parameter int unsigned maximum_points = 1 << 20,
    // how many points (in parallel) will be considered for being an inlier
    parameter int unsigned check_inlier_instance_count = 2,
    // size of the AXI bus address
    parameter int unsigned addr_width = 32,
    // data bits on the AXI bus
    parameter int unsigned data_width = vector::bits_in_single,
    // maximum number of cycles to wait before giving up
    parameter int unsigned maximum_read_latency = 1 << 10,
    // latency for check_inlier small fma
    parameter int unsigned small_fma_latency = vector::fma_latency_singles,
    // latency for check_inlier large fma
    parameter int unsigned large_fma_latency = vector::fma_latency_doubles,
    // latency for derive_plane fma
    parameter int unsigned plane_fma_latency = vector::fma_latency_doubles,
    // value required for resetting the logic
    parameter bit reset_polarity = 1)(
        
        input logic clock,
        input logic reset,

        input logic ivalid,
        output logic iready,
        input logic [$clog2(maximum_points)-1:0] cloud_length,
        input logic [2:0][$clog2(maximum_points)-1:0] plane_point_start_offsets,
        input vector::single_t threshold,

        output logic ovalid,
        input logic oacknowledge,
        output logic [$clog2(maximum_points):0] inlier_count,
        output ransac::plane_checking_unit_status_e status,
        output vector::vector3s_s plane_n,
        output vector::single_t plane_d,

        // interface to get points from some external memory
        // if you're familier with AXI, then this structure will
        // look very familiar to you -- it's an AXI-Lite bus stripped
        // down to the AR and R channels!

        // address (really offset into some structure) to find the point at.
        // this address is assumed to be a BYTE address, we will assume that
        // each fixed point value is padded to the next byte in size
        output logic [addr_width-1:0] point_addr,
        // 1 if point_addr is the value we want for the transaction and
        // we are trying to read a point
        output logic point_addr_valid,
        // 1 if the decoder is ready to receive a point
        input logic point_addr_ready,

        // the point received (if resp is ransac::AXI_RESP_OKAY)
        input logic [data_width-1:0] point_data,
        // hopefully always AXI_RESP_OKAY. If not, then the calculation
        // ends and status is set to PLANE_CHECKING_UNIT_STATUS_BUS_ERROR
        input logic [1:0] point_resp,
        // 1 if data is valid
        input logic point_data_valid,
        // 1 if the unit can accept data
        output logic point_data_ready
    );

    // from idle -> number of points
    //
    // 1. capture the offsets for each of the 3 plane-defining points
    // 2. read point a (point_addr = plane_point_offsets[0]; plane_addr_valid = 1, point_data_ready = 1) \ 
    // 3. await point a                                                                                  | can make one state
    // 4. capture point a                                                                               /
    // 5. repeat 2 through 4 for points b and c
    // 6. derive the plane
    // 7. wait on deriving the plane; set an internal point_offset value to 0
    // 8. capture the derived plane
    // 9. read points at point_offset_value and feed them to check inlier units as fast as possible
    //    on any cycle, increment an internal inlier count by the number of check_inlier instances 
    //    which (on this cycle) find an inlier
    // 10. report that the output is ready and return to the idle state.
    // 
    // to do the above, we need to capture / remember:
    //   1. offsets to points a, b, c
    //   2. points a, b, c
    //   3. derived n and d
    //   4. address of the next point to read
    //   5. count of inliers found so far
    //   6. what the cloud length was when we started

    
    // create internal data-read manager logic which 
    // can access a, b, c, and a "point fresh from memory" register

    typedef logic [$clog2(maximum_points)-1:0] point_offset_t;

    typedef enum logic [2:0] {
        // not read and don't want it yet
        MEMORY_DATA_STATE_KEEP_UNREAD,
        // not read but it's wanted
        MEMORY_DATA_STATE_UNREAD,
        // reading it
        MEMORY_DATA_STATE_READING,
        // up to date
        MEMORY_DATA_STATE_READ,
        MEMORY_DATA_STATE_ERROR
    } memory_data_state_e;
    
    typedef struct packed {
          union packed {
            logic [data_width-1:0] raw_bits;
            vector::single_t value;
        } memory_value;
        struct packed {
            // 1 if requesting to switch from KEEP_UNREAD or READ -> UNREAD
            // can only be driven by main state machine, to prevent
            // multi-driven nets
            logic unread;
            // 1 if requesting to switch from UNREAD -> READING
            // can only be driven by memory bus state machine, to
            // prevent multi-driven nets
            logic accept;
            // 1 if requesting to switch from READING -> ERROR
            // can only be driven by memory bus state machine, to
            // prevent multi-driven nets
            logic error;
            // 1 if requesting to switch from READING -> READ
            // can only be driven by memory bus state machine, to
            // prevent multi-driven nets
            logic finish;
            // 1 if requesting to switch from anything -> KEEP_UNREAD
            // can only be driven by main state machine, to prevent
            // multi-driven nets
            logic reset;
        } requests;
        memory_data_state_e state;
        point_offset_t offset;
    } point_from_memory_s;

    typedef enum logic [3:0] {
        // x component of point A for derive plane
        MEMORY_VARIABLE_POINT_A_X_PART,
        // y component of point A for derive plane
        MEMORY_VARIABLE_POINT_A_Y_PART,
        MEMORY_VARIABLE_POINT_A_Z_PART,
        MEMORY_VARIABLE_POINT_B_X_PART,
        MEMORY_VARIABLE_POINT_B_Y_PART,
        MEMORY_VARIABLE_POINT_B_Z_PART,
        MEMORY_VARIABLE_POINT_C_X_PART,
        MEMORY_VARIABLE_POINT_C_Y_PART,
        MEMORY_VARIABLE_POINT_C_Z_PART,
        // x component of the next point for a check_inlier instance
        MEMORY_VARIABLE_NEXT_POINT_X_PART,
        // y component of the next point for a check_inlier instance
        MEMORY_VARIABLE_NEXT_POINT_Y_PART,
        // z component of the next point for a check_inlier instance
        MEMORY_VARIABLE_NEXT_POINT_Z_PART
    } memory_variable_e;

    typedef enum logic [1:0] {
        MEMORY_BUS_STATUS_GOOD,
        MEMORY_BUS_STATUS_BUS_ERROR,
        MEMORY_BUS_STATUS_TIMEOUT
    } memory_bus_status_e;

    localparam int unsigned max_memory_variable_index = 12; // 4 * 3
    localparam int unsigned max_plane_point_index = 9; // 3 * 3

    point_from_memory_s [max_memory_variable_index-1:0] memory_variable;

    // logic for controlling the derive_plane instance
    struct packed {
        logic ivalid;
        logic iready;
        vector::point_t a;
        vector::point_t b;
        vector::point_t c;

        logic ovalid;
        logic oacknowledge;
        vector::vector3s_s n;
        vector::single_t d;
        vector::derive_plane_status_e status;
    } derive_plane_control;

    // logic for controlling the check_inlier instances
    typedef struct packed {
        logic ivalid;
        logic iready;
        vector::vector3s_s n;
        vector::point_t p;
        vector::single_t d;
        vector::single_t t;
        logic ovalid;
        logic oacknowledge;
        logic inlier;
    } check_inlier_control_s;

    check_inlier_control_s [check_inlier_instance_count-1:0] check_inlier_control;

    struct packed {
        logic [$clog2(maximum_points)-1:0] cloud_length;
        vector::single_t threshold;
        logic [$clog2(maximum_points):0] inliers;
        logic [$clog2(maximum_points)-1:0] next_point_offset;

        vector::vector3s_s plane_n;
        vector::single_t plane_d;
        // variables used within a cycle or that are always combinatorial
        
        // 1 if variables ax through cz are in the READ state
        logic received_all_plane_points;
        // 1 if variables next_x through next_z are in the READ state  
        logic received_next_point;
        // 1 if any variable is in the ERROR state
        logic bus_error_detected;

        // 1 if any check_inlier_control.iready is set
        logic any_check_inlier_instances_ready;
        // 1 if all check_inlier_control.ovalid set
        logic all_check_inlier_instances_finished;
        // index corresponding to the "first" ready check inlier instance
        // "first" -> highest index since that's just how the loop works
        // value is 0 if none are ready.
        logic [$clog2(check_inlier_instance_count)-1:0] first_ready_check_inlier_instance;
    } check_plane_vars;
    
    enum logic [2:0] {
        // idle and waiting for something to do
        CHECK_PLANE_STATE_IDLE,
        // wait on points a, b, and c to arrive
        CHECK_PLANE_STATE_AWAIT_PLANE_POINTS,
        // wait on the plane to be derived
        CHECK_PLANE_STATE_AWAIT_DERIVE_PLANE,
        // until all points have been processed, continue processing them.
        CHECK_PLANE_STATE_ITERATE_OVER_CLOUD
    } check_plane_state;

    // cycles spent reading the current value
    // placed before the logic for the memory variables since
    // the check_plane state machine needs this information to
    // differentiate a bus error from a bus timeout
    logic [$clog2(maximum_read_latency):0] read_duration;

    always @(posedge clock) begin : main_logic
        // update the within-cycle / combinatorial variables

        // received all plane points
        check_plane_vars.received_all_plane_points = '1;
        for (int unsigned i = 0; i < max_plane_point_index; i++) begin
            if (memory_variable[i].state != MEMORY_DATA_STATE_READ) begin
                check_plane_vars.received_all_plane_points = '0;
            end
        end
        
        // received next point
        check_plane_vars.received_next_point = 1;
        for (int unsigned i = max_plane_point_index; i < max_memory_variable_index; i++) begin
            if (memory_variable[i].state != MEMORY_DATA_STATE_READ) begin
                check_plane_vars.received_next_point = 0;
            end
        end

        // bus error detected
        check_plane_vars.bus_error_detected = '0;
        for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
            if (memory_variable[i].state == MEMORY_DATA_STATE_ERROR) begin
                check_plane_vars.bus_error_detected = '1;
            end
        end

        // any check_inlier units ready and first ready check_inlier unit
        check_plane_vars.any_check_inlier_instances_ready = '0;
        check_plane_vars.all_check_inlier_instances_finished = '1;
        check_plane_vars.first_ready_check_inlier_instance = '0;
        for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
            if (check_inlier_control[i].iready) begin
                check_plane_vars.any_check_inlier_instances_ready = '1;
                check_plane_vars.first_ready_check_inlier_instance = i;
            end

            if (!check_inlier_control[i].ovalid) begin
                check_plane_vars.all_check_inlier_instances_finished = '0;
            end
        end

        for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
            memory_variable[i].requests.unread = 0;
            memory_variable[i].requests.reset = 0;
        end
        

        if (reset == reset_polarity) begin : handle_reset
            check_plane_state = CHECK_PLANE_STATE_IDLE;
            // not our job to reset the memory variable values
            // or states but it is our job to reset their offsets
            for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
                memory_variable[i].offset = '0;
                memory_variable[i].requests.reset = '1;
            end
            // reset control signals to internal modules
            derive_plane_control.ivalid = 0;
            for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                check_inlier_control[i].ivalid = 0;
            end
            // reset captured vars
            check_plane_vars.cloud_length = '0;
            check_plane_vars.threshold = '0;
            check_plane_vars.inliers = '0;
            // ready for input, no valid output
            iready = 1;
            ovalid = 0;

            status = ransac::PLANE_CHECKING_UNIT_STATUS_SUCCESS;
        end : handle_reset

        else begin : main_state_machine

            case (check_plane_state)
            
            CHECK_PLANE_STATE_IDLE: begin
                // change the state of all memory variables to KEEP_UNREAD
                for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
                    memory_variable[i].requests.reset = '1;
                end
                // capture / set the state machine variables
                check_plane_vars.cloud_length = cloud_length;
                check_plane_vars.threshold = threshold;
                check_plane_vars.inliers = 0;
                check_plane_vars.next_point_offset = 0;
                // capture the point offsets
                memory_variable[MEMORY_VARIABLE_POINT_A_X_PART].offset = plane_point_start_offsets[0] + 0;
                memory_variable[MEMORY_VARIABLE_POINT_A_Y_PART].offset = plane_point_start_offsets[0] + 1;
                memory_variable[MEMORY_VARIABLE_POINT_A_Z_PART].offset = plane_point_start_offsets[0] + 2;
                memory_variable[MEMORY_VARIABLE_POINT_B_X_PART].offset = plane_point_start_offsets[1] + 0;
                memory_variable[MEMORY_VARIABLE_POINT_B_Y_PART].offset = plane_point_start_offsets[1] + 1;
                memory_variable[MEMORY_VARIABLE_POINT_B_Z_PART].offset = plane_point_start_offsets[1] + 2;
                memory_variable[MEMORY_VARIABLE_POINT_C_X_PART].offset = plane_point_start_offsets[2] + 0;
                memory_variable[MEMORY_VARIABLE_POINT_C_Y_PART].offset = plane_point_start_offsets[2] + 1;
                memory_variable[MEMORY_VARIABLE_POINT_C_Z_PART].offset = plane_point_start_offsets[2] + 2;
                // prepare next point offset too
                memory_variable[MEMORY_VARIABLE_NEXT_POINT_X_PART].offset = 0;
                memory_variable[MEMORY_VARIABLE_NEXT_POINT_Y_PART].offset = 1;
                memory_variable[MEMORY_VARIABLE_NEXT_POINT_Z_PART].offset = 2;

                // don't accidentally submit any calculations to derive plane or any check inlier instance
                // but if these modules have "junk" data, silently get rid of it  
                derive_plane_control.ivalid = 0;
                derive_plane_control.oacknowledge = 1;
                for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                    check_inlier_control[i].ivalid = 0;
                    check_inlier_control[i].oacknowledge = 1;
                end

                // if we aren't ready because the output has not been acknowledged yet
                // and the output is acknowledged on this cycle, set iready
                if (!iready && ovalid && oacknowledge) begin
                    iready = 1;
                end
                // otherwise, if iready and ivalid, begin the calculation
                else if (iready && ivalid) begin
                    // request the points, wait for ax - cz starting on the next cycle
                    for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
                        memory_variable[i].requests.reset = 0;
                        memory_variable[i].requests.unread = 1;
                    end
                    check_plane_state = CHECK_PLANE_STATE_AWAIT_PLANE_POINTS;
                end
            end

            CHECK_PLANE_STATE_AWAIT_PLANE_POINTS: begin
                if (check_plane_vars.received_all_plane_points && !check_plane_vars.bus_error_detected) begin
                    check_plane_state = CHECK_PLANE_STATE_AWAIT_DERIVE_PLANE;
                    derive_plane_control.a.v.x = memory_variable[MEMORY_VARIABLE_POINT_A_X_PART].memory_value.value;
                    derive_plane_control.a.v.y = memory_variable[MEMORY_VARIABLE_POINT_A_Y_PART].memory_value.value;
                    derive_plane_control.a.v.z = memory_variable[MEMORY_VARIABLE_POINT_A_Z_PART].memory_value.value;

                    derive_plane_control.b.v.x = memory_variable[MEMORY_VARIABLE_POINT_B_X_PART].memory_value.value;
                    derive_plane_control.b.v.y = memory_variable[MEMORY_VARIABLE_POINT_B_Y_PART].memory_value.value;
                    derive_plane_control.b.v.z = memory_variable[MEMORY_VARIABLE_POINT_B_Z_PART].memory_value.value;

                    derive_plane_control.c.v.x = memory_variable[MEMORY_VARIABLE_POINT_C_X_PART].memory_value.value;
                    derive_plane_control.c.v.y = memory_variable[MEMORY_VARIABLE_POINT_C_Y_PART].memory_value.value;
                    derive_plane_control.c.v.z = memory_variable[MEMORY_VARIABLE_POINT_C_Z_PART].memory_value.value;

                    derive_plane_control.ivalid = 1;
                end

                if (check_plane_vars.bus_error_detected) begin
                    ovalid = 1;
                    check_plane_state = CHECK_PLANE_STATE_IDLE;
                    if (read_duration >= maximum_read_latency) begin
                        status = ransac::PLANE_CHECKING_UNIT_STATUS_BUS_TIMEOUT;
                    end else begin
                        status = ransac::PLANE_CHECKING_UNIT_STATUS_BUS_ERROR;
                    end
                end
            end

            CHECK_PLANE_STATE_AWAIT_DERIVE_PLANE: begin
                
                // don't accidentally submit a new calculation
                derive_plane_control.ivalid = 0;
                
                // if the plane has been derived successfully and no error 
                // next point has occurred in reading the next point then
                // continue to iterating over the cloud.

                if (!check_plane_vars.bus_error_detected && derive_plane_control.status == vector::DERIVE_PLANE_STATUS_SUCCESS && derive_plane_control.ovalid) begin
                    
                    // capture n and d, move threshold for all check_inlier instances
                    // don't submit any check_inlier calculations yet since we want to
                    // continue to the next state where we will already need logic to check
                    // if the plane has arrived yet.
                    for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                        check_inlier_control[i].t = check_plane_vars.threshold;
                        check_inlier_control[i].n = derive_plane_control.n;
                        check_inlier_control[i].d = derive_plane_control.d;
                    end

                    check_plane_vars.plane_n = derive_plane_control.n;
                    check_plane_vars.plane_d = derive_plane_control.d;

                    check_plane_state = CHECK_PLANE_STATE_ITERATE_OVER_CLOUD;
                end

                // if there was a bus error (we're still reading at least some of the first point while deriving the plane)
                // end the calculation here.
                if (check_plane_vars.bus_error_detected) begin
                    ovalid = 1;
                    check_plane_state = CHECK_PLANE_STATE_IDLE;
                    if (read_duration >= maximum_read_latency) begin
                        status = ransac::PLANE_CHECKING_UNIT_STATUS_BUS_TIMEOUT;
                    end else begin
                        status = ransac::PLANE_CHECKING_UNIT_STATUS_BUS_ERROR;
                    end
                end

                // if there was a derive plane error, report that
                if (derive_plane_control.ovalid && derive_plane_control.status != vector::DERIVE_PLANE_STATUS_SUCCESS) begin
                    ovalid = 1;
                    check_plane_state = CHECK_PLANE_STATE_IDLE;
                    status = ransac::PLANE_CHECKING_UNIT_STATUS_DERIVE_PLANE_ERROR;
                end
            end

            CHECK_PLANE_STATE_ITERATE_OVER_CLOUD: begin

                // don't accidentally submit a check_inlier calculation, acknowledge any finished calculations
                for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                    check_inlier_control[i].ivalid = 0;
                    check_inlier_control[i].oacknowledge = 1;

                    if (check_inlier_control[i].ovalid && check_inlier_control[i].inlier) begin
                        check_plane_vars.inliers++;
                    end
                end

                // only process things on this cycle if we've gotten the next point
                // any can submit a new calculation
                if (check_plane_vars.received_next_point && check_plane_vars.any_check_inlier_instances_ready) begin
                    check_inlier_control[check_plane_vars.first_ready_check_inlier_instance].p.v.x = memory_variable[MEMORY_VARIABLE_NEXT_POINT_X_PART].memory_value.value;
                    check_inlier_control[check_plane_vars.first_ready_check_inlier_instance].p.v.y = memory_variable[MEMORY_VARIABLE_NEXT_POINT_Y_PART].memory_value.value;
                    check_inlier_control[check_plane_vars.first_ready_check_inlier_instance].p.v.z = memory_variable[MEMORY_VARIABLE_NEXT_POINT_Z_PART].memory_value.value;
                    check_inlier_control[check_plane_vars.first_ready_check_inlier_instance].ivalid = 1;

                    // begin reading the next point if there is one to read.
                    // do the calculation in this (kind of strange) order to
                    // ensure that if max_cloud_length is near a power of 2 and 
                    // cloud_length is near the max that next_point_offset never overflows.
                    if (check_plane_vars.cloud_length - check_plane_vars.next_point_offset >= 6) begin
                        check_plane_vars.next_point_offset += 3;
                        for (int unsigned i = 0; i < 3; i++) begin
                            memory_variable[i + max_plane_point_index].offset = check_plane_vars.next_point_offset + i;
                            memory_variable[i + max_plane_point_index].requests.unread = 1;
                        end
                    end else begin 
                        // if no new points to submit (at this point in the logic), 
                        // that means we're waiting on the last few points to finish the calculation
                        // we're finished iff all check_inlier instances have finished the calculation
                        if (check_plane_vars.all_check_inlier_instances_finished) begin
                            status = ransac::PLANE_CHECKING_UNIT_STATUS_SUCCESS;
                            ovalid = '1;
                            inlier_count = check_plane_vars.inliers;
                            plane_n = check_plane_vars.plane_n;
                            plane_d = check_plane_vars.plane_d;
                            check_plane_state = CHECK_PLANE_STATE_IDLE;
                        end
                    end
                end

                // check for a bus error, if one occurred, then the
                // calculation ends here
                if (check_plane_vars.bus_error_detected) begin
                    ovalid = '1;
                    check_plane_state = CHECK_PLANE_STATE_IDLE;
                    if (read_duration >= maximum_read_latency) begin
                        status = ransac::PLANE_CHECKING_UNIT_STATUS_BUS_TIMEOUT;
                    end else begin
                        status = ransac::PLANE_CHECKING_UNIT_STATUS_BUS_ERROR;
                    end
                end
            end

            endcase

        end : main_state_machine

    end : main_logic

    // handle the memory variable logic here

    enum logic [1:0] {
        MEMORY_READ_STATE_IDLE,
        MEMORY_READ_STATE_ADDRESS_STAGE,
        MEMORY_READ_STATE_DATA_STAGE
    } memory_read_state;

    typedef struct packed {
        logic out_of_date;
        logic [3:0] next_variable_to_read;
    } memory_variable_state_summary_s;

    function automatic memory_variable_state_summary_s summarize_memory_variables(input point_from_memory_s[max_memory_variable_index-1:0] variables);
        memory_variable_state_summary_s scratchpad;
        scratchpad.out_of_date = '0;
        scratchpad.next_variable_to_read = '0;
        for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
            if (!scratchpad.out_of_date && variables[i].state == MEMORY_DATA_STATE_UNREAD) begin
                scratchpad.out_of_date = '1;
                scratchpad.next_variable_to_read = i;
            end
        end
        summarize_memory_variables = scratchpad;
    endfunction : summarize_memory_variables

    memory_variable_state_summary_s current_memory_summary;

    logic [3:0] currently_reading;

    always @(posedge clock) begin : handle_memory_bus

        for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
            memory_variable[i].requests.accept = 0;
            memory_variable[i].requests.error = 0;
            memory_variable[i].requests.finish = 0;
        end

        if (reset == reset_polarity) begin : memory_bus_reset
            // not our job to reset the memory variable offsets
            // but it is our job to reset the read state and 
            // read values.
            memory_read_state = MEMORY_READ_STATE_IDLE;
            for (int unsigned i = 0; i < max_memory_variable_index; i++) begin
                memory_variable[i].memory_value.raw_bits = '0;
            end

            currently_reading = '0;
            // no valid address, cannot accept data
            point_addr_valid = '0;
            point_data_ready = '0;
        end : memory_bus_reset
        else begin : memory_bus_state_machine

            case (memory_read_state)
            
            MEMORY_READ_STATE_IDLE: begin
                point_addr_valid = 0;
                point_data_ready = 0;

                current_memory_summary = summarize_memory_variables(memory_variable);

                if (current_memory_summary.out_of_date) begin
                    currently_reading = current_memory_summary.next_variable_to_read;
                    memory_variable[currently_reading].requests.accept = 1;
                    point_addr = memory_variable[currently_reading].offset * (vector::bits_in_single / 8);
                    point_addr_valid = 1;
                    memory_read_state = MEMORY_READ_STATE_ADDRESS_STAGE;
                    read_duration = 0;
                end
            end

            MEMORY_READ_STATE_ADDRESS_STAGE: begin
                read_duration++;
                if (point_addr_ready) begin
                    point_addr_valid = 0;
                    point_data_ready = 1;
                    memory_variable[currently_reading].memory_value.raw_bits = point_data;
                    memory_read_state = MEMORY_READ_STATE_DATA_STAGE;
                end

                if (read_duration >= maximum_read_latency) begin
                    point_addr_valid = 0;
                    point_data_ready = 0;
                    memory_read_state = MEMORY_READ_STATE_IDLE;
                    memory_variable[currently_reading].requests.error = 1;
                end
            end

            MEMORY_READ_STATE_DATA_STAGE: begin
                read_duration++;

                if (point_data_valid && point_resp != ransac::AXI_RESP_OKAY) begin
                    point_addr_valid = 0;
                    point_data_ready = 0;
                    memory_read_state = MEMORY_READ_STATE_IDLE;
                    memory_variable[currently_reading].requests.error = 1;
                end

                else if (read_duration >= maximum_read_latency) begin
                    point_addr_valid = 0;
                    point_data_ready = 0;
                    memory_read_state = MEMORY_READ_STATE_IDLE;
                    memory_variable[currently_reading].requests.error = 1;
                end

                else if (point_data_valid) begin
                    read_duration = 0;
                    point_addr_valid = 0;
                    point_data_ready = 0;
                    memory_variable[currently_reading].memory_value.raw_bits = point_data;
                    memory_variable[currently_reading].requests.finish = 1;

                    // if another point is pending, immediately go to the address stage
                    current_memory_summary = summarize_memory_variables(memory_variable);

                    if (current_memory_summary.out_of_date) begin
                        currently_reading = current_memory_summary.next_variable_to_read;
                        memory_variable[currently_reading].requests.accept = 1;
                        point_addr = memory_variable[currently_reading].offset * (vector::bits_in_single / 8);
                        point_addr_valid = 1;
                        memory_read_state = MEMORY_READ_STATE_ADDRESS_STAGE;
                    end else begin
                        memory_read_state = MEMORY_READ_STATE_IDLE;
                    end
                end

                

            end

            endcase
            
        end : memory_bus_state_machine
    end : handle_memory_bus

    // instantiate internal modules here

    derive_plane#(
        .latency_fma(plane_fma_latency),
        .reset_polarity(reset_polarity)
    ) derive_plane_instance(
        .clock(clock),
        .reset(reset),
        .ivalid(derive_plane_control.ivalid),
        .iready(derive_plane_control.iready),
        .a(derive_plane_control.a),
        .b(derive_plane_control.b),
        .c(derive_plane_control.c),
        .ovalid(derive_plane_control.ovalid),
        .oacknowledge(derive_plane_control.oacknowledge),
        .n(derive_plane_control.n),
        .d(derive_plane_control.d),
        .status(derive_plane_control.status)
    );

    genvar i;
    generate
        for(i = 0; i < check_inlier_instance_count; i++) begin
            check_inlier#(
                .latency_large_fma(large_fma_latency),
                .latency_small_fma(small_fma_latency),
                .reset_polarity(reset_polarity)
            ) check_inlier_instance(
                .clock(clock),
                .reset(reset),
                .ivalid(check_inlier_control[i].ivalid),
                .iready(check_inlier_control[i].iready),
                .n(check_inlier_control[i].n),
                .p(check_inlier_control[i].p),
                .d(check_inlier_control[i].d),
                .t(check_inlier_control[i].t),
                .ovalid(check_inlier_control[i].ovalid),
                .oacknowledge(check_inlier_control[i].oacknowledge),
                .inlier(check_inlier_control[i].inlier)
            );
        end
    endgenerate

    // logic to update the states for each memory variable
    generate
        for (i = 0; i < max_memory_variable_index; i++) begin
            always @(posedge clock) begin
                if (reset == reset_polarity || memory_variable[i].requests.reset) begin
                    memory_variable[i].state = MEMORY_DATA_STATE_KEEP_UNREAD;
                end 
                else begin
                    (* complete_case *)
                    case (memory_variable[i].state) 
                    MEMORY_DATA_STATE_ERROR: begin
                        // only changes with reset signal
                    end
                    MEMORY_DATA_STATE_KEEP_UNREAD: begin
                        if (memory_variable[i].requests.unread) begin
                            memory_variable[i].state = MEMORY_DATA_STATE_UNREAD;
                        end
                    end
                    MEMORY_DATA_STATE_READ: begin
                        if (memory_variable[i].requests.unread) begin
                            memory_variable[i].state = MEMORY_DATA_STATE_UNREAD;
                        end
                    end
                    MEMORY_DATA_STATE_READING: begin
                        if (memory_variable[i].requests.error) begin
                            memory_variable[i].state = MEMORY_DATA_STATE_ERROR;
                        end else if (memory_variable[i].requests.finish) begin
                            memory_variable[i].state = MEMORY_DATA_STATE_READ;
                        end
                    end
                    MEMORY_DATA_STATE_UNREAD: begin
                        if (memory_variable[i].requests.accept) begin
                            memory_variable[i].state = MEMORY_DATA_STATE_READING;
                        end
                    end
                    endcase
                end
            end
        end
    endgenerate

endmodule : plane_checking_unit