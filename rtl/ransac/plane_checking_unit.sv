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
        // TODO: re-implement handling bus errors.
        input logic [1:0] point_resp,
        // 1 if data is valid
        input logic point_data_valid,
        // 1 if the unit can accept data
        output logic point_data_ready
    );

    // this unit is responsible for managing a derive_plane unit and some
    // number of check_inlier units. To do this, we manage two state machines:
    // 1. a state machine controlling the calculations
    // 2. a state machine which reads to and from memory.
    //
    // For this initial implementation, we assume that data_width is the same
    // as bits_in_single. Later implementations may make different assumptions.
    //
    //
    // Memory controller states:
    // 1. Sits idle
    // 2. Reads points A, B, and C to derive a plane. The memory controller can
    //    also prefetch the first point to derive at this point.
    // 3. Reads the requested point + 1 until that would exceed the cloud
    // bounds
    //
    // To request a point at some offset:
    // 1. read the word at 3 * offset
    // 2. read the word at 3 * offset + bytes in single
    // 3. read the word at 3 * offset + 2 * bytes in single
    //
    // Since our code has an AXI bus (albeit a trimmed down one), this process can be split into a state
    // machine consisting of 3 states:
    //
    // Port Controller States:
    // 1. Idle
    // 2. Address Stage
    // 3. Data Stage
    //
    // This controller will not be super high performance, but since it takes
    // multiple cycles to derive a plane or check for an inlier, it can likely
    // keep up. (we then get a third state machine from this process.)

    // a scalar which has been read from memory
    typedef enum logic [3:0] {
        MEMORY_VARIABLE_AX, // x component of point A
        MEMORY_VARIABLE_AY, // y component of point A
        MEMORY_VARIABLE_AZ, // z component of point A
        MEMORY_VARIABLE_BX, // x component of point B
        MEMORY_VARIABLE_BY, // y component of point B
        MEMORY_VARIABLE_BZ, // z component of point B
        MEMORY_VARIABLE_CX, // x component of point C
        MEMORY_VARIABLE_CY, // y component of point C
        MEMORY_VARIABLE_CZ, // z component of point C
        MEMORY_VARIABLE_NX, // x component of the next point to check
        MEMORY_VARIABLE_NY, // y component of the next point to check
        MEMORY_VARIABLE_NZ  // z component of the next point to check
    } memory_variable_e;

    // the point that can be requested from the memory controller.
    typedef enum logic [1:0] {
        MEMORY_POINT_A, // plane point A
        MEMORY_POINT_B, // plane point B
        MEMORY_POINT_C, // plane point C
        MEMORY_POINT_N  // the next point
    } memory_point_e;

    // the variables read from memory. This register file is controlled by the
    // port controller.
    vector::single_t memory_file [0:11];
    // set by the port controller if it can handle a new request.
    logic memory_port_request_ready;
    // set by the memory controller if it wants to issue a new request.
    logic memory_port_request_valid;
    // set by the memory controller if it is ready to receive the results of a
    // finished request.
    logic memory_port_response_ready;
    // set by the port controller if a request has finished.
    logic memory_port_response_valid;
    // requested destination variable.
    memory_variable_e memory_port_destination;
    // requested point address (in words!)
    logic [addr_width-1:0] memory_port_addr;
    // data read from memory.
    vector::single_t memory_port_data;
    // the port controller's copy of where to write the memory variable to.
    memory_variable_e memory_port_pending_destination;

    enum {
        PORT_STATE_IDLE,
        PORT_STATE_ADDR,
        PORT_STATE_DATA
    } port_state;

    always_ff @(posedge clock) begin : manage_port
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < 12; i++) begin
                memory_file[i] = '0;
            end
            memory_port_request_ready <= 1;
            memory_port_response_valid <= 0;
            memory_port_data <= '0;
            point_addr <= '0;
            point_addr_valid <= '0;
            point_data_ready <= '0;
            port_state <= PORT_STATE_IDLE;
            memory_port_pending_destination <= '0;
        end else begin
            case (port_state)
            PORT_STATE_IDLE: begin
                point_data_ready <= '0;
                if (memory_port_response_valid && memory_port_response_ready) begin
                    memory_port_response_valid <= '0;
                    memory_port_request_ready <= 1;
                    memory_file[memory_port_pending_destination] <= memory_port_data;
                end else if (memory_port_request_valid) begin
                    point_addr <= memory_port_addr;
                    point_addr_valid <= 1;
                    memory_port_request_ready <= 0;
                    memory_port_pending_destination <= memory_port_destination;
                    port_state <= PORT_STATE_ADDR;
                    memory_port_response_valid <= '0;
                end
            end
            PORT_STATE_ADDR: begin

                if (point_addr_valid && point_addr_ready) begin
                    point_addr_valid <= '0;
                    point_data_ready <= '1;
                    port_state <= PORT_STATE_DATA;
                end
            end
            PORT_STATE_DATA: begin
                if (point_data_valid && point_data_ready) begin
                    memory_port_response_valid <= 1;
                    memory_port_data <= point_data;
                    point_data_ready <= '0;
                    port_state <= PORT_STATE_IDLE;
                end
            end
            endcase
        end
    end : manage_port

    // the memory controller takes requests to read a point at an offset into
    // the cloud and converts it into a sequence of requests to the port
    // controller.

    // the memory controller is ready to accept an input. If this bit is a 1,
    // then it is understood that the previous request has also finished.
    logic memory_command_ready;
    // the command is valid.
    logic memory_command_valid;
    // the offset of the requested point in the cloud.
    logic [$clog2(maximum_points)-1:0] memory_command_point_offset;
    // which point is being requested.
    memory_point_e memory_command_point;

    logic [addr_width-1:0] memory_command_x_addr;
    logic [addr_width-1:0] memory_command_y_addr;
    logic [addr_width-1:0] memory_command_z_addr;
    memory_variable_e memory_command_x_dest;
    memory_variable_e memory_command_y_dest;
    memory_variable_e memory_command_z_dest;


    enum {
        MEMORY_CONTROLLER_IDLE, // no active request
        MEMORY_CONTROLLER_GETX, // retrieve the X component
        MEMORY_CONTROLLER_GETY, // retrieve the Y component
        MEMORY_CONTROLLER_GETZ  // retrieve the Z component
    } memory_controller_state;

    // for now, always ready to receive the next point.
    assign memory_port_response_ready = 1;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            memory_port_request_valid <= '0;
            memory_port_destination <= '0;
            memory_port_addr <= '0;
            memory_command_ready <= '1;
            memory_command_x_addr <= '0;
            memory_command_y_addr <= '0;
            memory_command_z_addr <= '0;
            memory_command_x_dest <= '0;
            memory_command_y_dest <= '0;
            memory_command_z_dest <= '0;
            memory_controller_state <= MEMORY_CONTROLLER_IDLE;
        end else begin
            case (memory_controller_state)
            MEMORY_CONTROLLER_IDLE: begin
                case (memory_command_point)
                MEMORY_POINT_A: begin
                    memory_port_destination <= MEMORY_VARIABLE_AX;
                    memory_command_x_dest <= MEMORY_VARIABLE_AX;
                    memory_command_y_dest <= MEMORY_VARIABLE_AY;
                    memory_command_z_dest <= MEMORY_VARIABLE_AZ;
                end
                MEMORY_POINT_B: begin
                    memory_port_destination <= MEMORY_VARIABLE_BX;
                    memory_command_x_dest <= MEMORY_VARIABLE_BX;
                    memory_command_y_dest <= MEMORY_VARIABLE_BY;
                    memory_command_z_dest <= MEMORY_VARIABLE_BZ;
                end
                MEMORY_POINT_C: begin
                    memory_port_destination <= MEMORY_VARIABLE_CX;
                    memory_command_x_dest <= MEMORY_VARIABLE_CX;
                    memory_command_y_dest <= MEMORY_VARIABLE_CY;
                    memory_command_z_dest <= MEMORY_VARIABLE_CZ;
                end
                MEMORY_POINT_N: begin
                    memory_port_destination <= MEMORY_VARIABLE_NX;
                    memory_command_x_dest <= MEMORY_VARIABLE_NX;
                    memory_command_y_dest <= MEMORY_VARIABLE_NY;
                    memory_command_z_dest <= MEMORY_VARIABLE_NZ;
                end
                endcase
                memory_port_addr <= 12 * memory_command_point_offset + 0;
                memory_command_x_addr <= 12 * memory_command_point_offset + 0;
                memory_command_y_addr <= 12 * memory_command_point_offset + 4;
                memory_command_z_addr <= 12 * memory_command_point_offset + 8;

                if (!memory_command_ready) begin
                    memory_command_ready <= 1;
                end

                if (memory_command_ready && memory_command_valid) begin
                    if (!memory_port_request_ready) begin
                        $error("Invalid condition: memory request active when not reading a point!");
                        $finish(2);
                    end
                    memory_command_ready <= '0;
                    memory_controller_state <= MEMORY_CONTROLLER_GETX;
                    memory_port_request_valid <= 1;
                    // memory_port_destination <= memory_command_x_dest;
                    // memory_port_addr <= memory_command_x_addr;
                end
            end
            MEMORY_CONTROLLER_GETX: begin
                memory_port_request_valid <= 1;
                memory_port_destination <= memory_command_y_dest;
                memory_port_addr <= memory_command_y_addr;
                if (memory_port_response_valid) begin
                    memory_controller_state <= MEMORY_CONTROLLER_GETY;
                end
            end
            MEMORY_CONTROLLER_GETY: begin
                memory_port_request_valid <= 1;
                memory_port_destination <= memory_command_z_dest;
                memory_port_addr <= memory_command_z_addr;
                if (memory_port_response_valid) begin
                    memory_controller_state <= MEMORY_CONTROLLER_GETZ;
                end
            end
            MEMORY_CONTROLLER_GETZ: begin
                memory_port_request_valid <= 0;
                if (memory_port_response_valid) begin
                    memory_command_ready <= 1;
                    memory_controller_state <= MEMORY_CONTROLLER_IDLE;
                end
            end
            endcase
        end
    end

    // the logic for controlling the various processing units and asking for 
    // points from memory.
    
    // control port to derive planes.
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

    // control port to each check inlier unit.
    struct packed {
        logic ivalid;
        logic iready;
        vector::vector3s_s n;
        vector::vector3s_s p;
        vector::single_t d;
        vector::single_t t;
        logic ovalid;
        logic oacknowledge;
        logic inlier;
    } check_inlier_control[check_inlier_instance_count-1:0];

    // issue requests to the check inlier units in a round-robin fashion.
    logic [check_inlier_instance_count == 0 ? 0 : ($clog2(check_inlier_instance_count)-1):0] next_inlier_unit;
    // quantity of inliers found
    logic [$clog2(maximum_points)-1:0] running_inlier_count;
    // how many points have been tested in this iteration.
    logic [$clog2(maximum_points)-1:0] tested_count;
    // copy of the configuration variables.
    logic [$clog2(maximum_points)-1:0] captured_cloud_length;
    logic [2:0][$clog2(maximum_points)-1:0] captured_plane_point_offset;
    vector::vector3s_s found_n;
    vector::single_t found_d;
    vector::single_t captured_threshold;

    logic [$clog2(check_inlier_instance_count)-1:0] check_inliers_finished_this_cycle;
    logic [$clog2(check_inlier_instance_count)-1:0] inliers_found_this_cycle;

    always_comb begin
        check_inliers_finished_this_cycle = 0;
        inliers_found_this_cycle = 0;
        for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
            if (check_inlier_control[i].ovalid) begin
                check_inliers_finished_this_cycle++;
                inliers_found_this_cycle += check_inlier_control[i].inlier;
            end
        end
    end

    enum {
        // the idle state
        CALCULATION_STATE_IDLE,
        // request point A.
        CALCULATION_STATE_REQUEST_A,
        // waits on point A
        CALCULATION_STATE_FETCH_A,
        // waits on point B
        CALCULATION_STATE_FETCH_B,
        // waits on point C
        CALCULATION_STATE_FETCH_C,
        // waits on the first point to check and waits on the plane derived from
        // A, B, and C.
        CALCULATION_STATE_PREFETCH_FIRST_N,
        // main calculation loop:
        // -> for each check inlier unit with finished results, acknowledge the
        //    result, increment running_inlier_count if inlier and unconditionally
        //    increment the tested count.
        // -> if tested_count == captured_cloud_length -> break and go to idle
        // -> if we increment tested count at all, fetch the next point
        //
        // note: since check_inlier happens to always take the same amount of time
        // to calculate whether a point is an inlier, this logic only needs to 
        // handle the case when any of the units finish on this cycle.
        CALCULATION_STATE_CALCULATE
    } calculation_state;
    // because of handshake signal timing issues mentioned in the
    // implementation for CALCULATION_STATE_REQUEST_A, this variable
    // exists to prevent the calculation state machine from requesting a point
    // from the cloud to test on two consecutive cycles. With how the 
    // memory controller is programmed, this feat should not be possible anywyay.
    logic may_request_memory;

    assign derive_plane_control.a.v.x = memory_file[MEMORY_VARIABLE_AX];
    assign derive_plane_control.a.v.y = memory_file[MEMORY_VARIABLE_AY];
    assign derive_plane_control.a.v.z = memory_file[MEMORY_VARIABLE_AZ];
    assign derive_plane_control.b.v.x = memory_file[MEMORY_VARIABLE_BX];
    assign derive_plane_control.b.v.y = memory_file[MEMORY_VARIABLE_BY];
    assign derive_plane_control.b.v.z = memory_file[MEMORY_VARIABLE_BZ];
    assign derive_plane_control.c.v.x = memory_file[MEMORY_VARIABLE_CX];
    assign derive_plane_control.c.v.y = memory_file[MEMORY_VARIABLE_CY];
    assign derive_plane_control.c.v.z = memory_file[MEMORY_VARIABLE_CZ];

    always_comb begin
        for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
            for (int unsigned j = 0; j < 3; j++) begin
                check_inlier_control[i].n.c[j] = derive_plane_control.n.c[j];
            end
            check_inlier_control[i].d = derive_plane_control.d;
            check_inlier_control[i].t = captured_threshold;
            check_inlier_control[i].p.v.x = memory_file[MEMORY_VARIABLE_NX];
            check_inlier_control[i].p.v.y = memory_file[MEMORY_VARIABLE_NY];
            check_inlier_control[i].p.v.z = memory_file[MEMORY_VARIABLE_NZ];
            check_inlier_control[i].oacknowledge = 1;
        end

        found_n = derive_plane_control.n;
        found_d = derive_plane_control.d;
    end

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            iready <= '1;
            ovalid <= '0;
            inlier_count <= '0;
            status <= ransac::PLANE_CHECKING_UNIT_STATUS_SUCCESS;
            for (int unsigned i = 0; i < 3; i++) begin
                plane_n.c[i] <= '0;
            end
            plane_d <= '0;
            memory_command_valid <= '0;
            memory_command_point_offset <= '0;
            memory_command_point <= '0;
            derive_plane_control.ivalid <= '0;
            for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                check_inlier_control[i].ivalid <= 0;
            end
            next_inlier_unit <= '0;
            running_inlier_count <= '0;
            tested_count <= '0;
            captured_cloud_length <= '0;
            for (int unsigned i = 0; i < 3; i++) begin
                captured_plane_point_offset[i] <= '0;
            end
            captured_threshold <= '0;
            calculation_state <= CALCULATION_STATE_IDLE;
            may_request_memory <= 1;
        end else begin
            case (calculation_state)
            CALCULATION_STATE_IDLE: begin
                derive_plane_control.oacknowledge <= 1;
                may_request_memory <= 1;
                for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                    check_inlier_control[i].ivalid <= 0;
                end
                captured_threshold <= threshold;
                captured_cloud_length <= cloud_length;
                for (int unsigned i = 0; i < 3; i++) begin
                    captured_plane_point_offset[i] <= plane_point_start_offsets[i];
                end
                tested_count <= '0;
                running_inlier_count <= '0;

                if (ovalid && oacknowledge) begin
                    ovalid <= '0;
                    iready <= '1;
                end
                if (ivalid && iready) begin
                    iready <= '0;
                    calculation_state <= CALCULATION_STATE_REQUEST_A;
                    memory_command_valid <= 1;
                    memory_command_point_offset <= plane_point_start_offsets[0];
                    memory_command_point <= MEMORY_POINT_A;
                end
            end
            CALCULATION_STATE_REQUEST_A: begin
                // This state is a 1 cycle delay inserted because of some oddity
                // with the handshake signals between the calculation state 
                // controller and the memory request state controller that seems
                // to leave the memory controller in the idle state one cycle 
                // longer than intended.
                calculation_state <= CALCULATION_STATE_FETCH_A;
            end
            CALCULATION_STATE_FETCH_A: begin
                // don't immediately acknowledge the output on the derive plane
                // control so that there is no race condition later between the
                // first point to read and the plane derived.
                derive_plane_control.oacknowledge <= '0;
                memory_command_valid <= 1;
                memory_command_point_offset <= captured_plane_point_offset[1];
                memory_command_point <= MEMORY_POINT_B;
                if (memory_command_ready) begin
                    calculation_state <= CALCULATION_STATE_FETCH_B;
                end
            end
            CALCULATION_STATE_FETCH_B: begin
                memory_command_valid <= 1;
                memory_command_point_offset <= captured_plane_point_offset[2];
                memory_command_point <= MEMORY_POINT_C;
                if (memory_command_ready) begin
                    calculation_state <= CALCULATION_STATE_FETCH_C;
                end
            end
            CALCULATION_STATE_FETCH_C: begin
                memory_command_valid <= 1;
                memory_command_point_offset <= '0;
                memory_command_point <= MEMORY_POINT_N;
                if (memory_command_ready) begin
                    calculation_state <= CALCULATION_STATE_PREFETCH_FIRST_N;
                    derive_plane_control.ivalid <= 1;
                    if (!derive_plane_control.iready) begin
                        $error("Derive plane unit not ready!");
                        $finish(2);
                    end
                end
            end
            CALCULATION_STATE_PREFETCH_FIRST_N: begin
                derive_plane_control.ivalid <= 0;
                // have plane and first point. Start the first check inlier
                // unit on this cycle and request the next point...
                // if we succeeded in deriving the plane.
                if (derive_plane_control.ovalid && memory_command_ready) begin
                    case (derive_plane_control.status)
                    // uh oh!
                    vector::DERIVE_PLANE_STATUS_LESS_THAN_THREE_UNIQUE_POINTS: begin
                        status <= ransac::PLANE_CHECKING_UNIT_STATUS_DERIVE_PLANE_ERROR;
                        calculation_state <= CALCULATION_STATE_IDLE;
                        ovalid <= 1;
                    end
                    // what we expect to happen.
                    vector::DERIVE_PLANE_STATUS_SUCCESS: begin
                        memory_command_point_offset <= 1;
                        memory_command_valid <= 1;
                        memory_command_point <= MEMORY_POINT_N;
                        check_inlier_control[next_inlier_unit].ivalid <= 1;
                        calculation_state <= CALCULATION_STATE_CALCULATE;
                        may_request_memory <= 0;
                        // special case for when there is only one check inlier unit.
                        if (check_inlier_instance_count > 1) begin
                            next_inlier_unit <= 1;
                        end
                    end
                    endcase
                end
            end
            CALCULATION_STATE_CALCULATE: begin
                tested_count <= tested_count + check_inliers_finished_this_cycle;
                running_inlier_count <= running_inlier_count + inliers_found_this_cycle;
                
                // for each check inlier unit, if ivalid, clear ivalid
                for (int unsigned i = 0; i < check_inlier_instance_count; i++) begin
                    if (check_inlier_control[i].ivalid) begin
                        check_inlier_control[i].ivalid <= 0;
                    end
                end

                if (!may_request_memory) begin
                    may_request_memory <= 1;
                end

                // if the next point is ready and the next check inlier unit is
                // ready and the next point would be in bounds
                if (may_request_memory && 
                    memory_command_ready && 
                    check_inlier_control[next_inlier_unit].iready && 
                    memory_command_point_offset + 1 < captured_cloud_length) begin
                    
                    memory_command_valid <= 1;
                    memory_command_point_offset <= memory_command_point_offset + 1;
                    memory_command_point <= MEMORY_POINT_N;
                    check_inlier_control[next_inlier_unit].ivalid <= 1;
                    may_request_memory <= 0;
                    if (next_inlier_unit + 1 == check_inlier_instance_count) begin
                        next_inlier_unit <= 0;
                    end else begin
                        next_inlier_unit <= next_inlier_unit + 1;
                    end
                end else begin
                    // don't read the same point twice if it can be avoided.
                    memory_command_valid <= 0;
                end

                if (tested_count + check_inliers_finished_this_cycle >= captured_cloud_length - 1) begin
                    calculation_state <= CALCULATION_STATE_IDLE;
                    plane_n <= found_n;
                    plane_d <= found_d;
                    inlier_count <= running_inlier_count;
                    status <= ransac::PLANE_CHECKING_UNIT_STATUS_SUCCESS;
                    ovalid <= 1;
                end
            end
            endcase
        end
    end

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
                .latency_fma(small_fma_latency),
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


endmodule : plane_checking_unit