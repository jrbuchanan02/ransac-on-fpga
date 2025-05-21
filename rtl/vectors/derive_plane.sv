`timescale 1ns/1ps

// derives a plane from 3 points over multiple cycles
// while there may exist some rounding error between
// the generated plane and the actual plane, this
// module will guarantee that the resulting plane is
// at least close and that n and d are small enough to
// prevent an unkind integer overflow within check_inlier
//
// it's theoretically possible for at least one of the internal values v1 and v2
// to be the zero vector, in this case, the result of the calculation is the plane
// defined by n = 0 and d = 0 and status indicates that less than 3 unique points 
// exist to define the plane.
module derive_plane#(
        parameter int unsigned latency_fma = vector::fma_latency_doubles,
        parameter int unsigned reset_polarity = 1
    )(
        input logic clock,
        input logic reset,

        input logic ivalid,
        output logic iready,
        input vector::point_t a,
        input vector::point_t b,
        input vector::point_t c,

        output logic ovalid,
        input logic oacknowledge,
        output vector::vector3s_s n,
        output vector::single_t d,
        output vector::derive_plane_status_e status
    );

    // steps for deriving the plane:
    // 1. find v1 and v2 (use adder / subtracter which may or may not have a latency)
    // 2. find n = v1 x v2 (use small fma unit)
    // 3. find d = n * a (use large fma unit)
    // 4. round n and d to use single_t for each component (use fp_truncate)
    
    // we can use one fp_fma unit for all of these calculations provided that
    // we appropriately truncate the values of v1, v2 to single_t (which they are
    // already capable of fitting in that type). Doing so will also allow using only
    // one FMA unit in total.
    //
    // to do this, we're basically creating a teeny-tiny out-of-order processor
    // with one instruction: fma
    //
    // We will store each variable as a vector::double_t, the inputs to a and b
    // of the FMA unit will be truncated from single_t to double_t.

    // store our inputs

    typedef enum logic [4:0] {
        // captured inputs
        VARIABLE_AX,    // input
        VARIABLE_AY,    // input
        VARIABLE_AZ,    // input
        VARIABLE_BX,    // input
        VARIABLE_BY,    // input
        VARIABLE_BZ,    // input
        VARIABLE_CX,    // input
        VARIABLE_CY,    // input
        VARIABLE_CZ,    // input
        // v1 and v2
        VARIABLE_V1X,   // init to 0, ax - bx
        VARIABLE_V1Y,   // init to 0, ay - by
        VARIABLE_V1Z,   // init to 0, az - bz
        VARIABLE_V2X,   // init to 0, ax - cx
        VARIABLE_V2Y,   // init to 0, ay - cy
        VARIABLE_V2Z,   // init to 0, az - cz
        // n
        VARIABLE_NX1,   // init to 0, v1z * v2y
        VARIABLE_NY1,   // init to 0, v1x * v2z
        VARIABLE_NZ1,   // init to 0, v1y * v2x
        VARIABLE_NX2,   // init to 0, v1y * v2z - nx1
        VARIABLE_NY2,   // init to 0, v1z * v2x - ny1
        VARIABLE_NZ2,   // init to 0, v1x * v2y - nz1
        // d
        VARIABLE_DX,    // init to 0, nx2 * ax
        VARIABLE_DY,    // init to 0, ny2 * ay + dx
        VARIABLE_DZ,    // init to 0, nz2 * az + dy
        // constants
        VARIABLE_ONE,       // +1
        VARIABLE_NEG_ONE,   // -1
        VARIABLE_ZERO       //  0
    } variable_e;

    typedef enum logic[1:0] {
        VARIABLE_STATE_PENDING,
        VARIABLE_STATE_RUNNING,
        VARIABLE_STATE_FINISHED
    } variable_state_e;

    typedef struct packed {
        vector::quad_t value;
        variable_state_e state;
        struct packed {
            variable_e a;
            variable_e b;
            variable_e c;
        } inputs;
        logic negate_c;
    } variable_s;

    variable_s [26:0] variable;

    struct packed {
        vector::double_t a;
        vector::double_t b;
        vector::quad_t c;
        vector::quad_t r;
        logic iready;
        logic ivalid;
        logic ovalid;
        logic oacknowledge;
        logic [4:0] iid;
        logic [4:0] oid;
    } fma_control;

    typedef struct packed {
        vector::quad_t ivalue;
        vector::double_t ovalue;
    } truncate_control_s;

    truncate_control_s truncate_a;
    truncate_control_s truncate_b;

    enum logic [1:0] {
        STATE_IDLE,
        STATE_SCALE_CHECK,
        STATE_CALC
    } state;

    logic found_params;
    logic calculations_remain;

    vector::quad_t d_absolute_value;

    always @(posedge clock) begin : main_logic
    
        // default the negate_c flag to "no" for all values (only needed in 
        // the second part of calculating N)
        for (int unsigned i = 0; i < 27; i++) begin
            variable[i].negate_c = 0;
        end

        

        // inputs to each variable, each variable depends on its inputs

        // constant value inputs and module input variable inputs
        // don't really matter since they should never be
        // calculated, assign them zero just in case
        variable[VARIABLE_AX].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_AX].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_AX].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_AY].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_AY].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_AY].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_AZ].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_AZ].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_AZ].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_BX].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_BX].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_BX].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_BY].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_BY].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_BY].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_BZ].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_BZ].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_BZ].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_CX].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_CX].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_CX].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_CY].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_CY].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_CY].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_CZ].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_CZ].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_CZ].inputs.c = VARIABLE_ZERO;

        variable[VARIABLE_ONE].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_ONE].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_ONE].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_NEG_ONE].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_NEG_ONE].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_NEG_ONE].inputs.c = VARIABLE_ZERO;
        variable[VARIABLE_ZERO].inputs.a = VARIABLE_ZERO;
        variable[VARIABLE_ZERO].inputs.b = VARIABLE_ZERO;
        variable[VARIABLE_ZERO].inputs.c = VARIABLE_ZERO;

        // v1 calculation inputs
        variable[VARIABLE_V1X].inputs.a = VARIABLE_BX;
        variable[VARIABLE_V1X].inputs.b = VARIABLE_NEG_ONE;
        variable[VARIABLE_V1X].inputs.c = VARIABLE_AX;
        variable[VARIABLE_V1Y].inputs.a = VARIABLE_BY;
        variable[VARIABLE_V1Y].inputs.b = VARIABLE_NEG_ONE;
        variable[VARIABLE_V1Y].inputs.c = VARIABLE_AY;
        variable[VARIABLE_V1Z].inputs.a = VARIABLE_BZ;
        variable[VARIABLE_V1Z].inputs.b = VARIABLE_NEG_ONE;
        variable[VARIABLE_V1Z].inputs.c = VARIABLE_AZ;

        // v2 calculation inputs
        variable[VARIABLE_V2X].inputs.a = VARIABLE_CX;
        variable[VARIABLE_V2X].inputs.b = VARIABLE_NEG_ONE;
        variable[VARIABLE_V2X].inputs.c = VARIABLE_AX;
        variable[VARIABLE_V2Y].inputs.a = VARIABLE_CY;
        variable[VARIABLE_V2Y].inputs.b = VARIABLE_NEG_ONE;
        variable[VARIABLE_V2Y].inputs.c = VARIABLE_AY;
        variable[VARIABLE_V2Z].inputs.a = VARIABLE_CZ;
        variable[VARIABLE_V2Z].inputs.b = VARIABLE_NEG_ONE;
        variable[VARIABLE_V2Z].inputs.c = VARIABLE_AZ;

        // first step of finding n inputs (x-component)
        variable[VARIABLE_NX1].inputs.a = VARIABLE_V1Z;
        variable[VARIABLE_NX1].inputs.b = VARIABLE_V2Y;
        variable[VARIABLE_NX1].inputs.c = VARIABLE_ZERO;

        // first step of finding n inputs (y-component)
        variable[VARIABLE_NY1].inputs.a = VARIABLE_V1X;
        variable[VARIABLE_NY1].inputs.b = VARIABLE_V2Z;
        variable[VARIABLE_NY1].inputs.c = VARIABLE_ZERO;

        // first step of finding n inputs (z-component)
        variable[VARIABLE_NZ1].inputs.a = VARIABLE_V1Y;
        variable[VARIABLE_NZ1].inputs.b = VARIABLE_V2X;
        variable[VARIABLE_NZ1].inputs.c = VARIABLE_ZERO;

        // second step of finding n inputs (x-component)
        variable[VARIABLE_NX2].inputs.a = VARIABLE_V1Y;
        variable[VARIABLE_NX2].inputs.b = VARIABLE_V2Z;
        variable[VARIABLE_NX2].inputs.c = VARIABLE_NX1;
        variable[VARIABLE_NX2].negate_c = 1;

        // first step of finding n inputs (y-component)
        variable[VARIABLE_NY2].inputs.a = VARIABLE_V1Z;
        variable[VARIABLE_NY2].inputs.b = VARIABLE_V2X;
        variable[VARIABLE_NY2].inputs.c = VARIABLE_NY1;
        variable[VARIABLE_NY2].negate_c = 1;

        // first step of finding n inputs (z-component)
        variable[VARIABLE_NZ2].inputs.a = VARIABLE_V1X;
        variable[VARIABLE_NZ2].inputs.b = VARIABLE_V2Y;
        variable[VARIABLE_NZ2].inputs.c = VARIABLE_NZ1;
        variable[VARIABLE_NZ2].negate_c = 1;

        // dot product of N and D inputs (x-component)
        variable[VARIABLE_DX].inputs.a = VARIABLE_NX2;
        variable[VARIABLE_DX].inputs.b = VARIABLE_AX;
        variable[VARIABLE_DX].inputs.c = VARIABLE_ZERO;
        // dot product of N and D inputs (y-component)
        variable[VARIABLE_DY].inputs.a = VARIABLE_NY2;
        variable[VARIABLE_DY].inputs.b = VARIABLE_AY;
        variable[VARIABLE_DY].inputs.c = VARIABLE_DX;
        // dot product of N and D inputs (z-component)
        variable[VARIABLE_DZ].inputs.a = VARIABLE_NZ2;
        variable[VARIABLE_DZ].inputs.b = VARIABLE_AZ;
        variable[VARIABLE_DZ].inputs.c = VARIABLE_DY;

        if (reset == reset_polarity) begin : handle_reset
            state <= STATE_IDLE;
            iready <= 1;
            ovalid <= 0;
            for (int unsigned i = 0; i < 3; i++) begin
                n.c[i] <= '0;
            end
            d <= '0;
            status <= vector::DERIVE_PLANE_STATUS_SUCCESS;
            fma_control.ivalid <= 0;
            fma_control.oacknowledge <= 0;
        end : handle_reset
        else begin : state_machine

            case (state)

            STATE_CALC: begin
                // 1. check for a pending result
                if (fma_control.ovalid) begin : pending_result_check
                    fma_control.oacknowledge <= 1;
                    
                    // determine if the result even makes sense
                    if (fma_control.oid > VARIABLE_ZERO) begin
                        $fatal(1, "Illegal oid value in derive_plane/fma unit");
                    end
                    else if (variable[fma_control.oid].state != VARIABLE_STATE_RUNNING) begin
                        $fatal(1, "Illegal oid value in derive_plane/fma unit");
                    end
                    else begin
                        // blocking assignments here to allow for the results to be used immediately
    
                        variable[fma_control.oid].value = fma_control.r;
                        variable[fma_control.oid].state = VARIABLE_STATE_FINISHED;
                    end
                end : pending_result_check
                if (fma_control.iready) begin : try_submit_new_calc    
                    found_params = 0;
                    for (int unsigned i = 0; i < 27; i++) begin : find_next_calc
                        if (!found_params &&
                            variable[i].state == VARIABLE_STATE_PENDING &&
                            variable[variable[i].inputs.a].state == VARIABLE_STATE_FINISHED &&
                            variable[variable[i].inputs.b].state == VARIABLE_STATE_FINISHED &&
                            variable[variable[i].inputs.c].state == VARIABLE_STATE_FINISHED) begin
                            
                            found_params = 1;
                            truncate_a.ivalue <= variable[variable[i].inputs.a].value;
                            truncate_b.ivalue <= variable[variable[i].inputs.b].value;
                            fma_control.iid <= i;
                            fma_control.c <= variable[i].negate_c ? -variable[variable[i].inputs.c].value : variable[variable[i].inputs.c].value;
                            fma_control.ivalid <= 1;
                            variable[i].state = VARIABLE_STATE_RUNNING;
                        end
                    end : find_next_calc
                    if (!found_params) begin
                        fma_control.ivalid <= 0;
                    end
                end : try_submit_new_calc
                // check if any more calculations remain, if not, then present the results,
                // go to the idle state, etc. etc.
                calculations_remain = 0;
                for (int unsigned i = 0; i < 27; i++) begin : check_if_done
                    if (variable[i].state != VARIABLE_STATE_FINISHED) begin
                        calculations_remain = 1;
                    end
                end : check_if_done

                if (!calculations_remain) begin
                    state <= STATE_SCALE_CHECK;
                end
            end

            STATE_SCALE_CHECK: begin
                // overflow could be possible in check_inlier when
                // adding D to a value reasonably exceeds the bit limit
                // 
                // conservatively, this will be defined when abs(d) is 
                // at least half the maximum allowed value (i.e., for 
                // 12.20 fp, if |d| is above 1024).
                
                // continue this adjustment for as many cycles as necessary
                d_absolute_value = vector::abs_quad(variable[VARIABLE_DZ].value);
                if (d_absolute_value[vector::bits_in_quad - 1-:vector::quad_ibits] > (1 << (vector::single_ibits - 2))) begin
                    variable[VARIABLE_DZ].value  = variable[VARIABLE_DZ].value >>> 1;
                    variable[VARIABLE_NX2].value = variable[VARIABLE_NX2].value >>> 1;
                    variable[VARIABLE_NY2].value = variable[VARIABLE_NY2].value >>> 1;
                    variable[VARIABLE_NZ2].value = variable[VARIABLE_NZ2].value >>> 1;
                end
                else begin
                    n.v.x = vector::double_to_single(vector::quad_to_double(variable[VARIABLE_NX2].value));
                    n.v.y = vector::double_to_single(vector::quad_to_double(variable[VARIABLE_NY2].value));
                    n.v.z = vector::double_to_single(vector::quad_to_double(variable[VARIABLE_NZ2].value));
                    d = vector::double_to_single(vector::quad_to_double(variable[VARIABLE_DZ].value));
                    ovalid <= 1;
                    state <= STATE_IDLE;
                    
                    
                    // if n is (or will be) the 0 vector, then there must have been less than
                    // 3 unique points.
                    
                    if (n.v.x == '0 && n.v.y == '0 && n.v.z == '0) begin
                        status <= vector::DERIVE_PLANE_STATUS_LESS_THAN_THREE_UNIQUE_POINTS;
                    end else begin
                        status <= vector::DERIVE_PLANE_STATUS_SUCCESS;
                    end
                end
                
                
            end

            STATE_IDLE: begin

                fma_control.ivalid <= 0;
                fma_control.oacknowledge <= 1;

                for (int unsigned i = 0; i < 27; i++) begin
                    variable[i].state = VARIABLE_STATE_PENDING;
                    variable[i].value = '0;
                end

                if (ovalid && oacknowledge && !iready) begin
                    iready <= 1;
                end else if (ivalid && iready) begin
                    ovalid <= 0;
                    iready <= 0;
                    state <= STATE_CALC;
                end

                // capture inputs on every cycle spent idle
                variable[VARIABLE_AX].value = $signed(a.v.x);
                variable[VARIABLE_AY].value = $signed(a.v.y);
                variable[VARIABLE_AZ].value = $signed(a.v.z);
                variable[VARIABLE_BX].value = $signed(b.v.x);
                variable[VARIABLE_BY].value = $signed(b.v.y);
                variable[VARIABLE_BZ].value = $signed(b.v.z);
                variable[VARIABLE_CX].value = $signed(c.v.x);
                variable[VARIABLE_CY].value = $signed(c.v.y);
                variable[VARIABLE_CZ].value = $signed(c.v.z);

                variable[VARIABLE_AX].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_AY].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_AZ].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_BX].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_BY].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_BZ].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_CX].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_CY].value <<<= vector::quad_fbits - vector::single_fbits;
                variable[VARIABLE_CZ].value <<<= vector::quad_fbits - vector::single_fbits;
            end

            endcase

        end : state_machine
        
                // inputs are special: they aren't technically constants but they are
        // not calculated either. As such, their state is always "finished"
        variable[VARIABLE_AX].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_AY].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_AZ].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_BX].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_BY].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_BZ].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_CX].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_CY].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_CZ].state = VARIABLE_STATE_FINISHED;
        // constants are always calculated and their value is hard-coded
        variable[VARIABLE_ONE].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_NEG_ONE].state = VARIABLE_STATE_FINISHED;
        variable[VARIABLE_ZERO].state = VARIABLE_STATE_FINISHED;

        variable[VARIABLE_ONE].value = { {vector::quad_ibits - 1{1'b0}}, 1'b1, {vector::quad_fbits{1'b0}} };
        variable[VARIABLE_NEG_ONE].value = -variable[VARIABLE_ONE].value;
        variable[VARIABLE_ZERO].value = '0;

    end : main_logic

    // connect the wires for fma_control and fma_truncate
    always_comb begin : connect_wires
        fma_control.a = truncate_a.ovalue;
        fma_control.b = truncate_b.ovalue;
    end : connect_wires

    fp_truncate#(
        .iibits(vector::quad_ibits),
        .ifbits(vector::quad_fbits),
        .oibits(vector::double_ibits),
        .ofbits(vector::double_fbits),
        .signed_values(1)
    ) fp_truncate_a(
        .ivalue(truncate_a.ivalue),
        .ovalue(truncate_a.ovalue)
    );

    fp_truncate#(
        .iibits(vector::quad_ibits),
        .ifbits(vector::quad_fbits),
        .oibits(vector::double_ibits),
        .ofbits(vector::double_fbits),
        .signed_values(1)
    ) fp_truncate_b(
        .ivalue(truncate_b.ivalue),
        .ovalue(truncate_b.ovalue)
    );

    fp_fma#(
        .reset_polarity(reset_polarity),
        .ibits(vector::double_ibits),
        .fbits(vector::double_fbits),
        .latency(latency_fma),
        .id_bits($bits(fma_control.iid))
    ) fma_unit(
        .clock(clock),
        .reset(reset),
        .a(fma_control.a),
        .b(fma_control.b),
        .c(fma_control.c),
        .r(fma_control.r),
        .iid(fma_control.iid),
        .oid(fma_control.oid),
        .iready(fma_control.iready),
        .ivalid(fma_control.ivalid),
        .ovalid(fma_control.ovalid),
        .oacknowledge(fma_control.oacknowledge)
    );

endmodule : derive_plane