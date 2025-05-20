`timescale 1ns / 1ps

`include "vectors/vector_pkg.svh"

// checks if a point lies within some plane given multiple cycles to do so.
//
// the check_inlier module is designed such that in the future it might process
// multiple points at a time (reasonably two or three per small_fma instance)
// and that the fma units may at some future point output their results in a 
// variable number of cycles or even out of order (e.g., multiplication 
// operations ending early)
module check_inlier#(
        parameter int unsigned latency_small_fma = vector::fma_latency_singles,
        parameter int unsigned latency_large_fma = vector::fma_latency_doubles,
        parameter bit reset_polarity = 1)(
        input logic clock,
        input logic reset,

        input logic ivalid,
        output logic iready,
        input vector::vector3s_s n,
        input vector::point_t p,
        input vector::single_t d,
        input vector::single_t t,

        output logic ovalid,
        input logic oacknowledge,
        output logic inlier
    );

    import vector::*;

    // logic which will allow variable latency on the FMA unit and 
    // allow for the FMA to (in theory) finish operations out of order
    //
    // we have 3 independent calculations until the calculation of our compare bit:
    // 1. t^2 = t * t + 0
    // 2. n * n = n.x * n.x + n.y * n.y + n.z * n.z + 0
    // 3. n * p - d = n.x * p.x + n.y * p.y * n.z * p.z + -d
    //
    // however, each fma in these calculations depends on the prior calculation
    //
    // i.e., we can start the final calculation in n * p - d before the first one in
    // t^2 finishes, but we have to know the results of all 3 fma's in calculation 3
    // beforehand
    //
    // we can assign each of these fma operations ID's then create a memory which
    // remembers these ID's and logic which minds their dependencies.
    // 
    // t^2 = t * t + 0 is special in that it requires only one fma, so we will say
    // it depends on no other operation
    //
    // calculation 2 is 3 fma's (x, y, and z components) which depend on the prior
    // one (except for the z component, which has no dependency)
    // 
    // calculation 3 has dependencies congruent with calculation 2.
    // from this perspective, we have 8 dependencies possible:
    // 1. None
    // 2. t * t + 0
    // 3. n.z * n.z + 0
    // 4. n.y * n.y + (3)
    // 5. n.x * n.x + (4)
    // 6. n.z * p.z + -d
    // 7. n.y * p.y + (6)
    // 8. n.x * p.x + (7)
    // the second and third levels of calculation, however, require a separate FMA unit
    // and must occur in order (the parallelism was fun while it lasted, I guess)
    // although, once we start the second level calculations, we are ready to start new
    // first level calculations (potentially we can also optimize s.t. the third level
    // calculations are also a different set)
    //

    // the captured input parameters in our first layer calculations
    typedef enum logic [2:0] {
        CAPTURED_INPUT_T,
        CAPTURED_INPUT_D,
        CAPTURED_INPUT_NX,
        CAPTURED_INPUT_NY,
        CAPTURED_INPUT_NZ,
        CAPTURED_INPUT_PX,
        CAPTURED_INPUT_PY,
        CAPTURED_INPUT_PZ
    } captured_input_param_e;

    typedef enum logic [2:0] {
        FIRST_LAYER_NULL_CALCULATION,
        FIRST_LAYER_FMA_T_T_0,
        FIRST_LAYER_N_DOT_N_Z,
        FIRST_LAYER_N_DOT_N_Y,
        FIRST_LAYER_N_DOT_N_X,
        FIRST_LAYER_N_DOT_P_Z,
        FIRST_LAYER_N_DOT_P_Y,
        FIRST_LAYER_N_DOT_P_X
    } first_layer_calculation_e;

    typedef enum logic [1:0] {
        CALCULATION_STATE_PENDING,
        CALCULATION_STATE_RUNNING,
        CALCULATION_STATE_FINISHED
    } calculation_state_e;

    typedef struct packed {
        double_t result;
        calculation_state_e state;
        first_layer_calculation_e depends_on;
        captured_input_param_e a_input;
        captured_input_param_e b_input;
    } first_layer_calculation_s;

    // information regarding the first layer of calculations
    first_layer_calculation_s [7:0] first_layer_calculation;

    single_t [7:0] captured_inputs;

    struct packed {
        single_t a;
        single_t b;
        double_t c;
        double_t r;
        logic iready;
        logic ivalid;
        logic ovalid;
        logic oacknowledge;
        logic [2:0] iid;
        logic [2:0] oid;
    } small_fma;

    quad_t n_dot_p_squared;
    quad_t compare_value;

    struct packed {
        double_t a;
        double_t b;
        quad_t c;
        quad_t r;
        logic iready;
        logic ivalid;
        logic ovalid;
        logic oacknowledge;
        logic iid;
        logic oid;
    } large_fma;

    enum logic [2:0] {
        // idle
        STATE_IDLE,
        // find the next first-layer parameter to submit
        STATE_FIRST_LAYER_FIND_PARAM,
        // set up params to square (n * p - d)
        STATE_PREPARE_N_DOT_P_MINUS_D_SQUARED,
        // submit params to square (n * p - d),
        // prepare params to submit t^2 (n * n) - (n * p - d) ^ 2
        STATE_WAIT_N_DOT_P_MINUS_D_SQUARED,
        // submit t^2 (n * n) - (n * p - d) ^ 2
        STATE_SUBMIT_FINAL_CALCULATION,
        // wait on t^2 (n * n) - (n * p - d) ^ 2
        // once this calculation is done, then go to the idle state.
        STATE_WAIT_ON_FINAL_CALCULATION,
        STATE_SETTLE_INLIER_VALUE
    } state;
    
    // "memory bit" used only within the cycle of checking for when
    // a calculation occurs.
    logic any_first_layer_calculations_remaining;
    // "memory bit" used for the parameter finding loop to know if
    // it should continue looking for a parameter
    logic still_looking_for_first_layer_params;

    always_comb begin : assign_constants
        first_layer_calculation[FIRST_LAYER_NULL_CALCULATION].state = CALCULATION_STATE_FINISHED;
        first_layer_calculation[FIRST_LAYER_NULL_CALCULATION].result = '0;

        first_layer_calculation[FIRST_LAYER_NULL_CALCULATION].depends_on = FIRST_LAYER_NULL_CALCULATION;
        first_layer_calculation[FIRST_LAYER_FMA_T_T_0].depends_on = FIRST_LAYER_NULL_CALCULATION;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_Z].depends_on = FIRST_LAYER_NULL_CALCULATION;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_Y].depends_on = FIRST_LAYER_N_DOT_N_Z;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_X].depends_on = FIRST_LAYER_N_DOT_N_Y;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_Z].depends_on = FIRST_LAYER_NULL_CALCULATION;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_Y].depends_on = FIRST_LAYER_N_DOT_P_Z;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_X].depends_on = FIRST_LAYER_N_DOT_P_Y;

        first_layer_calculation[FIRST_LAYER_NULL_CALCULATION].a_input = CAPTURED_INPUT_T;   // use as a sort-of null value
        first_layer_calculation[FIRST_LAYER_NULL_CALCULATION].b_input = CAPTURED_INPUT_T;   // use as a sort-of null vlaue
        first_layer_calculation[FIRST_LAYER_FMA_T_T_0].a_input = CAPTURED_INPUT_T;
        first_layer_calculation[FIRST_LAYER_FMA_T_T_0].b_input = CAPTURED_INPUT_T;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_Z].a_input = CAPTURED_INPUT_NZ;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_Z].b_input = CAPTURED_INPUT_NZ;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_Y].a_input = CAPTURED_INPUT_NY;        
        first_layer_calculation[FIRST_LAYER_N_DOT_N_Y].b_input = CAPTURED_INPUT_NY;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_X].a_input = CAPTURED_INPUT_NX;
        first_layer_calculation[FIRST_LAYER_N_DOT_N_X].b_input = CAPTURED_INPUT_NX;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_Z].a_input = CAPTURED_INPUT_NZ;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_Z].b_input = CAPTURED_INPUT_PZ;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_Y].a_input = CAPTURED_INPUT_NY;        
        first_layer_calculation[FIRST_LAYER_N_DOT_P_Y].b_input = CAPTURED_INPUT_PY;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_X].a_input = CAPTURED_INPUT_NX;
        first_layer_calculation[FIRST_LAYER_N_DOT_P_X].b_input = CAPTURED_INPUT_PX;
    end : assign_constants

    vector::double_t d_as_double;
    
    always_comb begin
        d_as_double = (captured_inputs[CAPTURED_INPUT_D] <<< (double_fbits - single_fbits));
    end

    always @(posedge clock) begin : main_logic

        // if reset occurs
        if (reset == reset_polarity) begin : handle_reset
            // don't accidentally submit input
            small_fma.ivalid = 0;
            large_fma.ivalid = 0;
            // don't acknowledge the nonexistent input
            small_fma.oacknowledge = 0;
            large_fma.oacknowledge = 0;
            // we haven't calculated anything yet
            for (int unsigned i = 1; i < 8; i++) begin
                first_layer_calculation[i].state = CALCULATION_STATE_PENDING;
                first_layer_calculation[i].result = '0;
            end

            
            // go to the idle state, ready for input, invalid output
            state <= STATE_IDLE;
            iready = 1;
            ovalid = 0;

        end : handle_reset

        else begin : state_machine

            case (state)

            STATE_FIRST_LAYER_FIND_PARAM: begin : find_param

                // 1. check if there is a pending result.
                if (small_fma.ovalid) begin : pending_result_check
                    // whether or not we use the result, clear it from the small fma unit so we can use it on the next cycle
                    small_fma.oacknowledge = 1;

                    // check if the result is actually there or if there was some garbage result
                    // to keep these if statement conditions short, I'm inverting the conditions in this check
                    // the calculation is valid when:
                    // 1. its id is not the null calculation
                    // 2. its id corresponds to a calculation which is currently running
                    // 3. its dependency has finished (which should always be the case when a calculation is running).
                    if (small_fma.oid == FIRST_LAYER_NULL_CALCULATION) begin : invalid_because_we_dont_do_the_null_calculation
                    end : invalid_because_we_dont_do_the_null_calculation
                    else if (first_layer_calculation[small_fma.oid].state != CALCULATION_STATE_RUNNING) begin : invalid_because_we_werent_expecting_this_id
                    end : invalid_because_we_werent_expecting_this_id
                    else if (first_layer_calculation[first_layer_calculation[small_fma.oid].depends_on].state != CALCULATION_STATE_FINISHED) begin : invalid_because_this_calculation_cannot_start
                    end : invalid_because_this_calculation_cannot_start
                    else begin : valid_result_found
                        first_layer_calculation[small_fma.oid].result = small_fma.r;
                        first_layer_calculation[small_fma.oid].state = CALCULATION_STATE_FINISHED;
                    end : valid_result_found
                end : pending_result_check

                // 2. determine which calculation we can do, choose the first available one we find.
                
                // if there won't be any more parameters, be sure not to re-submit any
                small_fma.ivalid = 0;
                still_looking_for_first_layer_params = 1;
                for (int unsigned i = 1; i < 8; i++) begin : find_first_layer_params_loop
                    // oops! actually can't calculate!
                    if (!small_fma.iready) begin
                        still_looking_for_first_layer_params = 0;
                    end
                    // only start a calculation which hasn't been submitted yet
                    if (still_looking_for_first_layer_params) begin
                        if (first_layer_calculation[i].state != CALCULATION_STATE_PENDING) begin
                        end
                        else if (first_layer_calculation[first_layer_calculation[i].depends_on].state != CALCULATION_STATE_FINISHED) begin
                        end
                        else begin
                            // found an available calculation, set up the inputs to it.
                            small_fma.ivalid = 1; // will be visible externally on next cycle
                            small_fma.a = captured_inputs[first_layer_calculation[i].a_input];
                            small_fma.b = captured_inputs[first_layer_calculation[i].b_input];
                            small_fma.iid = i;
                            first_layer_calculation[i].state = CALCULATION_STATE_RUNNING;
                            
                            // calculations which depend on the null calculation always use 0 as their c-input,
                            // everything else uses the result of the required calculation as the c-input.
                            if (first_layer_calculation[i].depends_on == FIRST_LAYER_NULL_CALCULATION) begin
                                // special case for first component of finding n * p - d
                                small_fma.c = (i == FIRST_LAYER_N_DOT_P_Z) ? -d_as_double : '0;
                            end else begin
                                small_fma.c = first_layer_calculation[first_layer_calculation[i].depends_on].result;
                            end
                            still_looking_for_first_layer_params = 0;
                        end
                    end
                end : find_first_layer_params_loop
                
                // check if any first-layer calculations need to be submitted or waited on
                any_first_layer_calculations_remaining = 0;
                for(int unsigned i = 0; i < 8; i++) begin
                    if (first_layer_calculation[i].state != CALCULATION_STATE_FINISHED) begin
                        any_first_layer_calculations_remaining = 1;
                        break;
                    end
                end
                // go to the next state if all calculations are finished
                if (!any_first_layer_calculations_remaining) begin
                    state <= STATE_PREPARE_N_DOT_P_MINUS_D_SQUARED;
                end

            end : find_param

            STATE_PREPARE_N_DOT_P_MINUS_D_SQUARED: begin
                // remember we calculate the x-component last, so the result of that
                // calculation contains the dot product.
                large_fma.a = first_layer_calculation[FIRST_LAYER_N_DOT_P_X].result;
                large_fma.b = first_layer_calculation[FIRST_LAYER_N_DOT_P_X].result;
                large_fma.c = '0;
                large_fma.ivalid = 1;
                large_fma.oacknowledge = 1;
                state <= STATE_WAIT_N_DOT_P_MINUS_D_SQUARED;
            end

            STATE_WAIT_N_DOT_P_MINUS_D_SQUARED: begin
                large_fma.ivalid = 0; // don't submit more calculations
                n_dot_p_squared = large_fma.r;
                if (large_fma.ovalid) begin
                    state <= STATE_SUBMIT_FINAL_CALCULATION;
                end
            end

            STATE_SUBMIT_FINAL_CALCULATION: begin
                large_fma.a = first_layer_calculation[FIRST_LAYER_FMA_T_T_0].result;
                // remember that the x-component is calculated last, so this is the
                // squared magnitude of n
                large_fma.b = first_layer_calculation[FIRST_LAYER_N_DOT_N_X].result;
                large_fma.c = -n_dot_p_squared;
                large_fma.ivalid = 1;
                large_fma.oacknowledge = 1;
                state <= STATE_WAIT_ON_FINAL_CALCULATION;
            end

            STATE_WAIT_ON_FINAL_CALCULATION: begin 
                large_fma.ivalid = 0; // don't submit more calculations
                compare_value = large_fma.r;
                inlier = ($signed(compare_value) >= $signed('0));
                if (large_fma.ovalid) begin
                    state <= STATE_SETTLE_INLIER_VALUE;
                end
            end
            
            STATE_SETTLE_INLIER_VALUE: begin
                compare_value = large_fma.r;
                inlier = ($signed(compare_value) >= $signed('0));
                state <= STATE_IDLE;
                ovalid = 1;
            end

            STATE_IDLE: begin
                if (ovalid && oacknowledge && !iready) begin
                    iready = 1;
                end else if (ivalid && iready) begin
                    ovalid = 0;
                    iready = 0;
                    state <= STATE_FIRST_LAYER_FIND_PARAM;
                end
                // always capture inputs on every cycle spent idle
                captured_inputs[CAPTURED_INPUT_T] = t;
                captured_inputs[CAPTURED_INPUT_D] = d;
                captured_inputs[CAPTURED_INPUT_NX] = n.v.x;
                captured_inputs[CAPTURED_INPUT_NY] = n.v.y;
                captured_inputs[CAPTURED_INPUT_NZ] = n.v.z;
                captured_inputs[CAPTURED_INPUT_PX] = p.v.x;
                captured_inputs[CAPTURED_INPUT_PY] = p.v.y;
                captured_inputs[CAPTURED_INPUT_PZ] = p.v.z;

                small_fma.ivalid = 0;
                large_fma.ivalid = 0;
                
                // we haven't calculated anything yet
                for (int unsigned i = 1; i < 8; i++) begin
                    first_layer_calculation[i].state = CALCULATION_STATE_PENDING;
                end

            end

            endcase

        end : state_machine 

    end : main_logic

    fp_fma#(
        .reset_polarity(reset_polarity),
        .ibits(single_ibits),
        .fbits(single_fbits),
        .latency(latency_small_fma),
        .id_bits($bits(small_fma.iid))
    ) small_fma_unit (
        .clock(clock),
        .reset(reset),
        .a(small_fma.a),
        .b(small_fma.b),
        .c(small_fma.c),
        .r(small_fma.r),
        .iid(small_fma.iid),
        .oid(small_fma.oid),
        .iready(small_fma.iready),
        .ivalid(small_fma.ivalid),
        .ovalid(small_fma.ovalid),
        .oacknowledge(small_fma.oacknowledge)
    );

    fp_fma#(
        .reset_polarity(reset_polarity),
        .ibits(double_ibits),
        .fbits(double_fbits),
        .latency(latency_large_fma),
        .id_bits($bits(large_fma.iid))
    ) large_fma_unit(
        .clock(clock),
        .reset(reset),
        .a(large_fma.a),
        .b(large_fma.b),
        .c(large_fma.c),
        .r(large_fma.r),
        .iid(large_fma.iid),
        .oid(large_fma.oid),
        .iready(large_fma.iready),
        .ivalid(large_fma.ivalid),
        .ovalid(large_fma.ovalid),
        .oacknowledge(large_fma.oacknowledge)
    );

endmodule : check_inlier
