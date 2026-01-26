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
        parameter int unsigned latency_fma = vector::fma_latency_singles,
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

    // variables for the calculation
    //
    // steps from method.docx, allow FMA to check ahead to reorder calculations:
    //
    // 1. v_1=d*d+0
	// 2. v_2=n.x*p.x+0
	// 3. v_3=n.y*p.y+v_2
	// 4. v_4=n.z*p.z+v_3
	// 5. v_5=(2*d)*v_4-v_1
	// 6. v_6=-v_4*v_4+v_5
	// 7. v_7=n.x*n.x+0
	// 8. v_8=n.y*n.y+v_7
	// 9. v_9=n.z*n.z+v_8
	// 10. v_10=t*t+0
	// 11. v_11=v_10*v_9+v_6

    // one of the values calculated or used as an input in this sequence.
    typedef enum logic [4:0] {
        REG_ZERO,   // the constant 0
        REG_NX,     // x component of n
        REG_NY,     // y component of n
        REG_NZ,     // z component of n
        REG_PX,     // x component of p
        REG_PY,     // y component of p
        REG_PZ,     // z component of p
        REG_D,      // scalar d in plane equation
        REG_T,      // threshold
        REG_V1,     // result from step 1
        REG_V2,     // result from step 2
        REG_V3,     // result from step 3
        REG_V4,     // result from step 4
        REG_V5,     // result from step 5
        REG_V6,     // result from step 6
        REG_V7,     // result from step 7
        REG_V8,     // result from step 8
        REG_V9,     // result from step 9
        REG_V10,    // result from step 10
        REG_V11     // result from step 11. Inlier if this value is valid and not negative
    } register_e;

    // operation for an FMA in the inlier check
    typedef struct packed {
        register_e dst; // destination register
        register_e rs1; // lhs of multiplicand
        register_e rs2; // rhs of multiplicand
        register_e rs3; // addend
        logic rs1_shift;    // if set, shift rs1 left once before multiply
        logic rs1_negate;   // if set, negate rs1 before multiply
        logic rs3_negate;   // if set, negate rs3 before multiply
    } fma_op_s;

    // a register for the calculations
    typedef struct packed {
        single_t value; // the value held in the register
        logic valid;    // set if the value is ready for use by a later calculation
    } reg_entry_s;

    reg_entry_s [0:19] regs;

    // FMA sequence. These instructions are pre-reordered to hopefully get a 
    // good throughput. FMAs are issued if all three inputs are marked valid
    // and FMA operations are not reordered.
    // localparam fma_op_s [0:10] fma_sequence = '{
    //     '{ REG_V2, REG_NX, REG_PX, REG_ZERO, 0, 0, 0 }, // v2 = nx * px + 0
    //     '{ REG_V1, REG_D, REG_D, REG_ZERO, 0, 0, 0 },   // v1 = d * d + 0
    //     '{ REG_V3, REG_NY, REG_PY, REG_V2, 0, 0, 0 },   // v3 = ny * py + v2
    //     '{ REG_V7, REG_NX, REG_NX, REG_ZERO, 0, 0, 0 }, // v7 = nx * nx + 0
    //     '{ REG_V4, REG_NZ, REG_PZ, REG_V3, 0, 0, 0 },   // v4 = nz * pz + v3 <-- n dot p finished here
    //     '{ REG_V10, REG_T, REG_T, REG_ZERO, 0, 0, 0 },  // v10 = t * t + 0
    //     '{ REG_V5, REG_D, REG_V4, REG_V1, 1, 0, 1 },    // v5 = 2d * v4 - v1 <-- 2d(n dot p) - d^2
    //     '{ REG_V8, REG_NY, REG_NY, REG_V7, 0, 0, 0 },   // v8 = ny * ny + v7
    //     '{ REG_V6, REG_V4, REG_V4, REG_V5, 0, 1, 0 },   // v6 = -v4 * v4 + v5 <-- -(n dot p)^2 + 2d(n dot p) - d^2
    //     '{ REG_V9, REG_NZ, REG_NZ, REG_V8, 0, 0, 0 },   // v9 = nz * nz + v8 <-- n dot n finished here
    //     '{ REG_V11, REG_V10, REG_V9, REG_V6, 0, 0, 1 }  // v11 = v10 * v9 - v6 <-- t^2 * (n dot n) - (n dot p)^2 + 2d(n dot p) - d^2
    // };

    fma_op_s [0:10] fma_sequence;
    always_comb begin
        fma_sequence[0] = '{ REG_V2, REG_NX, REG_PX, REG_ZERO, 0, 0, 0 }; // v2 = nx * px + 0
        fma_sequence[1] = '{ REG_V1, REG_D, REG_D, REG_ZERO, 0, 0, 0 };   // v1 = d * d + 0
        fma_sequence[2] = '{ REG_V3, REG_NY, REG_PY, REG_V2, 0, 0, 0 };   // v3 = ny * py + v2
        fma_sequence[3] = '{ REG_V7, REG_NX, REG_NX, REG_ZERO, 0, 0, 0 }; // v7 = nx * nx + 0
        fma_sequence[4] = '{ REG_V4, REG_NZ, REG_PZ, REG_V3, 0, 0, 0 };   // v4 = nz * pz + v3 <-- n dot p finished here
        fma_sequence[5] = '{ REG_V10, REG_T, REG_T, REG_ZERO, 0, 0, 0 };  // v10 = t * t + 0
        fma_sequence[6] = '{ REG_V5, REG_D, REG_V4, REG_V1, 1, 0, 1 };    // v5 = 2d * v4 - v1 <-- 2d(n dot p) - d^2
        fma_sequence[7] = '{ REG_V8, REG_NY, REG_NY, REG_V7, 0, 0, 0 };   // v8 = ny * ny + v7
        fma_sequence[8] = '{ REG_V6, REG_V4, REG_V4, REG_V5, 0, 1, 0 };   // v6 = -v4 * v4 + v5 <-- -(n dot p)^2 + 2d(n dot p) - d^2
        fma_sequence[9] = '{ REG_V9, REG_NZ, REG_NZ, REG_V8, 0, 0, 0 };   // v9 = nz * nz + v8 <-- n dot n finished here
        fma_sequence[10] = '{ REG_V11, REG_V10, REG_V9, REG_V6, 0, 0, 0 };  // v11 = v10 * v9 - v6 <-- t^2 * (n dot n) - (n dot p)^2 + 2d(n dot p) - d^2
    end

    // where we are in the FMA sequence
    logic [0:3] fma_sequence_position;

    // port to the fma unit.
    struct {
        single_t a; // rs1
        single_t b; // rs2
        double_t c; // rs3 as double_t
        logic [bits_in_double+1-1:0] r; // res as double_t + one more integer bit.
        register_e iid; // dst reg of submitted calculation
        register_e oid; // dst reg of most recently finished calculation
        logic ivalid;   // whether we attempt to submit a calculation this cycle
        logic iready;   // whether a calculation can be submitted this cycle
        logic ovalid;   // whether a calculation has finished this cycle
        logic oacknowledge; // whether we accept a calculation this cycle.
    } fma_port;

    // we always acknowledge the FMA outputs on the cycle that
    // they are ready
    assign fma_port.oacknowledge = 1;

    // port to the fixed point truncation module.
    // we truncate the result of the fma operation
    struct {
        logic [bits_in_double+1-1:0] ivalue;
        single_t ovalue;
    } rounding_port;

    // immediately round the fma result to a single_t.
    assign rounding_port.ivalue = fma_port.r;

    enum {
        IDLE,   // awaiting valid input and possibly broadcasting a previous valid output. Also capturing inputs on each cycle.
        CALC   // running calculations
    } unit_state;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < 20; i++) begin
                regs[i].value <= '0;
                regs[i].valid <= '0;
            end
            unit_state <= IDLE;
            iready <= 1;
            ovalid <= 0;
            inlier <= 0;
            fma_sequence_position <= 0;
            fma_port.a <= '0;
            fma_port.b <= '0;
            fma_port.c <= '0;
            fma_port.ivalid <= '0;
            fma_port.iid <= REG_ZERO;
        end else begin
            case (unit_state)
            IDLE: begin : proc_idle
                // handle iready.
                // we are not ready until output is acknowledged.
                if (ovalid && !oacknowledge) begin
                    iready <= 0;
                end else if (ovalid && oacknowledge) begin
                    iready <= 1;
                    ovalid <= 0;    // output has been read
                    inlier <= 0;    // somewhere else, the code seems to expect that
                                    // inlier is only set when there is an inlier?
                end else if (!ovalid) begin
                    // if statement since we can actually start a calculation
                    // on this cycle and iready should only be driven once.
                    if (!iready) begin
                        iready <= 1;
                    end
                    ovalid <= 0;
                end

                fma_port.a <= '0;
                fma_port.b <= '0;
                fma_port.c <= '0;
                fma_port.ivalid <= '0;
                fma_port.iid <= REG_ZERO;

                // read in special input registers, reset the others.
                regs[REG_ZERO].value <= '0;
                regs[REG_ZERO].valid <= 1;

                regs[REG_NX].value <= n.v.x;
                regs[REG_NX].valid <= 1;

                regs[REG_NY].value <= n.v.y;
                regs[REG_NY].valid <= 1;

                regs[REG_NZ].value <= n.v.z;
                regs[REG_NZ].valid <= 1;

                regs[REG_D].value <= d;
                regs[REG_D].valid <= 1;

                regs[REG_T].value <= t;
                regs[REG_T].valid <= 1;

                regs[REG_PX].value <= p.v.x;
                regs[REG_PX].valid <= 1;

                regs[REG_PY].value <= p.v.y;
                regs[REG_PY].valid <= 1;

                regs[REG_PZ].value <= p.v.z;
                regs[REG_PZ].valid <= 1;

                fma_sequence_position <= 0;

                // mark all other regs as invalid.
                for (int unsigned i = REG_V1; i <= REG_V11; i++) begin
                    regs[i].value <= '0;
                    regs[i].valid <= 0;
                end

                if (ivalid && iready) begin
                    iready <= 0;
                    ovalid <= 0; // ovalid should already be 0 at this point, however.
                    unit_state <= CALC;
                end
            end : proc_idle
            CALC: begin : proc_calc

                if (fma_sequence_position < 10 || fma_sequence_position == 10) begin
                    // set up the next calculation in the sequence.

                    if (fma_sequence[fma_sequence_position].rs1_negate) begin
                        if (fma_sequence[fma_sequence_position].rs1_shift) begin
                            fma_port.a <= -regs[fma_sequence[fma_sequence_position].rs1].value << 1;
                        end else begin
                            fma_port.a <= -regs[fma_sequence[fma_sequence_position].rs1].value;
                        end
                    end else begin
                        if (fma_sequence[fma_sequence_position].rs1_shift) begin
                            fma_port.a <= regs[fma_sequence[fma_sequence_position].rs1].value << 1;
                        end else begin
                            fma_port.a <= regs[fma_sequence[fma_sequence_position].rs1].value;
                        end
                    end

                    fma_port.b <= regs[fma_sequence[fma_sequence_position].rs2].value;
                    //                           |-------------- single_ibits + double_fbits - 1 -> bit index 
                    //                           |    |-------- double_fbits - 1 -> bit index
                    //                           |    |    |--- double_fbits - single_fbits - 1 -> bit index
                    // double             -> 0xIIIIII'FFFFFFFFFF
                    // single into double -> 0xsssIII'FFFFF00000
                    // s -> sign bit
                    // I -> integer bit
                    // F -> fraction bit
                    // 0 -> zero bit
                    if (fma_sequence[fma_sequence_position].rs3_negate) begin
                        fma_port.c <= -($signed(regs[fma_sequence[fma_sequence_position].rs3].value) <<< (double_fbits - single_fbits));
                    end else begin
                        fma_port.c <= $signed(regs[fma_sequence[fma_sequence_position].rs3].value) <<< (double_fbits - single_fbits);
                    end

                    fma_port.iid <= fma_sequence[fma_sequence_position].dst;

                    // are all inputs valid on this cycle? if so, set ivalid to
                    // 1. If iready is also high on this cycle, add 1 to the sequence
                    // position
                    //
                    // if any invalid input, set ivalid low.
                    if (regs[fma_sequence[fma_sequence_position].rs1].valid && regs[fma_sequence[fma_sequence_position].rs2].valid && regs[fma_sequence[fma_sequence_position].rs3].valid) begin
                        // all valid.
                        fma_port.ivalid <= 1;

                        if (fma_port.iready) begin
                            fma_sequence_position <= fma_sequence_position + 1;
                        end
                    end else begin // any input is invalid.
                        fma_port.ivalid <= 0;
                    end
                end else begin
                    fma_port.ivalid <= 0;   // ensure that no calculations start
                                            // on accident.
                end
                
                // regardless of whether a calculation starts this cycle,
                // accept any input that has just finished.
                if (fma_port.ovalid) begin
                    // mod 20 (which shouldn't be necessary)
                    // to ensure that reg index is never out of bounds.
                    regs[fma_port.oid].value <= rounding_port.ovalue;
                    regs[fma_port.oid].valid <= 1;
                end

                // if we just finished the last calculation, report the results.
                if (regs[REG_V11].valid && fma_sequence_position == 11) begin
                    inlier <= $signed(regs[REG_V11].value) >= $signed(0);
                    ovalid <= 1;
                    unit_state <= IDLE;
                end

            end : proc_calc
            endcase
        end
    end

    fp_fma#(
        .reset_polarity(reset_polarity),
        .ibits(single_ibits),
        .fbits(single_fbits),
        .id_bits(5),
        .latency(latency_fma),
        .add_latency(0)
    ) fma_unit(
        .clock(clock),
        .reset(reset),
        .a(fma_port.a),
        .b(fma_port.b),
        .c(fma_port.c),
        .r(fma_port.r),
        .iid(fma_port.iid),
        .oid(fma_port.oid),
        .ivalid(fma_port.ivalid),
        .iready(fma_port.iready),
        .ovalid(fma_port.ovalid),
        .oacknowledge(fma_port.oacknowledge)
    );

    fp_truncate#(
        .iibits(double_ibits + 1),
        .ifbits(double_fbits),
        .oibits(single_ibits),
        .ofbits(single_fbits),
        .signed_values(1)
    ) truncate_unit(
        .ivalue(rounding_port.ivalue),
        .ovalue(rounding_port.ovalue)
    );

endmodule : check_inlier
