`timescale 1ns/1ps

`include "vectors/vector_pkg.svh"

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
        parameter int unsigned latency_fma = vector::fma_latency_singles,
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

    import vector::*;

    // steps for deriving the plane:
    // 1. find v1 and v2 (use adder / subtracter which may or may not have a latency)
    // 2. find n = v1 x v2 (use small fma unit)
    // 3. find d = n * a (use large fma unit)
    // 4. round n and d to use single_t for each component (use fp_truncate)
    //
    // Note: unlike check_inlier, derive_plane can actually fail if the three 
    // points supposedly in the plane give a normal equal to the zero vector.
    //
    // Note: derive_plane will need to introduce a scale factor (possibly multiple
    // scale factors), s, to prevent saturation and overflow. 
    //
    // fp_fma is guaranteed to never encounter saturation / overflow on its output,
    // but that output may indeed saturate when converted back to a single_t.
    //
    // momentarily assuming no integer overflow, the N-bit product of 2 x N-bit
    // integers can overflow if either multiplicand is greater than or equal to
    // 2^(N/2).
    //
    // For a fixed point number with N integer bits and F fraction bits, this 
    // logic changes slightly. The raw result of the multiplication is 2(N+F)
    // bits wide, however, we round away the least significant F bits, meaning
    // the product saturates / overflows if its integer component exceeds N bits
    // in size. As such, multiplying any two single_t instances can overflow
    // if either single_t exceeds 2 ^ (single_ibits / 2). For an integer
    // representation, this is 2 ^ (single_ibits / 2 + single_fbits).
    //
    // Applying back in the potential overflow due to addition, we can simply 
    // add one bit of 'safety factor' for these operations. For any finite 
    // representation of integers x and y, if their sum z exceeds the limits of
    // this finite representaiton, then x/2 + y/2 = z/2 can be represented. At
    // least if x, y, and z are binary integers.

    // cross product, as from Wikipedia:
    // a cross b ->
    // x = a.y * b.z - a.z * b.y
    // y = a.z * b.x - a.x * b.z
    // z = a.x * b.y - a.y * b.x
    typedef enum logic [4:0] {
        REG_ZERO,   // constant 0
        REG_AX,     // a.x
        REG_AY,     // a.y
        REG_AZ,     // a.z
        REG_V1X,    // a.x - b.x    (calculated 'inline')
        REG_V1Y,    // a.y - b.y    (calculated 'inline')
        REG_V1Z,    // a.z - b.z    (calculated 'inline')
        REG_V2X,    // a.x - c.x    (calculated 'inline')
        REG_V2Y,    // a.y - c.y    (calculated 'inline')
        REG_V2Z,    // a.z - c.z    (calculated 'inline')
        REG_NX1,    // v1z * v2y - zero
        REG_NX2,    // v1y * v2z - nx1
        REG_NY1,    // v1x * v2z - zero
        REG_NY2,    // v1z * v2x - ny1
        REG_NZ1,    // v1y * v2x - zero
        REG_NZ2,    // v1x * v2y - nz1
        REG_D1,     // a.x * nx2 + zero
        REG_D2,     // a.y * ny2 + d1
        REG_D3      // a.z * nz2 = d2
    } register_e;

    typedef struct packed {
        register_e dst; // destination register
        register_e rs1; // lhs of multiplicand
        register_e rs2; // rhs of multiplicand
        register_e rs3; // addend
        logic negate_addend;    // if set, negate addend before adding.
    } fma_op_s;

    typedef struct packed {
        single_t value;
        logic valid;
    } reg_entry_s;

    reg_entry_s [0:18] regs;

    fma_op_s [0:8] fma_sequence;
    always_comb begin : assign_sequence
    // as with check_inlier, this sequence is pre-reordered to improve
    // throughput. This ordering might not be optimal but it does overlap some
    // calculations.
        fma_sequence[0] = '{REG_NX1, REG_V1Z, REG_V2Y, REG_ZERO, 1};
        fma_sequence[1] = '{REG_NY1, REG_V1X, REG_V2Z, REG_ZERO, 1};
        fma_sequence[2] = '{REG_NX2, REG_V1Y, REG_V2Z, REG_NX1, 1};
        fma_sequence[3] = '{REG_NZ1, REG_V1Y, REG_V2X, REG_ZERO, 1};
        fma_sequence[4] = '{REG_NY2, REG_V1Z, REG_V2X, REG_NY1, 1};
        fma_sequence[5] = '{REG_D1, REG_AX, REG_NX2, REG_ZERO, 0};
        fma_sequence[6] = '{REG_NZ2, REG_V1X, REG_V2Y, REG_NZ1, 1};
        fma_sequence[7] = '{REG_D2, REG_AY, REG_NY2, REG_D1, 0};
        fma_sequence[8] = '{REG_D3, REG_AZ, REG_NZ2, REG_D2, 0};

    end : assign_sequence

    logic [3:0] fma_sequence_position;

    struct {
        single_t a;
        single_t b;
        double_t c;
        logic [bits_in_double+1-1:0] r;
        register_e iid;
        register_e oid;
        logic ivalid;
        logic iready;
        logic ovalid;
        logic oacknowledge;
    } fma_port;

    assign fma_port.oacknowledge = 1;

    struct {
        logic [bits_in_double+1-1:0] ivalue;
        single_t ovalue;
        logic saturated;
    } rounding_port;

    assign rounding_port.ivalue = fma_port.r;

    enum {
        IDLE,
        CALC
    } unit_state;

    always_ff @(posedge clock) begin
        if (reset == reset_polarity) begin
            for (int unsigned i = 0; i < 19; i++) begin
                regs[i].value <= '0;
                regs[i].valid <= '0;
            end
            unit_state <= IDLE;
            iready <= 1;
            ovalid <= 0;
            for (int unsigned i = 0; i < 3; i++) begin
                n.c[i] <= '0;
            end
            d <= '0;
            status <= DERIVE_PLANE_STATUS_SUCCESS;
            fma_sequence_position <= '0;
            fma_port.a <= '0;
            fma_port.b <= '0;
            fma_port.c <= '0;
            fma_port.ivalid <= '0;
            fma_port.iid <= REG_ZERO;
        end else begin
            case (unit_state)
            IDLE: begin : proc_idle
                if (ovalid && !oacknowledge) begin
                    iready <= 0;
                end else if (ovalid && oacknowledge) begin
                    iready <= 1;
                    ovalid <= 0;
                    for (int unsigned i = 0; i < 3; i++) begin
                        n.c[i] <= '0;
                    end
                    d <= '0;
                end else if (!ovalid) begin
                    if (!iready) begin
                        iready <= 1;
                    end
                    ovalid <= 0;
                end

                fma_sequence_position <= 0;

                fma_port.a <= '0;
                fma_port.b <= '0;
                fma_port.c <= '0;
                fma_port.ivalid <= '0;
                fma_port.iid <= REG_ZERO;

                regs[REG_ZERO].value <= '0;
                regs[REG_ZERO].valid <= 1;

                regs[REG_AX].value <= a.v.x;
                regs[REG_AX].valid <= 1;

                regs[REG_AY].value <= a.v.y;
                regs[REG_AY].valid <= 1;

                regs[REG_AZ].value <= a.v.z;
                regs[REG_AZ].valid <= 1;

                regs[REG_V1X].value <= a.v.x - b.v.x;
                regs[REG_V1X].valid <= 1;

                regs[REG_V1Y].value <= a.v.y - b.v.y;
                regs[REG_V1Y].valid <= 1;

                regs[REG_V1Z].value <= a.v.z - b.v.z;
                regs[REG_V1Z].valid <= 1;

                regs[REG_V2X].value <= a.v.x - c.v.x;
                regs[REG_V2X].valid <= 1;

                regs[REG_V2Y].value <= a.v.y - c.v.y;
                regs[REG_V2Y].valid <= 1;

                regs[REG_V2Z].value <= a.v.z - c.v.z;
                regs[REG_V2Z].valid <= 1;

                for (int unsigned i = REG_NX1; i <= REG_D3; i++) begin
                    regs[i].value <= '0;
                    regs[i].valid <= 0;
                end

                if (ivalid && iready) begin
                    iready <= 0;
                    ovalid <= 0;
                    unit_state <= CALC;
                end
            end : proc_idle
            CALC: begin : proc_calc

                if (fma_sequence_position <= 8) begin
                    fma_port.a <= regs[fma_sequence[fma_sequence_position].rs1].value;
                    fma_port.b <= regs[fma_sequence[fma_sequence_position].rs2].value;
                    if (fma_sequence[fma_sequence_position].negate_addend) begin
                        fma_port.c <= -($signed(regs[fma_sequence[fma_sequence_position].rs3].value) <<< (double_fbits - single_fbits));
                    end else begin
                        fma_port.c <= $signed(regs[fma_sequence[fma_sequence_position].rs3].value) <<< (double_fbits - single_fbits);
                    end
                    fma_port.iid <= fma_sequence[fma_sequence_position].dst;

                    if (regs[fma_sequence[fma_sequence_position].rs1].valid && regs[fma_sequence[fma_sequence_position].rs2].valid && regs[fma_sequence[fma_sequence_position].rs3].valid) begin
                        fma_port.ivalid <= 1;
                        if (fma_port.iready) begin
                            fma_sequence_position <= fma_sequence_position + 1;
                        end
                    end else begin
                        fma_port.ivalid <= 0;
                    end
                end else begin
                    fma_port.ivalid <= 0;
                end

                if (fma_port.ovalid) begin
                    regs[fma_port.oid].value <= rounding_port.ovalue;
                    regs[fma_port.oid].valid <= 1;
                end

                // if we just finished calculating all of N and D, report the
                // results.
                if (regs[REG_D3].valid && fma_sequence_position > 8) begin
                    n.v.x <= regs[REG_NX2].value;
                    n.v.y <= regs[REG_NY2].value;
                    n.v.z <= regs[REG_NZ2].value;
                    d <= regs[REG_D3].value;
                    if (regs[REG_NX2].value == 0 && regs[REG_NY2].value == 0 && regs[REG_NZ2].value == 0) begin
                        status <= DERIVE_PLANE_STATUS_LESS_THAN_THREE_UNIQUE_POINTS;
                    end else begin
                        status <= DERIVE_PLANE_STATUS_SUCCESS;
                    end
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

endmodule : derive_plane