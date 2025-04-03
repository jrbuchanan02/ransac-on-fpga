
`ifndef RANSAC_FIXED_POINT_SVH
`define RANSAC_FIXED_POINT_SVH

package ransac_fixed;
    
    localparam int unsigned integral_bits = 24;
    localparam int unsigned fraction_bits = 24;

    function int value_bits(); 
    begin 
        value_bits = integral_bits + fraction_bits; 
    end 
    endfunction : value_bits

    typedef logic signed [value_bits()-1:0] fixed_t;
    // fixed point: our values as integers are 2^fraction_bits larger than we expect,
    // so when we multiply that difference doubles, so to fix it, we divide by 2^fraciton_bits
    // 
    // so for division, we end multiply by 2^fraction_bits to adjust
    typedef logic signed [2*value_bits()-1:0] product_t;
    typedef logic signed [2*value_bits()-1:0] quotient_t;
    
    function fixed_t one();
    begin
        one = 1 << fraction_bits;
    end
    endfunction : one


    // rough guess as to a number near 1 / sqrt(a)
    // only valid for a > 0.
    function fixed_t rsqrt_initial_guess(input fixed_t a);
        int unsigned lzc;
        int signed i;
        fixed_t log2_est;
        fixed_t log2_rsqrt_est;
        fixed_t rsqrt_est;
    begin
        // log base 2 of a number kind of follows 
        // the leading zero count
        for (i = value_bits() - 1; i > -1; i--) begin
            if (a[i] == 0) begin
                lzc++;
            end else begin
                break;
            end
        end

        log2_est = $signed(lzc) - integral_bits;
        log2_rsqrt_est = -log2_est >>> 1;

        // go backwards to exponentiate the estimate

        rsqrt_est = one();

        if (log2_rsqrt_est < 0) begin
            rsqrt_initial_guess = rsqrt_est >>> -log2_rsqrt_est;
        end else begin
            rsqrt_initial_guess = rsqrt_est << log2_rsqrt_est;
        end
        rsqrt_initial_guess = rsqrt_est;
    end
    endfunction : rsqrt_initial_guess

    typedef struct packed {
        fixed_t x;
        fixed_t y;
        fixed_t z;
    } vector3f_t;

    typedef vector3f_t point_t;

    // essentially a plane in cartesian form for 3D space
    typedef struct packed {
        vector3f_t normal;
        fixed_t d;
    } plane_t;

    typedef enum logic [1:0] {
        FMA_OPCODE_POS_A_POS_C, // R =  A * B + C
        FMA_OPCODE_POS_A_NEG_C, // R =  A * B - C
        FMA_OPCODE_NEG_A_POS_C, // R = -A * B + C
        FMA_OPCODE_NEG_A_NEG_C  // R = -A * B - C
    } fma_opcode_t;

endpackage : ransac_fixed

`endif // RANSAC_FIXED_POINT_SVH