
`ifndef RANSAC_FIXED_POINT_SVH
`define RANSAC_FIXED_POINT_SVH

package ransac_fixed;
    
    localparam int unsigned integral_bits = 8;
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