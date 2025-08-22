`ifndef VECTORS_VECTOR_PKG_SVH
`define VECTORS_VECTOR_PKG_SVH

package vector;

    localparam int unsigned single_ibits = 12;
    localparam int unsigned single_fbits = 20;

    localparam int unsigned double_ibits = 2 * single_ibits;
    localparam int unsigned double_fbits = 2 * single_fbits;

    localparam int unsigned quad_ibits = 2 * double_ibits;
    localparam int unsigned quad_fbits = 2 * double_fbits;

    localparam int unsigned bits_in_single = single_ibits + single_fbits;
    localparam int unsigned bits_in_double = double_ibits + double_fbits;
    localparam int unsigned bits_in_quad = quad_ibits + quad_fbits;

    localparam int unsigned fma_latency_singles = 4;
    localparam int unsigned fma_latency_doubles = 16;
    localparam int unsigned fma_latency_quads = 7;

    typedef logic signed [single_ibits+single_fbits-1:0] single_t;
    typedef logic signed [double_ibits+double_fbits-1:0] double_t;
    typedef logic signed [quad_ibits+quad_fbits-1:0] quad_t;

    typedef union packed {
        single_t [2:0] c;
        struct packed {
            single_t x;
            single_t y;
            single_t z;
        } v;
    } vector3s_s;

    typedef union packed {
        double_t [2:0] c;
        struct packed {
            double_t x;
            double_t y;
            double_t z;
        } v;
    } vector3d_s;

    typedef vector3s_s point3s_s;

    localparam type point_t = point3s_s;

    typedef enum logic [0:0] {
        DERIVE_PLANE_STATUS_SUCCESS,
        DERIVE_PLANE_STATUS_LESS_THAN_THREE_UNIQUE_POINTS
    } derive_plane_status_e;

    // basically fp_truncate but as a function
    function single_t double_to_single(input double_t double);
        logic signed [double_ibits+single_fbits-1:0] rounded_down;
        single_t truncated;
    begin
        rounded_down = $signed(double) >>> (double_fbits - single_fbits);
        truncated = rounded_down[bits_in_single-1:0];
        // check if saturation is necessary
        if (truncated != rounded_down) begin
            if (double < 0) begin
                double_to_single = {1'b1, {bits_in_single-1{1'b0}}};
            end else begin
                double_to_single = {1'b0, {bits_in_single-1{1'b1}}};
            end
        end else begin
            double_to_single = truncated;
        end
    end
    endfunction : double_to_single

    // basically fp_truncate but as a function
    function double_t quad_to_double(input quad_t double);
        logic signed [quad_ibits+double_fbits-1:0] rounded_down;
        double_t truncated;
    begin
        rounded_down = $signed(double) >>> (quad_fbits - double_fbits);
        truncated = rounded_down[bits_in_double-1:0];
        // check if saturation is necessary
        if (truncated != rounded_down) begin
            if (double < 0) begin
                quad_to_double = {1'b1, {bits_in_double-1{1'b0}}};
            end else begin
                quad_to_double = {1'b0, {bits_in_double-1{1'b1}}};
            end
        end else begin
            quad_to_double = truncated;
        end
    end
    endfunction : quad_to_double

    function single_t abs_single(input single_t x); begin 
        abs_single = x < 0 ? -x : x; 
        end 
    endfunction : abs_single

    function double_t abs_double(input double_t x); begin 
        abs_double = x < 0 ? -x : x; 
        end 
    endfunction : abs_double

    function quad_t abs_quad(input quad_t x);
        abs_quad = x < 0 ? -x : x;
    endfunction : abs_quad

    function real abs_real(input real x);
        abs_real = x < 0 ? -x : x;
    endfunction

    function automatic real random_real_between(input real lo, input real hi);
        int unsigned rng_result = $random();
        real result_from_zero_to_one; 
        real result; begin
        
        result_from_zero_to_one = rng_result / ($pow(2, 32) - 1);
        
        if (0 > result_from_zero_to_one || 1 < result_from_zero_to_one) begin
            $error("Random value from 0 to 1 out of range");
            $finish();
        end

        result = (result_from_zero_to_one * (hi - lo)) + lo;
        
        if (result < lo || result > hi) begin
            $error("Random value from %f to %f out of range: %f", lo, hi, result);
            $finish();
        end
        random_real_between = result;
        
    end
    endfunction : random_real_between
        
    typedef struct {
        real as_float;
        vector::single_t as_single;
        vector::double_t as_double;
    } real_and_fixed_s;

    function real_and_fixed_s real_to_fixed(input real value);
        real temp;
        real conv_temp;
        real also_value;
        real_and_fixed_s scratchpad; begin
        
        scratchpad.as_float = value;
        also_value = value;

        if (value < 0) begin
            also_value = -also_value;
        end

        scratchpad.as_single = '0;
        scratchpad.as_double = '0;

        // we actually start at one beyond the largest valid bit
        // (remember we used signed fixed point values)
        conv_temp = also_value;
        for (int unsigned i = vector::bits_in_single; i > 0; i--) begin : convert_single
            temp = $pow(2.0, i - 2.0 - vector::single_fbits);
            if (conv_temp >= temp) begin
                scratchpad.as_single[i - 2] = 1;
                conv_temp -= temp;
            end
        end : convert_single

        conv_temp = also_value;
        for (int unsigned i = vector::bits_in_double; i > 0; i--) begin : convert_double
            temp = $pow(2, i - 2 - vector::double_fbits);
            if (conv_temp >= temp) begin
                scratchpad.as_double[i - 2] = 1;
                conv_temp -= temp;
            end
        end : convert_double

        if (value < 0) begin
            scratchpad.as_single = -scratchpad.as_single;
            scratchpad.as_double = -scratchpad.as_double;
        end
        real_to_fixed = scratchpad;
    end
    endfunction : real_to_fixed

endpackage : vector

`endif // ifndef VECTORS_VECTOR_PKG_SVH