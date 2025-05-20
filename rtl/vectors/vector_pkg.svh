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
    localparam int unsigned fma_latency_doubles = 6;
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

    function single_t abs_single(input single_t x); begin 
        abs_single = x < 0 ? -x : x; 
        end 
    endfunction : abs_single

    function double_t abs_double(input double_t x); begin 
        abs_double = x < 0 ? -x : x; 
        end 
    endfunction : abs_double
endpackage : vector

`endif // ifndef VECTORS_VECTOR_PKG_SVH