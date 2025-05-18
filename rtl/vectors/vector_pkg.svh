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
    typedef vector3s_s point3s_s;

    localparam type point_t = point3s_s;
endpackage : vector

`endif // ifndef VECTORS_VECTOR_PKG_SVH