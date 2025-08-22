/**
 * Verilog wrapper for ransac_unit
 */

`timescale 1ns / 1ps


module ransac_unit_wrapper#(
    parameter integer memory_addr_width = 32,
    parameter integer memory_data_width = 32,   // keep this value synced up with
                                                // vector::bits_in_single
    parameter integer memory_maximum_latency = 1 << 10,
    parameter integer maximum_point_count = 1 << 20,
    parameter integer maximum_iteration_count = 1 << 20,
    parameter reg [31:0] memory_rid_base = 32'h0000_0000,
    parameter integer control_addr_width = 32,
    parameter integer control_data_width = 32,
    parameter integer plane_check_unit_count = 2,
    parameter integer check_inlier_units_per_plane_check_unit = 2,
    parameter integer small_fma_latency = 4,
    parameter integer large_fma_latency = 16,
    parameter integer plane_fma_latency = 16,
    parameter integer lfsr_register_width = 64,
    parameter integer lfsr_window_width = 32,
    parameter integer lfsr_window_start = 16,
    parameter reg [lfsr_register_width-1:0] lfsr_seed = {
        // seed chosen arbitrarily from the values which, given the default
        // polynomial, do not place the LFSR in an infinite loop.
        //
        // $random() was considered for the defualt value of this parameter but
        // Vivado synthesis would make $random return 0 for this parameter (which
        // would place the LFSR in an infinite loop)
        64'h0123_4567_89AB_CDEF
    },
    parameter reg [lfsr_register_width-1:0] lfsr_polynomial = 64'hD800_0000_0000_0000,
    parameter reg [control_addr_width-1:0] control_addr_base = 32'h4000_0000,
    parameter reg reset_polarity = 1)(
    input wire clock,
    input wire reset,

    // control port (responds via AXI Lite)
    input wire [control_addr_width-1:0] control_awaddr,
    input wire control_awvalid,
    output wire control_awready,

    input wire [control_addr_width-1:0] control_araddr,
    input wire control_arvalid,
    output wire control_arready,

    input wire [control_data_width-1:0] control_wdata,
    input wire [control_data_width/8-1:0] control_wstrb,
    input wire control_wvalid,
    output wire control_wready,

    output wire [control_data_width-1:0] control_rdata,
    output wire [1:0] control_rresp,
    output wire control_rvalid,
    input wire control_rready,

    output wire [1:0] control_bresp,
    output wire control_bvalid,
    input wire control_bready,

    // memory ports

    // Note: we do not use the write port but it still has to exist.
    output wire [memory_addr_width-1:0] memory_awaddr,
    output wire  memory_awvalid,
    input wire  memory_awready,

    // Note: we will heavily use the read port
    output wire [memory_addr_width-1:0] memory_araddr,
    output wire  memory_arvalid,
    input wire  memory_arready,

    // Note: we will not use the write port but it still has to exist
    output wire [memory_data_width-1:0] memory_wdata,
    output wire [memory_data_width/8-1:0] memory_wstrb,
    output wire  memory_wvalid,
    input wire  memory_wready,

    input wire [memory_data_width-1:0] memory_rdata,
    input wire [1:0] memory_rresp,
    input wire  memory_rvalid,
    output wire  memory_rready,

    input wire [1:0] memory_bresp,
    input wire  memory_bvalid,
    output wire  memory_bready
    );

    ransac_unit#(
        .memory_addr_width(memory_addr_width),
        .memory_data_width(memory_data_width),
        .memory_maximum_latency(memory_maximum_latency),
        .maximum_point_count(maximum_point_count),
        .control_addr_width(control_addr_width),
        .control_data_width(control_data_width),
        .plane_check_unit_count(plane_check_unit_count),
        .check_inlier_units_per_plane_check_unit(check_inlier_units_per_plane_check_unit),
        .small_fma_latency(small_fma_latency),
        .large_fma_latency(large_fma_latency),
        .plane_fma_latency(plane_fma_latency),
        .lfsr_register_width(lfsr_register_width),
        .lfsr_window_width(lfsr_window_width),
        .lfsr_window_start(lfsr_window_start),
        .lfsr_seed(lfsr_seed),
        .control_addr_base(control_addr_base),
        .reset_polarity(reset_polarity)
    ) wrapped(
        .clock(clock),
        .reset(reset),
        .control_awaddr(control_awaddr),
        .control_awvalid(control_awvalid),
        .control_awready(control_awready),
        .control_araddr(control_araddr),
        .control_arvalid(control_arvalid),
        .control_arready(control_arready),
        .control_wdata(control_wdata),
        .control_wstrb(control_wstrb),
        .control_wvalid(control_wvalid),
        .control_wready(control_wready),
        .control_rdata(control_rdata),
        .control_rresp(control_rresp),
        .control_rvalid(control_rvalid),
        .control_rready(control_rready),
        .control_bresp(control_bresp),
        .control_bvalid(control_bvalid),
        .control_bready(control_bready),
        .memory_awaddr(memory_awaddr),
        .memory_awvalid(memory_awvalid),
        .memory_awready(memory_awready),
        .memory_araddr(memory_araddr),
        .memory_arvalid(memory_arvalid),
        .memory_arready(memory_arready),
        .memory_wdata(memory_wdata),
        .memory_wstrb(memory_wstrb),
        .memory_wvalid(memory_wvalid),
        .memory_wready(memory_wready),
        .memory_rdata(memory_rdata),
        .memory_rresp(memory_rresp),
        .memory_rvalid(memory_rvalid),
        .memory_rready(memory_rready),
        .memory_bresp(memory_bresp),
        .memory_bvalid(memory_bvalid),
        .memory_bready(memory_bready)
    );

endmodule