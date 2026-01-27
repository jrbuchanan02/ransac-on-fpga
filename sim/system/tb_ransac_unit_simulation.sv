
`timescale 1ns / 1ps

`include "vectors/vector_pkg.svh"
`include "simulated_point_cloud.svh"    // the simulated point cloud



module tb_ransac_unit_simulation;

    // read simulation cloud from file and into a simulated
    // AXI-based memory

    // start RANSAC unit and wait for its test to finish.

    // Compare results with expectation. If results are close
    // to the expectation, pass. Otherwise, fail.

    typedef struct {
        vector::vector3s_s fixed;
    } point_in_cloud_s;

    // the point cloud
    point_in_cloud_s cloud[0:simulated_point_cloud::point_count-1];
    vector::single_t cloud_memory[0:3 * simulated_point_cloud::point_count - 1];

    always_comb begin : map_cloud_into_memory
        for (int unsigned i = 0; i < simulated_point_cloud::point_count; i++) begin
            for (int unsigned j = 0; j < 3; j++) begin
                cloud_memory[3 * i + j] = cloud[i].fixed.c[j];
            end
        end
    end : map_cloud_into_memory

    // instantiate a ransac unit, which we will program to find the plane
    // in the cloud. If the plane found is roughly in the range expected
    // with ground_truth_normal and ground_truth_distance_to_origin, then
    // pass. Otherwise, fail.

    logic ransac_clock;
    logic ransac_reset;
    logic [31:0] ransac_control_awaddr;
    logic ransac_control_awvalid;
    logic ransac_control_awready;
    logic [31:0] ransac_control_araddr;
    logic ransac_control_arvalid;
    logic ransac_control_arready;
    logic [31:0] ransac_control_wdata;
    logic [3:0] ransac_control_wstrb;
    logic ransac_control_wvalid;
    logic ransac_control_wready;
    logic [31:0] ransac_control_rdata;
    logic [1:0] ransac_control_rresp;
    logic ransac_control_rvalid;
    logic ransac_control_rready;
    logic [1:0] ransac_control_bresp;
    logic ransac_control_bvalid;
    logic ransac_control_bready;
    logic [31:0] ransac_memory_araddr;
    logic ransac_memory_arvalid;
    logic ransac_memory_arready;
    logic [2:0] ransac_memory_arprot;
    logic [31:0] ransac_memory_arid;
    logic [31:0] ransac_memory_rdata;
    logic [1:0] ransac_memory_rresp;
    logic ransac_memory_rvalid;
    logic ransac_memory_rready;
    logic [31:0] ransac_memory_rid;
    // don't connect Vivado-mandated but optional AXI signals
    // don't connect the AXI mandated but unused write port (for memory)

    localparam logic [31:0] ransac_control_base_addr = 32'h4000_0000;

    task automatic ransac_control_write(
        ref logic clock,
        ref logic [31:0] awaddr,
        ref logic awvalid,
        ref logic awready,
        ref logic [31:0] wdata,
        ref logic [3:0] wstrb,
        ref logic wvalid,
        ref logic wready,
        ref logic bready,
        ref logic bvalid,
        ref logic [1:0] bresp,
        input logic [31:0] offset, input logic [31:0] value, input logic [3:0] strb); 
 
    begin
        
        awaddr = ransac_control_base_addr + offset;
        @(posedge clock) awvalid = 1;
        @(posedge clock);
        // note: normally this is inadvisable in AXI due to how the handshakes
        // work. However, this works in the case of a ready before valid 
        // handshake. The problem is we assume that ready is set before we
        // pulse valid for one full clock cycle.
        //
        // This style of handshake works for this test, however.
        awvalid = 0;
        
        wdata = value;
        wstrb = strb;
        @(posedge clock) wvalid = 1;
        @(posedge clock); // generally inadvisable for same reasons as aw channel
        
        wvalid = 0;

        @(posedge clock) bready = 1;
        @(posedge clock);
        bready = 0;
        if (bresp != 2'b00) begin
            $error("RANSAC unit programming failed: bresp = %x", bresp);
        end
    end
    endtask : ransac_control_write 

    task automatic ransac_control_read(
        ref logic clock,
        ref logic [31:0] araddr,
        ref logic arvalid,
        ref logic arready,
        ref logic [31:0] rdata,
        ref logic [1:0] rresp,
        ref logic rready,
        ref logic rvalid,
        input logic [31:0] offset,
        output logic [31:0] value
    );

    begin
        // todo
    end

    endtask : ransac_control_read

    // generate a clock signal for the testbench at 100MHz
    logic sim_clock;
    initial begin : generate_sim_clock
        sim_clock = 0;
        forever begin
            sim_clock = ~sim_clock;
            #5ns;
        end
    end : generate_sim_clock
    // run the ransac unit at the rate given by sim clock
    assign ransac_clock = sim_clock;


    localparam logic [31:0] sim_memory_base = 32'h8000_0000;
    localparam logic [31:0] sim_memory_last = sim_memory_base + (3 * $bits(vector::single_t) / 8) * simulated_point_cloud::point_count;

    initial begin : run_test
        // hold the ransac unit in reset
        ransac_reset = 1;
        ransac_control_awaddr = 0;
        ransac_control_awvalid = 0;
        ransac_control_araddr = 0;
        ransac_control_arvalid = 0;
        ransac_control_wdata = 0;
        ransac_control_wvalid = 0;
        ransac_control_rready = 0;
        ransac_control_bready = 0;
        ransac_control_wstrb = 4'b1111;
        ransac_memory_rid = 0;
        @(posedge sim_clock);
        @(negedge sim_clock) ransac_reset = 0;

        // ransac_control_awaddr = ransac_control_base_addr + 12;
        // ransac_control_awvalid = 0;
        // @(posedge ransac_clock) ransac_control_awvalid = 1;
        // @(posedge ransac_clock);
        // ransac_control_awvalid = 0;
        // ransac_control_wdata = sim_memory_base;
        // ransac_control_wstrb = 4'b1111;
        // @(posedge ransac_clock) ransac_control_wvalid = 1;
        // @(posedge ransac_clock);

        // ransac_control_wvalid = 0;
        // @(posedge ransac_clock) ransac_control_bready = 1;
        // @(posedge ransac_clock);

        // ransac_control_bready = 0;

        // if (ransac_control_bresp != 2'b00) begin
        //     $error("RANSAC unit programming failed: bresp = %x", ransac_control_bresp);
        // end

        // offset 12 -> cloud base
        ransac_control_write(ransac_clock, ransac_control_awaddr, ransac_control_awvalid, ransac_control_awready, ransac_control_wdata, ransac_control_wstrb, ransac_control_wvalid, ransac_control_wready, ransac_control_bready, ransac_control_bvalid, ransac_control_bresp, 12, sim_memory_base, 4'b1111);
        // offset 16 -> cloud size in points
        ransac_control_write(ransac_clock, ransac_control_awaddr, ransac_control_awvalid, ransac_control_awready, ransac_control_wdata, ransac_control_wstrb, ransac_control_wvalid, ransac_control_wready, ransac_control_bready, ransac_control_bvalid, ransac_control_bresp, 16, simulated_point_cloud::point_count, 4'b1111);
        // offset 20 -> requested iteration count
        ransac_control_write(ransac_clock, ransac_control_awaddr, ransac_control_awvalid, ransac_control_awready, ransac_control_wdata, ransac_control_wstrb, ransac_control_wvalid, ransac_control_wready, ransac_control_bready, ransac_control_bvalid, ransac_control_bresp, 20, simulated_point_cloud::point_count / 10, 4'b1111);
        // offset 28 -> threshold
        ransac_control_write(ransac_clock, ransac_control_awaddr, ransac_control_awvalid, ransac_control_awready, ransac_control_wdata, ransac_control_wstrb, ransac_control_wvalid, ransac_control_wready, ransac_control_bready, ransac_control_bvalid, ransac_control_bresp, 28, simulated_point_cloud::reference_threshold, 4'b1111);
        // offset 0 -> calculation start strobe
        ransac_control_write(ransac_clock, ransac_control_awaddr, ransac_control_awvalid, ransac_control_awready, ransac_control_wdata, ransac_control_wstrb, ransac_control_wvalid, ransac_control_wready, ransac_control_bready, ransac_control_bvalid, ransac_control_bresp, 0, 1, 4'b0001);

        // kludge of sorts to poll the unit state every cycle.
        ransac_control_araddr = ransac_control_base_addr + 8; // caclculation state
        ransac_control_arvalid = 1; // request every cycle
        ransac_control_rready = 1; // always ready to listen.
        
        while (!ransac_control_rvalid) begin
            @(posedge ransac_clock);
        end

        while (ransac_control_rdata[0] == 0) begin
            @(posedge ransac_clock);
        end

        while (ransac_control_rdata[0] == 1) begin
            @(posedge ransac_clock);
        end

        $finish("Simulation complete. Please inspect results in waveform.");


        // note: IRQ not yet implemented.
    end : run_test


    initial begin : init_memory
        // assign the simulated cloud
        for (int unsigned i = 0; i < simulated_point_cloud::point_count; i++) begin
            for (int unsigned j = 0; j < 3; j++) begin
                // cloud[i].double[j] = simulated_point_cloud::cloud[i][j];
                cloud[i].fixed.c[j] = simulated_point_cloud::cloud_fixed_point[i][j];

                if (cloud[i].fixed.c[j] === 32'hxxxxxxxx) begin
                    $error("Bad Vivado. Very bad indeed.");
                    $finish();
                end

            end
        end
    end : init_memory


    logic [31:0] memory_offset;
    // run the memory AXI bus.
    always_comb begin : simulate_memory

        // ready if address is valid.
        if (ransac_memory_araddr >= sim_memory_base && ransac_memory_araddr < sim_memory_last) begin
            ransac_memory_arready = 1;
        end else begin
            ransac_memory_arready = 0;
        end

        // rdata valid and correct address if multiple of 4 bytes AND
        // in range.

        if (ransac_memory_arready && ransac_memory_araddr[1:0] == 0) begin
            memory_offset = ransac_memory_araddr - sim_memory_base;
            memory_offset = memory_offset % (sim_memory_last - sim_memory_base);
            // point index -> memory_offset / 3
            // component -> memory_offset % 3
            ransac_memory_rdata = cloud_memory[memory_offset / 4];
            ransac_memory_rvalid = 1;
            ransac_memory_rresp = 2'b00;
        end else begin
            ransac_memory_rdata = 32'h?;
            ransac_memory_rvalid = 1;
            ransac_memory_rresp = 2'b11;   // decode error
        end
    end : simulate_memory


    ransac_unit_wrapper#(
        .control_addr_base(ransac_control_base_addr),
        .plane_check_unit_count(1),
        .check_inlier_units_per_plane_check_unit(4)
    ) ransac_under_test(
        .clock(ransac_clock),
        .reset(ransac_reset),
        .control_awaddr(ransac_control_awaddr),
        .control_awvalid(ransac_control_awvalid),
        .control_awready(ransac_control_awready),
        .control_araddr(ransac_control_araddr),
        .control_arvalid(ransac_control_arvalid),
        .control_arready(ransac_control_arready),
        .control_wdata(ransac_control_wdata),
        .control_wstrb(ransac_control_wstrb),
        .control_wvalid(ransac_control_wvalid),
        .control_wready(ransac_control_wready),
        .control_rdata(ransac_control_rdata),
        .control_rresp(ransac_control_rresp),
        .control_rvalid(ransac_control_rvalid),
        .control_rready(ransac_control_rready),
        .control_bresp(ransac_control_bresp),
        .control_bvalid(ransac_control_bvalid),
        .control_bready(ransac_control_bready),
        .memory_awaddr( ),
        .memory_awid( ),
        .memory_awprot( ),
        .memory_awvalid( ),
        .memory_awready( ),
        .memory_araddr(ransac_memory_araddr),
        .memory_arvalid(ransac_memory_arvalid),
        .memory_arready(ransac_memory_arready),
        .memory_arprot(ransac_memory_arprot),
        .memory_arid(ransac_memory_arid),
        .memory_wdata( ),
        .memory_wstrb( ),
        .memory_wid( ),
        .memory_wvalid( ),
        .memory_wlast( ),
        .memory_wready( ),
        .memory_rdata(ransac_memory_rdata),
        .memory_rresp(ransac_memory_rresp),
        .memory_rvalid(ransac_memory_rvalid),
        .memory_rready(ransac_memory_rready),
        .memory_rid(ransac_memory_rid),
        .memory_bresp( ),
        .memory_bid( ),
        .memory_bvalid( ),
        .memory_bready( ),
        .memory_awlen( ),
        .memory_arlen( ),
        .memory_awsize( ),
        .memory_arsize( ),
        .memory_awcache( ),
        .memory_arcache( ),
        .memory_awlock( ),
        .memory_arlock( ),
        .memory_awqos( ),
        .memory_arqos( )
    );


endmodule : tb_ransac_unit_simulation