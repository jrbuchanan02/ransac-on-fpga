`timescale 1ns/1ps

`include "fixed_point.svh"

module simulation_top_level;

    
    // TODO: read a real point cloud from some (CSV) file

    localparam point_addr_width = 9;
    localparam max_point_count = 1 << point_addr_width;

    ransac_fixed::point_t example_cloud[max_point_count-1:0];
    ransac_fixed::plane_t resulting_plane;

    logic clock;
    logic reset;

    logic rng_clock;

    ransac_fixed::point_t point_to_logic;
    logic point_addr_valid;
    logic point_data_valid;
    logic [point_addr_width-1:0] point_addr;
    logic [point_addr_width-1:0] point_count;
    logic [31:0] iterations;
    logic calculation_start;
    ransac_fixed::fixed_t threshold;
    logic calculation_can_start;
    logic calculation_done;

    initial begin : test_design
        reset <= 1;
        calculation_start <= 0;
        point_count <= max_point_count - 1;
        @(posedge clock);
        @(negedge clock);
        @(posedge clock);
        // threshold: 0.75
        threshold <= (ransac_fixed::one() >> 1) + (ransac_fixed::one() >> 2);
        // iterations: 1000
        iterations <= 1000;

        // TODO: read in the point cloud from somewhere

        // for now, generate 1/2 points on the imaginary ground plane
        // then 1 / 4 points below threshold
        // then 1 / 4 points above threshold
        //
        // our example plane will be z = 0.5
        //
        // so we should expect something near that

        for (int i = 0; i < max_point_count; i++) begin
            case (i & 3)
            0, 1: begin
                for (int j = 0; j < ransac_fixed::value_bits(); j++) begin
                    example_cloud[i].x[j] = $random() & 1;
                    example_cloud[i].y[j] = $random() & 1;
                end
                example_cloud[i].z = 0;
                for (int j = 0; j < ransac_fixed::fraction_bits; j++) begin
                    example_cloud[i].z[j] = $random() & 1;
                end
            end
            2: begin
                for (int j = 0; j < ransac_fixed::value_bits(); j++) begin
                    example_cloud[i].x[j] = $random() & 1;
                    example_cloud[i].y[j] = $random() & 1;
                end
                example_cloud[i].z = -($random() & ((1 << ransac_fixed::integral_bits) - 1)) << ransac_fixed::fraction_bits;
                if (example_cloud[i].z > 0) begin
                    example_cloud[i].z = -example_cloud[i].z;
                end
                for (int j = 0; j < ransac_fixed::fraction_bits; j++) begin
                    example_cloud[i].z[j] = $random() & 1;
                end
            end
            3: begin
                for (int j = 0; j < ransac_fixed::value_bits(); j++) begin
                    example_cloud[i].x[j] = $random() & 1;
                    example_cloud[i].y[j] = $random() & 1;
                end
                example_cloud[i].z = ($random() & ((1 << ransac_fixed::integral_bits) - 1)) << ransac_fixed::fraction_bits;
                if (example_cloud[i].z < 0) begin
                    example_cloud[i].z = -example_cloud[i].z;
                end
                for (int j = 0; j < ransac_fixed::fraction_bits; j++) begin
                    example_cloud[i].z[j] = $random() & 1;
                end
            end
            endcase
        end

        // finished preparing the points, start the calculation
        @(negedge clock) begin 
            calculation_start <= 1;
            reset <= 0;
        end

        // wait until calculation finishes
        @(posedge calculation_done) begin
            $display("Plane found is 0x%0h * x + 0x%0h * y + 0x%0h * z = 0x%0h", resulting_plane.normal.x, resulting_plane.normal.y, resulting_plane.normal.z, resulting_plane.d);
        end

    end : test_design

    ransac_logic#(
        .point_pipeline_multiply_latency(ransac_fixed::value_bits() / 8),
        .plane_calculate_multiply_latency(ransac_fixed::value_bits() / 16),
        .rsqrt_iterations(100),
        .reset_polarity(1),
        .max_point_count(max_point_count)
        // leave the LFSR seeds untouched
    ) dut(
        .clock(clock),
        .rng_clock(rng_clock),
        .reset(reset),
        .point_in(point_to_logic),
        .point_addr_valid(point_addr_valid),
        .point_data_valid(point_data_valid),
        .point_addr(point_addr),
        .point_count(point_count),
        .iterations(iterations),
        .calculation_start(calculation_start),
        .threshold(threshold),
        .calculation_can_start(calculation_can_start),
        .calculation_done(calculation_done),
        .plane(resulting_plane)
    );

    always @* begin : control_point_memory
        point_to_logic <= example_cloud[point_addr];
        point_data_valid <= 1;
    end : control_point_memory

    initial begin : control_clock
        clock <= 0;
        forever begin
            # 5;
            clock <= ~clock;
        end
    end : control_clock

    initial begin : control_rng_clock
        rng_clock <= 0;
        forever begin
            # 3.333
            rng_clock <= ~rng_clock;
        end
    end : control_rng_clock

endmodule : simulation_top_level