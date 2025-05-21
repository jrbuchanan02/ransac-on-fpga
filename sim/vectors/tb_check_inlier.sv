`timescale 1ns/1ps

`include "vectors/vector_pkg.svh"



function real dot_product(input real ax, input real ay, input real az, input real bx, input real by, input real bz); begin
    dot_product = ax * bx + ay * by + az * bz;
end
endfunction

function real absolute_value(input real x); begin
    if (x < 0) absolute_value = -x;
    else absolute_value = x;
end
endfunction

module tb_check_inlier;

    // threshold as a real
    localparam real threshold_as_real = 0.25;

    // minimum x bounds
    localparam real min_x = -10;
    // minimum y bounds
    localparam real min_y = -10;
    // minimum z bounds
    localparam real min_z = -10;
    
    // maximum x bounds
    localparam real max_x = +10;
    // maximum y bounds
    localparam real max_y = +10;
    // maximum z bounds
    localparam real max_z = +10;

    // number of simulated points to generate
    // (approx. 1 million)
    localparam int unsigned point_count = 1 << 20;

    // list of randomly generated points in a point cloud
    // 1/2 of these points will be generated as inliers
    // the rest as outliers
    real cloud_x[point_count-1:0];
    real cloud_y[point_count-1:0];
    real cloud_z[point_count-1:0];

    struct {
        real x;
        real y;
        real z;
    } plane_n_as_real;
    real plane_d_as_real;

    logic [1:0] chosen_coordinate;

    // seed the RNG
    // don't care about the unix epoch overflow since I want
    // the simulation results to use different inputs at t = n
    // versus t = n + 1
    int unsigned unix_timestamp_as_int;
    int file_pointer;
    int unsigned discard;
    initial begin : seed_rng
    // for whatever reason Vivado runs $system in cmd.exe
    // and not in something with the usual commands that work
    // everywhere else (e.g., Powershell, Bash, xterm)
    //
    // and Vivado even has a linux version!
//        $system("date +%s > unix_timestamp");
//        file_pointer = $fopen("unix_timestamp", "r");
//        void'($fscanf(file_pointer, "%ud", unix_timestamp_as_int));
//        discard = $random(unix_timestamp_as_int);
//        $fclose(file_pointer);

          discard = $random(32'h1234_5678);
    end : seed_rng

    initial begin : derive_plane
        
        // other planes should be tested
        // but also remember that without seeding the rng properly,
        // a plane generated with $random will be the same one every
        // time the simulation runs.
        chosen_coordinate = $random() % 3;
        case (chosen_coordinate)
        0: begin
            plane_n_as_real.x = 1;
            plane_n_as_real.y = 0;
            plane_n_as_real.z = 0;
            plane_d_as_real = vector::random_real_between(0.75*min_x, 0.75*max_x);
        end
        1: begin
            plane_n_as_real.x = 0;
            plane_n_as_real.y = 1;
            plane_n_as_real.z = 0;
            plane_d_as_real = vector::random_real_between(0.75*min_y, 0.75*max_y);
        end
        2: begin
            plane_n_as_real.x = 0;
            plane_n_as_real.y = 0;
            plane_n_as_real.z = 1;
            plane_d_as_real = vector::random_real_between(0.75*min_z, 0.75*max_z);
        end
        endcase

        

    end : derive_plane

    initial begin : generate_points
        // generate every other point as an inlier
        for (int unsigned i = 0; i < point_count; i++) begin
            if (i & 1) begin
                // inlier
                
                cloud_x[i] = chosen_coordinate == 0 ? vector::random_real_between(min_x, max_x) : (plane_d_as_real + vector::random_real_between(-threshold_as_real, +threshold_as_real));
                cloud_y[i] = chosen_coordinate == 1 ? vector::random_real_between(min_y, max_y) : (plane_d_as_real + vector::random_real_between(-threshold_as_real, +threshold_as_real));
                cloud_z[i] = chosen_coordinate == 2 ? vector::random_real_between(min_z, max_z) : (plane_d_as_real + vector::random_real_between(-threshold_as_real, +threshold_as_real));
                
            end else begin
                // outlier (could happen to be an inlier)

                cloud_x[i] = vector::random_real_between(min_x, max_x);
                cloud_y[i] = vector::random_real_between(min_y, max_y);
                cloud_z[i] = vector::random_real_between(min_z, max_z);

            end
        end
    end : generate_points

    logic clock;
    logic reset;
    logic ivalid;
    logic iready;
    vector::vector3s_s n;
    vector::vector3s_s p;
    vector::single_t d;
    vector::single_t t;
    logic ovalid;
    logic oacknowledge;
    logic inlier;

    // use the default parameters
    check_inlier dut(
        .clock(clock),
        .reset(reset),
        .ivalid(ivalid),
        .iready(iready),
        .n(n),
        .p(p),
        .d(d),
        .t(t),
        .ovalid(ovalid),
        .oacknowledge(oacknowledge),
        .inlier(inlier)
    );

    initial begin : generate_clock_signal
        clock = 0;
        forever begin : clock_loop
            #5; // equiv. to a 100MHz clock rate
            clock <= ~clock;
        end : clock_loop
    end : generate_clock_signal

    initial begin : manage_reset
        reset = 1;
        #50;
        reset = 0;
    end : manage_reset
    
    int unsigned point_to_test = 0;
    
    real true_distance;

    always @(posedge clock) begin : actually_test
        n.v.x = vector::real_to_fixed(plane_n_as_real.x).as_single;
        n.v.y = vector::real_to_fixed(plane_n_as_real.y).as_single;
        n.v.z = vector::real_to_fixed(plane_n_as_real.z).as_single;
        d = vector::real_to_fixed(plane_d_as_real).as_single;
        t = vector::real_to_fixed(threshold_as_real).as_single;
        ivalid = 0;
        oacknowledge = 1;
        if (!reset) begin : test_logic
            // if ovalid is set, check the input
            // otherwise, if iready is set, provide new input
            // if neither are set, then do nothing

            if (ovalid && !iready) begin : verify_result
                true_distance = absolute_value(
                    dot_product(
                        plane_n_as_real.x,
                        plane_n_as_real.y,
                        plane_n_as_real.z,
                        cloud_x[point_to_test],
                        cloud_y[point_to_test],
                        cloud_z[point_to_test]) - plane_d_as_real
                ) / $sqrt(
                    dot_product(
                        plane_n_as_real.x,
                        plane_n_as_real.y,
                        plane_n_as_real.z,
                        plane_n_as_real.x,
                        plane_n_as_real.y,
                        plane_n_as_real.z
                    )
                );
                if (threshold_as_real >= true_distance) begin : should_be_inlier
                    if (inlier) begin
                        $display("Point %d correctly labeled inlier", point_to_test);
                    end 
                    else begin
                        $error("Point %d incorrectly labeled as outlier: distance is %f", point_to_test, true_distance);
                        $finish();
                    end
                end : should_be_inlier
                else begin : should_not_be_inlier
                    if (inlier) begin
                        $error("Point %d incorrectly labeled as inlier: distance is %f", point_to_test, true_distance);
                        $finish();
                    end
                    else begin
                        $display("Point %d correctly labeled outlier", point_to_test);
                    end
                end : should_not_be_inlier
                point_to_test++;
                if (point_to_test >= point_count) begin : check_if_done
                    $display("Finished");
                    $finish();
                end : check_if_done
            end : verify_result
            else if (iready) begin : provide_new_input
                ivalid = 1;
                p.v.x = vector::real_to_fixed(cloud_x[point_to_test]).as_single;
                p.v.y = vector::real_to_fixed(cloud_y[point_to_test]).as_single;
                p.v.z = vector::real_to_fixed(cloud_z[point_to_test]).as_single;
            end : provide_new_input
        end : test_logic
    end : actually_test

endmodule : tb_check_inlier
