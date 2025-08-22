`timescale 1ns / 1ps

`include "vectors/vector_pkg.svh"

module tb_derive_plane;
    
    typedef struct {
        real x;
        real y;
        real z;
    } vector3r_s;

    function vector3r_s subtract_vec3r(input vector3r_s lhs, input vector3r_s rhs);
        vector3r_s scratchpad;
        scratchpad.x = lhs.x - rhs.x;
        scratchpad.y = lhs.y - rhs.y;
        scratchpad.z = lhs.z - rhs.z;
        subtract_vec3r = scratchpad;
    endfunction : subtract_vec3r

    function vector3r_s cross_product(input vector3r_s lhs, input vector3r_s rhs);
        vector3r_s scratchpad;
        scratchpad.x = lhs.y * rhs.z - lhs.z * rhs.y;
        scratchpad.y = lhs.z * rhs.x - lhs.x * rhs.z;
        scratchpad.z = lhs.x * rhs.y - lhs.y * rhs.x;
        cross_product = scratchpad;
    endfunction : cross_product

    function real dot_product(input vector3r_s lhs, input vector3r_s rhs);
        dot_product = lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    endfunction : dot_product

    typedef struct {
        vector3r_s n;
        real d;
    } plane_of_real_s;

    function automatic plane_of_real_s ground_truth_derive_plane(input vector3r_s a, input vector3r_s b, input vector3r_s c);
        vector3r_s v1 = subtract_vec3r(a, b);
        vector3r_s v2 = subtract_vec3r(a, c);
        plane_of_real_s scratchpad;
        scratchpad.n = cross_product(v1, v2);
        scratchpad.d = dot_product(scratchpad.n, a);
        ground_truth_derive_plane = scratchpad;
    endfunction : ground_truth_derive_plane

    function bit reals_within_tolerance(input real a, input real b, input real tolerance);
        if (a - b < tolerance && a - b >= 0) begin
            reals_within_tolerance = 1;
        end else if (b - a < tolerance && b - a >= 0) begin
            reals_within_tolerance = 1;
        end else begin
            reals_within_tolerance = 0;
        end
    endfunction : reals_within_tolerance

    function automatic real divide_reals_but_0_over_0_is_one(input real num, input real den);
        if (num == 0 && den == 0) begin
            divide_reals_but_0_over_0_is_one = 1;
        end else begin
            divide_reals_but_0_over_0_is_one = num / den;
        end
    endfunction : divide_reals_but_0_over_0_is_one

    function automatic bit resulting_plane_near_ground_truth(input vector::vector3s_s n, input vector::single_t d, input plane_of_real_s ground_truth, input real tolerance);
        // take ratios of dividing some part of the result by the ground truth.
        // if all ratios are near each other then the plane is close enough

        real nx_ratio = divide_reals_but_0_over_0_is_one(n.v.x, ground_truth.n.x);
        real ny_ratio = divide_reals_but_0_over_0_is_one(n.v.y, ground_truth.n.y);
        real nz_ratio = divide_reals_but_0_over_0_is_one(n.v.z, ground_truth.n.z);
        real d_ratio = divide_reals_but_0_over_0_is_one(d, ground_truth.d);

        if (!reals_within_tolerance(nx_ratio, ny_ratio, tolerance)) begin
            resulting_plane_near_ground_truth = 0; 
        end
        else if (!reals_within_tolerance(nx_ratio, nz_ratio, tolerance)) begin
            resulting_plane_near_ground_truth = 0;
        end
        else if (!reals_within_tolerance(nx_ratio, d_ratio, tolerance)) begin
            resulting_plane_near_ground_truth = 0;
        end
        else if (!reals_within_tolerance(ny_ratio, nz_ratio, tolerance)) begin
            resulting_plane_near_ground_truth = 0;
        end
        else if (!reals_within_tolerance(ny_ratio, d_ratio, tolerance)) begin
            resulting_plane_near_ground_truth = 0;
        end
        else if (!reals_within_tolerance(nz_ratio, d_ratio, tolerance)) begin
            resulting_plane_near_ground_truth = 0;
        end else begin
            resulting_plane_near_ground_truth = 1; 
        end

    endfunction : resulting_plane_near_ground_truth

    // note: this tolerance may prove too large
    // in that case, consider switching from fixed point to floating
    // point.
    localparam real tolerance = 1e-2;

    localparam int unsigned test_plane_count = 1 << 20;

    logic clock;
    logic reset;
    logic ivalid;
    logic iready;
    vector::point_t a;
    vector::point_t b;
    vector::point_t c;
    logic ovalid;
    logic oacknowledge;
    vector::vector3s_s n;
    vector::single_t d;
    vector::derive_plane_status_e status;

    int unsigned test_number;

    // manage clock and reset
    initial begin : clock_signal
        clock = 0;
        forever begin
            #5;
            clock = ~clock;
        end
    end : clock_signal

    initial begin : reset_signal
        reset = 1;
        @(posedge clock);
        @(negedge clock);
        reset = 0;
    end : reset_signal

    struct {
        vector3r_s a;
        vector3r_s b;
        vector3r_s c;
    } generated_params;
    
    // used to allow putting the expected plane in the waveform.
    plane_of_real_s expected_ground_truth;

    task automatic randomize_params();
        generated_params.a.x = vector::random_real_between(-10, +10);
        generated_params.a.y = vector::random_real_between(-10, +10);
        generated_params.a.z = vector::random_real_between(-10, +10);

        generated_params.b.x = vector::random_real_between(-10, +10);
        generated_params.b.y = vector::random_real_between(-10, +10);
        generated_params.b.z = vector::random_real_between(-10, +10);

        generated_params.c.x = vector::random_real_between(-10, +10);
        generated_params.c.y = vector::random_real_between(-10, +10);
        generated_params.c.z = vector::random_real_between(-10, +10);
    endtask : randomize_params

    always @(posedge clock) begin : actually_test

        ivalid = 0;
        oacknowledge = 1;

        if (!reset) begin : test_logic
            if (ovalid && !iready) begin
                // check that the generated plane is within tolerance
                if (!resulting_plane_near_ground_truth(n, d, ground_truth_derive_plane(generated_params.a, generated_params.b, generated_params.c), tolerance)) begin
                    $error("Mismatched planes: difference between the two generated planes falls outside of the allowed tolerance");
                    $finish(2);
                end

                if (status == vector::DERIVE_PLANE_STATUS_SUCCESS && (a == b || b == c || a == c)) begin
                    $error("Less than 2 unique points but the derive_plane function succeeded (task failed by succeeding, I guess)");
                    $finish(2);
                end
                test_number++;
                randomize_params();
                if (test_number >= test_plane_count) begin
                    $info("Finished");
                    $finish(2);
                end
            end else if (iready) begin
                ivalid = 1;
                a.v.x = vector::real_to_fixed(generated_params.a.x).as_single;
                a.v.y = vector::real_to_fixed(generated_params.a.y).as_single;
                a.v.z = vector::real_to_fixed(generated_params.a.z).as_single;

                b.v.x = vector::real_to_fixed(generated_params.b.x).as_single;
                b.v.y = vector::real_to_fixed(generated_params.b.y).as_single;
                b.v.z = vector::real_to_fixed(generated_params.b.z).as_single;

                c.v.x = vector::real_to_fixed(generated_params.c.x).as_single;
                c.v.y = vector::real_to_fixed(generated_params.c.y).as_single;
                c.v.z = vector::real_to_fixed(generated_params.c.z).as_single;
                
                expected_ground_truth = ground_truth_derive_plane(generated_params.a, generated_params.b, generated_params.c);
            end
        end : test_logic
    end : actually_test

    derive_plane#(
        .reset_polarity(1)
    ) dut(
        .clock(clock),
        .reset(reset),
        .ivalid(ivalid),
        .iready(iready),
        .a(a),
        .b(b),
        .c(c),
        .ovalid(ovalid),
        .oacknowledge(oacknowledge),
        .n(n),
        .d(d),
        .status(status)
    );

endmodule : tb_derive_plane