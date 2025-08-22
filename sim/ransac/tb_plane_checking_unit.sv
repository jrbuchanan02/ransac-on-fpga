
`timescale 1ns / 1ps

`include "vectors/vector_pkg.svh"

module tb_plane_checking_unit;
    
    // number of points to put in the cloud
    localparam int unsigned cloud_length = 1 << 10;
    // number of planes to test
    localparam int unsigned plane_count = 1 << 10;
    // if set, memory latency will be random between
    // 0 and (max_memory_latency + 1)
    // and part of the test will be if the plane_checking_unit
    // errors out when the memory latency is too long
    localparam bit random_memory_latency = 0;
    // if set, there is a chance that some bus error will occur
    // and part of the test will be that the plane_checking_unit
    // errors out if a bus error occurs
    localparam bit random_bus_error_possible = 0;
    // if set, one of the planes will be the same point
    // 3 times in a row and plane_checking_unit must give an
    // error during that iteration
    localparam bit check_same_point_3x = 0;
    // if set, three of the planes will be the same point
    // twice in a row (a, b, and c will each have one test
    // as the unique point) and plane_checking_unit must give
    // an error during all 3 iterations
    localparam bit check_same_point_2x = 0;

    // minimum coordinate in x, y, or z for all iterations
    localparam real box_min = -10;
    // maximum coordinate in x, y, or z for all iterations
    localparam real box_max = +10;

    // threshold for whether a point is an inlier
    localparam real threshold_as_real = 0.25;

    // seed for the normal distribution used to 
    // generate points which are roughly on the
    // plane but have a chance to be outside the
    // plane
    int normal_dist_seed = $random();
    

    // test process:
    // 1. generate a random "true" ground plane using 3 points
    //    within box_min and box_max. Ensure that this ground
    //    plane has a normalized vector n and that d is a reasonable
    //    value.
    // 2. generate a mostly-random point cloud where points have a
    //    10% chance to be completely random and a 90% chance to fall
    //    within 1 sigma (sigma = threshold) of a point on the "true"
    //    ground plane.
    // --- 1 and 2 done before time starts ---
    // 3. if check_same_point_3x, make the first iteration (which will not
    //    count towards plane_iterations) use points a, b, and c as the
    //    points with an index equal to the same (randomly generated) number
    // 4. if check_same_point_3x, run the iteration for the above plane to
    //    "derive" and ensure that the dut reports an error of 
    //    PLANE_CHECKING_UNIT_STATUS_DERIVE_PLANE_ERROR (or however I 
    //    spelled it in the documentation). If the status is incorrect, then
    //    fail the test.
    // 5. if check_same_point_2x, make the next 3 iterations (which will
    //    not count towards plane_iterations) use two of a, b, and c as
    //    the points with an index equal to the same (randomly generated)
    //    number, the third of a, b, and c will use a point with an index
    //    equal to a different (randomly generated) number.
    // 6. if check_same_point_2x, run the above 3 iterations and ensure that
    //    an error occurs as described in (4)
    // --- 3, 4, 5, and 6 are special cases ---
    // 7. select a, b, and c as 3 randomly chosen points. Ensure all 3 points
    //    are unique. 
    // 8. submit the calculation
    // 9. if a bus error occurs when a bus error should have occurred, keep going
    //    and don't count this iteration against the number of plane_iterations
    // 10. if a bus error occurs when one should not have (or vice versa), stop.
    // 11. derive the plane ourselves and count the actual number of inliers.
    //     The results should be within a reasonable margin of error. If not, error out
    //     if the results are in a reasonable margin of error, keep going.
    // 12. repeat 7 - 11 until the number of intentionally-successful plane iterations equals the
    //     amount to test for.

    // vector type to hold a "true" point
    typedef struct {
        real x;
        real y;
        real z;
    } vector3r_s;

    typedef struct {
        vector3r_s n;
        real d;
    } plane_of_real_s;

    // function to generate a "true" point uniformly
    // placed within the box

    function automatic vector3r_s create_point_uniform();
        vector3r_s scratchpad;
        scratchpad.x = vector::random_real_between(box_min, box_max);
        scratchpad.y = vector::random_real_between(box_min, box_max);
        scratchpad.z = vector::random_real_between(box_min, box_max);
        create_point_uniform = scratchpad;
    endfunction : create_point_uniform

    function automatic bit points_too_close(input vector3r_s a, input vector3r_s b, input real epsilon);
        vector3r_s difference;
        difference.x = vector::abs_real(a.x - b.x);
        difference.y = vector::abs_real(a.y - b.y);
        difference.z = vector::abs_real(a.z - b.z);

        if (difference.x > epsilon) begin
            points_too_close = 0;
        end else if (difference.y > epsilon) begin
            points_too_close = 0;
        end  else if (difference.z > epsilon) begin
            points_too_close = 0;
        end else begin
            points_too_close = 1;
        end

    endfunction : points_too_close

    struct {
        vector3r_s a = create_point_uniform();
        vector3r_s b = create_point_uniform();
        vector3r_s c = create_point_uniform();
        real epsilon = 0.5;
        // goes to logic 1 when a, b, c are found
        logic well_formed = 0;
    } true_plane_points;

    // these should be two (almost) equivalent planes
    // expect_derived should have a value for n and d
    // similar to the one generated by derive_plane 
    // (accounting for differences in rounding mode)
    //
    // normalized_n normalizes the n-vector and scales
    // d accordingly. I expect that normalized_n will
    // be more useful to generate the cloud with.
    struct {
        plane_of_real_s expect_derived;
        plane_of_real_s normalized_n;
        // goes logic 1 when true plane is derived
        logic derived = 0;
    } true_plane;

    struct {
        vector3r_s v1;
        vector3r_s v2;
        real magnitude_of_expect_derived_n;
    } true_plane_scratchpad;

    // exactly what the label says, ensure that the true plane consists of 3 points which are
    // "far enough" in all 3 axes
    initial begin : ensure_well_formed_true_plane
        while (!true_plane_points.well_formed) begin : make_well_formed
            if (points_too_close(true_plane_points.a, true_plane_points.b, true_plane_points.epsilon)) begin
                true_plane_points.b = create_point_uniform();
                true_plane_points.well_formed = 0;
                continue;
            end
            if (points_too_close(true_plane_points.a, true_plane_points.c, true_plane_points.epsilon)) begin
                true_plane_points.c = create_point_uniform();
                true_plane_points.well_formed = 0;
                continue;
            end
            if (points_too_close(true_plane_points.b, true_plane_points.c, true_plane_points.epsilon)) begin
                true_plane_points.c = create_point_uniform();
                true_plane_points.well_formed = 0;
                continue;
            end
            true_plane_points.well_formed = 1;
        end : make_well_formed
    end : ensure_well_formed_true_plane

    initial begin : derive_true_plane
        // as soon as the plane is well formed (which technically happens at time = 0),
        // begin deriving the plane
        @(posedge true_plane_points.well_formed) begin
            true_plane_scratchpad.v1.x = true_plane_points.a.x - true_plane_points.b.x;
            true_plane_scratchpad.v1.y = true_plane_points.a.y - true_plane_points.b.y;
            true_plane_scratchpad.v1.z = true_plane_points.a.z - true_plane_points.b.z;

            true_plane_scratchpad.v2.x = true_plane_points.a.x - true_plane_points.c.x;
            true_plane_scratchpad.v2.y = true_plane_points.a.y - true_plane_points.c.y;
            true_plane_scratchpad.v2.z = true_plane_points.a.z - true_plane_points.c.z;

            // derived n is cross product of v1 and v2
            true_plane.expect_derived.n.x = true_plane_scratchpad.v1.y * true_plane_scratchpad.v2.z - true_plane_scratchpad.v1.z * true_plane_scratchpad.v2.y;
            true_plane.expect_derived.n.y = true_plane_scratchpad.v1.z * true_plane_scratchpad.v2.x - true_plane_scratchpad.v1.x * true_plane_scratchpad.v2.z;
            true_plane.expect_derived.n.z = true_plane_scratchpad.v1.x * true_plane_scratchpad.v2.y - true_plane_scratchpad.v1.y * true_plane_scratchpad.v2.x;

            true_plane.expect_derived.d = true_plane.expect_derived.n.x * true_plane_points.a.x + 
                                          true_plane.expect_derived.n.y * true_plane_points.a.y + 
                                          true_plane.expect_derived.n.z * true_plane_points.a.z;

            true_plane_scratchpad.magnitude_of_expect_derived_n = $sqrt(
                true_plane.expect_derived.n.x * true_plane.expect_derived.n.x + 
                true_plane.expect_derived.n.y * true_plane.expect_derived.n.y + 
                true_plane.expect_derived.n.z * true_plane.expect_derived.n.z
            );

            true_plane.normalized_n.n.x = true_plane.expect_derived.n.x / true_plane_scratchpad.magnitude_of_expect_derived_n;
            true_plane.normalized_n.n.y = true_plane.expect_derived.n.y / true_plane_scratchpad.magnitude_of_expect_derived_n;
            true_plane.normalized_n.n.z = true_plane.expect_derived.n.z / true_plane_scratchpad.magnitude_of_expect_derived_n;
            true_plane.normalized_n.d = true_plane.expect_derived.d / true_plane_scratchpad.magnitude_of_expect_derived_n;

            true_plane.derived = 1;
        end
    end : derive_true_plane

    // structure to hold the point cloud
    typedef struct {
        vector::real_and_fixed_s x;
        vector::real_and_fixed_s y;
        vector::real_and_fixed_s z;
    } point_in_cloud_s;

    // the point cloud
    point_in_cloud_s cloud[cloud_length-1:0];


    // function to generate a point in the cloud
    function automatic point_in_cloud_s generate_point_in_cloud();
        // safeguard: ensure that the "true" plane is valid

        if (!true_plane.derived) begin
            $error("generate_point_in_cloud called before true_plane.derived set.");
            $finish(2);
        end

        // 1. transform the box s.t. the derived plane is solved by y = 0. This transformation
        // will consist of a rotation and a translation (rotation moves the normal vector to 
        // equal the y-axis, translation makes the distance from the origin 0). For simplicity,
        // the translation can be applied before the rotation as we already know the required 
        // translation to bring the point on the plane corresponding to (0, ?, 0) to the origin
        // as that's just d / y_component

        real translation_amount = true_plane.normalized_n.d / true_plane.normalized_n.y;

        // rotation brings n.x -> 0, n.y -> 1, n.z -> 0
        

    endfunction : generate_point_in_cloud

endmodule : tb_plane_checking_unit