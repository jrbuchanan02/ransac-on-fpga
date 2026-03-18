#include "sw_ransac.h"

#include "stdbool.h"
#include "stddef.h"

// copy of gcc's rand function

uint32_t random_generator() {
    static uint32_t seed = 'rand'; // chosen arbitrarily
    seed = ((seed * 1103515245) + 12345) % 0x80000000;
    return seed;
}

int32_t fp_multiply(int32_t lhs, int32_t rhs) {
    int64_t product = lhs;
    product *= rhs;
    product >>= 20; // fraction bits
    return (int32_t)product;
}

struct sw_ransac_results sw_ransac_run(uint32_t const *cloud, uint32_t size, uint32_t iterations, uint32_t threshold) {
    struct sw_ransac_results results;
    results.inliers = 0;
    for (uint32_t i = 0; i < iterations; i++) {
        // 3 'random' points
        uint32_t a_index = random_generator() % size;
        uint32_t b_index = random_generator() % size;
        uint32_t c_index = random_generator() % size;

        int32_t a_point[3];
        int32_t b_point[3];
        int32_t c_point[3];

        for (uint32_t j = 0; j < 3; j++) {
            a_point[j] = (int32_t)cloud[3 * a_index + j];
            b_point[j] = (int32_t)cloud[3 * b_index + j];
            c_point[j] = (int32_t)cloud[3 * c_index + j];
        }

        int32_t v1[3];
        int32_t v2[3];
        
        for (uint32_t j = 0; j < 3; j++) {
            v1[j] = a_point[j] - b_point[j];
            v2[j] = a_point[j] - c_point[j];
        }

        int32_t n[3];
        n[0] = fp_multiply(v1[1], v2[2]) - fp_multiply(v1[2], v2[1]);
        n[1] = fp_multiply(v1[2], v2[0]) - fp_multiply(v1[0], v2[2]);
        n[2] = fp_multiply(v1[0], v2[1]) - fp_multiply(v1[1], v2[0]);
        int32_t d = 0;
        for (uint32_t j = 0; j < 3; j++) {
            d += fp_multiply(n[j], a_point[j]);
        }

        bool at_least_three_unique_points = false;
        for (uint32_t j = 0; j < 3; j++) {
            if (n[j] != 0) {
                at_least_three_unique_points = true;
                break;
            }
        }

        // if not at least 3 unique points, stop here and 
        // try again.
        if (!at_least_three_unique_points) {
            i--;
            continue;
        }

        struct sw_ransac_results iteration_results;
        iteration_results.inliers = 0;
        iteration_results.nx = n[0];
        iteration_results.ny = n[1];
        iteration_results.nz = n[2];
        iteration_results.d = d;

        for (uint32_t j = 0; j < size; j++) {
            int32_t p[3];
            for (uint32_t k = 0; k < 3; k++) {
                p[k] = cloud[3 * j + k];
            }
            
            // d * d - 2 * d * (n dot p) + (n dot p) * (n dot p) - threshold * threshold * (n dot n) <= 0
            int32_t n_dot_p = 0;
            int32_t n_dot_n = 0;
            int32_t d_squared = 0;
            int32_t t_squared = 0;

            for (uint32_t k = 0; k < 3; k++) {
                n_dot_p += fp_multiply(n[k], p[k]);
                n_dot_n += fp_multiply(n[k], n[k]);
            }
            d_squared = fp_multiply(d, d);
            t_squared = fp_multiply((int32_t)threshold, (int32_t)threshold);

            int32_t test_value = 
                d_squared - 
                2 * fp_multiply(d, n_dot_p) + 
                fp_multiply(n_dot_p, n_dot_p) - 
                fp_multiply(t_squared, n_dot_n);

            if (test_value <= 0) {
                iteration_results.inliers++;
            }
        }

        if (iteration_results.inliers > results.inliers) {
            results = iteration_results;
        }
    }
    return results;
}