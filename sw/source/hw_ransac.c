#include "hw_ransac.h"

#include "stdbool.h"

#include "stdint.h"

struct ransac_unit_ports {
    uint32_t volatile calculation_start_strobe; // 40000000
    uint32_t volatile calculation_abort_strobe; // 40000004
    uint32_t const volatile calculation_state;  // 40000008
    uint32_t const *volatile cloud_base;        // 4000000C
    uint32_t volatile cloud_size;               // 40000010
    uint32_t volatile iterations;               // 40000014
    uint32_t const volatile processed_iterations;   // 40000018
    uint32_t volatile inlier_threshold;         // 4000001C
    uint32_t const volatile inlier_count;       // 40000020
    uint32_t const volatile normalx;            // 40000024
    uint32_t const volatile normaly;            // 40000028
    uint32_t const volatile normalz;            // 4000002C
    uint32_t const volatile distance;           // 40000030
};

struct ransac_unit_ports volatile *const ransac_unit = (struct ransac_unit_ports volatile *)0x40000000;

void hard_ransac_set_cloud_base(uint32_t const *const cloud_base) {
    ransac_unit->cloud_base = cloud_base;
}
void hard_ransac_set_cloud_length(uintptr_t const cloud_length) {
    ransac_unit->cloud_size = cloud_length;
}
void hard_ransac_set_inlier_threshold(uint32_t const threshold) {
    ransac_unit->inlier_threshold = threshold;
}
void hard_ransac_set_iteration_count(uint32_t const iterations) {
    ransac_unit->iterations = iterations;
}

void hard_ransac_start_calculation(void) {
    ransac_unit->calculation_start_strobe = 0;
}
void hard_ransac_abort_calculation(void) {
    ransac_unit->calculation_abort_strobe = 0;
}

bool hard_ransac_calculation_active(void) {
    uint32_t state = ransac_unit->calculation_state;
    if (state & 1) {
        return true;
    } else {
        return false;
    }
}
bool hard_ransac_calculation_aborting(void) {
    uint32_t state = ransac_unit->calculation_state;
    if (state & 2) {
        return true;
    } else {
        return false;
    }
}
bool hard_ransac_calculation_finalizing(void) {
    uint32_t state = ransac_unit->calculation_state;
    if (state & 4) {
        return true;
    } else {
        return false;
    }
}

uint32_t hard_ransac_processed_iteration_count(void) {
    return ransac_unit->processed_iterations;
}
uint32_t hard_ransac_ground_plane_inliers(void) {
    return ransac_unit->inlier_count;
}
uint32_t hard_ransac_ground_plane_normalx(void) {
    return ransac_unit->normalx;
}
uint32_t hard_ransac_ground_plane_normaly(void) {
    return ransac_unit->normaly;
}
uint32_t hard_ransac_ground_plane_normalz(void) {
    return ransac_unit->normalz;
}
uint32_t hard_ransac_ground_plane_distance(void) {
    return ransac_unit->distance;
}