#ifndef RANSAC_DRIVER_H
#define RANSAC_DRIVER_H

#include "stdbool.h"
#include "stdint.h"

void hard_ransac_set_cloud_base(uint32_t const *const);
void hard_ransac_set_cloud_length(uintptr_t const);
void hard_ransac_set_inlier_threshold(uint32_t const);
void hard_ransac_set_iteration_count(uint32_t const);

void hard_ransac_start_calculation(void);
void hard_ransac_abort_calculation(void);

bool hard_ransac_calculation_active(void);
bool hard_ransac_calculation_aborting(void);
bool hard_ransac_calculation_finalizing(void);

uint32_t hard_ransac_processed_iteration_count(void);
uint32_t hard_ransac_ground_plane_inliers(void);
uint32_t hard_ransac_ground_plane_normalx(void);
uint32_t hard_ransac_ground_plane_normaly(void);
uint32_t hard_ransac_ground_plane_normalz(void);
uint32_t hard_ransac_ground_plane_distance(void);

#endif // ifndef RANSAC_DRIVER_H