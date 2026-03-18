#ifndef SW_RANSAC_H
#define SW_RANSAC_H

#include "stdint.h"

struct sw_ransac_results {
    uint32_t inliers;
    uint32_t nx;
    uint32_t ny;
    uint32_t nz;
    uint32_t d;
};

struct sw_ransac_results sw_ransac_run(uint32_t const *cloud, uint32_t size, uint32_t iterations, uint32_t threshold);

#endif // ifndef SW_RANSAC_H