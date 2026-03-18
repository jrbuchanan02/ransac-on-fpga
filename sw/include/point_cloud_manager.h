#ifndef POINT_CLOUD_MANAGER_H
#define POINT_CLOUD_MANAGER_H

// manages buffers for reading in the point clouds and setting
// them up in fixed point.
//
// data is received from uart in a csv format with each point being described as
// a 32-bit hex string. The length is known from a preprocessor define (source 
// code can be swapped out relatively quickly)

#include "stddef.h"
#include "stdint.h"

void point_cloud_manager_init(void);

inline void point_cloud_manager_reset(void) {
    point_cloud_manager_init();
}

// assumes uart is already initialized.
void read_in_from_uart(void);

uint32_t const *get_cloud_base_address(void);
uintptr_t get_cloud_length(void);

#endif // ifndef POINT_CLOUD_MANAGER_H