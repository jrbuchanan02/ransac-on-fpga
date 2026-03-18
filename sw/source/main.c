
#include <stdint.h>

#include "hw_ransac.h"
#include "point_cloud_manager.h"
#include "sw_ransac.h"
#include "text_utils.h"
#include "uart.h"

void main(void) {
    uint32_t const intended_iterations = get_cloud_length() / 10;
    uart_init();
    uart_puts("\x1b[3JInitializing...");
    point_cloud_manager_init();
    uart_puts("\x1b[3J\x1b[;HAwaiting Input Data:\n"); // row 1
    read_in_from_uart();
    uart_puts("Input Data Processed.\n"); // row 2
    hard_ransac_set_cloud_base(get_cloud_base_address());
    uart_puts("Cloud base address set to: "); // row 3
    uart_puts(u32_to_hex_string((uint32_t)get_cloud_base_address()).string);
    uart_puts("\n");
    hard_ransac_set_cloud_length(get_cloud_length());
    uart_puts("Cloud length set to: "); // row 4
    uart_puts(u32_to_hex_string(get_cloud_length()).string);
    uart_puts("\n");
    hard_ransac_set_inlier_threshold(1024);  // 2 ^ -10 or 1 / 1024
    uart_puts("Inlier threshold set to: "); // row 5
    uart_puts(u32_to_hex_string(1024).string);
    uart_puts(" / ");
    uart_puts(u32_to_hex_string(1 << 20).string);
    uart_puts("\n");
    hard_ransac_set_iteration_count(intended_iterations);
    uart_puts("Requesting "); // row 6
    uart_puts(u32_to_hex_string(intended_iterations).string);
    uart_puts(" iterations.\n");
    while (!hard_ransac_calculation_active()) {
        hard_ransac_start_calculation();
        uart_puts("Attempting to start calculations...\r");
    }
    while (hard_ransac_calculation_active() || hard_ransac_calculation_finalizing()) {
        uart_puts("\rOn Iteration: ");
        uart_puts(u32_to_hex_string(hard_ransac_processed_iteration_count()).string);
    }
    uart_puts("\n");

    if (hard_ransac_processed_iteration_count() != intended_iterations) {
        uart_puts("ERROR: mismatch between processed iteration count and intended iteration count!\n");
    }

    uart_puts("Inlier Count: ");
    uart_puts(u32_to_hex_string(hard_ransac_ground_plane_inliers()).string);
    uart_puts("\n");
    uart_puts("Normal X: ");
    uart_puts(u32_to_hex_string(hard_ransac_ground_plane_normalx()).string);
    uart_puts("\n");

    uart_puts("Normal Y: ");
    uart_puts(u32_to_hex_string(hard_ransac_ground_plane_normaly()).string);
    uart_puts("\n");

    uart_puts("Normal Z: ");
    uart_puts(u32_to_hex_string(hard_ransac_ground_plane_normalz()).string);
    uart_puts("\n");

    uart_puts("Scaled Distance: ");
    uart_puts(u32_to_hex_string(hard_ransac_ground_plane_distance()).string);
    uart_puts("\n");

    uart_puts("Running SW version...\n");
    struct sw_ransac_results results = sw_ransac_run(get_cloud_base_address(), get_cloud_length(), intended_iterations, 1024);
    uart_puts("Inlier Count: ");
    uart_puts(u32_to_hex_string(results.inliers).string);
    uart_puts("\n");
    uart_puts("Normal X: ");
    uart_puts(u32_to_hex_string(results.nx).string);
    uart_puts("\n");
    uart_puts("Normal Y: ");
    uart_puts(u32_to_hex_string(results.ny).string);
    uart_puts("\n");
    uart_puts("Normal Z: ");
    uart_puts(u32_to_hex_string(results.nz).string);
    uart_puts("\n");
    uart_puts("Scaled Distance: ");
    uart_puts(u32_to_hex_string(results.d).string);
    uart_puts("\n");


    while (1);
}