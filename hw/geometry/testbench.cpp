#include "geometry.hpp"
#include <cmath>
#include <iostream>
#include <limits>

int main() {
    float uvz[3*FPGA_GEOMETRY_MAX_SAMPLES] = {};
    float xyz[3*FPGA_GEOMETRY_MAX_SAMPLES] = {};
    std::uint32_t valid[FPGA_GEOMETRY_MAX_SAMPLES] = {};
    float depth[FPGA_GEOMETRY_MAX_SAMPLES] = {};
    const float ik[9] = {.01f, 0, -.5f, 0, .01f, -.5f, 0, 0, 1};
    const float transform[12] = {0, -1, 0, 1, 1, 0, 0, 2, 0, 0, 1, 3};
    std::uint32_t status = 99, counts[4];
    uvz[0] = 100; uvz[1] = 50; uvz[2] = 2;
    uvz[5] = std::numeric_limits<float>::quiet_NaN();
    fpga_backproject(uvz, ik, transform, 2, xyz, valid, &status);
    if (status || valid[0] != 1 || valid[1] != 0 ||
        std::fabs(xyz[0]-1) > 1e-6 || std::fabs(xyz[1]-3) > 1e-6 ||
        std::fabs(xyz[2]-5) > 1e-6 || xyz[3] || xyz[4] || xyz[5]) return 1;
    uvz[2] = 0.1f;
    fpga_backproject(uvz, ik, transform, 1, xyz, valid, &status);
    if (status || valid[0] || xyz[0] || xyz[1] || xyz[2]) return 2;
    // Strict equality boundaries, opposing evidence, and invalid samples.
    depth[0] = 1; depth[1] = 2; depth[2] = 3; depth[3] = 4;
    depth[4] = 0.1f; depth[5] = 10; depth[6] = uvz[5];
    depth[7] = std::numeric_limits<float>::infinity();
    fpga_visibility_counts(depth, 8, 2, 3, counts, &status);
    if (status || counts[0] != 8 || counts[1] != 4 ||
        counts[2] != 1 || counts[3] != 1) return 3;
    fpga_visibility_counts(depth, 0, 2, 3, counts, &status);
    if (status || counts[0] || counts[1] || counts[2] || counts[3]) return 4;
    std::cout << "geometry C++ testbench PASS\n";
    return 0;
}
