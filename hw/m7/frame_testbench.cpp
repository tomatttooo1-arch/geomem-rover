#include "frame_depth.hpp"

#include <iostream>
#include <limits>

static float depth[FPGA_M7_FRAME_MAX_PIXELS] = {};
static std::uint8_t flags[FPGA_M7_FRAME_GRID_CAPACITY] = {};
static std::uint32_t counts[3] = {};

static std::uint32_t at(int x, int y, int z) {
    return static_cast<std::uint32_t>((z * 4 + y) * 4 + x);
}

int main() {
    const float inverse_k[9] = {.000001f,0,0, 0,.000001f,0, 0,0,1};
    float world_camera[12] = {1,0,0,.5f, 0,1,0,.5f, 0,0,1,.5f};
    const double grid_origin[3] = {0,0,0};
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    std::uint32_t status = 99;

    depth[0] = 3;
    depth[1] = 1;
    depth[2] = invalid;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 1.,
                       3, 1, 4, 4, 4, flags, counts, &status);
    if (status || counts[0] != 2 || counts[1] != 1 || counts[2] != 0 ||
        flags[at(0,0,0)] != 0 || flags[at(0,0,1)] != 2 ||
        flags[at(0,0,2)] != 1 || flags[at(0,0,3)] != 2) return 1;

    // The traversal steps EVERY axis whose next boundary is reached at the
    // same instant. This ray leaves the centre of (0,0,0) for the centre of
    // (3,3,3) and reaches all three planes together, so under the half-open
    // voxel convention only the diagonal cells are observed free space. A
    // classic single-axis DDA walks six extra face neighbours; no other case
    // in this file, in the dense probe matrix, or on the Python side places
    // a ray exactly on a voxel edge, so nothing else can see that difference.
    const float tie_inverse_k[9] = {0,0,1, 0,0,1, 0,0,1};
    fpga_m7_depth_frame(depth, tie_inverse_k, world_camera, grid_origin, 1.,
                       1, 1, 4, 4, 4, flags, counts, &status);
    if (status || counts[0] != 1 || counts[1] != 0 || counts[2] != 0) return 10;
    if (flags[at(1,1,1)] != 1 || flags[at(2,2,2)] != 1 ||
        flags[at(3,3,3)] != 2) return 11;
    std::uint32_t tie_marked = 0;
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        if (flags[i] != 0) ++tie_marked;
    if (tie_marked != 3) return 12;  // Exactly the two free plus the one hit.

    // An explicitly empty frame must clear all 32^3 internal/output cells.
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 1.,
                       0, 0, 4, 4, 4, flags, counts, &status);
    if (status || counts[0] || counts[1] || counts[2]) return 2;
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        if (flags[i] != 0) return 3;

    // Both sides of old 4096-ray chunk boundary belong to ONE new frame.
    // All other pixels are invalid so co-simulation tests frame span and
    // conflict order without pretending to measure dense-frame throughput.
    for (std::uint32_t i = 0; i < 128 * 64; ++i) depth[i] = invalid;
    depth[0] = 3;
    depth[4096] = 1;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 1.,
                       128, 64, 4, 4, 4, flags, counts, &status);
    if (status || counts[0] != 2 || counts[1] != 8190 || counts[2] ||
        flags[at(0,0,1)] != 2 || flags[at(0,0,2)] != 1 ||
        flags[at(0,0,3)] != 2) return 4;
    depth[0] = 1;
    depth[4096] = 3;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 1.,
                       128, 64, 4, 4, 4, flags, counts, &status);
    if (status || counts[0] != 2 || counts[1] != 8190 || counts[2] ||
        flags[at(0,0,1)] != 2 || flags[at(0,0,2)] != 1 ||
        flags[at(0,0,3)] != 2) return 5;

    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        flags[i] = 99;
    counts[0] = counts[1] = counts[2] = 99;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 1.,
                       1281, 1, 4, 4, 4, flags, counts, &status);
    if (status != 1 || counts[0] || counts[1] || counts[2]) return 6;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 0.,
                       1, 1, 4, 4, 4, flags, counts, &status);
    if (status != 2 || counts[0] || counts[1] || counts[2]) return 7;
    world_camera[3] = 5.;
    fpga_m7_depth_frame(depth, inverse_k, world_camera, grid_origin, 1.,
                       1, 1, 4, 4, 4, flags, counts, &status);
    if (status != 3 || counts[0] || counts[1] || counts[2]) return 8;
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        if (flags[i] != 99) return 9;
    std::cout << "M7 depth frame C++ testbench PASS\n";
    return 0;
}
