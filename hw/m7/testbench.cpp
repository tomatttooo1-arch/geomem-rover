#include "ray_flags.hpp"
#include <iostream>

static std::int32_t origins[3 * FPGA_M7_RAY_CAPACITY] = {};
static std::int32_t ends[3 * FPGA_M7_RAY_CAPACITY] = {};
static std::uint32_t valid[FPGA_M7_RAY_CAPACITY] = {};
static std::uint8_t flags[FPGA_M7_GRID_CAPACITY] = {};

static std::uint32_t at(int x, int y, int z) {
    return static_cast<std::uint32_t>((z * 4 + y) * 4 + x);
}
static std::uint32_t at_max(int x, int y, int z) {
    return static_cast<std::uint32_t>((z * 32 + y) * 32 + x);
}

int main() {
    std::uint32_t status = 99;
    valid[0] = 1;
    origins[0] = 128; origins[1] = 128; origins[2] = 128;
    ends[0] = 3*256+128; ends[1] = 128; ends[2] = 128;
    fpga_m7_ray_flags(origins, ends, valid, 1, 4, 4, 4, flags, &status);
    if (status || flags[at(0,0,0)] != 0 || flags[at(1,0,0)] != 1 ||
        flags[at(2,0,0)] != 1 || flags[at(3,0,0)] != 2) return 1;

    // A later endpoint overrides the first ray's free vote.
    valid[1] = 1;
    origins[3] = 128; origins[4] = 128; origins[5] = 128;
    ends[3] = 256+128; ends[4] = 128; ends[5] = 128;
    // Separate diagonal: cells touched only at the crossing stay unknown.
    valid[2] = 1;
    origins[6] = 128; origins[7] = 128; origins[8] = 128;
    ends[6] = 256+128; ends[7] = 256+128; ends[8] = 128;
    fpga_m7_ray_flags(origins, ends, valid, 3, 4, 4, 4, flags, &status);
    if (status || flags[at(1,0,0)] != 2 || flags[at(0,1,0)] != 0 ||
        flags[at(1,1,0)] != 2 || flags[at(3,0,0)] != 2) return 2;

    valid[1] = valid[2] = 0;
    origins[0] = 3*256+128;
    ends[0] = 128;
    fpga_m7_ray_flags(origins, ends, valid, 1, 4, 4, 4, flags, &status);
    if (status || flags[at(3,0,0)] != 0 || flags[at(2,0,0)] != 1 ||
        flags[at(1,0,0)] != 1 || flags[at(0,0,0)] != 2) return 3;
    // Starting exactly on a boundary and moving negative crosses at t=0;
    // the origin's half-open cell (x=2) stays unknown.
    origins[0] = 2*256;
    fpga_m7_ray_flags(origins, ends, valid, 1, 4, 4, 4, flags, &status);
    if (status || flags[at(2,0,0)] != 0 || flags[at(1,0,0)] != 1 ||
        flags[at(0,0,0)] != 2) return 4;

    for (std::uint32_t i = 0; i < FPGA_M7_GRID_CAPACITY; ++i) flags[i] = 99;
    valid[1] = 1;
    ends[3] = 4 * 256;  // The final enabled ray invalidates the entire batch.
    fpga_m7_ray_flags(origins, ends, valid, 2, 4, 4, 4, flags, &status);
    if (status != 3) return 5;
    for (std::uint32_t i = 0; i < FPGA_M7_GRID_CAPACITY; ++i)
        if (flags[i] != 99) return 6;

    // Both kinds of malformed input reject an entire frame without clearing
    // any output. The oversized count must be checked BEFORE indexing rays.
    valid[0] = 2;
    fpga_m7_ray_flags(origins, ends, valid, 1, 4, 4, 4, flags, &status);
    if (status != 2) return 7;
    fpga_m7_ray_flags(origins, ends, valid, FPGA_M7_RAY_CAPACITY + 1,
                     4, 4, 4, flags, &status);
    if (status != 1) return 8;
    for (std::uint32_t i = 0; i < FPGA_M7_GRID_CAPACITY; ++i)
        if (flags[i] != 99) return 9;

    valid[0] = valid[1] = 0;
    fpga_m7_ray_flags(origins, ends, valid, 0, 4, 4, 4, flags, &status);
    if (status || flags[at(0,0,0)] != 0 || flags[at(3,3,3)] != 0 ||
        flags[FPGA_M7_GRID_CAPACITY - 1] != 99) return 10;

    // Last of 4096 inputs, highest cell index of 32^3 output, and a long
    // 3-axis ray with unequal slopes (crosses boundaries at distinct times).
    // The other rays are disabled. This checks the interface's actual upper
    // bounds without claiming that this tile represents a complete image.
    valid[FPGA_M7_RAY_CAPACITY - 1] = 1;
    const std::uint32_t last = 3 * (FPGA_M7_RAY_CAPACITY - 1);
    origins[last] = origins[last+1] = origins[last+2] = 128;
    ends[last] = 31*256+129;
    ends[last+1] = 31*256+127;
    ends[last+2] = 31*256+125;
    fpga_m7_ray_flags(origins, ends, valid, FPGA_M7_RAY_CAPACITY,
                     32, 32, 32, flags, &status);
    if (status || flags[at_max(0,0,0)] != 0 ||
        flags[at_max(1,0,0)] != 1 || flags[at_max(0,1,0)] != 0 ||
        flags[at_max(31,31,31)] != 2) return 11;
    valid[FPGA_M7_RAY_CAPACITY - 1] = 0;
    fpga_m7_ray_flags(origins, ends, valid, 0, 32, 32, 32, flags, &status);
    if (status) return 12;
    for (std::uint32_t i = 0; i < FPGA_M7_GRID_CAPACITY; ++i)
        if (flags[i] != 0) return 13;
    std::cout << "M7 ray flags C++ testbench PASS\n";
    return 0;
}
