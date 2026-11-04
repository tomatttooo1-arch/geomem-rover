#include "ray_flags.hpp"

static std::uint32_t offset(const std::int32_t cell[3],
                            std::uint32_t nx, std::uint32_t ny) {
    return (static_cast<std::uint32_t>(cell[2]) * ny +
            static_cast<std::uint32_t>(cell[1])) * nx +
            static_cast<std::uint32_t>(cell[0]);
}

extern "C" void fpga_m7_ray_flags(
    const std::int32_t origins[3 * FPGA_M7_RAY_CAPACITY],
    const std::int32_t ends[3 * FPGA_M7_RAY_CAPACITY],
    const std::uint32_t ray_valid[FPGA_M7_RAY_CAPACITY],
    std::uint32_t count, std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
    std::uint8_t flags[FPGA_M7_GRID_CAPACITY], std::uint32_t* status) {
    *status = 1;
    if (count > FPGA_M7_RAY_CAPACITY || nx == 0 || ny == 0 || nz == 0 ||
        nx > FPGA_M7_AXIS_CAPACITY || ny > FPGA_M7_AXIS_CAPACITY ||
        nz > FPGA_M7_AXIS_CAPACITY) return;

    const std::uint32_t dims[3] = {nx, ny, nz};
    // Validate the entire batch before any output write. This lets consumers
    // discard a failed frame atomically, even if the last ray is malformed.
    for (std::uint32_t i = 0; i < count; ++i) {
        if (ray_valid[i] > 1) {
            *status = 2;
            return;
        }
        if (ray_valid[i] == 0) continue;
        for (int axis = 0; axis < 3; ++axis) {
            const std::int32_t extent = static_cast<std::int32_t>(dims[axis] * 256);
            const std::int32_t start = origins[3*i+axis];
            const std::int32_t end = ends[3*i+axis];
            if (start < 0 || start >= extent || end < 0 || end >= extent) {
                *status = 3;
                return;
            }
        }
    }

    const std::uint32_t grid_size = nx * ny * nz;
    for (std::uint32_t cell = 0; cell < grid_size; ++cell) flags[cell] = 0;

    for (std::uint32_t i = 0; i < count; ++i) {
        if (ray_valid[i] == 0) continue;
        std::int32_t cell[3], target[3], direction[3], next[3], delta[3];
        for (int axis = 0; axis < 3; ++axis) {
            const std::int32_t from = origins[3*i+axis];
            const std::int32_t to = ends[3*i+axis];
            cell[axis] = from / FPGA_M7_UNITS_PER_VOXEL;
            target[axis] = to / FPGA_M7_UNITS_PER_VOXEL;
            direction[axis] = (to > from) ? 1 : (to < from ? -1 : 0);
            delta[axis] = (to > from) ? to - from : from - to;
            next[axis] = direction[axis] > 0
                ? (cell[axis] + 1) * FPGA_M7_UNITS_PER_VOXEL - from
                : from - cell[axis] * FPGA_M7_UNITS_PER_VOXEL;
        }

        // A monotone ray crosses at most 31 boundaries per axis. Fractions
        // next[a]/delta[a] are compared exactly by int64 cross products.
        // When two boundaries share a time, step BOTH axes: edge/corner-only
        // contacts are not observed free space.
        for (int step = 0; step < 3 * 32; ++step) {
            if (cell[0] == target[0] && cell[1] == target[1] &&
                cell[2] == target[2]) break;
            int first = -1;
            for (int axis = 0; axis < 3; ++axis) {
                if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
                if (first < 0 ||
                    static_cast<std::int64_t>(next[axis]) * delta[first] <
                    static_cast<std::int64_t>(next[first]) * delta[axis])
                    first = axis;
            }
            // Checked endpoints and monotone motion guarantee a next axis.
            if (first < 0) break;
            const std::int32_t threshold = next[first];
            const std::int32_t denominator = delta[first];
            for (int axis = 0; axis < 3; ++axis) {
                if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
                if (static_cast<std::int64_t>(next[axis]) * denominator ==
                    static_cast<std::int64_t>(threshold) * delta[axis]) {
                    cell[axis] += direction[axis];
                    next[axis] += FPGA_M7_UNITS_PER_VOXEL;
                }
            }
            if (cell[0] != target[0] || cell[1] != target[1] ||
                cell[2] != target[2]) {
                const std::uint32_t index = offset(cell, nx, ny);
                if (flags[index] == 0) flags[index] = 1;
            }
        }
        flags[offset(target, nx, ny)] = 2;
    }
    *status = 0;
}
