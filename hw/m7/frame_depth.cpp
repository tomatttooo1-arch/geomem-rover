#include "frame_depth.hpp"

#include <cfloat>
#include <cmath>

static bool finite_float(float value) {
    return value >= -FLT_MAX && value <= FLT_MAX;
}

static bool finite_double(double value) {
    return value >= -DBL_MAX && value <= DBL_MAX;
}

static std::uint32_t grid_offset(const std::int32_t cell[3],
                                 std::uint32_t nx, std::uint32_t ny) {
    return (static_cast<std::uint32_t>(cell[2]) * ny +
            static_cast<std::uint32_t>(cell[1])) * nx +
            static_cast<std::uint32_t>(cell[0]);
}

// Floor division by the Q8 scale. C++ '/' truncates toward zero, which would
// put a clipped endpoint in (-256, 0) into cell 0 instead of cell -1.
static std::int32_t floor_q8(std::int32_t value) {
    return value >= 0 ? value / FPGA_M7_FRAME_Q8
                      : -((-value + FPGA_M7_FRAME_Q8 - 1) / FPGA_M7_FRAME_Q8);
}

static bool cell_in_grid(const std::int32_t cell[3], const std::uint32_t dims[3]) {
    return cell[0] >= 0 && cell[0] < static_cast<std::int32_t>(dims[0]) &&
           cell[1] >= 0 && cell[1] < static_cast<std::int32_t>(dims[1]) &&
           cell[2] >= 0 && cell[2] < static_cast<std::int32_t>(dims[2]);
}

// origin is always inside the grid. end may lie outside it (a clipped ray,
// docs/design/M7_CLIP_SPEC_20261011.md): the walk then stops at the first
// cell outside the grid, every in-grid cell before it becomes FREE, and no
// HIT is written. For an in-grid end this is bit-identical to the 09-21 rule.
static void add_ray(const std::int32_t origin[3], const std::int32_t end[3],
                    const std::uint32_t dims[3],
                    std::uint8_t grid[FPGA_M7_FRAME_GRID_CAPACITY]) {
    std::int32_t cell[3], target[3], direction[3], next[3], delta[3];
    for (int axis = 0; axis < 3; ++axis) {
        const std::int32_t from = origin[axis];
        const std::int32_t to = end[axis];
        cell[axis] = from / FPGA_M7_FRAME_Q8;  // from >= 0: truncation == floor.
        target[axis] = floor_q8(to);
        direction[axis] = to > from ? 1 : (to < from ? -1 : 0);
        delta[axis] = to > from ? to - from : from - to;
        next[axis] = direction[axis] > 0
            ? (cell[axis] + 1) * FPGA_M7_FRAME_Q8 - from
            : from - cell[axis] * FPGA_M7_FRAME_Q8;
    }
    // Same half-open-cell and simultaneous-axis crossing rule as ray_flags.
    // Exact int64 cross products avoid floating-point tie decisions.
    // 3*32 steps suffice: each step moves >= 1 axis by one cell, monotonically,
    // so at most 31 moves per axis stay inside a 32-cell axis plus one step out.
    for (std::uint32_t step = 0; step < 3 * FPGA_M7_FRAME_GRID_AXIS; ++step) {
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
        if (first < 0) break;  // Bounded valid endpoints guarantee progress.
        const std::int32_t numerator = next[first];
        const std::int32_t denominator = delta[first];
        for (int axis = 0; axis < 3; ++axis) {
            if (direction[axis] == 0 || cell[axis] == target[axis]) continue;
            if (static_cast<std::int64_t>(next[axis]) * denominator ==
                static_cast<std::int64_t>(numerator) * delta[axis]) {
                cell[axis] += direction[axis];
                next[axis] += FPGA_M7_FRAME_Q8;
            }
        }
        // A segment meets the convex grid box in one interval: once out, the
        // walk never re-enters, so stopping here loses no in-grid cell.
        // This check also guards every grid write (HLS would wrap an index).
        if (!cell_in_grid(cell, dims)) break;
        if (cell[0] != target[0] || cell[1] != target[1] ||
            cell[2] != target[2]) {
            const std::uint32_t index = grid_offset(cell, dims[0], dims[1]);
            if (grid[index] == 0) grid[index] = 1;
        }
    }
    if (cell_in_grid(target, dims))
        grid[grid_offset(target, dims[0], dims[1])] = 2;
}

extern "C" void fpga_m7_depth_frame(
    const float depth_m[FPGA_M7_FRAME_MAX_PIXELS], const float inverse_k[9],
    const float world_camera[12], const double grid_origin_m[3],
    double voxel_size_m, std::uint32_t width, std::uint32_t height,
    std::uint32_t nx, std::uint32_t ny, std::uint32_t nz,
    std::uint8_t flags[FPGA_M7_FRAME_GRID_CAPACITY], std::uint32_t counts[3],
    std::uint32_t* status) {
    counts[0] = counts[1] = counts[2] = 0;
    *status = 1;
    if ((width == 0) != (height == 0) || width > FPGA_M7_FRAME_MAX_WIDTH ||
        height > FPGA_M7_FRAME_MAX_HEIGHT || nx == 0 || ny == 0 || nz == 0 ||
        nx > FPGA_M7_FRAME_GRID_AXIS || ny > FPGA_M7_FRAME_GRID_AXIS ||
        nz > FPGA_M7_FRAME_GRID_AXIS) return;
    *status = 2;
    if (!finite_double(voxel_size_m) || voxel_size_m <= 0) return;
    for (int axis = 0; axis < 3; ++axis)
        if (!finite_double(grid_origin_m[axis])) return;
    for (int i = 0; i < 9; ++i)
        if (!finite_float(inverse_k[i])) return;
    for (int i = 0; i < 12; ++i)
        if (!finite_float(world_camera[i])) return;

    const std::uint32_t dims[3] = {nx, ny, nz};
    std::int32_t origin[3];
    *status = 3;
    for (int axis = 0; axis < 3; ++axis) {
        const double coordinate = std::floor(
            (static_cast<double>(world_camera[4*axis+3]) - grid_origin_m[axis]) /
            voxel_size_m * FPGA_M7_FRAME_Q8);
        if (!finite_double(coordinate) || coordinate < 0 ||
            coordinate >= dims[axis] * FPGA_M7_FRAME_Q8) return;
        origin[axis] = static_cast<std::int32_t>(coordinate);
    }

    // This is the only per-frame accumulator. It is explicitly reset before
    // every successful frame; no 4096-ray tile output is externally merged.
    std::uint8_t grid[FPGA_M7_FRAME_GRID_CAPACITY];
#ifdef __SYNTHESIS__
#pragma HLS BIND_STORAGE variable=grid type=ram_2p impl=bram
#endif
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        grid[i] = 0;

    const std::uint32_t pixels = width * height;
    std::uint32_t accepted = 0, invalid = 0, clipped = 0;
    for (std::uint32_t pixel = 0; pixel < FPGA_M7_FRAME_MAX_PIXELS; ++pixel) {
        if (pixel >= pixels) break;
        const float z = depth_m[pixel];
        if (!(z > 0.1f && z < 10.0f)) {
            ++invalid;
            continue;
        }
        const float u = static_cast<float>(pixel % width);
        const float v = static_cast<float>(pixel / width);
        float camera[3], world[3];
        for (int row = 0; row < 3; ++row)
            camera[row] = (inverse_k[3*row]*u + inverse_k[3*row+1]*v +
                           inverse_k[3*row+2]) * z;
        bool good = true;
        for (int row = 0; row < 3; ++row) {
            world[row] = world_camera[4*row]*camera[0] +
                         world_camera[4*row+1]*camera[1] +
                         world_camera[4*row+2]*camera[2] +
                         world_camera[4*row+3];
            good = good && finite_float(world[row]);
        }
        if (!good) {
            ++invalid;
            continue;
        }
        std::int32_t end[3];
        bool inside = true, representable = true;
        for (int axis = 0; axis < 3; ++axis) {
            const double coordinate = std::floor(
                (static_cast<double>(world[axis]) - grid_origin_m[axis]) /
                voxel_size_m * FPGA_M7_FRAME_Q8);
            // Range gate (M7_CLIP_SPEC): a clipped end is still converted to
            // int32, so keep |Q8| < 2^24; the int64 crossing products then
            // stay below 2^39. NaN fails both comparisons.
            if (!(coordinate >= -FPGA_M7_FRAME_Q8_LIMIT &&
                  coordinate < FPGA_M7_FRAME_Q8_LIMIT)) {
                representable = false;
                end[axis] = 0;
                continue;
            }
            end[axis] = static_cast<std::int32_t>(coordinate);
            if (coordinate < 0 || coordinate >= dims[axis] * FPGA_M7_FRAME_Q8)
                inside = false;
        }
        if (!representable) {
            ++invalid;
            continue;
        }
        add_ray(origin, end, dims, grid);
        if (inside)
            ++accepted;
        else
            ++clipped;
    }
    for (std::uint32_t i = 0; i < FPGA_M7_FRAME_GRID_CAPACITY; ++i)
        flags[i] = grid[i];
    counts[0] = accepted;
    counts[1] = invalid;
    counts[2] = clipped;
    *status = 0;
}
