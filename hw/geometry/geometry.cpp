#include "geometry.hpp"
#include <cfloat>

static bool finite_float(float v) {
    return v >= -FLT_MAX && v <= FLT_MAX;
}

static bool valid_depth(float z) {
    // Comparisons also reject NaN and both infinities, without fast-math.
    return z > 0.1f && z < 10.0f;
}

extern "C" void fpga_backproject(
    const float uvz[3 * FPGA_GEOMETRY_MAX_SAMPLES], const float inverse_k[9],
    const float world_camera[12], std::uint32_t count,
    float xyz[3 * FPGA_GEOMETRY_MAX_SAMPLES],
    std::uint32_t valid[FPGA_GEOMETRY_MAX_SAMPLES], std::uint32_t* status) {
    *status = count > FPGA_GEOMETRY_MAX_SAMPLES ? 1 : 0;
    if (*status) return;
    for (std::uint32_t i = 0; i < count; ++i) {
        const float u = uvz[3*i], v = uvz[3*i+1], z = uvz[3*i+2];
        float out[3] = {0, 0, 0};
        bool good = valid_depth(z) && finite_float(u) && finite_float(v);
        if (good) {
            float camera[3];
            for (int row = 0; row < 3; ++row)
                camera[row] = (inverse_k[3*row]*u + inverse_k[3*row+1]*v +
                               inverse_k[3*row+2]) * z;
            for (int row = 0; row < 3; ++row) {
                out[row] = world_camera[4*row]*camera[0] +
                           world_camera[4*row+1]*camera[1] +
                           world_camera[4*row+2]*camera[2] + world_camera[4*row+3];
                good = good && finite_float(out[row]);
            }
        }
        valid[i] = good ? 1 : 0;
        for (int row = 0; row < 3; ++row) xyz[3*i+row] = good ? out[row] : 0;
    }
}

extern "C" void fpga_visibility_counts(
    const float depth[FPGA_GEOMETRY_MAX_SAMPLES], std::uint32_t count,
    float front, float back, std::uint32_t counts[4], std::uint32_t* status) {
    for (int j = 0; j < 4; ++j) counts[j] = 0;
    *status = count > FPGA_GEOMETRY_MAX_SAMPLES ? 1 : 0;
    if (*status) return;
    if (!finite_float(front) || !finite_float(back) || front > back) {
        *status = 2;
        return;
    }
    std::uint32_t good = 0, occluded = 0, free = 0;
    for (std::uint32_t i = 0; i < count; ++i) {
        const float z = depth[i];
        if (valid_depth(z)) {
            ++good;
            if (z < front) ++occluded;
            if (z > back) ++free;
        }
    }
    counts[0] = count; counts[1] = good; counts[2] = occluded; counts[3] = free;
}
