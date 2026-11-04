#ifndef FPGA_GEOMETRY_HPP
#define FPGA_GEOMETRY_HPP

#include <cstdint>

// A bounded tile, not an image-size limit. Host code owns tiling and frame IDs.
constexpr std::uint32_t FPGA_GEOMETRY_MAX_SAMPLES = 4096;

extern "C" {
// Row-major uvz triples, inverse K (3x3), camera-to-world [R|t] (3x4).
// Inputs must be non-aliased arrays with the declared capacity. Calibration,
// pose trust, alignment and timestamps remain host validation responsibilities.
// status: 0 success; 1 count exceeds tile capacity. Invalid points are zeroed
// and valid[i] = 0; consumers MUST use valid rather than testing xyz == 0.
void fpga_backproject(const float uvz[3 * FPGA_GEOMETRY_MAX_SAMPLES],
                     const float inverse_k[9], const float world_camera[12],
                     std::uint32_t count,
                     float xyz[3 * FPGA_GEOMETRY_MAX_SAMPLES],
                     std::uint32_t valid[FPGA_GEOMETRY_MAX_SAMPLES],
                     std::uint32_t* status);

// Counts over a host-projected ROI: total, valid, in front, behind.
// Thresholds ALREADY include the Python reference's depth margin.
// Comparisons are strict: 0.1 < z < 10, z < front, z > back.
// status: 0 success; 1 oversized count; 2 non-finite/inverted thresholds.
// Failure clears all counts. State decisions and valid fractions stay in Python.
void fpga_visibility_counts(const float depth[FPGA_GEOMETRY_MAX_SAMPLES],
                           std::uint32_t count, float front, float back,
                           std::uint32_t counts[4], std::uint32_t* status);
}
#endif
