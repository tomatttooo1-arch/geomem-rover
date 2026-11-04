// Plain CPU reference for the M7 rolling map (see rolling_map_ref.hpp).
#include "rolling_map_ref.hpp"

#include <cstring>
#include <new>

namespace {

const std::int32_t DIM[3] = {static_cast<std::int32_t>(FPGA_M7_ROLL_DIM_X),
                             static_cast<std::int32_t>(FPGA_M7_ROLL_DIM_Y),
                             static_cast<std::int32_t>(FPGA_M7_ROLL_DIM_Z)};

struct RefMap {
    std::int8_t cells[FPGA_M7_ROLL_CELLS];     // local order
    std::int8_t scratch[FPGA_M7_ROLL_CELLS];   // used while moving the window
    std::uint8_t observed[FPGA_M7_ROLL_CELLS]; // mutant decay bookkeeping
    bool initialized;
    bool has_frame;
    std::uint32_t epoch, delta_seq, last_frame;
    std::int32_t base[3];
    std::uint32_t accepted, rej_epoch, rej_frame, rej_malformed;
};

std::int64_t local_index(const std::int32_t l[3]) {
    return (static_cast<std::int64_t>(l[2]) * DIM[1] + l[1]) * DIM[0] + l[0];
}

bool inside(const std::int64_t l[3]) {
    for (int a = 0; a < 3; ++a)
        if (l[a] < 0 || l[a] >= DIM[a]) return false;
    return true;
}

bool coord_ok(std::int32_t v) {
    return v > -FPGA_M7_ROLL_COORD_LIMIT && v < FPGA_M7_ROLL_COORD_LIMIT;
}

#ifdef FPGA_M7_ROLL_MUTANT_NO_EVICT
std::int64_t floor_mod(std::int64_t v, std::int64_t m) {
    std::int64_t r = v % m;
    return r < 0 ? r + m : r;
}
#endif

void fill_header(const RefMap& s, std::uint32_t op, std::uint32_t status,
                 std::uint32_t n_changes, std::uint32_t changed_cells,
                 std::uint32_t outside, std::uint32_t inside_count,
                 std::uint32_t shift,
                 std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    for (std::uint32_t i = 0; i < FPGA_M7_ROLL_HEADER_WORDS; ++i) header[i] = 0;
    header[FPGA_M7_ROLL_H_STATUS] = status;
    header[FPGA_M7_ROLL_H_OP] = op;
    header[FPGA_M7_ROLL_H_EPOCH] = s.epoch;
    header[FPGA_M7_ROLL_H_DELTA_SEQ] = s.delta_seq;
    header[FPGA_M7_ROLL_H_LAST_FRAME] = s.last_frame;
    for (int a = 0; a < 3; ++a)
        header[FPGA_M7_ROLL_H_BASE + a] = static_cast<std::uint32_t>(s.base[a]);
    // No ring in the reference: offsets are reported as zero.
    header[FPGA_M7_ROLL_H_N_DELTA] = n_changes;
    header[FPGA_M7_ROLL_H_CHANGED_CELLS] = changed_cells;
    header[FPGA_M7_ROLL_H_OBS_OUTSIDE] = outside;
    header[FPGA_M7_ROLL_H_OBS_INSIDE] = inside_count;
    header[FPGA_M7_ROLL_H_SHIFT] = shift;
    header[FPGA_M7_ROLL_H_ACCEPTED] = s.accepted;
    header[FPGA_M7_ROLL_H_REJ_EPOCH] = s.rej_epoch;
    header[FPGA_M7_ROLL_H_REJ_FRAME] = s.rej_frame;
    header[FPGA_M7_ROLL_H_REJ_MALFORMED] = s.rej_malformed;
    header[FPGA_M7_ROLL_H_MAGIC] = FPGA_M7_ROLL_MAGIC;
    header[FPGA_M7_ROLL_H_DIMS] = FPGA_M7_ROLL_DIM_X | (FPGA_M7_ROLL_DIM_Y << 8) |
                                 (FPGA_M7_ROLL_DIM_Z << 16);
    header[FPGA_M7_ROLL_H_HAS_FRAME] = (s.initialized ? 2u : 0u) | (s.has_frame ? 1u : 0u);
}

// Physically move the window to new_base: every new local cell takes the
// value of the same WORLD cell in the old window, or UNKNOWN if that world
// cell was outside the old window.
void move_window(RefMap& s, const std::int32_t new_base[3]) {
    std::int64_t d[3];
    for (int a = 0; a < 3; ++a)
        d[a] = static_cast<std::int64_t>(new_base[a]) - s.base[a];
#ifdef FPGA_M7_ROLL_MUTANT_SHIFT_SIGN
    for (int a = 0; a < 3; ++a) d[a] = -d[a];
#endif
    for (std::int32_t z = 0; z < DIM[2]; ++z)
        for (std::int32_t y = 0; y < DIM[1]; ++y)
            for (std::int32_t x = 0; x < DIM[0]; ++x) {
                const std::int32_t now_local[3] = {x, y, z};
                std::int64_t old_local[3] = {x + d[0], y + d[1], z + d[2]};
#ifdef FPGA_M7_ROLL_MUTANT_NO_EVICT
                // Ghost bug: entering cells inherit the cell that aliased
                // them in a ring of the window size.
                for (int a = 0; a < 3; ++a) old_local[a] = floor_mod(old_local[a], DIM[a]);
#endif
                std::int8_t value = 0;
                if (inside(old_local))
                    value = s.cells[(old_local[2] * DIM[1] + old_local[1]) * DIM[0] + old_local[0]];
                s.scratch[local_index(now_local)] = value;
            }
    std::memcpy(s.cells, s.scratch, sizeof(s.cells));
    for (int a = 0; a < 3; ++a) s.base[a] = new_base[a];
}

}  // namespace

extern "C" std::uint32_t fpga_m7_roll_ref_state_bytes() {
    return static_cast<std::uint32_t>(sizeof(RefMap));
}

extern "C" void fpga_m7_roll_ref_init(void* state) {
    RefMap* s = new (state) RefMap;
    std::memset(s->cells, 0, sizeof(s->cells));
    std::memset(s->scratch, 0, sizeof(s->scratch));
    std::memset(s->observed, 0, sizeof(s->observed));
    s->initialized = false;
    s->has_frame = false;
    s->epoch = s->delta_seq = s->last_frame = 0;
    s->base[0] = s->base[1] = s->base[2] = 0;
    s->accepted = s->rej_epoch = s->rej_frame = s->rej_malformed = 0;
}

extern "C" void fpga_m7_roll_ref_reset(void* state, std::uint32_t epoch,
                                      const std::int32_t window_base[3],
                                      std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    RefMap& s = *static_cast<RefMap*>(state);
    if (!coord_ok(window_base[0]) || !coord_ok(window_base[1]) || !coord_ok(window_base[2])) {
        ++s.rej_malformed;
        fill_header(s, FPGA_M7_ROLL_OP_RESET, FPGA_M7_ROLL_BAD_ARGS, 0, 0, 0, 0, 0, header);
        return;
    }
    std::memset(s.cells, 0, sizeof(s.cells));
    for (int a = 0; a < 3; ++a) s.base[a] = window_base[a];
    s.epoch = epoch;
    s.delta_seq = 0;
    s.last_frame = 0;
    s.has_frame = false;
    s.initialized = true;
    fill_header(s, FPGA_M7_ROLL_OP_RESET, FPGA_M7_ROLL_OK, 0, 0, 0, 0, 0, header);
}

extern "C" void fpga_m7_roll_ref_integrate(
    void* state, std::uint32_t epoch, std::uint32_t frame_seq,
    const std::int32_t window_base[3], const std::int32_t obs_base[3],
    std::uint32_t onx, std::uint32_t ony, std::uint32_t onz,
    const std::uint8_t obs[FPGA_M7_ROLL_OBS_CAPACITY],
    std::int32_t changes[4 * FPGA_M7_ROLL_CELLS],
    std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    RefMap& s = *static_cast<RefMap*>(state);
    const std::uint32_t op = FPGA_M7_ROLL_OP_INTEGRATE;
    if (!s.initialized) {
        fill_header(s, op, FPGA_M7_ROLL_UNINITIALIZED, 0, 0, 0, 0, 0, header);
        return;
    }
    bool args_ok = onx >= 1 && ony >= 1 && onz >= 1 && onx <= FPGA_M7_ROLL_OBS_AXIS &&
                   ony <= FPGA_M7_ROLL_OBS_AXIS && onz <= FPGA_M7_ROLL_OBS_AXIS;
    for (int a = 0; a < 3; ++a)
        args_ok = args_ok && coord_ok(window_base[a]) && coord_ok(obs_base[a]);
    if (!args_ok) {
        ++s.rej_malformed;
        fill_header(s, op, FPGA_M7_ROLL_BAD_ARGS, 0, 0, 0, 0, 0, header);
        return;
    }
#ifndef FPGA_M7_ROLL_MUTANT_IGNORE_EPOCH
    if (epoch != s.epoch) {
        ++s.rej_epoch;
        fill_header(s, op, FPGA_M7_ROLL_STALE_EPOCH, 0, 0, 0, 0, 0, header);
        return;
    }
#else
    (void)epoch;
#endif
    if (s.has_frame && frame_seq <= s.last_frame) {
        ++s.rej_frame;
        fill_header(s, op, FPGA_M7_ROLL_STALE_FRAME, 0, 0, 0, 0, 0, header);
        return;
    }
    const std::uint32_t total = onx * ony * onz;
    for (std::uint32_t i = 0; i < total; ++i) {
        if (obs[i] > FPGA_M7_ROLL_OCCUPIED) {
            ++s.rej_malformed;
            fill_header(s, op, FPGA_M7_ROLL_MALFORMED_OBS, 0, 0, 0, 0, 0, header);
            return;
        }
    }

    std::uint32_t shift = 0;
    bool full = false;
    for (int a = 0; a < 3; ++a) {
        const std::int64_t d = static_cast<std::int64_t>(window_base[a]) - s.base[a];
        if (d != 0) shift |= 1u << a;
        if (d >= DIM[a] || -d >= DIM[a]) full = true;
    }
    if (full) shift |= 8u;
    if (shift != 0) move_window(s, window_base);

    std::memset(s.observed, 0, sizeof(s.observed));
    std::uint32_t n = 0, changed = 0, outside = 0, inside_count = 0;
    for (std::uint32_t oz = 0; oz < onz; ++oz)
        for (std::uint32_t oy = 0; oy < ony; ++oy)
            for (std::uint32_t ox = 0; ox < onx; ++ox) {
                const std::uint32_t v = obs[(oz * ony + oy) * onx + ox];
                const std::int64_t world[3] = {static_cast<std::int64_t>(obs_base[0]) + ox,
                                               static_cast<std::int64_t>(obs_base[1]) + oy,
                                               static_cast<std::int64_t>(obs_base[2]) + oz};
                const std::int64_t local[3] = {world[0] - s.base[0], world[1] - s.base[1],
                                               world[2] - s.base[2]};
                if (!inside(local)) {
                    if (v != 0) ++outside;
                    continue;
                }
                if (v != 0) ++inside_count;
                const std::int64_t idx = (local[2] * DIM[1] + local[1]) * DIM[0] + local[0];
                if (v != 0) s.observed[idx] = 1;
                const std::int32_t before = s.cells[idx];
                const std::int32_t after = fpga_m7_roll_update(before, v);
                s.cells[idx] = static_cast<std::int8_t>(after);
                const std::uint32_t ns = fpga_m7_roll_state(after);
                if (fpga_m7_roll_state(before) != ns) {
                    changes[4 * n + 0] = static_cast<std::int32_t>(world[0]);
                    changes[4 * n + 1] = static_cast<std::int32_t>(world[1]);
                    changes[4 * n + 2] = static_cast<std::int32_t>(world[2]);
                    changes[4 * n + 3] = static_cast<std::int32_t>(ns);
                    ++n;
                    ++changed;
                }
            }
#ifdef FPGA_M7_ROLL_MUTANT_DECAY
    // Fault model: "time passing / no return" treated as free evidence.
    for (std::int32_t z = 0; z < DIM[2]; ++z)
        for (std::int32_t y = 0; y < DIM[1]; ++y)
            for (std::int32_t x = 0; x < DIM[0]; ++x) {
                const std::int32_t l3[3] = {x, y, z};
                const std::int64_t idx = local_index(l3);
                if (s.observed[idx]) continue;
                const std::int32_t before = s.cells[idx];
                const std::int32_t after = fpga_m7_roll_update(before, FPGA_M7_ROLL_FREE);
                s.cells[idx] = static_cast<std::int8_t>(after);
                if (fpga_m7_roll_state(before) != fpga_m7_roll_state(after)) {
                    changes[4 * n + 0] = s.base[0] + x;
                    changes[4 * n + 1] = s.base[1] + y;
                    changes[4 * n + 2] = s.base[2] + z;
                    changes[4 * n + 3] = static_cast<std::int32_t>(fpga_m7_roll_state(after));
                    ++n;
                    ++changed;
                }
            }
#endif
    ++s.delta_seq;
    s.last_frame = frame_seq;
    s.has_frame = true;
    ++s.accepted;
    fill_header(s, op, FPGA_M7_ROLL_OK, n, changed, outside, inside_count, shift, header);
}

extern "C" void fpga_m7_roll_ref_query(
    void* state, std::uint32_t query_count,
    const std::int32_t query_xyz[3 * FPGA_M7_ROLL_QUERY_CAPACITY],
    std::uint32_t query_out[FPGA_M7_ROLL_QUERY_CAPACITY],
    std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    RefMap& s = *static_cast<RefMap*>(state);
    if (!s.initialized) {
        fill_header(s, FPGA_M7_ROLL_OP_QUERY, FPGA_M7_ROLL_UNINITIALIZED, 0, 0, 0, 0, 0, header);
        return;
    }
    if (query_count > FPGA_M7_ROLL_QUERY_CAPACITY) {
        fill_header(s, FPGA_M7_ROLL_OP_QUERY, FPGA_M7_ROLL_BAD_ARGS, 0, 0, 0, 0, 0, header);
        return;
    }
    for (std::uint32_t i = 0; i < query_count; ++i) {
        const std::int64_t local[3] = {
            static_cast<std::int64_t>(query_xyz[3 * i]) - s.base[0],
            static_cast<std::int64_t>(query_xyz[3 * i + 1]) - s.base[1],
            static_cast<std::int64_t>(query_xyz[3 * i + 2]) - s.base[2]};
        if (!inside(local)) {
            query_out[i] = 1u << 16;
            continue;
        }
        const std::int32_t l = s.cells[(local[2] * DIM[1] + local[1]) * DIM[0] + local[0]];
        query_out[i] = fpga_m7_roll_state(l) |
                       (static_cast<std::uint32_t>(static_cast<std::uint8_t>(l)) << 8);
    }
    fill_header(s, FPGA_M7_ROLL_OP_QUERY, FPGA_M7_ROLL_OK, 0, 0, 0, 0, 0, header);
}

extern "C" void fpga_m7_roll_ref_dump(void* state, std::int8_t cells[FPGA_M7_ROLL_CELLS],
                                     std::uint32_t header[FPGA_M7_ROLL_HEADER_WORDS]) {
    RefMap& s = *static_cast<RefMap*>(state);
    if (!s.initialized) {
        fill_header(s, FPGA_M7_ROLL_OP_DUMP, FPGA_M7_ROLL_UNINITIALIZED, 0, 0, 0, 0, 0, header);
        return;
    }
    std::memcpy(cells, s.cells, sizeof(s.cells));
    fill_header(s, FPGA_M7_ROLL_OP_DUMP, FPGA_M7_ROLL_OK, 0, 0, 0, 0, 0, header);
}

extern "C" void fpga_m7_roll_ref_config(std::uint32_t out[16]) {
    out[0] = FPGA_M7_ROLL_DIM_X;
    out[1] = FPGA_M7_ROLL_DIM_Y;
    out[2] = FPGA_M7_ROLL_DIM_Z;
    out[3] = FPGA_M7_ROLL_LANES;
    out[4] = FPGA_M7_ROLL_WORDS;
    out[5] = FPGA_M7_ROLL_OBS_AXIS;
    out[6] = FPGA_M7_ROLL_DELTA_CAPACITY;
    out[7] = FPGA_M7_ROLL_QUERY_CAPACITY;
    out[8] = FPGA_M7_ROLL_HEADER_WORDS;
    out[9] = static_cast<std::uint32_t>(FPGA_M7_ROLL_HIT);
    out[10] = static_cast<std::uint32_t>(FPGA_M7_ROLL_MISS);
    out[11] = static_cast<std::uint32_t>(FPGA_M7_ROLL_LMAX);
    out[12] = static_cast<std::uint32_t>(FPGA_M7_ROLL_LMIN);
    out[13] = static_cast<std::uint32_t>(FPGA_M7_ROLL_T_OCC);
    out[14] = static_cast<std::uint32_t>(FPGA_M7_ROLL_T_FREE);
    out[15] = fpga_m7_roll_mutant_mask();
}
