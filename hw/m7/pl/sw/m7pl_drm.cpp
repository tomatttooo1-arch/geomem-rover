// Display scheme 1: the PL display core renders into a DDR framebuffer and the
// PS DisplayPort controller (Linux DRM, zynqmp-dpsub) scans it out.
// Since the 2026-10-11 merge build the renderer is the AR overlay core
// (fpga_m7_ar_overlay_axi, XRGB8888, depth image below a 40-row text panel);
// the legacy display core is no longer in the block design.
//
// Two scanout modes, chosen automatically:
//   import  buffers come from a dma-heap (dma-buf fd): PRIME-imported into DRM,
//           the PL writes the very buffer the DP controller reads (zero copy);
//   copy    buffers from u-dma-buf/physmem cannot be imported: the PL renders
//           into a DMA buffer and the PS memcpy()s it into a DRM dumb buffer.
//           This copy is PS work and is reported separately.
// Two DRM framebuffers form the two watchdog "banks". Per frame:
//   render inactive bank -> watchdog commit (capture age, ids) -> page flip ->
//   flip-complete event -> watchdog frame_start.
// After --frames frames updates stop; the PL watchdog must go stale on its own
// clock, and the PS then flips to a fail-closed frame. In scheme 1 that last
// step needs the PS program alive; see M7_PL_INTEGRATION_20260926.md.
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <drm/drm.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include "m7pl_dev.hpp"

using namespace m7pl;

int render_ar(Device& d, const Buffer& bdepth, const Buffer& bgrid, const Buffer& bdesc, const Buffer& bfb,
              const std::uint32_t regs[24], double timeout, std::uint64_t* ns, std::uint64_t* cyc);
void ar_scene(std::uint16_t* depth, std::uint32_t w, std::uint32_t h, std::uint32_t phase);
void ar_grid(std::uint8_t* g, std::uint32_t phase);
void ar_desc(std::uint32_t* desc, std::uint32_t dw, std::uint32_t dh, std::uint32_t nbox, std::uint32_t magic);

namespace {

int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do r = ioctl(fd, req, arg); while (r == -1 && (errno == EINTR || errno == EAGAIN));
    return r;
}

template <typename T> std::vector<T> vec(std::uint32_t n) { return std::vector<T>(n ? n : 1); }
std::uint64_t ptr(const void* p) { return reinterpret_cast<std::uintptr_t>(p); }

struct Output {
    int fd = -1;
    std::uint32_t conn = 0, crtc = 0, plane = 0;
    drm_mode_modeinfo mode{};
    std::vector<std::uint32_t> formats;
    std::string driver;
};

bool plane_is_primary(int fd, std::uint32_t plane) {
    drm_mode_obj_get_properties gp{};
    gp.obj_id = plane;
    gp.obj_type = DRM_MODE_OBJECT_PLANE;
    if (xioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &gp)) return false;
    auto ids = vec<std::uint32_t>(gp.count_props);
    auto vals = vec<std::uint64_t>(gp.count_props);
    gp.props_ptr = ptr(ids.data());
    gp.prop_values_ptr = ptr(vals.data());
    if (xioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &gp)) return false;
    for (std::uint32_t i = 0; i < gp.count_props; ++i) {
        drm_mode_get_property p{};
        p.prop_id = ids[i];
        if (xioctl(fd, DRM_IOCTL_MODE_GETPROPERTY, &p)) continue;
        if (!std::strcmp(p.name, "type")) return vals[i] == 1;  // DRM_PLANE_TYPE_PRIMARY
    }
    return false;
}

bool open_output(Output& out, std::uint32_t max_w, std::uint32_t max_h, std::string& err) {
    for (int card = 0; card < 4; ++card) {
        char path[32];
        std::snprintf(path, sizeof path, "/dev/dri/card%d", card);
        const int fd = open(path, O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;
        char name[64] = {0};
        drm_version ver{};
        ver.name = name;
        ver.name_len = sizeof name - 1;
        xioctl(fd, DRM_IOCTL_VERSION, &ver);
        drm_set_client_cap cap{DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1};
        xioctl(fd, DRM_IOCTL_SET_CLIENT_CAP, &cap);
        drm_mode_card_res res{};
        if (xioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res)) { close(fd); continue; }
        auto conns = vec<std::uint32_t>(res.count_connectors);
        auto crtcs = vec<std::uint32_t>(res.count_crtcs);
        auto encs = vec<std::uint32_t>(res.count_encoders);
        auto fbs = vec<std::uint32_t>(res.count_fbs);
        res.connector_id_ptr = ptr(conns.data()); res.crtc_id_ptr = ptr(crtcs.data());
        res.encoder_id_ptr = ptr(encs.data()); res.fb_id_ptr = ptr(fbs.data());
        if (xioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res)) { close(fd); continue; }
        std::printf("drm %s driver=%s connectors=%u crtcs=%u\n", path, name, res.count_connectors, res.count_crtcs);
        for (std::uint32_t c = 0; c < res.count_connectors; ++c) {
            drm_mode_get_connector gc{};
            gc.connector_id = conns[c];
            if (xioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc)) continue;
            auto modes = vec<drm_mode_modeinfo>(gc.count_modes);
            auto props = vec<std::uint32_t>(gc.count_props);
            auto pvals = vec<std::uint64_t>(gc.count_props);
            auto cencs = vec<std::uint32_t>(gc.count_encoders);
            gc.modes_ptr = ptr(modes.data()); gc.props_ptr = ptr(props.data());
            gc.prop_values_ptr = ptr(pvals.data()); gc.encoders_ptr = ptr(cencs.data());
            if (xioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc)) continue;
            std::printf("  connector %u status=%u modes=%u\n", gc.connector_id, gc.connection, gc.count_modes);
            if (gc.connection != 1 || gc.count_modes == 0) continue;  // 1 = connected
            // Prefer 1280x800, then 1280x720, then the largest mode within the core's capacity.
            int best = -1;
            long best_score = -1;
            for (std::uint32_t m = 0; m < gc.count_modes; ++m) {
                const auto& mi = modes[m];
                std::printf("    mode %ux%u@%u%s\n", mi.hdisplay, mi.vdisplay, mi.vrefresh,
                            (mi.type & DRM_MODE_TYPE_PREFERRED) ? " preferred" : "");
                if (mi.hdisplay > max_w || mi.vdisplay > max_h) continue;
                long score = static_cast<long>(mi.hdisplay) * mi.vdisplay;
                if (mi.hdisplay == 1280 && mi.vdisplay == 800) score += 1L << 40;
                if (mi.hdisplay == 1280 && mi.vdisplay == 720) score += 1L << 39;
                if (mi.vrefresh == 60) score += 1;
                if (score > best_score) { best_score = score; best = static_cast<int>(m); }
            }
            if (best < 0) { err = "connected monitor offers no mode <= 1280x800"; continue; }
            // CRTC: from the current encoder, else the first possible one.
            std::uint32_t crtc = 0;
            drm_mode_get_encoder ge{};
            ge.encoder_id = gc.encoder_id ? gc.encoder_id : cencs[0];
            if (!xioctl(fd, DRM_IOCTL_MODE_GETENCODER, &ge)) {
                crtc = ge.crtc_id;
                for (std::uint32_t i = 0; !crtc && i < res.count_crtcs; ++i)
                    if (ge.possible_crtcs & (1u << i)) crtc = crtcs[i];
            }
            int crtc_index = -1;
            for (std::uint32_t i = 0; i < res.count_crtcs; ++i) if (crtcs[i] == crtc) crtc_index = static_cast<int>(i);
            if (!crtc || crtc_index < 0) { err = "no CRTC for connector"; continue; }
            // Primary plane of that CRTC and its formats.
            drm_mode_get_plane_res pr{};
            xioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
            auto planes = vec<std::uint32_t>(pr.count_planes);
            pr.plane_id_ptr = ptr(planes.data());
            xioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
            for (std::uint32_t p = 0; p < pr.count_planes; ++p) {
                drm_mode_get_plane gp{};
                gp.plane_id = planes[p];
                if (xioctl(fd, DRM_IOCTL_MODE_GETPLANE, &gp)) continue;
                auto fm = vec<std::uint32_t>(gp.count_format_types);
                gp.format_type_ptr = ptr(fm.data());
                if (xioctl(fd, DRM_IOCTL_MODE_GETPLANE, &gp)) continue;
                const bool primary = plane_is_primary(fd, planes[p]);
                std::printf("  plane %u primary=%d crtcs=0x%x formats:", planes[p], primary, gp.possible_crtcs);
                for (std::uint32_t f = 0; f < gp.count_format_types; ++f) {
                    const std::uint32_t v = fm[f];
                    std::printf(" %c%c%c%c", v & 0xff, (v >> 8) & 0xff, (v >> 16) & 0xff, v >> 24);
                }
                std::printf("\n");
                if (primary && (gp.possible_crtcs & (1u << crtc_index)) && !out.plane) {
                    out.plane = planes[p];
                    out.formats.assign(fm.begin(), fm.begin() + gp.count_format_types);
                }
            }
            out.fd = fd; out.conn = gc.connector_id; out.crtc = crtc; out.mode = modes[best];
            out.driver = name;
            return true;
        }
        close(fd);
    }
    if (err.empty()) err = "no connected DRM connector (monitor plugged into the KV260 DP port?)";
    return false;
}

struct Scanout {
    std::uint32_t handle = 0, fb = 0;
    void* map = nullptr;   // copy mode: dumb buffer mapping
    std::size_t size = 0;
    Buffer dev;            // what the PL writes
};

bool make_fb(Output& o, Scanout& s, std::uint32_t w, std::uint32_t h, std::uint32_t fourcc, bool import,
             std::string& err) {
    if (import) {
        drm_prime_handle ph{};
        ph.fd = s.dev.dmabuf_fd;
        if (xioctl(o.fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &ph)) { err = "PRIME_FD_TO_HANDLE: " + std::string(strerror(errno)); return false; }
        s.handle = ph.handle;
    } else {
        drm_mode_create_dumb cd{};
        cd.width = w; cd.height = h; cd.bpp = 32;
        if (xioctl(o.fd, DRM_IOCTL_MODE_CREATE_DUMB, &cd)) { err = "CREATE_DUMB: " + std::string(strerror(errno)); return false; }
        if (cd.pitch != w * 4) { err = "dumb pitch " + std::to_string(cd.pitch) + " != width*4"; return false; }
        drm_mode_map_dumb md{};
        md.handle = cd.handle;
        if (xioctl(o.fd, DRM_IOCTL_MODE_MAP_DUMB, &md)) { err = "MAP_DUMB"; return false; }
        s.map = mmap(nullptr, cd.size, PROT_READ | PROT_WRITE, MAP_SHARED, o.fd, static_cast<off_t>(md.offset));
        if (s.map == MAP_FAILED) { err = "mmap dumb"; return false; }
        s.handle = cd.handle; s.size = cd.size;
    }
    drm_mode_fb_cmd2 fc{};
    fc.width = w; fc.height = h; fc.pixel_format = fourcc;
    fc.handles[0] = s.handle; fc.pitches[0] = w * 4; fc.offsets[0] = 0;
    if (xioctl(o.fd, DRM_IOCTL_MODE_ADDFB2, &fc)) { err = "ADDFB2: " + std::string(strerror(errno)); return false; }
    s.fb = fc.fb_id;
    return true;
}

bool wait_flip(int fd, double timeout_s) {
    pollfd p = {fd, POLLIN, 0};
    if (poll(&p, 1, static_cast<int>(timeout_s * 1000)) != 1) return false;
    char buf[1024];
    const ssize_t n = read(fd, buf, sizeof buf);
    for (ssize_t off = 0; off + static_cast<ssize_t>(sizeof(drm_event)) <= n;) {
        const drm_event* e = reinterpret_cast<const drm_event*>(buf + off);
        if (e->type == DRM_EVENT_FLIP_COMPLETE) return true;
        off += e->length;
    }
    return false;
}

}  // namespace

int run_drm(Device& d, int frames, double timeout_s) {
    if (d.is_mock()) { std::printf("SKIP drm: needs the board\n"); return 0; }
    Output o;
    std::string err;
    if (!open_output(o, 1280, 800, err)) { std::printf("FAIL drm output: %s\n", err.c_str()); return 1; }
    if (xioctl(o.fd, DRM_IOCTL_SET_MASTER, nullptr)) {
        std::printf("FAIL DRM_IOCTL_SET_MASTER: %s (stop the desktop: sudo systemctl stop gdm)\n", strerror(errno));
        return 1;
    }
    const std::uint32_t w = o.mode.hdisplay, h = o.mode.vdisplay;
    std::uint32_t fourcc = 0;
    for (std::uint32_t f : o.formats) if (f == DRM_FORMAT_XRGB8888) fourcc = f;
    if (!fourcc) for (std::uint32_t f : o.formats) if (f == DRM_FORMAT_ARGB8888) fourcc = f;
    if (!fourcc) { std::printf("FAIL primary plane supports neither XRGB8888 nor ARGB8888\n"); return 1; }
    std::printf("drm: driver=%s mode %ux%u@%u plane=%u format=%s\n", o.driver.c_str(), w, h, o.mode.vrefresh,
                o.plane, fourcc == DRM_FORMAT_XRGB8888 ? "XRGB8888" : "ARGB8888 (PL alpha byte is 0; if the "
                "screen stays black the plane uses per-pixel alpha -> see checklist)");

    if ((w & 3) != 0 || h <= 40) { std::printf("FAIL mode %ux%u: AR core needs width%%4==0 and height>40\n", w, h); return 1; }
    const std::uint32_t dw = w & ~7u, dh = h - 40;
    Buffer depth, grid, desc;
    if (!d.alloc(depth, static_cast<std::size_t>(dw) * dh * 2 + 64, err) || !d.alloc(grid, 32768, err) ||
        !d.alloc(desc, 1024, err)) { std::printf("FAIL alloc depth/grid/desc: %s\n", err.c_str()); return 1; }
    ar_desc(static_cast<std::uint32_t*>(desc.va), dw, dh, 3, 0x4D374152u);
    d.to_device(desc, 0, 196 * 4);
    Scanout sc[2];
    bool import = true;
    for (Scanout& s : sc) {
        if (!d.alloc(s.dev, static_cast<std::size_t>(w) * h * 4, err)) { std::printf("FAIL alloc fb: %s\n", err.c_str()); return 1; }
        import = import && s.dev.dmabuf_fd >= 0;
    }
    for (Scanout& s : sc)
        if (!make_fb(o, s, w, h, fourcc, import, err)) {
            if (import) {  // fall back to dumb + copy
                std::printf("note: PRIME import failed (%s); using dumb buffer + PS copy\n", err.c_str());
                import = false;
                err.clear();
                if (make_fb(o, s, w, h, fourcc, false, err)) continue;
            }
            std::printf("FAIL framebuffer: %s\n", err.c_str());
            return 1;
        }
    std::printf("drm scanout mode: %s\n", import ? "import (PL writes the scanned buffer, zero copy)"
                                                  : "copy (PL -> DMA buffer -> PS memcpy -> dumb buffer)");
    const double hz = d.rd(SYSINFO, 0x10);
    const std::uint32_t max_age_ms = 200;
    auto present = [&](int bank, std::uint32_t fv, std::uint32_t age_ms, std::uint32_t phase, double* render_ms,
                       double* copy_ms) -> int {
        ar_scene(static_cast<std::uint16_t*>(depth.va), dw, dh, phase);
        ar_grid(static_cast<std::uint8_t*>(grid.va), phase);
        d.to_device(depth, 0, depth.size);
        d.to_device(grid, 0, 32768);
        // permit_present=1: the panel shows the live permit wire (DRIVE OK / STOP).
        const std::uint32_t regs[24] = {w, h, dw, dh, 0, 40, fv, 1, age_ms, 1, max_age_ms, 0, phase % 4, phase, 1, 0,
                                        1, 12, 450, 500, 16, 1, w >= 136 ? w - 132 : 0u, h >= 172 ? 44u : 0u};
        std::uint64_t ns = 0;
        const int st = render_ar(d, depth, grid, desc, sc[bank].dev, regs, timeout_s, &ns, nullptr);
        if (render_ms) *render_ms = ns / 1e6;
        if (!import) {
            const std::uint64_t t0 = mono_ns();
            d.from_device(sc[bank].dev, 0, sc[bank].dev.size);
            std::memcpy(sc[bank].map, sc[bank].dev.va, static_cast<std::size_t>(w) * h * 4);
            if (copy_ms) *copy_ms = (mono_ns() - t0) / 1e6;
        } else if (copy_ms) *copy_ms = 0;
        return st;
    };

    // Watchdog epoch; bank b <-> framebuffer sc[b].
    const std::uint64_t ctx = mono_ns() ^ 0x4452000000000000ull;
    d.wr(WATCHDOG, 0x08, static_cast<std::uint32_t>(ctx)); d.wr(WATCHDOG, 0x0C, static_cast<std::uint32_t>(ctx >> 32));
    d.wr(WATCHDOG, 0x04, 0x1);
    d.wr(WATCHDOG, 0x40, 0x7);
    const int first = static_cast<int>(d.rd(WATCHDOG, 0x24) & 1);  // active bank
    // Nothing valid yet: show the fail-closed pattern on the active bank.
    if (present(first, 0, 0, 0, nullptr, nullptr) != 0) { std::printf("FAIL initial render\n"); return 1; }
    drm_mode_crtc cr{};
    cr.crtc_id = o.crtc; cr.fb_id = sc[first].fb; cr.set_connectors_ptr = ptr(&o.conn);
    cr.count_connectors = 1; cr.mode = o.mode; cr.mode_valid = 1;
    if (xioctl(o.fd, DRM_IOCTL_MODE_SETCRTC, &cr)) { std::printf("FAIL SETCRTC: %s\n", strerror(errno)); return 1; }

    int bad = 0;
    double sum_render = 0, sum_copy = 0, sum_total = 0, max_total = 0;
    for (int i = 0; i < frames; ++i) {
        const std::uint64_t capture = mono_ns();  // synthetic "capture" instant of this frame
        const std::uint32_t st0 = d.rd(WATCHDOG, 0x24);
        const int bank = static_cast<int>((st0 >> 1) & 1);
        if (!(st0 & (1u << 2))) { ++bad; std::printf("frame %d: watchdog not write_ready (0x%x)\n", i, st0); }
        double rms = 0, cms = 0;
        const std::uint32_t age_ms = static_cast<std::uint32_t>((mono_ns() - capture) / 1000000);
        const int st = present(bank, 1, age_ms, static_cast<std::uint32_t>(i), &rms, &cms);
        const double age_s = (mono_ns() - capture) / 1e9;
        const double cyc = std::ceil(age_s * hz);
        const std::uint32_t age_cycles = cyc >= 4294967295.0 ? 0xFFFFFFFFu : static_cast<std::uint32_t>(cyc);
        d.wr(WATCHDOG, 0x10, static_cast<std::uint32_t>(ctx)); d.wr(WATCHDOG, 0x14, static_cast<std::uint32_t>(ctx >> 32));
        d.wr(WATCHDOG, 0x18, static_cast<std::uint32_t>(i + 1)); d.wr(WATCHDOG, 0x1C, 0);
        d.wr(WATCHDOG, 0x20, age_cycles);
        d.wr(WATCHDOG, 0x04, 0x4 | (st == 0 ? 0x100u : 0u) | (static_cast<std::uint32_t>(bank) << 9));
        drm_mode_crtc_page_flip pf{};
        pf.crtc_id = o.crtc; pf.fb_id = sc[bank].fb; pf.flags = DRM_MODE_PAGE_FLIP_EVENT;
        if (xioctl(o.fd, DRM_IOCTL_MODE_PAGE_FLIP, &pf) || !wait_flip(o.fd, timeout_s)) {
            ++bad; std::printf("frame %d: page flip failed: %s\n", i, strerror(errno)); continue;
        }
        d.wr(WATCHDOG, 0x04, 0x8);  // the new bank is on screen from this vblank
        const std::uint32_t st1 = d.rd(WATCHDOG, 0x24);
        if (!(st1 & (1u << 4)) || static_cast<int>(st1 & 1) != bank) {
            ++bad; std::printf("frame %d: watchdog not fresh on bank %d (0x%x)\n", i, bank, st1);
        }
        const double total = (mono_ns() - capture) / 1e6;
        sum_render += rms; sum_copy += cms; sum_total += total; if (total > max_total) max_total = total;
    }
    std::printf("drm %d frames: render(PL) avg %.2f ms, PS copy avg %.2f ms, capture->flip avg %.2f ms max %.2f ms, "
                "bad=%d\n", frames, sum_render / frames, sum_copy / frames, sum_total / frames, max_total, bad);

    // Stall: stop updating. The PL watchdog must time out on its own.
    const std::uint64_t t_stop = mono_ns();
    while (!(d.rd(WATCHDOG, 0x24) & (1u << 5)) && mono_ns() - t_stop < 5000000000ull) {}
    const double stale_ms = (mono_ns() - t_stop) / 1e6;
    const bool stale = (d.rd(WATCHDOG, 0x24) & (1u << 5)) && (d.rd(WATCHDOG, 0x40) & 1u);
    std::printf("%s watchdog stale + IRQ_STATUS after %.1f ms without updates\n", stale ? "PASS" : "FAIL", stale_ms);
    // PS response in scheme 1: flip to a fail-closed "STALE" frame.
    const int bank = static_cast<int>((d.rd(WATCHDOG, 0x24) >> 1) & 1);
    present(bank, 1, max_age_ms + 1, 0, nullptr, nullptr);
    drm_mode_crtc_page_flip pf{};
    pf.crtc_id = o.crtc; pf.fb_id = sc[bank].fb; pf.flags = DRM_MODE_PAGE_FLIP_EVENT;
    const bool flipped = !xioctl(o.fd, DRM_IOCTL_MODE_PAGE_FLIP, &pf) && wait_flip(o.fd, timeout_s);
    std::printf("%s PS flipped to the fail-closed STALE frame (look at the monitor: purple checker + STALE)\n",
                flipped ? "PASS" : "FAIL");
    sleep(3);
    xioctl(o.fd, DRM_IOCTL_DROP_MASTER, nullptr);
    return (bad == 0 && stale && flipped) ? 0 : 1;
}
