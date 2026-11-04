// m7ar_present: resident AR presenter for kv_server --ar drm (car AR display, 2026-10-11).
//
// Why a separate C++ process (docs/design/CAR_AR_RESULTS_20261011.md section 1):
//   * the fail-safe picture must not depend on kv_server's Python process being alive
//     (m7pl_drm.cpp: "in scheme 1 that last step needs the PS program alive"): this
//     process owns DRM master and switches to the STALE frame on its own timer, also
//     after kv_server died (stdin EOF), until SIGTERM;
//   * no GIL: register programming, the AR core wait and the page-flip wait never
//     compete with kv_server's geometry worker thread;
//   * DRM/KMS (PRIME import of dma-heap buffers, ADDFB2, SETCRTC, PAGE_FLIP events) is
//     already board-proven in hw/m7/pl/sw/m7pl_drm.cpp (pl-merge fa63a11, step 7); the
//     output/framebuffer/flip code below is extracted from it unchanged in substance.
//
// Interface. stdin: length-prefixed frame messages from host/kv_ar.py (latest wins: all
// complete messages waiting in the pipe are read, only the newest is rendered).
// stdout: one JSON object per line ("ready", "frame", "failsafe", "exit" events).
//   message = header "<4sIII" (magic "M7AF", version 1, type 1, payload length)
//             payload "<QQqI" frame_id, first_mono_ns (KV CLOCK_MONOTONIC of the request's
//             first chunk), age_basis_ns (Nano elapsed capture->send, -1 = none), depth_px;
//             24 x u32 registers (spec order; age_ms is overwritten here when age_basis>=0);
//             depth_px x u16, 32768 x u8 grid, 196 x u32 descriptor.
// Age written to the core (one formula, host/kv_ar.py ar_age_ms is the reference):
//   age_ms = ceil((age_basis_ns + (now_mono_ns - first_mono_ns)) / 1e6), saturated.
// Replay (10-12): regs[replay] in a message is the DATA mark (kv_server --ar-source replay or the
// request's replay bit). The core skips both age checks when replay=1, so a replay frame with no
// age or older than max_age_ms is written to the core with replay=0: the core then shows NOAGE /
// STALE by its own rules, exactly as for a live frame; a fresh replay frame keeps replay=1 (panel
// REPLAY + its age). Events carry both: "replay" (data) and "replay_core" (written to the core).
// Mode in events ("mode", "mode_name"): a PS-side mirror of the core's mode rule (mode_of below,
// from the registers as written, the core's AP_RETURN status and the depth sent), not read back.
// Colour main view (2026-10-16, owner 10-15; host/kv_ar.py MSG_VERSION_COLOUR): a version-2 message
// adds "<6I" flags (b0 colour mode on), pip_x, pip_y, pip_div, colour width, colour height, then
// width*height*3 RGB888 bytes. After the core finished a FRESH picture (mode 0) -- or a NODATA picture
// (mode 4: kv_server's no-depth self pull, all-zero depth, no geometry, 2026-10-17) -- and the colour has the
// depth size, compose_colour (1) copies the PL depth rectangle decimated by pip_div to (pip_x, pip_y)
// -- the PiP of what the PL drew -- and (2) replaces every depth-rectangle pixel the PL overlay does
// not own (geometric mask from the registers and descriptor written: panel, box lines, label bars,
// mini map ring/inner/label) by the colour pixel. Cache: invalidate before (CPU reads the PL's
// pixels), clean after (the scan-out sees the CPU's). Any other picture (fail-safe, stale, bad
// frame, pose bad, no age, colour missing) is shown exactly as the PL wrote it. Reference: tools/ar_display/
// colour_compose.py; checker: tools/kv_link/ar_dump_check.py. --colour-mutant mask_shift|pip_row are
// positive controls for that checker only.
// Mini map 2x (owner 10-29): with --mini-scale 2 AND a version-2 message whose flags have b0 (colour
// mode on) and b1 (kv_ar laid the mini map out for 2x, host/kv_ar.py ar_colour_layout mini_scale=2),
// scale_mini copies the PL's mini-map box -- 12-row label + 1 px ring + 128x128 inner, the 130 x 141
// pixels from (mini_x-1, mini_y-12) -- and writes it back nearest-neighbour 2x from the same top-left
// corner (260 x 282; the source box lies inside, so no 1x copy is left). Same CPU stage as
// compose_colour (after it), on EVERY flipped picture of such a frame: fresh, NODATA, colour refresh,
// colour missing, fail-safe STALE (it reuses the last frame's flags). The PL circuit is unchanged
// (still 4 px per cell). --mini-scale 1 (default) = the code before, byte for byte. --colour-mutant
// mini_row (odd target rows take the next source row) is the checker's positive control.
// stdin is read incrementally (non-blocking chunks into a buffer): a half-written message never
// blocks the loop, so the fail-safe timer keeps running while kv_server is slow to write.
// Registers only through generic-uio nodes (m7_sysinfo MAGIC + SLOT_MAP bit 8, m7_ar map0
// at 0xA008_0000, m7_permit MAGIC for read-only STATUS); never /dev/mem.
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <set>
#include <string>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#include <vector>

#include <drm/drm.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_mode.h>

#include "ar_overlay.hpp"
#include "ar_overlay_fast.hpp"
#include "m7ar_regs.h"
#include "m7ar_wait.h"

extern "C" std::uint32_t fpga_m7_ar_overlay_axi(
    const fpga_m7_ar_word*, const fpga_m7_ar_word*, const fpga_m7_ar_word*, fpga_m7_ar_word*,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, bool, std::uint32_t, std::uint32_t,
    std::int32_t, std::int32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t);

namespace {

const std::uint32_t kMsgMagic = 0x4641374Du;  // "M7AF" little endian
const std::uint32_t kSysMagic = 0x4D37504Cu, kPermitMagic = 0x4D37504Du;
const std::uint32_t kArBase = 0xA0080000u, kArSlot = 8;
const std::size_t kGridBytes = 32768, kDescU32 = 196;
const int kRegs = 24, kRegPermitI = 15, kRegAgeMs = 8, kRegAgeValid = 9, kRegReplay = 11;
const int kRegFrameValid = 6, kRegMaxAge = 10, kRegFrameId = 13;
const int kRegDepthW = 2, kRegDepthH = 3, kRegPoseValid = 7;

volatile std::sig_atomic_t g_stop = 0;
void on_signal(int) { g_stop = 1; }
// Screen bring-up (2026-10-15): SIGUSR1 = dump the next flipped render (any kind), on top of
// --dump-seq / --dump-failsafe. Needs --dump-dir.
volatile std::sig_atomic_t g_dump_req = 0;
void on_dump_signal(int) { g_dump_req = 1; }

std::uint64_t mono_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

// The one age formula (host/kv_ar.py ar_age_ms is the known-answer reference).
std::uint32_t age_ms_of(std::int64_t basis_ns, std::uint64_t first_mono_ns, std::uint64_t now) {
    const std::int64_t kv = now >= first_mono_ns ? static_cast<std::int64_t>(now - first_mono_ns) : 0;
    const std::int64_t total = basis_ns + kv;
    if (total <= 0) return 0;
    const std::uint64_t ms = (static_cast<std::uint64_t>(total) + 999999ull) / 1000000ull;
    return ms > 0xFFFFFFFFull ? 0xFFFFFFFFu : static_cast<std::uint32_t>(ms);
}

// PS-side mirror of the AR core's mode (hw/m7/ar_overlay.cpp "Mode: 0 fresh, 1 bad frame/control/
// descriptor, 2 stale, 3 pose bad, 4 no depth, 5 no age"), computed here from the registers as
// written, the core's returned status and the depth handed to it -- NOT read back from the core.
// -1: the core did not render (timeout, or status 1 = dimensions refused).
int mode_of(bool done, std::uint32_t status, const std::uint32_t* regs, const std::vector<std::uint8_t>& depth) {
    if (!done || status == 1) return -1;
    const std::size_t px = std::min<std::size_t>(static_cast<std::size_t>(regs[kRegDepthW]) * regs[kRegDepthH],
                                                 depth.size() / 2);
    bool has_depth = false;
    for (std::size_t i = 0; i < px && !has_depth; ++i) {
        const std::uint32_t z = depth[2 * i] | (static_cast<std::uint32_t>(depth[2 * i + 1]) << 8);
        has_depth = z >= 100 && z <= 9999;
    }
    if (status != 0 || regs[kRegFrameValid] == 0) return 1;
    if (regs[kRegReplay] == 0 && regs[kRegAgeValid] == 0) return 5;
    if (regs[kRegReplay] == 0 && regs[kRegAgeMs] > regs[kRegMaxAge]) return 2;
    if (regs[kRegPoseValid] == 0) return 3;
    if (!has_depth) return 4;
    return 0;
}

const char* mode_name(int mode, std::uint32_t replay_core) {
    switch (mode) {
        case 0: return replay_core == 1 ? "REPLAY" : "LIVE";
        case 1: return "BADFRAME";
        case 2: return "STALE";
        case 3: return "POSEBAD";
        case 4: return "NODATA";
        case 5: return "NOAGE";
        default: return nullptr;
    }
}

// --event-log: every event line also goes to this file (unbuffered), so the events after
// kv_server died (stdout has no reader then) are still on record.
FILE* g_event_log = nullptr;

void emit(const std::string& s) {
    std::fputs(s.c_str(), stdout);
    std::fputc('\n', stdout);
    std::fflush(stdout);
    if (g_event_log) {
        std::fputs(s.c_str(), g_event_log);
        std::fputc('\n', g_event_log);
        std::fflush(g_event_log);
    }
}

std::string fmt(const char* f, ...) __attribute__((format(printf, 1, 2)));
std::string fmt(const char* f, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, f);
    std::vsnprintf(buf, sizeof buf, f, ap);
    va_end(ap);
    return buf;
}

std::string jstr(const std::string& s) {
    std::string o = "\"";
    for (char c : s) {
        if (c == '"' || c == '\\') { o += '\\'; o += c; }
        else if (static_cast<unsigned char>(c) < 0x20) o += ' ';
        else o += c;
    }
    return o + "\"";
}

// ---------------------------------------------------------------- buffers
struct Buf {
    void* va = nullptr;
    std::uint64_t pa = 0;
    std::size_t size = 0;
    int dmabuf_fd = -1;
    bool cached = false;
};

void dcache_range(const void* p, std::size_t len, bool invalidate) {
#if defined(__aarch64__)
    std::uint64_t ctr;
    asm volatile("mrs %0, ctr_el0" : "=r"(ctr));
    const std::uintptr_t line = 4u << ((ctr >> 16) & 0xF);
    std::uintptr_t a = reinterpret_cast<std::uintptr_t>(p) & ~(line - 1);
    const std::uintptr_t end = reinterpret_cast<std::uintptr_t>(p) + len;
    for (; a < end; a += line) {
        if (invalidate) asm volatile("dc civac, %0" ::"r"(a) : "memory");
        else asm volatile("dc cvac, %0" ::"r"(a) : "memory");
    }
    asm volatile("dsb sy" ::: "memory");
#else
    (void)p; (void)len; (void)invalidate;
#endif
}

// ---------------------------------------------------------------- device
class Device {
public:
    virtual ~Device() {}
    virtual std::uint32_t rd(std::uint32_t off) = 0;
    virtual void wr(std::uint32_t off, std::uint32_t v) = 0;
    virtual bool alloc(Buf& b, std::size_t size, std::string& err) = 0;
    // permit STATUS b0 (the wire the core samples), -1 when not readable
    virtual int permit_bit() = 0;
    // start the core and wait for ap_done; returns false on timeout. last_run: what the wait saw.
    virtual bool run(double timeout_s, std::uint64_t* ns) = 0;
    // before a render (before the DMA inputs are rewritten): bring the core and its interrupt
    // path to an empty state (m7ar_wait.h resync_core); false = still busy, skip the render.
    virtual bool resync(double timeout_s, m7ar::ResyncInfo* info) { if (info) *info = m7ar::ResyncInfo(); (void)timeout_s; return true; }
    m7ar::RunInfo last_run;
    virtual std::string describe() = 0;
    // the permit_i wire for the next run, from the message's register 15 (AR2NANO --provider cpu only;
    // the board core samples its permit_o wire, the mock takes --mock-permit: both ignore this)
    virtual void set_wire_permit(std::uint32_t) {}
    void to_device(const Buf& b, std::size_t len) { if (b.cached) dcache_range(b.va, len, false); __sync_synchronize(); }
    void from_device(const Buf& b, std::size_t len) { __sync_synchronize(); if (b.cached) dcache_range(b.va, len, true); }
    void wr64(std::uint32_t off, std::uint64_t v) { wr(off, static_cast<std::uint32_t>(v)); wr(off + 4, static_cast<std::uint32_t>(v >> 32)); }
};

int find_uio(const std::string& sysfs, const std::string& name) {
    DIR* dir = opendir(sysfs.c_str());
    if (!dir) return -1;
    int found = -1, hits = 0;
    while (dirent* e = readdir(dir)) {
        if (std::strncmp(e->d_name, "uio", 3)) continue;
        FILE* f = std::fopen((sysfs + "/" + e->d_name + "/name").c_str(), "r");
        if (!f) continue;
        char n[128] = {0};
        if (std::fgets(n, sizeof n, f)) {
            n[std::strcspn(n, "\n")] = 0;
            if (name == n) { found = std::atoi(e->d_name + 3); ++hits; }
        }
        std::fclose(f);
    }
    closedir(dir);
    return hits == 1 ? found : -1;
}

bool read_hex(const std::string& path, std::uint64_t& v) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    unsigned long long x = 0;
    const int n = std::fscanf(f, "%lli", &x);
    std::fclose(f);
    v = x;
    return n == 1;
}

struct Uio {
    int fd = -1;
    volatile std::uint32_t* w = nullptr;
    std::size_t size = 0;
    std::uint64_t addr = 0;
    int index = -1;
    bool open_node(const std::string& sysfs, const std::string& dev, const std::string& name, std::string& err) {
        index = find_uio(sysfs, name);
        if (index < 0) { err = "no unique UIO node named " + name; return false; }
        const std::string m = sysfs + "/uio" + std::to_string(index) + "/maps/map0/";
        std::uint64_t sz = 0;
        if (!read_hex(m + "addr", addr) || !read_hex(m + "size", sz)) { err = name + ": map0 unreadable"; return false; }
        size = static_cast<std::size_t>(sz);
        fd = open((dev + "/uio" + std::to_string(index)).c_str(), O_RDWR | O_SYNC);
        if (fd < 0) { err = name + ": open uio: " + strerror(errno); return false; }
        void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        if (p == MAP_FAILED) { err = name + ": mmap: " + strerror(errno); close(fd); fd = -1; return false; }
        w = static_cast<volatile std::uint32_t*>(p);
        return true;
    }
};

struct dma_heap_allocation_data_compat {
    std::uint64_t len;
    std::uint32_t fd;
    std::uint32_t fd_flags;
    std::uint64_t heap_flags;
};
#define M7AR_DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_allocation_data_compat)

bool pagemap_phys(void* va, std::size_t size, std::uint64_t& pa, std::string& err) {
    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) { err = "open pagemap: " + std::string(strerror(errno)); return false; }
    const long page = sysconf(_SC_PAGESIZE);
    std::uint64_t first = 0;
    for (std::size_t off = 0; off < size; off += page) {
        const std::uintptr_t v = reinterpret_cast<std::uintptr_t>(va) + off;
        std::uint64_t e = 0;
        if (pread(fd, &e, 8, static_cast<off_t>(v / page) * 8) != 8) { err = "pread pagemap"; close(fd); return false; }
        if (!(e >> 63)) { err = "page not present"; close(fd); return false; }
        const std::uint64_t pfn = e & ((1ull << 55) - 1);
        if (pfn == 0) { err = "pagemap PFN hidden (need root)"; close(fd); return false; }
        const std::uint64_t p = pfn * page;
        if (off == 0) first = p;
        else if (p != first + off) { err = "dma-heap buffer not physically contiguous"; close(fd); return false; }
    }
    close(fd);
    pa = first;
    return true;
}

class UioDevice : public Device {
public:
    bool open_all(const std::string& sysfs, const std::string& dev, const std::string& heap, bool irq,
                  std::string& err) {
        // Gate 1: sysinfo MAGIC and SLOT_MAP bit 8 before the AR window is touched.
        if (!sys_.open_node(sysfs, dev, "m7_sysinfo", err)) return false;
        if (sys_.w[0] != kSysMagic) { err = fmt("m7_sysinfo MAGIC 0x%08x != 0x%08x", sys_.w[0], kSysMagic); return false; }
        build_id_ = sys_.w[0x08 / 4];
        slot_map_ = sys_.w[0x20 / 4];
        if (!((slot_map_ >> kArSlot) & 1)) {
            err = fmt("bitstream 0x%08x SLOT_MAP 0x%x has no AR slot (bit %u): the AR core exists only in m7_pl_merge",
                      build_id_, slot_map_, kArSlot);
            return false;
        }
        // Gate 2: the m7_ar node is the AR slot and the core is idle.
        if (!ar_.open_node(sysfs, dev, "m7_ar", err)) return false;
        if (ar_.addr != kArBase || ar_.size < 0x1000) { err = fmt("m7_ar map0 0x%llx+0x%zx is not 0x%08x", (unsigned long long)ar_.addr, ar_.size, kArBase); return false; }
        if (!(ar_.w[0] & 0x4)) { err = fmt("AR core not idle at open (AP_CTRL 0x%x)", ar_.w[0]); return false; }
        // Optional: permit STATUS (read only), for the before/after permit_i evidence.
        std::string perr;
        if (permit_.open_node(sysfs, dev, "m7_permit", perr) && permit_.w[0] == kPermitMagic) permit_ok_ = true;
        else permit_reason_ = perr.empty() ? "m7_permit MAGIC mismatch" : perr;
        heapfd_ = open(("/dev/dma_heap/" + heap).c_str(), O_RDWR);
        if (heapfd_ < 0) { err = "open /dev/dma_heap/" + heap + ": " + strerror(errno); return false; }
        heap_ = heap;
        irq_ = irq;
        if (irq_) {
            irqfd_ = ar_.fd;  // the same /dev/uioN: read() blocks for the interrupt count
        }
        return true;
    }
    std::uint32_t rd(std::uint32_t off) override { return ar_.w[off / 4]; }
    void wr(std::uint32_t off, std::uint32_t v) override { ar_.w[off / 4] = v; }
    int permit_bit() override { return permit_ok_ ? static_cast<int>(permit_.w[0x10 / 4] & 1u) : -1; }
    bool alloc(Buf& out, std::size_t size, std::string& err) override {
        dma_heap_allocation_data_compat d;
        std::memset(&d, 0, sizeof d);
        d.len = (size + 4095) & ~std::size_t(4095);
        d.fd_flags = O_RDWR | O_CLOEXEC;
        if (ioctl(heapfd_, M7AR_DMA_HEAP_IOCTL_ALLOC, &d) < 0) { err = "DMA_HEAP_IOCTL_ALLOC: " + std::string(strerror(errno)); return false; }
        void* va = mmap(nullptr, d.len, PROT_READ | PROT_WRITE, MAP_SHARED, d.fd, 0);
        if (va == MAP_FAILED) { err = "mmap dma-buf: " + std::string(strerror(errno)); close(d.fd); return false; }
        std::memset(va, 0, d.len);
        std::uint64_t pa = 0;
        if (!pagemap_phys(va, d.len, pa, err)) { munmap(va, d.len); close(d.fd); return false; }
        if (pa + d.len > 0x80000000ull) { err = "dma-heap buffer outside the PL DMA window (low 2 GiB)"; return false; }
        out.va = va; out.pa = pa; out.size = d.len; out.dmabuf_fd = static_cast<int>(d.fd); out.cached = true;
        dcache_range(va, d.len, false);
        return true;
    }
    // Hw adaptor for m7ar_wait.h (registers of the AR window, UIO irq line).
    struct Hw {
        UioDevice* d;
        std::uint32_t rd(std::uint32_t off) { return d->rd(off); }
        void wr(std::uint32_t off, std::uint32_t v) { d->wr(off, v); }
        bool irq() const { return d->irq_; }
        int irq_fd() const { return d->irqfd_; }
        bool irq_arm() { std::uint32_t one = 1; return write(d->irqfd_, &one, 4) == 4; }   // (re)arm the UIO line
        bool irq_read() {
            std::uint32_t count = 0;
            ssize_t n;
            do n = read(d->irqfd_, &count, 4); while (n < 0 && errno == EINTR);
            if (n == 4) ++d->irq_events_;
            return n == 4;
        }
        void pause_us(unsigned us) { timespec ts = {0, static_cast<long>(us) * 1000L}; nanosleep(&ts, nullptr); }
    };
    bool resync(double timeout_s, m7ar::ResyncInfo* info) override {
        Hw hw{this};
        return m7ar::resync_core(hw, mono_ns() + static_cast<std::uint64_t>(timeout_s * 1e9), info);
    }
    bool run(double timeout_s, std::uint64_t* ns) override {
        const std::uint64_t t0 = mono_ns();
        Hw hw{this};
        const bool ok = m7ar::run_core(hw, t0 + static_cast<std::uint64_t>(timeout_s * 1e9), &last_run);
        if (ns) *ns = mono_ns() - t0;
        return ok;
    }
    std::string describe() override {
        return fmt("{\"provider\":\"uio+dmaheap\",\"heap\":%s,\"build_id\":\"0x%08x\",\"slot_map\":\"0x%x\","
                   "\"ar_uio\":\"uio%d\",\"wait\":\"%s\",\"permit\":%s}",
                   jstr(heap_).c_str(), build_id_, slot_map_, ar_.index, irq_ ? "irq" : "poll",
                   permit_ok_ ? fmt("\"uio%d\"", permit_.index).c_str() : jstr("unavailable: " + permit_reason_).c_str());
    }

private:
    Uio sys_, ar_, permit_;
    bool permit_ok_ = false, irq_ = false;
    std::string permit_reason_, heap_;
    int heapfd_ = -1, irqfd_ = -1;
    std::uint32_t build_id_ = 0, slot_map_ = 0;
    std::uint64_t irq_events_ = 0;
};

// Host stand-in: registers in memory, ap_start executes the same HLS shell C code on the
// buffers (128-bit beats packed with range(), as hw/m7/pl/sw/m7pl_dev.cpp MockDevice);
// permit_i comes from --mock-permit. Evidence about marshaling and flow, not the RTL.
class MockDevice : public Device {
public:
    explicit MockDevice(int permit) : permit_(permit) { regs_.assign(0x1000 / 4, 0); regs_[0] = 0x4; }
    std::uint32_t rd(std::uint32_t off) override {
        const std::uint32_t v = regs_[off / 4];
        if (off == M7AR_AP_CTRL) regs_[0] &= ~0x2u;
        return v;
    }
    void wr(std::uint32_t off, std::uint32_t v) override {
        if (off == M7AR_AP_CTRL) { if (v & 1) { execute(); regs_[0] = 0x6; } return; }
        regs_[off / 4] = v;
    }
    int permit_bit() override { return permit_; }
    bool alloc(Buf& out, std::size_t size, std::string& err) override {
        const std::size_t sz = (size + 4095) & ~std::size_t(4095);
        void* p = nullptr;
        if (posix_memalign(&p, 4096, sz)) { err = "posix_memalign"; return false; }
        std::memset(p, 0, sz);
        out.va = p; out.pa = next_pa_; out.size = sz;
        mem_.push_back(out);
        next_pa_ += sz + 0x10000;
        return true;
    }
    bool run(double, std::uint64_t* ns) override {
        const std::uint64_t t0 = mono_ns();
        wr(M7AR_AP_CTRL, 1);
        const bool ok = rd(M7AR_AP_CTRL) & 0x2;
        if (ns) *ns = mono_ns() - t0;
        return ok;
    }
    std::string describe() override { return fmt("{\"provider\":\"mock\",\"permit_i\":%d}", permit_); }

protected:
    int permit_;
    // the run itself; AR2NANO2 FastCpuDevice overrides it with the fast twin of the core
    virtual void execute();
    std::uint32_t r(std::uint32_t off) { return regs_[off / 4]; }
    std::uint64_t r64(std::uint32_t off) { return r(off) | (static_cast<std::uint64_t>(r(off + 4)) << 32); }
    std::uint8_t* va(std::uint64_t pa) {
        for (const Buf& b : mem_)
            if (pa >= b.pa && pa < b.pa + b.size) return static_cast<std::uint8_t*>(b.va) + (pa - b.pa);
        std::fprintf(stderr, "MOCK: device access to unmapped physical 0x%llx\n", (unsigned long long)pa);
        std::abort();
    }
    void execute_ref() {
        const std::uint32_t w = r(M7AR_WIDTH), h = r(M7AR_HEIGHT), dw = r(M7AR_DEPTH_WIDTH), dh = r(M7AR_DEPTH_HEIGHT);
        const bool dims_ok = w && h && w <= FPGA_M7_AR_MAX_WIDTH && h <= FPGA_M7_AR_MAX_HEIGHT &&
                             dw <= FPGA_M7_AR_MAX_WIDTH && dh <= FPGA_M7_AR_MAX_HEIGHT;
        const std::size_t dwords = dims_ok ? (static_cast<std::size_t>(dw) * dh + 7) / 8 : 0;
        const std::size_t fwords = dims_ok ? (static_cast<std::size_t>(w) * h + 3) / 4 : 0;
        std::vector<fpga_m7_ar_word> dep(dwords + 1, 0), grid(FPGA_M7_AR_GRID_WORDS, 0), box(FPGA_M7_AR_DESC_WORDS, 0),
            fb(fwords + 1, 0);
        auto load = [&](std::vector<fpga_m7_ar_word>& v, std::size_t words, std::uint32_t reg) {
            if (!words) return;
            const std::uint8_t* p = va(r64(reg));
            for (std::size_t i = 0; i < words * 16; ++i) v[i / 16].range(8 * (i % 16) + 7, 8 * (i % 16)) = p[i];
        };
        load(dep, dwords, M7AR_DEPTH);
        load(grid, r(M7AR_GRID_VALID) == 1 ? FPGA_M7_AR_GRID_WORDS : 0, M7AR_GRID);
        load(box, dims_ok ? FPGA_M7_AR_DESC_WORDS : 0, M7AR_BOXES);
        const std::uint32_t st = fpga_m7_ar_overlay_axi(
            dep.data(), grid.data(), box.data(), fb.data(), w, h, dw, dh, r(M7AR_DEPTH_DX), r(M7AR_DEPTH_DY),
            r(M7AR_FRAME_VALID), r(M7AR_POSE_VALID), r(M7AR_AGE_MS), r(M7AR_AGE_VALID), r(M7AR_MAX_AGE_MS),
            r(M7AR_REPLAY), r(M7AR_OBJECT_EVIDENCE), r(M7AR_FRAME_ID), r(M7AR_PERMIT_PRESENT), permit_ != 0,
            r(M7AR_GRID_VALID), r(M7AR_SLICE_Y), static_cast<std::int32_t>(r(M7AR_SLICE_LO_MM)),
            static_cast<std::int32_t>(r(M7AR_SLICE_HI_MM)), r(M7AR_CAM_CELL_X), r(M7AR_CAM_CELL_Z), r(M7AR_MINI_X),
            r(M7AR_MINI_Y));
        regs_[M7AR_AP_RETURN / 4] = st;
        if (st != 1 && fwords) {
            std::uint32_t* px = reinterpret_cast<std::uint32_t*>(va(r64(M7AR_FB)));
            const std::size_t n = static_cast<std::size_t>(w) * h;
            for (std::size_t i = 0; i < n; ++i) px[i] = fb[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)).to_uint();
        }
    }
    std::vector<std::uint32_t> regs_;

private:
    std::vector<Buf> mem_;
    std::uint64_t next_pa_ = 0x10000000ull;
};
void MockDevice::execute() { execute_ref(); }

// AR2NANO (nationals 2026-11-02, owner: the car screen is drawn on the Jetson Orin Nano, the PL no
// longer carries the AR core): --provider cpu = the SAME C core as the mock (hw/m7/ar_overlay.cpp
// through the HLS AXI shell, unchanged), as the production renderer on the Nano CPU. The one
// difference: permit_i comes from the message (register 15, which the sender fills from the Nano's
// own permit knowledge) instead of a fixed --mock-permit, because there is no permit_o wire here.
class CpuDevice : public MockDevice {
public:
    CpuDevice() : MockDevice(0) {}
    void set_wire_permit(std::uint32_t v) override { permit_ = v != 0 ? 1 : 0; }
    std::string describe() override { return "{\"provider\":\"cpu\",\"impl\":\"ref\",\"permit_i\":\"message\"}"; }
};

// AR2NANO2 (2026-11-02): --provider cpu --cpu-impl fast (the default for cpu) = hw/m7/ar_overlay_fast.cpp,
// the byte-exact fast twin of the same core (tools/ar_display/fast_ar_check.py: 69 fixed + 256 seeded
// random cases, 0 differing bytes; mutant positive controls), straight on the device buffers: no 128-bit
// beat packing, row-wise painting. --cpu-impl ref keeps the CpuDevice above (the golden C core).
// Same register reads and the same "fb written only when status != 1" rule as MockDevice::execute_ref.
class FastCpuDevice : public CpuDevice {
public:
    std::string describe() override { return "{\"provider\":\"cpu\",\"impl\":\"fast\",\"permit_i\":\"message\"}"; }

protected:
    void execute() override {
        std::uint32_t regs[24];
        for (int i = 0; i < 24; ++i) regs[i] = i == kRegPermitI ? static_cast<std::uint32_t>(permit_ != 0) : r(kM7arRegOff[i]);
        const std::uint32_t w = regs[0], h = regs[1], dw = regs[2], dh = regs[3];
        const bool dims_ok = w && h && w <= FPGA_M7_AR_MAX_WIDTH && h <= FPGA_M7_AR_MAX_HEIGHT &&
                             dw <= FPGA_M7_AR_MAX_WIDTH && dh <= FPGA_M7_AR_MAX_HEIGHT;
        if (!dims_ok) { regs_[M7AR_AP_RETURN / 4] = 1; return; }   // the core returns 1 for these too
        static std::vector<std::uint8_t> zgrid(kGridBytes, 0);
        const std::uint8_t* grid = r(M7AR_GRID_VALID) == 1 ? va(r64(M7AR_GRID)) : zgrid.data();
        const std::uint16_t* dep = reinterpret_cast<const std::uint16_t*>(va(r64(M7AR_DEPTH)));
        const std::uint32_t* box = reinterpret_cast<const std::uint32_t*>(va(r64(M7AR_BOXES)));
        std::uint32_t* px = reinterpret_cast<std::uint32_t*>(va(r64(M7AR_FB)));
        regs_[M7AR_AP_RETURN / 4] = fpga_m7_ar_overlay_fast(dep, grid, box, px, regs, 1);
    }
};

// ---------------------------------------------------------------- DRM (from m7pl_drm.cpp)
int xioctl(int fd, unsigned long req, void* arg) {
    int r;
    do r = ioctl(fd, req, arg); while (r == -1 && (errno == EINTR || errno == EAGAIN));
    return r;
}
template <typename T> std::vector<T> vec(std::uint32_t n) { return std::vector<T>(n ? n : 1); }
std::uint64_t uptr(const void* p) { return reinterpret_cast<std::uintptr_t>(p); }

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
    gp.props_ptr = uptr(ids.data());
    gp.prop_values_ptr = uptr(vals.data());
    if (xioctl(fd, DRM_IOCTL_MODE_OBJ_GETPROPERTIES, &gp)) return false;
    for (std::uint32_t i = 0; i < gp.count_props; ++i) {
        drm_mode_get_property p{};
        p.prop_id = ids[i];
        if (xioctl(fd, DRM_IOCTL_MODE_GETPROPERTY, &p)) continue;
        if (!std::strcmp(p.name, "type")) return vals[i] == 1;
    }
    return false;
}

// Mode choice as m7pl_drm.cpp: 1280x800, then 1280x720, then the largest mode <= the core's
// 1280x800 capacity (60 Hz preferred). Screen bring-up overrides (2026-10-15, monitors without
// EDID): want_w/want_h > 0 -> only a listed mode of exactly that size (60 Hz preferred);
// custom != nullptr -> that mode as given (--modeline), the connector list is not consulted.
bool open_output(Output& out, std::uint32_t max_w, std::uint32_t max_h, std::string& err,
                 std::uint32_t want_w = 0, std::uint32_t want_h = 0,
                 const drm_mode_modeinfo* custom = nullptr) {
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
        res.connector_id_ptr = uptr(conns.data()); res.crtc_id_ptr = uptr(crtcs.data());
        res.encoder_id_ptr = uptr(encs.data()); res.fb_id_ptr = uptr(fbs.data());
        if (xioctl(fd, DRM_IOCTL_MODE_GETRESOURCES, &res)) { close(fd); continue; }
        for (std::uint32_t c = 0; c < res.count_connectors; ++c) {
            drm_mode_get_connector gc{};
            gc.connector_id = conns[c];
            if (xioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc)) continue;
            auto modes = vec<drm_mode_modeinfo>(gc.count_modes);
            auto props = vec<std::uint32_t>(gc.count_props);
            auto pvals = vec<std::uint64_t>(gc.count_props);
            auto cencs = vec<std::uint32_t>(gc.count_encoders);
            gc.modes_ptr = uptr(modes.data()); gc.props_ptr = uptr(props.data());
            gc.prop_values_ptr = uptr(pvals.data()); gc.encoders_ptr = uptr(cencs.data());
            if (xioctl(fd, DRM_IOCTL_MODE_GETCONNECTOR, &gc)) continue;
            if (gc.connection != 1 || (gc.count_modes == 0 && !custom)) continue;
            int best = -1;
            long best_score = -1;
            for (std::uint32_t m = 0; m < gc.count_modes; ++m) {
                const auto& mi = modes[m];
                if (mi.hdisplay > max_w || mi.vdisplay > max_h) continue;
                if (want_w && (mi.hdisplay != want_w || mi.vdisplay != want_h)) continue;
                long score = static_cast<long>(mi.hdisplay) * mi.vdisplay;
                if (mi.hdisplay == 1280 && mi.vdisplay == 800) score += 1L << 40;
                if (mi.hdisplay == 1280 && mi.vdisplay == 720) score += 1L << 39;
                if (mi.vrefresh == 60) score += 1;
                if (score > best_score) { best_score = score; best = static_cast<int>(m); }
            }
            if (custom) best = 0;
            if (best < 0) { err = want_w ? "connected monitor lists no mode of the --mode size"
                                         : "connected monitor offers no mode <= 1280x800"; continue; }
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
            drm_mode_get_plane_res pr{};
            xioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
            auto planes = vec<std::uint32_t>(pr.count_planes);
            pr.plane_id_ptr = uptr(planes.data());
            xioctl(fd, DRM_IOCTL_MODE_GETPLANERESOURCES, &pr);
            for (std::uint32_t p = 0; p < pr.count_planes; ++p) {
                drm_mode_get_plane gp{};
                gp.plane_id = planes[p];
                if (xioctl(fd, DRM_IOCTL_MODE_GETPLANE, &gp)) continue;
                auto fm = vec<std::uint32_t>(gp.count_format_types);
                gp.format_type_ptr = uptr(fm.data());
                if (xioctl(fd, DRM_IOCTL_MODE_GETPLANE, &gp)) continue;
                if (plane_is_primary(fd, planes[p]) && (gp.possible_crtcs & (1u << crtc_index)) && !out.plane) {
                    out.plane = planes[p];
                    out.formats.assign(fm.begin(), fm.begin() + gp.count_format_types);
                }
            }
            out.fd = fd; out.conn = gc.connector_id; out.crtc = crtc; out.mode = custom ? *custom : modes[best];
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
    Buf dev;               // what the PL writes
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
        if (cd.pitch != w * 4) { err = "dumb pitch != width*4"; return false; }
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

// Wait for the FLIP_COMPLETE of the flip just queued (until the deadline; a signal does not end
// the wait, other DRM events are skipped). *eintr += signals that hit the wait.
bool wait_flip(int fd, double timeout_s, int* eintr) {
    const std::uint64_t deadline = mono_ns() + static_cast<std::uint64_t>(timeout_s * 1e9);
    for (;;) {
        if (m7ar::wait_readable(fd, deadline, eintr) != 1) return false;
        char buf[1024];
        ssize_t n;
        do n = read(fd, buf, sizeof buf); while (n < 0 && errno == EINTR);
        if (n <= 0) return false;
        for (ssize_t off = 0; off + static_cast<ssize_t>(sizeof(drm_event)) <= n;) {
            const drm_event* e = reinterpret_cast<const drm_event*>(buf + off);
            if (e->type == DRM_EVENT_FLIP_COMPLETE) return true;
            if (e->length < sizeof(drm_event)) return false;
            off += e->length;
        }
    }
}

// Display-side resync, before PAGE_FLIP: read every DRM event still queued (a FLIP_COMPLETE left by a
// flip whose wait timed out would otherwise be taken for the next flip's). Returns the bank of the last
// stale FLIP_COMPLETE (user_data = bank + 1), -1 none; *drained = events read.
int drain_drm_events(int fd, int* drained) {
    int bank = -1;
    while (m7ar::readable_now(fd)) {
        char buf[1024];
        ssize_t n;
        do n = read(fd, buf, sizeof buf); while (n < 0 && errno == EINTR);
        if (n <= 0) break;
        for (ssize_t off = 0; off + static_cast<ssize_t>(sizeof(drm_event)) <= n;) {
            const drm_event* e = reinterpret_cast<const drm_event*>(buf + off);
            if (drained) ++*drained;
            if (e->type == DRM_EVENT_FLIP_COMPLETE) {
                const drm_event_vblank* v = reinterpret_cast<const drm_event_vblank*>(e);
                if (v->user_data >= 1 && v->user_data <= 2) bank = static_cast<int>(v->user_data) - 1;
            }
            if (e->length < sizeof(drm_event)) break;
            off += e->length;
        }
    }
    return bank;
}

// ---------------------------------------------------------------- frames
struct Frame {
    std::uint64_t frame_id = 0, first_mono = 0, recv_mono = 0;
    std::int64_t age_basis = -1;
    std::uint32_t regs[kRegs] = {0};
    std::vector<std::uint8_t> depth, grid, desc;
    bool valid = false;
    std::uint32_t version = 1;
    std::uint32_t colour_flags = 0, pip_x = 0, pip_y = 0, pip_div = 0, cw = 0, ch = 0;
    std::vector<std::uint8_t> rgb;
};

const std::size_t kMaxBody = 32u << 20;

// One message body (after the 16-byte header) -> Frame; false + err on a malformed body.
bool parse_body(std::uint32_t version, const std::uint8_t* b, std::size_t n, Frame& f, std::string& err) {
    const std::size_t fixed = 8 + 8 + 8 + 4 + 4 * kRegs;
    if (n < fixed) { err = "short message"; return false; }
    std::size_t off = 0;
    std::memcpy(&f.frame_id, b + off, 8); off += 8;
    std::memcpy(&f.first_mono, b + off, 8); off += 8;
    std::memcpy(&f.age_basis, b + off, 8); off += 8;
    std::uint32_t depth_px = 0;
    std::memcpy(&depth_px, b + off, 4); off += 4;
    std::memcpy(f.regs, b + off, 4 * kRegs); off += 4 * kRegs;
    const std::size_t v1 = fixed + 2ull * depth_px + kGridBytes + 4 * kDescU32;
    if (depth_px > FPGA_M7_AR_MAX_WIDTH * FPGA_M7_AR_MAX_HEIGHT) { err = "message size mismatch"; return false; }
    if (version == 1 && n != v1) { err = "message size mismatch"; return false; }
    if (version == 2 && n < v1 + 24) { err = "message size mismatch"; return false; }
    f.depth.assign(b + off, b + off + 2ull * depth_px); off += 2ull * depth_px;
    f.grid.assign(b + off, b + off + kGridBytes); off += kGridBytes;
    f.desc.assign(b + off, b + off + 4 * kDescU32); off += 4 * kDescU32;
    f.version = version;
    f.colour_flags = f.pip_x = f.pip_y = f.pip_div = f.cw = f.ch = 0;
    f.rgb.clear();
    if (version == 2) {
        std::uint32_t c[6];
        std::memcpy(c, b + off, 24); off += 24;
        f.colour_flags = c[0]; f.pip_x = c[1]; f.pip_y = c[2]; f.pip_div = c[3]; f.cw = c[4]; f.ch = c[5];
        if (f.cw > FPGA_M7_AR_MAX_WIDTH || f.ch > FPGA_M7_AR_MAX_HEIGHT || n != v1 + 24 + 3ull * f.cw * f.ch) {
            err = "colour block size mismatch"; return false; }
        f.rgb.assign(b + off, b + off + 3ull * f.cw * f.ch);
    }
    f.recv_mono = mono_ns();
    f.valid = true;
    return true;
}

// Incremental stdin reader: append what is there, cut complete messages.
struct InBuf {
    std::vector<std::uint8_t> buf;
    std::size_t pos = 0;
    // read what is available now (non-blocking fd); returns false on EOF / hard error
    bool fill(int fd, bool& eof) {
        for (int k = 0; k < 64; ++k) {
            if (buf.size() - pos > (64u << 20)) break;
            std::uint8_t tmp[1 << 16];
            const ssize_t r = read(fd, tmp, sizeof tmp);
            if (r == 0) { eof = true; return false; }
            if (r < 0) {
                if (errno == EINTR) continue;
                if (errno == EAGAIN || errno == EWOULDBLOCK) return true;
                eof = true; return false;
            }
            buf.insert(buf.end(), tmp, tmp + r);
        }
        return true;
    }
    // next complete message: 1 got, 0 need more, -1 protocol error
    int next(Frame& f, std::string& err) {
        const std::size_t have = buf.size() - pos;
        if (have < 16) return 0;
        std::uint32_t head[4];
        std::memcpy(head, &buf[pos], 16);
        if (head[0] != kMsgMagic || (head[1] != 1 && head[1] != 2) || head[2] != 1 || head[3] > kMaxBody) {
            err = "bad message header"; return -1; }
        if (have < 16ull + head[3]) return 0;
        const bool ok = parse_body(head[1], &buf[pos + 16], head[3], f, err);
        pos += 16ull + head[3];
        if (pos > (8u << 20) || pos == buf.size()) { buf.erase(buf.begin(), buf.begin() + pos); pos = 0; }
        return ok ? 1 : -1;
    }
};

// The overlay mask of compose_colour, in depth-rectangle coordinates (Wd x Hd, 1 = PL owns it).
void overlay_mask(const std::uint32_t* regs, const std::vector<std::uint8_t>& desc_bytes, int shift_x,
                  std::vector<std::uint8_t>& m) {
    const long W = regs[0], H = regs[1], Wd = regs[kRegDepthW], Hd = regs[kRegDepthH];
    const long dx = regs[4], dy = regs[5], mx = regs[22], my = regs[23];
    m.assign(static_cast<std::size_t>(Wd * Hd), 0);
    auto mark = [&](long x0, long y0, long x1, long y1) {   // canvas rect [x0,x1) x [y0,y1)
        x0 = std::max(x0, 0L) + shift_x; x1 = std::min(x1, W) + shift_x;
        y0 = std::max(y0, 0L); y1 = std::min(y1, H);
        for (long y = std::max(y0, dy); y < std::min(y1, dy + Hd); ++y)
            for (long x = std::max(x0, dx); x < std::min(x1, dx + Wd); ++x) m[(y - dy) * Wd + (x - dx)] = 1;
    };
    mark(0, 0, W, 40);
    std::uint32_t d[kDescU32];
    std::memcpy(d, desc_bytes.data(), sizeof d);
    const std::uint32_t n = d[0] == FPGA_M7_AR_MAGIC ? std::min<std::uint32_t>(d[1], 32) : 0;
    for (std::uint32_t i = 0; i < n; ++i) {
        const long X0 = d[2 + 6 * i] + dx, Y0 = d[3 + 6 * i] + dy, X1 = d[4 + 6 * i] + dx, Y1 = d[5 + 6 * i] + dy;
        mark(X0, Y0, std::min(X0 + 2, X1 + 1), Y1 + 1);
        mark(std::max(X1 - 1, X0), Y0, X1 + 1, Y1 + 1);
        mark(X0, Y0, X1 + 1, std::min(Y0 + 2, Y1 + 1));
        mark(X0, std::max(Y1 - 1, Y0), X1 + 1, Y1 + 1);
        const long LY = Y0 >= 52 ? Y0 - 12 : Y0 + 2;
        mark(X0, LY, X0 + 48, LY + 12);
    }
    mark(mx - 1, my - 1, mx + 129, my + 129);
    mark(mx, my - 12, mx + 128, my);
}

// PiP then colour (see the header). fb = W x H XRGB as the PL wrote it. Returns false when skipped.
const std::uint32_t kColourMini2x = 2;       // message flags b1
const long kMiniBoxW = 130, kMiniBoxH = 141;  // ring 130 wide; label 12 rows + ring + 128 + ring

// The 2x target of the mini-map box lies inside the W x H canvas.
bool mini_fits(const std::uint32_t* regs, std::uint32_t scale) {
    const long W = regs[0], H = regs[1], x0 = static_cast<long>(regs[22]) - 1, y0 = static_cast<long>(regs[23]) - 12;
    return scale >= 2 && x0 >= 0 && y0 >= 0 && x0 + kMiniBoxW * scale <= W && y0 + kMiniBoxH * scale <= H;
}

// Nearest-neighbour scale of the mini-map box in place (see the header). tmp: scratch.
void scale_mini(std::uint32_t* fb, const std::uint32_t* regs, std::uint32_t scale, int mutant,
                std::vector<std::uint32_t>& tmp) {
    const std::size_t W = regs[0];
    const std::size_t x0 = regs[22] - 1, y0 = regs[23] - 12;
    tmp.resize(static_cast<std::size_t>(kMiniBoxW * kMiniBoxH));
    for (long r = 0; r < kMiniBoxH; ++r)
        std::memcpy(&tmp[static_cast<std::size_t>(r * kMiniBoxW)], fb + (y0 + r) * W + x0, 4 * kMiniBoxW);
    for (long yy = 0; yy < kMiniBoxH * static_cast<long>(scale); ++yy) {
        long sr = yy / scale;
        if (mutant == 3 && yy % scale == scale - 1) sr = std::min(sr + 1, kMiniBoxH - 1);
        const std::uint32_t* src = &tmp[static_cast<std::size_t>(sr * kMiniBoxW)];
        std::uint32_t* dst = fb + (y0 + yy) * W + x0;
        for (long c = 0; c < kMiniBoxW; ++c)
            for (std::uint32_t k = 0; k < scale; ++k) dst[c * scale + k] = src[c];
    }
}

bool compose_colour(std::uint32_t* fb, const std::uint32_t* regs, const Frame& f, int mutant,
                    std::vector<std::uint8_t>& mask, bool& pip_drawn) {
    const std::uint32_t W = regs[0], H = regs[1], Wd = regs[kRegDepthW], Hd = regs[kRegDepthH];
    const std::uint32_t dx = regs[4], dy = regs[5];
    pip_drawn = false;
    if (!(f.colour_flags & 1) || f.cw != Wd || f.ch != Hd || f.rgb.size() != 3ull * Wd * Hd || !Wd || !Hd) return false;
    const std::uint32_t div = f.pip_div ? f.pip_div : 2;
    const std::uint32_t pw = (Wd + div - 1) / div, ph = (Hd + div - 1) / div;
    if (static_cast<std::uint64_t>(f.pip_x) + pw <= W && static_cast<std::uint64_t>(f.pip_y) + ph <= H) {
        for (std::uint32_t i = 0; i < ph; ++i) {
            const std::uint32_t sr = std::min(i * div + (mutant == 2 ? 1u : 0u), Hd - 1);
            const std::uint32_t* src = fb + static_cast<std::size_t>(dy + sr) * W + dx;
            std::uint32_t* dst = fb + static_cast<std::size_t>(f.pip_y + i) * W + f.pip_x;
            for (std::uint32_t j = 0; j < pw; ++j) dst[j] = src[j * div];
        }
        pip_drawn = true;
    }
    overlay_mask(regs, f.desc, mutant == 1 ? 1 : 0, mask);
    const std::uint8_t* c = f.rgb.data();
    for (std::uint32_t y = 0; y < Hd; ++y) {
        std::uint32_t* row = fb + static_cast<std::size_t>(dy + y) * W + dx;
        const std::uint8_t* mr = &mask[static_cast<std::size_t>(y) * Wd];
        const std::uint8_t* cr = c + 3ull * y * Wd;
        for (std::uint32_t x = 0; x < Wd; ++x)
            if (!mr[x]) row[x] = (static_cast<std::uint32_t>(cr[3 * x]) << 16) | (static_cast<std::uint32_t>(cr[3 * x + 1]) << 8) | cr[3 * x + 2];
    }
    return true;
}

struct Opts {
    std::string provider = "dmaheap", heap = "reserved", display = "drm", wait = "irq";
    std::string sysfs = "/sys/class/uio", dev = "/dev", dump_dir;
    std::set<long> dump_seq;
    int dump_failsafe = 0, mock_permit = 0;
    std::string cpu_impl = "fast";            // --cpu-impl fast|ref (AR2NANO2; --provider cpu only)
    std::uint32_t mock_w = 1024, mock_h = 768;
    std::uint32_t want_w = 0, want_h = 0;     // --mode WxH
    bool have_custom = false;                 // --modeline
    drm_mode_modeinfo custom{};
    double stale_ms = 200, refresh_ms = 500, timeout_s = 0.5;
    int colour_mutant = 0;                    // 1 mask_shift, 2 pip_row, 3 mini_row (checker positive controls)
    std::uint32_t mini_scale = 1;             // --mini-scale 1|2 (owner 10-29)
};

void write_file(const std::string& path, const void* p, std::size_t n) {
    FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return;
    std::fwrite(p, 1, n, f);
    std::fclose(f);
}

}  // namespace

int main(int argc, char** argv) {
    Opts o;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto val = [&]() -> std::string { if (i + 1 >= argc) { std::fprintf(stderr, "%s needs a value\n", a.c_str()); std::exit(2); } return argv[++i]; };
        if (a == "--provider") o.provider = val();
        else if (a == "--heap") o.heap = val();
        else if (a == "--display") o.display = val();
        else if (a == "--wait") o.wait = val();
        else if (a == "--uio-sysfs") o.sysfs = val();
        else if (a == "--uio-dev") o.dev = val();
        else if (a == "--stale-ms") o.stale_ms = std::atof(val().c_str());
        else if (a == "--refresh-ms") o.refresh_ms = std::atof(val().c_str());
        else if (a == "--timeout-s") o.timeout_s = std::atof(val().c_str());
        else if (a == "--dump-dir") o.dump_dir = val();
        else if (a == "--event-log") {
            const std::string path = val();
            g_event_log = std::fopen(path.c_str(), "a");
            if (!g_event_log) { std::fprintf(stderr, "--event-log %s: %s\n", path.c_str(), strerror(errno)); return 2; }
        }
        else if (a == "--dump-seq") {
            std::string s = val();
            for (char* t = std::strtok(&s[0], ","); t; t = std::strtok(nullptr, ",")) o.dump_seq.insert(std::atol(t));
        } else if (a == "--dump-failsafe") o.dump_failsafe = std::atoi(val().c_str());
        else if (a == "--cpu-impl") o.cpu_impl = val();
        else if (a == "--mock-permit") o.mock_permit = std::atoi(val().c_str());
        else if (a == "--mode") {
            std::string s = val();
            if (std::sscanf(s.c_str(), "%ux%u", &o.want_w, &o.want_h) != 2 || !o.want_w || !o.want_h) {
                std::fprintf(stderr, "--mode WxH\n"); return 2; }
        } else if (a == "--modeline") {
            // "clock_khz hdisp hss hse htot vdisp vss vse vtot [+|-]hsync [+|-]vsync" (xrandr/cvt
            // numbers, clock in kHz)
            std::string s = val();
            unsigned c, hd, hs, he, ht, vd, vs, ve, vt; char hp = '-', vp = '+';
            if (std::sscanf(s.c_str(), "%u %u %u %u %u %u %u %u %u %c%*s %c", &c, &hd, &hs, &he, &ht, &vd, &vs,
                            &ve, &vt, &hp, &vp) < 9 || !(hd <= hs && hs <= he && he <= ht && vd <= vs && vs <= ve &&
                                                         ve <= vt) || !c) {
                std::fprintf(stderr, "--modeline 'clock_khz hd hss hse ht vd vss vse vt [+|-]hsync [+|-]vsync'\n");
                return 2;
            }
            drm_mode_modeinfo& m = o.custom;
            m.clock = c; m.hdisplay = hd; m.hsync_start = hs; m.hsync_end = he; m.htotal = ht;
            m.vdisplay = vd; m.vsync_start = vs; m.vsync_end = ve; m.vtotal = vt;
            m.vrefresh = static_cast<std::uint32_t>((c * 1000.0) / (ht * static_cast<double>(vt)) + 0.5);
            m.flags = (hp == '+' ? DRM_MODE_FLAG_PHSYNC : DRM_MODE_FLAG_NHSYNC) |
                      (vp == '-' ? DRM_MODE_FLAG_NVSYNC : DRM_MODE_FLAG_PVSYNC);
            m.type = DRM_MODE_TYPE_USERDEF;
            std::snprintf(m.name, sizeof m.name, "%ux%u", hd, vd);
            o.have_custom = true;
        }
        else if (a == "--colour-mutant") {
            const std::string v = val();
            if (v == "none") o.colour_mutant = 0;
            else if (v == "mask_shift") o.colour_mutant = 1;
            else if (v == "pip_row") o.colour_mutant = 2;
            else if (v == "mini_row") o.colour_mutant = 3;
            else { std::fprintf(stderr, "--colour-mutant none|mask_shift|pip_row|mini_row\n"); return 2; }
        }
        else if (a == "--mini-scale") {
            const std::string v = val();
            if (v == "1") o.mini_scale = 1;
            else if (v == "2") o.mini_scale = 2;
            else { std::fprintf(stderr, "--mini-scale 1|2\n"); return 2; }
        }
        else if (a == "--mock-size") { std::string s = val(); std::sscanf(s.c_str(), "%ux%u", &o.mock_w, &o.mock_h); }
        else if (a == "--age-selftest") {
            // known-answer hook for tests: basis first now -> age_ms
            const long long b = std::atoll(val().c_str());
            const unsigned long long f = std::strtoull(val().c_str(), nullptr, 10);
            const unsigned long long n = std::strtoull(val().c_str(), nullptr, 10);
            std::printf("%u\n", age_ms_of(b, f, n));
            return 0;
        } else { std::fprintf(stderr, "unknown option %s\n", a.c_str()); return 2; }
    }
    if ((o.provider != "dmaheap" && o.provider != "mock" && o.provider != "cpu") || (o.display != "drm" && o.display != "none") ||
        (o.wait != "irq" && o.wait != "poll") ||
        (o.cpu_impl != "fast" && o.cpu_impl != "ref")) { std::fprintf(stderr, "bad --provider/--display/--wait\n"); return 2; }
    std::signal(SIGTERM, on_signal);
    std::signal(SIGUSR1, on_dump_signal);
    std::signal(SIGINT, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    std::string err;
    Device* dev = nullptr;
    if (o.provider == "mock") dev = new MockDevice(o.mock_permit);
    else if (o.provider == "cpu") dev = o.cpu_impl == "ref" ? static_cast<Device*>(new CpuDevice()) : new FastCpuDevice();
    else {
        UioDevice* u = new UioDevice();
        if (!u->open_all(o.sysfs, o.dev, o.heap, o.wait == "irq", err)) {
            emit(fmt("{\"ev\":\"error\",\"error\":%s}", jstr(err).c_str()));
            return 3;
        }
        dev = u;
    }

    // Display
    Output out;
    std::uint32_t W = o.mock_w, H = o.mock_h, hz = 0, fourcc = DRM_FORMAT_XRGB8888;
    if (o.display == "drm") {
        if (!open_output(out, FPGA_M7_AR_MAX_WIDTH, FPGA_M7_AR_MAX_HEIGHT, err, o.want_w, o.want_h,
                         o.have_custom ? &o.custom : nullptr)) { emit(fmt("{\"ev\":\"error\",\"error\":%s}", jstr(err).c_str())); return 4; }
        if (xioctl(out.fd, DRM_IOCTL_SET_MASTER, nullptr)) {
            emit(fmt("{\"ev\":\"error\",\"error\":%s}", jstr(std::string("DRM SET_MASTER: ") + strerror(errno) + " (xmutil desktop_disable first)").c_str()));
            return 4;
        }
        W = out.mode.hdisplay; H = out.mode.vdisplay; hz = out.mode.vrefresh;
        fourcc = 0;
        for (std::uint32_t f : out.formats) if (f == DRM_FORMAT_XRGB8888) fourcc = f;
        if (!fourcc) for (std::uint32_t f : out.formats) if (f == DRM_FORMAT_ARGB8888) fourcc = f;
        if (!fourcc) { emit("{\"ev\":\"error\",\"error\":\"primary plane has neither XRGB8888 nor ARGB8888\"}"); return 4; }
    }
    if (W > FPGA_M7_AR_MAX_WIDTH || H > FPGA_M7_AR_MAX_HEIGHT) { emit(fmt("{\"ev\":\"error\",\"error\":\"mode %ux%u over the core's capacity\"}", W, H)); return 4; }
    if ((W & 3) || H <= 40) { emit(fmt("{\"ev\":\"error\",\"error\":\"mode %ux%u unusable (width%%4, height>40)\"}", W, H)); return 4; }

    Buf bdepth, bgrid, bdesc;
    Scanout sc[2];
    if (!dev->alloc(bdepth, FPGA_M7_AR_MAX_WIDTH * FPGA_M7_AR_MAX_HEIGHT * 2 + 64, err) || !dev->alloc(bgrid, kGridBytes, err) ||
        !dev->alloc(bdesc, 1024, err)) { emit(fmt("{\"ev\":\"error\",\"error\":%s}", jstr(err).c_str())); return 5; }
    bool import = o.display == "drm";
    for (Scanout& s : sc) {
        if (!dev->alloc(s.dev, static_cast<std::size_t>(W) * H * 4, err)) { emit(fmt("{\"ev\":\"error\",\"error\":%s}", jstr(err).c_str())); return 5; }
        import = import && s.dev.dmabuf_fd >= 0;
    }
    if (o.display == "drm")
        for (Scanout& s : sc)
            if (!make_fb(out, s, W, H, fourcc, import, err)) {
                if (import) {
                    import = false;
                    err.clear();
                    if (make_fb(out, s, W, H, fourcc, false, err)) continue;
                }
                emit(fmt("{\"ev\":\"error\",\"error\":%s}", jstr(err).c_str()));
                return 5;
            }
    const char* scanout = o.display != "drm" ? "none" : (import ? "import" : "copy");

    int shown = -1;            // bank on screen
    long seq = 0, failsafe_count = 0;
    std::uint64_t last_flip_done = 0;
    Frame last;                // newest frame received (its inputs stay in the DMA buffers)
    std::uint32_t max_age_ms = 200;
    std::vector<std::uint8_t> mask_buf;
    std::vector<std::uint32_t> mini_buf;
    bool reload_pending = false;      // a render skipped by resync left the DMA inputs of an older frame
    long tot_wait_eintr = 0, tot_flip_eintr = 0, tot_resync = 0, tot_stale_irq = 0, tot_skipped = 0;

    // render + flip one picture from `f` with registers `regs`; returns event json
    auto present = [&](const Frame& f, const std::uint32_t* regs_in, const char* kind, bool reload) -> std::string {
        const std::uint64_t t_begin = mono_ns();
        // resync first: the core may still be reading the DMA inputs of an interrupted render
        m7ar::ResyncInfo rs;
        const bool core_ok = dev->resync(o.timeout_s, &rs);
        int flip_drained = 0;
        if (o.display == "drm" && shown >= 0) {
            const int b = drain_drm_events(out.fd, &flip_drained);
            if (b >= 0) shown = b;     // a flip that completed after its wait timed out
            if (flip_drained) rs.bits |= m7ar::kResyncFlip;
        }
        const int bank = shown < 0 ? 0 : 1 - shown;
        if (rs.bits) ++tot_resync;
        if (!core_ok) { ++tot_skipped; if (reload) reload_pending = true; }
        if (core_ok && (reload || reload_pending)) {
            reload_pending = false;
            if (!f.depth.empty()) std::memcpy(bdepth.va, f.depth.data(), f.depth.size());
            std::memcpy(bgrid.va, f.grid.data(), kGridBytes);
            std::memcpy(bdesc.va, f.desc.data(), 4 * kDescU32);
            dev->to_device(bdepth, f.depth.size());
            dev->to_device(bgrid, kGridBytes);
            dev->to_device(bdesc, 4 * kDescU32);
        }
        std::uint32_t regs[kRegs];
        std::memcpy(regs, regs_in, sizeof regs);
        if (core_ok) {
            dev->wr64(M7AR_DEPTH, bdepth.pa);
            dev->wr64(M7AR_GRID, bgrid.pa);
            dev->wr64(M7AR_BOXES, bdesc.pa);
            dev->wr64(M7AR_FB, sc[bank].dev.pa);
        }
        const std::uint64_t t_age = mono_ns();
        if (regs[kRegAgeValid] == 1 && f.age_basis >= 0) regs[kRegAgeMs] = age_ms_of(f.age_basis, f.first_mono, t_age);
        // replay = where the data comes from; the core's age rules still apply to it
        const std::uint32_t replay_data = f.regs[kRegReplay];
        if (regs[kRegReplay] == 1 && (regs[kRegAgeValid] == 0 || regs[kRegAgeMs] > regs[kRegMaxAge])) regs[kRegReplay] = 0;
        if (core_ok) for (int i = 0; i < kRegs; ++i) if (i != kRegPermitI) dev->wr(kM7arRegOff[i], regs[i]);
        if (core_ok) dev->set_wire_permit(regs[kRegPermitI]);
        const int permit_before = dev->permit_bit();
        std::uint64_t ar_ns = 0;
        dev->last_run = m7ar::RunInfo();
        const bool done = core_ok && dev->run(o.timeout_s, &ar_ns);
        const m7ar::RunInfo run_info = dev->last_run;
        int flip_eintr = 0;
        const int permit_after = dev->permit_bit();
        const std::uint32_t status = done ? dev->rd(M7AR_AP_RETURN) : 0xFFFFFFFFu;
        const int mode = mode_of(done, status, regs, f.depth);
        const char* mname = mode_name(mode, regs[kRegReplay]);
        const std::uint64_t t_core = mono_ns();
        bool flipped = false;
        double copy_ms = 0;
        // colour main view: compose on the CPU between ap_done and the flip (fresh pictures only)
        const bool is_fs0 = std::strcmp(kind, "failsafe") == 0;
        // SIGUSR1 is latched ONCE per render (board 10-16): read here and used for both the pre-composition
        // copy (.fbpl) and the dump decision below; a signal arriving in between is folded into this
        // dump (before, a render could be dumped without its .fbpl copy and fail the checker).
        const bool asked = g_dump_req != 0;
        const bool will_dump = !o.dump_dir.empty() && (asked || (!is_fs0 && o.dump_seq.count(seq + 1)) ||
                                                       (is_fs0 && failsafe_count < o.dump_failsafe));
        bool composited = false, pip_drawn = false;
        std::uint64_t composite_ns = 0;
        std::vector<std::uint32_t> pl_copy;
        // mode 4 NODATA too (2026-10-17, docs/design/AR_DEPTH_INVALID_STALL_20261017.md): kv_server's no-depth
        // self pull sends the newest colour with an all-zero depth and no geometry; the PL draws no boxes then
        const bool want_compose = done && status != 1 && (mode == 0 || mode == 4) && f.version == 2 &&
                                  (f.colour_flags & 1) && f.cw && f.ch;
        // mini map 2x (owner 10-29): every flipped picture of a colour-mode frame laid out for it
        const bool want_mini = o.mini_scale > 1 && done && status != 1 && f.version == 2 && (f.colour_flags & 1) &&
                               (f.colour_flags & kColourMini2x) && mini_fits(regs, o.mini_scale);
        bool mini_applied = false;
        std::uint64_t mini_ns = 0;
        if (want_compose || want_mini) {
            const std::uint64_t c0 = mono_ns();
            const std::size_t fbytes = static_cast<std::size_t>(W) * H * 4;
            dev->from_device(sc[bank].dev, fbytes);
            std::uint32_t* fbp = static_cast<std::uint32_t*>(sc[bank].dev.va);
            if (will_dump) pl_copy.assign(fbp, fbp + static_cast<std::size_t>(W) * H);
            if (want_compose) composited = compose_colour(fbp, regs, f, o.colour_mutant, mask_buf, pip_drawn);
            const std::uint64_t c1 = mono_ns();
            if (want_mini) { scale_mini(fbp, regs, o.mini_scale, o.colour_mutant, mini_buf); mini_applied = true; }
            mini_ns = mono_ns() - c1;
            dev->to_device(sc[bank].dev, fbytes);
            composite_ns = mono_ns() - c0;
        }
        if (done && status != 1) {
            if (o.display == "drm") {
                if (!import) {
                    const std::uint64_t c0 = mono_ns();
                    dev->from_device(sc[bank].dev, static_cast<std::size_t>(W) * H * 4);
                    std::memcpy(sc[bank].map, sc[bank].dev.va, static_cast<std::size_t>(W) * H * 4);
                    copy_ms = (mono_ns() - c0) / 1e6;
                }
                if (shown < 0) {
                    drm_mode_crtc cr{};
                    cr.crtc_id = out.crtc; cr.fb_id = sc[bank].fb; cr.set_connectors_ptr = uptr(&out.conn);
                    cr.count_connectors = 1; cr.mode = out.mode; cr.mode_valid = 1;
                    flipped = !xioctl(out.fd, DRM_IOCTL_MODE_SETCRTC, &cr);
                } else {
                    drm_mode_crtc_page_flip pf{};
                    pf.crtc_id = out.crtc; pf.fb_id = sc[bank].fb; pf.flags = DRM_MODE_PAGE_FLIP_EVENT;
                    pf.user_data = static_cast<std::uint64_t>(bank) + 1;
                    flipped = !xioctl(out.fd, DRM_IOCTL_MODE_PAGE_FLIP, &pf) && wait_flip(out.fd, o.timeout_s, &flip_eintr);
                }
            } else {
                flipped = true;
            }
            if (flipped) shown = bank;
        }
        const std::uint64_t t_flip = mono_ns();
        const double interval_ms = (flipped && last_flip_done) ? (t_flip - last_flip_done) / 1e6 : -1;
        if (flipped) last_flip_done = t_flip;
        ++seq;
        std::string dumped;
        const bool is_fs = std::strcmp(kind, "failsafe") == 0;
        if (!o.dump_dir.empty() && flipped && (asked || (!is_fs && o.dump_seq.count(seq)) || (is_fs && failsafe_count < o.dump_failsafe))) {
            if (asked) g_dump_req = 0;
            const std::string base = o.dump_dir + fmt("/ar_%06ld", seq);
            dev->from_device(sc[bank].dev, static_cast<std::size_t>(W) * H * 4);
            write_file(base + ".fb", sc[bank].dev.va, static_cast<std::size_t>(W) * H * 4);
            write_file(base + ".depth", f.depth.data(), f.depth.size());
            write_file(base + ".grid", f.grid.data(), kGridBytes);
            write_file(base + ".desc", f.desc.data(), 4 * kDescU32);
            std::string cj = "null";
            if (f.version == 2) {
                if (composited) {
                    write_file(base + ".fbpl", pl_copy.data(), pl_copy.size() * 4);
                    write_file(base + ".rgb", f.rgb.data(), f.rgb.size());
                } else if (mini_applied) {
                    write_file(base + ".fbpl", pl_copy.data(), pl_copy.size() * 4);
                }
                cj = fmt("{\"mode_on\":%u,\"composited\":%d,\"pip\":[%u,%u,%u],\"width\":%u,\"height\":%u,"
                         "\"pip_drawn\":%d,\"mutant\":%d}", f.colour_flags & 1, composited ? 1 : 0, f.pip_x, f.pip_y,
                         f.pip_div, f.cw, f.ch, pip_drawn ? 1 : 0, o.colour_mutant);
                if (o.mini_scale != 1)
                    cj.insert(cj.size() - 1, fmt(",\"mini2x\":%u", (f.colour_flags & kColourMini2x) ? 1u : 0u));
            }
            std::string rj = "[";
            for (int i = 0; i < kRegs; ++i) rj += fmt("%s%u", i ? "," : "", regs[i]);
            rj += "]";
            // --mini-scale 1: the meta bytes as before
            const std::string mj = o.mini_scale == 1 ? std::string() :
                fmt(",\"mini\":{\"scale\":%u,\"applied\":%d,\"mutant\":%d}", o.mini_scale, mini_applied ? 1 : 0,
                    o.colour_mutant);
            const std::string meta = fmt("{\"seq\":%ld,\"kind\":\"%s\",\"frame_id\":%llu,\"status\":%u,\"regs\":%s,"
                                         "\"permit_before\":%d,\"permit_after\":%d,\"width\":%u,\"height\":%u,"
                                         "\"colour\":%s%s}",
                                         seq, kind, (unsigned long long)f.frame_id, status, rj.c_str(), permit_before,
                                         permit_after, W, H, cj.c_str(), mj.c_str());
            write_file(base + ".json", meta.data(), meta.size());
            dumped = base;
        }
        tot_wait_eintr += run_info.eintr;
        tot_flip_eintr += flip_eintr;
        tot_stale_irq += run_info.stale_irq;
        // relay 5: only when non-zero, so the event bytes of an undisturbed render are unchanged
        std::string extra;
        if (run_info.eintr) extra += fmt(",\"wait_eintr\":%d", run_info.eintr);
        if (flip_eintr) extra += fmt(",\"flip_eintr\":%d", flip_eintr);
        if (run_info.stale_irq) extra += fmt(",\"stale_irq\":%d", run_info.stale_irq);
        if (run_info.late_done) extra += fmt(",\"late_done\":%d", run_info.late_done);
        if (rs.bits) extra += fmt(",\"resync\":%d,\"resync_wait_us\":%llu", rs.bits, (unsigned long long)(rs.wait_ns / 1000));
        if (flip_drained) extra += fmt(",\"flip_drained\":%d", flip_drained);
        // --mini-scale 1: the event bytes as before
        if (o.mini_scale != 1) extra += fmt(",\"mini\":%d,\"mini_ns\":%llu", mini_applied ? 1 : 0, (unsigned long long)mini_ns);
        return fmt("{\"ev\":\"%s\",\"seq\":%ld,\"frame_id\":%llu,\"status\":%d,\"done\":%d,\"flipped\":%d,\"ar_ns\":%llu,"
                   "\"handoff_ns\":%lld,\"prep_ns\":%llu,\"flip_wait_ns\":%llu,\"interval_ms\":%.3f,\"copy_ms\":%.3f,"
                   "\"age_ms\":%u,\"age_valid\":%u,\"replay\":%u,\"replay_core\":%u,\"frame_valid\":%u,"
                   "\"mode\":%s,\"mode_name\":%s,\"permit_before\":%d,\"permit_after\":%d,"
                   "\"since_frame_ms\":%.3f,\"flip_mono_ns\":%llu,\"dump\":%s,"
                   "\"colour_mode\":%u,\"colour\":%d,\"pip\":%d,\"composite_ns\":%llu%s}",
                   kind, seq, (unsigned long long)f.frame_id, static_cast<int>(status), done ? 1 : 0, flipped ? 1 : 0,
                   (unsigned long long)ar_ns, f.valid ? (long long)(t_age - f.recv_mono) : -1LL,
                   (unsigned long long)(t_age - t_begin), (unsigned long long)(t_flip - t_core), interval_ms, copy_ms,
                   regs[kRegAgeMs], regs[kRegAgeValid], replay_data, regs[kRegReplay], regs[kRegFrameValid],
                   mode < 0 ? "null" : std::to_string(mode).c_str(), mname ? jstr(mname).c_str() : "null",
                   permit_before, permit_after, f.valid ? (t_flip - f.recv_mono) / 1e6 : -1.0, (unsigned long long)t_flip,
                   dumped.empty() ? "null" : jstr(dumped).c_str(),
                   f.version == 2 ? (f.colour_flags & 1) : 0u, composited ? 1 : 0, pip_drawn ? 1 : 0,
                   (unsigned long long)composite_ns, extra.c_str());
    };

    // Startup picture: no frame yet -> NODATA (all-invalid depth, fresh controls, no boxes).
    {
        Frame z;
        z.depth.assign(8 * 2, 0);
        z.grid.assign(kGridBytes, 0);
        z.desc.assign(4 * kDescU32, 0);
        const std::uint32_t m = FPGA_M7_AR_MAGIC;
        std::memcpy(z.desc.data(), &m, 4);
        std::uint32_t r[kRegs] = {W, H, 8, 1, 0, 40, 1, 1, 0, 1, max_age_ms, 0, 0, 0, 1, 0, 0, 0, 0, 0, 32, 32,
                                  W >= 136 ? W - 132 : 0u, H >= 192 ? 60u : 0u};
        const std::string ev = present(z, r, "startup", true);
        emit(ev);
    }
    emit(fmt("{\"ev\":\"ready\",\"pid\":%d,\"width\":%u,\"height\":%u,\"refresh_hz\":%u,\"scanout\":\"%s\",\"display\":\"%s\","
             "\"driver\":%s,\"device\":%s,\"stale_ms\":%.1f,\"refresh_ms\":%.1f,\"wait\":\"%s\"}",
             static_cast<int>(getpid()), W, H, hz, scanout, o.display.c_str(), jstr(out.driver).c_str(),
             dev->describe().c_str(), o.stale_ms, o.refresh_ms, o.wait.c_str()));

    bool eof = false;
    InBuf in;
    fcntl(0, F_SETFL, fcntl(0, F_GETFL) | O_NONBLOCK);
    std::uint64_t last_failsafe = 0;
    std::uint64_t frames_rx = 0, superseded = 0;
    while (!g_stop) {
        int timeout_ms = -1;
        const std::uint64_t now = mono_ns();
        if (last.valid) {
            const double since = (now - last.recv_mono) / 1e6;
            if (since < o.stale_ms) timeout_ms = static_cast<int>(std::ceil(o.stale_ms - since)) + 1;
            else timeout_ms = last_failsafe ? std::max(1, static_cast<int>(std::ceil(o.refresh_ms - (now - last_failsafe) / 1e6))) : 0;
        }
        if (eof && timeout_ms < 0) timeout_ms = 1000;
        int pr = 0;
        if (!eof) {
            pollfd p = {0, POLLIN, 0};
            pr = poll(&p, 1, timeout_ms);
            if (pr < 0 && errno == EINTR) continue;
            if (pr == 1 && (p.revents & (POLLIN | POLLHUP | POLLERR))) {
                in.fill(0, eof);
                Frame f, g;
                std::string merr;
                int got = 0;
                for (;;) {      // latest wins: every complete message in the buffer, keep the newest
                    const int r = in.next(g, merr);
                    if (r == 0) break;
                    if (r < 0) {            // protocol error = EOF: the stream position is lost
                        emit(fmt("{\"ev\":\"protocol_error\",\"error\":%s}", jstr(merr).c_str()));
                        eof = true;
                        break;
                    }
                    ++frames_rx;
                    if (got) ++superseded;
                    f = g;
                    got = 1;
                }
                if (got) {
                    max_age_ms = f.regs[kRegMaxAge];
                    last = f;
                    last_failsafe = 0;
                    emit(present(last, last.regs, "frame", true));
                    if (!eof) continue;
                }
                if (eof) emit(fmt("{\"ev\":\"eof\",\"frames_rx\":%llu}", (unsigned long long)frames_rx));
                continue;
            }
        } else {
            pollfd none = {-1, 0, 0};
            poll(&none, 1, timeout_ms);
        }
        if (g_stop) break;
        // Fail-safe: no new frame for stale_ms -> STALE picture from the last inputs with
        // the age grown by the time since the frame arrived (Nano basis + KV, live or replay
        // alike; no basis: KV time only). stale_ms >= max_age_ms so the core is in mode 2.
        if (last.valid) {
            const std::uint64_t t = mono_ns();
            const double since = (t - last.recv_mono) / 1e6;
            if (since >= o.stale_ms && (!last_failsafe || (t - last_failsafe) / 1e6 >= o.refresh_ms)) {
                std::uint32_t r[kRegs];
                std::memcpy(r, last.regs, sizeof r);
                Frame fs = last;
                if (fs.age_basis < 0) fs.age_basis = 0;
                r[kRegReplay] = 0;
                r[kRegAgeValid] = 1;
                const std::string ev = present(fs, r, "failsafe", false);
                ++failsafe_count;
                last_failsafe = t;
                emit(ev);
            }
        }
    }
    if (o.display == "drm" && out.fd >= 0) xioctl(out.fd, DRM_IOCTL_DROP_MASTER, nullptr);
    std::string tot;
    if (tot_wait_eintr) tot += fmt(",\"wait_eintr\":%ld", tot_wait_eintr);
    if (tot_flip_eintr) tot += fmt(",\"flip_eintr\":%ld", tot_flip_eintr);
    if (tot_stale_irq) tot += fmt(",\"stale_irq\":%ld", tot_stale_irq);
    if (tot_resync) tot += fmt(",\"resync\":%ld", tot_resync);
    if (tot_skipped) tot += fmt(",\"resync_skipped\":%ld", tot_skipped);
    emit(fmt("{\"ev\":\"exit\",\"frames_rx\":%llu,\"superseded\":%llu,\"renders\":%ld,\"failsafe\":%ld%s}",
             (unsigned long long)frames_rx, (unsigned long long)superseded, seq, failsafe_count, tot.c_str()));
    return 0;
}
