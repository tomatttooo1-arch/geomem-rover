// M7 PL user-space access layer implementation (see m7pl_dev.hpp).
#include "m7pl_dev.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <map>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "m7pl_addrmap.h"
#include "m7pl_hls_regs.h"
#include "../../ar_overlay.hpp"
#include "../../object_evidence.hpp"

// HLS wrapper C sources, executed by the mock device (and nothing else).
extern "C" std::uint32_t fpga_m7_frame_axi(
    const float*, std::uint8_t*, const float[9], const float[12], double,
    double, double, double, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t*, std::uint32_t*,
    std::uint32_t*);
extern "C" std::uint32_t fpga_m7_object_roi_axi(
    const float*, std::uint32_t, std::uint32_t, std::uint32_t,
    const std::uint32_t[FPGA_M7_ROI_MAX_BOXES], const std::uint32_t[4 * FPGA_M7_ROI_MAX_BOXES],
    const double[FPGA_M7_ROI_MAX_BOXES], const double[FPGA_M7_ROI_MAX_BOXES],
    const std::uint32_t[FPGA_M7_ROI_MAX_BOXES], double, std::uint32_t, double, double,
    std::uint32_t[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES]);
extern "C" std::uint32_t fpga_m7_ar_overlay_axi(
    const fpga_m7_ar_word*, const fpga_m7_ar_word*, const fpga_m7_ar_word*, fpga_m7_ar_word*,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t,
    std::uint32_t, std::uint32_t, std::uint32_t, bool, std::uint32_t, std::uint32_t,
    std::int32_t, std::int32_t, std::uint32_t, std::uint32_t, std::uint32_t, std::uint32_t);

namespace m7pl {

const char* const kBlockName[NBLOCKS] = {"sysinfo", "frame", "ar",
    "permit", "roi"};
const char* const kUioName[NBLOCKS] = {"m7_sysinfo", "m7_frame", "m7_ar",
    "m7_permit", "m7_roi"};
const std::uint32_t kBlockBase[NBLOCKS] = {M7PL_BASE_SYSINFO, M7PL_BASE_FRAME,
    M7PL_BASE_AR, M7PL_BASE_PERMIT, M7PL_BASE_ROI};

std::uint64_t mono_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

void Device::wrf(Block b, std::uint32_t off, float v) {
    std::uint32_t u;
    std::memcpy(&u, &v, 4);
    wr(b, off, u);
}
void Device::wrd(Block b, std::uint32_t off, double v) {
    std::uint64_t u;
    std::memcpy(&u, &v, 8);
    wr64(b, off, u);
}

static void dcache_range(const void* p, std::size_t len, bool invalidate) {
#if defined(__aarch64__)
    std::uint64_t ctr;
    asm volatile("mrs %0, ctr_el0" : "=r"(ctr));
    const std::uintptr_t line = 4u << ((ctr >> 16) & 0xF);
    std::uintptr_t a = reinterpret_cast<std::uintptr_t>(p) & ~(line - 1);
    const std::uintptr_t end = reinterpret_cast<std::uintptr_t>(p) + len;
    // EL0 may issue DC CVAC/CIVAC (Linux sets SCTLR_EL1.UCI); DC IVAC is EL1-only,
    // so "invalidate" uses clean+invalidate. The CPU must not have dirtied the
    // output range while the device was writing it.
    for (; a < end; a += line) {
        if (invalidate) asm volatile("dc civac, %0" ::"r"(a) : "memory");
        else asm volatile("dc cvac, %0" ::"r"(a) : "memory");
    }
    asm volatile("dsb sy" ::: "memory");
#else
    (void)p; (void)len; (void)invalidate;
#endif
}

void Device::to_device(const Buffer& b, std::size_t off, std::size_t len) {
    if (b.cached) dcache_range(static_cast<char*>(b.va) + off, len, false);
    __sync_synchronize();
}
void Device::from_device(const Buffer& b, std::size_t off, std::size_t len) {
    __sync_synchronize();
    if (b.cached) dcache_range(static_cast<char*>(b.va) + off, len, true);
}

// ---------------------------------------------------------------- providers
namespace {

bool read_sysfs_u64(const std::string& path, std::uint64_t& v) {
    FILE* f = std::fopen(path.c_str(), "r");
    if (!f) return false;
    unsigned long long x = 0;
    const int n = std::fscanf(f, "%llx", &x);
    std::fclose(f);
    if (n != 1) return false;
    v = x;
    return true;
}

// A single contiguous region carved with a bump allocator (udmabuf/physmem).
struct Region {
    void* va = nullptr;
    std::uint64_t pa = 0;
    std::size_t size = 0, used = 0;
    bool cached = false;
    std::string provider;
    bool carve(Buffer& out, std::size_t size_req, std::string& err) {
        const std::size_t sz = (size_req + 4095) & ~std::size_t(4095);
        if (used + sz > size) {
            err = provider + ": region exhausted (" + std::to_string(size) + " bytes)";
            return false;
        }
        out.va = static_cast<char*>(va) + used;
        out.pa = pa + used;
        out.size = sz;
        out.cached = cached;
        out.provider = provider;
        used += sz;
        return true;
    }
};

struct dma_heap_allocation_data_compat {  // linux/dma-heap.h (5.6+) UAPI
    std::uint64_t len;
    std::uint32_t fd;
    std::uint32_t fd_flags;
    std::uint64_t heap_flags;
};
#define M7PL_DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_allocation_data_compat)

bool pagemap_phys(void* va, std::size_t size, std::uint64_t& pa, std::string& err) {
    int fd = open("/proc/self/pagemap", O_RDONLY);
    if (fd < 0) { err = "open pagemap: " + std::string(strerror(errno)); return false; }
    const long page = sysconf(_SC_PAGESIZE);
    std::uint64_t first = 0;
    for (std::size_t off = 0; off < size; off += page) {
        const std::uintptr_t v = reinterpret_cast<std::uintptr_t>(va) + off;
        std::uint64_t e = 0;
        if (pread(fd, &e, 8, static_cast<off_t>(v / page) * 8) != 8) {
            err = "pread pagemap"; close(fd); return false;
        }
        if (!(e >> 63)) { err = "page not present"; close(fd); return false; }
        const std::uint64_t pfn = e & ((1ull << 55) - 1);
        if (pfn == 0) { err = "pagemap PFN hidden (need root/CAP_SYS_ADMIN)"; close(fd); return false; }
        const std::uint64_t p = pfn * page;
        if (off == 0) first = p;
        else if (p != first + off) {
            err = "dma-heap buffer is not physically contiguous"; close(fd); return false;
        }
    }
    close(fd);
    pa = first;
    return true;
}

}  // namespace

// ---------------------------------------------------------------- hardware
class HwDevice : public Device {
public:
    explicit HwDevice(const DevOptions& o) : opt_(o) {}
    ~HwDevice() override {
        for (int b = 0; b < NBLOCKS; ++b)
            if (win_[b]) munmap(const_cast<std::uint32_t*>(win_[b]), M7PL_WINDOW_BYTES);
        if (region_.va) munmap(region_.va, region_.size);
        if (memfd_ >= 0) close(memfd_);
        if (regfd_ >= 0) close(regfd_);
    }
    bool open_all(std::string& err) {
        memfd_ = open("/dev/mem", O_RDWR | O_SYNC);
        if (memfd_ < 0) { err = "open /dev/mem: " + std::string(strerror(errno)) + " (run as root)"; return false; }
        for (int b = 0; b < NBLOCKS; ++b) {
            void* p = mmap(nullptr, M7PL_WINDOW_BYTES, PROT_READ | PROT_WRITE,
                           MAP_SHARED, memfd_, kBlockBase[b]);
            if (p == MAP_FAILED) { err = std::string("mmap ") + kBlockName[b] + ": " + strerror(errno); return false; }
            win_[b] = static_cast<volatile std::uint32_t*>(p);
        }
        return pick_provider(err);
    }
    std::uint32_t rd(Block b, std::uint32_t off) override { return win_[b][off / 4]; }
    void wr(Block b, std::uint32_t off, std::uint32_t v) override { win_[b][off / 4] = v; }
    bool alloc(Buffer& out, std::size_t size, std::string& err) override {
        bool ok;
        if (kind_ == "dmaheap") ok = heap_alloc(out, size, err);
        else ok = region_.carve(out, size, err);
        if (ok && out.pa + out.size > kDmaLimit) {
            err = "buffer at 0x" + std::to_string(out.pa) + " is outside the PL DMA window (low 2 GiB)";
            return false;
        }
        return ok;
    }
    void release(Buffer& b) override {
        if (kind_ == "dmaheap" && b.va) {
            munmap(b.va, b.size);
            close(b.dmabuf_fd);
        }
        b = Buffer();
    }
    std::string kind_;

private:
    bool pick_provider(std::string& err) {
        const std::string p = opt_.provider;
        std::string e1, e2, e3;
        if ((p == "auto" || p == "udmabuf") && open_udmabuf(e1)) return true;
        if ((p == "auto" || p == "dmaheap") && open_heap(e2)) return true;
        if (p == "physmem" && open_physmem(e3)) return true;
        err = "no DMA buffer provider: udmabuf[" + e1 + "] dmaheap[" + e2 + "] physmem[" + e3 + "]";
        return false;
    }
    bool open_udmabuf(std::string& err) {
        const std::string n = opt_.udmabuf;
        std::uint64_t pa = 0, size = 0;
        bool found = false;
        for (const char* cls : {"/sys/class/u-dma-buf/", "/sys/class/udmabuf/"}) {
            if (read_sysfs_u64(cls + n + "/phys_addr", pa)) {
                FILE* f = std::fopen((cls + n + "/size").c_str(), "r");
                unsigned long long s = 0;
                if (f) { if (std::fscanf(f, "%llu", &s) != 1) s = 0; std::fclose(f); }
                size = s;
                found = size > 0;
                break;
            }
        }
        if (!found) { err = "no /sys/class/u-dma-buf/" + n; return false; }
        regfd_ = open(("/dev/" + n).c_str(), O_RDWR | O_SYNC);
        if (regfd_ < 0) { err = "open /dev/" + n + ": " + strerror(errno); return false; }
        void* va = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_SHARED, regfd_, 0);
        if (va == MAP_FAILED) { err = "mmap udmabuf: " + std::string(strerror(errno)); return false; }
        region_.va = va; region_.pa = pa; region_.size = size; region_.cached = false;
        region_.provider = "udmabuf";
        kind_ = "udmabuf";
        return true;
    }
    bool open_heap(std::string& err) {
        heapfd_ = open(("/dev/dma_heap/" + opt_.heap).c_str(), O_RDWR);
        if (heapfd_ < 0) { err = "open /dev/dma_heap/" + opt_.heap + ": " + strerror(errno); return false; }
        kind_ = "dmaheap";
        return true;
    }
    bool open_physmem(std::string& err) {
        if (!opt_.phys_size) { err = "--phys-base/--phys-size required"; return false; }
        void* va = mmap(nullptr, opt_.phys_size, PROT_READ | PROT_WRITE, MAP_SHARED, memfd_,
                        static_cast<off_t>(opt_.phys_base));
        if (va == MAP_FAILED) { err = "mmap /dev/mem physmem: " + std::string(strerror(errno)); return false; }
        region_.va = va; region_.pa = opt_.phys_base; region_.size = opt_.phys_size;
        region_.cached = false; region_.provider = "physmem";
        kind_ = "physmem";
        return true;
    }
    bool heap_alloc(Buffer& out, std::size_t size, std::string& err) {
        dma_heap_allocation_data_compat d;
        std::memset(&d, 0, sizeof d);
        d.len = (size + 4095) & ~std::size_t(4095);
        d.fd_flags = O_RDWR | O_CLOEXEC;
        if (ioctl(heapfd_, M7PL_DMA_HEAP_IOCTL_ALLOC, &d) < 0) {
            err = "DMA_HEAP_IOCTL_ALLOC: " + std::string(strerror(errno)); return false;
        }
        void* va = mmap(nullptr, d.len, PROT_READ | PROT_WRITE, MAP_SHARED, d.fd, 0);
        if (va == MAP_FAILED) { err = "mmap dma-buf: " + std::string(strerror(errno)); close(d.fd); return false; }
        std::memset(va, 0, d.len);  // fault every page in before pagemap
        std::uint64_t pa = 0;
        if (!pagemap_phys(va, d.len, pa, err)) { munmap(va, d.len); close(d.fd); return false; }
        out.va = va; out.pa = pa; out.size = d.len; out.dmabuf_fd = static_cast<int>(d.fd);
        out.cached = true; out.provider = "dmaheap";
        dcache_range(va, d.len, false);  // push the zeroing out before any DMA
        return true;
    }
    DevOptions opt_;
    int memfd_ = -1, regfd_ = -1, heapfd_ = -1;
    volatile std::uint32_t* win_[NBLOCKS] = {};
    Region region_;
};

// ---------------------------------------------------------------- mock
// Host-side stand-in: registers are plain memory; writing ap_start decodes
// the HLS register map (lo word at ADDR, hi at ADDR+4; float/double arrays
// byte-linear from *_BASE) and runs the SAME wrapper C function the HLS
// export was built from. It validates the driver's marshaling and flow; it is
// not evidence about the RTL or the board.
class MockDevice : public Device {
public:
    MockDevice() : t0_(mono_ns()) {
        for (auto& r : regs_) r.assign(M7PL_WINDOW_BYTES / 4, 0);
        regs_[SYSINFO][0] = 0x4D37504C; regs_[SYSINFO][1] = 0x00010000;
        regs_[SYSINFO][4] = 100000000; regs_[SYSINFO][8] = 0x363;
        // AR2NANO2: M7PL_MOCK_SLOT_MAP (hex) stands in for a FPGA_M7_PL_NO_AR=1 bitstream (0x263) on the host
        if (const char* sm = std::getenv("M7PL_MOCK_SLOT_MAP")) regs_[SYSINFO][8] = std::strtoul(sm, nullptr, 16);
        // RTL blocks: identity registers only (MAGIC, build parameters); their
        // behaviour is not emulated. PERMIT STATUS stays 0 = no permit, which
        // is also what the mock feeds to the AR core's permit_i wire.
        regs_[PERMIT][0] = 0x4D37504D; regs_[PERMIT][0x44 / 4] = 100000000; regs_[PERMIT][0x48 / 4] = 500000;
        regs_[PERMIT][0x58 / 4] = 10000000; regs_[PERMIT][0x5C / 4] = 500000000;  // window min/max (v3fc2)
        for (int b = 0; b < NBLOCKS; ++b) if (is_hls_core(b)) regs_[b][0] = 0x4;  // ap_idle
    }
    ~MockDevice() override { for (auto& kv : mem_) std::free(kv.second.first); }
    bool is_mock() const override { return true; }
    std::uint32_t rd(Block b, std::uint32_t off) override {
        if (b == SYSINFO && off == 0x14) {
            const std::uint64_t c = (mono_ns() - t0_) / 10;
            regs_[b][6] = static_cast<std::uint32_t>(c >> 32);
            return static_cast<std::uint32_t>(c);
        }
        std::uint32_t v = regs_[b][off / 4];
        if (off == 0 && is_hls_core(b)) regs_[b][0] &= ~0x2u;  // ap_done COR
        return v;
    }
    void wr(Block b, std::uint32_t off, std::uint32_t v) override {
        if (b == SYSINFO && off != 0x1C) return;
        if (is_hls_core(b) && off == 0 && (v & 1)) { regs_[b][0] = 0x4; execute(b); return; }
        regs_[b][off / 4] = v;
    }
    bool alloc(Buffer& out, std::size_t size, std::string& err) override {
        const std::size_t sz = (size + 4095) & ~std::size_t(4095);
        void* p = nullptr;
        if (posix_memalign(&p, 4096, sz)) { err = "posix_memalign"; return false; }
        std::memset(p, 0, sz);
        out.va = p; out.pa = next_pa_; out.size = sz; out.provider = "mock";
        mem_[next_pa_] = {p, sz};
        next_pa_ += sz + 0x10000;
        return true;
    }
    void release(Buffer& b) override {
        auto it = mem_.find(b.pa);
        if (it != mem_.end()) { std::free(it->second.first); mem_.erase(it); }
        b = Buffer();
    }
    // Test hook: corrupt one output byte after the next core run.
    long corrupt_next_ = -1;

private:
    std::uint64_t r64(Block b, std::uint32_t off) {
        return regs_[b][off / 4] | (static_cast<std::uint64_t>(regs_[b][off / 4 + 1]) << 32);
    }
    double rdbl(Block b, std::uint32_t off) { double d; std::uint64_t u = r64(b, off); std::memcpy(&d, &u, 8); return d; }
    void* va(std::uint64_t pa) {
        for (auto& kv : mem_)
            if (pa >= kv.first && pa < kv.first + kv.second.second)
                return static_cast<char*>(kv.second.first) + (pa - kv.first);
        std::fprintf(stderr, "MOCK: device access to unmapped physical 0x%llx\n", (unsigned long long)pa);
        std::abort();
    }
    void execute(Block b) {
        std::uint32_t* R = regs_[b].data();
        if (b == FRAME) {
            float k[9], pose[12];
            std::memcpy(k, &R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_INVERSE_K_BASE / 4], sizeof k);
            std::memcpy(pose, &R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_WORLD_CAMERA_BASE / 4], sizeof pose);
            std::uint32_t a = 0, c = 0, o = 0;
            std::uint8_t* flags = static_cast<std::uint8_t*>(va(r64(b, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_FLAGS_DATA)));
            const std::uint32_t st = fpga_m7_frame_axi(
                static_cast<const float*>(va(r64(b, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_DEPTH_M_DATA))), flags, k, pose,
                rdbl(b, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_GRID_ORIGIN_X_DATA),
                rdbl(b, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_GRID_ORIGIN_Y_DATA),
                rdbl(b, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_GRID_ORIGIN_Z_DATA),
                rdbl(b, XFPGA_M7_FRAME_AXI_CONTROL_ADDR_VOXEL_SIZE_M_DATA),
                R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_WIDTH_DATA / 4], R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_HEIGHT_DATA / 4],
                R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NX_DATA / 4], R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NY_DATA / 4],
                R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_NZ_DATA / 4], &a, &c, &o);
            R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_AP_RETURN / 4] = st;
            R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_COUNT_ACCEPTED_DATA / 4] = a;
            R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_COUNT_INVALID_DATA / 4] = c;
            R[XFPGA_M7_FRAME_AXI_CONTROL_ADDR_COUNT_OUTSIDE_DATA / 4] = o;  // = clipped rays (被裁剪)
            if (corrupt_next_ >= 0) { flags[corrupt_next_] ^= 0x3; corrupt_next_ = -1; }
        } else if (b == AR) {
            // 128-bit beats packed with ap_uint::range() from the little-endian
            // DDR bytes (never relying on the ap_uint object layout).
            const std::uint32_t w = R[XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_WIDTH_DATA / 4];
            const std::uint32_t h = R[XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_HEIGHT_DATA / 4];
            const std::uint32_t dw = R[XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_WIDTH_DATA / 4];
            const std::uint32_t dh = R[XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_HEIGHT_DATA / 4];
            const bool dims_ok = w && h && w <= FPGA_M7_AR_MAX_WIDTH && h <= FPGA_M7_AR_MAX_HEIGHT &&
                                 dw <= FPGA_M7_AR_MAX_WIDTH && dh <= FPGA_M7_AR_MAX_HEIGHT;
            const std::size_t dwords = dims_ok ? (static_cast<std::size_t>(dw) * dh + 7) / 8 : 0;
            const std::size_t fwords = dims_ok ? (static_cast<std::size_t>(w) * h + 3) / 4 : 0;
            std::vector<fpga_m7_ar_word> dep(dwords + 1, 0), grid(FPGA_M7_AR_GRID_WORDS, 0),
                box(FPGA_M7_AR_DESC_WORDS, 0), fb(fwords + 1, 0);
            auto load = [&](std::vector<fpga_m7_ar_word>& v, std::size_t words, std::uint32_t reg) {
                if (!words) return;
                const std::uint8_t* p = static_cast<const std::uint8_t*>(va(r64(b, reg)));
                for (std::size_t i = 0; i < words * 16; ++i) v[i / 16].range(8 * (i % 16) + 7, 8 * (i % 16)) = p[i];
            };
            const bool grid_on = R[XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_GRID_VALID_DATA / 4] == 1;
            load(dep, dwords, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_DATA);
            load(grid, grid_on ? FPGA_M7_AR_GRID_WORDS : 0, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_GRID_DATA);
            load(box, dims_ok ? FPGA_M7_AR_DESC_WORDS : 0, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_BOXES_DATA);
            auto reg = [&](std::uint32_t off) { return R[off / 4]; };
            const std::uint32_t st = fpga_m7_ar_overlay_axi(
                dep.data(), grid.data(), box.data(), fb.data(), w, h, dw, dh,
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_DX_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_DEPTH_DY_DATA),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_FRAME_VALID_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_POSE_VALID_DATA),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_AGE_MS_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_AGE_VALID_DATA),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_MAX_AGE_MS_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_REPLAY_DATA),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_OBJECT_EVIDENCE_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_FRAME_ID_DATA),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_PERMIT_PRESENT_DATA),
                (regs_[PERMIT][0x10 / 4] & 1u) != 0,  // permit_i wire <- permit/permit_o
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_GRID_VALID_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_SLICE_Y_DATA),
                static_cast<std::int32_t>(reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_SLICE_LO_MM_DATA)),
                static_cast<std::int32_t>(reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_SLICE_HI_MM_DATA)),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_CAM_CELL_X_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_CAM_CELL_Z_DATA),
                reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_MINI_X_DATA), reg(XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_MINI_Y_DATA));
            R[XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_AP_RETURN / 4] = st;
            if (st != 1 && fwords) {
                std::uint32_t* px = static_cast<std::uint32_t*>(va(r64(b, XFPGA_M7_AR_OVERLAY_AXI_CONTROL_ADDR_FB_DATA)));
                const std::size_t n = static_cast<std::size_t>(w) * h;
                for (std::size_t i = 0; i < n; ++i) px[i] = fb[i / 4].range(32 * (i % 4) + 31, 32 * (i % 4)).to_uint();
                if (corrupt_next_ >= 0) { px[corrupt_next_] ^= 0x10; corrupt_next_ = -1; }
            }
        } else if (b == ROI) {
            // Box tables are s_axilite register blocks (byte-linear from *_BASE);
            // the result block is an OUTPUT register block that keeps its old
            // contents when the core returns nonzero status (no row written).
            std::uint32_t en[FPGA_M7_ROI_MAX_BOXES], xyxy[4 * FPGA_M7_ROI_MAX_BOXES], oid[FPGA_M7_ROI_MAX_BOXES];
            double nz[FPGA_M7_ROI_MAX_BOXES], fz[FPGA_M7_ROI_MAX_BOXES];
            std::uint32_t res[M7_RB_FIELDS * FPGA_M7_ROI_MAX_BOXES];
            std::memcpy(en, &R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_BOX_ENABLED_BASE / 4], sizeof en);
            std::memcpy(xyxy, &R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_BOX_XYXY_BASE / 4], sizeof xyxy);
            std::memcpy(oid, &R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_OBJECT_ID_BASE / 4], sizeof oid);
            std::memcpy(nz, &R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_NEAR_Z_BASE / 4], sizeof nz);
            std::memcpy(fz, &R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_FAR_Z_BASE / 4], sizeof fz);
            std::memcpy(res, &R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_RESULT_BASE / 4], sizeof res);
            const std::uint32_t w = R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_WIDTH_DATA / 4];
            const std::uint32_t h = R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_HEIGHT_DATA / 4];
            const bool dims_ok = w && h && w <= FPGA_M7_OBJECT_MAX_WIDTH && h <= FPGA_M7_OBJECT_MAX_HEIGHT;
            const float* depth = dims_ok ? static_cast<const float*>(va(r64(b, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_DEPTH_DATA)))
                                         : nullptr;
            const std::uint32_t st = fpga_m7_object_roi_axi(
                depth, w, h, R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_BOX_COUNT_DATA / 4], en, xyxy, nz, fz, oid,
                rdbl(b, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_DEPTH_MARGIN_M_DATA),
                R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_MIN_PIXELS_DATA / 4],
                rdbl(b, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_MIN_VALID_FRACTION_DATA),
                rdbl(b, XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_FREE_FRACTION_DATA), res);
            R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_AP_RETURN / 4] = st;
            if (corrupt_next_ >= 0) { res[corrupt_next_] ^= 1; corrupt_next_ = -1; }
            std::memcpy(&R[XFPGA_M7_OBJECT_ROI_AXI_CONTROL_ADDR_RESULT_BASE / 4], res, sizeof res);
        }
        regs_[b][0] = 0x4 | 0x2;  // idle + done
    }
    std::vector<std::uint32_t> regs_[NBLOCKS];
    std::map<std::uint64_t, std::pair<void*, std::size_t>> mem_;
    std::uint64_t next_pa_ = 0x10000000ull;
    std::uint64_t t0_;
};

Device* open_device(const DevOptions& o, std::string& err) {
    if (o.provider == "mock") return new MockDevice();
    HwDevice* d = new HwDevice(o);
    if (!d->open_all(err)) { delete d; return nullptr; }
    return d;
}

void mock_corrupt_next(Device& d, long byte_index) {
    if (d.is_mock()) static_cast<MockDevice&>(d).corrupt_next_ = byte_index;
}

std::uint64_t pl_cycles_now(Device& d) {
    const std::uint32_t lo = d.rd(SYSINFO, 0x14);
    const std::uint32_t hi = d.rd(SYSINFO, 0x18);
    return (static_cast<std::uint64_t>(hi) << 32) | lo;
}

int run_core(Device& d, Block b, double timeout_s, std::uint64_t* elapsed_ns,
             std::uint64_t* pl_cycles) {
    const std::uint64_t c0 = pl_cycles_now(d);
    const std::uint64_t t0 = mono_ns();
    d.wr(b, 0x0, 0x1);  // ap_start
    const std::uint64_t deadline = t0 + static_cast<std::uint64_t>(timeout_s * 1e9);
    int rc = -1;
    for (;;) {
        const std::uint32_t ctrl = d.rd(b, 0x0);
        if (ctrl & 0x2) { rc = 0; break; }  // ap_done (clear-on-read)
        if (mono_ns() > deadline) break;
    }
    const std::uint64_t t1 = mono_ns();
    const std::uint64_t c1 = pl_cycles_now(d);
    if (elapsed_ns) *elapsed_ns = t1 - t0;
    if (pl_cycles) *pl_cycles = c1 - c0;
    return rc;
}

}  // namespace m7pl
