// M7 PL user-space access layer (2026-09-26): register windows + DMA buffers.
//
// Choice and reason (see M7_PL_INTEGRATION_20260926.md section 5):
//   registers  /dev/mem mmap of the fixed 64 KiB windows from bd/m7_bd.tcl.
//              PL addresses are not RAM, so CONFIG_STRICT_DEVMEM allows it;
//              UIO (generic-uio nodes in the overlay) is optional and only
//              used by the interrupt test.
//   buffers    physically contiguous DDR is needed because the HLS masters
//              use physical addresses (no SMMU mapping is set up). Providers:
//                udmabuf  u-dma-buf kernel module (/dev/<name>, sysfs phys_addr),
//                         opened O_SYNC -> uncached, no cache maintenance needed
//                dmaheap  /dev/dma_heap/<heap> (CMA) + /proc/self/pagemap for
//                         the physical address (root), contiguity verified page
//                         by page; cached mapping -> explicit DC CVAC / CIVAC;
//                         the dma-buf fd can be PRIME-imported into DRM for a
//                         zero-copy scanout buffer
//                physmem  a caller-given physical window through /dev/mem
//                         (only for memory Linux does not own, e.g. mem= or a
//                         boot-time reserved-memory carve-out)
//                mock     host-only: malloc + fake physical addresses; the mock
//                         device executes the same HLS wrapper C code.
//   XRT/zocl is not used: this is a Vivado (not Vitis) design without xclbin
//   metadata, and zocl buffer allocation without an xclbin is unverified.
#ifndef M7PL_DEV_HPP
#define M7PL_DEV_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace m7pl {

// Control windows (bd/m7_bd.tcl, merge build 2026-10-11). AR replaced the
// legacy display core; ROI is the E2 region check. v3a (2026-10-26): the
// legacy object core (slot 2) is removed from the design and from the driver.
// v3fc (2026-10-26): the PL watchdog (slot 4) and encoder UART (slot 7) are
// removed as well. The rolling core (slot 5) has its own host-side driver.
enum Block { SYSINFO = 0, FRAME, AR, PERMIT, ROI, NBLOCKS };
extern const char* const kBlockName[NBLOCKS];
extern const char* const kUioName[NBLOCKS];   // overlay node / UIO name
extern const std::uint32_t kBlockBase[NBLOCKS];
// HLS cores (ap_ctrl_hs at offset 0); the others are hand-written RTL.
inline bool is_hls_core(int b) { return b == FRAME || b == AR || b == ROI; }

// Upper bound of the DMA masters' address map (low DDR only, bd/m7_bd.tcl).
constexpr std::uint64_t kDmaLimit = 0x80000000ull;

struct Buffer {
    void* va = nullptr;
    std::uint64_t pa = 0;
    std::size_t size = 0;
    int dmabuf_fd = -1;       // dmaheap only
    bool cached = false;      // needs explicit cache maintenance
    std::string provider;
};

class Device {
public:
    virtual ~Device() {}
    virtual std::uint32_t rd(Block b, std::uint32_t off) = 0;
    virtual void wr(Block b, std::uint32_t off, std::uint32_t v) = 0;
    void wr64(Block b, std::uint32_t off, std::uint64_t v) {
        wr(b, off, static_cast<std::uint32_t>(v));
        wr(b, off + 4, static_cast<std::uint32_t>(v >> 32));
    }
    void wrf(Block b, std::uint32_t off, float v);
    void wrd(Block b, std::uint32_t off, double v);
    virtual bool is_mock() const { return false; }
    // Buffers
    virtual bool alloc(Buffer& out, std::size_t size, std::string& err) = 0;
    virtual void release(Buffer& b) = 0;
    // CPU wrote -> device will read / device wrote -> CPU will read.
    void to_device(const Buffer& b, std::size_t off, std::size_t len);
    void from_device(const Buffer& b, std::size_t off, std::size_t len);
};

struct DevOptions {
    std::string provider = "auto";   // auto|udmabuf|dmaheap|physmem|mock
    std::string udmabuf = "udmabuf0";
    std::string heap = "linux,cma";
    std::uint64_t phys_base = 0;
    std::uint64_t phys_size = 0;
};

// Returns nullptr and sets err on failure.
Device* open_device(const DevOptions& o, std::string& err);

// Run one HLS core: optional IER/GIE, ap_start, poll ap_done with timeout.
// Returns 0 on done, -1 on timeout. elapsed_ns / pl_cycles are measured
// around the start->done window (PL cycles from the sysinfo counter).
int run_core(Device& d, Block b, double timeout_s, std::uint64_t* elapsed_ns,
             std::uint64_t* pl_cycles);
std::uint64_t pl_cycles_now(Device& d);
// Mock only: flip bits in one output byte/word of the next core run, so the
// comparator's failure path can be exercised (positive control).
void mock_corrupt_next(Device& d, long index);
std::uint64_t mono_ns();

}  // namespace m7pl
#endif
