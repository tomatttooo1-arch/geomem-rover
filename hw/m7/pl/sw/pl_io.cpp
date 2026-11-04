// DRV5 访问层实现（见 pl_io.hpp）。只用 Linux 系统调用。
#include "pl_io.hpp"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace plio {

std::uint64_t mono_ns() {
  timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (std::uint64_t)t.tv_sec * 1000000000ull + (std::uint64_t)t.tv_nsec;
}

std::string find_uio(const std::string& name) {
  if (name.empty()) return "";
  DIR* d = opendir("/sys/class/uio");
  if (!d) return "";
  std::string hit;
  while (dirent* e = readdir(d)) {
    if (std::strncmp(e->d_name, "uio", 3)) continue;
    std::string p = std::string("/sys/class/uio/") + e->d_name + "/name";
    FILE* f = std::fopen(p.c_str(), "r");
    if (!f) continue;
    char b[128] = {0};
    if (std::fgets(b, sizeof b, f)) {
      size_t n = std::strlen(b);
      while (n && (b[n - 1] == '\n' || b[n - 1] == ' ')) b[--n] = 0;
      if (name == b) hit = std::string("/dev/") + e->d_name;
    }
    std::fclose(f);
    if (!hit.empty()) break;
  }
  closedir(d);
  return hit;
}

namespace {

// m7pl_dev.cpp dcache_range 同写法：EL0 可发 DC CVAC / CIVAC（Linux 置 SCTLR_EL1.UCI）
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

struct dma_heap_alloc { std::uint64_t len; std::uint32_t fd; std::uint32_t fd_flags; std::uint64_t heap_flags; };
#define PLIO_DMA_HEAP_IOCTL_ALLOC _IOWR('H', 0x0, struct dma_heap_alloc)
// xlnx_dpu 的 CMA 接口（VART 3.0 vart/buffer-object/src/dpu.h；detpost_drv.cpp 同定义）
struct dpcma_req_alloc { size_t size; std::uint64_t phy_addr; size_t capacity; };
struct dpcma_req_free { std::uint64_t phy_addr; size_t capacity; };
struct dpcma_req_sync { std::uint64_t phy_addr; size_t size; int direction; };
#define PLIO_DPU_IOC_MAGIC 'D'
#define PLIO_DPUIOC_CREATE_BO _IOWR(PLIO_DPU_IOC_MAGIC, 1, struct dpcma_req_alloc*)
#define PLIO_DPUIOC_FREE_BO _IOWR(PLIO_DPU_IOC_MAGIC, 2, struct dpcma_req_free*)
#define PLIO_DPUIOC_SYNC_BO _IOWR(PLIO_DPU_IOC_MAGIC, 3, struct dpcma_req_sync*)

bool pagemap_phys(void* va, std::size_t size, std::uint64_t* pa, std::string* why) {
  int fd = ::open("/proc/self/pagemap", O_RDONLY | O_CLOEXEC);
  if (fd < 0) { *why = std::string("open pagemap: ") + strerror(errno); return false; }
  const long page = sysconf(_SC_PAGESIZE);
  std::uint64_t first = 0;
  for (std::size_t off = 0; off < size; off += page) {
    const std::uintptr_t v = reinterpret_cast<std::uintptr_t>(va) + off;
    std::uint64_t e = 0;
    if (pread(fd, &e, 8, (off_t)(v / page) * 8) != 8) { *why = "pread pagemap"; ::close(fd); return false; }
    if (!(e >> 63)) { *why = "page not present"; ::close(fd); return false; }
    const std::uint64_t pfn = e & ((1ull << 55) - 1);
    if (!pfn) { *why = "pagemap PFN hidden (run as root)"; ::close(fd); return false; }
    if (off == 0) first = pfn * page;
    else if (pfn * page != first + off) { *why = "dma-heap buffer not physically contiguous"; ::close(fd); return false; }
  }
  ::close(fd);
  *pa = first;
  return true;
}

class HwBus : public Bus {
 public:
  explicit HwBus(const Options& o) : o_(o) {}
  ~HwBus() override {
    if (regs_) munmap((void*)regs_, kWindow);
    if (mem_fd_ >= 0) ::close(mem_fd_);
    if (uio_fd_ >= 0) ::close(uio_fd_);
    if (heap_fd_ >= 0) ::close(heap_fd_);
    if (dpu_fd_ >= 0) ::close(dpu_fd_);
  }
  bool open(std::string* why) {
    const std::string uio = find_uio(o_.uio_name);
    const bool want_irq = o_.wait == "irq" || (o_.wait == "auto" && !uio.empty());
    if (o_.wait == "irq" && uio.empty()) { *why = "--wait irq but no UIO node named " + o_.uio_name; return false; }
    int fd;
    off_t off;
    if (want_irq) {
      uio_fd_ = ::open(uio.c_str(), O_RDWR | O_CLOEXEC);
      if (uio_fd_ < 0) { *why = "open " + uio + ": " + strerror(errno); return false; }
      fd = uio_fd_; off = 0;     // UIO map 0 = 节点 reg（64 KiB 窗口）
      uio_ = uio;
    } else {
      mem_fd_ = ::open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
      if (mem_fd_ < 0) { *why = std::string("open /dev/mem: ") + strerror(errno) + " (run as root)"; return false; }
      fd = mem_fd_; off = (off_t)o_.base;
    }
    void* p = mmap(nullptr, kWindow, PROT_READ | PROT_WRITE, MAP_SHARED, fd, off);
    if (p == MAP_FAILED) { *why = std::string("mmap regs: ") + strerror(errno); return false; }
    regs_ = (volatile std::uint32_t*)p;
    return pick_provider(why);
  }
  std::uint32_t rd(std::uint32_t off) override { return regs_[off / 4]; }
  void wr(std::uint32_t off, std::uint32_t v) override { regs_[off / 4] = v; }

  bool alloc(Buf& out, std::size_t size, std::string* why) override {
    out = Buf();
    const std::size_t len = (size + 4095) & ~std::size_t(4095);
    if (kind_ == "dpu") {
      dpcma_req_alloc r{len, 0, 0};
      if (ioctl(dpu_fd_, PLIO_DPUIOC_CREATE_BO, &r)) { *why = std::string("DPUIOC_CREATE_BO: ") + strerror(errno); return false; }
      void* va = mmap(nullptr, r.capacity, PROT_READ | PROT_WRITE, MAP_SHARED, dpu_fd_, (off_t)r.phy_addr);
      if (va == MAP_FAILED) { *why = std::string("mmap /dev/dpu: ") + strerror(errno); return false; }
      out.va = va; out.pa = r.phy_addr; out.size = r.capacity; out.cached = true; out.fd = dpu_fd_; out.provider = "dpu";
    } else {
      dma_heap_alloc d;
      std::memset(&d, 0, sizeof d);
      d.len = len;
      d.fd_flags = O_RDWR | O_CLOEXEC;
      if (ioctl(heap_fd_, PLIO_DMA_HEAP_IOCTL_ALLOC, &d) < 0) { *why = std::string("DMA_HEAP_IOCTL_ALLOC: ") + strerror(errno); return false; }
      void* va = mmap(nullptr, d.len, PROT_READ | PROT_WRITE, MAP_SHARED, (int)d.fd, 0);
      if (va == MAP_FAILED) { *why = std::string("mmap dma-buf: ") + strerror(errno); ::close((int)d.fd); return false; }
      std::memset(va, 0, d.len);   // 先把每页都碰到，pagemap 才有物理页
      std::uint64_t pa = 0;
      if (!pagemap_phys(va, d.len, &pa, why)) { munmap(va, d.len); ::close((int)d.fd); return false; }
      out.va = va; out.pa = pa; out.size = d.len; out.cached = true; out.fd = (int)d.fd; out.provider = kind_;
      dcache_range(va, d.len, false);
    }
    if (out.pa + out.size > kDmaLimit) {
      *why = "buffer above 2 GiB: the LPD masters only map LPD_DDR_LOW";
      release(out);
      return false;
    }
    return true;
  }
  void release(Buf& b) override {
    if (b.va) {
      munmap(b.va, b.size);
      if (b.provider == "dpu") { dpcma_req_free r{b.pa, b.size}; ioctl(dpu_fd_, PLIO_DPUIOC_FREE_BO, &r); }
      else if (b.fd >= 0) ::close(b.fd);
    }
    b = Buf();
  }
  void to_device(const Buf& b, std::size_t off, std::size_t len) override {
    if (b.provider == "dpu") { dpcma_req_sync r{b.pa + off, len, 0}; ioctl(dpu_fd_, PLIO_DPUIOC_SYNC_BO, &r); }
    else if (b.cached) dcache_range((char*)b.va + off, len, false);
    __sync_synchronize();
  }
  void from_device(const Buf& b, std::size_t off, std::size_t len) override {
    __sync_synchronize();
    if (b.provider == "dpu") { dpcma_req_sync r{b.pa + off, len, 1}; ioctl(dpu_fd_, PLIO_DPUIOC_SYNC_BO, &r); }
    else if (b.cached) dcache_range((char*)b.va + off, len, true);
  }
  bool start_and_wait(int timeout_ms, std::uint64_t* ns) override {
    bool done = false;
    if (uio_fd_ >= 0) {
      wr(0x04, 1);   // GIE
      wr(0x08, 1);   // IER：只开 ap_done
      std::uint32_t en = 1;
      if (write(uio_fd_, &en, 4) != 4) return false;
      const std::uint64_t t0 = mono_ns();
      wr(0x00, AP_START);
      pollfd pf{uio_fd_, POLLIN, 0};
      if (poll(&pf, 1, timeout_ms) == 1) {
        std::uint32_t cnt;
        done = read(uio_fd_, &cnt, 4) == 4;
      }
      if (ns) *ns = mono_ns() - t0;
      wr(0x0c, 1);   // ISR bit0 写 1 翻转 = 清 ap_done 中断
      (void)rd(0x00); // ap_done 读清
    } else {
      const std::uint64_t t0 = mono_ns();
      const std::uint64_t lim = (std::uint64_t)timeout_ms * 1000000ull;
      wr(0x00, AP_START);
      std::uint64_t t;
      while (!(done = (rd(0x00) & AP_DONE) != 0)) {
        t = mono_ns() - t0;
        if (t > lim) break;
      }
      if (ns) *ns = mono_ns() - t0;
    }
    return done;
  }
  bool irq_mode() const override { return uio_fd_ >= 0; }
  std::string describe() const override {
    char b[200];
    std::snprintf(b, sizeof b, "{\"regs\":\"%s\",\"wait\":\"%s\",\"dma\":\"%s\"}",
                  uio_.empty() ? "devmem" : uio_.c_str(), uio_fd_ >= 0 ? "irq" : "poll", kind_.c_str());
    return b;
  }

 private:
  bool pick_provider(std::string* why) {
    std::string errs;
    std::vector<std::string> order;
    if (o_.provider == "auto") order = {"dmaheap:reserved", "dmaheap:linux,cma", "dpu"};
    else order = {o_.provider};
    for (const auto& p : order) {
      if (p == "dpu") {
        dpu_fd_ = ::open("/dev/dpu", O_RDWR | O_CLOEXEC);
        if (dpu_fd_ >= 0) { kind_ = "dpu"; return true; }
        errs += " dpu[" + std::string(strerror(errno)) + "]";
      } else if (p.compare(0, 8, "dmaheap:") == 0) {
        const std::string dev = "/dev/dma_heap/" + p.substr(8);
        heap_fd_ = ::open(dev.c_str(), O_RDWR | O_CLOEXEC);
        if (heap_fd_ >= 0) { kind_ = p; return true; }
        errs += " " + p + "[" + strerror(errno) + "]";
      } else {
        errs += " unknown provider " + p;
      }
    }
    *why = "no DMA buffer provider:" + errs;
    return false;
  }
  Options o_;
  volatile std::uint32_t* regs_ = nullptr;
  int mem_fd_ = -1, uio_fd_ = -1, heap_fd_ = -1, dpu_fd_ = -1;
  std::string kind_, uio_;
};

}  // namespace

std::unique_ptr<Bus> open_hw(const Options& o, std::string* why) {
  std::unique_ptr<HwBus> b(new HwBus(o));
  if (!b->open(why)) return nullptr;
  return std::unique_ptr<Bus>(b.release());
}

// ------------------------------------------------------------------ mock
MockBus::~MockBus() {
  for (auto& k : blks_) std::free(k.va);
}
std::uint32_t MockBus::rd(std::uint32_t off) {
  std::uint32_t v = regs_[off / 4];
  if (off == 0) regs_[0] &= ~(AP_DONE | AP_READY);   // ap_done / ap_ready 读清
  for (auto c : clear_on_read)
    if (c == off) regs_[off / 4] = 0;
  return v;
}
void MockBus::wr(std::uint32_t off, std::uint32_t v) {
  if (off == 0x0c) { regs_[3] ^= v; return; }   // ISR 写 1 翻转
  regs_[off / 4] = v;
}
bool MockBus::start_and_wait(int, std::uint64_t* ns) {
  const std::uint64_t t0 = mono_ns();
  ++starts;
  regs_[0] = 0;                                   // 忙
  if (hang_next) { hang_next = false; if (ns) *ns = mono_ns() - t0; return false; }
  if (hook_) hook_(*this);
  regs_[0] = AP_IDLE | AP_DONE;
  (void)rd(0);
  if (ns) *ns = mono_ns() - t0;
  return true;
}
bool MockBus::alloc(Buf& out, std::size_t size, std::string*) {
  out = Buf();
  const std::size_t len = (size + 4095) & ~std::size_t(4095);
  void* va = std::calloc(1, len);
  blks_.push_back({next_pa_, len, va});
  out.va = va; out.pa = next_pa_; out.size = len; out.provider = "mock";
  next_pa_ += len + 0x10000;
  return true;
}
void MockBus::release(Buf& b) {
  for (size_t i = 0; i < blks_.size(); ++i)
    if (blks_[i].pa == b.pa) { std::free(blks_[i].va); blks_.erase(blks_.begin() + i); break; }
  b = Buf();
}
void* MockBus::va_of(std::uint64_t pa, std::size_t len) {
  for (auto& k : blks_)
    if (pa >= k.pa && pa + len <= k.pa + k.size) return (char*)k.va + (pa - k.pa);
  return nullptr;
}

}  // namespace plio
