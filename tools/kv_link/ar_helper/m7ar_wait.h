// Signal-safe waits and the core resync of m7ar_present (2026-10-16, relay 5).
//
// Why (docs/design/AR_COLOUR_MAIN_20261016.md 11.5): the helper waited for the AR core's
// ap_done with one bare poll(irqfd). SIGUSR1 (dump request, also sent by ar_tune.sh shot/dump)
// interrupts poll() with EINTR whatever SA_RESTART says, the render was declared failed while
// the core was still computing, and the completion of that frame (ap_done latched, ISR set, a
// UIO event pending) was then taken as the completion of the next frame: from then on every
// render "finished" in ~47 us with status 0 and showed a half-written picture, until exit.
//
// The fix has three parts, all here so tests/test_ar_helper_wait.py can build them on the host
// against a simulated core (tools/kv_link/ar_helper/m7ar_wait_selftest.cpp):
//   * wait_readable: poll until readable or a CLOCK_MONOTONIC deadline; EINTR -> recompute the
//     remaining time and poll again (counted), never re-arm, never restart the core;
//   * resync_core (before every render, before the DMA inputs are rewritten): wait until the core
//     is idle (bounded), consume a latched ap_done (clear-on-read), clear ISR (toggle on write)
//     BEFORE draining the UIO event count (a level line still asserted would re-fire as soon as
//     run re-arms it), drain the UIO count with a zero-timeout poll. Still busy at the deadline
//     -> the render fails without touching a register;
//   * run_core: arm, start, then wait; an event that arrives without ap_done (an interrupt of
//     the previous run that landed after the drain) is consumed and the line re-armed, the wait
//     continues to the same deadline; at the deadline ap_done is read once more (an interrupt
//     lost in that re-arm window still completes the frame).
// Hw (template) provides: uint32_t rd(off), void wr(off, v), bool irq() (false = poll mode),
// int irq_fd(), bool irq_arm(), bool irq_read() (consume the pending count), void pause_us(us).
#ifndef M7AR_WAIT_H
#define M7AR_WAIT_H
#include <cerrno>
#include <cstdint>
#include <ctime>
#include <poll.h>

namespace m7ar {

const std::uint32_t kApCtrl = 0x00, kGie = 0x04, kIer = 0x08, kIsr = 0x0c;
const std::uint32_t kApDone = 0x2, kApIdle = 0x4;

inline std::uint64_t now_ns() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<std::uint64_t>(ts.tv_sec) * 1000000000ull + ts.tv_nsec;
}

// 1 readable, 0 deadline passed, -1 poll error (errno kept). *eintr += signals that hit the wait.
inline int wait_readable(int fd, std::uint64_t deadline_ns, int* eintr) {
    for (;;) {
        const std::uint64_t now = now_ns();
        const std::uint64_t left = deadline_ns > now ? deadline_ns - now : 0;
        const int ms = static_cast<int>((left + 999999ull) / 1000000ull);   // ceil: never wake early
        pollfd p = {fd, POLLIN, 0};
        const int pr = poll(&p, 1, ms);
        if (pr < 0 && errno == EINTR) { if (eintr) ++*eintr; continue; }
        if (pr < 0) return -1;
        if (pr > 0) return (p.revents & POLLIN) ? 1 : -1;
        if (now_ns() >= deadline_ns) return 0;   // a rounding-short poll: wait out the rest
    }
}

// Zero-timeout readiness check (EINTR retried).
inline bool readable_now(int fd) {
    for (;;) {
        pollfd p = {fd, POLLIN, 0};
        const int pr = poll(&p, 1, 0);
        if (pr < 0 && errno == EINTR) continue;
        return pr > 0 && (p.revents & POLLIN);
    }
}

// resync bits (event field "resync", only when non-zero)
enum {
    kResyncBusy = 1,       // core not idle at render entry (waited for it)
    kResyncDone = 2,       // a latched ap_done consumed
    kResyncIsr = 4,        // ISR was set, cleared
    kResyncIrq = 8,        // a pending UIO event drained
    kResyncNeverIdle = 16, // still busy at the deadline: render skipped, no register written
    kResyncFlip = 32,      // a stale DRM event drained before PAGE_FLIP (set by the helper)
};

struct ResyncInfo {
    int bits = 0;
    std::uint64_t wait_ns = 0;   // time spent waiting for idle
};

template <class Hw>
bool resync_core(Hw& hw, std::uint64_t idle_deadline_ns, ResyncInfo* info) {
    int bits = 0;
    const std::uint64_t t0 = now_ns();
    std::uint32_t ctrl = hw.rd(kApCtrl);            // reading also clears a latched ap_done
    if (ctrl & kApDone) bits |= kResyncDone;
    if (!(ctrl & kApIdle)) {
        bits |= kResyncBusy;
        for (;;) {
            if (now_ns() >= idle_deadline_ns) {
                if (info) { info->bits = bits | kResyncNeverIdle; info->wait_ns = now_ns() - t0; }
                return false;
            }
            hw.pause_us(200);
            ctrl = hw.rd(kApCtrl);
            if (ctrl & kApDone) bits |= kResyncDone;
            if (ctrl & kApIdle) break;
        }
        hw.pause_us(200);   // let the interrupt of that completion reach the UIO count before the drain
    }
    if (hw.rd(kApCtrl) & kApDone) bits |= kResyncDone;
    const std::uint32_t isr = hw.rd(kIsr);
    if (isr) { hw.wr(kIsr, isr); bits |= kResyncIsr; }
    if (hw.irq()) {
        while (readable_now(hw.irq_fd())) {
            if (!hw.irq_read()) break;
            bits |= kResyncIrq;
        }
    }
    if (info) { info->bits = bits; info->wait_ns = (bits & kResyncBusy) ? now_ns() - t0 : 0; }
    return true;
}

struct RunInfo {
    int eintr = 0;       // signals that interrupted the wait (the wait went on)
    int stale_irq = 0;   // events without ap_done (consumed, line re-armed)
    int late_done = 0;   // ap_done found at the deadline without an event
};

// Start the core (registers already written) and wait for ap_done until deadline_ns.
template <class Hw>
bool run_core(Hw& hw, std::uint64_t deadline_ns, RunInfo* info) {
    RunInfo r;
    bool ok = false;
    if (hw.irq()) {
        if (!hw.irq_arm()) { if (info) *info = r; return false; }
        hw.wr(kGie, 1);
        hw.wr(kIer, 1);
        hw.wr(kApCtrl, 1);
        for (;;) {
            const int pr = wait_readable(hw.irq_fd(), deadline_ns, &r.eintr);
            if (pr != 1) {
                if (pr == 0 && (hw.rd(kApCtrl) & kApDone)) { ok = true; ++r.late_done; }
                break;
            }
            if (!hw.irq_read()) break;
            if (hw.rd(kApCtrl) & kApDone) { ok = true; break; }   // consumes ap_done
            ++r.stale_irq;
            if (!hw.irq_arm()) break;
        }
        hw.wr(kIsr, hw.rd(kIsr));   // toggle-on-write clear
    } else {
        hw.wr(kApCtrl, 1);
        for (;;) {
            if (hw.rd(kApCtrl) & kApDone) { ok = true; break; }
            if (now_ns() > deadline_ns) break;
        }
    }
    if (info) *info = r;
    return ok;
}

}  // namespace m7ar
#endif
