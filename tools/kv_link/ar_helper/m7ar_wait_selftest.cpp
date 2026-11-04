// Host self-test of m7ar_wait.h (built and judged by tests/test_ar_helper_wait.py).
//
// Part 1 (wait_readable, fork-driven so the signal lands on this single-threaded process):
//   signal at 20 ms, pipe written at 100 ms -> 1 at >= 100 ms with eintr >= 1;
//   signal at 20 ms, nothing written, 50 ms deadline -> 0 at >= 50 ms (remaining time recomputed);
//   positive control: a bare poll() under the same signal returns -1/EINTR before 50 ms.
// Part 2 (resync_core + run_core against a simulated HLS core with UIO irq semantics):
//   AP_CTRL ap_done clear-on-read, ap_idle, ap_start latched while busy (starts on completion);
//   ISR toggle-on-write, level line = GIE && IER&ISR; the "UIO" masks the line when it fires
//   (one byte into a pipe = one event) and irq_arm re-enables it (fires again if still high).
//   Each start request is numbered; a render is DESYNCED when run says done but the run that
//   finished is not the one this render started. The old algorithm (9c2c6c0 UioDevice::run,
//   copied below as run_old, no resync) is the positive control in every scenario.
// Output: one JSON line per case.
#include <atomic>
#include <cerrno>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <pthread.h>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "m7ar_wait.h"

namespace {

void on_usr1(int) {}

double ms_since(std::uint64_t t0) { return (m7ar::now_ns() - t0) / 1e6; }

// ------------------------------------------------------------------ part 1
// child: sleep sig_ms, kill(parent, SIGUSR1); if write_ms > 0 sleep until write_ms and write the pipe
pid_t spawn_child(int wfd, int sig_ms, int write_ms) {
    const pid_t p = fork();
    if (p == 0) {
        usleep(static_cast<useconds_t>(sig_ms) * 1000);
        kill(getppid(), SIGUSR1);
        if (write_ms > 0) {
            usleep(static_cast<useconds_t>(write_ms - sig_ms) * 1000);
            if (write(wfd, "x", 1) != 1) _exit(1);
        }
        usleep(200 * 1000);
        _exit(0);
    }
    return p;
}

void part1() {
    {   // signal then data
        int fds[2];
        if (pipe(fds)) return;
        const std::uint64_t t0 = m7ar::now_ns();
        const pid_t c = spawn_child(fds[1], 20, 100);
        int eintr = 0;
        const int r = m7ar::wait_readable(fds[0], t0 + 1000000000ull, &eintr);
        std::printf("{\"case\":\"wait_signal_then_data\",\"ret\":%d,\"eintr\":%d,\"ms\":%.3f}\n", r, eintr, ms_since(t0));
        waitpid(c, nullptr, 0);
        close(fds[0]); close(fds[1]);
    }
    {   // signal, no data, 50 ms deadline
        int fds[2];
        if (pipe(fds)) return;
        const std::uint64_t t0 = m7ar::now_ns();
        const pid_t c = spawn_child(fds[1], 20, 0);
        int eintr = 0;
        const int r = m7ar::wait_readable(fds[0], t0 + 50000000ull, &eintr);
        std::printf("{\"case\":\"wait_signal_deadline\",\"ret\":%d,\"eintr\":%d,\"ms\":%.3f}\n", r, eintr, ms_since(t0));
        waitpid(c, nullptr, 0);
        close(fds[0]); close(fds[1]);
    }
    {   // positive control: bare poll as the old helper
        int fds[2];
        if (pipe(fds)) return;
        const std::uint64_t t0 = m7ar::now_ns();
        const pid_t c = spawn_child(fds[1], 20, 100);
        pollfd p = {fds[0], POLLIN, 0};
        const int r = poll(&p, 1, 1000);
        const int e = errno;
        std::printf("{\"case\":\"bare_poll_control\",\"ret\":%d,\"eintr\":%d,\"ms\":%.3f}\n", r, (r < 0 && e == EINTR) ? 1 : 0,
                    ms_since(t0));
        waitpid(c, nullptr, 0);
        close(fds[0]); close(fds[1]);
    }
}

// ------------------------------------------------------------------ part 2: simulated core
struct SimCore {
    std::mutex m;
    std::condition_variable cv;
    bool busy = false, done = false, pending = false, irq_enabled = false, quit = false;
    std::uint32_t gie = 0, ier = 0, isr = 0;
    std::uint64_t finish_at = 0, byte_due = 0;
    std::uint64_t next_dur_ns = 5000000ull;   // duration of the next run that starts
    bool next_stuck = false;
    std::uint64_t irq_delay_ns = 0;
    long req = 0, run_id = 0, finished_id = 0, writes_busy = 0;
    int pfd[2] = {-1, -1};
    std::thread worker;

    SimCore() {
        if (pipe(pfd)) std::perror("pipe");
        worker = std::thread([this] { loop(); });
    }
    ~SimCore() {
        { std::lock_guard<std::mutex> g(m); quit = true; }
        cv.notify_all();
        worker.join();
        close(pfd[0]); close(pfd[1]);
    }
    void start_locked() {
        busy = true;
        run_id = req;
        finish_at = next_stuck ? ~0ull : m7ar::now_ns() + next_dur_ns;
        cv.notify_all();
    }
    void line_locked() {
        const bool line = gie && (ier & isr & 1);
        if (line && irq_enabled) {
            irq_enabled = false;   // the UIO handler masks the line
            if (irq_delay_ns == 0) { if (write(pfd[1], "x", 1) != 1) std::perror("write"); }
            else { byte_due = m7ar::now_ns() + irq_delay_ns; cv.notify_all(); }
        }
    }
    void loop() {
        std::unique_lock<std::mutex> lk(m);
        while (!quit) {
            const std::uint64_t now = m7ar::now_ns();
            if (byte_due && now >= byte_due) { byte_due = 0; if (write(pfd[1], "x", 1) != 1) std::perror("write"); }
            if (busy && now >= finish_at) {
                busy = false;
                done = true;
                finished_id = run_id;
                if (ier & 1) isr |= 1;
                line_locked();
                if (pending) { pending = false; start_locked(); }
                continue;
            }
            std::uint64_t wake = ~0ull;
            if (busy) wake = finish_at;
            if (byte_due) wake = std::min(wake, byte_due);
            if (wake == ~0ull) cv.wait(lk);
            else cv.wait_for(lk, std::chrono::nanoseconds(wake > now ? wake - now : 0));
        }
    }
    std::uint32_t rd(std::uint32_t off) {
        std::lock_guard<std::mutex> g(m);
        if (off == m7ar::kApCtrl) {
            const std::uint32_t v = (busy || pending ? 1u : 0u) | (done ? 2u : 0u) | (busy ? 0u : 4u);
            done = false;
            return v;
        }
        if (off == m7ar::kIsr) return isr;
        return 0;
    }
    void wr(std::uint32_t off, std::uint32_t v) {
        std::lock_guard<std::mutex> g(m);
        if (off == m7ar::kApCtrl) {
            if (v & 1) { ++req; if (!busy) start_locked(); else pending = true; }
        } else if (off == m7ar::kGie) { gie = v & 1; line_locked(); }
        else if (off == m7ar::kIer) { ier = v; line_locked(); }
        else if (off == m7ar::kIsr) { isr ^= v; line_locked(); }
        else if (busy) ++writes_busy;
    }
    void unstick() {
        std::lock_guard<std::mutex> g(m);
        if (busy && finish_at == ~0ull) finish_at = m7ar::now_ns();
        next_stuck = false;
        cv.notify_all();
    }
};

struct SimHw {
    SimCore* c;
    std::uint32_t rd(std::uint32_t off) { return c->rd(off); }
    void wr(std::uint32_t off, std::uint32_t v) { c->wr(off, v); }
    bool irq() const { return true; }
    int irq_fd() const { return c->pfd[0]; }
    bool irq_arm() { std::lock_guard<std::mutex> g(c->m); c->irq_enabled = true; c->line_locked(); return true; }
    bool irq_read() { char b[64]; ssize_t n; do n = read(c->pfd[0], b, sizeof b); while (n < 0 && errno == EINTR); return n > 0; }
    void pause_us(unsigned us) { usleep(us); }
};

// 9c2c6c0 UioDevice::run (irq branch), unchanged in substance: bare poll, no resync.
bool run_old(SimHw& hw, double timeout_s) {
    if (!hw.irq_arm()) return false;
    hw.wr(m7ar::kGie, 1);
    hw.wr(m7ar::kIer, 1);
    hw.wr(m7ar::kApCtrl, 1);
    pollfd p = {hw.irq_fd(), POLLIN, 0};
    const int pr = poll(&p, 1, static_cast<int>(timeout_s * 1000));
    bool ok = pr == 1 && hw.irq_read();
    const std::uint32_t ctrl = hw.rd(m7ar::kApCtrl);
    ok = ok && (ctrl & 0x2);
    hw.wr(m7ar::kIsr, hw.rd(m7ar::kIsr));
    return ok;
}

// signal sender thread (SIGUSR1 blocked in it): pthread_kill(main) at t_ms after `go`
pthread_t g_main;

struct Case {
    const char* name;
    int renders = 20;
    int event_render = 3;          // the render the disturbance hits (0-based)
    int sig_at_us = -1;            // SIGUSR1 this long after that render's run starts
    std::uint64_t dur_ns = 5000000ull, slow_ns = 0;   // slow_ns: that render's run lasts this long
    bool stuck = false;            // that render's run never ends until unstick before render+2
    std::uint64_t irq_delay_ns = 0;
};

void run_case(const Case& k, bool use_new) {
    SimCore core;
    SimHw hw{&core};
    core.irq_delay_ns = k.irq_delay_ns;
    const double timeout_s = 0.05;
    int ok_n = 0, fail_n = 0, desync = 0, fast_ok = 0, eintr = 0, stale = 0, late = 0, resync_n = 0, skipped = 0;
    int resync_bits_or = 0;
    long writes_busy_before = 0;
    for (int i = 0; i < k.renders; ++i) {
        {
            std::lock_guard<std::mutex> g(core.m);
            core.next_dur_ns = (i == k.event_render && k.slow_ns) ? k.slow_ns : k.dur_ns;
            core.next_stuck = (i == k.event_render && k.stuck);
        }
        if (k.stuck && i == k.event_render + 2) core.unstick();
        bool core_ok = true;
        if (use_new) {
            m7ar::ResyncInfo rs;
            core_ok = m7ar::resync_core(hw, m7ar::now_ns() + static_cast<std::uint64_t>(timeout_s * 1e9), &rs);
            if (rs.bits) { ++resync_n; resync_bits_or |= rs.bits; }
            if (!core_ok) ++skipped;
        }
        writes_busy_before = core.writes_busy;
        if (core_ok) for (int r = 0; r < 24; ++r) hw.wr(0x48 + 8 * r, static_cast<std::uint32_t>(i));
        std::thread sig;
        if (i == k.event_render && k.sig_at_us >= 0) {
            sig = std::thread([&k] {
                sigset_t s; sigemptyset(&s); sigaddset(&s, SIGUSR1); pthread_sigmask(SIG_BLOCK, &s, nullptr);
                usleep(static_cast<useconds_t>(k.sig_at_us));
                pthread_kill(g_main, SIGUSR1);
            });
        }
        bool ok = false;
        long my_req = 0;
        const std::uint64_t t0 = m7ar::now_ns();
        if (core_ok) {
            if (use_new) {
                m7ar::RunInfo ri;
                ok = m7ar::run_core(hw, t0 + static_cast<std::uint64_t>(timeout_s * 1e9), &ri);
                eintr += ri.eintr; stale += ri.stale_irq; late += ri.late_done;
            } else {
                ok = run_old(hw, timeout_s);
            }
        }
        const double t_ms = ms_since(t0);
        long fin = 0;
        { std::lock_guard<std::mutex> g(core.m); my_req = core.req; fin = core.finished_id; }
        if (sig.joinable()) sig.join();
        if (ok) { ++ok_n; if (fin != my_req) ++desync; if (t_ms < 1.0) ++fast_ok; }
        else ++fail_n;
        usleep(10000);   // the render period around the core run
    }
    std::printf("{\"case\":\"%s\",\"algo\":\"%s\",\"renders\":%d,\"ok\":%d,\"fail\":%d,\"desync\":%d,\"fast_ok\":%d,"
                "\"eintr\":%d,\"stale_irq\":%d,\"late_done\":%d,\"resync_renders\":%d,\"resync_bits\":%d,\"skipped\":%d,"
                "\"writes_while_busy\":%ld}\n",
                k.name, use_new ? "new" : "old", k.renders, ok_n, fail_n, desync, fast_ok, eintr, stale, late, resync_n,
                resync_bits_or, skipped, core.writes_busy);
    (void)writes_busy_before;
}

}  // namespace

int main(int argc, char** argv) {
    struct sigaction sa;
    std::memset(&sa, 0, sizeof sa);
    sa.sa_handler = on_usr1;   // no SA_RESTART (poll is never restarted anyway)
    sigaction(SIGUSR1, &sa, nullptr);
    g_main = pthread_self();
    const std::string only = argc > 1 ? argv[1] : "";
    if (only.empty() || only == "wait") part1();
    if (only.empty() || only == "sim") {
        Case eintr_case; eintr_case.name = "eintr"; eintr_case.sig_at_us = 2000;
        Case timeout_case; timeout_case.name = "timeout"; timeout_case.slow_ns = 80000000ull;
        Case stuck_case; stuck_case.name = "stuck"; stuck_case.stuck = true;
        Case late_case; late_case.name = "late_irq"; late_case.slow_ns = 57000000ull; late_case.irq_delay_ns = 5000000ull;
        Case calm; calm.name = "calm";
        for (const Case* k : {&calm, &eintr_case, &timeout_case, &stuck_case, &late_case}) {
            run_case(*k, false);
            run_case(*k, true);
        }
    }
    std::fflush(stdout);
    return 0;
}
