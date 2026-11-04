// DPUPIPE0: FrameRing + geometry unit test (host). Exit 0 = pass. Built and run by tests/test_dpu_pipe.py.
#include <atomic>
#include <cassert>
#include <cstdio>
#include <set>
#include <thread>

#include "dpu_rx.h"

using namespace dpupipe;
#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL %s:%d %s\n", __FILE__, __LINE__, #c); return 1; } } while (0)

int main() {
  // geometry = host/dpu_prep.py LetterboxGeometry for the car camera
  Geometry g = letterbox(640, 480, 640);
  CHECK(g.w == 640 && g.h == 480 && g.top == 80 && g.left == 0 && g.direct());
  g = letterbox(640, 480, 512);
  CHECK(g.w == 512 && g.h == 384 && g.top == 64 && g.direct());
  g = letterbox(640, 480, 416);
  CHECK(g.w == 416 && g.h == 312 && g.top == 52 && g.direct());
  g = letterbox(480, 640, 640);  // portrait: left/right pad -> staging path
  CHECK(g.left == 80 && g.top == 0 && !g.direct());
  g = letterbox(1280, 720, 640);
  CHECK(g.w == 640 && g.h == 360 && g.top == 140);
  CHECK(quant_pad(6) == 29 && quant_pad(7) == 57);

  // ring: alignment, pad fill, states
  FrameRing r(3, 640 * 640 * 3, 29);
  for (size_t i = 0; i < r.slots(); ++i) {
    CHECK(reinterpret_cast<uintptr_t>(r.data(i)) % 4096 == 0);
    CHECK(static_cast<int8_t>(r.data(i)[0]) == 29 && static_cast<int8_t>(r.data(i)[640 * 640 * 3 - 1]) == 29);
  }
  int a = r.acquire_free(10), b = r.acquire_free(10), c = r.acquire_free(10);
  CHECK(a >= 0 && b >= 0 && c >= 0 && a != b && b != c && a != c);
  CHECK(r.acquire_free(10) == -1);                 // all FILLING
  FrameMeta m;
  m.source_key = 1;
  r.publish(a, m);
  m.source_key = 2;
  r.publish(b, m);                                 // a is stale -> FREE
  CHECK(r.stale_dropped() == 1 && r.state(a) == FrameRing::FREE);
  CHECK(!r.wait_ready_below(1, 10));               // one READY waiting: producer must hold
  FrameMeta got;
  int s = r.acquire_ready(10, &got);
  CHECK(s == b && got.source_key == 2 && r.state(b) == FrameRing::INFER);
  CHECK(r.acquire_ready(10) == -1);
  CHECK(r.wait_ready_below(1, 10));
  r.abort(c);
  r.release(b);
  bool threw = false;
  try { r.release(b); } catch (const std::logic_error&) { threw = true; }
  CHECK(threw);

  // producer/consumer threads: consumer always gets increasing keys, never a slot being filled
  FrameRing q(3, 4096, 0);
  std::atomic<bool> stop{false};
  std::atomic<uint64_t> last{0}, bad{0}, n{0};
  std::thread cons([&] {
    while (!stop) {
      FrameMeta mm;
      int k = q.acquire_ready(5, &mm);
      if (k < 0) continue;
      if (mm.source_key <= last) ++bad;
      last = mm.source_key;
      if (q.data(k)[0] != static_cast<uint8_t>(mm.source_key & 0xFF)) ++bad;  // slot content = its frame
      std::this_thread::sleep_for(std::chrono::microseconds(300));
      if (q.data(k)[0] != static_cast<uint8_t>(mm.source_key & 0xFF)) ++bad;  // nobody wrote it meanwhile
      ++n;
      q.release(k);
    }
  });
  for (uint64_t key = 1; key <= 3000; ++key) {
    if (!q.wait_ready_below(1, 50)) { --key; continue; }
    int k = q.acquire_free(50);
    if (k < 0) { --key; continue; }
    std::memset(q.data(k), static_cast<int>(key & 0xFF), 4096);
    FrameMeta mm;
    mm.source_key = key;
    q.publish(k, mm);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  stop = true;
  cons.join();
  CHECK(bad == 0 && n > 100);
  printf("{\"ring_test\":\"pass\",\"consumed\":%llu,\"stale\":%llu}\n", (unsigned long long)n.load(),
         (unsigned long long)q.stale_dropped());
  return 0;
}
