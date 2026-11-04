/* perf_ctrl (NN-CAM1, 2026-11-04): known-answer program to calibrate perf + the utime/stime split on the KV260
 * before the depth-path perf arms (PERF runbook step 1). Three busy threads, ~1 core each, for --seconds:
 *   user_spin : integer hash loop in this binary (symbol user_spin)         -> perf: perf_ctrl / user_spin
 *   user_copy : memcpy between two 4 MB buffers                             -> perf: libc memcpy (__memcpy_*)
 *   kern_copy : read() from /dev/zero into a 4 MB buffer (kernel copies)    -> perf: [kernel] (clear_user / __arch_clear_user ...)
 * plus one thread that only sleeps (must not appear). Expected: ~1/3 each of the samples; /proc stime ~1 core.
 * Build: aarch64-linux-gnu-gcc -O2 -g -fno-omit-frame-pointer -pthread -o perf_ctrl perf_ctrl.c
 */
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

static double secs = 30;
static volatile uint64_t sink;
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }

__attribute__((noinline)) static uint64_t user_spin_step(uint64_t x) {
  for (int i = 0; i < 100000; i++) x = (x ^ (x >> 7)) * 0x9E3779B97F4A7C15ull + i;
  return x;
}
static void *user_spin(void *a) { (void)a; double e = now() + secs; uint64_t x = 1; while (now() < e) x = user_spin_step(x); sink = x; return 0; }
static void *user_copy(void *a) {
  (void)a; size_t n = 4 << 20; char *s = malloc(n), *d = malloc(n); memset(s, 1, n);
  double e = now() + secs; while (now() < e) { memcpy(d, s, n); sink += (uint8_t)d[n / 2]; }
  return 0;
}
static void *kern_copy(void *a) {
  (void)a; size_t n = 4 << 20; char *d = malloc(n); int fd = open("/dev/zero", O_RDONLY);
  double e = now() + secs; while (now() < e) { if (read(fd, d, n) < 0) break; sink += (uint8_t)d[n / 2]; }
  close(fd); return 0;
}
static void *sleeper(void *a) { (void)a; double e = now() + secs; while (now() < e) usleep(100000); return 0; }

int main(int argc, char **argv) {
  if (argc > 2 && !strcmp(argv[1], "--seconds")) secs = atof(argv[2]);
  pthread_t t[4];
  pthread_create(&t[0], 0, user_spin, 0);
  pthread_create(&t[1], 0, user_copy, 0);
  pthread_create(&t[2], 0, kern_copy, 0);
  pthread_create(&t[3], 0, sleeper, 0);
  for (int i = 0; i < 4; i++) pthread_join(t[i], 0);
  printf("perf_ctrl done %.0f s\n", secs);
  return 0;
}
