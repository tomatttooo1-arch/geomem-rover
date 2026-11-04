/* GMAP1 A53 timing program (the document's "before" column for the global-map operations; format as A53BENCH0).
 *
 * The same algorithm as the PL operations: tools/gmap/gmap_model.c (included below, unchanged), on the same input
 * the PL check uses (two real trips' frame sequences, tools/gmap/make_a53_data.py), every output hashed and compared
 * with the PS reference's golden (host/kv_mapper.py rule q13). Only the gmap_run call is timed (CLOCK_MONOTONIC);
 * copying the frame and the parameter block into the map area (the PS's part, also before the PL call) is not.
 *
 *   a53_gmap_bench --data DIR [--reps N] [--scan-reps K] [--cpu C] [--fifo P] [--flip-byte]
 *   prints one JSON line per item and a markdown table; exit 0 when every item is bit-exact.
 * --flip-byte: positive control (both items must report bitexact false): the first known byte of each trip's LAST
 *   frame toggled free <-> occupied (a hit and a miss never clamp to the same value, so the map must change; a flip
 *   in an early frame can be absorbed by the clamp), and the scan's image table entry for "occupied" 0 -> 1.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "gmap_model.c"

/* ------------------------------------------------------------------ sha256 (FIPS 180-4, compact) */
typedef struct { uint32_t h[8]; uint64_t len; uint8_t buf[64]; uint32_t n; } sha256_t;
static const uint32_t K256[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))
static void sha_block(sha256_t *s, const uint8_t *p) {
    uint32_t w[64], a, b, c, d, e, f, g, h;
    for (int i = 0; i < 16; ++i) w[i] = (uint32_t)p[4 * i] << 24 | (uint32_t)p[4 * i + 1] << 16 | (uint32_t)p[4 * i + 2] << 8 | p[4 * i + 3];
    for (int i = 16; i < 64; ++i) {
        uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    a = s->h[0]; b = s->h[1]; c = s->h[2]; d = s->h[3]; e = s->h[4]; f = s->h[5]; g = s->h[6]; h = s->h[7];
    for (int i = 0; i < 64; ++i) {
        uint32_t t1 = h + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + K256[i] + w[i];
        uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    s->h[0] += a; s->h[1] += b; s->h[2] += c; s->h[3] += d; s->h[4] += e; s->h[5] += f; s->h[6] += g; s->h[7] += h;
}
static void sha_init(sha256_t *s) {
    static const uint32_t h0[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    memcpy(s->h, h0, sizeof h0); s->len = 0; s->n = 0;
}
static void sha_update(sha256_t *s, const uint8_t *p, size_t n) {
    s->len += n;
    while (n--) { s->buf[s->n++] = *p++; if (s->n == 64) { sha_block(s, s->buf); s->n = 0; } }
}
static void sha_hex(sha256_t *s, char out[65]) {
    uint64_t bits = s->len * 8; uint8_t pad = 0x80, z = 0, lb[8];
    sha_update(s, &pad, 1);
    while (s->n != 56) sha_update(s, &z, 1);
    for (int i = 0; i < 8; ++i) lb[i] = (uint8_t)(bits >> (56 - 8 * i));
    sha_update(s, lb, 8);
    for (int i = 0; i < 8; ++i) sprintf(out + 8 * i, "%08x", s->h[i]);
}

/* ------------------------------------------------------------------ helpers */
static uint64_t now_ns(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return (uint64_t)t.tv_sec * 1000000000ull + (uint64_t)t.tv_nsec; }
static int cmp_u64(const void *a, const void *b) { uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b; return x < y ? -1 : x > y; }
static uint64_t pct(uint64_t *v, size_t n, double q) {      /* nearest rank */
    qsort(v, n, sizeof *v, cmp_u64);
    size_t k = (size_t)(q * (double)n + 0.999999); if (k < 1) k = 1; if (k > n) k = n;
    return v[k - 1];
}
static void put_params(uint8_t *mem, int ix, int iy, const uint8_t ct[3], const uint8_t it[3], int flip) {
    uint8_t *p = mem + GMAP_OFF_PARAM;
    memset(p, 0, 16);
    p[0] = GMAP_MAGIC & 0xFF; p[1] = GMAP_MAGIC >> 8;
    p[2] = (uint8_t)ix; p[3] = (uint8_t)((uint16_t)ix >> 8); p[4] = (uint8_t)iy; p[5] = (uint8_t)((uint16_t)iy >> 8);
    memcpy(p + 6, ct, 3); memcpy(p + 9, it, 3); p[12] = (uint8_t)flip;
}

typedef struct { char name[64]; uint32_t n; uint8_t *rec; char g_merge[65], g_scan[65]; } trip_t;

static int load_trip(const char *dir, const char *name, const char *file, uint32_t n, trip_t *t) {
    char path[1024]; snprintf(path, sizeof path, "%s/%s", dir, file);
    FILE *f = fopen(path, "rb"); if (!f) { fprintf(stderr, "open %s: %s\n", path, strerror(errno)); return -1; }
    char mg[8]; uint32_t cnt;
    if (fread(mg, 1, 8, f) != 8 || memcmp(mg, "GMAPFRM1", 8) || fread(&cnt, 4, 1, f) != 1 || cnt != n) { fclose(f); fprintf(stderr, "bad %s\n", path); return -1; }
    t->rec = malloc((size_t)n * (4 + 6400));
    if (fread(t->rec, 4 + 6400, n, f) != n) { fclose(f); fprintf(stderr, "short %s\n", path); return -1; }
    fclose(f); snprintf(t->name, sizeof t->name, "%s", name); t->n = n;
    return 0;
}

int main(int argc, char **argv) {
    const char *dir = "data"; int reps = 5, scan_reps = 20, cpu = 3, fifo = 0, flip = 0;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--data") && i + 1 < argc) dir = argv[++i];
        else if (!strcmp(argv[i], "--reps") && i + 1 < argc) reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--scan-reps") && i + 1 < argc) scan_reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--cpu") && i + 1 < argc) cpu = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fifo") && i + 1 < argc) fifo = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--flip-byte")) flip = 1;
        else { fprintf(stderr, "usage: %s --data DIR [--reps N] [--scan-reps K] [--cpu C] [--fifo P] [--flip-byte]\n", argv[0]); return 2; }
    }
    cpu_set_t cs; CPU_ZERO(&cs); CPU_SET(cpu, &cs);
    int aff = sched_setaffinity(0, sizeof cs, &cs) == 0 ? cpu : -1;
    int fifo_ok = 0;
    if (fifo > 0) { struct sched_param sp = {.sched_priority = fifo}; fifo_ok = sched_setscheduler(0, SCHED_FIFO, &sp) == 0 ? fifo : -1; }

    char gpath[1024]; snprintf(gpath, sizeof gpath, "%s/golden.txt", dir);
    FILE *g = fopen(gpath, "r"); if (!g) { fprintf(stderr, "open %s\n", gpath); return 2; }
    trip_t trips[8]; int nt = 0; char line[1024];
    while (fgets(line, sizeof line, g) && nt < 8) {
        if (line[0] == '#') continue;
        char name[64], file[256]; uint32_t n;
        if (sscanf(line, "%63s %u %255s %64s %64s", name, &n, file, trips[nt].g_merge, trips[nt].g_scan) != 5) continue;
        if (load_trip(dir, name, file, n, &trips[nt])) return 2;
        if (flip) {                                   /* first known byte of the last frame toggled free <-> occupied */
            uint8_t *fr = trips[nt].rec + (size_t)(n - 1) * (4 + 6400) + 4;
            for (int k = 0; k < 6400; ++k) if (fr[k] != 0xFF) { fr[k] = fr[k] == 0x64 ? 0x00 : 0x64; break; }
        }
        ++nt;
    }
    fclose(g);

    uint8_t *mem = aligned_alloc(4096, (GMAP_AREA_BYTES + 4095) / 4096 * 4096);
    const uint8_t ct[3] = {0, 0, 254}, it[3] = {254, 205, (uint8_t)(flip ? 1 : 0)};
    int all_ok = 1;
    printf("| item | trip | unit | n | median us | p99 us | per-pass ms | bitexact |\n|---|---|---|---|---|---|---|---|\n");
    char rows[16][2048]; int nr = 0;
    for (int t = 0; t < nt; ++t) {
        trip_t *tr = &trips[t];
        uint64_t *ts = malloc(sizeof(uint64_t) * (size_t)tr->n * (size_t)reps);
        uint64_t pass_ns[64]; int bit_merge = 1;
        for (int r = 0; r < reps + 1; ++r) {          /* pass 0 = warm-up (not counted) */
            memset(mem, 0xA5, GMAP_AREA_BYTES);
            put_params(mem, 0, 0, ct, it, 1);
            gmap_run(mem, GMAP_OP_CLEAR);
            uint64_t p0 = 0;
            for (uint32_t i = 0; i < tr->n; ++i) {
                const uint8_t *rec = tr->rec + (size_t)i * (4 + 6400);
                int16_t ix = (int16_t)(rec[0] | rec[1] << 8), iy = (int16_t)(rec[2] | rec[3] << 8);
                put_params(mem, ix, iy, ct, it, 1);
                memcpy(mem + GMAP_OFF_FRAME, rec + 4, 6400);
                uint64_t a = now_ns();
                gmap_run(mem, GMAP_OP_MERGE);
                uint64_t d = now_ns() - a;
                p0 += d;
                if (r > 0) ts[(size_t)(r - 1) * tr->n + i] = d;
            }
            if (r > 0 && r - 1 < 64) pass_ns[r - 1] = p0;
            sha256_t s; char hx[65]; sha_init(&s); sha_update(&s, mem + GMAP_OFF_GLOBAL, 360000); sha_hex(&s, hx);
            if (strcmp(hx, tr->g_merge)) bit_merge = 0;
        }
        size_t nn = (size_t)tr->n * (size_t)reps;
        uint64_t med = pct(ts, nn, 0.5), p99 = pct(ts, nn, 0.99);
        uint64_t pmed = pct(pass_ns, (size_t)(reps < 64 ? reps : 64), 0.5);
        printf("| merge | %s | frame | %zu | %.1f | %.1f | %.3f | %s |\n", tr->name, nn, med / 1e3, p99 / 1e3, pmed / 1e6, bit_merge ? "true" : "false");
        snprintf(rows[nr++], sizeof rows[0], "{\"item\":\"merge\",\"trip\":\"%s\",\"unit\":\"frame\",\"n\":%zu,\"median_ns\":%llu,\"p99_ns\":%llu,\"pass_median_ns\":%llu,\"bitexact\":%s,\"cpu\":%d,\"fifo\":%d}",
                 tr->name, nn, (unsigned long long)med, (unsigned long long)p99, (unsigned long long)pmed, bit_merge ? "true" : "false", aff, fifo_ok);
        all_ok &= bit_merge;
        /* SCAN on the final map (idempotent): PGM table, y flip */
        uint64_t sc[256]; int ks = scan_reps < 256 ? scan_reps : 256, bit_scan = 1;
        put_params(mem, 0, 0, ct, it, 1);
        gmap_run(mem, GMAP_OP_SCAN);                  /* warm-up */
        for (int k = 0; k < ks; ++k) {
            uint64_t a = now_ns();
            gmap_run(mem, GMAP_OP_SCAN);
            sc[k] = now_ns() - a;
            sha256_t s; char hx[65]; sha_init(&s);
            sha_update(&s, mem + GMAP_OFF_IMAGE, 160000); sha_update(&s, mem + GMAP_OFF_COARSE, 40000); sha_hex(&s, hx);
            if (strcmp(hx, tr->g_scan)) bit_scan = 0;
        }
        uint64_t smed = pct(sc, (size_t)ks, 0.5), sp99 = pct(sc, (size_t)ks, 0.99);
        printf("| scan | %s | call | %d | %.1f | %.1f | - | %s |\n", tr->name, ks, smed / 1e3, sp99 / 1e3, bit_scan ? "true" : "false");
        snprintf(rows[nr++], sizeof rows[0], "{\"item\":\"scan\",\"trip\":\"%s\",\"unit\":\"call\",\"n\":%d,\"median_ns\":%llu,\"p99_ns\":%llu,\"bitexact\":%s,\"cpu\":%d,\"fifo\":%d}",
                 tr->name, ks, (unsigned long long)smed, (unsigned long long)sp99, bit_scan ? "true" : "false", aff, fifo_ok);
        all_ok &= bit_scan;
        free(ts);
    }
    for (int i = 0; i < nr; ++i) printf("JSON %s\n", rows[i]);
    printf("ALL_BITEXACT %s\n", all_ok ? "true" : "false");
    free(mem);
    return all_ok ? 0 : 1;
}
