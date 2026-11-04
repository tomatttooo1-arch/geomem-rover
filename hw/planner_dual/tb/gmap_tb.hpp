// GMAP2 key comparison 2 driver: testbench --gmap VEC.bin
// VEC.bin = "GMVEC001", u32 init (0 = random bytes, 1 = random bytes then a C-model-only CLEAR), u32 nops, then per op:
// u32 op, u8 param[16], u32 has_frame, [u8 frame[6400]]. Before each op the PS part (frame + params) is written into
// both the DUT image and the C-model image (tools/gmap/gmap_model.c); the DUT gets map = image bytes, field = image
// words; afterwards the DUT image = field words. After EVERY op the whole 526,432-byte area and the status word must
// equal the C model's. tb_out/outputs.bin = the final DUT image; tb_out/gmap_ops.txt = one line per op.
// GMAP3: "GMVEC002" adds, after the frame part of every op, u32 has_win, [u8 aux[6400], u8 wparam[256]] (WINDOW
// inputs written into the aux slot and the WINDOW parameter block); init 2 = random bytes then a C-model-only CLEAR
// with byte 13 bit 0 (T map "never" too). The area is 1,173,088 bytes since GMAP3.
#pragma once
#include <cstdint>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <cstdio>
#include <fstream>
#include <vector>
extern "C" {
#include "gmap_model.c"
}
static unsigned gtb_u32(std::istream& f) {
  unsigned v = 0;
  for (int j = 0; j < 4; ++j) { int b = f.get(); if (b < 0) throw std::runtime_error("truncated"); v |= unsigned(b) << (8 * j); }
  return v;
}
int gmap_main(const char* vec) {
  try {
    std::ifstream f(vec, std::ios::binary);
    char magic[8]; f.read(magic, 8);
    const std::string mg(magic, 8);
    if (!f || (mg != "GMVEC001" && mg != "GMVEC002")) return 3;
    const bool v2 = mg == "GMVEC002";
    const unsigned init = gtb_u32(f), nops = gtb_u32(f);
    static ap_uint<8> map[kGmapAreaBytes]; static ap_uint<32> field[kGmapAreaWords], flags[kPlnFlagsDepth];
    std::vector<uint8_t> dut(kGmapAreaBytes), ref(kGmapAreaBytes);
    uint32_t s = 20261104u;
    for (int i = 0; i < kGmapAreaBytes; ++i) { s = s * 1664525u + 1013904223u; dut[i] = ref[i] = uint8_t(s >> 24); }
    if (init == 1 || init == 2) {   // start from the cleared state without a DUT CLEAR (CLEAR is covered by vector 1)
      uint8_t p[16] = {0x4D, 0x47, 0, 0, 0, 0, 0, 0, 254, 254, 205, 0, 1, uint8_t(init == 2), 0, 0};
      for (int i = 0; i < 16; ++i) ref[kGmapOffParam + i] = p[i];
      gmap_run(ref.data(), GMAP_OP_CLEAR);
      dut = ref;
    }
    mkdir("tb_out", 0700);
    std::ofstream log("tb_out/gmap_ops.txt");
    unsigned bad = 0;
    for (unsigned k = 0; k < nops; ++k) {
      const unsigned op = gtb_u32(f);
      uint8_t prm[16]; f.read(reinterpret_cast<char*>(prm), 16);
      const unsigned has_frame = gtb_u32(f);
      std::vector<uint8_t> fr(6400);
      if (has_frame) f.read(reinterpret_cast<char*>(fr.data()), 6400);
      const unsigned has_win = v2 ? gtb_u32(f) : 0u;
      std::vector<uint8_t> aux(6400), wp(256);
      if (has_win) {
        f.read(reinterpret_cast<char*>(aux.data()), 6400);
        f.read(reinterpret_cast<char*>(wp.data()), 256);
      }
      if (!f) throw std::runtime_error("truncated op");
      if (has_win) {
        for (int i = 0; i < 6400; ++i) dut[kGmapOffAux + i] = ref[kGmapOffAux + i] = aux[i];
        for (int i = 0; i < 256; ++i) dut[kGmapOffWparam + i] = ref[kGmapOffWparam + i] = wp[i];
      }
      for (int i = 0; i < 16; ++i) dut[kGmapOffParam + i] = ref[kGmapOffParam + i] = prm[i];
      if (has_frame) for (int i = 0; i < 6400; ++i) dut[kGmapOffFrame + i] = ref[kGmapOffFrame + i] = fr[i];
      for (int i = 0; i < kGmapAreaBytes; ++i) map[i] = dut[i];
      for (int i = 0; i < kGmapAreaWords; ++i)
        field[i] = unsigned(dut[4 * i]) | unsigned(dut[4 * i + 1]) << 8 | unsigned(dut[4 * i + 2]) << 16 | unsigned(dut[4 * i + 3]) << 24;
      ap_uint<32> st, rd, va;
      planner_field_dual(map, field, flags, 0, 0, 0, 0, 0, st, rd, va, 0, 0, op);
      for (int i = 0; i < kGmapAreaWords; ++i) {
        const unsigned w = field[i].to_uint();
        for (int j = 0; j < 4; ++j) dut[4 * i + j] = uint8_t(w >> (8 * j));
      }
      const uint32_t mst = gmap_run(ref.data(), op);
      unsigned nd = 0; int first = -1;
      for (int i = 0; i < kGmapAreaBytes; ++i) if (dut[i] != ref[i]) { if (first < 0) first = i; ++nd; }
      const uint32_t* rw = reinterpret_cast<const uint32_t*>(&ref[kGmapOffResult]);
      log << "op " << k << " code " << op << " status 0x" << std::hex << st.to_uint() << " model 0x" << mst
          << std::dec << " w1 " << rw[1] << " w2 " << rw[2] << " w3 " << rw[3] << " diff_bytes " << nd
          << " first " << first << "\n";
      if ((nd || st.to_uint() != mst) && !bad) {   // first bad op: both images for diagnosis
        std::ofstream d1("tb_out/first_bad_dut.bin", std::ios::binary), d2("tb_out/first_bad_ref.bin", std::ios::binary);
        d1.write(reinterpret_cast<const char*>(dut.data()), dut.size());
        d2.write(reinterpret_cast<const char*>(ref.data()), ref.size());
      }
      if (nd || st.to_uint() != mst) ++bad;
    }
    std::ofstream o("tb_out/outputs.bin", std::ios::binary);
    o.write(reinterpret_cast<const char*>(dut.data()), dut.size());
    std::printf("gmap vector %s: %u ops, %u bad\n", vec, nops, bad);
    if (!bad) std::printf("PASS gmap vector (area + status == C model after every op)\n");
    return bad ? 1 : 0;
  } catch (const std::exception& e) { std::fprintf(stderr, "%s\n", e.what()); return 2; }
}
