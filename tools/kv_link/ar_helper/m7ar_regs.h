// AR overlay core register offsets (fpga_m7_ar_overlay_axi, s_axilite 'control').
// Copied from hw/m7/pl/sw/m7pl_hls_regs.h at pl-merge 7784f75 (generated from the
// exported IP of m7_pl_merge 0x07784f75); tests/test_kv_ar.py re-parses that header from git
// and compares every entry.
#ifndef M7AR_REGS_H
#define M7AR_REGS_H
#include <cstdint>
#define M7AR_AP_CTRL            0x00
#define M7AR_GIE                0x04
#define M7AR_IER                0x08
#define M7AR_ISR                0x0c
#define M7AR_AP_RETURN          0x10
#define M7AR_DEPTH              0x18
#define M7AR_GRID               0x24
#define M7AR_BOXES              0x30
#define M7AR_FB                 0x3c
#define M7AR_WIDTH              0x48
#define M7AR_HEIGHT             0x50
#define M7AR_DEPTH_WIDTH        0x58
#define M7AR_DEPTH_HEIGHT       0x60
#define M7AR_DEPTH_DX           0x68
#define M7AR_DEPTH_DY           0x70
#define M7AR_FRAME_VALID        0x78
#define M7AR_POSE_VALID         0x80
#define M7AR_AGE_MS             0x88
#define M7AR_AGE_VALID          0x90
#define M7AR_MAX_AGE_MS         0x98
#define M7AR_REPLAY             0xa0
#define M7AR_OBJECT_EVIDENCE    0xa8
#define M7AR_FRAME_ID           0xb0
#define M7AR_PERMIT_PRESENT     0xb8
#define M7AR_GRID_VALID         0xc0
#define M7AR_SLICE_Y            0xc8
#define M7AR_SLICE_LO_MM        0xd0
#define M7AR_SLICE_HI_MM        0xd8
#define M7AR_CAM_CELL_X         0xe0
#define M7AR_CAM_CELL_Z         0xe8
#define M7AR_MINI_X             0xf0
#define M7AR_MINI_Y             0xf8
// 24 scalar registers in spec order (= tools/ar_display/golden.py REG_NAMES); permit_i
// (index 15) is the 1-bit ap_none wire from permit/permit_o, not a register.
static const std::uint32_t kM7arRegOff[24] = {
    M7AR_WIDTH, M7AR_HEIGHT, M7AR_DEPTH_WIDTH, M7AR_DEPTH_HEIGHT,
    M7AR_DEPTH_DX, M7AR_DEPTH_DY, M7AR_FRAME_VALID, M7AR_POSE_VALID,
    M7AR_AGE_MS, M7AR_AGE_VALID, M7AR_MAX_AGE_MS, M7AR_REPLAY,
    M7AR_OBJECT_EVIDENCE, M7AR_FRAME_ID, M7AR_PERMIT_PRESENT, 0xFFFFFFFFu,
    M7AR_GRID_VALID, M7AR_SLICE_Y, M7AR_SLICE_LO_MM, M7AR_SLICE_HI_MM,
    M7AR_CAM_CELL_X, M7AR_CAM_CELL_Z, M7AR_MINI_X, M7AR_MINI_Y,
};
#endif
