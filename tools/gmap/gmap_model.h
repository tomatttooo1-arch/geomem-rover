 * (gmap_op 0x98: 1 MERGE, 2 SCAN, 3 CLEAR, 4 WINDOW (GMAP3); 0 = the planner's own function, not modelled here).
 * Design: runs/nationals-prelim-20261102/GMAP0/DESIGN.md sections 3.3 (Q2.13 rule), 4 (data path), 5 (layout).
 * PS reference: host/kv_mapper.py rule "q13" (+ coarse / image / bbox).
 *
 * The model works on the DDR "map area" image only (1,173,088 bytes since GMAP3, base A): it reads bytes (the core's 8-bit
 * map port) and writes whole little-endian 32-bit words (the core's 32-bit field port), in the region / order the
 * design gives, so the HLS of the second stage can be compared with it byte for byte (check (2)).
 */
#ifndef GMAP_MODEL_H
#define GMAP_MODEL_H

#include <stdint.h>

#define GMAP_N          400                 /* global grid 400 x 400, 0.05 m */
#define GMAP_W          80                  /* frame 80 x 80 */
#define GMAP_CN         (GMAP_N / 2)        /* coarse 200 x 200 */

/* byte offsets in the map area (DESIGN section 4.1) */
#define GMAP_OFF_GLOBAL 0u                  /* int16 [400][400], row = y, little-endian */
#define GMAP_OFF_COARSE 320000u             /* uint8 [200][200], row = y */
#define GMAP_OFF_FRAME  360000u             /* uint8 [80][80], row = ly: 0xFF unknown, 0x64 occupied, other free */
#define GMAP_OFF_PARAM  366400u             /* 16-byte parameter block */
#define GMAP_OFF_IMAGE  366416u             /* uint8 [400][400] scan image */
#define GMAP_OFF_RESULT 526416u             /* 4 x uint32 result words */
/* GMAP3: WINDOW (gmap_op 4, DESIGN_ADD_PLGRID.md section 2.1, positions Q16) */
#define GMAP_OFF_T      526432u             /* int32 [400][400] swept time map, ms, row = y; INT32_MIN = never */
#define GMAP_OFF_AUX    1166432u            /* uint8 [80][80] aux plane of the fusion core's GRID op */
#define GMAP_OFF_WPARAM 1172832u            /* 256-byte WINDOW parameter block (tools/gmap/window_ref.py) */
#define GMAP_AREA_BYTES 1173088u

#define GMAP_MAGIC      0x474Du             /* param bytes 0-1 */
#define GMAP_WMAGIC     0x5747u             /* WINDOW param bytes 0-1 */
#define GMAP_QS         65536               /* Q16: 1 cell = 65536 */
#define GMAP_NEVER      ((int32_t)0x80000000u)
#define GMAP_PTS_MAX    21

/* Q2.13 rule (DESIGN section 3.3); overridable for the positive control */
#ifndef GMAP_HIT
#define GMAP_HIT        6941
#endif
#define GMAP_MISS       (-3322)
#define GMAP_LMIN       (-16384)
#define GMAP_LMAX       28762
#define GMAP_UNSEEN     (-32768)

#define GMAP_OP_MERGE   1
#define GMAP_OP_SCAN    2
#define GMAP_OP_CLEAR   3
#define GMAP_OP_WINDOW  4                   /* body seed + swept clearing (PlGridLayer), frame slot output */

/* result word 0 */
#define GMAP_ST_BAD_MAGIC   0x1u
#define GMAP_ST_BBOX_EMPTY  0x2u
#define GMAP_ST_BAD_OP      0x4u
#define GMAP_VERSION        0x02u           /* bits 15:8 (2 = GMAP3: WINDOW, T map, CLEAR byte 13 bit 0) */

#ifdef __cplusplus
extern "C" {
#endif
/* Runs one operation on the map area; returns result word 0. */
uint32_t gmap_run(uint8_t *mem, uint32_t op);
#ifdef __cplusplus
}
#endif

#endif
