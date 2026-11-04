// MEMK0 semantic-memory core: block layout shared by the core, the C model ABI and the PS driver.
// Design: runs/nationals-prelim-20261102/MEMK0/DESIGN.md (sections 2-4); 11-04 ruling: command and result
// blocks are s_axilite arrays (no m_axi), 1024 32-bit words each (one BRAM36 each). Little endian.
// Persistent regions: the PS writes the class table once (and a row whenever a new label appears); the
// observation region is written by every ASSOC and read by the EXEC calls of the same frame; the core keeps
// the per-observation derived values (sd, weight) in the result block, which EXEC calls do not overwrite.
#pragma once

#define MK_MAGIC      0x4D4Bu
#define MK_CMD_WORDS  1024
#define MK_RES_WORDS  1024
#define MK_SLOTS      64
#define MK_OBS        32
#define MK_KSAMP      32          // sample ring capacity (product fuse_keep 20)
#define MK_RING       16          // hit-record ring per slot
#define MK_CLASSES    64
#define MK_PAIRS      256

// commands (header word 0 bits 19:16)
#define MK_CMD_INIT   1
#define MK_CMD_ASSOC  2
#define MK_CMD_EXEC   3

// header (16 words)
//  h0  magic[15:0] | cmd[19:16] | flags[27:20]  (flag0 pose_valid, flag1 detections_known, flag2 skip_miss)
//  h1  n_obs/boxes[7:0] | mu[15:8] | n_ops[23:16]
//  h2  window (int32)           h3  frame seq (u24)
//  h4  odometer path (2^-22 m)  h5  odometer turn (2^-21 rad)
//  h6  ground z (2^-18 m)       h8..10 camera T (2^-18 m)   h11..13 R[:,2] (2^-24)
#define MK_HDR        16
#define MK_F_POSE     1
#define MK_F_DET      2
#define MK_F_SKIPMISS 4

// ASSOC observations, words 16..399, 12 words each:
//  o0 cls[7:0] | truncated[8] | range_known[9] | extent_known[10]
//  o1 score (2^-24)  o2..4 position (2^-18 m)  o5 range (2^-20 m)  o6,7 extent w,h (2^-16 m)
//  o8..11 bbox x1,y1,x2,y2 (2^-8 px)
#define MK_OBS_BASE   16
#define MK_OBS_W      12

// class table, words 400..783, 6 words per class (PS-owned, persistent):
//  c0 slack (2^-20 m)  c1 max_w*margin  c2 max_h*margin (2^-16 m)  c3 class P_D factor (2^-24)
//  c4 create min score (2^-24)  c5 has_size[0] | clip_may_create[1]
#define MK_CLS_BASE   400
#define MK_CLS_W      6

// EXEC: per-slot geometry codes, words 784..791 (4 bits per slot: code[1:0] protected[2] state_invalid[3]),
//  code 0 skip (no word, geometry-only entry), 1 no vote, 2 free, 3 surface_or_uncertain
// ops from word 792, 4 words each: p0 opcode[7:0] | a[15:8] | b[23:16] | i[31:24]; p1 flags; p2, p3 imm
#define MK_GEO_BASE   784
#define MK_OP_BASE    792
#define MK_OP_W       4
#define MK_OPS        58
enum {
  OP_END = 0, OP_TOUCH = 1, OP_CAND = 2, OP_NEW = 3, OP_COPY_SAMPLES = 4, OP_CLEAR_SAMPLES = 5,
  OP_MERGE = 6, OP_COPY_PLACE = 7, OP_RESET_L = 8, OP_FREE = 9, OP_RESTORE = 10, OP_QUERY = 11,
  OP_MERGE_SCAN = 12, OP_EXIST = 13, OP_DUMP = 14, OP_STOP = 15
};
// OP_TOUCH p1 bit0 append; OP_NEW p1 = type (1 track, 2 candidate), b = class; OP_MERGE p1 bit0 drop newer;
// OP_RESTORE (two op slots) p1 = type | lvalid<<8, b = class, p2, p3 = x, y; next slot p0 = z, p1 = L (2^-24);
// OP_QUERY p1 = kind[1:0] (0 ref obs i, 1 ref slot a with obs i, 2 ref slot a alone) | mask[3:2] (bit2 tracks,
//   bit3 candidates) | tag[15:8];  OP_MERGE_SCAN p1 = stop[0] | tag[15:8] (stop: records found -> end the call)

// result block
//  r0 status: error bits[7:0] | merge_stop[8]   r1 n_pairs   r2 n_dump_words   r3 config digest low 32
//  r8..103  ASSOC per observation 3 words: q0 reason[1:0] | dup[2] | eligible[3] | may_create[4] | too_far[5]
//                 | beyond[6] | score_ok[7] | dedupe rank[12:8] | assigned slot[20:14] (127 none)
//                 q1 observation sd So (2^-24 m)   q2 weight 1/So^2 (2^-12 m^-2)   (kept for the EXEC calls)
//  r104..615 pair records, 2 words: p0 q[6:0] | slot[13:7] | tag[21:14] | flags[31:22]; p1 d (2^-20 m)
//  r616..935 dumps, 5 words per slot j < mu: px, py, pz (2^-18), L (2^-24),
//           flags: lvalid[0] below[1] free_at_vote[2] n_samples[8:3] fused[9] type[11:10]
#define MK_RES_OBS    8
#define MK_RES_OBS_W  3
#define MK_RES_PAIRS  104
#define MK_RES_DUMP   616
#define MK_DUMP_W     5
// pair flags
#define PF_GATE   1
#define PF_FAR    2
#define PF_RADIUS 4
#define PF_EXCL   8
#define PF_MERGE  16
#define PF_NEAR   32
#define PF_ASSIGN 64
#define PF_CAND   128
// errors
#define ERR_MAGIC 1
#define ERR_SIZE  2
#define ERR_SLOT  4
#define ERR_PAIRS 8
#define ERR_RANGE 16
#define ERR_OP    32
