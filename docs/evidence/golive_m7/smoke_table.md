# Core-level smoke test on the m7 full-board bitstream m7_pl_m7s17 (BUILD_ID 0x063ba99b), KV260 bench, 2026-11-04

Excerpt of the board smoke summary (TABLE.md, MEMK/PLN summary lines and the per-record search-core log of the same session).
Each extension core runs a key vector set; outputs are compared bit for bit with the host C model / golden output.

| core | result | compared | board timing |
|---|---|---|---|
| memory core MEMK4 (bram layout, 0xA00E_0000) | PASS | calls=500, compared_words=512000, mismatching_calls=0 | call median 2.502 ms, p95 3.008 ms (PS write 1024 + start + poll + read 1024, Python) |
| planner GMAP3 v9 (0xA00C_0000) | PASS | A_bytes=186904, B_bytes=187732, C_ops=73, C_bytes_per_op=1173088, all identical | - |
| search core v2, microcode engine (hw/srch_v2, 0xA00D_0000) | PASS | records=32, compared_bytes=12582912, mismatching_records=0 (golden: c8 fixed-point C model recording; all 32 records "diff_bytes 0 ok") | observe (op 10) mean 0.81 ms over 15 calls; select (op 12) mean 1.26 ms over 3 calls |

Note: the smoke kit labels the search row with the c8 name because the golden trace and driver are shared between the two
versions; the bitstream loaded was m7_pl_m7s17 (BUILD_ID read back 0x063ba99b).
