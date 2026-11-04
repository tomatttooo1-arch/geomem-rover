#!/usr/bin/env bash
# E7 on the KV260 (2026-11-02): build, check, measure and switch the C++ data plane of kv_server (--kv-fast).
# WRITTEN, NOT RUN (prelim phase: no board). Run ON the KV260 as root (sudo), with the board lock held and the
# car's KV service stopped (two servers cannot share the PL): on the Nano/host side first
#   ssh nano 'ssh kv260 sudo /usr/local/sbin/geomem-kv-start stop'   (or kv_board.sh stop <run>)
# and the bitstream of the profile loaded (sudo xmutil unloadapp; sudo xmutil loadapp m7_pl_v3fc2).
#
#   board_e7.sh check     <repo> <prep_rec30> <out>   cpu flags; native build of kv_fast.cpp + kvfast_check;
#                         byte-exact check against numpy/zlib on the A53 itself (rec30 310 frames + edge cases,
#                         both CRC paths, odd offsets); raw C++ timings (bench)                  ~1 min, no PL
#   board_e7.sh segments  <repo> <prep_rec30> <out>   py_arm.py PL arm (devmem, unpaced, 1 warm-up + 3 x 310 frames)
#                         with --convert python, then native: the segments_a2.md caliber (only convert changes)
#                                                                                            ~2 x 15 s + builds
#   board_e7.sh server-ab <repo> <prep_rec30> <out> [loopback|nano]   kv_server off vs native with the car's
#                         flags minus AR (--core pl --device devmem --wait irq --roi-core pl --rolling pl), port
#                         47810, driven by nano_replay.py (rec30, 10 Hz, 3 passes, ROI boxes, rolling remote_pl);
#                         loopback = client on this KV260 (default), nano = prints the Nano command and waits
#   board_e7.sh enable|disable [<snapshot>]   flip FPGA_KV_FAST_DEFAULT in the INSTALLED competition snapshot
#                         sudo /usr/local/sbin/geomem-kv-start stop; sudo /usr/local/sbin/geomem-kv-start v3fc2
#                         disable = the one-step rollback to the Python path (the server is otherwise unchanged;
#                         a failed native build at start also falls back to Python by itself, with a warning)
# Afterwards: python3 <repo>/tools/kv_link/kvfast/summarize_ab.py --server off=<out>/ab/off/replay
#   native=<out>/ab/native/replay --segments off=<out>/seg/C_pl_free_python/frames.csv
#   native=<out>/seg/C_pl_free_native/frames.csv   (server-ab/segments print it themselves too)
set -u
cmd=${1:-}; shift || true
die() { echo "E7: $*" >&2; exit 1; }
need_root() { [ "$(id -u)" = 0 ] || die "run with sudo (devmem, dma_heap, UIO, SO_RCVBUFFORCE)"; }

case "$cmd" in
check)
  repo=$(readlink -f "${1:?repo}"); prep=$(readlink -f "${2:?prep}"); out=${3:?out}
  [ -e "$out" ] && die "$out exists (fresh directory please)"
  mkdir -p "$out"
  { date -Is; uname -a; grep -m1 -i 'features' /proc/cpuinfo; nproc; g++ --version | head -1
    python3 -c 'import numpy,sys;print("numpy",numpy.__version__,"python",sys.version.split()[0])'
    cat "$repo/COMMIT" 2>/dev/null; } > "$out/env.txt" 2>&1
  grep -qw crc32 "$out/env.txt" || echo "WARNING: /proc/cpuinfo lists no crc32: the table CRC will run (slower)" | tee -a "$out/env.txt"
  F="-std=c++11 -O2 -Wall -Wextra -Werror -ffp-contract=off"
  g++ $F "$repo/tools/kv_link/kvfast/kv_fast.cpp" "$repo/tools/kv_link/kvfast/kvfast_check.cpp" -o "$out/kvfast_check" \
    > "$out/build.log" 2>&1 || die "build failed, see $out/build.log"
  "$out/kvfast_check" info | tee "$out/info.txt"
  PYTHONPATH="$repo" python3 "$repo/tools/kv_link/kvfast/cross_check.py" --runner "" --checker "$out/kvfast_check" \
    --prep "$prep" --work "$out/xc" | tee "$out/cross_check.txt"
  n=$(python3 -c "import json;m=json.load(open('$prep/meta.json'));print(m['width']*m['height'])")
  bits=$(python3 -c "import json,struct;m=json.load(open('$prep/meta.json'));print('%08x'%struct.unpack('<I',struct.pack('<f',m['depth_unit_m']))[0])")
  taskset -c 1 "$out/kvfast_check" bench "$prep/depth_u16.bin" "$n" "$bits" 3 | tee "$out/bench.json"
  # the numpy/zlib reference of the same three steps, same core, same frames (what the switch replaces)
  PYTHONPATH="$repo" taskset -c 1 python3 - "$prep" <<'PY' | tee "$out/bench_python.json"
import json, sys, time, zlib, numpy as np
sys.path.insert(0, ".")
from host.kv_link import bits_f32, codes_to_m, crc32
prep = sys.argv[1]
m = json.load(open(prep + "/meta.json")); n = m["width"] * m["height"]
d = np.fromfile(prep + "/depth_u16.bin", dtype="<u2").reshape(-1, n)
unit = np.float32(m["depth_unit_m"]); a, b = [], []
for rep in range(3):
    for f in range(d.shape[0]):
        c = d[f]
        t0 = time.perf_counter(); crc32(c); t1 = time.perf_counter()
        x = codes_to_m(c, unit); crc32(x); t2 = time.perf_counter()
        a.append((t1 - t0) * 1e3); b.append((t2 - t1) * 1e3)
a.sort(); b.sort()
print(json.dumps({"frames": len(a), "pixels": n, "codes_crc_ms_median": a[len(a) // 2],
                  "convert_crc_ms_median": b[len(b) // 2], "convert_crc_ms_p95": b[len(b) * 95 // 100]}))
PY
  ;;
segments)
  need_root
  repo=$(readlink -f "${1:?repo}"); prep=$(readlink -f "${2:?prep}"); out=${3:?out}
  [ -e "$out" ] && die "$out exists (fresh directory please)"
  mkdir -p "$out"
  export PYTHONPATH="$repo" FPGA_M7_DMA_PROVIDER=${FPGA_M7_DMA_PROVIDER:-dmaheap} FPGA_M7_DMA_HEAP=${FPGA_M7_DMA_HEAP:-reserved}
  for conv in python native; do   # same order every time; 3 s rest between arms (kv260_pl_arm.sh rule)
    taskset -c 1 python3 "$repo/tools/geo_same_input/py_arm.py" --prep "$prep" --arm c --device devmem \
      --out "$out/C_pl_free_$conv" --passes 3 --warmup 1 --pace-hz 0 --convert $conv \
      > "$out/C_pl_free_$conv.stdout" 2> "$out/C_pl_free_$conv.stderr" || die "py_arm $conv failed ($out/C_pl_free_$conv.stderr)"
    sleep 3
  done
  cmp "$out/C_pl_free_python/flags.dump" "$out/C_pl_free_native/flags.dump" \
    && echo "flags.dump identical (python vs native convert)" | tee "$out/flags_equal.txt" \
    || die "flags.dump differ between the two convert paths -- stop, do not enable"
  python3 "$repo/tools/kv_link/kvfast/summarize_ab.py" --segments off="$out/C_pl_free_python/frames.csv" \
    native="$out/C_pl_free_native/frames.csv" --json "$out/segments_ab.json" | tee "$out/segments_ab.md"
  ;;
server-ab)
  need_root
  repo=$(readlink -f "${1:?repo}"); prep=$(readlink -f "${2:?prep}"); out=${3:?out}; client=${4:-loopback}
  [ -e "$out" ] && die "$out exists (fresh directory please)"
  pgrep -f '[k]v_server.py' >/dev/null && die "a kv_server is running (the car service?): stop it first"
  mkdir -p "$out/ab"
  board=$repo/tools/kv_link/kv_board.sh
  PYTHONPATH="$repo" python3 "$repo/tools/kv_link/nano_replay.py" --prep "$prep" --make-roi-memory "$out/roi_snap.json" \
    > "$out/roi_snap.log" 2>&1 || die "roi snapshot failed"
  for mode in off native; do
    env FPGA_M7_DMA_PROVIDER=dmaheap FPGA_M7_DMA_HEAP=reserved bash "$board" start "$repo" "$out/ab/$mode/server" \
      --core pl --device devmem --wait irq --roi-core pl --rolling pl --port 47810 --max-idle-s 300 \
      --kv-fast $mode || die "kv_server $mode did not start ($out/ab/$mode/server/server.err)"
    if [ "$client" = nano ]; then
      echo "On the Nano now (KV address as seen from the Nano, e.g. 198.18.31.x):"
      echo "  PYTHONPATH=<repo on Nano> python3 <repo>/tools/kv_link/nano_replay.py --prep <prep_rec30 on Nano> \\"
      echo "    --out <dir>/$mode/replay --hz 10 --passes 3 --backend remote_pl --server <KV_IP>:47810 \\"
      echo "    --rolling-backend remote_pl --roi-memory <copy of $out/roi_snap.json> --compare"
      echo "then copy <dir>/$mode/replay back to $out/ab/$mode/replay and press Enter"; read -r _
    else
      PYTHONPATH="$repo" taskset -c 0 python3 "$repo/tools/kv_link/nano_replay.py" --prep "$prep" \
        --out "$out/ab/$mode/replay" --hz 10 --passes 3 --backend remote_pl --server 127.0.0.1:47810 \
        --rolling-backend remote_pl --roi-memory "$out/roi_snap.json" --compare \
        > "$out/ab/$mode/replay.log" 2>&1 || echo "WARNING nano_replay $mode rc=$? ($out/ab/$mode/replay.log)"
    fi
    bash "$board" stop "$out/ab/$mode/server"
    python3 -c "import json,sys;s=json.load(open(sys.argv[1]));print('$mode kv_fast:',json.dumps(s.get('kv_fast')))" \
      "$out/ab/$mode/server/summary.json" | tee -a "$out/kv_fast_seen.txt"
    sleep 3
  done
  python3 "$repo/tools/kv_link/kvfast/summarize_ab.py" --server off="$out/ab/off/replay" native="$out/ab/native/replay" \
    --json "$out/server_ab.json" | tee "$out/server_ab.md"
  echo "voxel / ROI equality of each arm with the CPU reference: $out/ab/<mode>/replay/summary.json (compare block)"
  ;;
enable|disable)
  need_root
  snap=${1:-/opt/geomem-dev3-kv/source}
  env_file=$snap/tools/car_run/fpga_profiles.env
  [ -f "$env_file" ] || die "$env_file not found (snapshot without E7?)"
  grep -q '^FPGA_KV_FAST_DEFAULT=' "$env_file" || die "$env_file has no FPGA_KV_FAST_DEFAULT line (deploy the E7 branch first)"
  [ -f "$snap/host/kv_fast_native.py" ] || die "$snap has no host/kv_fast_native.py"
  want=native; [ "$cmd" = disable ] && want=off
  cp -p "$env_file" "$env_file.bak-$(date +%Y%m%d-%H%M%S)"
  sed -i "s/^FPGA_KV_FAST_DEFAULT=.*/FPGA_KV_FAST_DEFAULT=$want/" "$env_file"
  grep '^FPGA_KV_FAST_DEFAULT=' "$env_file"
  echo "now restart the KV service: sudo /usr/local/sbin/geomem-kv-start stop; sudo /usr/local/sbin/geomem-kv-start v3fc2"
  echo "and confirm in <run>/server.err / summary.json: kv_fast impl native (enable) or no kv_fast key (disable)"
  ;;
*)
  sed -n '2,30p' "$0"; exit 2;;
esac
