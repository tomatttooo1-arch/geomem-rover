#!/bin/bash
# Line D2 (2026-10-14): offline render probe -- one fresh Gazebo world (the run's image, launch file and
# camera), the robot at its start pose, N RGB frames from /fpga/rgb saved as PNG. No navigation, no runtime.
#   bash tools/sim_loop/render_probe.sh WORLD.sdf OUT_DIR [N_FRAMES]
# WORLD.sdf absolute; it is copied as probe.sdf into a copy of the checkout's worlds/ (OUT/worlds_mnt, removed at
# the end) mounted at the installed worlds path (props/ meshes resolve there). BOTTLE_ASSET (optional): directory mounted read-only at
# /fpga_bottle_asset (the scanned bottle). GZ_CPUS (default 36-41). Container wd2-probe-<OUT basename>, removed
# by name at the end. Never pkill -f.
set -u
WORLD=${1:?WORLD.sdf}; OUT=${2:?OUT_DIR}; N=${3:-3}
case "$WORLD$OUT" in /*/*) ;; esac
REPO=$(cd "$(dirname "$0")/../.." && pwd)
TAG=$(basename "$OUT"); NAME=wd2-probe-$TAG
: "${GZ_CPUS:=36-41}"
mkdir -p "$OUT" || exit 2
IMAGE=fpga-dev1-sim:humble
MNT=(-v "$REPO:/src:ro" -v "$OUT:/o")
for name in launch config maps; do
  MNT+=(-v "$REPO/robot_sim/fpga_sim/$name:/opt/fpga_ws/install/share/fpga_sim/$name:ro")
done
# worlds: a copy of the checkout's worlds/ plus probe.sdf (a file cannot be bind-mounted into a read-only bind)
rm -rf "$OUT/worlds_mnt" && cp -r "$REPO/robot_sim/fpga_sim/worlds" "$OUT/worlds_mnt" && cp "$WORLD" "$OUT/worlds_mnt/probe.sdf" || exit 2
MNT+=(-v "$OUT/worlds_mnt:/opt/fpga_ws/install/share/fpga_sim/worlds:ro")
[ -n "${BOTTLE_ASSET:-}" ] && MNT+=(-v "$BOTTLE_ASSET:/fpga_bottle_asset:ro")
trap 'docker logs "$NAME" > "$OUT/gz.log" 2>&1; docker rm -f "$NAME" > /dev/null 2>&1; rm -rf "$OUT/worlds_mnt"' EXIT
docker run -d --init --name "$NAME" --network none --shm-size 1g --cpuset-cpus "$GZ_CPUS" \
  --user "$(id -u):$(id -g)" -e ROS_DOMAIN_ID=94 -e ROS_LOCALHOST_ONLY=1 \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/src/robot_sim/fpga_sim/config/fastdds_server.xml -e IGN_PARTITION="$TAG" \
  -e HOME=/tmp -e LIBGL_ALWAYS_SOFTWARE=1 -e FPGA_SIM_CLOUD=adapter -e PYTHONDONTWRITEBYTECODE=1 \
  -e FPGA_SIM_WORLD=/opt/fpga_ws/install/share/fpga_sim/worlds/probe.sdf "${MNT[@]}" --entrypoint bash "$IMAGE" -c \
  'source /opt/ros/humble/setup.bash; source /opt/fpga_ws/install/setup.bash;
   exec xvfb-run -a -s "-screen 0 1280x720x24" ros2 launch fpga_sim simulation.launch.py' > /dev/null || exit 5
docker exec "$NAME" bash -c 'source /opt/ros/humble/setup.bash; cd /src; timeout 240 python3 - "$0" <<"PY"
import sys, time, zlib, struct
import numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
n = int(sys.argv[1]); rclpy.init(); node = rclpy.create_node("fpga_render_probe"); got = []
node.create_subscription(Image, "/fpga/rgb", lambda m: got.append(m), qos_profile_sensor_data)
t = time.time()
while len(got) < 10 + n and time.time() - t < 220:     # skip the first frames (scene still loading)
    rclpy.spin_once(node, timeout_sec=0.2)
def png(path, a):
    h, w, _ = a.shape
    raw = b"".join(b"\x00" + a[y].tobytes() for y in range(h))
    def chunk(t, d): return struct.pack(">I", len(d)) + t + d + struct.pack(">I", zlib.crc32(t + d) & 0xffffffff)
    open(path, "wb").write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
                           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
for i, m in enumerate(got[-n:]):
    a = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
    png("/o/frame_%02d.png" % i, a)
print("saved", min(n, len(got)), "of", len(got), flush=True)
PY' "$N" > "$OUT/grab.log" 2>&1
RC=$?
echo "grab_rc=$RC" > "$OUT/rc.txt"
exit $RC
