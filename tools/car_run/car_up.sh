#!/bin/bash
# Line E (2026-10-14): start the whole car with one command; it returns once every step is ready and leaves
# the car running (no time limit) until tools/car_run/car_down.sh.
#
#   bash tools/car_run/car_up.sh [sim|replay|live] [--camera sim|replay|live] [--pose reference|rtabmap|static]
#        [--nav costmap|nav2] [--detector stub|ssdlite|yolo] [--chassis none|pty|uart]
#        [--yolo-backend torch|ort|trt] [--gpu N] [--run-dir DIR] [--record] [--dry-run]
#   presets: sim    = sim + reference + nav2 + yolo (host torch GPU) + pty (mcu_sim chain)
#            replay = replay (rec30 prep) + reference + costmap + stub + none
#            live   = live + rtabmap + nav2 + yolo (board trt) + uart   (board; on this host --dry-run only)
#   line-4 bench (10-15): live --pose static --chassis none --nav costmap -- camera fixed on a bench, no chassis:
#            map->odom->base_link identity (the true pose of a car that cannot move; NOT mapping) + a free stand-in /map
#   real-car drive (10-15): CAR_TELEOP=1 car_up.sh live --pose odom --nav costmap  (chassis uart from the preset):
#            remote control (host/ros_teleop_safe.py, keys: tools/car_run/teleop_keys.py) + wheel odometry pose
#            (map->odom identity, odom->base_link + /odom from host/ros_wheel_odom.py); NOT a navigation run
#   real-car navigation (10-17): CAR_MAP_YAML=<rtabmap-export yaml> car_up.sh live --pose odom --nav nav2 (chassis uart):
#            wheel odometry pose as above, /map from Nav2's map_server (the export; bench_pose publishes no /map),
#            Nav2 on the rolling costmap; refused (2) without a map, without chassis uart, with CAR_TELEOP=1 or
#            CAR_NAV_DRYRUN=1. The car must start at the mapping start, same heading (map->odom identity)
#   the data-source marker follows the camera (sim -> sim, replay -> replay, live -> live); it is not a switch.
#   --record   test-only recorders (see car_inner.sh) so that tools/car_run/car_scenario.sh + verify_bottle_run.py
#              can check a run; never needed by the car.
#   --dry-run  print what would run (host and inner plan), start nothing, create nothing.
# Exit codes: 0 ready; 2 bad switches / combination; 3 not on this platform (board: also no Orbbec camera on
#   USB); 9 board: the KV260 at KV_SERVER is not serving (a KV serving another BUILD_ID, registered or not, only
#   warns: its id is used, an unregistered one with the Nano CPU fusion); 10 a car is already running;
#   4 inference service not ready; 5 a container did not start; 6 GPU lock; 7 not ready in CAR_READY_S
#   (stack_fail.txt says which step); 8 source snapshot. 3 and 9 are refused before anything is created; every
#   failure after the run directory exists is rolled back by car_down.sh (what was started is stopped, the lock released).
#
# Real-car drive chain (2026-10-15):
#   CAR_TELEOP (0)   1 = start host/ros_teleop_safe.py in the stack: the only /fpga/cmd_vel_safe publisher, speed-capped
#                    (CAR_TELEOP_V_MAX 0.15 m/s, CAR_TELEOP_W_MAX 0.6 rad/s, CAR_TELEOP_MAX_CMD_S 3 s; caps 0.3 / 1.0 / 10),
#                    dead man 0.3 s, zero on exit; NOT gated by the motion permit (the MCU heartbeat latch is the backstop).
#                    Needs --chassis uart|pty; refused with sim (velocity guard) and nav2 (a navigation run). Input socket
#                    CAR_STATE_DIR/<tag>/teleop.sock; keys: python3 tools/car_run/teleop_keys.py (finds the running car).
#   CAR_WHEEL_ODOM   (1 with --chassis uart, else 0) host/ros_wheel_odom.py --source chassis; live: odom->base_link on /tf
#                    and /odom; replay/sim (pty, forced with 1): shadow topics /fpga/wheel_odom(/tf) only (they own the edge)
#   CAR_HEADING      gyro (default on the board with --chassis uart, owner 10-27) | wheel (default elsewhere): heading source of host/ros_wheel_odom.py (gyro needs the MCU's 0x82 frame,
#                    docs/design/CHASSIS_UART_V02_IMU_HANDOFF_20261021.md section 8/10); CAR_WHEEL_ODOM_ARGS extra args
#                    passed to it verbatim (e.g. "--wheel-signs 1,1,-1,-1 --gyro-scale-pos 1.02")
#   CAR_WHEEL_DIAM_M CAR_COUNTS_PER_REV CAR_TRACK_M  wheel parameters. Board (live): empty = the real car's values
#                    0.081 / 1560 / 0.394 (10-16 calibration, mecanum equivalent track; owner 10-16, DECISIONS D-041);
#                    host (sim/replay): empty = the PLACEHOLDER 0.065 / 1320 / 0.160 (marks.txt and car.json say
#                    "placeholder"). Given = used as given on both; car.json wheel_odom.source says which.
#   CAR_NAV_DRYRUN (0) 1 = the board Nav2 DRY RUN (controller 10-17): ONLY live --pose static --nav nav2 --chassis none
#                    (car_validate refuses it with anything else; not set = the static+nav2 refusal as before). No
#                    chassis, the car cannot move: Nav2, the velocity guard and the runner's goals (fpgapc goto) end at
#                    /fpga/cmd_vel_safe. Static has no RTABMAP_DB: give CAR_MAP_YAML explicitly. car.json "nav_dryrun".
#   CAR_NAV_BACKEND (nav2) CTRLPS2 2026-11-03, runs/nationals-prelim-20261102/CTRLPS0/DESIGN.md 3: nav2 = Nav2 drives, every
#                    line as before (the default; it stays the default until the real-car A/B passes and owner approves).
#                    kv = the KV260 PS navigation core drives (kv_server --nav soft|pl on the KV): Nav2 keeps only map_server
#                    + planner_server (G1 global route), host/ros_kv_nav_proxy.py serves navigate_to_pose and publishes
#                    /cmd_vel into the unchanged velocity guard / teleop arbitration / chassis driver. shadow = Nav2 drives,
#                    the proxy records the KV's commands next to Nav2's ($RUN/kv_nav_cmds.jsonl), publishes nothing; on the
#                    bench: CAR_NAV_DRYRUN=1 CAR_NAV_BACKEND=shadow car_up.sh live --pose static --chassis none --nav nav2.
#                    Both need --nav nav2 (refused, exit 2, otherwise). Host: the fake KV gets --nav soft --nav-core
#                    CAR_KV_NAV_CORE (mock = host stand-in, default; kv_nav = CTRLPS1's core). car.json "nav_backend".
#   BENCH_MAP_M with --nav nav2 is always 0 (no stand-in /map: map_server owns /map; static and odom alike, 10-17)
#   CAR_STATE_DIR / CAR_RUNS_DIR default on the board (aarch64 or live) /data/fpga/car/state and /data/fpga/car/runs
#
# Host layout (from tools/sim_loop/run_sim_loop.sh, line D2; CPU sets, DDS settings and images unchanged):
#   host process  host.inference_service (ssdlite | yolo torch on GPU INF_GPU, under its lock | yolo ort on CPU)
#   wde-car-gz-<tag>     (sim) Gazebo + ros_gz bridge + velocity guard; network none, ipc shareable
#   wde-car-kv-<tag>     the fake KV260 (CAP_NET_ADMIN only, 8 MB receive buffer like the board's root
#                        kv_server); in the Gazebo container's namespaces (sim) or owning them (replay)
#   wde-car-stack-<tag>  car_inner.sh: everything else, in the same network and /dev/shm
# Run directory (pids.txt + host_pids.txt + containers.txt + every log); IPC socket directory
# CAR_STATE_DIR/<tag> (short path: Unix socket length limit); CAR_STATE_DIR/car.current -> the run directory
# while a car is up (one car per host). Never pkill -f: processes by pid, containers by name.
#
#      GZ_CPUS KV_CPUS KV_THREADS STACK_CPUS RUNTIME_CPUS INF_CPUS  YOLO_PT YOLO_ONNX YOLO_UL_PATH YOLO_THREADS
#      INF_WARMUP (2)  MCU_HEARTBEAT (kv)  FPGA_SIM_WORLD_FILE (room_carmount_d2.sdf; the D2 YOLO world)
#      CAR_FAILSTOP (0 = hub/bridge/preview/producer restarted, any other child exiting stops the stack; 1 = any child exiting
#        stops it (before 10-26 the default); log = an exit is only logged, e.g. the cut test that stops the producer)
#        (10-26 owner：重成功率，放宽自 1)
#      board (live): KV_SERVER (198.51.100.79:47800) KV_BUILD_ID (0x00da5041) YOLO_ENGINE (/data/fpga/yolo_engine_20261015/...fp16.engine)
#        NANO_PYLIB (/home/robot/fpga/p2/pylib) RTABMAP_DB CHASSIS_PORT BOARD_IMAGE
#      rtabmap (POSE rtabmap|map, 10-15): RTABMAP_ODOM (live visual, replay wheel) RTABMAP_DB_DIR (board
#        /data/fpga/rtabmap; host always $RUN/rtabmap_db) CAR_RTABMAP_STOP_S (240; car_down waits +90 via
#        $RUN/stack_stop_s.txt) CAR_MAP_YAML (board nav2; default the one *.yaml in <RTABMAP_DB dir>/run_<stem>/export)
#        FPGA_NAV_COSTMAP (board nav2: rolling | sim; unset or empty = rolling, the rolling-map local costmap + DWB;
#        sim = the RPP heading-precision controller with the /fpga/points obstacle layers, as before 10-15; host: unused)
#        GRID_READY_S (60; rolling: the wait for the bridge's /fpga/rolling_map/grid before Nav2 is activated)
#        CAR_NAV_TIMEOUT_S (90) / CAR_TASK_TIMEOUT_S (300), board only (10-22 find-nav dig: 30 s ended goals the car
#        was still turning/driving towards): the runner's per-goal budget (its wall-clock watchdog AND the task
#        controller's navigating clock, one value) and the whole task's; host runs keep the runner's 30 / 120
#        (10-26 owner：重成功率，放宽自 60 / 180)
#        CAR_MAX_NAV_FAILURES (5; board only, the runner's --max-nav-failures; 10-26 owner：重成功率，放宽自 3)
#        CAR_POSE_TRUST (off; board only, the runner's --pose-trust; 10-26 owner：重成功率，陀螺比例标定好再设 on)
#        CAR_SEARCH_MODE (prob; 10-26): prob = the belief search picks each search look (host/search_belief.py,
#        tools/car_run/search_config.json; needs the mapping run's trajectory, else the runner keeps the MVP list),
#        mvp = the 10-26 search MVP's waypoint list; anything else refused (exit 2)
#        CAR_LIVE_ALIGN (1; 10-27, CAR_MAPPING=1 only): 1 = while mapping, every RTAB-Map loop closure / graph
#        optimization also corrects map -> odom and the object memory within ~2 s (mapping_ctl --live-align reads
#        /mapData, host/live_align.py; drift duplicates merge without stopping the map; docs/design/LIVE_ALIGN_20261027.md);
#        0 = off, exactly the behavior before 10-27 (one alignment at the mapping stop); anything else refused (exit 2)
#        CAR_NAV_LIVE_GLOBAL (1; 10-29, CAR_MAPPING=1 + nav2 + CAR_LIVE_ALIGN=1 only): 1 = while mapping, Nav2's GLOBAL
#        costmap gets map_server's map with rtabmap's live occupied cells on top (host/ros_nav_global_map.py; unknown
#        stays plannable, places the car drove stay free, a failed plan falls back to the plain map for 20 s), so
#        global paths go round walls / furniture before 结束建图; after the stop the export takes over as before;
#        0 = off, the global costmap on /map (the blank start map) as before; anything else refused (exit 2)
#        CAR_NAV_GLOBAL_ROLLING (1; 10-29, with CAR_NAV_LIVE_GLOBAL=1): 1 = that global map also gets the rolling local
#        layer's current obstacles (the cells DWB refuses), in every state incl. after 结束建图, so a global path never
#        runs through or grazes an obstacle only the local layer has (the car stopped at its edge and turned on the
#        spot until the goal failed); a failed plan falls back to the plain map for 20 s as above; 0 = rtabmap's
#        cells only (as before); anything else refused (exit 2)
#        CAR_STREAM_TIMEOUT_S (5.0; 10-26), board only: the runner's processed-frame gap that ends a task as
#        failed:stream_lost (no retry). 2 <= value <= 20 s, else refused (exit 2): below 2 s the Nano's routine ~2 s
#        stalls at ~96 % CPU would end tasks (2.0 is the host simulation's own value), above 20 s (a third of the 60 s
#        per-goal budget) a dead camera would keep the car driving on stale data for most of a goal; car.json
#        "runner_budgets" (with CAR_NAV_TIMEOUT_S / CAR_TASK_TIMEOUT_S)
#        CAR_FRONT_SEED_M (10-26; empty = OFF, the default until the owner rules: it widens the 09-27 rolling-layer
#        rule): e.g. 0.65 = the rolling bridge also sends UNKNOWN cells within 0.65 m ahead (+-35 deg of the current
#        heading) as FREE, current pose only, OCCUPIED untouched. A positive number <= 1.0 m; car.json "front_seed"
#        board Nav2 switches (10-26; unset or empty = not passed, the launch's default; the container got only its -e
#        list, so before this an export on the Nano never reached Nav2): FPGA_NAV_ROTATION_SHIM (true | false; default
#        true), FPGA_NAV_PROGRESS_CHECKER (pose | simple; default pose), FPGA_NAV_CAR_PLANNER (strict | relaxed; default
#        strict). Car (FPGA_NAV_SPEED_LIMITS=car) only; another value refused (exit 2); car.json "nav_switches"
#        CAR_DWB_MIN_SPEED_THETA (<= 0.4 rad/s) / CAR_DWB_MIN_SPEED_XY (<= 0.1 m/s) / CAR_ROT_STOPPED_VELOCITY
#        (<= 0.4 rad/s) (10-26; empty = nav_input_profile.py's 0.10 / 0.015 / 0.15): the car's DWB minimum speeds and
#        StoppedGoalChecker rot_stopped_velocity for a measured chassis dead band; a positive number, and the effective
#        rot_stopped_velocity must be > the effective min_speed_theta, else refused (exit 2); car.json "dwb_min_speeds"
#        CAR_DWB_FORWARD_PRUNE_M (10-29; empty = nav_input_profile.py's 0.8 m): the car's DWB forward_prune_distance --
#        how far along the path DWB's goal critics aim (0.8: a point on the path ahead, so a path that first leads away
#        from a nearby goal is driven; 2.0 = Nav2's default = the behaviour before). 0.3..4.0 m, else refused (exit 2);
#        car.json "dwb_forward_prune"
#        CAR_GATE_MODE (correction | output | frame; default correction, owner 10-15 evening) and CAR_GATE_X_S (2; the
#        gate's --max-correction-age-s X, correction mode only): POSE rtabmap, ODOM visual only (the gate); passed to
#        car_inner.sh as RTABMAP_GATE_MODE / RTABMAP_GATE_X_S, checked by rtabmap_car.sh.
#        X tiers (host/localization_gate.py COUNT_MODES): measure on the board first the rtabmap localization output
#        rate and the p90 stamp age at which the gate receives them; >= 3 Hz and p90 <= 1.0 s -> 2; 1-3 Hz and
#        p90 <= 1.5 s -> 3; < 1 Hz -> no navigation on estimated poses (reduce the board load first); then fixed.
#        CAR_GATE_ODOM_JUMP (1 | 0; default 1), CAR_GATE_ODOM_V_MPS (0.5) and CAR_GATE_ODOM_E_M (0.10): the gate's
#        odometry jump check (10-17 第 10 棒; --odom-jump-check / --max-odom-speed-mps V / --odom-jump-margin-m E,
#        correction mode only): a healthy odometry step longer than V x dt + E revokes (valid) or drops the entry
#        candidates; V is the platform's top speed (per platform); CAR_GATE_ODOM_JUMP=0 is the 4edc10f gate. Passed to
#        car_inner.sh as RTABMAP_GATE_ODOM_JUMP / RTABMAP_GATE_ODOM_V_MPS / RTABMAP_GATE_ODOM_E_M, checked by
#        rtabmap_car.sh.
#        LIVE_START_TRIES (10) LIVE_START_WAIT_S (20) LIVE_START_GAP_S (10) LIVE_DEPTH_MSGS (30)  (the Orbbec driver's
#        start retry: a start is good when tools/car_run/depth_probe.py receives LIVE_DEPTH_MSGS depth images; 30, not 3:
#        cam-start-dig 10-15, a failed start can leak 1-4 depth images and stop, 3 let it through; line-4 bench 10-15
#        ran all 5 cycles with 30 and every start came up on try 1)
#        CAR_CAMERA_RATE_CHECK (1 | 0; board default 1, 10-17) the half-rate check of every driver start: the depth probe
#        keeps listening CAR_CAMERA_RATE_S (3; integer 1-5) more seconds and judges the header-stamp intervals -- depth
#        median < 32.8 ms or colour 25th percentile > 50 ms = the camera's half-rate state (colour 15.5 Hz, depth
#        32.3 ms; docs/design/COLOUR_RATE_NAVMODE_20261017.md section 7) = a failed start, restarted by the start retry
#        above; every try half-rate = refused (stack_fail, rollback 7). Result $RUN/camera_rate_check.json.
#        0 = the plan of before, byte for byte (nothing passed into the stack)
#        CAR_CAMERA_FPS (15 | 30 | 10 | 5; default 15 since 10-29, owner: the Nano CPU was saturated while mapping and
#        finding together) colour AND depth rate of the Orbbec driver (color_fps / depth_fps of the default
#        LIVE_LAUNCH_ARGS; a color_fps:= given in LIVE_LAUNCH_ARGS wins). The rate-dependent rules follow it: the rate
#        check above at 15 fps judges colour p25 > 100 ms only (depth rule off; tools/car_run/depth_probe.py), the
#        runner's colour/depth pairing (20 -> 38 ms = half a period + 5 ms) and producer-frame match (50 -> 73 ms),
#        rtabmap's approx sync (20 -> 38 ms), the car screen's colour pairing on the KV (17 -> 39 ms, via the colour
#        sender's HELLO), the
#        preview still 10 fps (due-time schedule). 30 = everything as before.
#        CAR_PRIORITY (1 | 0; board default 1 since 10-29, owner: the Nano CPU was saturated) CPU priorities in the
#        stack: the container gets --cap-add SYS_NICE and, once started, tools/car_run/car_priority.py runs in it as
#        root (docker exec -u 0; pid in host_pids.txt, log $RUN/priority.log, applied values in up_marks.txt after
#        READY). Every 3 s, every thread: task runner, wheel odometry (pose on /tf), chassis serial, camera driver
#        nice -5; hub +5; preview, car screen colour, 3D map display (live_mapping.py, map_assembler) +10 (CPU share
#        only: no rate or quality changed); everything else (navigation avoidance "bridge", Nav2, rtabmap, producer)
#        stays 0. 0 = the container line of before, byte for byte, and no priority process.
#        CAR_CAM_HEIGHT (0.39 m) CAR_CAM_PITCH_DEG (16.0 since 10-28; 7.0 from 10-17)  the camera mount, MEASURED (owner 10-17 14:16 re-tilted the
#        camera by hand, "about 10 deg"; the board's start-up ground fit c10_1 14:31 gave 0.3915 m / 7.05 deg / roll
#        0.59 deg; before: 0.375 m / 30 deg, fit 0.3799 m / 27.4 deg) -> the base_link -> camera static TF and the
#        producer's --height-cell. The fitted 0.3915 m was above the old 0.39 m hard limit of the adopted height;
#        owner 10-17 approved 0.395 (ground_calib.HARD_HEIGHT_M), so CAR_GROUND_CALIB=auto adopts the fit on this car
#        (offline on c10_1: 0.3921 m / 7.09 deg / roll 0.59, cell 8). docs/design/CAM_TILT_10_20261017.md section 9
#        CAR_HEIGHT_CELL (empty = derived from the height: 0.39 m -> 8; given = checked)   (tools/car_run/car_mount.py)
#        CAR_ROBOT_HEIGHT (0.31 m, owner 10-16) the car body height of the rolling bridge's obstacle band
#        (robot_height_m: band ground + 0.05 .. CAR_ROBOT_HEIGHT + 0.05 = 0.36 m; host sim/replay keep 0.25). The stack
#        refuses the start (exit 7) when the camera height in use (the adopted one after the ground calibration) is
#        below CAR_ROBOT_HEIGHT + 0.05 + 0.005 m: the band would reach the lens (car_mount.py camera_above_band)
#        CAR_CAM_HEIGHT_UNC_M (0.01: hand-measured, owner 10-15; recorded in car.json only)
#        CAR_GROUND_CALIB (auto | off | check; board default auto, 10-15) start-up ground self-calibration in the stack
#        (tools/car_run/ground_calib.py, after camera_check, before the mount TF; the car must stand on a flat floor
#        with 0.3-2 m of free floor ahead): auto = fit within +-5 deg / +-3 cm / roll +-5 deg -> the mount TF and the
#        producer's --height-cell use the FITTED values, else the start is refused (exit 7); check = keep the
#        configured mount, refused when the fit differs by > 1 deg / 1 cm; off = the configured mount as before.
#        Result $RUN/ground_calib.json, car.json camera_mount "ground_calib" (filled after READY). Mapping (--pose map)
#        stores the adopted mount beside the new database (<db stem>.ground_calib.json); localization prints the
#        difference to it (marks.txt; > 1 deg / 1 cm a warning only -- the current mount is used)
#        (10-15, passed into the stack, printed by --dry-run) LIVE_LAUNCH_ARGS (the driver's launch arguments)
#        LIVE_FRAME_DROP_LOG (true: live adds enable_frame_drop_log:=true to them, 10-17; false = as given)
#        LIVE_CAMERA_FRAME (camera_color_optical_frame) LIVE_COLOR_TOPIC LIVE_DEPTH_TOPIC LIVE_INFO_TOPIC (the DEPTH
#        CameraInfo) LIVE_COLOR_INFO_TOPIC CAMERA_CHECK_S (20) RUNNER_CAMERA_QOS (auto = reliable on hardware)
#        (10-16 heartbeat-gap T2, measurement switches; the defaults keep the chain exactly as before)
#        PRODUCER_DEPTH_QOS (auto | reliable | sensor_data; auto = the producer's own default, reliable:5 on the real
#        camera; anything but auto is passed as --depth-qos) LIVE_DRIVER_DDS_PROFILE (a Fast DDS XML profile,
#        repo-relative or absolute, given to the camera DRIVER process alone as FASTRTPS_DEFAULT_PROFILES_FILE; board
#        default since 10-17 when unset: tools/nano_ros2/fastdds_shm8_localhost.xml = 8 MB shared-memory segment;
#        set it to empty for Fast DDS defaults; do not use the 32 MB fastdds_shm_localhost.xml, it falls back silently;
#        10-17 guard, whenever a profile is given: fastdds shm clean before every driver retry, and the start is
#        refused (stack_fail, rollback 7) when the driver node's own segment is smaller than the profile's segment_size)
#        BENCH_ODOM_HZ (0 = static) BENCH_MAP_M (8; 0 = no stand-in /map)   (--pose static only)
#        --pose odom (10-16, docs/design/SELECT_SEMANTICS_20261016.md section 3): the /map is a mapping export when one is
#        given -- CAR_MAP_YAML, or the one *.yaml in <dir of RTABMAP_DB>/run_<db stem>/export/ (refused, exit 2, when
#        given but not exactly one file); its frame matches only when the car starts at the mapping start, same heading.
#        Else the free stand-in, BENCH_MAP_M default 20 m (8 m left objects past ~3.7 m without an observation pose).
#        --pose odom --nav nav2 (10-17): the export is REQUIRED (refused, exit 2, dry run included, when neither
#        CAR_MAP_YAML nor an RTABMAP_DB is given) and goes to Nav2's map_server only (FPGA_NAV_MAP_YAML); bench_pose then
#        publishes map -> odom alone (--map-m 0, no --map-yaml): one publisher of /map.
#        the source snapshot: a git work tree, or (deployed with git archive) a tree with a COMMIT file
#      tests: CAR_DOCKER (docker) CAR_LOCK (the shared lock.sh) CAR_PY_HOST (conda python3.13) CAR_PY_BOARD (python3)
#        CAR_ARCH (uname -m) CAR_LSUSB (lsusb) CAR_USB_ROOT (/dev/bus/usb) CAR_KV_CHECK (hello_connect_loop.py)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
source "$HERE/car_common.sh"

CAMERA="" POSE="" NAV="" DETECTOR="" CHASSIS="" YOLO_BACKEND="" RUN="" RECORD=0 DRY=0
usage() { sed -n '2,20p' "$0" | sed 's/^# \{0,1\}//'; }
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then car_preset "$1" || { echo "unknown preset '$1' (sim|replay|live)" >&2; exit 2; }; shift; fi
while [ $# -gt 0 ]; do
  case "$1" in
    --camera) CAMERA=${2:-};; --pose) POSE=${2:-};; --nav) NAV=${2:-};; --detector) DETECTOR=${2:-};;
    --chassis) CHASSIS=${2:-};; --yolo-backend) YOLO_BACKEND=${2:-};; --gpu) INF_GPU=${2:-};;
    --run-dir) RUN=${2:-};;
    --record) RECORD=1; shift; continue;; --dry-run) DRY=1; shift; continue;;
    -h|--help) usage; exit 0;;
    *) echo "unknown argument '$1'" >&2; usage >&2; exit 2;;
  esac
  [ $# -ge 2 ] || { echo "$1 needs a value" >&2; exit 2; }
  shift 2
done
TELEOP=${CAR_TELEOP:-0}
MAPPING=${CAR_MAPPING:-0}   # 10-18: in-stack mapping beside Nav2 (car_validate; docs/design/TELEOP_NAV_ARB_20261018.md)
PREVIEW=${CAR_PREVIEW:-0}
case "$PREVIEW" in 0|1) ;; *) echo "CAR_PREVIEW must be 0 or 1" >&2; exit 2;; esac
# 10-26 car screen colour (D-042): CAR_AR_COLOUR=1 runs tools/kv_link/nano_colour.py in the stack (colour to the KV
# AR core on KV_SERVER's host, UDP CAR_AR_COLOUR_PORT 47801; display only: its exit never stops the stack).
# Default 0 = the container line as before (bench_line4 & co. unchanged); the competition nano_up.sh turns it on.
AR_COLOUR=${CAR_AR_COLOUR:-0}
case "$AR_COLOUR" in 0|1) ;; *) echo "CAR_AR_COLOUR must be 0 or 1" >&2; exit 2;; esac
case "${CAR_AR_COLOUR_PORT:=47801}" in ''|*[!0-9]*) echo "CAR_AR_COLOUR_PORT must be a port number" >&2; exit 2;; esac
# AR2NANO2 (2026-11-02): CAR_AR_NANO=1 = the car screen is drawn on this Nano (KV bitstream without the AR core):
# the producer gets --ar-nano (host/nano_ar.py), the board stack container gets /dev/dri, the colour sender sends to
# 127.0.0.1. Default 0 = every line as before; the competition nano_up.sh sets it from the KV's ar_reason.
AR_NANO=${CAR_AR_NANO:-0}
case "$AR_NANO" in 0|1) ;; *) echo "CAR_AR_NANO must be 0 or 1" >&2; exit 2;; esac
NAV_BACKEND=${CAR_NAV_BACKEND:-nav2}   # CTRLPS2 11-03 (header; car_validate)
NAV_DRYRUN=${CAR_NAV_DRYRUN:-0}   # 10-17: the board Nav2 dry run (car_validate: live static nav2 none only)
for v in CAMERA POSE NAV DETECTOR CHASSIS; do
  [ -n "${!v}" ] || { echo "missing --$(echo "$v" | tr 'A-Z' 'a-z') (or give a preset sim|replay|live)" >&2; exit 2; }
done
car_validate || exit 2
car_nav_costmap "${FPGA_NAV_COSTMAP:-}" || exit 2   # board nav2: empty = rolling (car_common.sh)

car_default_dirs "$PLATFORM"   # board (aarch64 or live): /data/fpga/car/{state,runs}; host: /data1/... (unchanged)
# the drive chain (10-15): remote control and wheel odometry (see the header)
: "${CAR_TELEOP_V_MAX:=}" "${CAR_TELEOP_W_MAX:=}" "${CAR_TELEOP_MAX_CMD_S:=}"
# rtabmap (10-15, POSE rtabmap|map): RTABMAP_ODOM = the odometry under rtabmap (rtabmap_car.sh ODOM). live: visual by
# default (rgbd_odometry publishes odom -> base_link, so the wheel odometry is forced off: one publisher of that edge;
# CAR_WHEEL_ODOM=1 given explicitly is refused); replay: wheel only (the replay publishes odom -> base_link itself).
# rtabmap (localization) + wheel is refused: the localization gate needs odom_info, which only rgbd_odometry gives.
# live + wheel (mapping) needs CAR_WHEEL_ODOM=1 (the wheel odometry is then the publisher of odom -> base_link).
WO_GIVEN=${CAR_WHEEL_ODOM-}
if [ "$POSE" = rtabmap ] || [ "$POSE" = map ]; then
  if [ "$CAMERA" = replay ]; then : "${RTABMAP_ODOM:=wheel}"; else : "${RTABMAP_ODOM:=visual}"; fi
  case "$RTABMAP_ODOM" in visual|wheel) ;; *) echo "RTABMAP_ODOM 只能是 visual 或 wheel（给的是 '$RTABMAP_ODOM'）" >&2; exit 2;; esac
  if [ "$CAMERA" = replay ] && [ "$RTABMAP_ODOM" = visual ]; then
    echo "不合法：replay 配 RTABMAP_ODOM=visual——回放自己发 odom->base_link，视觉里程会双发；回放建图用 wheel（TF 来自回放）" >&2; exit 2; fi
  if [ "$POSE" = rtabmap ] && [ "$RTABMAP_ODOM" = wheel ]; then
    echo "不合法：rtabmap 定位配 RTABMAP_ODOM=wheel——定位门要 odom_info，只有视觉里程给" >&2; exit 2; fi
  if [ "$RTABMAP_ODOM" = visual ]; then
    if [ -n "$WO_GIVEN" ] && [ "$WO_GIVEN" != 0 ]; then
      echo "不合法：RTABMAP_ODOM=visual 配 CAR_WHEEL_ODOM=$WO_GIVEN——odom->base_link 只能一个发布者（视觉里程）；去掉 CAR_WHEEL_ODOM 或给 0" >&2; exit 2; fi
    CAR_WHEEL_ODOM=0
  elif [ "$CAMERA" = live ]; then
    [ "$CHASSIS" = uart ] && : "${CAR_WHEEL_ODOM:=1}"
    if [ "${CAR_WHEEL_ODOM:-}" != 1 ]; then
      echo "不合法：live 配 RTABMAP_ODOM=wheel 而 CAR_WHEEL_ODOM≠1——odom->base_link 无人发（轮式里程要 --chassis uart）" >&2; exit 2; fi
  fi
fi
[ "$CHASSIS" = uart ] && : "${CAR_WHEEL_ODOM:=1}"; : "${CAR_WHEEL_ODOM:=0}"
case "$CAR_WHEEL_ODOM" in 0|1) ;; *) echo "CAR_WHEEL_ODOM 只能是 0 或 1（给的是 '$CAR_WHEEL_ODOM'）" >&2; exit 2;; esac
if [ "$CAR_WHEEL_ODOM" = 1 ] && [ "$CHASSIS" = none ]; then
  echo "不合法：CAR_WHEEL_ODOM=1 配 --chassis none——轮式里程的计数来自底盘驱动" >&2; exit 2; fi
if [ "$POSE" = odom ] && [ "$CAR_WHEEL_ODOM" != 1 ]; then
  echo "不合法：odom 位姿配 CAR_WHEEL_ODOM=0——odom->base_link 无人发" >&2; exit 2; fi
: "${CAR_WHEEL_DIAM_M:=}" "${CAR_COUNTS_PER_REV:=}" "${CAR_TRACK_M:=}"
# owner 10-16 (DECISIONS D-041): the real car's wheel parameters are the board default -- wheel diameter 0.081 m
# the MCU's mecanum factor; PROGRESS-22). Host (sim/replay) keeps empty = the placeholder of host/wheel_odom.py (the
# fake MCU and the PL reference model are built on it). WHEEL_GIVEN keeps the raw environment for car.json.
WHEEL_GIVEN=("$CAR_WHEEL_DIAM_M" "$CAR_COUNTS_PER_REV" "$CAR_TRACK_M")
WHEEL_SRC=""
if [ "$PLATFORM" = board ]; then
  WHEEL_SRC="wheel_diam_m=$([ -n "$CAR_WHEEL_DIAM_M" ] && echo env || echo default)"
  WHEEL_SRC+=" counts_per_rev=$([ -n "$CAR_COUNTS_PER_REV" ] && echo env || echo default)"
  WHEEL_SRC+=" track_m=$([ -n "$CAR_TRACK_M" ] && echo env || echo default)"
  : "${CAR_WHEEL_DIAM_M:=0.081}" "${CAR_COUNTS_PER_REV:=1560}" "${CAR_TRACK_M:=0.394}"
  case "$WHEEL_SRC" in *env*default*|*default*env*) WHEEL_SRC="mixed ($WHEEL_SRC)";; *env*) WHEEL_SRC="env (CAR_WHEEL_DIAM_M/CAR_COUNTS_PER_REV/CAR_TRACK_M)";;
                       *) WHEEL_SRC="default (real car, 10-16 calibration, D-041)";; esac
fi
: "${CAR_READY_S:=900}" "${INF_READY_S:=120}" "${INF_GPU:=2}" "${CAR_LOCK_USER:=dev1}"
: "${CAR_DOCKER:=docker}" "${CAR_LOCK:=bash ${FPGA_HOME}/tools/lock.sh}"
: "${CAR_PY_HOST:=${FPGA_HOME}/miniconda3/bin/python3.13}"
: "${GZ_CPUS:=36-41}" "${KV_CPUS:=18,42,19,43}" "${KV_THREADS:=4}" "${STACK_CPUS:=23,47,12-17}"
: "${RUNTIME_CPUS:=22,46}" "${INF_CPUS:=20,21,44,45}" "${INF_WARMUP:=2}" "${INF_WARMUP_SIZE:=640x400}" "${AUG:=none}"
: "${MCU_HEARTBEAT:=kv}" "${FPGA_SIM_WORLD_FILE=room_carmount_d2.sdf}" "${YOLO_THREADS:=4}"
: "${YOLO_PT:=${FPGA_SHARED}/models/20260914-dev3-m3-single-image/yolov8n-seg.pt}"
: "${YOLO_ONNX:=${FPGA_SRC}/runs/yolo-runner-20261014-results/export/yolov8n-seg-640.onnx}"
: "${YOLO_UL_PATH:=${FPGA_HOME}/dev1/.t/ya/ul8340}"
: "${MODELS:=${FPGA_SHARED}/models/20260907-dev1-replay}"
: "${PREP_HOST:=${FPGA_SRC}/runs/ar-display-20261011-scratch/rec30_prep}"
: "${IMAGE:=fpga-dev1-sim:humble}"
# board (live) defaults: the KV260 of the integration runs, the Nano image with the Orbbec driver.
# KV_BUILD_ID: owner 10-14, the A2 merged bitstream m7_pl_merge_a2 (BUILD_ID 0x00da5041, session_v3.sh BID) is this
# batch's record bitstream -- board only: the host's fake KV (kv_server --core cpu) is not checked for a build id.
: "${KV_SERVER:=198.51.100.79:47800}" "${RTABMAP_DB:=}" "${CHASSIS_PORT:=/dev/fpga_chassis}"
# FPGA profile (go-live 2026-10-26, ONE place: tools/car_run/fpga_profiles.env; board only): default v3fc2 since its 10-26 board
# short test; v3fc2 | v3fc2_166 | v3fc | m1_166 | m1 | m2_166 | m2 = the producer fusing on the KV260 PL (--rolling-backend remote_pl; v3fc3_187 is paused
# and runs v3fc2 with a warning); FPGA_PROFILE=a2 = the rollback, the 10-14 chain verbatim (KV_BUILD_ID 0x00da5041, rolling on the Nano CPU). kv_precheck accepts any registered BUILD_ID
# (kv_accept_registered). The host keeps cpu, no build id.
if [ "$PLATFORM" = board ]; then
  source "$HERE/fpga_profile.sh"; fpga_profile || exit 2
  : "${KV_BUILD_ID:=$FPGA_BUILD_ID}" "${ROLLING_BACKEND:=$FPGA_NANO_ROLLING}"
  # SW_M1 (2026-11-03): a profile whose bitstream has no AR core (fpga_profiles.env <P>_AR=nano: m1_166 / m1 / m2_166 / m2) draws the
  # car screen on this Nano by itself; CAR_AR_NANO given by the caller (0|1) wins; every other profile: as before
  if [ -z "${CAR_AR_NANO:-}" ] && [ "${FPGA_AR:-kv}" = nano ]; then AR_NANO=1; fi
else : "${KV_BUILD_ID:=}" "${ROLLING_BACKEND:=cpu}"; fi
# the real camera's wiring (10-15; same defaults as car_inner.sh, passed in so that they can be changed from outside
# and the dry run shows them): driver launch arguments (the start-up investigation may change them), ONE camera frame
# for the mount TF / producer / runner (depth hardware-aligned to colour), the driver's topics (CameraInfo = the depth
# one: colour intrinsics, colour frame, zero distortion), the runner's camera QoS (auto = reliable on hardware)
: "${CAR_CAMERA_FPS:=15}"   # 10-29 owner (Nano CPU): colour and depth rate (header); 30 = before
: "${CAR_PRIORITY:=1}"   # 10-29 owner (Nano CPU): stack CPU priorities on the board (header); 0 = before
case "$CAR_PRIORITY" in 0|1) ;; *) echo "CAR_PRIORITY 只能是 0|1（给的是 '$CAR_PRIORITY'）" >&2; exit 2;; esac
case "$CAR_CAMERA_FPS" in 5|10|15|30) ;; *) echo "CAR_CAMERA_FPS 只能是 5|10|15|30（给的是 '$CAR_CAMERA_FPS'）" >&2; exit 2;; esac
: "${LIVE_LAUNCH_ARGS:=color_width:=640 color_height:=480 color_fps:=$CAR_CAMERA_FPS color_format:=RGB depth_width:=640 depth_height:=400 depth_fps:=$CAR_CAMERA_FPS depth_format:=Y16 enable_ir:=false enable_point_cloud:=false depth_registration:=true align_mode:=HW publish_tf:=false}"
# an explicit color_fps:= in LIVE_LAUNCH_ARGS is the rate the driver gets (car_inner.sh reads it the same way)
case " $LIVE_LAUNCH_ARGS " in *" color_fps:="*) CAR_CAMERA_FPS=$(printf '%s\n' " $LIVE_LAUNCH_ARGS " | sed -n 's/.* color_fps:=\([^ ]*\) .*/\1/p');; esac
case "$CAR_CAMERA_FPS" in 5|10|15|30) ;; *) echo "LIVE_LAUNCH_ARGS color_fps:= 只能是 5|10|15|30（给的是 '$CAR_CAMERA_FPS'）" >&2; exit 2;; esac
# the driver's frame-drop log (10-17, car_common.sh car_frame_drop_log): live adds enable_frame_drop_log:=true;
# LIVE_FRAME_DROP_LOG=false turns it off (passed into the stack so that car_inner.sh does not add it again)
if [ "$CAMERA" = live ]; then car_frame_drop_log || exit 2; fi
: "${LIVE_CAMERA_FRAME:=camera_color_optical_frame}" "${LIVE_COLOR_TOPIC:=/camera/color/image_raw}"
: "${LIVE_DEPTH_TOPIC:=/camera/depth/image_raw}" "${LIVE_INFO_TOPIC:=/camera/depth/camera_info}"
: "${LIVE_COLOR_INFO_TOPIC:=/camera/color/camera_info}" "${CAMERA_CHECK_S:=20}" "${RUNNER_CAMERA_QOS:=auto}"
: "${PRODUCER_DEPTH_QOS:=auto}"
# camera driver's Fast DDS profile (10-17, owner approved 10-17 00:10 after T2 A2/B1b): on the board an UNSET variable
# means the 8 MB shared-memory profile (the 512 KB default segment holds neither a 614472 B depth nor a 921672 B colour
# message; reliable repair then blocks the producer: heartbeat gaps). Explicitly empty = Fast DDS defaults (the chain
# before 10-17). Never the 32 MB fastdds_shm_localhost.xml: through ros2 launch it silently falls back (10-16).
if [ "$PLATFORM" = board ]; then : "${LIVE_DRIVER_DDS_PROFILE=tools/nano_ros2/fastdds_shm8_localhost.xml}"
else : "${LIVE_DRIVER_DDS_PROFILE:=}"; fi
case "$PRODUCER_DEPTH_QOS" in auto|reliable|sensor_data) ;;
  *) echo "PRODUCER_DEPTH_QOS 只能是 auto|reliable|sensor_data（给的是 '$PRODUCER_DEPTH_QOS'）" >&2; exit 2;; esac
if [ -n "$LIVE_DRIVER_DDS_PROFILE" ]; then
  case "$LIVE_DRIVER_DDS_PROFILE" in /*) DDSP_HOST=$LIVE_DRIVER_DDS_PROFILE ;; *) DDSP_HOST=$REPO/$LIVE_DRIVER_DDS_PROFILE ;; esac
  [ -f "$DDSP_HOST" ] || { echo "LIVE_DRIVER_DDS_PROFILE '$LIVE_DRIVER_DDS_PROFILE': no such file ($DDSP_HOST)" >&2; exit 2; }
fi
: "${BENCH_ODOM_HZ:=0}"
if [ "$POSE" = odom ]; then : "${BENCH_MAP_M:=20}"; else : "${BENCH_MAP_M:=8}"; fi
# nav2 (10-17): no stand-in /map -- Nav2's map_server publishes /map; two publishers of one latched topic would
# give the runner and the costmaps whichever came last. bench_pose.py --map-m 0 (static and odom alike)
[ "$NAV" = nav2 ] && BENCH_MAP_M=0
# YOLO engine and the board's Python packages (TensorRT engine built by P2; ultralytics etc. installed with
# pip --target into ~/fpga/p2/pylib, docs/design/NANO_DETECTOR_20260927.md; tools/nano_loop/run_p3.sh uses both).
# Engine default since 2026-10-15: rebuilt under the super firmware from the same ONNX (f92183cf) with the same
# trtexec flags, sha ee8ce118...; re-checked against the 10-14 criteria (runs/yolo-engine-rebuild-20261015-results).
# The 09-27 engine ~/fpga/p2/w/yolov8n-seg-640-fp16.engine (sha 0a111806...) is kept; YOLO_ENGINE=<it> selects it.
: "${YOLO_ENGINE:=/data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine}" "${NANO_PYLIB:=/home/robot/fpga/p2/pylib}"
# BOARD_IMAGE default (10-17, controller): the scipy derivative of fpga-ros:humble-nav-orbbec-20260927 (base + python3-scipy
# 1.8.0 only, 7 packages added, none changed; tools/nano_ros2/Dockerfile.scipy) -- the localization gate and odom adapter need scipy
: "${BOARD_IMAGE:=fpga-ros:humble-nav-orbbec-scipy-20261017}" "${CAR_PY_BOARD:=python3}" "${CAR_ARCH:=$(uname -m)}"
: "${LIVE_START_TRIES:=10}" "${LIVE_START_WAIT_S:=20}" "${LIVE_START_GAP_S:=10}" "${LIVE_DEPTH_MSGS:=30}"
: "${CAR_CAMERA_RATE_CHECK:=1}" "${CAR_CAMERA_RATE_S:=3}"
case "$CAR_CAMERA_RATE_CHECK" in 0|1) ;; *) echo "CAR_CAMERA_RATE_CHECK 只能是 0|1（给的是 '$CAR_CAMERA_RATE_CHECK'）" >&2; exit 2;; esac
case "$CAR_CAMERA_RATE_S" in [1-5]) ;; *) echo "CAR_CAMERA_RATE_S 只能是 1-5 的整数秒（给的是 '$CAR_CAMERA_RATE_S'）" >&2; exit 2;; esac
: "${CAR_KV_CHECK:=env PYTHONPATH=$REPO python3 $REPO/tools/kv_link/hello_connect_loop.py}"
# the live car's camera mount (owner 10-14: lens centre 0.36 m, 30 deg down; 10-16: 0.375 m, see below). tools/car_geo_producer/common.py keeps
# 0.35 m / 27 deg for replay and the host runs (comparable with the earlier runs); only the board gets these.
# owner 10-16: hand-measured on the car 0.375 m (the start-up ground fit 0.3788 m / 26.8 deg), 30 deg.
# owner 10-17 14:16: the camera re-tilted by hand ("about 10 deg"); the board's ground fit (c10_1, 14:31) measured
# 0.3915 m / 7.05 deg / roll 0.59 deg -> defaults 0.39 m / 7.0 deg (controller 10-17). On the board auto at 0.375 / 10
# was refused by the inlier share (48 %), at 0.39 / 7.0 by the then 0.39 m height hard limit; since then (same day)
# the ground calibration selects 0.3-1.4 m below 20 deg pitch and the hard limit is 0.395 m (owner approved): auto
# adopts on the c10_1 depth offline. docs/design/CAM_TILT_10_20261017.md section 9. Car body height 0.31 m (the tower's top plate 0.325 m, the camera itself up to about
# 0.40 m: overhangs below 0.40 m are kept out of the demo site; DECISIONS.md, owner 10-16).
# 10-28: the ground calib no longer depends on this pitch to find the floor (last good prior, then this, then a
# 0-35 deg sweep) and falls back to the LAST adopted fit ($CAR_STATE_DIR/ground_calib_last_good.json) before it;
# 10-28 afternoon the owner re-tilted the camera; settled mount measured by the board fit 0.392 m / 15.75 deg /
# roll 0.87 deg (6128 inliers, rms 1.6 mm, bend 0.37 mm) -> default 16 (below 20: selection range 0.3-1.4 m).
: "${CAR_CAM_HEIGHT:=0.39}" "${CAR_CAM_PITCH_DEG:=16.0}" "${CAR_HEIGHT_CELL:=}" "${CAR_ROBOT_HEIGHT:=0.31}"
# owner 10-15: the height is hand-measured; the two lens centres are at about the same height.
# Recorded in car.json camera_mount; the line-4 bench passes CAR_HEIGHT_CELL=8 (holds the ground over 0.34-0.38 m).
: "${CAR_CAM_HEIGHT_UNC_M:=0.01}"
: "${CAR_GROUND_CALIB:=auto}"
case "$CAR_GROUND_CALIB" in auto|off|check) ;;
  *) echo "CAR_GROUND_CALIB 只能是 auto|off|check（给的是 '$CAR_GROUND_CALIB'）" >&2; exit 2;; esac
MOUNT_ENV=()
if [ "$PLATFORM" = board ]; then
  MOUNT=$(python3 "$HERE/car_mount.py" --height-m "$CAR_CAM_HEIGHT" --pitch-deg "$CAR_CAM_PITCH_DEG" --height-cell "$CAR_HEIGHT_CELL") \
    || { echo "board: camera mount CAR_CAM_HEIGHT=$CAR_CAM_HEIGHT CAR_CAM_PITCH_DEG=$CAR_CAM_PITCH_DEG CAR_HEIGHT_CELL=$CAR_HEIGHT_CELL refused (above)" >&2; exit 2; }
  read -r HEIGHT_CELL MOUNT_TF <<< "$MOUNT"
  # the configured camera against the car body height already here (the stack checks the adopted height again)
  python3 "$HERE/car_mount.py" --height-m "$CAR_CAM_HEIGHT" --robot-height-m "$CAR_ROBOT_HEIGHT" > /dev/null \
    || { echo "board: CAR_CAM_HEIGHT=$CAR_CAM_HEIGHT with CAR_ROBOT_HEIGHT=$CAR_ROBOT_HEIGHT refused (above)" >&2; exit 2; }
  MOUNT_ENV=(CAM_HEIGHT="$CAR_CAM_HEIGHT" CAM_PITCH_DEG="$CAR_CAM_PITCH_DEG" HEIGHT_CELL="$HEIGHT_CELL" CAM_MOUNT_TF="$MOUNT_TF"
             GROUND_CALIB="$CAR_GROUND_CALIB" HEIGHT_CELL_GIVEN="$CAR_HEIGHT_CELL" ROBOT_HEIGHT="$CAR_ROBOT_HEIGHT")
fi
DK() { $CAR_DOCKER "$@"; }
POINTER=$CAR_STATE_DIR/car.current

# ---------------------------------------------------------------------------------------------------
if [ "$PLATFORM" = board ] && [ "$DRY" != 1 ]; then
  if [ "$CAR_ARCH" != aarch64 ]; then
    echo "live 组合只在板上起（上板归第二类）；本宿主只能 --dry-run" >&2; exit 3; fi
  [ "${CAR_BOARD_DRAFT:-0}" = 1 ] || { echo "board path is a draft not yet run on a board: set CAR_BOARD_DRAFT=1 to try it" >&2; exit 3; }
  # owner 10-14: the board hub listens only on the car's LAN address (token required), never on all interfaces
  case "${CAR_HUB_BIND:-}" in ''|0.0.0.0|::)
    echo "board hub: set CAR_HUB_BIND to the car's LAN IP (not empty, 0.0.0.0 or ::)" >&2; exit 2;; esac
fi
[ -n "$RUN" ] || RUN=$CAR_RUNS_DIR/$(date +%Y%m%d-%H%M%S)-$CAMERA
case "$RUN" in /*) ;; *) RUN=$PWD/$RUN;; esac
# rtabmap databases (POSE rtabmap|map): board RTABMAP_DB_DIR (/data/fpga/rtabmap), mounted read-write at the same path
# (map: the new DB_DIR/<time>.db + run_<time>/; loc: run_<time>/ with the working copy); host: $RUN/rtabmap_db, which
# the stack sees as /o/rtabmap_db. The stack's stop wait: rtabmap saves its database on INT (rtabmap_car.sh:
# SAVE_TIMEOUT_S 120 + the other children + the export) -- car_inner waits CAR_RTABMAP_STOP_S for it, car_down
# CAR_RTABMAP_STOP_S + 90 for the whole stack ($RUN/stack_stop_s.txt; an explicit CAR_STACK_STOP_S wins there).
: "${CAR_RTABMAP_STOP_S:=240}" "${CAR_GATE_MODE:=correction}" "${CAR_GATE_X_S:=2}"
: "${CAR_GATE_ODOM_JUMP:=1}" "${CAR_GATE_ODOM_V_MPS:=0.5}" "${CAR_GATE_ODOM_E_M:=0.10}"
case "$CAR_RTABMAP_STOP_S" in ''|*[!0-9]*) echo "CAR_RTABMAP_STOP_S must be whole seconds (given '$CAR_RTABMAP_STOP_S')" >&2; exit 2;; esac
RTAB=0; [ "$POSE" = rtabmap ] || [ "$POSE" = map ] && RTAB=1
# CAR_MAPPING=1 (10-18): the database directory, the stop wait and car.json "rtabmap" as for a mapping run
[ "$MAPPING" = 1 ] && { RTAB=1; RTABMAP_ODOM=wheel; }   # mapping_ctl runs rtabmap_car.sh with ODOM=wheel
if [ "$MAPPING" = 1 ] && { [ -n "${CAR_MAP_YAML:-}" ] || [ -n "${RTABMAP_DB:-}" ]; }; then
  echo "不合法：CAR_MAPPING=1 配 CAR_MAP_YAML/RTABMAP_DB——并栈建图起步用空白图（blank_map.py），结束建图后换成本趟导出图；不要给已有地图" >&2
  exit 2
fi
RTABMAP_DB_DIR_IN=""
if [ "$RTAB" = 1 ]; then
  if [ "$PLATFORM" = board ]; then : "${RTABMAP_DB_DIR:=/data/fpga/rtabmap}"; RTABMAP_DB_DIR_IN=$RTABMAP_DB_DIR
  else RTABMAP_DB_DIR=$RUN/rtabmap_db; RTABMAP_DB_DIR_IN=/o/rtabmap_db; fi
fi
# board Nav2 (NAV nav2): the map is rtabmap-export --map of the pinned database: CAR_MAP_YAML, or the one *.yaml in
# <dir of RTABMAP_DB>/run_<db stem>/export/ (rtabmap_car.sh map writes it there); the real start refuses (2) without
# it; its directory is mounted read-only at the same path
: "${CAR_MAP_YAML:=}"
MAP_YAML_DESC=""
if [ "$NAV" = nav2 ] && [ "$PLATFORM" = board ]; then
  if [ -z "$CAR_MAP_YAML" ] && [ -n "$RTABMAP_DB" ]; then
    EXPORT_DIR="$(dirname "$RTABMAP_DB")/run_$(basename "$RTABMAP_DB" .db)/export"
    YAMLS=("$EXPORT_DIR"/*.yaml)
    if [ ${#YAMLS[@]} = 1 ] && [ -f "${YAMLS[0]}" ]; then CAR_MAP_YAML=${YAMLS[0]}
    else MAP_YAML_DESC="$EXPORT_DIR/*.yaml (not exactly one file now)"; fi
  fi
  [ -n "$CAR_MAP_YAML" ] && MAP_YAML_DESC=$CAR_MAP_YAML
  [ -n "$MAP_YAML_DESC" ] || MAP_YAML_DESC="<CAR_MAP_YAML: none (RTABMAP_DB empty)>"
  [ "$MAPPING" = 1 ] && MAP_YAML_DESC="blank start map RUN/blank_map/blank.yaml (CAR_MAPPING=1: car_inner makes it before the pose step)"
fi
# --pose odom (10-16): a mapping export as /map instead of the stand-in (see the header). ODOM_MAP_YAML empty = stand-in.
ODOM_MAP_YAML=""
if [ "$POSE" = odom ]; then
  if [ -n "$CAR_MAP_YAML" ]; then ODOM_MAP_YAML=$CAR_MAP_YAML
  elif [ -n "$RTABMAP_DB" ]; then
    EXPORT_DIR="$(dirname "$RTABMAP_DB")/run_$(basename "$RTABMAP_DB" .db)/export"
    YAMLS=("$EXPORT_DIR"/*.yaml)
    if [ ${#YAMLS[@]} = 1 ] && [ -f "${YAMLS[0]}" ]; then ODOM_MAP_YAML=${YAMLS[0]}; CAR_MAP_YAML=$ODOM_MAP_YAML
    else ODOM_MAP_YAML="$EXPORT_DIR/*.yaml (not exactly one file now)"; fi
  fi
fi
# odom + nav2 (10-17): Nav2's map_server needs the mapping export -- no stand-in for Nav2. Refused here, before
# anything is created and in the dry run too, when nothing names one (a named path that is not exactly one file is
# refused by the real start below, as for every board nav2 run).
if [ "$POSE" = odom ] && [ "$NAV" = nav2 ] && [ -z "$CAR_MAP_YAML" ] && [ -z "$RTABMAP_DB" ] && [ "$MAPPING" != 1 ]; then
  echo "不合法：odom 位姿配 nav2 没有地图——Nav2 的 map_server 要建图导出的地图（rtabmap-export）：给 CAR_MAP_YAML=<导出的 .yaml>，或 RTABMAP_DB=<建图库>（取 <库目录>/run_<库名>/export/ 下唯一的 *.yaml）；车须从建图起点、同朝向起栈" >&2
  exit 2
fi
# --- object memory across runs (10-18, docs/design/MEMORY_PERSIST_20261018.md; host/memory_store.py) ----------------
# CAR_MEMORY_SAVE (1): the runner writes RUN/memory_final.json at a normal stop (car_down's INT) and
#   RUN/memory_snapshot.json while running (on change, at most every CAR_MEMORY_SNAPSHOT_S = 5 s); 0 = neither.
# CAR_MEMORY_IN=<memory file of an earlier run>: loaded by the runner at start. Refused (exit 2, dry run included,
#   before anything is created) unless the file is a complete memory of this format/version whose map identity
#   equals this run's: odom = the served export (CAR_MAP_YAML / the RTABMAP_DB export; its run_<stem>/sha256.txt names
#   the database), rtabmap = RTABMAP_DB, sim = the simulator's Nav2 map; no map (identity none) never matches.
#   The checked file is copied to RUN/memory_in.json (the runner reads that copy; car.json "memory" records both sha256).
: "${CAR_MEMORY_IN:=}" "${CAR_MEMORY_SAVE:=1}" "${CAR_MEMORY_SNAPSHOT_S:=5}"
# runner budgets on the board (header: CAR_NAV_TIMEOUT_S / CAR_TASK_TIMEOUT_S); refused (exit 2) before anything starts
# 10-26 owner：重成功率，放宽自 60 / 180
: "${CAR_NAV_TIMEOUT_S:=90}" "${CAR_TASK_TIMEOUT_S:=300}"
# 10-26 owner：重成功率，失败上限放宽自 3；位姿可信门车上先关（陀螺比例标定好再开）
: "${CAR_MAX_NAV_FAILURES:=5}" "${CAR_POSE_TRUST:=off}"
for _v in CAR_NAV_TIMEOUT_S CAR_TASK_TIMEOUT_S; do
  [[ "${!_v}" =~ ^[0-9]+([.][0-9]+)?$ ]] && [[ ! "${!_v}" =~ ^0+([.]0+)?$ ]] \
    || { echo "$_v 须是正数秒（给的是 '${!_v}'）" >&2; exit 2; }
done
# 10-26 frame-gap watchdog on the board (header: CAR_STREAM_TIMEOUT_S), 2..20 s; refused (exit 2) before anything starts
: "${CAR_STREAM_TIMEOUT_S:=5.0}"
[[ "$CAR_STREAM_TIMEOUT_S" =~ ^[0-9]+([.][0-9]+)?$ ]] \
  && python3 -c 'import sys; sys.exit(0 if 2.0 <= float(sys.argv[1]) <= 20.0 else 1)' "$CAR_STREAM_TIMEOUT_S" \
  || { echo "CAR_STREAM_TIMEOUT_S 须是 2 到 20 之间的秒数（给的是 '$CAR_STREAM_TIMEOUT_S'）" >&2; exit 2; }
# 10-26 search mode (header: CAR_SEARCH_MODE): prob (default, the belief search) | mvp (the 10-26 search MVP's list)
: "${CAR_SEARCH_MODE:=prob}"
case "$CAR_SEARCH_MODE" in prob|mvp) ;; *) echo "CAR_SEARCH_MODE 只能是 prob 或 mvp（给的是 '$CAR_SEARCH_MODE'）" >&2; exit 2;; esac
# 10-27 live alignment while mapping (header: CAR_LIVE_ALIGN), default on
: "${CAR_LIVE_ALIGN:=1}"
case "$CAR_LIVE_ALIGN" in 0|1) ;; *) echo "CAR_LIVE_ALIGN 只能是 0 或 1（给的是 '$CAR_LIVE_ALIGN'）" >&2; exit 2;; esac
# 10-29 Nav2 global map while mapping (header: CAR_NAV_LIVE_GLOBAL), default on
: "${CAR_NAV_LIVE_GLOBAL:=1}"
case "$CAR_NAV_LIVE_GLOBAL" in 0|1) ;; *) echo "CAR_NAV_LIVE_GLOBAL 只能是 0 或 1（给的是 '$CAR_NAV_LIVE_GLOBAL'）" >&2; exit 2;; esac
# 10-29 local-layer obstacles in that global map (header: CAR_NAV_GLOBAL_ROLLING), default on
: "${CAR_NAV_GLOBAL_ROLLING:=1}"
case "$CAR_NAV_GLOBAL_ROLLING" in 0|1) ;; *) echo "CAR_NAV_GLOBAL_ROLLING 只能是 0 或 1（给的是 '$CAR_NAV_GLOBAL_ROLLING'）" >&2; exit 2;; esac
# 10-26 front-sector seed (header: CAR_FRONT_SEED_M), empty = off; refused (exit 2) before anything starts
: "${CAR_FRONT_SEED_M:=}"
if [ -n "$CAR_FRONT_SEED_M" ]; then
  [[ "$CAR_FRONT_SEED_M" =~ ^[0-9]+([.][0-9]+)?$ ]] && [[ ! "$CAR_FRONT_SEED_M" =~ ^0+([.]0+)?$ ]] \
    && python3 -c 'import sys; sys.exit(0 if float(sys.argv[1]) <= 1.0 else 1)' "$CAR_FRONT_SEED_M" \
    || { echo "CAR_FRONT_SEED_M 须为空（关）或不超过 1.0 的正数米（给的是 '$CAR_FRONT_SEED_M'）" >&2; exit 2; }
fi
# 10-26 host sim only: CAR_SIM_NAV_PROFILE empty | sim (as before) | car (the car's Nav2 switches + runner budgets)
: "${CAR_SIM_NAV_PROFILE:=}"
case "$CAR_SIM_NAV_PROFILE" in ''|sim) CAR_SIM_NAV_PROFILE=;; car) ;; *) echo "CAR_SIM_NAV_PROFILE 只能是 sim 或 car（给的是 '$CAR_SIM_NAV_PROFILE'）" >&2; exit 2;; esac
[ -n "$CAR_SIM_NAV_PROFILE" ] && [ "$CAMERA" != sim ] && { echo "CAR_SIM_NAV_PROFILE=car 只用于宿主仿真（sim 相机）；真车本就走车档" >&2; exit 2; }
[ -n "$CAR_SIM_NAV_PROFILE" ] && [ "$NAV" != nav2 ] && { echo "CAR_SIM_NAV_PROFILE=car 须配 --nav nav2" >&2; exit 2; }
# 10-26 board Nav2 switches and DWB minimum speeds (header), refused (exit 2) before anything starts
NAV_SW_ENV=()
for _v in FPGA_NAV_ROTATION_SHIM:true,false FPGA_NAV_PROGRESS_CHECKER:pose,simple FPGA_NAV_CAR_PLANNER:strict,relaxed; do
  _n=${_v%%:*}; _ok=${_v#*:}; _val=${!_n:-}
  [ -n "$_val" ] || continue
  case ",$_ok," in *",${_val,,},"*) ;; *) echo "$_n 只能是 ${_ok/,/ 或 }（给的是 '$_val'）" >&2; exit 2;; esac
  NAV_SW_ENV+=("$_n=$_val")
done
DWB_OVR_ENV=()
[ -n "${CAR_DWB_MIN_SPEED_THETA:-}" ] && DWB_OVR_ENV+=(DWB_MIN_SPEED_THETA="$CAR_DWB_MIN_SPEED_THETA")
[ -n "${CAR_DWB_MIN_SPEED_XY:-}" ] && DWB_OVR_ENV+=(DWB_MIN_SPEED_XY="$CAR_DWB_MIN_SPEED_XY")
[ -n "${CAR_ROT_STOPPED_VELOCITY:-}" ] && DWB_OVR_ENV+=(ROT_STOPPED_VELOCITY="$CAR_ROT_STOPPED_VELOCITY")
# the launch's own check (nav_input_profile.car_min_speed_overrides), so car_up and Nav2 accept the same values. A
# deployed tree without robot_sim (bench archives): nothing given = no check, car.json says so; given = refused there
NIP=$REPO/robot_sim/fpga_sim/config/nav_input_profile.py
DWB_JSON='{"min_speed_xy": null, "min_speed_theta": null, "rot_stopped_velocity": null, "overridden": []}'
if [ ! -f "$NIP" ] && [ ${#DWB_OVR_ENV[@]} -gt 0 ]; then
  echo "CAR_DWB_MIN_SPEED_THETA / CAR_DWB_MIN_SPEED_XY / CAR_ROT_STOPPED_VELOCITY 被拒：本部署树没有 $NIP（无车档 Nav2）" >&2; exit 2
fi
if [ -f "$NIP" ]; then DWB_JSON=$(FPGA_NAV_DWB_MIN_SPEED_THETA="${CAR_DWB_MIN_SPEED_THETA:-}" FPGA_NAV_DWB_MIN_SPEED_XY="${CAR_DWB_MIN_SPEED_XY:-}" \
  FPGA_NAV_ROT_STOPPED_VELOCITY="${CAR_ROT_STOPPED_VELOCITY:-}" python3 -c '
import json, os, runpy, sys
try:
    m, r, o = runpy.run_path(sys.argv[1])["car_min_speed_overrides"](os.environ)
except ValueError as e:
    sys.exit(str(e))
print(json.dumps(dict(m, rot_stopped_velocity=r, overridden=sorted(o))))' "$NIP") \
  || { echo "CAR_DWB_MIN_SPEED_THETA / CAR_DWB_MIN_SPEED_XY / CAR_ROT_STOPPED_VELOCITY 被拒（见上；空=用常量 0.10 / 0.015 / 0.15）" >&2; exit 2; }
fi
# 10-29 the car's DWB forward_prune_distance (header), checked by the launch's own function
PRUNE_JSON='{"m": null, "given": false}'
if [ ! -f "$NIP" ] && [ -n "${CAR_DWB_FORWARD_PRUNE_M:-}" ]; then
  echo "CAR_DWB_FORWARD_PRUNE_M 被拒：本部署树没有 $NIP（无车档 Nav2）" >&2; exit 2
fi
if [ -f "$NIP" ]; then PRUNE_JSON=$(FPGA_NAV_DWB_FORWARD_PRUNE_M="${CAR_DWB_FORWARD_PRUNE_M:-}" python3 -c '
import json, os, runpy, sys
try:
    m, g = runpy.run_path(sys.argv[1])["car_forward_prune"](os.environ)
except ValueError as e:
    sys.exit(str(e))
print(json.dumps({"m": m, "given": g}))' "$NIP") \
  || { echo "CAR_DWB_FORWARD_PRUNE_M 被拒（见上；空=用常量 0.8 m，2.0=改前的 Nav2 默认）" >&2; exit 2; }
fi
[ -n "${CAR_DWB_FORWARD_PRUNE_M:-}" ] && DWB_OVR_ENV+=(DWB_FORWARD_PRUNE_M="$CAR_DWB_FORWARD_PRUNE_M")   # 10-29
case "$CAR_MEMORY_SAVE" in 0|1) ;; *) echo "CAR_MEMORY_SAVE 只能是 0 或 1（给的是 '$CAR_MEMORY_SAVE'）" >&2; exit 2;; esac
MEM_PY=python3   # stdlib only (as the car.json step); CAR_PY_HOST/BOARD are the inference interpreters
MEM_ID_ARGS=() MEM_ID_DESC="none (no map in this run)"
# CAR_MAPPING=1 (10-18, docs/design/TELEOP_NAV_ARB_20261018.md section 2.4): this run builds the map -- identity
# mapping_pending on the database directory as the STACK sees it (rtabmap_car.sh's sha256.txt lists the database by its
# stack path; memory_store bind compares the two); car_down binds the memory file to the saved database
# (tools/car_run/mapping_memory_bind.py). A pending identity matches nothing, so CAR_MEMORY_IN is refused here.
if [ "$MAPPING" = 1 ]; then MEM_ID_ARGS=(--mapping-db-dir "$RTABMAP_DB_DIR_IN"); MEM_ID_DESC="mapping run (pending: car_down binds the new database in $RTABMAP_DB_DIR_IN)"
elif [ "$POSE" = odom ] && [ -n "$ODOM_MAP_YAML" ]; then MEM_ID_ARGS=(--map-yaml "$ODOM_MAP_YAML"); MEM_ID_DESC="odom export $ODOM_MAP_YAML"
elif [ "$POSE" = rtabmap ] && [ -n "$RTABMAP_DB" ]; then MEM_ID_ARGS=(--rtabmap-db "$RTABMAP_DB"); MEM_ID_DESC="rtabmap db $RTABMAP_DB"
elif [ "$CAMERA" = sim ] && [ "$POSE" = reference ]; then
  MEM_ID_ARGS=(--sim-map-yaml "$REPO/robot_sim/fpga_sim/maps/room.yaml"
               --sim-world "$REPO/robot_sim/fpga_sim/worlds/${FPGA_SIM_WORLD_FILE:-room.sdf}")
  MEM_ID_DESC="sim map robot_sim/fpga_sim/maps/room.yaml (world ${FPGA_SIM_WORLD_FILE:-default})"
fi
MEM_IDENT=$(PYTHONPATH="$REPO" "$MEM_PY" -m host.memory_store identity "${MEM_ID_ARGS[@]}" 2>&1) || {
  if [ -n "$CAR_MEMORY_IN" ]; then
    echo "不合法：CAR_MEMORY_IN 给了，但本趟地图身份算不出（$MEM_ID_DESC）：$MEM_IDENT" >&2; exit 2; fi
  echo "warning: map identity of this run not computed ($MEM_ID_DESC): $MEM_IDENT; the memory file says none" >&2
  MEM_IDENT=$(PYTHONPATH="$REPO" "$MEM_PY" -m host.memory_store identity) || { echo "memory_store identity failed" >&2; exit 2; }
}
MEM_CHECK=""
if [ -n "$CAR_MEMORY_IN" ]; then
  if [ "$POSE" = map ]; then echo "不合法：CAR_MEMORY_IN 配 --pose map——建图趟不起运行器，没有谁载入记忆" >&2; exit 2; fi
  [ -f "$CAR_MEMORY_IN" ] || { echo "不合法：CAR_MEMORY_IN='$CAR_MEMORY_IN' 不是文件" >&2; exit 2; }
  MEM_CHECK=$(PYTHONPATH="$REPO" "$MEM_PY" -m host.memory_store check --memory "$CAR_MEMORY_IN" --identity /dev/stdin \
              <<< "$MEM_IDENT" 2>&1) || {
    echo "不合法：CAR_MEMORY_IN 被拒（不降级为空记忆）——$CAR_MEMORY_IN 对本趟地图（$MEM_ID_DESC）：$MEM_CHECK" >&2; exit 2; }
  MEM_IN_SHA=$(echo "$MEM_CHECK" | "$MEM_PY" -c 'import json,sys; print(json.load(sys.stdin)["file_sha256"])')
fi
INNER_ENV_MEM=(MEMORY_IDENTITY=1 MEMORY_SAVE="$CAR_MEMORY_SAVE" MEMORY_SNAPSHOT_S="$CAR_MEMORY_SNAPSHOT_S"
               MEMORY_POSE_MODE="$POSE")
[ -n "$CAR_MEMORY_IN" ] && INNER_ENV_MEM+=(MEMORY_IN=1)   # car_inner: $O/map_identity.json, $O/memory_in.json
# ground calibration vs the mapping run (10-15): rtabmap loc + live + calibration on -> the mount stored beside the
# pinned database (<db stem>.ground_calib.json, written by the mapping run), mounted read-only, compared in the stack
GC_MAP=""
if [ "$POSE" = rtabmap ] && [ "$PLATFORM" = board ] && [ "$CAR_GROUND_CALIB" != off ] && [ -n "$RTABMAP_DB" ]; then
  GC_MAP="${RTABMAP_DB%.db}.ground_calib.json"
fi
TAG=$(basename "$RUN")
[[ "$TAG" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "run directory basename: letters, digits, - and _ only" >&2; exit 2; }
GZ=wde-car-gz-$TAG; KV=wde-car-kv-$TAG; STACK=wde-car-stack-$TAG
IPC=$CAR_STATE_DIR/$TAG
# Unix socket paths: 108 bytes with the NUL, so 107; the longest is IPC/teleop.sock (12 more than IPC)
if [ $((${#IPC} + 12)) -gt 107 ]; then
  echo "IPC directory $IPC is ${#IPC} bytes: its sockets ($IPC/teleop.sock) would exceed the 107-byte Unix socket limit;" \
       "use a shorter CAR_STATE_DIR or run directory name" >&2; exit 2; fi
KV_ADDR=127.0.0.1:47811; [ "$PLATFORM" = board ] && KV_ADDR=$KV_SERVER
# the hub address (10-15): board CAR_HUB_BIND (checked above); host 127.0.0.1, or CAR_HUB_BIND (the stack namespace is
# --network none: only 127/8 can be bound there). Passed into the stack and recorded in car.json "hub" -- car_pc.sh
# connects to exactly this address (no 127.0.0.1 guess on the board)
: "${HUBPORT:=47431}"
HUB_BIND=${CAR_HUB_BIND:-127.0.0.1}; [ "$PLATFORM" = board ] && HUB_BIND=${CAR_HUB_BIND:-<CAR_HUB_BIND>}

INNER_ENV=(CAMERA="$CAMERA" POSE="$POSE" NAV="$NAV" DETECTOR="$DETECTOR" CHASSIS="$CHASSIS" YOLO_BACKEND="$YOLO_BACKEND"
           RECORD="$RECORD" INF_DEVICE="$INF_DEVICE" KV_ADDR="$KV_ADDR" KV_BUILD_ID="$KV_BUILD_ID" MCU_HEARTBEAT="$MCU_HEARTBEAT"
           STACK_CPUS="$STACK_CPUS" RUNTIME_CPUS="$RUNTIME_CPUS" CHASSIS_PORT="$CHASSIS_PORT" RTABMAP_DB="$RTABMAP_DB"
           CAR_FAILSTOP="${CAR_FAILSTOP:-0}" TELEOP="$TELEOP" WHEEL_ODOM="$CAR_WHEEL_ODOM" HUB_BIND="$HUB_BIND" HUBPORT="$HUBPORT")   # into the stack container: car_inner.sh reads it there
[ "$TELEOP" = 1 ] && INNER_ENV+=(TELEOP_V_MAX="$CAR_TELEOP_V_MAX" TELEOP_W_MAX="$CAR_TELEOP_W_MAX" TELEOP_MAX_CMD_S="$CAR_TELEOP_MAX_CMD_S")
[ "$CAR_WHEEL_ODOM" = 1 ] && INNER_ENV+=(WHEEL_DIAM_M="$CAR_WHEEL_DIAM_M" COUNTS_PER_REV="$CAR_COUNTS_PER_REV" TRACK_M="$CAR_TRACK_M")
# 10-27 owner: the real car's heading comes from the gyro by default (in-place 3 turns: gyro within 1 %, wheel +1.5..+5.7 %)
[ "$PLATFORM" = board ] && [ "$CHASSIS" = uart ] && : "${CAR_HEADING:=gyro}"
case "${CAR_HEADING:=wheel}" in wheel|gyro) ;; *) echo "CAR_HEADING 只能是 wheel 或 gyro（给的是 '$CAR_HEADING'）" >&2; exit 2;; esac
[ "$CAR_WHEEL_ODOM" = 1 ] && INNER_ENV+=(HEADING="$CAR_HEADING" WHEEL_ODOM_ARGS="${CAR_WHEEL_ODOM_ARGS:-}")
[ "$ROLLING_BACKEND" != cpu ] && INNER_ENV+=(ROLLING_BACKEND="$ROLLING_BACKEND")   # 10-26 FPGA profile v3fc
[ "$NAV_BACKEND" != nav2 ] && INNER_ENV+=(NAV_BACKEND="$NAV_BACKEND")   # CTRLPS2 11-03; nav2 = not passed = as before
[ "$PLATFORM" = board ] && INNER_ENV+=(NAV_TIMEOUT_S="$CAR_NAV_TIMEOUT_S" TASK_TIMEOUT_S="$CAR_TASK_TIMEOUT_S" \
  STREAM_TIMEOUT_S="$CAR_STREAM_TIMEOUT_S" MAX_NAV_FAILURES="$CAR_MAX_NAV_FAILURES" POSE_TRUST="$CAR_POSE_TRUST")
# 10-25 find-retry fault injection (host/ros_runtime.py --fault-nav-fail-first reads it): host simulation ONLY and only
# when set non-empty -- the board container never gets it, the default container line is unchanged
[ "$PLATFORM" = host ] && [ -n "${FPGA_FAULT_NAV_FAIL_FIRST:-}" ] && INNER_ENV+=(FPGA_FAULT_NAV_FAIL_FIRST="$FPGA_FAULT_NAV_FAIL_FIRST")
[ -n "$CAR_FRONT_SEED_M" ] && INNER_ENV+=(FRONT_SEED_M="$CAR_FRONT_SEED_M")   # 10-26, empty = off = not passed
# B1 nationals 11-02 (NANO_SIDE.md): the PL navigation grid; 0 / unset = not passed = the car exactly as before
[ "${CAR_PL_GRID:-0}" = 1 ] && INNER_ENV+=(CAR_PL_GRID=1)
[ "${CAR_PL_GRID_SHADOW:-0}" = 1 ] && INNER_ENV+=(CAR_PL_GRID_SHADOW=1)
INNER_ENV+=(SEARCH_MODE="$CAR_SEARCH_MODE")   # 10-26 probabilistic search (prob) or the MVP list (mvp)
# 10-26 e2e flow sim (tools/car_run/e2e_find_flow.py): CAR_SIM_NAV_PROFILE=car = the host simulation with the car's Nav2
# switches (FPGA_NAV_SPEED_LIMITS=car: speed limits, 0.10 m / yaw goal tolerance, rotation shim) and the board's runner
# budgets above, with the car's controller (DWB: dwb.yaml, the rotation shim refuses RPP); the simulation map and costmap
# layers stay the host's. Unset leaves the container line as before.
[ "$CAR_SIM_NAV_PROFILE" = car ] && INNER_ENV+=(SIM_NAV_PROFILE=car NAV_TIMEOUT_S="$CAR_NAV_TIMEOUT_S" TASK_TIMEOUT_S="$CAR_TASK_TIMEOUT_S")
# 10-26: the board Nav2 switches and DWB minimum speeds (header), only when given: unset = the container line of before
INNER_ENV+=("${NAV_SW_ENV[@]}" "${DWB_OVR_ENV[@]}")
[ "$PLATFORM" = board ] && INNER_ENV+=(STACK_CPUS= RUNTIME_CPUS=
                                       LIVE_START_TRIES="$LIVE_START_TRIES" LIVE_START_WAIT_S="$LIVE_START_WAIT_S"
                                       LIVE_START_GAP_S="$LIVE_START_GAP_S" LIVE_DEPTH_MSGS="$LIVE_DEPTH_MSGS" "${MOUNT_ENV[@]}"
                                       LIVE_LAUNCH_ARGS="$LIVE_LAUNCH_ARGS" LIVE_FRAME_DROP_LOG="${LIVE_FRAME_DROP_LOG:-true}"
                                       LIVE_CAMERA_FRAME="$LIVE_CAMERA_FRAME"
                                       LIVE_COLOR_TOPIC="$LIVE_COLOR_TOPIC" LIVE_DEPTH_TOPIC="$LIVE_DEPTH_TOPIC"
                                       LIVE_INFO_TOPIC="$LIVE_INFO_TOPIC" LIVE_COLOR_INFO_TOPIC="$LIVE_COLOR_INFO_TOPIC"
                                       CAMERA_CHECK_S="$CAMERA_CHECK_S" RUNNER_CAMERA_QOS="$RUNNER_CAMERA_QOS")
# T2 switches (10-16): only when not default, so the default container line is unchanged
[ "$PRODUCER_DEPTH_QOS" != auto ] && INNER_ENV+=(PRODUCER_DEPTH_QOS="$PRODUCER_DEPTH_QOS")
[ "$PLATFORM" = board ] && [ -n "$LIVE_DRIVER_DDS_PROFILE" ] && INNER_ENV+=(LIVE_DRIVER_DDS_PROFILE="$LIVE_DRIVER_DDS_PROFILE")
# half-rate check (10-17): only when on, so that CAR_CAMERA_RATE_CHECK=0 leaves the container line as before
[ "$PLATFORM" = board ] && [ "$CAR_CAMERA_RATE_CHECK" = 1 ] && INNER_ENV+=(CAMERA_RATE_CHECK=1 CAMERA_RATE_S="$CAR_CAMERA_RATE_S")
[ -n "$GC_MAP" ] && [ -f "$GC_MAP" ] && INNER_ENV+=(GROUND_CALIB_MAP="$GC_MAP")
[ "$POSE" = static ] && INNER_ENV+=(BENCH_ODOM_HZ="$BENCH_ODOM_HZ" BENCH_MAP_M="$BENCH_MAP_M")
[ "$NAV_DRYRUN" = 1 ] && INNER_ENV+=(NAV_DRYRUN=1)
[ -n "${CAR_PRODUCER_HZ:-}" ] && INNER_ENV+=(PRODUCER_HZ="$CAR_PRODUCER_HZ")   # 10-27 plan A override (default: car_inner, live 30)
# in-stack mapping (10-18): car_inner writes the blank start map and runs tools/car_run/mapping_ctl.py
[ "$MAPPING" = 1 ] && INNER_ENV+=(MAPPING=1 MAPPING_BLANK_M="${CAR_MAPPING_BLANK_M:-20}" LIVE_ALIGN="$CAR_LIVE_ALIGN" NAV_LIVE_GLOBAL="$CAR_NAV_LIVE_GLOBAL" NAV_GLOBAL_ROLLING="$CAR_NAV_GLOBAL_ROLLING")   # 10-17: car_inner's car_validate needs it too (absent = 0)
# 10-26: the host stand-in export's wall (tools/car_run/mapping_standin.py WALL_X_M, default 1.0); host sim only
[ "$MAPPING" = 1 ] && [ "$PLATFORM" != board ] && [ -n "${WALL_X_M:-}" ] && INNER_ENV+=(WALL_X_M="$WALL_X_M")
INNER_ENV+=(PREVIEW="$PREVIEW")
[ -n "${YOLO_PROFILE:-}" ] && INNER_ENV+=(YOLO_PROFILE="$YOLO_PROFILE")
# 10-25: the preview rate reaches preview_publisher.py only through the container env (it read CAR_PREVIEW_FPS but
# car_up never passed it, so only the 10 fps default ever ran); unset leaves the container line as before
[ -n "${CAR_PREVIEW_FPS:-}" ] && INNER_ENV+=(CAR_PREVIEW_FPS="$CAR_PREVIEW_FPS")
[ "$AR_COLOUR" = 1 ] && INNER_ENV+=(AR_COLOUR=1 AR_COLOUR_PORT="$CAR_AR_COLOUR_PORT")   # 10-26; 0 = line unchanged
[ "$AR_NANO" = 1 ] && INNER_ENV+=(AR_NANO=1)   # AR2NANO2; 0 = line unchanged
# 10-27 geometry v2 (docs/design/GEOMETRY_V2_20261027.md): CAR_GEOMETRY_V2=1 turns it on in the producer AND the
# runtime together (car_inner); anything else = off, the lines unchanged. Not a gate: never refuses a start.
[ "${CAR_GEOMETRY_V2:-0}" = 1 ] && INNER_ENV+=(GEOMETRY_V2=1)
# 10-27 candidate merge (docs/design/MEMORY_EXISTENCE_20261027.md): both default on in the code; =0 turns one off in
# the runtime (CAR_EXISTENCE_V3=0 = the 0f377be memory rules). Unset = the lines unchanged. Not a gate.
[ "${CAR_EXISTENCE_V3:-1}" = 0 ] && INNER_ENV+=(EXISTENCE_V3=0)
[ "${CAR_SEE_THROUGH_FILTER:-1}" = 0 ] && INNER_ENV+=(SEE_THROUGH_FILTER=0)
# 10-31 one object per class (Config.one_per_class, on in the code): CAR_ONE_PER_CLASS=0 lets a second same-class
# object into memory (tests with several cups). Unset = on. Not a gate.
[ "${CAR_ONE_PER_CLASS:-1}" = 0 ] && INNER_ENV+=(ONE_PER_CLASS=0)
# 10-27 night fast removal (docs/design/FAST_REMOVAL_20261027.md): on in the code; CAR_FAST_REMOVAL=0 = 036f82a rules,
# CAR_FAST_REMOVAL_VOTE=-1.7 / -2.5 = the faster tiers. Unset = the lines unchanged. Not a gate.
[ "${CAR_FAST_REMOVAL:-1}" = 0 ] && INNER_ENV+=(FAST_REMOVAL=0)
[ -n "${CAR_FAST_REMOVAL_VOTE:-}" ] && INNER_ENV+=(FAST_REMOVAL_VOTE="$CAR_FAST_REMOVAL_VOTE")
[ "$POSE" = odom ] && INNER_ENV+=(BENCH_MAP_M="$BENCH_MAP_M")
# pose odom + costmap: the export bench_pose publishes as /map; with nav2 the board nav2 line below passes the same map
# (for map_server) -- one MAP_YAML in the container either way
[ -n "$ODOM_MAP_YAML" ] && [ "$NAV" != nav2 ] && INNER_ENV+=(MAP_YAML="$ODOM_MAP_YAML")
[ "$RTAB" = 1 ] && INNER_ENV+=(RTABMAP_ODOM="$RTABMAP_ODOM" RTABMAP_DB_DIR="$RTABMAP_DB_DIR_IN" RTABMAP_STOP_S="$CAR_RTABMAP_STOP_S"
                               RTABMAP_GATE_MODE="$CAR_GATE_MODE" RTABMAP_GATE_X_S="$CAR_GATE_X_S")
[ "$RTAB" = 1 ] && INNER_ENV+=(RTABMAP_GATE_ODOM_JUMP="$CAR_GATE_ODOM_JUMP" RTABMAP_GATE_ODOM_V_MPS="$CAR_GATE_ODOM_V_MPS"
                               RTABMAP_GATE_ODOM_E_M="$CAR_GATE_ODOM_E_M")
[ "$NAV" = nav2 ] && [ "$PLATFORM" = board ] && [ "$MAPPING" != 1 ] && INNER_ENV+=(MAP_YAML="${CAR_MAP_YAML:-$MAP_YAML_DESC}")   # MAPPING: car_inner's blank map
# FPGA_NAV_COSTMAP: the effective board value (car_nav_costmap: empty -> rolling), always written out on the board
# nav2 so the container env shows it; nothing on the host (the host launch keeps its default sim)
[ -n "$CAR_NAV_COSTMAP" ] && INNER_ENV+=(NAV_COSTMAP="$CAR_NAV_COSTMAP")
[ "$CAR_NAV_COSTMAP" = rolling ] && INNER_ENV+=(GRID_READY_S="${GRID_READY_S:-60}")   # grid_probe.py wait before Nav2 activation
[ "$POSE" != map ] && INNER_ENV+=("${INNER_ENV_MEM[@]}")   # 10-18 object memory (the runner's files in /o = RUN)

# inference service command (host: ssdlite | yolo torch | yolo ort; board: yolo trt)
case "${CAR_YOLO_ROUTE:-}" in
  ""|all33|primary) ;;
  *) echo "CAR_YOLO_ROUTE 只能是 all33|primary（给的是 '$CAR_YOLO_ROUTE'；空=all33）" >&2; exit 2;;
esac
INF_CMD=()
# POSE map (mapping run): no runner, so no inference service and no GPU lock either
[ "$POSE" = map ] && NEED_GPU_LOCK=0
if [ "$DETECTOR" != stub ] && [ "$POSE" != map ]; then
  case "$DETECTOR/$YOLO_BACKEND" in
    yolo/torch) INF_ARGS=(--detector yolo --pt "$YOLO_PT");;
    yolo/ort) INF_ARGS=(--detector yolo --onnx "$YOLO_ONNX" --threads "$YOLO_THREADS");;
    yolo/trt) INF_ARGS=(--detector yolo --engine "${YOLO_ENGINE:-<YOLO_ENGINE>}");;
    # 10-22: CAR_CLASS_MIN_SCORE='backpack=0.5,...' overrides the per-class score floors ('' = none)
    *) INF_ARGS=(--detector ssdlite --models "$MODELS" --augmentation "$AUG");;
  esac
  [ "$DETECTOR" = yolo ] && [ -n "${CAR_CLASS_MIN_SCORE+x}" ] && INF_ARGS+=(--class-min-score "$CAR_CLASS_MIN_SCORE")
  [ "$DETECTOR" = yolo ] && [ -n "${YOLO_PROFILE:-}" ] && INF_ARGS+=(--yolo-profile "$YOLO_PROFILE")
  # 10-30 (owner): CAR_YOLO_ROUTE=all33 (default: every enabled class the 33-class model knows -> it; COCO only for
  # the rest, not loaded when it gets none) | primary (rollback: the old PRIMARY33 split); combined33 only
  [ "$DETECTOR" = yolo ] && [ "${YOLO_PROFILE:-}" = combined33 ] && [ -n "${CAR_YOLO_ROUTE:-}" ] && INF_ARGS+=(--yolo-route "$CAR_YOLO_ROUTE")
  [ "$DETECTOR" = yolo ] && { [ "${YOLO_PROFILE:-}" = combined ] || [ "${YOLO_PROFILE:-}" = combined33 ]; } && INF_ARGS+=(--coco-model "${YOLO_COCO_MODEL:-/data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine}")
  if [ "$PLATFORM" = host ]; then
    INF_PATH=$RUN/src; [ "$YOLO_BACKEND/$DETECTOR" = torch/yolo ] && INF_PATH=$RUN/src:$YOLO_UL_PATH
    INF_CMD=(env CUDA_VISIBLE_DEVICES="$([ "$INF_DEVICE" = cuda ] && echo "$INF_GPU")" TORCH_HOME="$MODELS" PYTHONPATH="$INF_PATH"
             PYTHONDONTWRITEBYTECODE=1 YOLO_CONFIG_DIR="$IPC/ultralytics" YOLO_OFFLINE=1
             taskset -c "$INF_CPUS" "$CAR_PY_HOST" -m host.inference_service --socket "$IPC/inf.sock" --device "$INF_DEVICE"
             "${INF_ARGS[@]}" --warmup-frames "$INF_WARMUP" --warmup-size "$INF_WARMUP_SIZE")
  else
    # the snapshot first (its tools/ namespace package holds tools/nano_detector/nano_detector.py, imported by
    # perception/fpga_replay/yolo_detector.py's trt backend as tools/nano_loop/nano_loop.py:177 does), then the
    # board's packages (NANO_PYLIB); torch / tensorrt come from JetPack and robot's ~/.local
    INF_CMD=(env PYTHONPATH="$RUN/src:$NANO_PYLIB" "$CAR_PY_BOARD" -m host.inference_service --socket "$IPC/inf.sock" --device cuda
             "${INF_ARGS[@]}" --warmup-frames "$INF_WARMUP" --warmup-size 640x480)
  fi
fi

DDS=(-e ROS_DOMAIN_ID=93 -e ROS_LOCALHOST_ONLY=1 -e FASTRTPS_DEFAULT_PROFILES_FILE=/src/robot_sim/fpga_sim/config/fastdds_server.xml
     -e IGN_PARTITION="$TAG" -e GZ_PARTITION="$TAG" -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 -e HOME=/tmp)
MNT=(-v "$RUN/src:/src:ro")
for name in launch config worlds maps; do MNT+=(-v "$RUN/src/robot_sim/fpga_sim/$name:/opt/fpga_ws/install/share/fpga_sim/$name:ro"); done
MNT+=(-v "$RUN/src/robot_sim/fpga_sim/scripts/velocity_guard.py:/opt/fpga_ws/install/lib/fpga_sim/velocity_guard.py:ro")
U="$(id -u):$(id -g)"
GZ_CMD=(); KV_CMD=(); STACK_CMD=()
if [ "$PLATFORM" = host ]; then
  if [ "$CAMERA" = sim ]; then
    if [ "$CHASSIS" = pty ]; then GZ_ENV=(-e FPGA_SIM_CMD_TOPIC=/fpga/chassis/sim_twist -e FPGA_SIM_ESTOP_TOPIC=)
    else GZ_ENV=(-e FPGA_SIM_ESTOP_TOPIC=/fpga/sim_estop); fi
    # teleop + nav2 = arbitration (10-18): the Gazebo container's guard publishes /fpga/nav/cmd_vel_safe instead
    [ "$TELEOP" = 1 ] && [ "$NAV" = nav2 ] && GZ_ENV+=(-e FPGA_SIM_GUARD_OUTPUT=/fpga/nav/cmd_vel_safe)
    [ -n "$FPGA_SIM_WORLD_FILE" ] && GZ_ENV+=(-e FPGA_SIM_WORLD="/opt/fpga_ws/install/share/fpga_sim/worlds/$FPGA_SIM_WORLD_FILE")
    GZ_CMD=(run -d --init --name "$GZ" --network none --ipc shareable --shm-size 1g --cpuset-cpus "$GZ_CPUS" --user "$U"
            "${DDS[@]}" -e LIBGL_ALWAYS_SOFTWARE=1 -e FPGA_SIM_CLOUD=adapter "${GZ_ENV[@]}" "${MNT[@]}" --entrypoint bash "$IMAGE" -c
            'source /opt/ros/humble/setup.bash; source /opt/fpga_ws/install/setup.bash;
             exec xvfb-run -a -s "-screen 0 1280x720x24" ros2 launch fpga_sim simulation.launch.py')
    KV_NS=(--network "container:$GZ"); OWNER=$GZ
  else
    KV_NS=(--network none --ipc shareable --shm-size 1g); OWNER=$KV
  fi
  PERMIT_MODEL=off; [ "$CHASSIS" = pty ] && PERMIT_MODEL=realtime
  KV_CMD=(run -d --init --name "$KV" "${KV_NS[@]}" --cpuset-cpus "$KV_CPUS" --user 0
          --cap-drop ALL --cap-add NET_ADMIN --cap-add SETUID --cap-add SETGID --cap-add SETPCAP --cap-add KILL
          -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e PYTHONUNBUFFERED=1 -v "$RUN/src:/src:ro" -v "$RUN:/o"
          --entrypoint setpriv "$IMAGE" --reuid="$(id -u)" --regid="$(id -g)" --clear-groups
          --inh-caps=-all,+net_admin --ambient-caps=+net_admin --bounding-set=-all,+net_admin
          bash -c 'cd /src && exec python3 tools/kv_link/kv_server.py --core cpu --cpu-threads "$0" --roi-core cpu \
                   --permit-model "$1" --port 47811 --out /o/kv > /o/kv.json 2> /o/kv.err' "$KV_THREADS" "$PERMIT_MODEL")
  # CTRLPS2 11-03: the fake KV serves "nav" only when CAR_NAV_BACKEND is kv|shadow (nav2: the line above as it is)
  if [ "$NAV_BACKEND" != nav2 ]; then
    case "${CAR_KV_NAV_CORE:=mock}" in mock|kv_nav) ;; *) echo "CAR_KV_NAV_CORE 只能是 mock|kv_nav" >&2; exit 2;; esac
    KV_CMD[${#KV_CMD[@]}-3]=${KV_CMD[${#KV_CMD[@]}-3]/--port 47811/--nav soft --nav-core $CAR_KV_NAV_CORE --port 47811}
  fi
  SENV=(); for e in "${INNER_ENV[@]}"; do SENV+=(-e "$e"); done
  STACK_CMD=(run -d --init --name "$STACK" --network "container:$OWNER" --ipc "container:$OWNER"
             --cpuset-cpus "$STACK_CPUS${RUNTIME_CPUS:+,$RUNTIME_CPUS}" --user "$U" "${DDS[@]}" "${SENV[@]}"
             -e HEAD="@HEAD@" -e O=/o -e SRCD=/src -e IPC=/ipc "${MNT[@]}" -v "$RUN:/o" -v "$IPC:/ipc")
  [ "$CAMERA" = replay ] && STACK_CMD+=(-v "$PREP_HOST:/prep:ro")
  STACK_CMD+=(--entrypoint bash "$IMAGE" /src/tools/car_run/car_inner.sh)
else   # board (draft): one container, host network (the KV260 link), the camera's USB node and the chassis port
  # the camera (10-17, docs/design/CAM_USB_REENUM_20261017.md): the whole USB root bind-mounted + the usbfs major 189
  # allowed by a device cgroup rule, NOT --device of one node: the Gemini 2 can drop off USB and come back with the
  # next device number in the first second of a driver start (board 14:36 002/003->004, 16:26 002/004->005), and a
  # --device node (and its one-minor cgroup entry) is fixed at docker run -- every retry inside the container then
  # opened a dead node. The bind mount shows the new node; 189:* is usbfs only (the chassis tty, GPU, shm are other
  # majors); rw, no m. Non-root --user: only the camera's node is writable (udev 0666), the others 0664 read-only.
  # The node from lsusb (car_orbbec_busdev) stays the presence check: the real start refuses (3) when there is none.
  BUSDEV=$(car_orbbec_busdev) || BUSDEV=""
  CAMUSB=(-v "${CAR_USB_ROOT:-/dev/bus/usb}:/dev/bus/usb" --device-cgroup-rule 'c 189:* rw')
  SENV=(); for e in "${INNER_ENV[@]}"; do SENV+=(-e "$e"); done
  # the chassis port only with --chassis uart: on the bench (none) /dev/fpga_chassis does not exist and docker run
  # would refuse the whole container for it
  # --user is uid:gid only, so the device's group (dialout, 0660) must be added or the driver gets EACCES (board 10-16)
  CHDEV=(); [ "$CHASSIS" = uart ] && CHDEV=(--device "$CHASSIS_PORT" --group-add "$(stat -L -c %g "$CHASSIS_PORT" 2>/dev/null || echo 20)")
  STACK_CMD=(run -d --init --name "$STACK" --network host --user "$U" -e ROS_DOMAIN_ID=48 -e ROS_LOCALHOST_ONLY=1
             -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -e ROS_LOG_DIR=/tmp/roslog "${SENV[@]}" -e HEAD="@HEAD@"
             -e O=/o -e SRCD=/src -e IPC=/ipc "${CAMUSB[@]}" "${CHDEV[@]}"
             -v "$RUN/src:/src:ro" -v "$RUN:/o" -v "$IPC:/ipc")
  [ -n "$RTABMAP_DB" ] && STACK_CMD+=(-v "$RTABMAP_DB:$RTABMAP_DB:ro")
  [ -n "$GC_MAP" ] && [ -f "$GC_MAP" ] && STACK_CMD+=(-v "$GC_MAP:$GC_MAP:ro")
  [ "$RTAB" = 1 ] && STACK_CMD+=(-v "$RTABMAP_DB_DIR:$RTABMAP_DB_DIR")
  [ -n "$CAR_MAP_YAML" ] && STACK_CMD+=(-v "$(dirname "$CAR_MAP_YAML"):$(dirname "$CAR_MAP_YAML"):ro")
  # 10-29: raising a priority (nice < 0) needs CAP_SYS_NICE; only car_priority.py (root via docker exec) uses it
  [ "$CAR_PRIORITY" = 1 ] && STACK_CMD+=(--cap-add SYS_NICE)
  # AR2NANO2: the DRM nodes for m7ar_present --display drm (+ their group, video); only with CAR_AR_NANO=1
  if [ "$AR_NANO" = 1 ]; then
    DRI_GID=$(stat -L -c %g /dev/dri/card* 2>/dev/null | head -1); DRI_GID=${DRI_GID:-44}
    STACK_CMD+=(--device /dev/dri --group-add "$DRI_GID")
  fi
  STACK_CMD+=(--entrypoint bash "$BOARD_IMAGE" /src/tools/car_run/car_inner.sh)
fi

if [ "$DRY" = 1 ]; then
  echo "# car_up dry run: camera=$CAMERA pose=$POSE nav=$NAV detector=$DETECTOR chassis=$CHASSIS yolo_backend=$YOLO_BACKEND"
  echo "#   source marker=$SOURCE platform=$PLATFORM inference_device=$INF_DEVICE gpu_lock=$NEED_GPU_LOCK record=$RECORD"
  echo "# already-running check: $POINTER absent and no container named wde-car-*"
  echo "# run directory: $RUN (pids.txt, host_pids.txt, containers.txt, logs); IPC $IPC; pointer $POINTER -> run directory"
  if git -C "$REPO" rev-parse --is-inside-work-tree > /dev/null 2>&1; then echo "# source snapshot: git ls-files of $REPO -> $RUN/src"
  else echo "# source snapshot: no git work tree -- the deployed tree of $REPO/COMMIT ($(head -1 "$REPO/COMMIT" 2>/dev/null || echo 'missing: real start exits 8')) -> $RUN/src"; fi
  if [ "$PLATFORM" = board ]; then
    echo "# camera device: ${BUSDEV:-dry-run 未解析 (${CAR_LSUSB:-lsusb} -d 2bc5: 无输出; 真启动会以退出码 3 拒绝)}${BUSDEV:+ (from ${CAR_LSUSB:-lsusb} -d 2bc5:)}"
    echo "# camera usb: the whole ${CAR_USB_ROOT:-/dev/bus/usb} -> /dev/bus/usb + device cgroup rule 'c 189:* rw' (usbfs only), not --device of the node above (presence check only): a re-enumeration in the driver start (new device number) stays visible to the retry"
    echo "[kv precheck] $CAR_KV_CHECK --server $KV_SERVER --trials 1 --hello-timeout-s 1.0 --expect-build-id ${KV_BUILD_ID:-<KV_BUILD_ID>} --out /dev/stdout  (up to 3 tries; no answer -> exit 9; another BUILD_ID -> warn, use it; unregistered -> Nano CPU fusion)"
    [ "$POSE" = rtabmap ] && echo "# rtabmap precheck: RTABMAP_DB='${RTABMAP_DB}' must be a file (pinned localization database), else exit 2 before anything"
    [ "$NAV" = nav2 ] && echo "# nav2 map precheck: $MAP_YAML_DESC must be a file, else exit 2 before anything"
    [ "$POSE" = static ] && echo "# pose static: bench only (camera fixed, no chassis) -- map->odom->base_link identity, stand-in free /map; NOT mapping/localization"
    [ "$NAV_DRYRUN" = 1 ] && echo "# NAV DRY RUN (CAR_NAV_DRYRUN=1): board Nav2 on the static bench pose, chassis none -- the car cannot move; no stand-in /map (map_server: $MAP_YAML_DESC)"
    if [ "$POSE" = odom ] && [ "$NAV" = nav2 ]; then
      echo "# pose odom + nav2: map->odom identity (start = origin; bench_pose --no-odom-base --map-m 0) + odom->base_link and /odom from the wheel odometry; /map from Nav2's map_server = $MAP_YAML_DESC (must be a file, else exit 2 before anything); the car must start at the mapping start, same heading; NOT mapping/localization"
    elif [ "$POSE" = odom ] && [ -n "$ODOM_MAP_YAML" ]; then
      echo "# pose odom: map->odom identity (start = origin) + odom->base_link from the wheel odometry, /map = the mapping export $ODOM_MAP_YAML (must be a file, else exit 2 before anything; right only when the car starts at the mapping start, same heading); NOT mapping/localization"
    elif [ "$POSE" = odom ]; then
      echo "# pose odom: map->odom identity (start = origin) + odom->base_link from the wheel odometry, stand-in free /map; NOT mapping/localization"
    fi
    echo "# car body: CAR_ROBOT_HEIGHT=$CAR_ROBOT_HEIGHT m -> obstacle band top $(python3 -c 'import sys; print("%.3f" % (float(sys.argv[1]) + 0.05))' "$CAR_ROBOT_HEIGHT") m; the camera in use must be >= that + 0.005 m (checked in the stack after the ground calib)"
    [ "$CAR_CAMERA_RATE_CHECK" = 1 ] && echo "# camera rate check: CAR_CAMERA_RATE_CHECK=1 -- every driver start: the depth probe listens $CAR_CAMERA_RATE_S s more; $([ "$CAR_CAMERA_FPS" = 30 ] && echo 'depth median < 32.8 ms or colour p25 > 50 ms' || echo "at $CAR_CAMERA_FPS fps colour p25 > $((1500 / CAR_CAMERA_FPS)) ms (no depth rule)") = half-rate = a failed start (retried); result $RUN/camera_rate_check.json (CAR_CAMERA_RATE_CHECK=0: the plan of before)"
    echo "# camera rate: CAR_CAMERA_FPS=$CAR_CAMERA_FPS (colour and depth; 30 = before)"
    [ "$AR_NANO" = 1 ] && echo "# car screen on this Nano (CAR_AR_NANO=1): producer --ar-nano (m7ar_present --provider cpu --display drm, 1024x600), container --device /dev/dri --group-add $DRI_GID, colour sender -> 127.0.0.1:$CAR_AR_COLOUR_PORT; logs $RUN/ar_nano/"
    echo "# ground calib: CAR_GROUND_CALIB=$CAR_GROUND_CALIB (configured mount ${CAR_CAM_HEIGHT} m / ${CAR_CAM_PITCH_DEG} deg / roll 0; height_cell ${CAR_HEIGHT_CELL:-derived})$([ "$CAR_GROUND_CALIB" = off ] && echo ' -- off: the configured mount, no fit' || echo " -- result $RUN/ground_calib.json, filled into car.json after READY")"
    [ "$CAR_WHEEL_ODOM" = 1 ] && echo "# wheel params: wheel_diam_m=$CAR_WHEEL_DIAM_M counts_per_rev=$CAR_COUNTS_PER_REV track_m=$CAR_TRACK_M -- source $WHEEL_SRC"
    [ -n "$GC_MAP" ] && echo "# ground calib vs map: $GC_MAP $([ -f "$GC_MAP" ] && echo '(mounted read-only, difference printed in marks.txt)' || echo '(absent: not compared)')"
  fi
  [ "$TELEOP" = 1 ] && echo "# TELEOP: remote control run (NOT a navigation run; not gated by the motion permit; MCU heartbeat latch = backstop); keys: python3 $HERE/teleop_keys.py --socket $IPC/teleop.sock"
  [ "$NEED_GPU_LOCK" = 1 ] && echo "[gpu_lock] $CAR_LOCK acquire gpu$INF_GPU \"car_up $TAG\" -u $CAR_LOCK_USER  (token -> $RUN/gpu_lock.json)"
  [ ${#INF_CMD[@]} -gt 0 ] && echo "[inference] setsid ${INF_CMD[*]} > $RUN/inference.log  (ready: \"ready\": true + socket, <= $INF_READY_S s)"
  [ ${#GZ_CMD[@]} -gt 0 ] && echo "[container gz] $CAR_DOCKER ${GZ_CMD[*]}  (ready: running after 3 s)"
  [ ${#KV_CMD[@]} -gt 0 ] && echo "[container kv] $CAR_DOCKER ${KV_CMD[*]}"
  echo "[container stack] $CAR_DOCKER ${STACK_CMD[*]}  (ready: $RUN/READY <= $CAR_READY_S s)"
  [ "$PLATFORM" = board ] && [ "$CAR_PRIORITY" = 1 ] && echo "[priority] $CAR_DOCKER exec -u 0 $STACK python3 /src/tools/car_run/car_priority.py --interval-s 3 > $RUN/priority.log  (after the stack container starts; runner/wheel odom/chassis/camera -5, hub +5, preview/colour/3D map display +10; CAR_PRIORITY=0: off)"
  [ "$RTAB" = 1 ] && echo "# rtabmap: odom=$RTABMAP_ODOM wheel_odom=$CAR_WHEEL_ODOM db_dir=$RTABMAP_DB_DIR (stack: $RTABMAP_DB_DIR_IN; mkdir -p + writable, else exit 2); stop wait: rtabmap $CAR_RTABMAP_STOP_S s, stack $((CAR_RTABMAP_STOP_S + 90)) s -> $RUN/stack_stop_s.txt"
  echo "# memory: save=$CAR_MEMORY_SAVE (RUN/memory_final.json at stop, RUN/memory_snapshot.json every >= ${CAR_MEMORY_SNAPSHOT_S} s on change); map identity: $MEM_ID_DESC -> $(echo "$MEM_IDENT" | "$MEM_PY" -c 'import json,sys; d=json.load(sys.stdin); print(d["kind"], (d.get("db_sha256") or d.get("map_yaml_sha256") or "")[:12])')"
  if [ -n "$CAR_MEMORY_IN" ]; then echo "# memory in: $CAR_MEMORY_IN ACCEPTED -> RUN/memory_in.json; $(echo "$MEM_CHECK" | tr -d '\n' | tr -s ' ')"
  else echo "# memory in: none (CAR_MEMORY_IN not given: the runner starts with an empty memory)"; fi
  echo "# inner plan (car_inner.sh, DRY_RUN=1):"
  env "${INNER_ENV[@]}" DRY_RUN=1 O="$RUN" SRCD="$REPO" IPC="$IPC" HEAD="$(git -C "$REPO" rev-parse HEAD 2>/dev/null)" \
    PREP=/prep bash "$HERE/car_inner.sh" | sed 's/^/    /'
  exit "${PIPESTATUS[0]}"
fi

# --------------------------------------------------------------------------------------------- real start
if [ -L "$POINTER" ] || [ -e "$POINTER" ]; then
  echo "a car is already running: $POINTER -> $(readlink "$POINTER") (stop it with car_down.sh first)" >&2; exit 10; fi
LEFT=$(DK ps -a --format '{{.Names}}' 2>/dev/null | grep '^wde-car-')
[ -z "$LEFT" ] || { echo "containers of a car still exist: $(echo $LEFT) (car_down.sh first)" >&2; exit 10; }
# board prechecks, before anything is created: the KV260 first (it needs nothing from this board), then the camera
KV_CHECK_LOG=""
KV_UNREG_WARN=""
kv_accept_registered() {   # 10-26 (owner: success rate): a KV260 serving ANY registered build (fpga_profiles.env) is
                           # used as it is and its BUILD_ID recorded; if that build has no rolling core (a2) the producer
                           # falls back to the Nano CPU fusion (a cpu request stays cpu). 10-26 2nd pass: an UNREGISTERED
                           # build no longer exits 9 -- a warning (stderr, kv_precheck.txt, up_marks.txt), its id is used
                           # and the producer fuses on the Nano CPU, as for a2 (no rolling core it knows of)
  local actual prof roll
  case "$1" in build_id_mismatch:*) ;; *) return 1;; esac
  actual=${1#build_id_mismatch:}; actual=${actual%%!=*}
  if prof=$(fpga_profile_of_build "$actual"); then
    roll=$(FPGA_PROFILE=$prof; fpga_profile >/dev/null && echo "$FPGA_NANO_ROLLING") || roll=cpu
  else
    prof=unregistered; roll=cpu
    KV_UNREG_WARN="KV260 serves BUILD_ID $actual, registered in no profile of $FPGA_PROFILES_ENV (expected $KV_BUILD_ID, profile ${FPGA_PROFILE:-a2}): using it with the producer on the Nano CPU fusion"
    KV_CHECK_LOG+="WARN unregistered: $KV_UNREG_WARN"$'\n'
    echo "WARNING board: $KV_UNREG_WARN. To use the profile: on the KV260 (root) load ${FPGA_APP:-m7_pl_merge_a2} (xmutil loadapp) and restart the server" \
         "(sudo /usr/local/sbin/geomem-kv-start ${FPGA_PROFILE:-a2}; or register the id in fpga_profiles.env; docs/team/FPGA_PROFILE_RUNBOOK_20261026.md)" >&2
  fi
  [ "$roll" = cpu ] && ROLLING_BACKEND=cpu
  # SW_M1: the Nano car screen was planned from the requested profile; the served build has an AR core -> say so (display only)
  if [ "$AR_NANO" = 1 ] && [ "$prof" != unregistered ] && \
     [ "$(FPGA_PROFILE=$prof; fpga_profile >/dev/null 2>&1 && echo "$FPGA_AR")" != nano ]; then
    KV_CHECK_LOG+="WARN car screen: drawn on this Nano (CAR_AR_NANO=1) but the KV260 serves profile $prof (it has an AR core)"$'\n'
    echo "WARNING board: the car screen is drawn on this Nano (CAR_AR_NANO=1) but the KV260 serves profile $prof, which has its own AR core (display only; the stack starts as planned)" >&2
  fi
  if [ "$prof" != unregistered ]; then
    KV_CHECK_LOG+="accepted: KV260 serves registered BUILD_ID $actual (profile $prof), expected $KV_BUILD_ID (profile $FPGA_PROFILE); producer rolling backend $ROLLING_BACKEND"$'\n'
    echo "board: KV260 serves BUILD_ID $actual (registered profile $prof, requested $FPGA_PROFILE): using it; producer rolling backend $ROLLING_BACKEND" >&2
  fi
  # the stack's environment was fixed before this check: swap the two values in place
  STACK_CMD=("${STACK_CMD[@]/#KV_BUILD_ID=$KV_BUILD_ID/KV_BUILD_ID=$actual}")
  STACK_CMD=("${STACK_CMD[@]/#ROLLING_BACKEND=*/ROLLING_BACKEND=$ROLLING_BACKEND}")
  KV_BUILD_ID=$actual
  return 0
}
kv_precheck() {   # the KV260 serves on KV_SERVER with KV_BUILD_ID loaded: the producer's own connect (KvGeometry.connect:
                  # HELLO, roi_ext, roi core, BUILD_ID via TELEM) through tools/kv_link/hello_connect_loop.py, up to 3 tries
  local i out reason
  [ -n "$KV_BUILD_ID" ] || { echo "board: KV_BUILD_ID is empty (the bitstream the KV260 must have loaded)" >&2; return 1; }
  for i in 1 2 3; do
    out=$($CAR_KV_CHECK --server "$KV_SERVER" --trials 1 --hello-timeout-s 1.0 --expect-build-id "$KV_BUILD_ID" --out /dev/stdout 2>&1)
    KV_CHECK_LOG+="try $i: $out"$'\n'
    echo "$out" | grep -q '"ok": true' && return 0
    [ "$i" = 3 ] || sleep 1
  done
  reason=$(echo "$out" | python3 -c 'import json, sys
for line in sys.stdin:
    try:
        r = json.loads(line)
    except ValueError:
        continue
    if isinstance(r, dict) and "trial" in r:
        print(r.get("reason")); break' 2>/dev/null)
  kv_accept_registered "$reason" && return 0
  echo "board: the KV260 at $KV_SERVER is not usable (${reason:-no answer from the check: $(echo "$out" | tail -1)});" \
       "expected BUILD_ID $KV_BUILD_ID (FPGA profile ${FPGA_PROFILE:-a2}). First, on the KV260 (root): load ${FPGA_APP:-m7_pl_merge_a2} (xmutil loadapp) and start the server," \
       "bash tools/kv_link/kv_board.sh start <repo> <out> --core pl --device devmem --wait irq --roi-core pl --port ${KV_SERVER##*:}$([ "${FPGA_KV_ROLLING:-none}" = none ] || echo " --rolling $FPGA_KV_ROLLING")" \
       "(installed helper: sudo /usr/local/sbin/geomem-kv-start ${FPGA_PROFILE:-a2}; fall back: FPGA_PROFILE=a2; docs/team/FPGA_PROFILE_RUNBOOK_20261026.md)" >&2
  return 1
}
if [ "$PLATFORM" = board ]; then
  # rtabmap localizes on a pinned, pre-built database; without one the stack would only fail after the camera start
  if [ "$POSE" = rtabmap ] && [ ! -f "$RTABMAP_DB" ]; then
    echo "board: --pose rtabmap needs RTABMAP_DB = the pinned localization database (given: '${RTABMAP_DB}', not a file);" \
         "on a bench without a map use --pose static --chassis none --nav costmap" >&2; exit 2; fi
  if [ "$NAV" = nav2 ] && [ ! -f "$CAR_MAP_YAML" ] && [ "$MAPPING" != 1 ]; then
    echo "board: --nav nav2 needs the map of the pinned database: CAR_MAP_YAML or exactly one $MAP_YAML_DESC" >&2; exit 2; fi
  if [ -n "$ODOM_MAP_YAML" ] && [ ! -f "$ODOM_MAP_YAML" ]; then
    echo "board: --pose odom map export '$ODOM_MAP_YAML' is not a file (CAR_MAP_YAML / RTABMAP_DB given); unset both for the stand-in" >&2; exit 2; fi
  if [ "$RTAB" = 1 ]; then
    mkdir -p "$RTABMAP_DB_DIR" && [ -w "$RTABMAP_DB_DIR" ] \
      || { echo "board: RTABMAP_DB_DIR $RTABMAP_DB_DIR cannot be made or is not writable" >&2; exit 2; }
  fi
  kv_precheck || exit 9
  if [ -z "$BUSDEV" ] || [ ! -c "$BUSDEV" ]; then
    echo "board: no Orbbec camera on USB (${CAR_LSUSB:-lsusb} -d 2bc5: gave '${BUSDEV:-nothing}', not a character device); plug it in (USB 3) and retry" >&2
    exit 3
  fi
fi
mkdir -p "$CAR_STATE_DIR" "$(dirname "$RUN")" || { echo "cannot create $CAR_STATE_DIR or $(dirname "$RUN") (board default /data/fpga/car must be writable; or export CAR_STATE_DIR / CAR_RUNS_DIR)" >&2; exit 2; }
mkdir "$RUN" || { echo "run directory $RUN exists or cannot be made" >&2; exit 2; }
ln -s "$RUN" "$POINTER" 2>/dev/null || { rmdir "$RUN"; echo "a car is already running ($POINTER appeared)" >&2; exit 10; }
mkdir "$IPC" && chmod 700 "$IPC" || { rm -f "$POINTER"; echo "IPC directory $IPC exists" >&2; exit 2; }
if [ "$RTAB" = 1 ]; then   # car_down reads it (the stack's stop wait); host: the database directory inside the run
  echo $((CAR_RTABMAP_STOP_S + 90)) > "$RUN/stack_stop_s.txt"
  [ "$PLATFORM" = board ] || mkdir -p "$RTABMAP_DB_DIR"
fi
ROLLED=0
rollback() {   # CODE MESSAGE
  trap - INT TERM
  echo "car_up FAILED ($2); rolling back" >&2
  echo "$(date +%s.%N) car_up FAIL $1 $2" >> "$RUN/up_marks.txt"
  [ "$ROLLED" = 1 ] || { ROLLED=1; bash "$HERE/car_down.sh" --run "$RUN" --rollback "$2" >&2; }
  exit "$1"
}
trap 'rollback 130 "interrupted"' INT TERM
umark() { echo "$(date +%s.%N) $*" >> "$RUN/up_marks.txt"; }
umark "car_up camera=$CAMERA pose=$POSE nav=$NAV detector=$DETECTOR chassis=$CHASSIS"
if [ "$PLATFORM" = board ]; then
  printf '%s' "$KV_CHECK_LOG" > "$RUN/kv_precheck.txt"
  [ -z "$KV_UNREG_WARN" ] || umark "WARN board precheck: $KV_UNREG_WARN"
  umark "board precheck: kv $KV_SERVER build id $KV_BUILD_ID ok; camera $BUSDEV"
  # 10-29 health check facts the stack container cannot see (host/hub/health.py reads $RUN/host_facts.json): the
  # Nano power mode, the KV wall clock against the Nano's (one ssh `date`, KV NTP is not synced), and the KV
  # service's ready JSON (car screen on/off, screen link) handed over by nano_up.sh in CAR_KV_READY_JSON.
  # Best effort, a few seconds at most; nothing here refuses the start.
  HF_NVP=$( { sudo -n nvpmodel -q 2>/dev/null || nvpmodel -q 2>/dev/null; } | sed -n 's/.*Power Mode: *//p' | head -1)
  HF_CONF=${CAR_KV_SSH_CONF:-/home/robot/.ssh/geomem-kv.conf}; HF_T0=""; HF_KV=""; HF_T1=""
  if [ -f "$HF_CONF" ]; then
    HF_T0=$(date +%s.%N)
    HF_KV=$(timeout 6 ssh -F "$HF_CONF" -o BatchMode=yes -o ConnectTimeout=4 geomem-kv260 'date +%s.%N' < /dev/null 2>/dev/null | tail -1)
    HF_T1=$(date +%s.%N)
  fi
  python3 - "$RUN/host_facts.json" "$HF_NVP" "$HF_T0" "$HF_KV" "$HF_T1" "${CAR_KV_READY_JSON:-}" <<'PY' \
    && umark "host facts: $(tr -d '\n' < "$RUN/host_facts.json" | cut -c1-200)" || umark "host facts: not written"
import json, sys
out, nvp, t0, kv, t1, ready = sys.argv[1:7]
facts = {"power_mode": nvp or None, "kv_wall_minus_nano_s": None, "kv_clock_rtt_s": None, "kv_ready": None}
try:
    a, k, b = float(t0), float(kv), float(t1)
    facts["kv_wall_minus_nano_s"] = round(k - (a + b) / 2.0, 3)
    facts["kv_clock_rtt_s"] = round(b - a, 3)
except ValueError:
    pass
try:
    r = json.loads(ready) if ready else None
    facts["kv_ready"] = r if isinstance(r, dict) else None
except ValueError:
    pass
json.dump(facts, open(out, "w"), ensure_ascii=False, indent=1)
PY
fi
echo "$IPC" > "$RUN/ipc.txt"
SNAP_DIRS="host robot_sim tools pc perception hw"
if git -C "$REPO" rev-parse --is-inside-work-tree > /dev/null 2>&1; then
  git -C "$REPO" rev-parse HEAD > "$RUN/head.txt" 2>/dev/null
  git -C "$REPO" status --porcelain > "$RUN/dirty.txt" 2>/dev/null
  mkdir "$RUN/src" && (cd "$REPO" && git ls-files -z --cached --others --exclude-standard -- $SNAP_DIRS \
     | tar --null -T - -cf -) | tar -xf - -C "$RUN/src" || rollback 8 "source snapshot"
elif [ -s "$REPO/COMMIT" ]; then
  # a tree deployed with git archive (the board runs: session_v3.sh, bench_line4.sh) has no .git: its COMMIT file
  # names the commit, the archive itself is the clean tree (nothing untracked can be in it)
  head -1 "$REPO/COMMIT" > "$RUN/head.txt"
  echo "# no git work tree: deployed archive of the commit in $REPO/COMMIT" > "$RUN/source_kind.txt"
  : > "$RUN/dirty.txt"
  DIRS=(); for d in $SNAP_DIRS; do [ -d "$REPO/$d" ] && DIRS+=("$d"); done
  mkdir "$RUN/src" && (cd "$REPO" && tar -cf - "${DIRS[@]}") | tar -xf - -C "$RUN/src" || rollback 8 "source snapshot"
else
  rollback 8 "source snapshot: $REPO is neither a git work tree nor a deployed tree with a COMMIT file"
fi
HEAD=$(cat "$RUN/head.txt")
# 10-28 ground calib last good: the last adopted fit, into the run dir (the stack sees it as /o/...; car_inner.sh
# passes it to ground_calib.py --last-good, the fallback before the configured mount)
if [ "$PLATFORM" = board ] && [ "$CAR_GROUND_CALIB" = auto ] && [ -s "$CAR_STATE_DIR/ground_calib_last_good.json" ]; then
  cp "$CAR_STATE_DIR/ground_calib_last_good.json" "$RUN/ground_calib_last_good_in.json" \
    && umark "ground calib last good: $(tr -d '\n' < "$RUN/ground_calib_last_good_in.json" | cut -c1-160)"
fi
# 10-18 object memory: this run's map identity and the checked memory file, as the runner will read them
printf '%s\n' "$MEM_IDENT" > "$RUN/map_identity.json" || rollback 2 "memory: map_identity.json"
if [ -n "$CAR_MEMORY_IN" ]; then
  cp "$CAR_MEMORY_IN" "$RUN/memory_in.json" || rollback 2 "memory: copy CAR_MEMORY_IN"
  [ "$(sha256sum "$RUN/memory_in.json" | cut -c1-64)" = "$MEM_IN_SHA" ] \
    || rollback 2 "memory: $CAR_MEMORY_IN changed between the check and the copy"
fi
"$MEM_PY" - "$RUN" "$CAR_MEMORY_IN" "${MEM_IN_SHA:-}" "$CAR_MEMORY_SAVE" "$MEM_ID_DESC" <<'PY' > "$RUN/memory_plan.json" \
  || rollback 2 "memory: memory_plan.json"
import json, sys
run, src, sha, save, desc = sys.argv[1:6]
ident = json.load(open(run + "/map_identity.json"))
print(json.dumps({"save": save == "1", "in": src or None, "in_sha256": sha or None,
                  "in_copy": "memory_in.json" if src else None, "map_identity_source": desc,
                  "map_identity_kind": ident["kind"], "db_sha256": ident.get("db_sha256")}, indent=1))
PY
DIRTY=False; [ -s "$RUN/dirty.txt" ] && DIRTY=True
YMODEL=""; [ "$DETECTOR" = yolo ] && case "$YOLO_BACKEND" in torch) YMODEL=$YOLO_PT;; ort) YMODEL=$YOLO_ONNX;; trt) YMODEL=$YOLO_ENGINE;; esac
python3 - "$RUN" <<PY || rollback 8 "config"
import json, sys, hashlib, os
run = sys.argv[1]
def sha(p):
    try:
        return hashlib.sha256(open(p, 'rb').read()).hexdigest()
    except OSError:
        return ''
car = {"camera": "$CAMERA", "pose": "$POSE", "nav": "$NAV", "detector": "$DETECTOR", "chassis": "$CHASSIS",
       "yolo_backend": "$YOLO_BACKEND" if "$DETECTOR" == "yolo" else "", "source": "$SOURCE", "platform": "$PLATFORM",
       "record": $RECORD == 1, "tag": "$TAG", "containers": {"gz": "$GZ", "kv": "$KV", "stack": "$STACK"},
       "ipc": "$IPC", "hub": {"bind": "$HUB_BIND", "port": int("$HUBPORT"), "token_file": "/o/token.txt",
                              "note": "hub 实际绑定地址；tools/car_run/car_pc.sh 据此连接"},
       "inference_device": "$INF_DEVICE", "gpu": "$INF_GPU" if "$INF_DEVICE" == "cuda" else "",
       "host_numbers_note": "宿主，不代表板上"}
if car["platform"] == "board":
    car["camera_mount"] = {"height_m": float("$CAR_CAM_HEIGHT"), "pitch_deg": float("$CAR_CAM_PITCH_DEG"),
                           "height_cell": int("${HEIGHT_CELL:-0}"), "base_link_to_camera_xyz_q": "${MOUNT_TF:-}",
                           "camera_frame": "$LIVE_CAMERA_FRAME",
                           "height_uncertainty_m": float("$CAR_CAM_HEIGHT_UNC_M"),
                           "height_measurement": "10-28 owner 重调相机，板上地面自标定 0.392 m / 15.75° / 横滚 0.87°，默认取 0.39 m / 16.0°（此前 10-17 起 0.39 m / 7.0°，10-16 手测 0.375 m / 30°）",
                           "lens_to_front_m": 0.007,
                           "lens_reference": "彩色与深度镜头中心高度近似相同（owner 10-15 已答）；TF 挂彩色光学系",
                           "height_cell_given": "$CAR_HEIGHT_CELL" != "",
                           "source": "board ground fit 10-28 after the owner's re-tilt (common.py default 0.35/27 unchanged)",
                           "robot_height_m": float("$CAR_ROBOT_HEIGHT"),
                           "robot_height_note": "车身高度（障碍带上沿 = 此值 + 0.05 m），owner 10-16 定 0.31",
                           "ground_calib": {"mode": "$CAR_GROUND_CALIB",
                                            "note": ("off: 用上面的配置值，不拟合" if "$CAR_GROUND_CALIB" == "off"
                                                     else "READY 后由 ground_calib.json 回填采用值与来源")}}
    car["live_wiring"] = {"launch_args": "$LIVE_LAUNCH_ARGS", "camera_frame": "$LIVE_CAMERA_FRAME",
                          "color": "$LIVE_COLOR_TOPIC", "depth": "$LIVE_DEPTH_TOPIC", "camera_info": "$LIVE_INFO_TOPIC",
                          "runner_camera_qos": "$RUNNER_CAMERA_QOS", "producer_depth_qos": "$PRODUCER_DEPTH_QOS",
                          "driver_dds_profile": "$LIVE_DRIVER_DDS_PROFILE", "camera_fps": int("$CAR_CAMERA_FPS"),
                          "camera_rate_check": {"on": int("$CAR_CAMERA_RATE_CHECK"), "window_s": int("$CAR_CAMERA_RATE_S"),
                                                "result": "camera_rate_check.json" if "$CAR_CAMERA_RATE_CHECK" == "1" else None}}
    car["host_numbers_note"] = ""
car["pose_note"] = {"reference": "参考位姿（仿真/回放）", "rtabmap": "rtabmap 定位（草案，固定定位库）",
                    "map": "rtabmap 建图（新库，不是定位；不起导航与运行器）",
                    "static": "台架静态位姿（相机固定、无底盘：map->odom->base_link 恒等即真位姿），不是建图定位",
                    "odom": "轮式里程位姿：map->odom 恒等（起点即原点）+ odom->base_link 由轮式里程发，不是建图定位"}["$POSE"]
if "$RTAB" == "1":
    car["rtabmap"] = {"odom": "${RTABMAP_ODOM:-}", "db_dir": "${RTABMAP_DB_DIR:-}", "db_dir_in_stack": "$RTABMAP_DB_DIR_IN",
                      "stop_s": int("$CAR_RTABMAP_STOP_S"), "stack_stop_s": int("$CAR_RTABMAP_STOP_S") + 90}
if "$NAV" == "nav2" and "$PLATFORM" == "board":
    car["nav2_map_yaml"] = "$CAR_MAP_YAML"
if "$PLATFORM" == "board":
    car["runner_budgets"] = {"navigation_timeout_s": float("$CAR_NAV_TIMEOUT_S"), "task_timeout_s": float("$CAR_TASK_TIMEOUT_S"),
                             "stream_timeout_s": float("$CAR_STREAM_TIMEOUT_S"),
                             "note": "运行器生效值（car_up CAR_NAV_TIMEOUT_S / CAR_TASK_TIMEOUT_S / CAR_STREAM_TIMEOUT_S，仅板上）；stream_timeout_s = 处理帧断流多久判 failed:stream_lost"}
car["front_seed"] = ({"enabled": True, "m": float("$CAR_FRONT_SEED_M"), "half_angle_deg": 35.0,
                      "note": "滚动层前向扇区播种：当前朝向 ±35° 内、距车心该半径内的未知格当空闲（只看当前位姿，障碍格不改）；未见过的矮障碍会被当空闲"}
                     if "$CAR_FRONT_SEED_M" else {"enabled": False, "m": None,
                                                  "note": "默认关（触及 09-27 滚动层规则，待 owner 拍板）"})
car["nav_switches"] = {"given": dict(kv.split("=", 1) for kv in "${NAV_SW_ENV[*]}".split()),
                       "defaults": {"FPGA_NAV_ROTATION_SHIM": "true", "FPGA_NAV_PROGRESS_CHECKER": "pose",
                                    "FPGA_NAV_CAR_PLANNER": "strict"},
                       "note": "车档 Nav2 开关（仅 board nav2 生效）；given 为空=全用默认"}
car["dwb_forward_prune"] = dict(json.loads('$PRUNE_JSON'), note="车档 DWB 目标评分瞄准路径前方多远（m）；given=由 CAR_DWB_FORWARD_PRUNE_M 覆盖，2.0=改前的 Nav2 默认（仅 board nav2 生效）")
car["dwb_min_speeds"] = dict(json.loads('$DWB_JSON'), note="车档 DWB 最小速度与停转判据的生效值；overridden 列出由 CAR_* 环境变量覆盖的项，其余为 nav_input_profile.py 常量（仅 board nav2 生效）")
car["nav_dryrun"] = ({"enabled": True, "note": "板上 Nav2 空跑：static 位姿、无底盘，车不动；/fpga/cmd_vel_safe 不接任何东西"}
                     if "$NAV_DRYRUN" == "1" else {"enabled": False})
if "$NAV_BACKEND" != "nav2":   # CTRLPS2 11-03: only when not the default, so car.json stays as before
    car["nav_backend"] = {"backend": "$NAV_BACKEND", "note": "kv=KV260 PS 导航开车（Nav2 只留 map_server+planner_server 出全局路线）；shadow=Nav2 开车、KV 命令只记录（kv_nav_cmds.jsonl）"}
car["teleop"] = {"enabled": "$TELEOP" == "1",
                 "note": ("遥控＋Nav2 仲裁趟：遥控在手时接管、在手期间拒新导航；遥控不经运动许可；单片机心跳锁存兜底"
                          if "$NAV" == "nav2" else "遥控趟：不是导航趟，不经运动许可；单片机心跳锁存兜底") if "$TELEOP" == "1" else "",
                 "arbitration": "$TELEOP" == "1" and "$NAV" == "nav2",
                 "v_max": "$CAR_TELEOP_V_MAX" or "0.15 (default)", "w_max": "$CAR_TELEOP_W_MAX" or "0.6 (default)",
                 "max_cmd_s": "$CAR_TELEOP_MAX_CMD_S" or "3 (default)"} if "$TELEOP" == "1" else {"enabled": False}
wheel = {"wheel_diam_m": "${WHEEL_GIVEN[0]}", "counts_per_rev": "${WHEEL_GIVEN[1]}", "track_m": "${WHEEL_GIVEN[2]}"}
used = {"wheel_diam_m": "$CAR_WHEEL_DIAM_M", "counts_per_rev": "$CAR_COUNTS_PER_REV", "track_m": "$CAR_TRACK_M"}
car["wheel_odom"] = {"enabled": "$CAR_WHEEL_ODOM" == "1", "params_given": wheel,
                     "params_used": used if all(used.values()) else "placeholder 0.065 / 1320 / 0.160 (host/wheel_odom.py)",
                     "source": "$WHEEL_SRC" or ("env (CAR_WHEEL_DIAM_M/CAR_COUNTS_PER_REV/CAR_TRACK_M)" if all(wheel.values())
                                                else "placeholder (host default)" if not any(wheel.values())
                                                else "mixed (given + placeholder)"),
                     "placeholder": not all(used.values()),
                     "placeholder_note": "占位轮参数（65 mm / 1320 / 160 mm）：里程只是粗推算" if not all(used.values()) else "",
                     "tf": ("/tf + /odom" if "$CAMERA" == "live" else "shadow /fpga/wheel_odom/tf (replay/sim own odom->base_link)")}
car["wheel_odom"] = car["wheel_odom"] if car["wheel_odom"]["enabled"] else {"enabled": False}
if "$POSE" == "odom" and "$NAV" == "nav2":
    car["bench"] = {"stand_in_map_m": float("$BENCH_MAP_M"),
                    "stand_in_map_note": "nav2：bench_pose 只发 map->odom 恒等，不发 /map；/map 由 Nav2 map_server 发（nav2_map_yaml）"}
elif "$POSE" == "odom":
    car["bench"] = {"stand_in_map_m": float("$BENCH_MAP_M"), "stand_in_map_note": "全空替身 /map（frame map），只为让运行器接受查询；不是建出的地图"}
if "$POSE" == "static":
    car["bench"] = {"odom_hz": float("$BENCH_ODOM_HZ"), "stand_in_map_m": float("$BENCH_MAP_M"),
                    "stand_in_map_note": "全空替身 /map（frame map），只为让运行器接受查询；不是建出的地图"}
json.dump(car, open(os.path.join(run, 'car.json'), 'w'), indent=1, ensure_ascii=False)
# the layout tools/verify_bottle_run.py --layout sim reads (same keys as tools/sim_loop/run_sim_loop.sh)
ymodel = "$YMODEL"
cfg = {"mode": "car_up", "tag": "$TAG", "head": "$HEAD", "dirty": $DIRTY, "image": "$IMAGE",
       "gz_container": "$GZ", "kv_container": "$KV", "stack_container": "$STACK",
       "gz_cpus": "$GZ_CPUS", "kv_cpus": "$KV_CPUS", "kv_threads": $KV_THREADS, "stack_cpus": "$STACK_CPUS",
       "inference_cpus": "$INF_CPUS", "runtime_cpus": "$RUNTIME_CPUS",
       "inference": {"detector": "$DETECTOR", "device": "$INF_DEVICE", "gpu": "$INF_GPU" if "$INF_DEVICE" == "cuda" else "",
                     "augmentation": "$AUG", "models": "$MODELS", "warmup_frames": $INF_WARMUP, "warmup_size": "$INF_WARMUP_SIZE",
                     "yolo_backend": "$YOLO_BACKEND" if "$DETECTOR" == "yolo" else "", "yolo_model": ymodel,
                     "yolo_profile": "${YOLO_PROFILE:-}", "yolo_route": ("${CAR_YOLO_ROUTE:-}" or "all33") if "${YOLO_PROFILE:-}" == "combined33" else "",
                     "yolo_model_sha256": sha(ymodel) if ymodel else "",
                     "yolo_onnx": "$YOLO_ONNX" if "$DETECTOR/$YOLO_BACKEND" == "yolo/ort" else "",
                     "yolo_threads": "$YOLO_THREADS" if "$DETECTOR/$YOLO_BACKEND" == "yolo/ort" else ""},
       "world_file": "$FPGA_SIM_WORLD_FILE" if "$CAMERA" == "sim" else "",
       "chassis": {"pty": "mcu", "none": "guard"}.get("$CHASSIS", "$CHASSIS") if "$CAMERA" == "sim" else "$CHASSIS",
       "car_chassis": "$CHASSIS", "mcu_heartbeat": "$MCU_HEARTBEAT", "controller": "dwb" if "$CAR_NAV_COSTMAP" == "rolling" else "rpp_heading_precision",
       "nav_costmap": "$CAR_NAV_COSTMAP",
       "car_failstop": "${CAR_FAILSTOP:-0}",
       "host_numbers_note": "宿主，不代表板上"}
json.dump(cfg, open(os.path.join(run, 'run_config.json'), 'w'), indent=1, ensure_ascii=False)
PY

if [ "$NEED_GPU_LOCK" = 1 ]; then
  $CAR_LOCK acquire "gpu$INF_GPU" "car_up $TAG (line E one-command car, $CAMERA)" -u "$CAR_LOCK_USER" > "$RUN/gpu_lock_acquire.txt" 2>&1 \
    || rollback 6 "GPU lock gpu$INF_GPU not acquired ($(tail -1 "$RUN/gpu_lock_acquire.txt"))"
  python3 - "$RUN/gpu_lock_acquire.txt" "gpu$INF_GPU" "$CAR_LOCK_USER" > "$RUN/gpu_lock.json" <<'PY' || rollback 6 "GPU lock: no token in the answer"
import json, sys
text = open(sys.argv[1]).read()
for line in text.splitlines():
    line = line.strip()
    if line.startswith('{'):
        token = json.loads(line).get('token')
        if token:
            print(json.dumps({"resource": sys.argv[2], "token": token, "user": sys.argv[3]}))
            sys.exit(0)
sys.exit(1)
PY
  umark "gpu lock gpu$INF_GPU acquired"
fi

if [ ${#INF_CMD[@]} -gt 0 ]; then
  # setsid: its own session (outlives this script); SIGINT back to default (a background child of a
  # non-interactive bash starts with it ignored, and this host's env has no --default-signal)
  setsid python3 -c 'import os, signal, sys; signal.signal(signal.SIGINT, signal.SIG_DFL); os.execvp(sys.argv[1], sys.argv[1:])' \
    "${INF_CMD[@]}" > "$RUN/inference.log" 2>&1 < /dev/null &
  INF=$!
  echo "inference $INF $(car_pid_start "$INF")" >> "$RUN/host_pids.txt"
  for ((i=0; i<INF_READY_S*10; i++)); do
    grep -q '"ready": true' "$RUN/inference.log" 2>/dev/null && [ -S "$IPC/inf.sock" ] && break
    car_pid_alive "$INF" || rollback 4 "inference service exited ($(tail -1 "$RUN/inference.log"))"
    sleep 0.1
  done
  grep -q '"ready": true' "$RUN/inference.log" && [ -S "$IPC/inf.sock" ] || rollback 4 "inference service not ready in $INF_READY_S s"
  umark "ready infer"
fi

start_container() {   # NAME CMD...
  local n=$1; shift
  echo "$n" >> "$RUN/containers.txt"
  DK "$@" > "$RUN/container-$n.id" 2> "$RUN/container-$n.err" || rollback 5 "container $n did not start ($(tail -1 "$RUN/container-$n.err"))"
}
running() { [ "$(DK inspect -f '{{.State.Running}}' "$1" 2>/dev/null)" = true ]; }
if [ ${#GZ_CMD[@]} -gt 0 ]; then
  start_container "$GZ" "${GZ_CMD[@]}"
  sleep 3; running "$GZ" || rollback 5 "Gazebo container exited at start"
  umark "ready gz container"
fi
if [ ${#KV_CMD[@]} -gt 0 ]; then   # host only: the fake KV260 (the board uses the real one, checked above)
  start_container "$KV" "${KV_CMD[@]}"
  sleep 1; running "$KV" || rollback 5 "fake KV container exited at start"
  umark "kv container started"
fi
STACK_CMD=("${STACK_CMD[@]/@HEAD@/$HEAD}")
start_container "$STACK" "${STACK_CMD[@]}"
umark "stack container started"
# 10-29 CPU priorities (header CAR_PRIORITY): root in the stack container, every 3 s; ends with the container. Never
# stops the start: a missing or failing priority process only leaves the nice values as they were (priority.log).
if [ "$PLATFORM" = board ] && [ "$CAR_PRIORITY" = 1 ]; then
  DK exec -u 0 "$STACK" python3 /src/tools/car_run/car_priority.py --interval-s 3 > "$RUN/priority.log" 2>&1 < /dev/null &
  PRIO=$!
  # pid + start time (identity for car_down); a docker exec that already ended is not recorded (nothing to stop)
  PRIO_ST=$(car_pid_start "$PRIO") && echo "priority $PRIO $PRIO_ST" >> "$RUN/host_pids.txt"
  umark "priority started (docker exec pid $PRIO, log priority.log)"
fi

for ((i=0; i<CAR_READY_S*2; i++)); do
  [ -e "$RUN/READY" ] && break
  [ -s "$RUN/stack_fail.txt" ] && rollback 7 "stack: $(cat "$RUN/stack_fail.txt")"
  running "$STACK" || { sleep 1; rollback 7 "stack container exited before READY ($(cat "$RUN/stack_fail.txt" 2>/dev/null))"; }
  [ -z "${INF:-}" ] || car_pid_alive "$INF" || rollback 4 "inference service exited"
  sleep 0.5
done
[ -e "$RUN/READY" ] || rollback 7 "not ready in $CAR_READY_S s ($(tail -1 "$RUN/marks.txt" 2>/dev/null))"
trap - INT TERM
umark "READY"
# the priorities really applied (the last processes start just before READY: one more 3 s scan first; background)
[ "$PLATFORM" = board ] && [ "$CAR_PRIORITY" = 1 ] && \
  { sleep 4; umark "$(python3 "$HERE/car_priority.py" --summarize "$RUN/priority.log" 2>&1 | tail -1)"; } > /dev/null 2>&1 < /dev/null &
# the ground calibration's adopted mount into car.json camera_mount (the stack wrote ground_calib.json before the TF)
if [ "$PLATFORM" = board ] && [ "$CAR_GROUND_CALIB" != off ]; then
  python3 - "$RUN" <<'PY' || umark "car.json: ground_calib not filled in"
import json, os, sys
run = sys.argv[1]
car = json.load(open(os.path.join(run, "car.json")))
g = json.load(open(os.path.join(run, "ground_calib.json")))
a = g.get("adopted") or {}
car["camera_mount"]["ground_calib"].update({
    "verdict": g.get("verdict"), "adopted_source": a.get("source"), "adopted_height_m": a.get("height_m"),
    "adopted_pitch_deg": a.get("pitch_deg"), "adopted_roll_deg": a.get("roll_deg"), "fitted": g.get("fitted"),
    "adopted_height_cell": g.get("height_cell"), "adopted_base_link_to_camera_xyz_q": g.get("mount_tf"),
    "result": "ground_calib.json",
    "alert": g.get("alert"),
    "note": "fit = 采用开机拟合值（上面的 height_m/pitch_deg 是配置值）；config = 核对通过，沿用配置值；"
            "last_good = 拟合没过、用上次采用的拟合值；config_fallback = 拟合没过且无上次值、用配置值"})
json.dump(car, open(os.path.join(run, "car.json"), "w"), indent=1, ensure_ascii=False)
PY
  # 10-28: an adopted FIT becomes the last good mount for the next starts' fallback (a fallback is never written)
  [ "$CAR_GROUND_CALIB" = auto ] && umark "$(python3 "$HERE/ground_calib.py" --write-last-good "$RUN/ground_calib.json" \
    "$CAR_STATE_DIR/ground_calib_last_good.json" 2>&1 | tail -1)"
fi
echo "car up: $CAMERA/$POSE/$NAV/$DETECTOR/$CHASSIS (source $SOURCE) run $RUN"
[ "$POSE" = static ] && echo "  pose static = 台架静态位姿（相机固定、无底盘），不是建图定位；/map 为全空替身"
[ "$POSE" = odom ] && echo "  pose odom = 轮式里程位姿（起点即原点），不是建图定位；/map 为$([ "$NAV" = nav2 ] && echo "Nav2 map_server 发的建图导出 $CAR_MAP_YAML（车须从建图起点同位同向出发）" || { [ -n "$ODOM_MAP_YAML" ] && echo "建图导出 $ODOM_MAP_YAML（车须从建图起点同位同向出发）"; } || echo "全空替身 ${BENCH_MAP_M} m")"
[ "$TELEOP" = 1 ] && [ "$NAV" = nav2 ] && echo "  TELEOP＋Nav2 仲裁：遥控在手时接管（首条撤导航），在手期间新导航目标被拒；电脑端 car_pc.sh teleop --keys"
[ "$TELEOP" = 1 ] && [ "$NAV" != nav2 ] && echo "  TELEOP 遥控趟（不是导航趟）：python3 $HERE/teleop_keys.py --socket $IPC/teleop.sock   (w/s/a/d/q/e，空格停，Ctrl-C 退)"
grep ' ready ' "$RUN/marks.txt" | sed 's/^[0-9.]* /  /'
echo "PC commands: bash $HERE/car_pc.sh find 瓶子 | probe | estop ...    stop: bash $HERE/car_down.sh"
exit 0
