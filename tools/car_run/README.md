# 整车一键起停（线 E，2026-10-14）

```bash
bash tools/car_run/car_up.sh sim          # 仿真车：sim + reference + nav2 + yolo(宿主 torch GPU) + pty(mcu_sim 链)
bash tools/car_run/car_up.sh replay       # 回放：replay(rec30) + reference + costmap + stub + none
bash tools/car_run/car_up.sh live --dry-run   # 真车（板上，草案）：live + rtabmap + nav2 + yolo(trt) + uart；宿主只能 dry-run
bash tools/car_run/car_up.sh live --pose static --chassis none --nav costmap --dry-run   # 线 4 台架（见下「线 4 台架」）
bash tools/car_run/car_pc.sh find 瓶子     # 电脑端命令（在栈容器里跑 fpgapc，带本次 token，连本趟 hub 实际绑定地址；查询用 probe）
bash tools/car_run/car_down.sh            # 逐级停、查残留（down.json），释放 GPU 锁
```

预设之后可逐项覆盖：`--camera sim|replay|live --pose reference|rtabmap|static --nav costmap|nav2 --detector stub|ssdlite|yolo
--chassis none|pty|uart --yolo-backend torch|ort|trt --gpu N --run-dir DIR --record --dry-run`。
来源标记随相机（sim→sim、replay→replay、live→live），不是开关。`car_up` 就绪后即返回，车一直跑（无时限），直到 `car_down`。

## 合法组合

| 相机 | 位姿 | 导航 | 识别 | 底盘 | 平台 |
|---|---|---|---|---|---|
| sim | reference | nav2 | ssdlite / yolo（torch GPU 或 ort CPU） | pty（mcu_sim 链）/ none（D1 速度守护＋KV 急停替身） | 宿主 |
| replay | reference | costmap | stub | none / pty | 宿主 |
| live | rtabmap | nav2 / costmap | yolo（trt） | uart / none | 板上（草案；起之前的检查见下「真车（live）预设」） |
| live | static | costmap | yolo（trt） | none | 板上，**线 4 台架**：相机按装车姿态固定、无底盘，车不会动（见下「线 4 台架」） |

其余一律在启动前拒绝（退出码 2，逐条说明原因）：如 sim/replay 配 uart（真串口接真车）、live 配 reference（真车没有参考位姿）、
live 配 pty（假单片机，真车不动）、replay 配 nav2（回放车走录好的轨迹）、stub 配非 replay（stub 读回放 RGB 里的帧号）、
replay 配真识别器（回放 RGB 无图像）、板上 ssdlite / 宿主 trt 等；static 只配 live（仿真/回放用 reference）、只配 chassis none
（接了底盘车会动，恒等位姿就不是真位姿）、只配 costmap（台架车不动，Nav2 无从验证）。板上 `--pose rtabmap` 没给 `RTABMAP_DB`
（一个存在的定位库文件）时在建任何东西之前以 2 拒绝。

## 起停顺序与就绪判据

宿主：GPU 锁（仅宿主 GPU 推理）→ 推理服务（`"ready": true`＋socket，≤120 s；预热在 ready 前）→ Gazebo 容器（3 s 后仍在跑）
→ 假 KV 容器 → 栈容器 `car_inner.sh`：kv（kv.json 端口）→ 相机（sim：Gazebo `/fpga/depth`＋`/clock` ≤240 s、深度适配
`/fpga/depth16`；replay：replay.csv 出帧）→ 底盘（pty：mcu_sim 出 pty、底盘驱动收到单片机状态帧）→ 导航（nav2：
`nav_startup_probe.py --activate`；costmap：生命周期 "Managed nodes are active"）→ hub（"hub listening"）→ 滚动桥（活 3 s）
→ 生产节点（首个帧行）→ stub（socket）→ 运行器（startup 事件＋首个 frame_processed）→ `READY`。
板上 nav2 且滚动代价地图（`FPGA_NAV_COSTMAP` 默认 rolling，10-15）时导航一步挪到生产节点之后：hub → 滚动桥 → 生产节点 →
导航（launch 后 `grid_probe.py` 须在 `GRID_READY_S`（60 s）内收到 `/fpga/rolling_map/grid`，再 `--activate`）→ 运行器；停车仍按启动逆序。
每步写 `marks.txt` 的 `ready <步>`；任何一步不过：写 `stack_fail.txt`、栈内按 pid 逆序停，`car_up` 调 `car_down --rollback`
停掉已起的全部（容器、推理服务、锁、IPC、指针），非 0 退出。就绪后每秒巡检：任一子进程退出即整栈停（fail-stop）。

停：栈容器 `docker kill -s INT`（栈内逐个 INT→TERM→KILL）→ 假 KV（INT，写 summary）→ Gazebo → 推理服务（pid＋启动时刻，
INT→TERM→KILL）→ 释放锁 → 删 IPC 目录与指针；残留自检写 `down.json`（容器、进程、IPC、锁、指针全无才退出 0）。
栈内每停一个子进程先在 `marks.txt` 记 `stop <名>`（逐个停车耗时）。C++ 节点（代价地图、生命周期管理、安装 TF）**直接起可执行文件**
（`/opt/ros/humble/lib/<包>/<程序>`，按 `AMENT_PREFIX_PATH` 找），不经 `ros2 run`：`ros2 run` 是个 Python 外壳，收到 INT 只接着等
子进程、不转发（它假设终端 Ctrl-C 打到整个进程组），按 pid 发的 INT 到不了节点——线 4 台架 10-15 这三个各等 10 s 再 TERM（rc 143，
节点成孤儿），栈停 39–41 s。宿主 replay 实测（10-15）：正常停栈内 25.7/25.9 s → 2.0/2.4/2.0 s，`car_down` 全程 27.0/27.6 s → 3.4/4.1/4.1 s；fail-stop 杀生产节点到栈容器退出 25.4 s → 2.3 s。独立代价地图（Nav2 1.1.20）收 INT 时的退出码看
生命周期状态（宿主 10-15 逐态实测）：激活中 134（更新线程未回收，`terminate called without an active exception`）、已停用 0、
经 cleanup（RESET/SHUTDOWN）后 139。所以**停车先经生命周期管理器 `check_lifecycle` 的 `manage_nodes` 做 PAUSE（只停用）**：
停车一开始在后台发起（`ros2 service call`，宿主约 0.5–1.7 s，与停前面几个进程并行），停生命周期管理器之前等它（最多
`COSTMAP_PAUSE_S`=8 s），失败或超时只在 `marks.txt` 记一行照常往下停；顺序仍是逆序。宿主 replay 两趟：`costmap_rc=0`，栈内停
2.2/2.0 s（修前同条件 1.9/1.9 s），代价地图析构时另有一行无害的 `Unable to start transition 2 ... context is invalid`。
同时修了 `alive()` 在 EXIT 陷阱里恒答「不在」的缺陷（裸 `return` 取陷阱前的 130）。板上未验。

退出码：0 就绪；2 开关/组合（含板上相机安装高度放不进块）；3 本平台不能起（板上：USB 上没有奥比中光相机）；
9 板上 KV260 不在服务或比特流 BUILD_ID 不对；4 推理服务；5 容器；6 GPU 锁；7 未就绪（`stack_fail.txt` 说哪一步）；8 拷源；10 已在运行。
3、9 都在建运行目录之前拒绝，什么也不留下。

## 测试用开关

`--record`：运动记录、许可记录、单源检查、显示电脑监听（让出控制权）、runtime_info、运行器逐帧证据与 stall 诊断——
车不需要，只为 `car_scenario.sh`（D2 的 `bottle_scenario.py`，经 hub `fpgapc find 找瓶子`）＋ `verify_bottle_run.py --layout sim` 核验。
`car_scenario.sh` 的额外参数原样交给 `bottle_scenario.py`，用于 D2 的停车测试：急停 `car_scenario.sh --estop-during-return-s 3 --estop-hold-s 3 --estop-release-after-s 1`
（核验 `--layout sim-estop`）；断流 `CAR_FAILSTOP=log car_up.sh sim --record` 后 `car_scenario.sh --cut-during-return-s 3 --cut-pid-file /o/producer.pid`
（核验 `--layout sim-cut`；断流测试要停生产节点，须用 `log`（只记录不重启）；10-26 起默认 `CAR_FAILSTOP=0`：hub/bridge/preview/生产节点退出自动重启）。`CAR_FAILSTOP` 由 car_up 传进栈容器，记入 `run_config.json`。
替身：`CAR_DOCKER`、`CAR_LOCK`、`CAR_PY_HOST`；板上路径另有 `CAR_ARCH`、`CAR_LSUSB`、`CAR_USB_ROOT`、`CAR_KV_CHECK`、`CAR_PY_BOARD`
（单测 `tests/test_car_run.py`，宿主上用替身走到板上真启动分支，不碰相机、KV260、板子）。

## 真车（live）预设：起之前要有什么（2026-10-14 解阻塞）

`car_up.sh live` 在 Orin Nano 宿主上跑（镜像 `fpga-ros:humble-nav-orbbec-scipy-20261017`），真启动前按序检查，任何一条不过就拒绝、不建运行目录：

1. **KV260 在服务且比特流对**（退出码 9）：用生产节点同一套连接检查（`tools/kv_link/hello_connect_loop.py` → `KvGeometry.connect`：
   HELLO、roi_ext、ROI 核、经 TELEM 读 BUILD_ID），对 `KV_SERVER`（默认 `198.51.100.79:47800`）试 3 次，要求 BUILD_ID 等于
   `KV_BUILD_ID`（板上默认取 FPGA 档：默认 v3fc2＝`0x03c087c9`（m7_pl_v3fc2，10-26 板上短测通过后改为默认），`FPGA_PROFILE=a2`（`0x00da5041` A2 合并比特流 m7_pl_merge_a2）回退，`v3fc2_166|v3fc` 临时切（`v3fc3_187` 暂停：告警后按 v3fc2 跑）、长期改 `fpga_profiles.env` 一行；已登记的任一 BUILD_ID 都接受；
   档位只在 `tools/car_run/fpga_profiles.env` 一处定义，切换/部署/短测/回退见 `docs/team/FPGA_PROFILE_RUNBOOK_20261026.md`；宿主假 KV 不查）。
   结果记 `kv_precheck.txt`。不过时**先在 KV260 上（root）**载入比特流并起服务：
   `xmutil unloadapp; xmutil loadapp m7_pl_merge_a2`，然后
   `bash tools/kv_link/kv_board.sh start <repo> <out> --core pl --device devmem --wait irq --roi-core pl --port 47800`
   （参数照 `docs/design/integ_car_chain_v3_20261011/session_v3.sh` 的 SARGS；那里另加 `--max-idle-s 900`，空闲 15 分钟自退，整车长跑时按需去掉）。
   Nano 上没有远程起 KV 的现成路（session_v3 是从服务器经 `ssh kv260 sudo` 起的），所以 car_up 只查不起。
2. **相机 USB 节点**（退出码 3）：`lsusb -d 2bc5:` 第一行解析出 `/dev/bus/usb/BBB/DDD`，须是字符设备（只作存在性检查与记录）。
   10-17 起栈容器不再 `--device` 这一个节点，而是挂整个 `/dev/bus/usb` + `--device-cgroup-rule 'c 189:* rw'`（只放行 usbfs 主号；
   相机在驱动起流时会重新枚举、设备号 +1，见 `docs/design/CAM_USB_REENUM_20261017.md`）。不用 `--privileged`。`--dry-run` 打印解析结果，或「dry-run 未解析」。
3. **相机安装位姿**：`CAR_CAM_HEIGHT`（默认 0.39 m）、`CAR_CAM_PITCH_DEG`（默认 7.0°）——实测值：owner 10-17 14:16 手调相机（口称下倾约 10°），
   板上开机地面自标定 c10_1 拟合 0.3915 m / 7.05° / 横滚 0.59°（此前 10-16 装车手量 0.375 m / 30°，10-14 台架 0.36 m）。
   拟合高度 0.3915 m 超过原采用值高度硬限 0.39 m；owner 10-17 批准把上沿放到 0.395 m（镜头在 0.35–0.40 m 同一体素格、height_cell 8 不变，
   0.40 m 起块检查拒），`CAR_GROUND_CALIB=auto` 在这台车上可采用拟合值；配置下倾 < 20° 时自标定只用镜头前 0.3–1.4 m 的地面
   （`docs/design/CAM_TILT_10_20261017.md` §9）。
   `tools/car_run/car_mount.py` 由此算 base_link→相机光学系的静态 TF（栈里 `static_transform_publisher` 发 `/tf_static`，驱动
   `publish_tf:=false`）和生产节点 `--height-cell`：不给 `CAR_HEIGHT_CELL` 时按高度推（生产节点首帧同一个块检查：
   0.31–0.34 m → 7，0.36–0.39 m → 8，≥ 0.40 m 放不下、退出码 2），给了就只核对。`tools/car_geo_producer/common.py` 的
   0.35 m / 27° 不动：回放与宿主各趟照旧，保持可比。安装记在 `car.json` 的 `camera_mount`。
   **车身高度** `CAR_ROBOT_HEIGHT`（默认 0.31 m，owner 10-16，D-039）：滚动地图桥的障碍带上沿 = 车身高度 + 0.05 m；
   宿主仿真/回放仍是 0.25。栈里在地面自标定之后、发安装 TF 之前硬检查采用的相机高度 ≥ 车身高度 + 0.055 m，
   否则拒绝启动（退出码 7，原因写进 `stack_fail.txt`）；car_up 起栈前先用配置值查一次（退出码 2）。
4. **推理服务**：Nano 宿主 `python3 -m host.inference_service --detector yolo --engine $YOLO_ENGINE`（默认
   `/data/fpga/yolo_engine_20261015/yolov8n-seg-640-fp16.engine`，10-15 在 super 固件下用同一 ONNX、同一参数重建；09-27 旧引擎 `~/fpga/p2/w/` 保留，可用 `YOLO_ENGINE=` 指回），`PYTHONPATH=<快照 src>:$NANO_PYLIB`（默认 `/home/robot/fpga/p2/pylib`）。
   trt 后端按 `tools/nano_loop/nano_loop.py:177` 的写法 import `tools.nano_detector.nano_detector`（快照在前，`tools` 命名空间包不被遮）。
5. **驱动深度流重试**（栈内）：次数与间隔照 `tools/car_geo_producer/rig_inner.sh`，**判据改为真收到深度数据**：每次起驱动后
   `tools/car_run/depth_probe.py` 订阅 `LIVE_DEPTH_TOPIC`（`/camera/depth/image_raw`，sensor-data QoS），`LIVE_START_WAIT_S`（20 s）内
   收到 `LIVE_DEPTH_MSGS`（30）帧深度图才算起来（10-15 由 3 改 30：cam-start-dig 实测失败的起流会先漏出 1–4 帧深度再停，
   3 条会放过「出几帧就停」；30 条约 1 s 深度，同容器 10 个会话假通过 0；线 4 台架 5 趟用 30 条，每趟第 1 次就起来）；否则停掉（进程及其子进程，INT→KILL）、等 `LIVE_START_GAP_S`（10 s）再起，最多
   `LIVE_START_TRIES`（10）次。不用 rig 的「driver.log 出现 depth Frame」：控制器 10-15 板上整链 l_chain32 打印了这行档位、之后一帧深度
   都没来（rig 记 try=1 成功、60 s 后 exit 5）。每次写 `marks.txt` 的 `camera driver try=N depth_stream=0|1 probe_rc=R`，失败那次的
   日志存 `driver_failed_start_N.log`、探针输出存 `driver_probe_N.log`（含收到几帧、末帧非零占比）；全部失败即 `stack_fail`。
   板上实测约三次有两次深度流起不来（rig_inner.sh 10-13 记录：28 次 10 次好），所以这条必须有。

## 板上待验（2026-10-14 登记，未改；宿主只验到 dry-run 与替身）

- **运行目录与状态目录**（10-15 已改）：板上（`CAR_ARCH`/`uname -m`=aarch64，或 live 预设）默认 `CAR_RUNS_DIR=/data/fpga/car/runs`、`CAR_STATE_DIR=/data/fpga/car/state`（car_up/car_down/car_pc/car_scenario 共用 `car_common.sh` 的 `car_default_dirs`）；宿主默认不变。`/data/fpga/car` 须事先可写。IPC 目录＋`/teleop.sock` 超 107 字节时 car_up 以 2 拒绝。
- **源码快照**（10-15 已支持）：git 副本用 `git ls-files` 拷源；`git archive` 部署的树（无 `.git`）须有 `COMMIT` 文件（首行=提交号，
  `bench_line4.sh` 部署时写），按它拷 `host robot_sim tools pc perception hw`，`head.txt` 记该提交；两者都不是则以 8 回滚。
- **hub 绑定**：板上必须设 `CAR_HUB_BIND=<车局域网 IP>`（owner 10-14 定）。car_up 把实际绑定地址与端口记进 `car.json` 的 `hub`（`bind`/`port`），`car_pc.sh` 照它连（旧运行目录没有该键时读 `marks.txt` 的 `ready hub 地址:端口`，两处不一致拒绝）；两处都读不到就以 2 退出、命令不发，**不猜 127.0.0.1**。`CAR_HUB_HOST`/`HUBPORT` 可覆盖。宿主也可设 `CAR_HUB_BIND`，但栈容器是 `--network none`，只能绑 127/8 地址。
- **rtabmap 草案（下一项，10-15 不修）**：须给 `RTABMAP_DB`（固定定位库），否则 car_up 在建任何东西之前退 2（预期结果；
  `bench_line4.sh` 没给库时直接跳过、记一行原因）。按代码看给了库也起不来，三个问题待修：①rgbd_odometry 的 `/tf` 被
  `-r /tf:=/fpga/localization_raw_tf` 重映射走，没有节点把 odom→base_link 送回 `/tf`，生产节点会一直 `tf_missing`；
  ②定位门订的 `/probe/rgb` 等无人发布；③追踪帧 `camera_link` 不存在（驱动 `publish_tf:=false`，只有 base_link→彩色光学系）。
- **板上 Nav2**：源码快照里的 `navigation.launch.py`（按路径起），地图＝定位库导出（`MAP_YAML`），`FPGA_NAV_SPEED_LIMITS=car`
  （车速限＋xy 到点公差 0.10 m，仿真仍 0.03）。`FPGA_NAV_COSTMAP` 空/不给＝rolling：局部代价地图＝滚动桥的 `/fpga/rolling_map/grid`、
  控制器 DWB（覆盖文件 `controllers/dwb.yaml`＝nav2_overrides.yaml，yaw 公差回到它的值，不再是 RPP 那份的 0.005）；
  显式 `FPGA_NAV_COSTMAP=sim` 回到 10-15 前的 RPP heading precision＋`/fpga/points` 障碍层（真车无人发 /fpga/points）。其他值退 2。
- **相机帧名（10-15 已定）**：`LIVE_CAMERA_FRAME` 默认 `camera_color_optical_frame`——驱动 `depth_registration:=true align_mode:=HW`
  把深度对齐到彩色，深度图、彩色图、深度 CameraInfo 都带彩色光学系（板上 10-15 记录）；安装 TF、生产节点、运行器同用这一个帧。
  起流后 `camera_check.py` 照运行器的规则核一次（三话题帧名、尺寸、零畸变、编码），不符即 FAIL 并给原因（`camera_check.json`）。
  安装高度量的镜头：owner 10-15 答，彩色与深度镜头中心高度近似相同（手测，约 0.35–0.37 m，car.json 记 `height_uncertainty_m` 0.01）。
- **USB 重枚举**：起流阶段的重枚举（10-17 板上两次，设备号 +1）由挂整个 USB 根覆盖，驱动重试的下一次 try 拿到新节点（marks `camera driver usb try=N`）；栈起来之后运行中掉线仍不自动恢复，须 car_down 后重起。
- **推理服务**依赖 robot 的 `~/.local`（JetPack torch）与 pylib，须以 robot 身份跑；引擎文件存在与否 car_up 不预查（不在会以 4 回滚）。
- **深度探针**：`depth_probe.py` 未在板上跑过（rclpy 起节点在 Nano 上要 1–2 s，计在 20 s 内）；它只计帧数，深度全 0（近距激光保护）只记录不判。
- **KV 预检与生产节点先后两个会话**：预检连上即关，生产节点再连；板上 kv_server 是否接受这样的先后会话未在本条实测。
- **底盘串口** `/dev/fpga_chassis` 须先由 `tools/chassis_uart/nano_uart_setup.sh` 建好；只在 `--chassis uart` 时以 `--device` 挂进栈容器
  （台架 none 时没有这个设备，挂了 docker run 会整个失败）；容器以 `--user` 宿主 uid 跑，串口与 USB 节点权限未在板上核。
- **驱动起流参数**：`LIVE_LAUNCH_ARGS`（及话题、帧名、`CAMERA_CHECK_S`、`RUNNER_CAMERA_QOS`）全部可由环境覆盖，car_up 传进栈容器、
  `--dry-run` 打印；相机起流失败的根因修法（另一分支）若改 launch 参数，照覆盖即可。
- **驱动丢帧日志（10-17）**：实车（live）起链默认在 `LIVE_LAUNCH_ARGS` 末尾加 `enable_frame_drop_log:=true`（彩色帧率查因 10-16 的建议），
  事后 `driver.log` 里能看到 `Frame drop detected: stage=SDK_RECEIVE|ROS_PUBLISH`；板上镜像的 `gemini2.launch.py` 声明了该参数（默认 false，10-17 核）。
  `LIVE_FRAME_DROP_LOG=false` 关掉；`LIVE_LAUNCH_ARGS` 里已写 `enable_frame_drop_log:=` 时照原样不再加。回放/仿真不起驱动，不受影响。

## 线 4 台架（2026-10-15）

两板＋真相机 Gemini 2 按装车姿态固定在台架（镜头中心约 0.36 m、下倾 30°），**无底盘，车不会动**。组合
`live --pose static --chassis none --nav costmap`：`bench_pose.py` 发 map→odom→base_link 恒等（相机固定时这就是真位姿，
**不是建图定位**）和一张全空替身 `/map`（frame map，8 m 见方；运行器无 /map 一律拒查询，非 Nav2 路径没人发 /map），
car.json 记 `pose_note`、`bench.stand_in_map_note`。运行器读驱动话题（重映射、深度 CameraInfo、彩色光学系），台架用 hub 回环。

一键会话由服务器驱动（`bench_line4.sh`：自取 kv260 / orin-nano / camera 三把锁、部署 HEAD 提交、A2 比特流、每趟各起一个
kv_server、拉回、读数；EXIT 收尾恢复 m7_pl、按 token 放锁）。预注册：
`${FPGA_SRC}/runs/car-bench-20261015-results/PREREG.md`（含 10-15 追加节）。

```bash
W=${FPGA_SRC}/runs/car-bench-20261015-worktree
R=${FPGA_SRC}/runs/car-bench-20261015-results
bash $W/tools/car_run/bench_line4.sh run --dry-run          # 先看计划（什么也不碰）
# 正式：3 趟 fail-stop + 1 趟正常停；没量位置就不给 BENCH_OBJ（第 5 条记「未判」），SITE_NOTE 写现场缺什么
SITE_NOTE=现场无光照 FIND_TEXT=瓶子 nohup bash $W/tools/car_run/bench_line4.sh run > $R/run.nohup 2>&1 &
# 有光照、量好位置后，只补跑第 5 条：起一次 → 预热+120 s 窗口 → 电脑端听记忆 → 查询 → 正常停
FIND_ONLY=1 BENCH_OBJ="bottle <x> <y>" FIND_TEXT=瓶子 nohup bash $W/tools/car_run/bench_line4.sh run > $R/find.nohup 2>&1 &
```

- `BENCH_OBJ="<类> <x> <y>"`（米，base_link = map）：x = 镜头正下方地面点往前的距离 + 0.10 m（base_link 原点在镜头后 0.10 m），
  y 向左为正；类与 `FIND_TEXT` 一致（瓶子→bottle、杯子→cup、书包→backpack、手机→cell phone）。
- `CAR_HEIGHT_CELL=8` 由 `bench_line4.sh` 显式传（高度手测 0.35–0.37 m；8 在 0.34–0.38 m 都容得下地面，自动推导在 0.35 会给 7）。**台架专用**：实车起栈不给 `CAR_HEIGHT_CELL`，由地面自标定按拟合高度自动选格（owner 10-16，D-041；显式给定而拟合高度放不进该格位会拒绝启动）。
- rtabmap：没给 `RTABMAP_DB` 不尝试（car_up 会以 2 拒绝，预期），正式趟用 static；`NORMAL_STOP=0` 关正常停趟，`JETSON_CLOCKS_ARM=1`
  多跑一趟锁频（只报数）。
- 读数 `tools/car_run/bench_line4_readout.py <会话目录> [--expect "bottle x y"] [--site-note ...] [--cycles 3] [--normal-stops 1]`：
  每条 true / false / "not_judged"；退出码 0 全过、1 有一条败、3 无败但有未判；每趟另记窗口内运行器的检测数（光照旁证）。

## 实车行驶最小链：遥控＋轮式里程（2026-10-15）

板上一条命令（Nano 宿主，仓库根；`CAR_HUB_BIND` 换成车的局域网 IP）：

```bash
CAR_BOARD_DRAFT=1 CAR_HUB_BIND=<车IP> CAR_TELEOP=1 bash tools/car_run/car_up.sh live --pose odom --nav costmap
python3 tools/car_run/teleop_keys.py          # 另开终端（或电脑端 ssh -t <nano> 'python3 <仓库>/tools/car_run/teleop_keys.py'）
bash tools/car_run/car_down.sh                # 停
```

这一条就是「真底盘（uart）、不建图、不要 Nav2 目标」的一档（手册第一小时第 5 步地面遥控用它）：不需要定位库（`RTABMAP_DB`）、不起 Nav2。
`--pose static` 配 uart 仍以 2 拒绝：车会动，恒等位姿就是假位姿，而且会与轮式里程双发 `odom->base_link`。

- **卡顿时停在安全侧**（宿主实测 10-15，replay＋pty，0.1 m/s 行驶中用 SIGSTOP 冻结进程约 0.7 s）：冻结遥控节点 → 底盘驱动上游过期，
  在 STOP 发出后 343 ms（含 docker exec 约 100 ms）改发 enable=0（`upstream_stale`），假单片机执行速度回 0；冻结底盘驱动 → 单片机 200 ms 命令超时自行停轮。
  遥控节点自身按「现在 − 最后一次输入」判松手，节点或客户端卡住都只会更早发零，不会把旧速度延长；节点卡住时不再发布，下游 200 ms 过期兜底。
  恢复后若操作者仍按着键会继续走（输入是新的）；停顿期间排队的旧数据报（客户端带 `sent_ns`，同板单调时钟）超过 0.3 s 一律丢弃，不会重放已松开的键。手册第 5 步看的计数：`/fpga/chassis/status` 的丢帧/命令超时计数、`chassis_record.jsonl` 的 `upstream_stale` 行。
- **遥控**（`CAR_TELEOP=1`，`host/ros_teleop_safe.py`）：栈内唯一的 `/fpga/cmd_vel_safe` 发布者，20 Hz 恒发（空闲发零，与速度守卫一致）。
  限速默认 |v|≤0.15 m/s、|ω|≤0.6 rad/s（`CAR_TELEOP_V_MAX`/`CAR_TELEOP_W_MAX`，封顶 0.3 / 1.0，超封顶起不来）；超限输入被夹住，非有限值当零。
  **松手即停**：0.3 s 没有新输入就发零（封顶 0.5 s）。**单次指令最长** 3 s（`CAR_TELEOP_MAX_CMD_S`，封顶 10 s）：不松手连按超时即发零并锁住，松手（发零或 0.3 s 无输入）才解。
  退出（INT/TERM）先连发 0.5 s 零再关；底盘驱动另有 200 ms 上游过期置 enable=0。
- **它不经运动许可**：`/fpga/motion_permit` 要有已接受的 Nav2 目标，遥控时永远为假；遥控与速度守卫/许可并列，不串在许可后面。
  **单片机心跳锁存仍是兜底**（KV260 心跳断 → 单片机锁存停车，与本节点无关）；电脑端急停 `car_pc.sh estop` 照常锁 KV、进而锁单片机：板上它连 `car.json` 记的车 IP（栈容器是 host 网络；10-15 起，宿主以 127.0.0.2 实测过，板上未验）；另一台电脑用 `python3 pc/fpgapc.py --host <车IP> --port 47431 --token-file <板上 $RUN/token.txt 的拷贝> estop`。
- **互斥**：遥控配 sim（Gazebo 容器里速度守卫已在发）或 nav2（导航趟）由 car_validate 以 2 拒绝；运行中若发现 `/fpga/cmd_vel_safe` 还有别的发布者，
  节点发零并以 3 退出，fail-stop 拉下整栈。
- **记录**：`marks.txt` 起始行带 `teleop=1`，另有一行 `teleop: REMOTE CONTROL run -- NOT a navigation run`；运行目录有 `TELEOP` 文件；
  `car.json` 的 `teleop.enabled=true`；`teleop.jsonl` 记每个输入与每条发出的指令（时钟同 `chassis_record.jsonl`）。遥控趟不得当导航趟用。
- **输入方式**：节点在 IPC 目录（700，同 uid，无网络监听）开 Unix 数据报套接字 `teleop.sock`；键盘客户端 `teleop_keys.py`（w/s 前后、a/d 原地转、q/e 弧线、
  空格停、Ctrl-C/Esc 退出并发零）。电脑端走 `ssh -t` 进 Nano 跑同一个客户端；hub 协议没有速度通道，不改它。
  终端自动重复的首个间隔（约 0.25–0.66 s，看操作电脑设置）长于 0.3 s，按住键起步会顿一下；`--hold-s 0.4` 可平滑，但松手到停变成 0.7 s（0.15 m/s 下约 10 cm）。
  方向键首字节是 ESC，会被当成「退出」（发零后退出）；只用字母键。键盘模式未做交互实测（套接字路径由 `send` 模式验过）。脚本输入：`teleop_keys.py send V W --seconds S`。
- **轮式里程 TF**（`CAR_WHEEL_ODOM`，uart 时默认 1）：`host/ros_wheel_odom.py --source chassis`，live 时发 `odom->base_link`（/tf）和 `/odom`
  （可靠 QoS），另有 `/fpga/wheel_odom`、`wheel_odom.jsonl`。replay/sim（pty，`CAR_WHEEL_ODOM=1` 强开）只发影子话题 `/fpga/wheel_odom/tf`，
  因为那里 `replay_camera.py`/Gazebo 已发 `odom->base_link`。static 台架（bench_pose 发该边）只能配 chassis none，与轮式里程互斥。
  轮参数 `CAR_WHEEL_DIAM_M`、`CAR_COUNTS_PER_REV`、`CAR_TRACK_M`：板上（live）空则用**实车标定值** 0.081 m / 1560 / 0.394 m（麦轮等效轮距，
  10-16 真车标定，owner 批准，DECISIONS D-041）；宿主（sim/replay）空则仍用**占位**值 0.065 m / 1320 / 0.160 m，`marks.txt` 与 `car.json` 记
  `placeholder`。给了就照用（可只给一个）；板上 `--dry-run` 打印 `# wheel params: … -- source …`，`car.json` 的 `wheel_odom` 记 `params_used` 与 `source`。
  没选 `ros_odom_base_adapter.py`：它的状态把速度来源写死为 rtabmap、依赖 scipy（板上未用过）、`/odom` 发 best-effort。
- **`--pose odom`**：map->odom 恒等（起点即原点，`bench_pose.py --no-odom-base`）＋轮式里程的 odom->base_link＋全空替身 /map；运行器走 reference（TF）。
  只许 live＋uart；不是建图定位，是轮式里程推算（10-16 起板上默认实车标定轮参，D-041）。
- **`--pose odom --nav nav2`（10-17 放开，`docs/design/REAL_CAR_NAV_GOLIVE_20261016.md` §12）**：真车导航档。必须有建图导出的地图
  （`CAR_MAP_YAML=<rtabmap-export 的 .yaml>`，或 `RTABMAP_DB` 让 car_up 取 `<库目录>/run_<库名>/export/` 下唯一的 `*.yaml`），没有就以 2 拒（dry-run 也拒）；
  地图只给 Nav2 的 map_server（`FPGA_NAV_MAP_YAML`），bench_pose 只发 map->odom 恒等（`--map-m 0`，不发 /map）；车须从建图起点、同朝向起栈。
  仍须 `--chassis uart`；配遥控（`CAR_TELEOP=1`）或空跑开关（`CAR_NAV_DRYRUN=1`）照旧拒。急停取消导航、release 后须新目标、桥 `body_margin_m=0.05` 与 static 空跑档相同。
  真车第一次怎么跑（架空→落地）见该文 §12.4，owner 在场才跑。

## 物体记忆跨趟存载（2026-10-18）

详见 `docs/design/MEMORY_PERSIST_20261018.md`。

- 存：默认开（`CAR_MEMORY_SAVE=1`）。`car_down` 收车时运行器写 `RUN/memory_final.json`（完整件）；运行中记忆变了且距上次
  ≥ `CAR_MEMORY_SNAPSHOT_S`（5 s）写 `RUN/memory_snapshot.json`（快照，异常退出时留底；要载入先
  `python3 -m host.memory_store promote --memory RUN/memory_snapshot.json --out <新文件>`）。
- 载：`CAR_MEMORY_IN=<上趟 memory_final.json> CAR_MAP_YAML=<上趟导出> car_up.sh live --pose odom --nav nav2`。
  文件的地图身份必须与本趟一致（odom：导出旁 sha256.txt 指向的建图库 sha256；仿真：Nav2 地图 sha），
  坏文件 / 版本不对 / 缺字段 / 快照 / 地图不符 / 本趟无地图一律退出 2（dry-run 同样），不会降级为空记忆。
  查一个文件对某张图能不能载：`python3 -m host.memory_store check --memory F --map-yaml Y`。
- 载入后：界面的 `memory.scope` 是本趟新的（上趟的 select 被桥拒 `scope_changed`，按本趟重选）；物体 id 不变；
  上趟「在」的物体变「历史记忆」，上趟「已确认不在」的保持；`reason = loaded_prior_run`。
  find 时车照常开过去重新看一眼：看到了才算找到（同 id 变 current_confirmed），看空两次照常标「已确认不在」并转搜索。

## 已定事项（owner 2026-10-14）

- **fail-stop 为默认**：就绪后任一子进程退出即整栈停（理由：半条链在跑比直接停更危险也更难查；单片机 30 ms 心跳锁存是底层兜底，脚本层停下能留下原因）。非关键进程（如显示推送）的「只记录不停」例外留到需要时再加。
- **板上 hub 只监听车的局域网地址**并要求 token：板上必须设 `CAR_HUB_BIND=<车局域网 IP>`，空、`0.0.0.0`、`::` 一律拒绝（退出码 2）。板上 Nav2/rtabmap 正式入口与定位 TF 路由上板时再定。
- **宿主不把 hub 端口开到宿主上**：服务器多人共用，hub 只在栈容器回环，电脑端命令走 `car_pc.sh`。宿主仅用于联调与回归，不用于答辩。

## 验收读数（2026-10-25，`accept_readout.py`）

一趟跑完（或跑的过程中）读运行目录，8 节各给「通过 / 不过 / 无数据」及依据，末尾列阈值假设与待对齐字段名。**只读**运行目录，
JSON 不许写进运行目录（写了以 2 拒）。标准库、py3.8 可跑，Nano 上直接跑：

```bash
RUN=$(readlink -f /data/fpga/car/state/car.current)        # 当前（或刚收的）运行目录；宿主上给具体目录
python3 tools/car_run/accept_readout.py "$RUN" --json /tmp/accept_$(basename "$RUN").json
python3 tools/car_run/accept_readout.py "$RUN" --expect-nvpmodel <模式名>   # 给了才判功率模式；模式名以 sudo nvpmodel -q 的输出为准
```

各节数据源：1 版本与配置（head/dirty/marks/car.json/run_config；时限读 runtime.jsonl startup 的 `navigation_timeout_s`/`task_timeout_s`，
旧版本标无数据）；2 建图对齐（mapping_status.json 的 align；失败时从 reason 抽根节点修正量，≥0.3 m 标红「疑似 poses 为相机系」）；
3 寻物任务（runtime.jsonl task 行切任务，列查询文本 `goal`、终态、原因、用时、`attempts`、每段导航用时、fault）；4 导航配置
（navigation.log real_car 行的 rotation_shim / yaw_goal_tolerance；仿真档、旧版本分别标无数据）；5 深度（frames.jsonl 连续
`dropped:no_valid_depth` ≥3 s 判不过）；6 识别丢帧比；7 负载；8 1.5 cm 三项（体素比对耗时、快照间隔、深度丢帧）。

**第 7、8 节没有自动落盘，验收现场手动存到 `$RUN/accept/`**（文件名照抄，工具按名找；也认运行目录根下的 `tegrastats*`/`pidstat*`/`nvpmodel*`）。
栈 READY 后在 Nano 上另开终端：

```bash
RUN=$(readlink -f /data/fpga/car/state/car.current); mkdir -p "$RUN/accept"
sudo nvpmodel -q > "$RUN/accept/nvpmodel.txt" 2>&1                                  # 功率模式
tegrastats --interval 1000 > "$RUN/accept/tegrastats.txt" &                          # 负载，每秒一行（%1）
pidstat -u 5 > "$RUN/accept/pidstat.txt" &                                           # %2，各进程 CPU（需 sysstat；没有就 top -b -d 5 > $RUN/accept/top.txt，只存不判）
# 建图趟：体素比对耗时（live_mapping /status 的 voxel.ms），每 5 s 采一次，5 分钟后自己结束（%3）
for i in $(seq 60); do curl -s http://127.0.0.1:47802/status; echo; sleep 5; done > "$RUN/accept/live_mapping_status.jsonl" &
# 控制台快照间隔：在 Nano 本机连网关看 60 s（路径同 car_up 用的部署目录）
python3 ui/geomem/tools/competition/ws_probe.py 60 > "$RUN/accept/ws_probe.txt"
```

收车（car_down）**之后**，网关退出时才写桥的统计，再拷进来：

```bash
cp /data/fpga/car/gateway/bridge-stats-$(basename "$RUN").json "$RUN/accept/"
kill %1 %2 %3 2>/dev/null      # 同一终端里；换了终端就 kill 对应 pid
python3 tools/car_run/accept_readout.py "$RUN"
```

两个小脚本（同样只读）：

```bash
# 陀螺 3 圈：原地转 3 圈后读，陀螺/轮速转角对真值 6π 的比值；--t0/--t1 截转圈那段（相对首行秒数）
python3 tools/odom_gyro/turn_check.py "$RUN" --turns 3 [--t0 20 --t1 60]
# 书包门槛：列 memory_final.json（无则 memory_snapshot.json）里 backpack 的分数；给真假标注求能分开的门槛区间
python3 tools/car_run/backpack_gate.py "$RUN" --gate 0.45 --truth object-0001=1,object-0003=0
```
turn_check 优先用 wheel_odom.jsonl 的 0x82 融合行（gyro_yaw 与 yaw）；没有就回退 chassis_record.jsonl（0x81 的 gyro_z_cdps，
16 位 mcu_ms 回绕已展开；0x82 的 gz_sum/gz_n）。backpack_gate 的分数只覆盖每条 track 最后 20 条 history（memory_store HISTORY_KEEP）。
测试：`tests/test_accept_readout_20261025.py`。

## 上车工具：一键部署与只读体检（2026-10-25）

两个脚本都在**服务器**上跑，经 `ssh nano`（校园反向隧道；车载路由下可 `--host robot@198.18.31.59`）连 Nano。

```bash
# 先看要做什么（不连 Nano，只打印每条命令）
bash tools/car_run/deploy_nano.sh --ref origin/main --dry-run
bash tools/car_run/deploy_nano.sh --ref origin/main
# 部署并只重启网关（车栈不动）/ 重启整个车栈（console_remote.sh down 再 up，最长约 6 分钟）
bash tools/car_run/deploy_nano.sh --ref origin/main --restart gateway
bash tools/car_run/deploy_nano.sh --ref origin/main --restart stack
# 只读体检，期望 console-current 等于 origin/main；退出码 = FAIL 项数
bash tools/car_run/board_preflight.sh --ref origin/main
```

- 部署：解析 ref 为完整提交号 → Nano 上已有 `console-direct-<sha7>` 且 `COMMIT` 一致就跳过传输 → 否则 `git archive`（整棵树）
  经 ssh 解到临时目录、写 `COMMIT`（完整提交号一行，`car_up.sh` 靠它做源码快照，缺了退出码 8）、`mv -T` 改名 →
  校验 COMMIT 与关键脚本 → 原子切软链（打印回滚命令）→ 按 `--restart` 执行 → `console_remote.sh status`。
  任何一步失败即停，失败时软链不动。退出码：4 Nano 不可达、5 同名目录 COMMIT 不符（不覆盖）、6 传输失败、7 校验失败、
  8 切软链失败、9 重启失败。`--restart none` 时正在跑的车栈和网关仍是旧代码。ref 用本地已有的，需要最新先 `git fetch origin`。
- 体检（不改状态；唯一例外是 §4b 的 `docker run --rm` 临时容器）逐项 PASS/FAIL/INFO：Nano 可达与主机名；
  `console-current` 目录名与 `COMMIT` 是否等于期望提交；镜像 `fpga-ros:humble-nav-orbbec-scipy-20261017` 在且 packages.tsv 含
  rotation-shim（没有就起车栈前设 `FPGA_NAV_ROTATION_SHIM=false`）；`nvpmodel -q`（默认期望 MAXN_SUPER，`--expect-nvpmodel 25W`
  可改）；`jetson_clocks --show` 频率（INFO）；`/data` 与 `/` 剩余空间（`--min-free-gb`，默认 5）；KV260 ping 与
  `geomem-kv.conf` ssh（PASS/FAIL），KV 服务 HELLO（INFO，车栈未起时不应答是正常的）；车栈状态（`check_current.py`，INFO）；
  Orbbec 相机在 USB。Nano 不可达时只报这一项、退出码 1。
- 测试：`tests/test_board_tools_20261025.py`（PATH 前置假 ssh/docker/nvpmodel 等，临时目录当 Nano）。
