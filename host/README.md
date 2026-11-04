# host/ —— 处理系统（PS）侧程序

跑在板子 ARM 侧的代码：应用主程序、通过 XRT 调用 PL 加速核的会话与调度层、ROS 节点、
与相机 / 底盘 / IMU 打交道的驱动封装。

边界：凡是"跑在 CPU 上、负责把数据喂给硬件或把结果接出来"的都在这里；
纯算法（不依赖板子就能跑的那部分）放 `perception/`；PL 里的逻辑放 `hw/`。

**不设 `src/`** —— 它和 `host/` 语义重叠，一定会放错。
# 已有入口：离线回放

`replay.py` 是主机侧 CPU 起步入口：`python -m host.replay --help`。
支持受控案例、TUM RGB-D 目录和规范 JSONL 输入；输出状态记录、查询候选与证据图。
当前未部署到板上 ARM，不依赖 ROS，也不驱动底盘。使用说明见 `docs/design/REPLAY_START.md`。

# M2(b) 定位交付：门节点与里程计适配器

- `localization_gate.py` —— 纯 Python 的失效即关闭定位门禁（不碰 ROS，也不看真值）：只接受与钉死
  先验地图连通的采集，三次健康采集才进 `valid`，任何丢失/超时/重启都抬 epoch 撤销在途消费者。
  同文件内 `MapOdomHold`（`map→odom` 持续重发的纯逻辑，时钟注入）与 `LocalizationLease`（消费者租约）。
- `ros_localization_gate.py` —— 它的 ROS 适配层：`python -m host.ros_localization_gate --database DB
  --map-id SHA256`。WP1 新增开关：`--world-frame/--odom-frame`（打通原先硬编码的 `map`/`odom`）、
  `--bundle`（读 map bundle 的 `bundle.json` 定 `navigation_ready`，缺省 false）、`--odom-source`、
  `--publish-tracked-frame/--no-publish-tracked-frame`（联调时关掉，避免 `camera_link` 双父）、
  `--map-odom-hold-hz` 与 `--map-odom-future-s`（都为 0/默认时保持原行为）。status 只增字段，`schema` 仍是 1。
  10-15 新增（owner 定，挖因见 `runs/real-car-integ-20261015-results/gate_dig/DIG.md`）：
  - `--count-mode correction`（**10-15 晚起节点默认**，owner 定方案二；评估见上述 `gate_dig/RELAX_EVAL.md`）：批准的是
    rtabmap 输出隐含的 `map→odom` 修正（同戳 info＋localization_pose＋odom＋odom_info 四件健康），不再逐帧批位姿；
    不订任何相机话题（`--acquisition-topic`/`--rgb-reliable` 不适用）。进入：连续 3 次（ref_id 逐次 +1）链到先验、
    且相邻两次差不超跳变门（追踪点 10 cm／5°，与 `nav_localization` D-035 同函数同阈值）；跳号/不一致/候选龄 > X 只重数、
    不换代；入场期未链输出不计数也不清零（10-16 第 7 棒，owner 授权：X 窗内 3 条互相一致的链上输出，中间可夹未链）；例外：跳变撤销后以被拒那条起头的一轮，中途遇未链即按旧规则清零重数（`entry_reset:unlinked`，10-16 第 8 棒，owner 定）。可用期：门内的链先验修正刷新所持修正；超跳变门 → `correction_jump` 撤销换代、该条当新一轮第 1 次；
    未链或偶丢一条输出不刷新也不撤销。每对健康 odom＋odom_info 合成一项批准（所持修正 × 里程计，戳 = 里程计戳），
    进入可用那一刻立即用最新一对合成一项。撤销：所持修正龄 > X（`--max-correction-age-s`，默认 2 s，分档见
    `localization_gate.py` 的 `COUNT_MODES` 注释）→ `correction_too_old`；最新健康里程计对在任一时钟上 > 1 s →
    `odometry_stale`；里程计报丢/坏值立即；rtabmap 输出 0.5 s 内没凑齐 → `incomplete_output`。
    里程计跳变（10-17 第 10 棒，设计见 `runs/real-car-integ-20261015-results/gate_dig/HOST_G5_RESULT.md` 第 9 棒）：每来一个新的
    健康里程计对，与门留着的上一对比（任何状态都比；启动、时钟回退、回放不连续之后的第一对没有上一对，不查），平面位移
    > V·Δt + E（Δt 取两对的戳差）即判里程计自己跳了：可用期 → `odometry_jump` 撤销换代（该对不再合成批准）；入场期有候选 →
    清空（`entry_reset:odometry_jump`）。V = `--max-odom-speed-mps`（默认 0.5 m/s，按平台调），E = `--odom-jump-margin-m`
    （默认 0.10 m，= 修正跳变门 10 cm）；`--no-odom-jump-check` 关掉，与 4edc10f（第 8 棒）逐字相同。已知局限：物理可达范围内
    的来回跳（如宿主 g8r_3 14.50 s 那步 0.195 m / 0.336 s，界 0.268 m）不触发。运行器侧这两条 reason 归里程计类故障（同
    `odometry_stale`），不跳帧。**status 语义**：
    `stamp_ns` = 历次合成批准的最大戳（同 instance 内单调），`correction_stamp_ns` = 本行最新合成批准戳；
    `received_*` events 行有修正判定时多一个 `corrections` 字段（戳、ref_id、判定、相对所持/上一候选的平移与转角）。
    运行器侧：租约已越过而未批准的帧直接跳过（`LocalizationLease.superseded`，`host/ros_runtime.py` 计
    `localization_skipped`），不再等 0.5 s 后报 `acquisition_clock_or_tf_wait_expired`。
    排在被卡住的帧后面、一次没试就在队列里到期的帧（宿主 g4r_re：首个获批帧在运行器里卡 1.3 s），若最近一条 status
    被租约接受且其 (state, reason) 属「无定位」三类（10-15 第 5 棒续起**按 reason 判**，白名单
    `LOCALIZATION_SKIP_REASONS`）：「等待」（valid 且健康、无本帧批准）、「未就绪」（initializing/confirming，非时钟回退/
    回放不连续）、「无修正」（stale:correction_too_old、lost:correction_jump；10-16 第 5 棒收口起含 lost:incomplete_output、
    stale:localization_stream_timeout——板上 rtabmap 满载 0.3–1.3 Hz，输出断续是常态，兜底靠租约 1 s 与 2 s 无处理帧），且时钟与 base←camera、tracked←camera
    外参正常、本帧没等过时钟/TF，也按跳帧计（stall-diag 原因 `expired_before_localization`，计入 `localization_skipped`）；
    里程计类（odometry_stale、odom/odom_info 报丢或坏）、时钟类（clock_rollback、replay_discontinuity、future_stamp）、
    裁定未点名的其余 reason（incomplete_acquisition_timeout、rtabmap_output_gap 等，表见 `host/ros_runtime.py` 注释）、外参缺、时钟未到仍走原
    `wait_expired` 分支（`Runtime.expired_only_for_localization`，10-15 第 5 棒）。同一白名单下，已获批的帧在运行器里
    卡成过旧（>1 s）也按跳帧计（stall-diag `stale_after_approval`），不报 `sensor_or_pose_invalid`（第 5 棒续）。
  - `--count-mode output|frame`（10-15 白天的节点默认是 output）：「连续 3 次健康」改数 rtabmap 实际处理的连续输出
    ——rtabmap 没处理的相机帧跳过不清零（但带坏输出的跳帧照样撤销）；输出先于采集消息到达时暂存（上限 32、
    同 0.5 s/1 s 时限，过期/溢出照样撤销）；相邻两次计数的 `Info.ref_id` 必须恰好 +1，否则 `rtabmap_output_gap`
    清零。其余条件（同戳四件健康、info 链到先验图、N=3、全部时限）原样。`frame` 是旧行为，只留作对照
    （板上相机 30 fps、odom 约 10 Hz，旧行为结构上不可能 valid）。纯 Python 类 `LocalizationGate` 的
    `count_mode` 缺省仍是 `frame`：既有回放工具/契约测试按旧语义构造，切换只在节点层。
    `pending_overflow`（采集队列或暂存溢出）次数记在每条 events 行的 `pending_overflows`。
  - `--acquisition-topic rgb|camera_info`（默认 rgb）：采集戳取自 `PREFIX/rgb`（彩色图）或 `PREFIX/camera_info`
    （彩色 CameraInfo，小消息、门收得早）。**前提：彩色 camera_info 与彩色图同戳**——宿主 `tum_rgbd_pub
    --stamp-now` 满足；车上 Orbbec 未核，须上板录 100 对逐帧比戳、全等才切，未核前车上默认不切。
    remap 时接**彩色** camera_info，不是 rtabmap 用的深度 camera_info。`--rgb-reliable` 作用于所选话题。
- `odom_base_adapter.py` —— 纯数值核：把 RTAB-Map 相机系里程计换算到 base_link
  （`T_odom_base = T_odom_cam · inv(T_base_cam)`，twist 与两个协方差按伴随变换），并过滤 lost 帧。
- `ros_odom_base_adapter.py` —— 它的 ROS 适配层，是交付里 `odom→base_link` 与 `/odom` 的**唯一**持续
  发布者；来源自报 `estimated`（视觉里程计，M2(a) 的轮速/IMU 到位后按同一接口替换）。

这几个文件里真正跑在 ROS 里的部分本机无法执行（无 ROS 2）：主机侧单测只用源码检查钉住接线，
运行时行为要靠容器内的端到端排练证明。测试跑法见 `tests/README.md`。

# 车上链路运行规程（2026-10-11 深夜，整链第 3 版，控制器定）

适用：车上 Nano 的 ROS 2 进程（`host/ros_geo_producer.py` 生产节点、`host/ros_rolling_nav.py` 桥、运行器、hub、Nav2）。
背景：宿主与板上实测，运行中有新的 ROS 进程（DDS 参与者）加入或退出后约 1–3 s，生产节点的 `publish` 偶发在 DDS 底层阻塞
约 230–320 ms 且不放 GIL，整个进程冻住 → 许可到期、桥拒收过期增量并重同步（安全侧，但这一帧丢）。详见
`docs/design/INTEG_CAR_CHAIN_20261011.md`「第 3 版：发布卡顿」。

1. **运行期间不起新的 ROS 进程、不跑 ROS 命令行**：`ros2 topic echo/list/hz`、`ros2 node list`、`rqt`、`rviz2`、临时脚本里
   `rclpy.init()` 都算（启动和退出都会触发）。要看的东西在**起车前**就起好，跟车上节点一起起、一起停；电脑端只走 hub 的 TCP
   （`pc/fpgapc.py`），不加入 DDS。
2. 测量探针（`tools/car_geo_producer/board_probe.py`）在预热阶段就加入（`rig_inner.sh` 默认 `PROBE_JOIN=early`，
   `--start-at-wall` 到点才开始记），测量窗内没有新参与者加入。
3. 生产节点的 DDS 发布模式开关 `--dds-publish-mode env|sync|async`（只设本进程的 `RMW_FASTRTPS_PUBLICATION_MODE`，在
   `rclpy.init` 之前；不改系统、不影响别的进程）。注意：只给 Fast DDS XML 写 `publishMode` 而不设这个变量**不生效**
   （Humble 的 rmw 默认 SYNCHRONOUS 会覆盖它，宿主实测）。默认值与 A/B 结果见上面那份文档。
4. 200 ms 规则不因卡顿放宽：卡顿那一帧过期、许可到期、桥重同步都是设计内的安全侧行为。
