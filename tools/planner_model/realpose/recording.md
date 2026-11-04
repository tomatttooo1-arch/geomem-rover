# A：真实位姿录制清单与回放


## 下次 owner 在场录什么

先按仓库流程取得设备锁、确认实际运行目录和ROS环境，再由获授权现场人员录制。本文件不是自动启动设备的脚本。

- 第一段3–5分钟检查完整性；正式采集分多趟覆盖直路、转弯、窄门、家具、未知边界和窗口外目标，保留正常操作产生的实际请求，不人为重复查询凑200对。少于200对照报。
- 必录：`/tf`、`/tf_static`、`/fpga/rolling_map/grid`、`/local_costmap/costmap_raw`、`/global_costmap/costmap_raw`、`/plan`、`/odom`、`/clock`（使用仿真时间时）。优先raw代价，OccupancyGrid编码不能直接当uint8代价。
- 目标：录`/goal_pose`（若请求真的经过它）或已有的明确请求镜像。Humble的action目标经服务传输，**ros2 bag record话题并不能自动录下ComputePathToPose/NavigateToPose的请求体**；`_action/status`只含状态不能恢复目标。未捕获请求时只能使用`/plan`结果代理，去重并单列，不能声称已有真实请求捕获。
- 保存实际Nav2参数（插件、膨胀、footprint、尺寸/分辨率、frame、allow_unknown、tolerance）以及`ros2 topic info -v`输出。时间需要ROS消息header，不能用主机CLOCK_MONOTONIC_RAW日志直接替代ROS时间。

现场授权后，在**实际车ROS环境**使用下面的命令；不要在当前会话执行：

```bash
ros2 bag record --include-hidden-topics \
  --qos-profile-overrides-path tools/planner_model/realpose/record_qos.yaml \
  -o "$RUN/planner_realpose/bag" \
  /tf /tf_static /odom /clock /fpga/rolling_map/grid \
  /local_costmap/costmap_raw /global_costmap/costmap_raw /plan \
```

停止用Ctrl-C，等metadata.yaml写完；用`ros2 bag info`核查计数和时间。TF至少覆盖请求前后100ms，地图更新应覆盖请求前200ms。话题缺失、地图不为80×80或分辨率不为0.05m直接记录，不改图凑条件。
若只有`/fpga/rolling_map/grid`，还需完整固定膨胀参数，使用`--inflation-params`明确重建并单列；只有增量、没有完整图时暂不支持重建，报告缺口，不猜基图。

## 一条命令直接读包比较

在Humble环境（或宿主隔离镜像）先构建C++库：

```bash
cmake -S tools/planner_model -B runs/planner-realpose-build -DCMAKE_BUILD_TYPE=Release
cmake --build runs/planner-realpose-build -j2
python3 -m tools.planner_model.realpose.replay \
  --bag <录包目录> --source-kind real \
  --lib runs/planner-realpose-build/libplanner_model.so \
  --navfn-lib runs/planner-realpose-build/libplanner_realpose_navfn.so \
  --out runs/planner-realpose-replay-<新编号>
```

回放目录必须不存在。只读原包；先保存原始文件SHA256，再解码；不在线发请求、不调用生产服务。缺/stale数据写入rejected.jsonl，不据此调参。`--source-kind`必须如实填写，宿主结果用simulation，不能标real。

结果在comparison/summary.json、pairs.jsonl、paths.jsonl、rejected.jsonl；extraction保存topic清单、解码错误、原始时间、TF和输入哈希。实际TF链采用有界插值，不外推；跨ROS时间回退分epoch。

外部目标只由关联全局路径与有效窗口[1,78]²的首次离开交点决定，同时保存物理窗口交点（若路径确实离开物理窗）；无关联路径拒绝，不能直线猜目标。只有标准完整raw Costmap得到实际验证；增量合成和真实日志适配仍属于后续输入扩展。

## 宿主链路验证


运行使用独立容器网络`--network none`、ROS_DOMAIN_ID=173、ROS_LOCALHOST_ONLY=1，无设备/GPU/host网络，不运行car_up。镜像入口脚本不可执行时显式用`--entrypoint /bin/bash`并source `/opt/ros/humble/setup.bash`；无需修改镜像权限。

补充坐标口径：两方共同采用固定Humble的navfn_round代表点约定；额外保存ROS元数据真实物理窗[0,W]×[0,H]交点。有效局部目标窗和物理窗不同，几何检查不是车体/地面真值碰撞验收。该口径在真实合格样本仍为0时已追加到预注册。
