# M8 WebSocket v1 前端接入契约


这是新增前端所期望的协议提案，并非已有 ROS 接口。本仓库只交付客户端与演示源；团队仍需实现桥接服务。首次连接应立即发送完整快照，后续建议每秒 1–5 帧，必须有心跳式快照更新；浏览器不直连底盘、不发送速度。

## 坐标与时间

统一 `map` 坐标系，右手系：x/y 为水平面，z 向上；单位米，yaw 单位弧度，从 +x 朝 +y 逆时针。所有点云、物体、位姿与路径必须转换到同一个 map 坐标系。前端转换为 Three.js `(x,z,-y)`。不得将 M3 的图像 frame_id 哈希作为坐标系，不得用二维 bbox 填三维位置。

timestamp 与 updatedAt / lastSeen 使用 ISO 8601 含时区时间，推荐 UTC。snapshot.timestamp 应表示本次组合数据的有效状态时间，而不是把过期传感器数据重新标上发送时间。服务必须保证位姿、路径、对象语义时效正确；缺失位姿用 null。时间超过浏览器 5 秒或超前超过 2 秒将被判为过期，部署必须校时。

地图 id 变更表示地图切换或修订。物体 id 是 M4 的长期身份，不是 M3 单帧 instance_id。若地图与正在执行任务不一致，桥接服务必须明确终止/拒绝任务或给出经过转换的新状态，不能默默复用旧坐标。

## 服务 → 浏览器：完整快照

```json
{
  "type": "snapshot", "version": 1, "seq": 1,
  "timestamp": "2026-10-14T04:00:00.000Z",
  "map": {"id":"lab-v1", "name":"实验室", "frame":"map", "units":"m"},
  "robot": {"position":[1.0,2.0,0.0],"yaw":1.57,"speed":0.0,"localized":true,"status":"待命"},
  "objects": [{"id":"bag-01","name":"书包","className":"backpack","aliases":["背包"],"location":"书桌旁","position":[2.0,3.0,0.5],"confidence":0.87,"lastSeen":"2026-10-14T03:58:00.000Z"}],
  "points": [[1.0,2.0,0.0,0.5,0.7,0.4]],
  "path": [],
  "task": null
}
```

- seq：当前连接内单调递增的非负安全整数；旧帧、重复帧被丢弃。服务重启序号归零时需要重建连接。
- robot：可为 null；position `[x,y,z]`，yaw 弧度，speed 为非负速度大小，localized 只有可靠定位时才为 true。没有电量数据就不显示猜测电量。
- objects：完整数组，最多 1,000 条；position 为世界坐标中的物体参考点，不能直接充当停靠点。confidence 为模型分数 0–1，不是校准概率；lastSeen 未知时为 null。缺少位置的二维检测不进入此数组。
- points：最多 60,000 个 `[x,y,z,r,g,b]`，颜色 0–1；缺少几何时发送空数组，前端不会补入演示房间。示意家具只在 demo 模式显示。
- path：最多 5,000 个 `[x,y,z]`；必须来自 M6 的实际规划，不由前端编造。没有路径时空数组。
- 单帧文本最多 12,000,000 字符；不接受二进制、NaN、无穷、错误版本、错误坐标系。无效帧不刷新新鲜度。
- task：无任务为 null，否则如下；成功后建议保留终态直到下一任务，便于重连客户端看到结果。

```json
{"id":"task-unique-id","targetId":"bag-01","state":"confirming","message":"已到停靠点，正在观察目标","updatedAt":"2026-10-14T04:00:12.000Z"}
```

状态枚举：idle / searching / planning / navigating / confirming / paused / succeeded / failed / cancelled。仅有 succeeded 表示 M5 已实际完成现场确认；到达历史坐标不能直接发 succeeded。失败 message 应给出未知目标、不可达、物体移位、观测失败等用户可理解原因。前端收到状态后显示，不自行推导实车成功。

## 浏览器 → 服务：请求

```json
{"type":"command","version":1,"id":"request-uuid","action":"find","payload":{"targetId":"bag-01","mapId":"lab-v1"},"timestamp":"2026-10-14T04:00:00.000Z"}
```

find 按指定物体实例发起任务；服务再次核验地图、实例、定位、可达停靠点和任务互斥。前端查询仅在已收取物体数组中进行类别/别名匹配，不把本地模糊匹配当作导航成功。

pause / resume / cancel：payload 为 `{"taskId":"task-unique-id"}`。服务必须核验 taskId，并把操作落实到 M5/M6。暂停期间须确实停止运动，恢复必须重新评估导航条件。取消请求不等同于硬件急停。

服务按 request id 去重，避免网络重放或多客户端重复执行；也必须实现跨客户端任务互斥。浏览器不会自动重试命令，也不自动重连后重新发任务。

## 服务 → 浏览器：回执

```json
{"type":"ack","version":1,"id":"request-uuid","accepted":true}
```

拒绝：`{"type":"ack","version":1,"id":"request-uuid","accepted":false,"reason":"当前有另一个活动任务"}`。

accepted 只表示请求受理，必须随后发实际 task 状态快照。浏览器等待回执最多 5 秒；超时/断线显示执行结果未知，不显示已停止。受理后如果 10 秒内没有对应状态，显示未收到状态，操作员应核对实车。取消在连接仍打开但遥测过期时仍允许请求；服务器对安全负责。

## 部署与安全边界

浏览器 WebSocket 不支持任意认证头。生产部署建议同源 WSS 网关与安全会话 Cookie，校验 Origin 和操作权限。不在 URL、日志、页面或 localStorage 中放永久令牌。当前前端仅保存服务器地址与最近终态记录，不存密钥。

静态服务仅用于本机开发；未包含认证、ROS 运行时、文件上传、数据库访问或控制代理。部署者必须完成这些服务端约束和真实设备验收，不能以演示测试代替。

## 服务 → 浏览器：搜索热度图 `belief`（10-26，第二版协议新增文本消息）

> 本节属第二版协议（本文件其余为 v1 记录）。旧页面不认识 `type:"belief"`，按第二版「其余类型静默忽略」丢弃，不影响别的消息。

链路：runner `/fpga/search/belief`（std_msgs/String JSON，latched，`host/search_belief_feed.py` 定时机）→ hub 订阅
（`host/hub/hub_ros.py`，`hub_core.parse_belief`）→ FPGAC `MAP` 帧 `which:"belief"`（int8+zlib 格，显示字段在
meta `belief`，≤4 Hz、只留最新，`hub_server.LATEST_ORDER` 槽 `MAP:belief`）→ 桥（`gmb_proto.compose_belief`）组成下面这条
文本消息，收到即推给所有页面，只留最新；新页面接上时补发最近一条（`ageMs` 加上桥里放的时间）。

```json
{"type": "belief", "version": 2, "phase": "search", "task": 3, "seq": 41, "frame": "map",
 "resolution": 0.2, "width": 50, "height": 40, "origin": [-0.5, -4.0, 0.0],
 "cells": "<base64：width×height 个 int8，行优先>", "next": [2.0, 0.5, 1.5708], "seen": [[1.0, 0.0, 0.0]],
 "massLeft": 0.62, "result": null, "reason": null, "ageMs": 35.0}
```

- **格子约定同 `nav_msgs/OccupancyGrid`**：第 0 行第 0 列格子的**左下角**在 `origin` [x, y, yaw]（map 系，米/弧度）；
  列沿 +x、行沿 +y（再整体按 yaw 转）；格 (行 r, 列 c) 中心 = origin + R(yaw)·((c+0.5)·res, (r+0.5)·res)。界面照第二版惯例
  map (x, y, z) → Three.js (x, z, −y)（`operator/src/belief-layer.js`）。
- `cells` 值：0..100 = 该处可能性，**相对本任务先验的最大值**（看过的地方只降不升、渐淡；没看的保持原色）；−1 = 不在搜索范围（没建过图/看不到的地方）。
- `phase`：`preview`（有记忆目标、出发去原位置前，「万一不在原处」的先验，每任务一次；`next`/`seen` 空）→
  `search`（开始搜索后，选出新看点时立即、否则 ≤1 Hz 且只在变化时；`next` 为正驶向的看点，`seen` 为已看点）→
  `done`（任务终态一次，`result` 为 runner 终态词 found/not_found/unable_to_confirm/cancelled/failed，`reason` 原因；
  若尚未开始搜索则 width=height=0，只表示「结束」）；`none`：新任务没有概率搜索（无地图/无稠密轨迹节点/退回 MVP 列表）或中途退回，
  width=height=0，页面清图。无记忆直接搜索时第一条就是 `search`。
- 页面（`operator` 控制台 `#remote` 三维地图）：地面上 3 cm 一层半透明格（越可能越红越不透明，看过的淡黄近透明），下一个看点画红圈加箭头，
  已看点白点；`preview` 整体更淡，左上角小字「若不在原处，将这样找」；`search`「搜索中 · 越红越可能」；`done` 显示结果词，3 s 后 1.5 s 淡出隐藏；
  `ageMs` > 8 s 的 `done`（新页面接上时补发的旧图）直接不画。断开重连、`none` 清图。
- 大小：10×8 m 场地 50×40 格，消息约 3 KB；runner 每条构建＋序列化 < 1 ms（宿主）。

## 浏览器 → 服务：重置地图 `map_reset`（10-26，第二版协议新增命令）

> 本节属第二版协议。owner 定：控制台点「重置地图」→ 二次确认「将清空地图和记忆，车会停下」→ **不停栈**清空三维建图和物体记忆
> → 页面显示「已重置，可以重新建图」。只支持建图模式；定位模式（使用已存地图）按钮置灰并给中文原因。

请求（须显式确认，同 `release`）：

```json
{"type": "command", "version": 2, "id": "request-uuid", "action": "map_reset", "payload": {"confirm": true}}
```

能力表 `capabilities.mapReset: true`（桥 `gmb_proto.make_hello`）；旧页面不认识，按「不认识的能力一律当 false」不显示。

**链路**：页面（`operator/src/map-reset.js` + `remote.js`；`app/src/map-reset.js` + `competition.js` 同一份）→ 桥
`gmb_proto._classify_motion`（缺 `confirm:true` 拒 `confirm_required`，链路不通拒 `link_down`，否则转发；ack 期限 3 s、结果等 35 s）
→ FPGAC → hub op `map_reset`（`hub_core`：先拒，见下表；通过则发 `/fpga/map_reset` String JSON `{"id": hub 命令 id}`，
监视 33 s）→ **runner**（`host/ros_runtime.py` + `host/map_reset.py`，按序执行，每步计时写 runtime.jsonl `map_reset_step`）：

| 步 | 做什么 |
|---|---|
| stop | 走现有停车路径：撤销任务/巡航/goto/末端接近，/cmd_vel 零、permit false、Nav2 目标撤销 |
| memory | 新 Memory；**新 runtime_boot_id**（＝页面 `memory.scope`，候选/选择随之失效）；`/fpga/memory`、`/fpga/runtime_info` 重发 |
| caches | 搜索航点/稠密节点、搜索规划器及其缓存、观测视角、记忆对齐挂起；本次对齐键 `reset:<id>` 记为已处理 |
| mapping | 发 `/fpga/mapping/reset` → `tools/car_run/mapping_ctl.py`：建图中调 `/rtabmap/reset`（约 2.4 s）；已导出/失败后则重开建图进程（换 run id）；导出后换过图的，map→odom 回单位阵、Nav2 回起步空白图；`/operator_map/map_assembler/reset` 清三维拼图；完成后 state 回 `mapping`、`session`+1 |
| costmaps | Nav2 ClearEntireCostmap（全局＋局部） |
| display | `/fpga/search/belief` 发 `phase:"none"`（热力图清空，见上节） |

重置期间物体记忆不写入；重置结束前采集的帧也不写入（旧地图坐标系下的观测不进新记忆）。重置中 find/select/goto/巡航一律拒（同换图门）。
runner 在 `/fpga/map_reset/status`（String JSON，latched）回 `{"id", "state": "done"|"failed", "steps": [...], "reason_zh", "scope", "previous_scope", "mapping_session", "elapsed_s"}`；
hub 据此回 CMD_RESULT：`done`，或 `rejected` + `map_reset_failed` + `detail`（runner 的中文：哪一步、为什么）；超时为 `unknown`。
一步失败不回滚，后面的步照做并如实报告。

**三维显示**：`ui/geomem/tools/competition/live_mapping.py` 订 `/fpga/runtime_info`（scope 跟随 runtime_boot_id，`/status` 里
`scopeSource:"topic"`）与 `/fpga/mapping/status`（进入 `resetting` 或 `session` 变化即清体素：新纪元、计数清零；重置期间与
时间戳早于清空时刻的拼图云丢弃）。桥在 `map_reset` 结果 done/rejected 时清缓存云与各页体素基线；新图第一批体素以新纪元全量推送。

**页面**（`/#remote`「结束建图」旁）：可用条件＝已连接且新鲜、本机持有控制权、`snapshot.mapping.state` ∈ mapping / exported（已载入）/ failed；
**急停锁存时也允许**（车本来不动，hub 同样放行）。`snapshot.mapping` 为空（hub 没有建图状态）＝定位模式：按钮置灰，提示
「定位模式（使用已存地图）下不能重置地图；要重新建图请用建图模式起栈」——定位模式没有建图会话可重开，清空已存地图的记忆也不是这个按钮承诺的事。
保存中（saving / exported 未载入）、重置中置灰并说明。进度文案「正在重置：停车、清空记忆和地图…」（结果最多等 40 s）；
done → 清空页面上的三维体素/点云/底图、轨迹、物体标记与记忆卡片、候选、热力图，显示「已重置，可以重新建图」；
rejected/unknown → 不清页面，显示原因（以车上为准）。

拒绝码（桥/hub）：

| reasonCode | 来源 | 中文 |
|---|---|---|
| `confirm_required` | 桥 | 重置地图会清空地图和记忆，请求里须带 {confirm: true} |
| `map_reset_no_mapping` | hub | 定位模式（使用已存地图）或本趟没有启用建图，不能重置地图 |
| `mapping_saving` | hub | 建图正在保存 |
| `map_reset_busy` | hub | 上一次重置地图还没结束 |
| `map_reset_failed` | hub（结果） | 地图重置没有全部完成：＋runner 的原因 |

**耗时（估计，待上车实测）**：建图中 ≈ 3–4 s（rtabmap reset ≈ 2.4 s ＋ 其余 < 1 s）；结束建图导出后再重置 ≈ 6–10 s
（load_map 回空白图 ≈ 1 s ＋ 重开 RTAB-Map 进程 ≈ 5–8 s）。上限：runner 等 mapping_ctl 30 s、hub 33 s、桥 35 s、页面 40 s。
