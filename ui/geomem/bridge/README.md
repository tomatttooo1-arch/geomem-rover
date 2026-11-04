# GeoMem 桥（`ui/geomem/bridge/`）

笔记本上跑的小服务：一头用浏览器协议第二版（WebSocket）对 GeoMem 显示界面，一头用 `pc/fpgac.py` 连车上 hub。
设计依据：`ui/geomem/docs/PROTOCOL_V2_DESIGN.md` §2（协议）、§3（桥）。纯 Python 标准库，3.8 起可跑，Windows 也能跑。

| 文件 | 内容 |
|---|---|
| `geomem_bridge.py` | 主程序：静态托管 `ui/geomem/app/`、`/ws`、5 Hz 组帧、命令两段回执、统计 |
| `gmb_proto.py` | 纯函数：翻译表、档位判定、快照/任务/云组帧、命令分类、**拒绝码表 `REJECT_CODES`** |
| `gmb_ws.py` | RFC 6455 编解码、Host/Origin/升级头校验、静态路径白名单 |

## 启动

```
python ui/geomem/bridge/geomem_bridge.py --hub <车IP>:47431 --token-file <口令文件> \
    [--map-id <库sha256> --map-name 实验室] [--listen 127.0.0.1:8788] \
    [--basemap <底图包目录> ...] [--dump-cloud <目录>] [--stats-out <文件.json>]
```

然后浏览器开 `http://127.0.0.1:8788/`（控制台）或 `http://127.0.0.1:8788/kiosk.html`（展台页，只读，10-17）。Ctrl+C（或 SIGTERM）退出，退出时打印统计、按 `--stats-out` 写 JSON。

| 参数 | 说明 |
|---|---|
| `--hub HOST:PORT` | 车上 hub 地址，照 `car.json` 的 `hub`（`HUBPORT` 默认 47431）。必填 |
| `--token-file F` | FPGAC 口令文件，与 `fpgapc.py` 同一份。口令只在桥里，页面/URL/localStorage 都不出现。必填 |
| `--map-id` | 定位档的 `map.id`（库 sha256）。STATE `pose.map_id` 若将来出现则优先用它；非定位档一律 null |
| `--basemap DIR` | 展台底图包（`tools/basemap/make_basemap.py` 生成，可重复）。启动时读入并核 sha256，坏包拒绝启动；只托管 `/basemap/<包的库 sha256>/{basemap.json,cells.bin}`；快照加 `map.basemap`（只在定位档且 `map.id` 等于包的库 sha 时 `status=ok`）。设计 `docs/PROTOCOL_V2_DESIGN.md` §2.10 |
| `--map-name` | 定位档的 `map.name`（人工起的名字），缺省 null |
| `--listen` | 默认 `127.0.0.1:8788`。**主机只能是 127.0.0.1**，别的直接报错退出（开放局域网须 owner 同意，§3.6） |
| `--dump-cloud DIR` | 每次生成新点云时另存一份 PLY，方便肉眼核 |
| `--stats-out F` | 退出时写统计：浏览器命令数、拒绝原因分布、点云发送次数、快照异常计数、FPGAC 链路统计 |

## 浏览器侧要知道的形状（给前端）

- 连上 `/ws` 后桥立即发 `hello`，接着发一次二进制 `cloud`（没有点云来源时 `count:0, rev:null`，用来清旧云），之后 5 Hz `snapshot`。
  `hello` 在内容变化时重发（hub 重连换 boot、控制权易手、链路 up/stale/down 变化）。
- `capabilities`：`findText:true, findById:false, select:true, cancel:true, pause:false, resume:false, estop:true`，
  `teleop:true, mappingStop:true, goto:true, release:true`（10-18 控制器裁定 4：桥转发给电脑端控制界面；展台页只读、
  不加按钮——`app/src/model.js` 只留它认识的键），`control` 为 `controller` / `observer` / `none`（与 hub 断开时为 `none`）。
  `HANDOVER_BRIDGE.md` 里「`estop=false`」的旧裁定由裁定 4 取代。
- 10-18 新命令（`action` / `payload`，都只核形状；限速、夹限、范围、急停编号以车上 hub 为准，桥不放宽也不夹）：
  - `estop` `{}`：**永远转发**（链路 stale/down、observer、数据源不一致都不拦；带参数照转、`note` 注明已忽略），
    走 FPGAC 客户端自己的急停规则（不设 deadline、每 0.5 s 重发直到 hub 回执）。hub 回执 `latched_on_nano` 记为
    `ack{accepted:true}`；KV 确认后 `result{status:"done"}`（hub `latched`）；30 s 没回执 → `ack expired no_ack`（客户端仍在重发）。
  - `release` `{"confirm": true, "epoch": 整数}`：**解除急停会让车重新可动**，`confirm` 不是 `true` 一律拒 `confirm_required`；
    `epoch` 缺省时取 hub 最新 STATE 的 `estop.epoch`（都没有 → `bad_request`）；链路非 up 拒。KV 解除后 `result done`。
  - `teleop` `{"v": 米每秒, "w": 弧度每秒}` 或 `{"stop": true}`：原值转发（hub 夹限 |v|≤0.30、|w|≤1.0，超了 hub 拒 `bad_args`）；
    非零须链路 up；`stop` 任何时候都转发。hub 不发 `CMD_RESULT`，所以只有 `ack`、没有 `result`（不会 3 s 后补「结果未知」）。
    车上 deadman 0.3 s：界面要按 10–20 Hz 连发，松手发一条 `stop`。
  - `goto` `{"forwardM": 米, "turnRad": 弧度}`（至少一个）：转成 hub 的 `forward_m`/`turn_rad`；链路 up、数据源一致才转。
  - `mapping_stop` `{}`（结束建图）：链路 up 才转；保存＋导出要分钟级，`result` 最多等 185 s（hub 看建图状态 180 s）。
- 10-26 搜索热度图：hub 的 `MAP which=belief` 组成文本消息 `{"type":"belief",…}` 推给页面（只留最新，新页面补发最近一条），
  格式与时机见 `../vehicle_interface_protocol.md`「搜索热度图」。
- **无任务时 `task` 是对象不是 null**：`{"id":null,"state":"idle","reasonCode":null,"message":"待命","targetId":null,"ageMs":…,"candidates":[]}`；
  另有新增状态 `ambiguous`。现版 `app/src/model.js:27` 的 v1 校验器会拒这种形状，前端 v2 校验器要放行。
- `task.outcome`（可选，10-16 加，`docs/design/SELECT_SEMANTICS_20261016.md`）：runner 终态 `unable_to_confirm`（无法确认）或
  `needs_manual_approach`（已找到但车上没开导航，需人工前往）时，协议 `state` 仍是 `failed`（旧页面不认识的 state 会整帧拒收），
  另带 `outcome` 为该词；`needs_manual_approach` 再带 `targetPosition:[x,y,z]`（记忆位置，米，runner 世界系），`message` 含位置。
  其余状态不带这两个键。前端对缺省/表外值/格式不对一律当没有，不拒帧。
- `hello` 与 `snapshot` 都带可选 `sourceDetail`（10-15 联调第 2 棒补）：只在 `source=="replay"` 时可能非空——
  STATE `replay_id=="hub-replay-feeder"`（hub `--feed synthetic` 合成数据，先判）→ `"synthetic"`；STATE 带非空 `replay_file`
  （`fpgac.ReplayServer` 播放录制文件）→ `"recording"`；其余 `null`。前端对缺省/表外值一律按 `null`，不拒帧。
- 档位判不出（没有 runtime_info 也没有定位状态，例如回放录制、`--feed synthetic`）：`robot:null`，`map.frame` 取 MEMORY `world_frame`，不发点云。
- 命令回执两段：先 `ack`，被转发的再等 `result`。`ack.accepted=false` 时就结束了（不再发 `result`），
  唯一例外是 `no_ack`：`ack{accepted:false,status:"expired"}` 之后跟一条 `result{status:"unknown"}`（命令可能其实已到车上）。
  `ack` 被转发后 3 s 没有 hub 结果 → `result{status:"unknown"}`（`mapping_stop` 185 s、`estop`/`release` 30 s；`teleop` 没有 `result`）。
- `result.rejected` 的 `reasonCode` 是 runner 原句（如 `recent processed input, TF and map required`），`reason` 是中文；表外原词原样给，不编中文。
- `ack`/`result` 可能带 `note`（中文补充说明，如「已把“椅子”改成英文“chair”发送」「taskId 与当前任务不符，仍已转发停止」）。

## 拒绝码表（`ack.accepted=false` 时的 `reasonCode`）

与 `gmb_proto.REJECT_CODES` 一一对应（单测核两边一致）。

| reasonCode | 谁定 | 中文（`ack.reason`） | 什么时候出现 |
|---|---|---|---|
| `not_controller` | hub | 另一台电脑持有控制权 | 桥在 hub 那边不是控制者，却发了找物/选择/停止 |
| `estop_latched` | hub | 急停锁存中 | 急停锁存时找物/选择 |
| `bad_args` | hub | 请求格式不对 | hub 认为参数不对（桥已先核，正常不出现） |
| `bad_op` | hub | 车上不认识这个命令 | 同上 |
| `not_enabled` | hub | 车上没有启用这个功能 | 同上 |
| `replay` | hub | 回放模式下不能操作小车 | 连的是 `fpgac.ReplayServer`（回放），任何命令都拒 |
| `not_supported` | bridge | 当前版本不支持这个操作 | `pause`/`resume`；带 `targetId` 的按物体找物（`findById:false`） |
| `scope_changed` | bridge | 记忆已重置，请重新选择 | `select`（或带 `targetId` 的 `find`）带的 `scope` 不等于当前 `memory.scope` |
| `no_such_candidate` | bridge | 该候选不在当前候选列表里 | `select` 的 `candidateId` 不在当前任务 `candidates` 里 |
| `stale_task` | bridge | 任务已经变了，请按最新任务重新操作 | `select` 的 `taskId` 不是当前任务 id |
| `not_ambiguous` | bridge | 当前不需要选择候选 | 当前任务不在 `ambiguous` |
| `link_down` | bridge | 和小车的连接断了，请等连接恢复再操作 | 与 hub 链路不是 `up` 时的 `find`/`select`/`goto`/`mapping_stop`/`release`/非零 `teleop`（直接拒、不排队，免得重连后车突然起步）；`cancel`、`estop`、`teleop stop` 不受影响 |
| `source_inconsistent` | bridge | 车上数据来源不一致，暂不接受找物或选择 | STATE 帧级 `source` 为 `inconsistent` 时的 `find`/`select` |
| `bad_request` | bridge | 请求格式不对 | 不是 JSON 对象、`type`/`version`/`id`/`action`/`payload` 不合规、找物文字空或超 64 字、缺 `candidateId`；10-18 新命令形状不对（见上） |
| `no_ack` | bridge | 小车没有回应这条命令 | 转发后超过 deadline＋0.5 s（find/select/goto/mapping_stop 3.5 s，cancel/release 5.5 s，teleop 1.5 s；estop 30 s）没收到 hub 回执；此时 `status:"expired"` |
| `confirm_required` | bridge | 解除急停会让小车重新可以动，请求里须显式确认 | `release` 的 `payload.confirm` 不是 `true`（10-18 裁定 4） |
| `not_latched` | hub | 当前没有急停锁存 | `release` 时 hub 没锁存 |
| `epoch_mismatch` | hub | 急停已更新，请按最新急停再解除 | `release` 的 `epoch` 不是 hub 当前的（或 KV 未确认） |
| `estop_in_flight` | hub | 急停还在下发，稍后再解除 | 急停尚未得到 KV 确认时 `release` |
| `superseded_by_estop` | hub | 已被急停取代 | `release` 进行中又来了急停（在 `result` 里） |
| `no_mapping_status` | hub | 车上没有在建图 | `mapping_stop` 但本趟没起并栈建图（`CAR_MAPPING=1`） |
| `mapping_saving` | hub | 建图正在保存 | `mapping_stop` 重复发（hub 原词 `mapping_<状态>`） |
| `mapping_exported` | hub | 建图已结束并导出 | 同上 |
| `mapping_failed` | hub | 建图已失败 | 同上 |
| `teleop_active` | hub | 遥控在手，导航命令被拒；松手后重发 | 遥控在手时 `find`/`select`/`goto`（在 `result` 里，runner 原因） |
| `map_reset_no_mapping` | hub | 定位模式（使用已存地图）或本趟没有启用建图，不能重置地图 | `map_reset` 但本趟没起并栈建图（10-26） |
| `map_reset_busy` | hub | 上一次重置地图还没结束 | `map_reset` 重复发或建图状态 resetting（10-26） |
| `mapping_resetting` | hub | 地图正在重置 | 重置中发 `mapping_stop`（hub 原词 `mapping_<状态>`，10-26） |
| `map_reset_failed` | hub | 地图重置没有全部完成 | `map_reset` 的 `result`：某步失败，后接车上中文原因（10-26） |

`cancel` 任何时候都转发（含 observer、链路断开时由 FPGAC 客户端排队 5 s）；`taskId` 不符只在 `note` 里注明。

## 保守取法（设计没写死的地方）

1. `findById:false`（不做桥仿真按 id 发起）；`pause/resume:false`；`estop` 10-18 起为 `true`（裁定 4）。
2. 档位：`pose_source=="estimated"` 或 `pose.loc_state` 非空 → 定位档（`map` 系，`robot.pose` 仅 `loc_state=="valid"` 且 estimated 为 `map`，否则 `none`）；
   帧级 `source=="sim"` → 仿真（`map` 系）；`pose_source=="reference"` → 里程档（`odom` 系，`map.id=null`，STATE 没有 `odom.x/y` 前 `robot=null`）；都不是 → 未知。
3. 点云来源：定位/仿真档用非替身且 `frame_id` 等于 `map.frame` 的全局栅格（`cloudRev="global:<rev>"`）；里程档用滚动栅格（`rolling:<crc32>`，至多 2 Hz）；其余不出云。
   全局栅格 `rev` 为空时也用 `global:<crc32>`。
4. 无 MEMORY 时 `memory` 三字段全 null、`objects` 空、`serverTime` null。
5. `map.name` 只在定位档给（其它档没有「地图」）。
6. 物体：没有类名（`label` 不是非空字符串）的物体与候选丢弃并计数（`object_no_label`/`candidate_dropped`，10-15 联调定：前端要求 `className` 必填）；`state` 不在四值内给 null；位置非有限的丢弃；重复 id 只留第一条；超 1000 条按 `lastSeen` 留最新；`confidence`/`lastSeen` 非有限给 null。
7. runner 任务状态不在已知集合内（正常不会出现）：`state:"idle"`，`reasonCode` 为原状态词，`message`「车上任务状态未知：…」。
8. `fault:` 开头的原因 →「车上出错：＋原词」；runner `fault()` 的裸原因（如 `map_invalid: …`）不带前缀，原样给。
9. `selection_rejected` 两句原句的中文（「当前没有待选择的候选」「该候选已不在列表中」）是桥补的，§3.5 表里没有。
10. 找物文字含 runner 认的中文词或任何英文字母 → 原样转发；否则按最长匹配找表外 COCO 中文名（如 椅子、酒杯、花瓶）→ 改发 `"<英文颜色> <英文类名>"` 并在 `ack.note` 说明。
11. 入站 WebSocket 消息上限 1 MiB；同时最多 8 个浏览器连接。

## 测试

联调试验台（T1 回放、T2 hub 夹具，全在 `unshare -rn` 里）：见 `testbed/README.md`。


```
cd <仓库根> && PYTHONPATH=. python3 -m unittest discover -s tests -p 'test_geomem_bridge.py'
```

单测不开任何监听端口：纯函数直接测，HTTP/WS 壳用 `socket.socketpair()` 驱动真 `Handler`，FPGAC 用替身客户端。
3.13 与 `/usr/bin/python3.8` 都要过。实车/回放的端到端试验（T1 回放、T2 hub 夹具）见交接 `ui/geomem/docs/HANDOVER_BRIDGE.md`。

## 已知限制

- 发送线程对卡死的浏览器没有发送超时（本机回环，浏览器关页即断）；快照/点云槽位只留最新，不会堆内存。
- 连的是 `fpgac.ReplayServer`（回放）时，命令拿不到 `replay` 拒绝：`ReplayServer` 播放期间不读入站帧
  （`pc/fpgac.py` `Connection.recv(0)` 在 `left<0` 时直接返回），命令既无回执也无 PONG，桥按超时回 `no_ack`、界面提示一次「结果未知」。
  回放下界面本来就置灰找物/选择，只有停止会发出去；无害，本线不改 `pc/`。
- 全局栅格转点是纯 Python，几百万格的大图转一次要秒级（只在 `rev` 变化时做一次）。
