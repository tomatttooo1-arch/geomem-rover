# GeoMem 联调试验台（T1 回放、T2 hub 夹具）

**只在 `unshare -rn` 隔离网络命名空间里跑**：宿主任何接口都不监听 hub 端口（owner 10-14）。两个 Python 入口没有 `GEOMEM_TB_NETNS=1` 直接拒绝运行（单测 `tests/test_geomem_integ.py` 核）。
回放与合成数据只作工程验证，**不是比赛或答辩画面**。

| 文件 | 内容 |
|---|---|
| `run_tb.sh` | 入口：`TB_TOOLS=<含 node_modules 的目录> sh run_tb.sh <脚本> [参数]`。把 `ui/geomem/tools/test/{*.mjs,integ/*.mjs}` 复制到 `$TB_TOOLS`，再 `unshare -rn` 起 `lo` 跑 node |
| `t1_replay.py` | `fpgac.ReplayServer` 的最小命令行（fpgac 本身没有）：`t1_replay.py <pc.fpgarec> <口令文件> [--port] [--rate]` |
| `t2_hub_fixture.py` | 进程内 `HubCore + HubServer(127.0.0.1)`（只 import `host/`），按场景喂 STATE/MEMORY/MAP 输入，带替身 runner 与控制口 `/ctl?op=…`（文件头列了全部 op） |
| `t3_state_tap.py` | T3 仪器：桥之后连上的 FPGAC 旁听客户端，每帧 STATE 记任务状态、`nav.motion_permit` 与 odom v/x/y（第 3 棒加，JSONL）；无 `GEOMEM_TB_NETNS=1` 拒绝运行 |

node 依赖 `ws`、`playwright-core`（装在仓外，`TB_TOOLS/node_modules`）；无头浏览器路径见 `ui/geomem/tools/test/lib.mjs`。
展台页脚本（10-17，`integ/kiosk_*.mjs`）改用 `puppeteer-core`（宿主没有 playwright-core），公共件 `kiosk_tb.mjs` 不 import `tb.mjs`；须另设 `GEOMEM_BASEMAP=<底图包目录>`：`kiosk_smoke.mjs <输出> gpu|swr`（冒烟）、`kiosk_t2.mjs <输出> gpu|swr`（判据 K2–K5 与外观截图，`KIOSK_BENCH_REPS` 改 bench 次数）、`kiosk_t1.mjs <输出>`（K6 回放）。夹具为展台加了 op `objects&file=`、`pose`、`walk`、`kv`（见 `t2_hub_fixture.py` 文件头）。预注册与结果：`docs/STAGE1_KIOSK_PREREG.md`、`STAGE1_KIOSK_RESULTS.md`。
判据脚本在 `ui/geomem/tools/test/integ/`（`tb.mjs` 公共件：按 pid 起停子进程、最小 WS 客户端、页内仪器）。预注册与结果：`ui/geomem/docs/STAGE1_INTEG_PREREG.md`、`STAGE1_INTEG_RESULTS.md`。

例：

```
TB_TOOLS=/path/to/tools sh ui/geomem/bridge/testbed/run_tb.sh t2_a9_scope.mjs runs/geomem-integ-20261015-results/a9
```

已知坑：
- SwiftShader 下首连首帧/着色器编译会卡页面主线程 3–5 s，脚本一律等连接标签（`waitPill`），不用固定等待。
- `ReplayServer` 播放期间不读入站帧（`Connection.recv(0)` 在 `left<0` 时直接返回），命令拿不到 `replay` 拒绝，桥回 `no_ack`。
- （10-15 第 2 棒已修）hub 收 MEMORY 时剔掉含 NaN/inf 的物体（`memory_nonfinite_tracks`）、其余非有限字段置 null（`memory_nonfinite_fields`），发送处兜底只丢 MEMORY、同拍 STATE 照发（`memory_unsendable:*`）。修前整帧编不出、连同那拍 STATE 丢（`tick_error:ValueError`）；所以桥自身的非有限处理仍要靠单测。
- 夹具换 global 图要等 hub 的 5 s 最小间隔；从定位档切 odom 档时夹具会删掉 hub 里旧的 `loc_status`/`loc_pose`/`runtime_info`。
