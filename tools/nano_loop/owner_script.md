# P3 需要人动手的两组对照：拿走物体、真手遮镜头（约 5 分钟）

对象：owner，在 Orin Nano 旁边执行。相机**保持不动**（固定相机，位姿写死为恒等）。
背景：自主跑的时候没人能动场景，所以「拿走」「遮挡」只做了离线构造（见 `docs/design/NANO_LOOP_20260927.md` §6 的 J4）。
这里用真实动作补上。不涉及 KV260、底盘。

## 0. 准备（1 分钟）

```bash
ssh orin-nano
df -h / | tail -1                                                  # 剩余 < 8 GB 就不跑
```

〔09-27 午后更正：服务器连 Nano 的日常入口是 `ssh nano`（校园网反向隧道，09-26 实测约 1.5 MB/s，owner 09-26 确认）；`ssh orin-nano`（Tailscale 经香港中继，慢且会间歇停顿）只作备用。上面命令里的 `orin-nano` 换成 `jetson` 即可，数据也可以直接走这条链路拉回；见 `docs/team/RECORDING_PROTOCOL_20260927.md` §6.1。原文保留。〕

场景：两个矿泉水瓶（相机左侧，约 25–28 cm 远，间隔约 8 cm）和笔记本都在画面里，和 09-27 凌晨一样。
**更正（09-27 验收后）**：其中约 28 cm 那个是白色外带纸杯（被识别器误认成 bottle），不是矿泉水瓶；「离相机最近的那个瓶子」就是那个矿泉水瓶（约 25 cm，红色标签），拿它。表里「另一个瓶子」指纸杯。
下面命令里 `R=~/fpga/p3/code/src/tools/nano_loop/run_p3.sh`。

## 1. 拿走物体（约 1.5 分钟）

```bash
R=~/fpga/p3/code/src/tools/nano_loop/run_p3.sh
bash $R ~/fpga/p3/runs/owner_remove --frames 600 --warmup-frames 30 --snap-every 50 &
until [ -e ~/fpga/p3/runs/owner_remove/ledger.jsonl ]; do sleep 1; done; echo "== 开始计时"
```

- 启动要编译和加载引擎，约 10 秒。看到 `== 开始计时` 后**数 20 秒**，把**离相机最近的那个瓶子**整个拿出画面，放远处，不要再回来。
- 手伸进画面的那 1–2 秒会有遮挡，没关系。
- 等命令结束（处理 630 帧约 65 秒，屏幕打印 `rc=0`）。

## 2. 真手遮镜头（约 1.5 分钟）

先把瓶子放回原位，然后：

```bash
bash $R ~/fpga/p3/runs/owner_cover --frames 600 --warmup-frames 30 --snap-every 50 &
until [ -e ~/fpga/p3/runs/owner_cover/ledger.jsonl ]; do sleep 1; done; echo "== 开始计时"
```

- 看到 `== 开始计时` 后数 20 秒，用手掌**贴住镜头**（彩色和深度两个窗口都盖住）约 10 秒，然后拿开。
- 再数 10 秒后，用一张纸板**挡在镜头前 10 cm 左右**（不贴住）约 10 秒，然后拿开。
- 等命令结束。

## 3. 看结果（1 分钟）

```bash
cd ~/fpga/p3/code/src
for r in owner_remove owner_cover; do
  echo "== $r"; cat ~/fpga/p3/runs/$r/DONE
  python3 tools/nano_loop/summarize.py ~/fpga/p3/runs/$r --timeline      # 每 2 秒一行
  PYTHONPATH=$PWD python3 tools/nano_loop/query.py --store ~/fpga/p3/runs/$r/memory.json bottle laptop \
     > ~/fpga/p3/runs/$r/query.json
done
```

`--timeline` 每行是 2 秒里的计数：`support:bottle`（瓶子有深度支持的观测数）、`geometry_invalid`（整帧没有有效深度）、
`watch:bottle:<候选>:<原因>`（已记住的瓶子这帧没被识别到时，用它的历史框做的「移走证据」检查）。

## 4. 预期与判读

| 对照 | 预期 | 不符合时说明什么 |
|---|---|---|
| 拿走瓶子后 | 被拿走那个瓶子的 `watch:bottle:free_space_conflict:conservative_free_depth` 出现并持续（它后面的桌面/背景比原来远）；另一个瓶子的 `support:bottle` 不中断 | 若一直是 `surface_or_uncertain`：历史框里还有别的表面（比如另一个瓶子挡进了框），记下来，不改阈值 |
| 拿走后查询 `bottle` | 仍返回两个候选（记忆**没有**「移走」这一状态转移，这是 M4 的设计）；scope 声明原样在 | 若少了一个候选：记忆被改写了，属于缺陷，停下报告 |
| 手掌贴住镜头 | 这段时间 `geometry_invalid` 为主，或 `watch:*:indeterminate:invalid_historical_region_depth`；**`free_space_conflict` 必须为 0** | 出现 `free_space_conflict` = 无效深度被当成了空闲，属于缺陷，停下报告 |
| 纸板挡在前面 | `watch:*:indeterminate:historical_region_occluded`；**`free_space_conflict` 必须为 0** | 同上 |

## 5. 收尾

```bash
exit   # 回到服务器
cd ${FPGA_HOME} && bash tools/lock.sh release orin-nano <token> && bash tools/lock.sh release camera <token>
```


只需要把两次的 `summary.json`、`query.json` 和 `--timeline` 的屏幕输出贴给我；图片和账本留在板上（链路慢）。〔09-27 午后更正：`ssh nano` 约 1.5 MB/s，图片和账本也可以直接拉回服务器，见 `docs/team/RECORDING_PROTOCOL_20260927.md` §6.1；原文保留。〕
