# board/results/ —— 上板实测输出汇总

只放汇总文件（原始逐帧日志未放入）。文件内的本机目录已换成目录变量（含义见顶层 README）。

共同条件：KV260 Starter Kit（K26 SOM，XCK26-SFVC784-2LV-C），板端 KV260 官方 Ubuntu 22.04 镜像；
比特流：`golive_m7/` 为产品默认的整板比特流 `bitstream_m7_fullboard/m7_pl_m7s17/`（构建号 0x063ba99b），其余为 `bitstream/m7_pl_m5_166/`（构建号 0x0f35f07f）；每次加载后从 PL 系统信息寄存器读回构建号核对；
相机 Orbbec Gemini 2（USB3，约 30 帧/秒）；KV260 整机连续运行（2026-11-04）。

| 文件 | 对应的测量 | 条件 | 关键结果（以文件内容为准） |
|---|---|---|---|
| `golive_m7/chain_report.txt` | **产品默认配置**：整板比特流下一条命令 `bash tools/golive/start_all.sh` 启动整机后的台架连续运行，各段帧率与时延（起点 = 相机进程拿到帧）；判读用 `judge.py <日志目录> 200 gmap3`，判据 C1–C9（C7 搜索核、C8 规划核终版、C9 记忆核） | 640 识别模型、前处理与后处理走 PL，`MEM=pl`、`SRCH=pl`、`PLN=gmap3` | 连续 250.6 s；深度链稳态 26.3 帧/秒；势场 509 次（1.90 次/秒）；识别稳态 7.11 帧/秒；相机→几何中位 12.8 ms，相机→势场中位 / p95 46.4 / 65.1 ms，相机→物体框中位 / p95 159.2 / 177.1 ms；规划核取路径 510 次全部与 PS 参考一致；507 条上行的记忆全部来自 PL 记忆核；搜索 26 轮全部在核上完成（搜索核第二版 `hw/srch_v2/`）；整机功耗均值 6.27 W（空闲 3.96 W） |
| `chassis_bridge/compare_nano.json`、`kv_server_smoke.json`、`env.txt` | KV 侧底盘桥与原驱动的对照：用实车录制的底盘串口数据回放，比较 KV 侧底盘桥与原 Nano 驱动的输出 | 脚本 `tools/kv_chassis/compare_nano.py`；`kv_server_smoke.json` 为 KV 服务接入底盘桥后的启动冒烟 | 3,994 帧命令逐字节相同，103,966 帧里程逐位相同 |
| `golive/judge.json` | m5 比特流（`BIT=m5`）下整机一条命令启动后的台架连续运行判读（判据 C1 连续时长 ≥300 s、C2 深度帧在走、C3 势场在更新、C4 识别出框、C5 Nano 收到框与地图、C6 无崩溃无重启） | 640 识别模型（`models/dpu/y8s640/`）、1 个推理线程、后处理走 CPU；`tools/golive/start_all.sh` 启动，`tools/golive/judge.py` 判读 | 连续 317.9 s；深度链稳态 27.4 帧/秒；势场 617 次（1.87 次/秒）；识别稳态 6.2 帧/秒 |
| `golive/chain_report.txt` | 同一次运行的各段帧率与时延（起点 = 相机进程拿到帧） | 同上 | 相机→几何完成中位 13.2 ms；相机→代价图完成中位 17.7 ms |
| `golive/chain_report_post_pl.txt` | 识别后处理走 PL 核（`--post pl`，即本包启动脚本的默认值）时的识别链各段时延 | 640 模型、只跑彩色链 60 s | 识别 7.13 帧/秒；相机→物体框中位 157.9 ms |
| `e2e/m5-640-w1/summary.txt`、`e2e0_summary.csv`、`e2e0_summary.json` | 端到端时延与整链功耗：只跑彩色识别链 | 640 模型、1 个推理线程、CPU 后处理，120 s | 相机→物体框中位 165.1 / p95 180.8 ms，6.84 帧/秒；DPU 段 133.7 ms；整链功耗 5.83 W |
| `e2e/m5-w1both/summary.txt`、`e2e0_summary.csv`、`e2e0_summary.json` | 端到端时延与整链功耗：深度链（几何→融合→代价图→规划）与识别链同时跑 | 512 与 640 两个模型各测一次，1 个推理线程，各约 100 s | 640：相机→势场中位 47.8 ms（0.5 s 重规划周期），相机→物体框中位 157.1 ms，6.79 帧/秒，功耗 5.97 W；512：相机→物体框中位 99.4 ms，13.58 帧/秒，功耗 6.00 W |

说明：
- `summary.txt` 中 "build_id.txt missing" 只表示该次汇总脚本没有在输出目录里找到构建号文件；构建号在测量开始时已从板上读回并核对为 0x0f35f07f。
- `idle_all power nan` 表示这一次运行没有单独测空闲功耗；同一块板、同一比特流下的空闲功耗（比特流在、所有进程停）为约 3.9 W。
- 判读脚本与汇总脚本：`tools/golive/judge.py`；端到端测量的汇总口径见各 `summary.txt` 文件头。
