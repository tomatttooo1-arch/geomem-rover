# tools/nn_cam2：单板数据面台架（NN-CAM2，2026-11-04）

kv_server、dpu_rx 改从本机 kv_cam 环取帧（`--frame-source kvcam`）之后的测试与板上工具。说明与结果见 `runs/nationals-prelim-20261102/NN-CAM2/README.md`。

| 文件 | 作用 |
|---|---|
| `run_equiv.py` | 宿主：同一段录像走 UDP（Nano 自己的客户端代码）与走 kv_cam 回放环，两进程交给核的输入逐字节比；阳性对照（错帧、翻字节） |
| `check_prep.py` | 宿主：dpu_rx 的软件前处理与 numpy YUYV 对 PREP1/PREP0 参考二进制、对 Nano 的 dpu_prep |
| `chain_report.py` | 两条链打点 JSONL → 各级时延（中位/p95/p99）、帧率、丢帧率 |
| `cpu_sample.py` | /proc/stat 四核占用与指定进程 CPU（`--selftest` 已知答案） |
| `kv2_chain.sh` | 板上 KV-2 会话：编译、仪器门、kv_cam 起停、台架运行（带 CPU 与功耗采样）、汇总 |
| `params_car_bench.json` | 板上本机帧源的台架参数（相机位姿、导航目标；内参取自环头） |
| `yuyv_rgb.py`、`pack.py` | numpy YUYV→RGB（PREP1 算法）、kv_cam 回放包读取 |
