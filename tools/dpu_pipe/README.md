# tools/dpu_pipe —— 彩色帧 Nano → KV260 → DPU 输入的通路（DPUPIPE0，2026-11-03）

测量记录与判据在 `runs/nationals-prelim-20261102/DPUPIPE0/README.md`（runs/ 不入库）。本文件只讲接口。

## 数据流

```
Nano  相机 rgb8 640x480 ──> ColourRing ──> ColourSource（host/kv_colour.py，nano_colour.py --dpu）
                                            │  收到 WANT_DPU：取 after_key 之后最新的一帧，
                                            │  只发内容行（640: 921,600 B；512: 589,824 B；416: 389,376 B）＋64 字节帧头
                                            ▼
KV    dpu_rx（本目录，C++ 独立进程，不进 kv_server）
        接收线程：环里有空槽且等待推理的帧 < max_ahead 时发 WANT_DPU；每块直接收进槽内的目标位置（零拷贝）
        FrameRing（dpu_ring.h）：页对齐槽，每槽 = 一整块 DPU 输入张量 size x size x 3 int8（灰边建环时填一次）
        推理线程：acquire_ready → InferBackend::run → release（现在是桩 StubBackend，VART 就位后换 VartBackend）
```

- 同一个 ColourSource 可同时服务多个接收端：`--kv` 可重复，HELLO 发给每个接收端，回答发回 WANT 的来源地址。
  车屏（Nano 本机 127.0.0.1:47801，原始 RGB）与 KV 的 DPU 接收端（47900，int8）可以同时开。
- 预处理：`DpuPrep(size, fix_point, backend)`。fix_point 读自编译好的 xmodel（640: 6；512/416: 7）。
  `backend="auto"`：先自检本机 cv2.resize 是否与 numpy 复刻逐字节一致，一致才用 cv2（快），否则用 numpy（慢但处处一致）。
  numpy 复刻只支持缩小（相机 640x480 在三档都只缩不放）。

## 消息（kv_link 分块帧，类型 44/45，与彩色旁路同一个 UDP 端口）

| 消息 | 方向 | 内容 |
|---|---|---|
| WANT_DPU (44) | KV→Nano | `<BBHBB2sQIIQ` ver fmt=2 size fix_point flags rsv after_key wait_ms chunk_bytes kv_t_ns（32 B） |
| FRAME_DPU (45) | Nano→KV | 64 B 帧头 `<BBHBbBBQQQIIIIHHHHHHI`（ver fmt size fix_point pad_q backend rsv / source_key after_key kv_t_ns / prep_ns hold_ns crc32 content_bytes / src_w src_h w h top left / seq）＋内容行 |
| NONE (43) | Nano→KV | reason 4 = no_new（wait_ms 内没有更新的帧）；3 = error（预处理失败或发送端没开 --dpu） |

接收端按 WANT 里自己给的 chunk_bytes 算每块的落点：块 i 落在 `槽 + content_offset − 64 + i × chunk_bytes`，
帧头正好落在上灰边的最后 64 字节里，收齐后拷出帧头并把这 64 字节改回灰边值。

## 推理循环接口（给 VART 接入的人）

```cpp
dpupipe::FrameRing ring(3, size*size*3, quant_pad(fp));     // 或 FrameRing(external_ptrs, bytes)：直接用 VART TensorBuffer 的内存
dpupipe::DpuRx rx(cfg, &ring); rx.start();
for (;;) { FrameMeta m; int s = ring.acquire_ready(100, &m); if (s < 0) continue;
           backend.run(s, ring.data(s), size*size*3, m);  ring.release(s); }
```
`InferBackend::run` 的 VART 写法见 `dpu_rx.h` 注释（execute_async / wait / sync_for_read，后处理接 DETPOST0/DETPOST1）。

## 构建与测试

```
make OUT=build            # KV 上原生编（g++ 11 + zlib），或宿主
PYTHONPATH=. python3 -m unittest discover -s tests -p test_dpu_prep.py   # 预处理一致性（DPUPIPE_ULTRA=1 另跑 ultralytics 对照）
PYTHONPATH=. python3 -m unittest discover -s tests -p test_dpu_pipe.py   # 协议、多接收端、C++ 环与端到端逐字节
```
其余脚本：`bench_tx.py`（假相机发送端）、`verify_tensors.py`（整块张量 crc 对照）、`prep_consistency.py`（对 ultralytics 验证器逐字节）、
`nano_prep_check.py`（板上 T5）。

## DPUPIPE1（2026-11-03）：VART 推理后端与 CPU 输出处理

```
make OUT=build VART=1 [GLOG_SHIM=1]     # KV：/opt/vitis_ai/3.0；板上没有 glog 头文件时加 GLOG_SHIM=1（只用于编译）
. /opt/vitis_ai/3.0/env.sh
./build/dpu_rx --size 512 --fix 7 --engine vart --xmodel y8s_six_512_b1024.xmodel --workers 2 --inflight 2 \
               --seconds 150 --warm-s 20 --out-log boxes.jsonl
```
- 工作线程（`infer_engine.h`）：取最新 READY 帧 → `stage` 拷进本线程 VART runner 自己的输入缓冲 → 立即还槽 → `infer`（execute_async/wait）→ `post`。
  每个线程一个 runner；N 个线程让 DPU 队列里总有下一帧，VART 每任务的固定开销与 A53 拷贝/后处理藏在 DPU 时间后面。
- `--inflight N`：只在"READY 帧＋工作线程里的帧 < N"时向 Nano 要帧（N = 线程数）。0 = DPUPIPE0 的规则（READY < `--ahead`）。
- 输出处理 `det_post.{h,cpp}`：与 DPUB1 `full1.py` 快写法（先筛后解码＋类别偏移 NMS）逐运算同口径，float32；
  Python 参考 `det_post_ref.py`；对照工具 `det_post_cli`（宿主）、`dpu_infer_check`＋`board_check.py`（板上）。
- 出框日志一行一帧：seq、source_key、worker、t_want/t_first/t_done/t_acq/t_staged/t_infer/t_out（KV 单调时钟 ns）、ndet、cand、post_us、dets。
- 测试：`PYTHONPATH=. python3 -m unittest discover -s tests -p test_det_post.py`。

## NN-CAM2（2026-11-04）：本机帧源与前处理接口

`dpu_rx --frame-source kvcam`（默认 `udp` 与原来一字不差）：不再从 Nano 拉预处理好的张量，而是从 kv_cam 的共享内存环
（`tools/kv_cam/cam_ring.h` 的 `RingReader`，原地读、读后用序号锁复核）取最新一帧 YUYV，在 KV 上做前处理，直接写进 FrameRing 槽。
流控与 UDP 路径相同（`--inflight` / `--ahead`），DPU 的速度决定用多少相机帧，其余按"只取最新"跳过并计数。新文件 `local_src.{h,cpp}`、`colour_prep.h`。

前处理接口（`colour_prep.h`）：
```cpp
struct PrepInput { const uint8_t* data; uint64_t phys; int w, h, stride; uint32_t fourcc; };  // YUYV；phys 在环放进连续缓冲时非 0
class ColourPrep { virtual bool run(const PrepInput& in, uint8_t* tensor, size_t tensor_bytes) = 0; ... };  // 写整张张量（含灰边）
std::unique_ptr<ColourPrep> make_pl_prep(src_w, src_h, size, fix_point, kset, ups, std::string* why);   // PL 驱动入口，现返回 nullptr
```
- `--prep soft`（默认）：`SoftPrep`＝PREP1 `hw/prep/ref/yuyv_ref.c` 的 YUYV→RGB（`--prep-kset 0` JFIF 全范围／1 BT.601 有限范围；`--prep-ups 1` libjpeg fancy 上采样／0 复制）＋PREP0 `prep_model.c` 的定点缩放、灰边、量化，逐行照搬；宿主上对两份参考二进制 60 例 0 字节差（`tools/nn_cam2/check_prep.py`）。
- `--prep pl`：PREP1 驱动（`hw/prep/hls1/prep_drv.h` 建参数块，m5 比特流带核）合入后在 `make_pl_prep` 里实现：拷（或在连续环上直接指）YUYV 进核的输入缓冲 → 启动 → 等完成 → 张量交给槽。现在没合入：告警一次，退回 soft，不拒起。
- 打点：`--stamp-log FILE` 每帧一行 JSONL（相机时间戳单独几列；读到 → 前处理完 → DPU 完 → 后处理完，单调时钟），`tools/nn_cam2/chain_report.py` 解析。测试用：`--dump-tensors DIR`、`--prep-perturb`。
